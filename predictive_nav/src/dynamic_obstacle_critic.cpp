#include "predictive_nav/dynamic_obstacle_critic.hpp"

#include <algorithm>
#include <cmath>

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

namespace mppi::critics
{

void DynamicObstacleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(params_.weight, "cost_weight", 20.0);
  getParam(params_.collision_cost, "collision_cost", 10000.0);
  getParam(params_.safety_margin, "safety_margin", 0.5);
  getParam(params_.uncertainty_growth, "uncertainty_growth", 0.15);
  getParam(params_.obstacle_radius, "obstacle_radius", 0.25);
  getParam(params_.circle_radius, "circle_radius", 0.375);
  getParam(params_.use_prediction, "use_prediction", true);
  getParam(params_.extension_time, "extension_time", 3.0);
  getParam(params_.extension_dt, "extension_dt", 0.1);
  getParam(max_track_age_, "max_track_age", 0.5);
  getParam(max_obstacle_distance_, "max_obstacle_distance", 6.0);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);
  std::vector<double> offsets;
  getParam(offsets, "circle_offsets", std::vector<double>{-0.2, 0.2}, ParameterType::Static);
  params_.circle_offsets.assign(offsets.begin(), offsets.end());
  getParam(topic_, "topic", std::string("/tracked_obstacles"), ParameterType::Static);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    topic_, rclcpp::QoS(50),
    std::bind(&DynamicObstacleCritic::onTrack, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "DynamicObstacleCritic: topic %s, prediction %s, weight %.1f, margin %.2f m "
    "(+%.2f m/s), %zu body circles r=%.3f m, terminal extension %.1f s",
    topic_.c_str(), params_.use_prediction ? "ON" : "OFF (ablation)", params_.weight,
    params_.safety_margin, params_.uncertainty_growth, params_.circle_offsets.size(),
    params_.circle_radius, params_.extension_time);
}

void DynamicObstacleCritic::onTrack(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  if (stamp > tracks_stamp_) {       // a new scan: forget the tracks of the previous one
    tracks_.clear();
    tracks_stamp_ = stamp;
  } else if (stamp < tracks_stamp_) {
    return;                          // late message from an older scan
  }
  Track tr;
  tr.stamp = stamp;
  tr.obs.x = static_cast<float>(msg->pose.pose.position.x);
  tr.obs.y = static_cast<float>(msg->pose.pose.position.y);
  tr.obs.vx = static_cast<float>(msg->twist.twist.linear.x);
  tr.obs.vy = static_cast<float>(msg->twist.twist.linear.y);
  tracks_.push_back(tr);
}

void DynamicObstacleCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.path)) {
    return;   // let the robot settle on its goal
  }

  std::vector<Track> tracks;
  double stamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tracks = tracks_;
    stamp = tracks_stamp_;
  }
  const double now = clock_->now().seconds();
  if (tracks.empty() || now - stamp > max_track_age_) {
    return;
  }
  const float latency = static_cast<float>(std::max(0.0, now - stamp));

  // Tracks are in the map frame; the trajectories are in the costmap frame (odom).
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const std::string track_frame = "map";
  double tx = 0.0, ty = 0.0, tyaw = 0.0;
  if (costmap_frame != track_frame) {
    try {
      const auto tf = costmap_ros_->getTfBuffer()->lookupTransform(
        costmap_frame, track_frame, tf2::TimePointZero);
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      tyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "DynamicObstacleCritic: no TF %s -> %s: %s",
        track_frame.c_str(), costmap_frame.c_str(), e.what());
      return;
    }
  }
  const float c = static_cast<float>(std::cos(tyaw));
  const float s = static_cast<float>(std::sin(tyaw));
  const float rx = static_cast<float>(data.state.pose.pose.position.x);
  const float ry = static_cast<float>(data.state.pose.pose.position.y);

  std::vector<predictive_nav::Obstacle> obstacles;
  for (const auto & tr : tracks) {
    predictive_nav::Obstacle o;
    o.x = static_cast<float>(tx) + c * tr.obs.x - s * tr.obs.y;
    o.y = static_cast<float>(ty) + s * tr.obs.x + c * tr.obs.y;
    o.vx = c * tr.obs.vx - s * tr.obs.vy;
    o.vy = s * tr.obs.vx + c * tr.obs.vy;
    if (std::hypot(o.x - rx, o.y - ry) <= max_obstacle_distance_) {
      obstacles.push_back(o);
    }
  }
  if (obstacles.empty()) {
    return;
  }

  const auto & traj = data.trajectories;
  const std::size_t batch = traj.x.shape(0);
  const std::size_t steps = traj.x.shape(1);
  predictive_nav::score_trajectories(
    traj.x.data(), traj.y.data(), traj.yaws.data(), batch, steps, data.model_dt, latency,
    obstacles, params_, data.costs.data());
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicObstacleCritic, mppi::critics::CriticFunction)
