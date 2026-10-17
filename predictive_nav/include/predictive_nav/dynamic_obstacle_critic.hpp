// MPPI critic that penalises candidate trajectories passing where tracked
// moving obstacles are predicted to be (see dynamic_obstacle_cost.hpp).
//
// Input: /tracked_obstacles (nav_msgs/Odometry, one message per confirmed
// track per scan, frame map, twist = velocity in the map frame), published by
// the obstacle_tracker package.
//
// Built for the Humble MPPI controller (nav2_mppi_controller 1.1.x, xtensor).
#ifndef PREDICTIVE_NAV__DYNAMIC_OBSTACLE_CRITIC_HPP_
#define PREDICTIVE_NAV__DYNAMIC_OBSTACLE_CRITIC_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "predictive_nav/dynamic_obstacle_cost.hpp"

namespace mppi::critics
{

class DynamicObstacleCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  struct Track
  {
    double stamp;
    predictive_nav::Obstacle obs;  // map frame
  };

  void onTrack(const nav_msgs::msg::Odometry::SharedPtr msg);

  predictive_nav::CostParams params_;
  std::string topic_;
  double max_track_age_{0.5};          // s, ignore tracks older than this
  double max_obstacle_distance_{6.0};  // m, ignore obstacles further from the robot
  double near_goal_distance_{0.5};     // m, critic off this close to the goal

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mutex_;
  std::vector<Track> tracks_;          // tracks of the most recent scan
  double tracks_stamp_{-1.0};
};

}  // namespace mppi::critics

#endif  // PREDICTIVE_NAV__DYNAMIC_OBSTACLE_CRITIC_HPP_
