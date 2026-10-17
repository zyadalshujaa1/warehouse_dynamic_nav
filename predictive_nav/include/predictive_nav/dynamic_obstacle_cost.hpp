// Cost of MPPI candidate trajectories against moving obstacles whose future
// positions are predicted with a constant-velocity model.
//
// Pure C++ (no ROS, no xtensor) so it can be unit-tested on its own; the MPPI
// critic plugin (dynamic_obstacle_critic.cpp) only collects the inputs and
// calls score_trajectories().
//
// For candidate trajectory i and time step t (time tau = (t + 1) * dt, the
// same indexing as the MPPI optimizer, which integrates the first command over
// one model_dt), every obstacle j is placed at
//
//     use_prediction = true :  p_j + v_j * (tau + latency)
//     use_prediction = false:  p_j                         (ablation: obstacle
//                                                           assumed to stand still)
//
// The robot body is covered by circles placed along its x axis. With
//     d = |circle centre - obstacle| - circle_radius - obstacle_radius
// a trajectory gets
//     collision_cost                   if d < 0 at any step (hard collision), and
//     weight * mean_t( ((m_t - d) / m_t)^2 for d < m_t )   (soft penalty),
// where m_t = safety_margin + uncertainty_growth * tau grows with the
// prediction horizon, because the prediction error grows with time
// (measured in this project: median 0.18 m at 1 s, 0.32 m at 2 s, 0.50 m at 3 s).
#ifndef PREDICTIVE_NAV__DYNAMIC_OBSTACLE_COST_HPP_
#define PREDICTIVE_NAV__DYNAMIC_OBSTACLE_COST_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

namespace predictive_nav
{

struct Obstacle
{
  float x, y;     // position, same frame as the trajectories
  float vx, vy;   // velocity, same frame
};

struct CostParams
{
  std::vector<float> circle_offsets{-0.2f, 0.2f};  // m, along the robot x axis
  float circle_radius{0.375f};      // m, the two circles cover the 0.82 x 0.62 m footprint
  float obstacle_radius{0.25f};     // m, a person
  float safety_margin{0.5f};        // m, soft penalty starts this far away
  float uncertainty_growth{0.15f};  // m/s, margin growth with prediction time
  float collision_cost{10000.0f};
  float weight{20.0f};
  bool use_prediction{true};
};

// x, y, yaw: row-major [batch][time_steps]; costs: [batch], values are ADDED.
inline void score_trajectories(
  const float * x, const float * y, const float * yaw,
  std::size_t batch, std::size_t steps, float dt, float latency,
  const std::vector<Obstacle> & obstacles, const CostParams & p, float * costs)
{
  if (obstacles.empty() || steps == 0) {
    return;
  }
  const float r_sum = p.circle_radius + p.obstacle_radius;
  for (std::size_t i = 0; i < batch; ++i) {
    float soft = 0.0f;
    bool collided = false;
    for (std::size_t t = 0; t < steps; ++t) {
      const std::size_t k = i * steps + t;
      const float tau = static_cast<float>(t + 1) * dt;
      const float c = std::cos(yaw[k]);
      const float s = std::sin(yaw[k]);
      const float margin = p.safety_margin + p.uncertainty_growth * tau;
      for (const auto & o : obstacles) {
        float ox = o.x, oy = o.y;
        if (p.use_prediction) {
          ox += o.vx * (tau + latency);
          oy += o.vy * (tau + latency);
        }
        float d_min = INFINITY;
        for (float off : p.circle_offsets) {
          const float dx = x[k] + off * c - ox;
          const float dy = y[k] + off * s - oy;
          const float d = std::sqrt(dx * dx + dy * dy) - r_sum;
          d_min = d < d_min ? d : d_min;
        }
        if (d_min < 0.0f) {
          collided = true;
        } else if (d_min < margin) {
          const float u = (margin - d_min) / margin;
          soft += u * u;
        }
      }
    }
    costs[i] += p.weight * soft / static_cast<float>(steps) +
      (collided ? p.collision_cost : 0.0f);
  }
}

}  // namespace predictive_nav

#endif  // PREDICTIVE_NAV__DYNAMIC_OBSTACLE_COST_HPP_
