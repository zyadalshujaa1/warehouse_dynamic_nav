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
// Terminal extension (v0.3): the MPPI horizon (56 x 0.05 s = 2.8 s, i.e. 0.84 m
// at 0.3 m/s) is shorter than the space a robot needs to stop clear of a person
// crossing its path. Each candidate is therefore extended for `extension_time`
// seconds beyond its last state with its final velocity (a trajectory that ends
// stopped stays where it stopped). Collisions with the predicted obstacles in
// this extension count like collisions inside the horizon, so "stop in the
// person's path" is recognised as a collision instead of looking safe.
//
// The robot body is covered by circles placed along its x axis. With
//     d = |circle centre - obstacle| - circle_radius - obstacle_radius
// a trajectory gets
//     collision_cost                   if d < 0 at any evaluated time, and
//     weight * mean( ((m - d) / m)^2 for d < m )   (soft penalty),
// where m = safety_margin + uncertainty_growth * tau grows with the
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
  float extension_time{3.0f};       // s, terminal extension (0 = off)
  float extension_dt{0.1f};         // s, step inside the extension
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
  const std::size_t n_ext = (p.extension_time > 0.0f && p.extension_dt > 0.0f && steps >= 2) ?
    static_cast<std::size_t>(std::lround(p.extension_time / p.extension_dt)) : 0;
  const float horizon = static_cast<float>(steps) * dt;

  for (std::size_t i = 0; i < batch; ++i) {
    float soft = 0.0f;
    bool collided = false;

    // one robot pose at prediction time tau
    auto evaluate = [&](float px, float py, float pyaw, float tau) {
        const float c = std::cos(pyaw);
        const float s = std::sin(pyaw);
        const float margin = p.safety_margin + p.uncertainty_growth * tau;
        for (const auto & o : obstacles) {
          float ox = o.x, oy = o.y;
          if (p.use_prediction) {
            ox += o.vx * (tau + latency);
            oy += o.vy * (tau + latency);
          }
          float d_min = INFINITY;
          for (float off : p.circle_offsets) {
            const float dx = px + off * c - ox;
            const float dy = py + off * s - oy;
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
      };

    for (std::size_t t = 0; t < steps; ++t) {
      const std::size_t k = i * steps + t;
      evaluate(x[k], y[k], yaw[k], static_cast<float>(t + 1) * dt);
    }
    if (n_ext > 0) {
      const std::size_t last = i * steps + steps - 1;
      const float vx_end = (x[last] - x[last - 1]) / dt;
      const float vy_end = (y[last] - y[last - 1]) / dt;
      for (std::size_t e = 1; e <= n_ext; ++e) {
        const float s_ext = static_cast<float>(e) * p.extension_dt;
        evaluate(x[last] + vx_end * s_ext, y[last] + vy_end * s_ext, yaw[last], horizon + s_ext);
      }
    }
    costs[i] += p.weight * soft / static_cast<float>(steps + n_ext) +
      (collided ? p.collision_cost : 0.0f);
  }
}

}  // namespace predictive_nav

#endif  // PREDICTIVE_NAV__DYNAMIC_OBSTACLE_COST_HPP_
