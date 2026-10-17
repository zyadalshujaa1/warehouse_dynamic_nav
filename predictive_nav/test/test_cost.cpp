// Offline test of dynamic_obstacle_cost.hpp (no ROS).
// g++ -std=c++17 -O2 -I../include test_cost.cpp -o test_cost && ./test_cost
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
#include "predictive_nav/dynamic_obstacle_cost.hpp"
using namespace predictive_nav;

int main()
{
  const std::size_t T = 56; const float dt = 0.05f;   // MPPI default horizon: 2.8 s
  // three candidates from the origin, heading +x: 0 = straight 0.3 m/s,
  // 1 = stop, 2 = straight but slower 0.1 m/s
  const float speeds[3] = {0.3f, 0.0f, 0.1f};
  std::vector<float> x(3 * T), y(3 * T, 0.f), yaw(3 * T, 0.f);
  for (int i = 0; i < 3; ++i) for (std::size_t t = 0; t < T; ++t) x[i * T + t] = speeds[i] * (t + 1) * dt;

  // worker 1.6 m ahead-right, walking across the robot's path at 0.6 m/s
  std::vector<Obstacle> obs{{1.0f, -1.2f, 0.0f, 0.6f}};
  CostParams p;
  for (bool pred : {true, false}) {
    p.use_prediction = pred;
    float c[3] = {0, 0, 0};
    score_trajectories(x.data(), y.data(), yaw.data(), 3, T, dt, 0.0f, obs, p, c);
    std::printf("prediction=%d  cost straight=%.1f  stop=%.1f  slow=%.1f\n", pred, c[0], c[1], c[2]);
    if (pred) { assert(c[0] >= p.collision_cost); assert(c[1] < p.collision_cost); }
    else      { assert(c[0] < p.collision_cost); }   // without prediction the danger is not seen
  }

  // the two circles must cover every corner of the 0.82 x 0.62 m footprint
  for (float cx : {0.41f, -0.41f}) for (float cy : {0.31f, -0.31f}) {
    float best = 1e9f;
    for (float off : p.circle_offsets) best = std::fmin(best, std::hypot(cx - off, cy));
    assert(best <= p.circle_radius + 1e-3f);
  }
  // no obstacles -> costs unchanged
  float c0[1] = {1.5f}; score_trajectories(x.data(), y.data(), yaw.data(), 1, T, dt, 0.f, {}, p, c0);
  assert(c0[0] == 1.5f);
  std::puts("all checks passed");
}
