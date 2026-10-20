// Toy closed-loop check of the critic (NOT MPPI, NOT a proof): every 0.1 s a
// unicycle robot picks the cheapest of 9 constant (v, w) candidates over a
// 2.8 s horizon. Cost = dynamic-obstacle critic + progress/path terms that
// stand in for MPPI's path critics. The worker is non-reactive, like in the
// simulation. Reports whether the robot touches the worker.
// g++ -std=c++17 -O2 -I../include test_closed_loop.cpp -o tcl && ./tcl
#include <cmath>
#include <cstdio>
#include <vector>
#include "predictive_nav/dynamic_obstacle_cost.hpp"
using namespace predictive_nav;

struct Result { bool contact; float min_clear; float final_x; };

Result run(Obstacle w0, float ext, bool pred)
{
  const std::size_t T = 56; const float dt = 0.05f;
  CostParams p; p.extension_time = ext; p.use_prediction = pred;
  float rx = 0, ry = 0, ryaw = 0;            // robot, path = the x axis, goal far ahead
  Obstacle w = w0;
  const float vs[3] = {0.0f, 0.15f, 0.3f}, ws[3] = {-0.8f, 0.0f, 0.8f};
  float min_clear = 1e9f;
  for (int k = 0; k < 250; ++k) {            // 25 s
    std::vector<float> X(9 * T), Y(9 * T), Th(9 * T); float cost[9] = {0};
    for (int c = 0; c < 9; ++c) {
      float x = rx, y = ry, th = ryaw;
      for (std::size_t t = 0; t < T; ++t) {
        th += ws[c % 3] * dt; x += vs[c / 3] * std::cos(th) * dt; y += vs[c / 3] * std::sin(th) * dt;
        X[c * T + t] = x; Y[c * T + t] = y; Th[c * T + t] = th;
      }
      cost[c] = 4.0f * (rx + 0.84f - x) + 3.0f * std::fabs(y) + 1.0f * std::fabs(th);  // progress + path
    }
    score_trajectories(X.data(), Y.data(), Th.data(), 9, T, dt, 0.0f, {w}, p, cost);
    int best = 0; for (int c = 1; c < 9; ++c) if (cost[c] < cost[best]) best = c;
    for (int s = 0; s < 2; ++s) {            // apply for 0.1 s
      ryaw += ws[best % 3] * dt; rx += vs[best / 3] * std::cos(ryaw) * dt; ry += vs[best / 3] * std::sin(ryaw) * dt;
      w.x += w.vx * dt; w.y += w.vy * dt;
    }
    // true clearance: worker circle vs robot rectangle 0.82 x 0.62
    float dx = w.x - rx, dy = w.y - ry, px = std::cos(ryaw) * dx + std::sin(ryaw) * dy,
          py = -std::sin(ryaw) * dx + std::cos(ryaw) * dy;
    float ex = std::fmax(std::fabs(px) - 0.41f, 0), ey = std::fmax(std::fabs(py) - 0.31f, 0);
    float clear = std::hypot(ex, ey) - 0.25f;
    if (clear < min_clear) min_clear = clear;
    if (clear <= 0.0f) return {true, clear, rx};
  }
  return {false, min_clear, rx};
}

int main()
{
  // crossing: meeting point 2.2 m ahead, worker arrives there after 7 s (as in the scenario trigger)
  Obstacle crossing{2.2f, -4.2f, 0.0f, 0.6f};
  // head-on: worker 6 m ahead walking towards the robot on its path
  Obstacle head_on{6.0f, 0.05f, -0.6f, 0.0f};
  for (auto [name, o] : {std::pair<const char*, Obstacle>{"crossing", crossing}, {"head_on ", head_on}}) {
    for (auto [label, ext, pred] : {std::tuple<const char*, float, bool>{"no prediction        ", 0.f, false},
                                    {"prediction, no ext   ", 0.f, true},
                                    {"prediction + ext 3 s ", 3.f, true}}) {
      Result r = run(o, ext, pred);
      std::printf("%s | %s | contact=%-3s min clearance %.2f m | robot reached x=%.1f m\n",
                  name, label, r.contact ? "YES" : "no", r.min_clear, r.final_x);
    }
  }
}
