# Warehouse robot navigation among moving people (ROS 2 Humble, Nav2, Gazebo Fortress)

Tech Project Internship, ITMO University (M.Sc. Robotics and AI) — Zyad Alshujaa.

A shelf-carrying warehouse robot runs a full logistics mission (drive to a shelf, lift it,
carry it to a drop-off point, return) while a simulated worker walks into its way.
This repository adds, on top of an existing simulation (see *Base project*):

1. a **test bench** with moving workers and repeatable, scripted trials,
2. a **LiDAR tracker** (constant-velocity Kalman filter) with ground-truth evaluation,
3. a **prediction-aware MPPI critic** for Nav2 (C++ plugin),
4. a **comparison of five navigation methods** in 108 valid simulated trials.

Every number below is either **measured** in this project (the data is in `results/`),
taken from a **cited source**, or explicitly marked as a **hypothesis**.

<!--
  ZYAD: write the "Base project" section here yourself
  (what the original project provides, its author, link, permission).
-->
## Base project

_To be written._

---

## 1. What I added

| Package | Language | What it does |
|---|---|---|
| `dynamic_obstacles` | Python | Spawns workers in Gazebo, moves them on scripted lines that start when the robot reaches a trigger point (repeatable). `trial_runner` runs one mission and records robot/worker trajectories, minimum clearance and contacts; `run_trials.sh` runs batches with a fresh simulation per trial, detects failed Nav2 start-up and retries; `summarize.py` builds the result table. |
| `obstacle_tracker` | Python | Detects and tracks people from the 2D LiDAR: map-based static filtering, scan clustering, constant-velocity Kalman filter per track, 1.5 s coasting of lost tracks. Publishes `/tracked_obstacles` (nav_msgs/Odometry, map frame). `tracker_eval` compares the tracks with the true worker position. |
| `predictive_nav` | C++ | `DynamicObstacleCritic`, an MPPI critic plugin for the Humble MPPI controller, plus the Nav2 configurations of all compared methods. The cost model is a ROS-free header with unit tests. |

Fixes to the base simulation that were needed to run it (portable model paths, hardware-only
dependencies removed, LiDAR-based get-in controller in the full cycle) are in the first commits.

## 2. Test bench

* Three scenarios on the loaded leg of the mission (pick-up → drop-off), worker = cylinder
  r = 0.25 m, non-reactive (it never yields), started by a trigger so every run starts the
  same way: **crossing** (0.6 m/s, across the path), **head_on** (0.6 m/s, towards the robot),
  **overtaking** (0.2 m/s, robot catches up).
* A trial ends at the first contact (clearance ≤ 0.02 m) because the simulated worker then
  pushes the robot, which says nothing about navigation.
* Ground truth: the worker has no gravity/friction and moves at the commanded velocity, so
  its integrated command is its true position.

## 3. Tracker and prediction (measured)

| Quantity | Result | Data |
|---|---|---|
| Coverage when the LiDAR sees the worker (≥ 3 points) | **99 %** (200/202 samples) | `results/tracker/` |
| Position error | median **12.7 cm**, p95 26 cm | same |
| Velocity error (worker at 0.6 m/s) | median **0.14 m/s** | same |
| LiDAR loses the worker | when closer than ≈ 1–1.5 m or behind the robot | same |

Prediction error of the worker position (median, one crossing run, 65–85 samples):

| Horizon | constant-velocity prediction | no prediction (worker assumed still) |
|---|---|---|
| 1 s | **0.18 m** | 0.55 m |
| 2 s | **0.32 m** | 1.13 m |
| 3 s | **0.50 m** | 1.72 m |

## 4. The critic

For every MPPI candidate trajectory the critic places each tracked person at its predicted
position `p + v·(t + latency)` and covers the robot (0.82 × 0.62 m) with two circles.

* overlap at any time → collision cost (10 000); closer than a margin → soft penalty;
* the margin grows with prediction time by 0.15 m/s — chosen from the measured growth of
  the prediction error above;
* **terminal extension (v2):** each candidate is extended 3 s beyond the 2.8 s MPPI horizon
  with its final velocity. Reason, measured: with v1 the robot braked *in* the worker's path
  (see 5.2). At 0.3 m/s the MPPI horizon covers only 0.84 m, less than the space needed
  to stop clear of a person.
* `use_prediction: false` gives the ablation (same critic, worker assumed to stand still).

Unit tests: `predictive_nav/test/test_cost.cpp` (cost model) and `test_closed_loop.cpp`
(a toy 9-candidate controller, *not* MPPI, used only as a sanity check).

## 5. Experiments

Methods (identical Nav2 configuration except the controller part):

| Name | Controller |
|---|---|
| `dwb` | DWB, the base project's configuration (≈ Nav2 Humble defaults) |
| `mppi` | MPPI, values of the Nav2 Humble README example, same speed limits as DWB |
| `mppi_static` | MPPI + critic **without** prediction (ablation) |
| `mppi_pred` | MPPI + critic v1 (prediction, no terminal extension) |
| `mppi_pred2` | MPPI + critic v2 (prediction + terminal extension) |

### 5.1 Results (all valid trials; contact = trial ended in a collision)

| Scenario | dwb | mppi | mppi_static | mppi_pred (v1) | mppi_pred2 (v2) |
|---|---|---|---|---|---|
| crossing | 5/5 | 10/16 | 12/15 | 4/6 | 10/14 |
| head_on | 0/5 | 0/5 | 3/6 | 3/5 | 1/5 |
| overtaking | 4/5 | 1/6 | 0/5 | 0/5 | 0/5 |
| **all** | **9/15** | **11/27** | **15/26** | **7/16** | **11/24** |
| timeouts | 0 | 1 | 1 | 2 | 1 |

Fisher exact test, contacts of `mppi_pred2` vs `mppi`: p = 0.78 (all), p = 0.71 (crossing).
**No significant difference.** Raw data: `results/summary_all_valid.csv`.

Rules: trials where Nav2 failed to start, or where the robot never reached the worker,
are excluded (they are kept, marked, in `results/summary.csv`); every other trial counts,
including repeats.

### 5.2 Findings

1. **MPPI collides less than DWB** (11/27 vs 9/15) and keeps more distance in head-on
   (median 0.60 m vs 0.13 m).
2. **Critic v1 caused the "freezing robot" behaviour.** Before contact the robot slowed to
   0.00–0.07 m/s and was hit from the front by the (non-reactive) worker, whereas plain MPPI
   never slowed below 0.24 m/s and was clipped from the back or side (measured from the
   trajectories). This is the known freezing robot problem (Trautman & Krause, 2010).
3. **The terminal extension removed most of it in head-on** (3/5 → 1/5 contacts).
4. **In crossing the critic does not reduce contacts** (10/14 vs 10/16 for MPPI). In 9 of the
   10 crossing contacts with v2 the robot had slowed below 0.12 m/s in the last 5 s, i.e.
   it still brakes near the worker's path instead of before it, while plain MPPI drives
   through and is clipped. The first five crossing runs (2/5 contacts) looked promising,
   but ten more runs did not confirm it.

### 5.3 Limitations and next steps (hypotheses, not tested)

* The worker never yields and walks at twice the robot's speed; a reactive pedestrian model
  (e.g. social force) would be a fairer test of a predictive planner.
* Stopping still looks attractive to the critic in crossing; a cost on the *time spent in
  the predicted swept area*, or a longer extension, may help.
* The critic weights are first choices, not tuned.
* The LiDAR loses a person closer than ≈ 1–1.5 m; 1.5 s coasting may not be enough.
* Simulation only (Gazebo Fortress under WSL2, software rendering).

## 6. How to run

```bash
# build (ROS 2 Humble; needs ros-humble-nav2-mppi-controller)
cd ~/logistic_ws && colcon build --symlink-install && source install/setup.bash

# one batch: method name, runs, scenarios
CFG=$(ros2 pkg prefix predictive_nav)/share/predictive_nav/config
bash src/warehouse_dynamic_nav/dynamic_obstacles/scripts/run_trials.sh dwb 5
NAV2_CONFIG=$CFG/nav2_mppi.yaml bash .../run_trials.sh mppi 5
TRACKER=1 NAV2_CONFIG=$CFG/nav2_mppi_predictive.yaml bash .../run_trials.sh mppi_pred2 5
TRACKER=1 NAV2_CONFIG=$CFG/nav2_mppi_static.yaml bash .../run_trials.sh mppi_static 5

# tracker accuracy against ground truth (with a simulation and the tracker running)
ros2 run obstacle_tracker tracker_eval --ros-args -p tag:=crossing_1   # Ctrl+C prints the summary
```

## References

* Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and
  Navigation*, Wiley, 2001 — constant-velocity Kalman filter.
* P. Trautman, A. Krause, "Unfreezing the robot: Navigation in dense, interacting crowds",
  IROS 2010 — freezing robot problem.
* Nav2 MPPI controller (Humble), `nav2_mppi_controller` README and source, v1.1.20.
