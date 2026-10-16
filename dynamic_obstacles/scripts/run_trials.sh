#!/usr/bin/env bash
# Run a batch of trials, one fresh simulation per trial.
#
# Usage:   bash run_trials.sh <method> <runs> [scenario ...]
# Example: bash run_trials.sh dwb 5                  # 3 scenarios x 5 runs
#          bash run_trials.sh dwb 1 crossing         # one quick run
#
# Results: ~/logistic_ws/results/summary.csv (one row per trial)
# Logs:    ~/logistic_ws/results/logs/

METHOD=${1:-dwb}
RUNS=${2:-5}
shift 2 2>/dev/null
SCENARIOS=${*:-crossing head_on overtaking}

source ~/logistic_ws/env.sh
LOGS=~/logistic_ws/results/logs
mkdir -p "$LOGS"

cleanup() {
  pkill -INT -f "ros2 launch" 2>/dev/null
  sleep 5
  pkill -f "ign gazebo" 2>/dev/null
  pkill -f "parameter_bridge|robot_state_publisher|worker_mover|trial_runner" 2>/dev/null
  pkill -f "nav2_|orgnizer|nav2_clint|getin_controller|controller_manager" 2>/dev/null
  sleep 3
}

for s in $SCENARIOS; do
  for r in $(seq 1 "$RUNS"); do
    tag="${s}_${METHOD}_${r}"
    echo "=== $tag : starting $(date +%H:%M:%S) ==="
    cleanup
    ros2 launch robot_bringup igin_sim.xml > "$LOGS/sim_$tag.log" 2>&1 &
    sleep 40   # Gazebo + Nav2 start-up
    ros2 launch dynamic_obstacles workers.launch.py scenario:="$s" > "$LOGS/workers_$tag.log" 2>&1 &
    sleep 5
    timeout 1200 ros2 run dynamic_obstacles trial_runner --ros-args \
      -p scenario:="$s" -p method:="$METHOD" -p run:="$r" -p timeout:=400.0 \
      2>&1 | tee "$LOGS/trial_$tag.log" | grep -E "RESULT|Contact|Timeout"
  done
done

cleanup
echo
echo "=== Summary ==="
column -s, -t ~/logistic_ws/results/summary.csv
