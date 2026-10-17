#!/usr/bin/env bash
# Run a batch of trials, one fresh simulation per trial.
#
# Usage:   bash run_trials.sh <method> <runs> [scenario ...]
# Example: bash run_trials.sh dwb 5                  # 3 scenarios x 5 runs
#          bash run_trials.sh dwb 1 crossing         # one quick run
#          bash run_trials.sh mppi 1,2 overtaking    # only runs 1 and 2 (re-run)
#
# Another Nav2 parameter file (e.g. the MPPI baseline):
#   NAV2_CONFIG=/path/to/nav2_mppi.yaml bash run_trials.sh mppi 5
#
# Results: ~/logistic_ws/results/summary.csv (one row per trial)
# Logs:    ~/logistic_ws/results/logs/

METHOD=${1:-dwb}
RUNS=${2:-5}
shift 2 2>/dev/null
SCENARIOS=${*:-crossing head_on overtaking}
if [[ "$RUNS" == *,* ]]; then RUN_LIST=${RUNS//,/ }; else RUN_LIST=$(seq 1 "$RUNS"); fi

source ~/logistic_ws/env.sh
NAV2_ARG=()
if [ -n "$NAV2_CONFIG" ]; then
  [ -f "$NAV2_CONFIG" ] || { echo "NAV2_CONFIG not found: $NAV2_CONFIG"; exit 1; }
  NAV2_ARG=(nav2_config:="$NAV2_CONFIG")
  echo "Nav2 parameters: $NAV2_CONFIG"
fi
LOGS=~/logistic_ws/results/logs
mkdir -p "$LOGS"

cleanup() {
  pkill -INT -f "ros2 launch" 2>/dev/null
  sleep 5
  PATTERN="ign gazebo|parameter_bridge|robot_state_publisher|worker_mover|trial_runner|nav2_|orgnizer|nav2_clint|getin_controller|controller_manager|controller_server|planner_server|bt_navigator|behavior_server|amcl|map_server|lifecycle_manager|velocity_smoother|waypoint_follower|smoother_server|spawner|ros2 launch"
  pkill -f "$PATTERN" 2>/dev/null
  # wait until everything is gone; force-kill what is left after 15 s
  for i in $(seq 1 15); do
    pgrep -f "$PATTERN" > /dev/null || break
    sleep 1
  done
  pkill -9 -f "$PATTERN" 2>/dev/null
  sleep 2
}

run_one() {   # scenario run tag
  cleanup
  ros2 launch robot_bringup igin_sim.xml "${NAV2_ARG[@]}" > "$LOGS/sim_$3.log" 2>&1 &
  sleep 40   # Gazebo + Nav2 start-up
  ros2 launch dynamic_obstacles workers.launch.py scenario:="$1" > "$LOGS/workers_$3.log" 2>&1 &
  sleep 5
  timeout 1200 ros2 run dynamic_obstacles trial_runner --ros-args \
    -p scenario:="$1" -p method:="$METHOD" -p run:="$2" -p timeout:=400.0 \
    2>&1 | tee "$LOGS/trial_$3.log" | grep -E "RESULT|Contact|Timeout|Start-up failed"
}

for s in $SCENARIOS; do
  for r in $RUN_LIST; do
    tag="${s}_${METHOD}_${r}"
    echo "=== $tag : starting $(date +%H:%M:%S) ==="
    run_one "$s" "$r" "$tag"
    if grep -q "outcome=startup_failed" "$LOGS/trial_$tag.log" || ! grep -q "RESULT" "$LOGS/trial_$tag.log"; then
      echo "=== $tag : no valid result (start-up failed or hung), retrying once ==="
      mv "$LOGS/sim_$tag.log" "$LOGS/sim_${tag}_failed.log" 2>/dev/null
      run_one "$s" "$r" "$tag"
    fi
  done
done
cleanup
echo
echo "=== Summary ==="
python3 "$(dirname "$0")/summarize.py" ~/logistic_ws/results/summary.csv
