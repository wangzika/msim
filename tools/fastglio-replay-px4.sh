#!/usr/bin/env bash
set -euo pipefail

MSIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_GLIO_ROOT="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
run="$MSIM_ROOT/runs/uav-px4-smoke2-20261105"
mode=tight
rate=1.0
port=11312
rviz=false
hold=false
output=""
while (($#)); do
  case "$1" in
    --run) run="$2"; shift 2;;
    --mode) mode="$2"; shift 2;;
    --rate) rate="$2"; shift 2;;
    --port) port="$2"; shift 2;;
    --rviz) rviz=true; shift;;
    --hold) hold=true; shift;;
    --output) output="$2"; shift 2;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
case "$mode" in lio|loose|tight|gins) ;; *) echo "Invalid mode: $mode" >&2; exit 2;; esac
run="$(realpath -m "$run")"
bag="$run/combined-signalsim.bag"
if [[ ! -f "$bag" ]]; then echo "Missing $bag" >&2; exit 2; fi
if [[ -z "$output" ]]; then
  output="$run/replays/fastglio-$mode-$(date +%Y%m%d-%H%M%S)"
fi
output="$(realpath -m "$output")"
mkdir -p "$output/ros-home" "$output/roslogs"
export PATH="$FAST_GLIO_ROOT/.pixi/envs/default/bin:$PATH"
export CONDA_PREFIX="$FAST_GLIO_ROOT/.pixi/envs/default"
set +u
source "$FAST_GLIO_ROOT/devel/setup.bash"
set -u
export ROS_MASTER_URI="http://127.0.0.1:$port"
export ROS_IP=127.0.0.1
export ROS_HOME="$output/ros-home"
export ROS_LOG_DIR="$output/roslogs"
master_pid=""; mapping_pid=""; record_pid=""; truth_pid=""
cleanup() {
  local pid
  for pid in "$record_pid" "$truth_pid" "$mapping_pid" "$master_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM
if rosparam list >/dev/null 2>&1; then
  echo "ROS master already uses port $port; choose another with --port" >&2; exit 2
fi
roscore -p "$port" > "$output/roscore.log" 2>&1 & master_pid=$!
for _ in {1..100}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  if ! kill -0 "$master_pid" 2>/dev/null; then
    echo "roscore exited; see $output/roscore.log" >&2; exit 1
  fi
  sleep 0.1
done
rosparam set /use_sim_time true
roslaunch fast_lio gnss2iekf_experiment.launch \
  config_file:="$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_replay.yaml" \
  mode:="$mode" rviz:="$rviz" \
  diag_log_path:="$output/gnss-diagnostics.csv" \
  tight_pose_log_path:="$output/gnss-tight-pose.csv" \
  lever_arm_body:='[0.05, 0.0, 0.04]' \
  > "$output/fastglio.log" 2>&1 & mapping_pid=$!
for _ in {1..200}; do
  if rosnode list 2>/dev/null | /bin/grep -Fx '/laserMapping' >/dev/null; then break; fi
  if ! kill -0 "$mapping_pid" 2>/dev/null; then
    echo "FAST_GLIO exited; see $output/fastglio.log" >&2; exit 1
  fi
  sleep 0.1
done
if ! rosnode list | /bin/grep -Fx '/laserMapping' >/dev/null; then
  echo "FAST_GLIO node did not start; see $output/fastglio.log" >&2; exit 1
fi
python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/truth_path.py" \
  > "$output/truth-path.log" 2>&1 & truth_pid=$!
rosbag record -O "$output/algorithm.bag" \
  /Odometry /path /loose_odom /loose_path /rtk_path /rtk_odom_aligned \
  /gnss_loose/diagnostics /ground_truth/state /ground_truth/aligned_path \
  > "$output/record.log" 2>&1 & record_pid=$!
sleep 2
echo "回放：$bag"
echo "算法模式：$mode；输出：$output"
rosbag play --clock -r "$rate" "$bag" --topics \
  /imu/data /velodyne_points /gnss_raw /rtklib_odom /ground_truth/state \
  > "$output/play.log" 2>&1
sleep 2
if ! kill -0 "$mapping_pid" 2>/dev/null; then
  echo "FAST_GLIO exited during replay; see $output/fastglio.log" >&2; exit 1
fi
kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=""
python "$MSIM_ROOT/tools/evaluate-fastglio-replay.py" \
  --bag "$output/algorithm.bag" --output "$output/summary.json"
echo "回放结束：$output"
if $hold; then
  read -r -p 'RViz 保持打开，按回车结束查看：' _ || true
fi
