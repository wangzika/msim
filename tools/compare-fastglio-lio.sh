#!/usr/bin/env bash
# Replay one recorded LiDAR/IMU stream into an isolated ROS master.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 RUN_DIR {fast-lio-velodyne|fast-lio-velodyne-full|fast-lio-tilt-identity|glio-front-end|lio-sam|lio-sam-patched|lio-sam-gyro-orientation|ligo} [CONFIG_YAML]" >&2
  exit 2
fi
ROOT=$(cd "$(dirname "$0")/.." && pwd)
RUN=$(realpath "$1")
ALGORITHM=$2
FAST=/home/wzb/FAST_GLIO
PLAY_RATE=${MSIM_LIO_PLAY_RATE:-1.0}
POINT_FILTER=${MSIM_LIO_POINT_FILTER:-2}
PLAY_BAG="$RUN/comparisons/annotated-sensors.bag"
POINT_TOPIC=/msim/velodyne_points_annotated
case "$ALGORITHM" in
  fast-lio-velodyne)
    PORT=11320
    ODOM_TOPIC=/Odometry
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.yaml"
    READY_NODE=/laserMapping
    ;;
  fast-lio-velodyne-full)
    PORT=11321
    ODOM_TOPIC=/Odometry
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.yaml"
    READY_NODE=/laserMapping
    ;;
  fast-lio-tilt-identity)
    PORT=11324
    ODOM_TOPIC=/Odometry
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/fast_lio_velodyne_benchmark.yaml"
    READY_NODE=/laserMapping
    ;;
  glio-front-end)
    PORT=11316
    ODOM_TOPIC=/odom
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/glio_lio_benchmark.launch"
    CONFIG_FILE="$FAST/GLIO/config/config_sim_velodyne.yaml"
    READY_NODE=/glio_lidar_odometry
    PLAY_BAG="$RUN/sensors.bag"
    POINT_TOPIC=/velodyne_points
    ;;
  lio-sam)
    PORT=11317
    ODOM_TOPIC=/lio_sam/mapping/odometry
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/lio_sam_lio_benchmark.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/lio_sam_benchmark.yaml"
    READY_NODE=/lio_sam_mapOptmization
    ;;
  lio-sam-patched)
    PORT=11319
    ODOM_TOPIC=/lio_sam/mapping/odometry
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/lio_sam_patched_benchmark.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/lio_sam_benchmark.yaml"
    READY_NODE=/lio_sam_mapOptmization
    ;;
  lio-sam-gyro-orientation)
    PORT=11322
    ODOM_TOPIC=/lio_sam/mapping/odometry
    LAUNCH_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/lio_sam_patched_benchmark.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/lio_sam_benchmark.yaml"
    READY_NODE=/lio_sam_mapOptmization
    PLAY_BAG="$RUN/comparisons/gyro-orientation-sensors.bag"
    ;;
  ligo)
    PORT=11318
    ODOM_TOPIC=/Odometry
    LAUNCH_FILE="$FAST/LIGO/launch/mapping_sim_velodyne.launch"
    CONFIG_FILE="$ROOT/adapters/ros1_noetic/fast_glio_uav/ligo_benchmark.yaml"
    READY_NODE=/laserMapping
    ;;
  *) echo "Unknown algorithm: $ALGORITHM" >&2; exit 2 ;;
esac
if [[ $# -eq 3 ]]; then
  CONFIG_FILE=$(realpath "$3")
fi
OUT="$RUN/comparisons/$ALGORITHM"
mkdir -p "$OUT/ros-home" "$OUT/roslogs"
if [[ "$ALGORITHM" == lio-sam-patched && ! -x "$OUT/lio_sam_mapOptmization" ]]; then
  python3 "$ROOT/tools/build-lio-sam-benchmark.py" --run "$RUN"
fi
MAPPER_BINARY="$OUT/lio_sam_mapOptmization"
if [[ "$ALGORITHM" == lio-sam-gyro-orientation ]]; then
  MAPPER_BINARY="$RUN/comparisons/lio-sam-patched/lio_sam_mapOptmization"
fi
export PATH="$FAST/.pixi/envs/default/bin:$PATH"
export CONDA_PREFIX="$FAST/.pixi/envs/default"
set +u
source "$FAST/devel/setup.bash"
set -u
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_IP=127.0.0.1
export ROS_HOME="$OUT/ros-home"
export ROS_LOG_DIR="$OUT/roslogs"
master_pid='' mapping_pid='' mapper_pid='' record_pid=''
cleanup() {
  for pid in "$record_pid" "$mapper_pid" "$mapping_pid" "$master_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM
roscore -p "$PORT" > "$OUT/roscore.log" 2>&1 & master_pid=$!
for _ in {1..100}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
rosparam set /use_sim_time true
if [[ "$ALGORITHM" == ligo ]]; then
  roslaunch "$LAUNCH_FILE" rviz:=false config_file:="$CONFIG_FILE" \
    > "$OUT/algorithm.log" 2>&1 & mapping_pid=$!
else
  extra_args=()
  if [[ "$ALGORITHM" == fast-lio-velodyne-full ]]; then
    extra_args=(point_filter_num:=1)
  elif [[ "$ALGORITHM" == fast-lio-velodyne || "$ALGORITHM" == fast-lio-tilt-identity ]]; then
    extra_args=(point_filter_num:="$POINT_FILTER")
  fi
  roslaunch "$LAUNCH_FILE" config_file:="$CONFIG_FILE" "${extra_args[@]}" \
    > "$OUT/algorithm.log" 2>&1 & mapping_pid=$!
fi
if [[ "$ALGORITHM" == lio-sam-patched || "$ALGORITHM" == lio-sam-gyro-orientation ]]; then
  for _ in {1..100}; do
    if [[ "$(rosparam get /lio_sam/sensor 2>/dev/null || true)" == velodyne ]]; then
      break
    fi
    sleep 0.1
  done
  "$MAPPER_BINARY" __name:=lio_sam_mapOptmization \
    > "$OUT/mapper.log" 2>&1 & mapper_pid=$!
fi
for _ in {1..200}; do
  if rosnode list 2>/dev/null | /bin/grep -Fx "$READY_NODE" >/dev/null; then break; fi
  sleep 0.1
done
rosparam dump "$OUT/parameters.yaml"
cp "$CONFIG_FILE" "$OUT/config.yaml"
rosbag record -O "$OUT/algorithm.bag" "$ODOM_TOPIC" /ground_truth/state \
  > "$OUT/record.log" 2>&1 & record_pid=$!
sleep 2
rosbag play -q --clock -r "$PLAY_RATE" "$PLAY_BAG" --topics \
  /imu/data "$POINT_TOPIC" /ground_truth/state \
  > "$OUT/play.log" 2>&1
sleep 3
kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=''
python "$ROOT/tools/evaluate-fastglio-replay.py" \
  --bag "$OUT/algorithm.bag" --odom-topic "$ODOM_TOPIC" \
  --output "$OUT/summary.json"
