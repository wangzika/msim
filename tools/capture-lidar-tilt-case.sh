#!/usr/bin/env bash
# Capture one controlled sensor-tilt flight, then replay pure FAST-LIO.
set -euo pipefail
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 FORWARD_DOWN_DEG GROUP_DIR [DURATION_S]" >&2
  exit 2
fi
ROOT=$(cd "$(dirname "$0")/.." && pwd)
ANGLE=$1
GROUP=$(realpath -m "$2")
DURATION=${3:-42}
if ((DURATION < 4)); then
  echo "Duration must be at least 4 seconds" >&2
  exit 2
fi
case "$ANGLE" in
  0|5|10|15) ;;
  *) echo "Angle must be 0, 5, 10, or 15 degrees" >&2; exit 2 ;;
esac
FAST=${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}
PX4=${PX4_ROOT:-$ROOT/build/deps/PX4-Autopilot}
LABEL="forward-down-$ANGLE"
CASE_DIR="$GROUP/assets/$LABEL"
RUN="$GROUP/$LABEL"
python3 "$ROOT/tools/prepare-lidar-tilt-case.py" \
  --case-dir "$CASE_DIR" --forward-down-deg "$ANGLE" \
  --px4-root "$PX4" --fast-glio-root "$FAST"
export PATH="$FAST/.pixi/envs/default/bin:$PATH"
export CONDA_PREFIX="$FAST/.pixi/envs/default"
set +u
source "$FAST/devel/setup.bash"
source "$PX4/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
  "$PX4" "$PX4/build/px4_sitl_default"
set -u
export PX4_ROOT="$PX4"
"$ROOT/tools/build-gazebo-sensors.sh" > "$CASE_DIR/build-sensors.log" 2>&1
export GAZEBO_PLUGIN_PATH="$ROOT/build/gazebo-sensors:${GAZEBO_PLUGIN_PATH:-}"
export PYTHONPATH="$ROOT/build/px4-pydeps${PYTHONPATH:+:$PYTHONPATH}"
export ROS_PACKAGE_PATH="$ROOT/adapters/ros1_noetic:$PX4:$PX4/Tools/simulation/gazebo-classic:${ROS_PACKAGE_PATH:-}"
export MSIM_SENSOR_PROFILE="$ROOT/adapters/ros1_noetic/msim_sensors/config/sensor_profile.json"
export PX4_HOME_LAT=30.5312 PX4_HOME_LON=114.3601 PX4_HOME_ALT=50.0
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export ROS_LOG_DIR="$RUN/roslogs" ROS_HOME="$RUN/ros-home"
export MSIM_PX4_FLIGHT_PATTERN=route
echo "Capturing $ANGLE° forward-down tilt: $RUN"
python "$ROOT/adapters/ros1_noetic/fast_glio_uav/ros1_bag.py" capture \
  --run "$RUN" --duration-s "$DURATION" \
  --launch-file "$ROOT/adapters/ros1_noetic/fast_glio_uav/px4_world.launch" \
  --launch-arg "world:=$CASE_DIR/urban_canyon_msim.world" \
  --launch-arg "sdf:=$CASE_DIR/iris_msim.sdf" \
  --controller-script "$ROOT/adapters/ros1_noetic/fast_glio_uav/px4_patrol.py"
python "$ROOT/adapters/ros1_noetic/fast_glio_uav/ros1_bag.py" trajectory \
  --run "$RUN" --duration-s "$((DURATION - 2))" \
  --antenna-offset 0.05 0 0.04 > "$RUN/trajectory-build.log"
python "$ROOT/adapters/ros1_noetic/fast_glio_uav/verify_px4_flight.py" \
  --run "$RUN" > "$RUN/flight-verification.log"
python "$ROOT/tools/annotate-gazebo-lidar.py" --run "$RUN"
bash "$ROOT/tools/compare-fastglio-lio.sh" "$RUN" fast-lio-velodyne \
  "$CASE_DIR/fast_lio_tilt.yaml"
