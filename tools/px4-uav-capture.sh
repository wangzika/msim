#!/usr/bin/env bash
set -euo pipefail

MSIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_GLIO_ROOT="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
PX4_ROOT="${PX4_ROOT:-$MSIM_ROOT/build/deps/PX4-Autopilot}"
duration=300
gui=false
run=""
postprocess=false
manual=false
online_gnss=false
gnss_east_bias=0
gnss_north_bias=0
while (($#)); do
  case "$1" in
    --duration-s) duration="$2"; shift 2;;
    --gui) gui=true; shift;;
    --run) run="$2"; shift 2;;
    --postprocess-only) postprocess=true; shift;;
    --manual) manual=true; shift;;
    --online-gnss) online_gnss=true; shift;;
    --gnss-east-bias-m) gnss_east_bias="$2"; shift 2;;
    --gnss-north-bias-m) gnss_north_bias="$2"; shift 2;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
if [[ -z "$run" ]]; then
  run="$MSIM_ROOT/runs/uav-px4-$(date +%Y%m%d-%H%M%S)"
fi
run="$(realpath -m "$run")"
if ! $postprocess; then
  if [[ ! -x "$PX4_ROOT/build/px4_sitl_default/bin/px4" ]]; then
    echo "PX4 SITL not built: $PX4_ROOT/build/px4_sitl_default/bin/px4" >&2
    echo "Run $MSIM_ROOT/tools/setup-px4-sitl.sh first." >&2
    exit 2
  fi
  if [[ ! -f "$FAST_GLIO_ROOT/devel/setup.bash" ]]; then
    echo "FAST_GLIO ROS1 workspace not built: $FAST_GLIO_ROOT/devel/setup.bash" >&2
    exit 2
  fi
  export PATH="$FAST_GLIO_ROOT/.pixi/envs/default/bin:$PATH"
  set +u
  source "$FAST_GLIO_ROOT/devel/setup.bash"
  source "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
    "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
  set -u
  export PX4_ROOT
  export PYTHONPATH="$MSIM_ROOT/build/px4-pydeps${PYTHONPATH:+:$PYTHONPATH}"
  export ROS_PACKAGE_PATH="$PX4_ROOT:$PX4_ROOT/Tools/simulation/gazebo-classic:${ROS_PACKAGE_PATH:-}"
  export PX4_HOME_LAT=30.5312 PX4_HOME_LON=114.3601 PX4_HOME_ALT=50.0
  export ROS_LOG_DIR="$run/roslogs"
  export ROS_HOME="$run/ros-home"
  assets="$MSIM_ROOT/build/px4-assets"
  prepare=(--px4-root "$PX4_ROOT" --fast-glio-root "$FAST_GLIO_ROOT" --output-dir "$assets")
  if $online_gnss; then prepare+=(--online-gnss); fi
  /usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/prepare_px4_assets.py" "${prepare[@]}"
  model="$assets/iris_msim.sdf"
  if $online_gnss; then model="$assets/iris_msim_online_gnss.sdf"; fi
  capture=(--run "$run" --duration-s "$duration" \
    --launch-file "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_world.launch" \
    --launch-arg "world:=$assets/urban_canyon_msim.world" \
    --launch-arg "sdf:=$model")
  if $online_gnss; then
    export MSIM_GNSS_EAST_BIAS_M="$gnss_east_bias" MSIM_GNSS_NORTH_BIAS_M="$gnss_north_bias"
    capture+=(--injector-script "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_online_gnss.py")
  fi
  if ! $manual; then
    capture+=(--controller-script \
      "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_patrol.py")
  fi
  if $gui; then capture+=(--gui); fi
  echo "PX4 SITL 城市无人机采集：$run"
  python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/ros1_bag.py" capture "${capture[@]}"
  python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/ros1_bag.py" trajectory \
    --run "$run" --duration-s "$duration" --antenna-offset 0.05 0 0.04
  if ! $manual; then
    verify=(--run "$run")
    if $online_gnss; then verify+=(--online-gnss); fi
    python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/verify_px4_flight.py" "${verify[@]}"
  fi
fi
"$MSIM_ROOT/tools/fastglio-uav-capture.sh" --postprocess-only \
  --run "$run" --duration-s "$duration"
