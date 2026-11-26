#!/usr/bin/env bash
set -euo pipefail

MSIM_ROOT="${MSIM_CAPTURE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# Bash streams long scripts. Run a private copy so editing the repository
# during a flight cannot shift the reader's offset near post-processing.
if [[ "${MSIM_CAPTURE_SNAPSHOT:-}" != "${BASH_SOURCE[0]}" ]]; then
  capture_snapshot=$(mktemp /tmp/msim-px4-capture.XXXXXX.sh)
  cp -- "${BASH_SOURCE[0]}" "$capture_snapshot"
  export MSIM_CAPTURE_ROOT="$MSIM_ROOT" MSIM_CAPTURE_SNAPSHOT="$capture_snapshot"
  exec bash "$capture_snapshot" "$@"
fi
cleanup_capture_snapshot() { rm -f -- "$MSIM_CAPTURE_SNAPSHOT"; }
trap cleanup_capture_snapshot EXIT
FAST_GLIO_ROOT="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
PX4_ROOT="${PX4_ROOT:-$MSIM_ROOT/build/deps/PX4-Autopilot}"
sensor_profile=""
scene_profile=""
sensor_args=()
scene=simple
world_input=""
custom_world=false
frontend=auto
point_filter_num=auto
local_map_demo=""
wait_for_recording=false
cube_side_length=1000
fusion_delay_s=0.65
route_clock=wall
duration=300
duration_supplied=false
teleop=false
autonomy=false
localization_map=""
autonomy_distance=8
return_after_s=30
autonomy_timeout=150
goal_enu=""
mission_speed=2.0
goal_radius=2.0
autonomy_timeout_supplied=false
teleop_speed=2.0
teleop_height=5.0
gui=false
gazebo_viz=false
fastglio_live=false
web_console=false
web_port=8766
headless=false
skip_signalsim=false
verification_ok=true
run=""
postprocess=false
manual=false
flight_pattern=patrol
online_gnss=true
gnss_east_bias=0
gnss_north_bias=0
spoof_start_s=inf
spoof_east_rate=0
spoof_north_rate=0
while (($#)); do
  case "$1" in
    --duration-s) duration="$2"; duration_supplied=true; shift 2;;
    --teleop) teleop=true; shift;;
    --autonomy) autonomy=true; shift;;
    --localization-map) localization_map="$2"; shift 2;;
    --autonomy-distance-m) autonomy_distance="$2"; shift 2;;
    --return-after-s) return_after_s="$2"; shift 2;;
    --autonomy-timeout-s) autonomy_timeout="$2"; autonomy_timeout_supplied=true; shift 2;;
    --goal-enu) goal_enu="$2,$3,$4"; autonomy=true; teleop_height="$4"; shift 4;;
    --mission-speed-mps) mission_speed="$2"; shift 2;;
    --goal-radius-m) goal_radius="$2"; shift 2;;
    --teleop-speed-mps) teleop_speed="$2"; shift 2;;
    --teleop-height-m) teleop_height="$2"; shift 2;;
    --gui) gui=true; shift;;
    --gazebo-viz) gazebo_viz=true; shift;;
    --fastglio-live) fastglio_live=true; shift;;
    --web) web_console=true; shift;;
    --web-port) web_port="$2"; shift 2;;
    --headless) headless=true; shift;;
    --skip-signalsim) skip_signalsim=true; shift;;
    --run) run="$2"; shift 2;;
    --sensor-profile) sensor_profile="$2"; shift 2;;
    --scene-profile) scene_profile="$2"; shift 2;;
    --list-sensors) /usr/bin/python3 "$MSIM_ROOT/tools/sensor-config.py" --list; exit 0;;
    --lidar-model|--imu-model|--gps-model|--lidar-timing) sensor_args+=("$1" "$2"); shift 2;;
    --help) echo 'px4-uav-capture.sh [--web [--web-port 8766] [--teleop]] [--goal-enu E N U --mission-speed-mps 2 --goal-radius-m 2] [--autonomy [--localization-map directory] [--autonomy-distance-m 8] [--return-after-s 30] [--autonomy-timeout-s 150] [--teleop-height-m 5]] [--fastglio-live --flight-pattern route] [--duration-s 60] [--scene simple|complex_urban|industrial_yard|ksql_airport] [--scene-profile file.json] [--lidar-model vlp16|hdl32|os1-64|mid360] [--imu-model mems-baseline|low-noise] [--gps-model f9p-dgnss-ucr(default)|standalone(uncalibrated)|native|ideal] [--lidar-timing rolling|snapshot] [--list-sensors] [--list-scenes] [--local-map-demo before|after] [--wait-for-recording]'; exit 0;;
    --scene) scene="$2"; shift 2;;
    --list-scenes) /usr/bin/python3 "$MSIM_ROOT/tools/scene-list.py"; exit 0;;
    --world) world_input="$2"; custom_world=true; shift 2;;
    --fastglio-frontend) frontend="$2"; shift 2;;
    --point-filter-num) point_filter_num="$2"; shift 2;;
    --local-map-demo) local_map_demo="$2"; shift 2;;
    --wait-for-recording) wait_for_recording=true; shift;;
    --fusion-delay-s) fusion_delay_s="$2"; shift 2;;
    --route-clock) route_clock="$2"; shift 2;;
    --postprocess-only) postprocess=true; shift;;
    --manual) manual=true; shift;;
    --flight-pattern) flight_pattern="$2"; shift 2;;
    --online-gnss) online_gnss=true; shift;;
    --gnss-east-bias-m) gnss_east_bias="$2"; shift 2;;
    --gnss-north-bias-m) gnss_north_bias="$2"; shift 2;;
    --spoof-start-s) spoof_start_s="$2"; shift 2;;
    --spoof-east-rate-mps) spoof_east_rate="$2"; shift 2;;
    --spoof-north-rate-mps) spoof_north_rate="$2"; shift 2;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
if [[ -n "$local_map_demo" ]]; then
  case "$local_map_demo" in
    before) cube_side_length=200;;
    after) cube_side_length=1000;;
    *) echo '--local-map-demo 只接受 before 或 after。' >&2; exit 2;;
  esac
  if $autonomy; then echo '局部地图问题展示使用 GPS 航线，不与依赖 LIO 的自主任务同时运行。' >&2; exit 2; fi
  fastglio_live=true
fi
export MSIM_LOCAL_MAP_DEMO="$local_map_demo"
if $autonomy; then
  teleop=true
  if [[ -n "$goal_enu" ]]; then teleop_height="${goal_enu##*,}"; fi
  fusion_delay_s=0
  if [[ -n "$goal_enu" ]] && ! $autonomy_timeout_supplied; then autonomy_timeout=900; fi
  /usr/bin/python3 - "$autonomy_distance" "$return_after_s" "$autonomy_timeout" "$localization_map" "$scene" "$custom_world" "$goal_enu" "$mission_speed" "$goal_radius" <<'PY'
import hashlib,json,math,sys
from pathlib import Path
distance,after,timeout=map(float,sys.argv[1:4])
goal=sys.argv[7]
if not all(math.isfinite(v) for v in (distance,after,timeout)) or not 2<=distance<=30 or not 15<=after<=120 or not after+20<=timeout<=(3600 if goal else 300):
    raise SystemExit('自主模式距离须为 2～30 m，返航时间为 15～120 s，总超时至少再加 20 s；短距模式最多 300 s，目标任务最多 3600 s。')
speed,radius=map(float,sys.argv[8:10])
if not math.isfinite(speed) or not .3<=speed<=3 or not math.isfinite(radius) or not .5<=radius<=30:raise SystemExit('任务速度须为 0.3～3 m/s，目标区域半径须为 0.5～30 m。')
if goal:
    e,n,u=map(float,goal.split(','))
    if radius>=math.hypot(e,n)/2:raise SystemExit('目标区域半径须小于目标水平距离的一半。')
    if not all(math.isfinite(v) for v in (e,n,u)) or not 5<=math.hypot(e,n)<=1000 or not 3<=u<=30:raise SystemExit('目标须为距起飞点 5～1000 m，高度 3～30 m 的相对坐标。')
if sys.argv[4] and not (Path(sys.argv[4])/'map.json').is_file():raise SystemExit('缺少先验地图包 map.json')
if sys.argv[4]:
    path=Path(sys.argv[4]);info=json.loads((path/'map.json').read_text())
    if info.get('scene')!=('custom' if sys.argv[6]=='true' else sys.argv[5]):raise SystemExit('先验地图与当前场景不一致；拒绝启动。')
    if not (path/'map.npz').is_file() or info.get('sha256')!=hashlib.sha256((path/'map.npz').read_bytes()).hexdigest():raise SystemExit('先验地图校验失败；拒绝启动。')
PY
elif [[ -n "$localization_map" ]]; then
  echo '--localization-map 须与 --autonomy 同时使用。' >&2; exit 2
fi
if $web_console; then
  headless=true
  fastglio_live=true
  gui=false
  if [[ ! "$web_port" =~ ^[0-9]+$ ]] || ((web_port<1024 || web_port>65535)); then
    echo '--web-port 须为 1024～65535 的端口。' >&2; exit 2
  fi
  /usr/bin/python3 - "$web_port" <<'PY'
import socket,sys
port=int(sys.argv[1])
with socket.socket() as listener:
    listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    try:listener.bind(('127.0.0.1',port))
    except OSError:raise SystemExit(f'网页端口 {port} 已被占用。请先结束旧采集或旧预览，再启动新命令；同一套 PX4/ROS 不能重复启动。')
PY
fi
if $teleop; then
  if $postprocess; then echo '--teleop 与 --postprocess-only 不能同时使用。' >&2; exit 2; fi
  manual=true
  fastglio_live=true
  skip_signalsim=true
  if ! $duration_supplied; then duration=0; fi
  /usr/bin/python3 - "$teleop_speed" "$teleop_height" <<'PY'
import math,sys
speed,height=map(float,sys.argv[1:])
if not math.isfinite(speed) or not .2<=speed<=8 or not math.isfinite(height) or not .5<=height<=30:
    raise SystemExit('手动模式速度须为 0.2～8 m/s，起飞高度须为 0.5～30 m。')
PY
fi
if [[ ! "$duration" =~ ^[0-9]+$ ]] || { ((duration<2)) && { ! $teleop || ((duration!=0)); }; }; then
  echo '--duration-s 须为至少 2 秒的整数；手动操纵模式可用 0 表示持续采集。' >&2; exit 2
fi
/usr/bin/python3 - "$fusion_delay_s" "$spoof_start_s" "$spoof_east_rate" "$spoof_north_rate" "$gnss_east_bias" "$gnss_north_bias" <<'PY'
import math,sys
delay,start,east,north,be,bn=map(float,sys.argv[1:])
if not math.isfinite(delay) or not 0<=delay<=2:
    raise SystemExit('--fusion-delay-s 必须在 0～2 秒之间')
if math.isnan(start) or start<0 or not all(math.isfinite(v) and abs(v)<=10 for v in (east,north)):
    raise SystemExit('欺骗开始时间必须非负，漂移速度必须在 ±10 m/s 内')
if not all(math.isfinite(v) and abs(v)<=1000 for v in (be,bn)):
    raise SystemExit('固定 GPS 偏移必须在 ±1000 m 内')
PY
if [[ "$frontend" != auto && "$frontend" != marsim && "$frontend" != velodyne && "$frontend" != livox ]]; then
  echo "--fastglio-frontend must be auto, marsim, velodyne or livox" >&2; exit 2
fi
if [[ "$route_clock" != wall && "$route_clock" != sim ]]; then
  echo "--route-clock must be wall or sim" >&2; exit 2
fi
if [[ "$point_filter_num" != auto && ! "$point_filter_num" =~ ^[1-9][0-9]*$ ]]; then
  echo "--point-filter-num must be auto or a positive integer" >&2; exit 2
fi
case "$scene" in
  simple|complex_urban|industrial_yard) ;;
  ksql_airport)
    if [[ -z "$scene_profile" ]]; then scene_profile="$MSIM_ROOT/scenes/ksql_airport.profile.json"; fi
    ;;
  *) echo "--scene must be simple, complex_urban, industrial_yard or ksql_airport" >&2; exit 2;;
esac
if ! $postprocess && [[ -z "$world_input" && "$scene" != simple ]]; then
  if [[ "$scene" == ksql_airport ]]; then
    /usr/bin/python3 "$MSIM_ROOT/tools/prepare-ksql-scene.py" --profile "$scene_profile"
  fi
  world_input="$MSIM_ROOT/scenes/$scene.world"
fi
if [[ "$flight_pattern" != patrol && "$flight_pattern" != hover && "$flight_pattern" != route ]]; then
  echo "--flight-pattern must be patrol, hover or route" >&2
  exit 2
fi
if $fastglio_live; then gazebo_viz=true; fi
if $wait_for_recording && ! $fastglio_live; then
  echo '--wait-for-recording 须与 --fastglio-live 或 --local-map-demo 一起使用。' >&2; exit 2
fi
if $gazebo_viz; then
  gui=true
  online_gnss=true
fi
if $headless; then gui=false; fi
if [[ -z "$run" ]]; then
  recording_root=$(/usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/run_storage.py" root "$MSIM_ROOT")
  run="$recording_root/uav-px4-$(date +%Y%m%d-%H%M%S)"
fi
run="$(realpath -m "$run")"
/usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/run_storage.py" link "$MSIM_ROOT" "$run"
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
  export CONDA_PREFIX="$FAST_GLIO_ROOT/.pixi/envs/default"
  set +u
  source "$FAST_GLIO_ROOT/devel/setup.bash"
  source "$PX4_ROOT/Tools/simulation/gazebo-classic/setup_gazebo.bash" \
    "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
  set -u
  export PX4_ROOT
  export PYTHONPATH="$MSIM_ROOT/build/px4-pydeps${PYTHONPATH:+:$PYTHONPATH}"
  export ROS_PACKAGE_PATH="$MSIM_ROOT/adapters/ros1_noetic:$PX4_ROOT:$PX4_ROOT/Tools/simulation/gazebo-classic:${ROS_PACKAGE_PATH:-}"
  runtime_dir=$(/usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/run_storage.py" runtime "$run")
  export ROS_LOG_DIR="$runtime_dir/roslogs"
  export ROS_HOME="$runtime_dir/ros-home"
  if [[ "$runtime_dir" != "$run" ]]; then
    export MSIM_RUNTIME_DIR="$runtime_dir"
    finish_runtime_storage() {
      capture_exit_code=$?
      trap - EXIT
      /usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/run_storage.py" archive "$runtime_dir" "$run" --cleanup || echo "运行日志仍保留在 $runtime_dir，请检查归档。" >&2
      cleanup_capture_snapshot
      exit "$capture_exit_code"
    }
    trap finish_runtime_storage EXIT
  else
    unset MSIM_RUNTIME_DIR
  fi
  assets="$MSIM_ROOT/build/px4-assets/$scene-${run##*/}"
  scene_args=()
  if [[ -n "$scene_profile" ]]; then scene_args+=(--profile "$(realpath "$scene_profile")"); fi
  /usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/scene_profile.py" \
    "${scene_args[@]}" --output "$assets/scene-profile.json"
  export MSIM_SCENE_PROFILE="$assets/scene-profile.json"
  mapfile -t scene_values < <(/usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/scene_profile.py" \
    --profile "$MSIM_SCENE_PROFILE" --values)
  export PX4_HOME_LAT="${scene_values[0]}" PX4_HOME_LON="${scene_values[1]}" PX4_HOME_ALT="${scene_values[2]}"
  export MSIM_ROUTE_PLAN="${MSIM_ROUTE_PLAN:-${scene_values[7]}}"
  if [[ -n "${scene_values[8]:-}" ]]; then
    export GAZEBO_RESOURCE_PATH="${scene_values[8]}:${GAZEBO_RESOURCE_PATH:-}"
  fi
  if [[ -n "$sensor_profile" ]]; then sensor_args+=(--sensor-profile "$(realpath "$sensor_profile")"); fi
  /usr/bin/python3 "$MSIM_ROOT/tools/sensor-config.py" "${sensor_args[@]}" --output "$assets/sensor_profile.json"
  sensor_profile="$assets/sensor_profile.json"
  pattern=$(/usr/bin/python3 - "$sensor_profile" <<'PY'
import json,sys
print(json.load(open(sys.argv[1]))['lidar']['scan_pattern'])
PY
)
  if [[ "$frontend" == auto ]]; then
    if [[ "$pattern" == livox_csv ]]; then frontend=livox; else frontend=velodyne; fi
  fi
  if [[ "$point_filter_num" == auto ]]; then
    if [[ "$pattern" == livox_csv ]]; then point_filter_num=1; else point_filter_num=2; fi
  fi
  if [[ "$pattern" == livox_csv && "$frontend" != livox ]] || [[ "$pattern" != livox_csv && "$frontend" == livox ]]; then
    echo 'Mid-360 使用 livox 原生入口，其他雷达使用 velodyne 或快照 marsim 入口。' >&2; exit 2
  fi
  if [[ "$frontend" == marsim ]] && ! /usr/bin/python3 - "$sensor_profile" <<'PY'
import json,sys
sys.exit(json.load(open(sys.argv[1]))['lidar']['timing']!='snapshot')
PY
  then
    echo 'MARSIM 入口只支持快照；请选择 --fastglio-frontend velodyne，或显式使用 --lidar-timing snapshot。' >&2
    exit 2
  fi
  "$MSIM_ROOT/tools/build-gazebo-sensors.sh" > "$assets/build-sensors.log" 2>&1 || { tail -40 "$assets/build-sensors.log"; exit 1; }
  export GAZEBO_PLUGIN_PATH="$MSIM_ROOT/build/gazebo-sensors:${GAZEBO_PLUGIN_PATH:-}"
  export MSIM_SENSOR_PROFILE="$sensor_profile"
  prepare=(--px4-root "$PX4_ROOT" --fast-glio-root "$FAST_GLIO_ROOT" --output-dir "$assets" --sensor-profile "$sensor_profile")
  if [[ -n "$world_input" ]]; then prepare+=(--world-input "$world_input"); fi
  if $online_gnss; then prepare+=(--online-gnss); fi
  if $gazebo_viz; then prepare+=(--gazebo-viz); fi
  /usr/bin/python3 "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/prepare_px4_assets.py" "${prepare[@]}"
  model="$assets/iris_msim.sdf"
  if $online_gnss; then model="$assets/iris_msim_online_gnss.sdf"; fi
  export MSIM_WORLD_SOURCE="$assets/urban_canyon_msim.world" MSIM_MODEL_SOURCE="$model"
  export MSIM_SCENE_KEY="$scene"
  if $custom_world; then export MSIM_SCENE_KEY=custom; fi
  export MSIM_FASTGLIO_FRONTEND="$frontend"
  export MSIM_FUSION_DELAY_S="$fusion_delay_s"
  export MSIM_AUTONOMY=0 MSIM_GOAL_ENU="" MSIM_LOCALIZATION_MAP=""
  if $teleop; then
    export MSIM_TELEOP_RUN="$run" MSIM_TELEOP_STOP_FILE="$run/capture-stop.json"
    export DISABLE_ROS1_EOL_WARNINGS=1
    export MSIM_TELEOP_SPEED_MPS="$teleop_speed" MSIM_TELEOP_HEIGHT_M="$teleop_height"
    export MSIM_TELEOP_HEADLESS=0
    if $headless; then export MSIM_TELEOP_HEADLESS=1; fi
    export MSIM_PX4_FLIGHT_PATTERN=teleop
    if $autonomy; then
      export MSIM_AUTONOMY=1 MSIM_AUTONOMY_DISTANCE_M="$autonomy_distance" MSIM_RETURN_AFTER_S="$return_after_s" MSIM_AUTONOMY_TIMEOUT_S="$autonomy_timeout"
      export MSIM_LOCALIZATION_MAP="$localization_map"
      export MSIM_GOAL_ENU="$goal_enu" MSIM_MISSION_SPEED_MPS="$mission_speed" MSIM_GOAL_RADIUS_M="$goal_radius"
      export MSIM_PX4_FLIGHT_PATTERN=autonomy
    fi
  fi
  capture=(--run "$run" --duration-s "$duration" \
    --launch-file "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_world.launch" \
    --launch-arg "world:=$assets/urban_canyon_msim.world" \
    --launch-arg "sdf:=$model" \
    --launch-arg "sensor_profile:=$sensor_profile")
  capture+=(--launch-arg "fusion_delay_s:=$fusion_delay_s")
  if $web_console; then
    capture+=(--web-console-script "$MSIM_ROOT/tools/web-sim-console.py" --web-console-port "$web_port")
  fi
  capture+=(--launch-arg "spawn_x:=${scene_values[3]}" \
    --launch-arg "spawn_y:=${scene_values[4]}" --launch-arg "spawn_z:=${scene_values[5]}")
  if $online_gnss; then
    export MSIM_GNSS_SOURCE=gazebo_plugin
    export MSIM_GNSS_EAST_BIAS_M="$gnss_east_bias" MSIM_GNSS_NORTH_BIAS_M="$gnss_north_bias"
    export MSIM_SPOOF_START_S="$spoof_start_s"
    export MSIM_SPOOF_EAST_RATE_MPS="$spoof_east_rate"
    export MSIM_SPOOF_NORTH_RATE_MPS="$spoof_north_rate"
    capture+=(--injector-script "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_online_gnss.py")
  fi
  if $gazebo_viz && { ! $headless || [[ -n "$local_map_demo" ]]; }; then
    capture+=(--gazebo-viz-script \
      "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/gazebo_spoof_viz.py")
  fi
  if $fastglio_live; then
    fastglio_config="$assets/fastglio_${frontend}.yaml"
    python - "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_replay.yaml" "$fastglio_config" "$sensor_profile" "$frontend" <<'PY'
import sys,yaml,json
from pathlib import Path
config=yaml.safe_load(Path(sys.argv[1]).read_text())
config['common']['lid_topic']='/msim/livox_lidar_for_fusion' if sys.argv[4]=='livox' else '/msim/velodyne_points_annotated'
config['preprocess']['lidar_type']={'livox':1,'velodyne':2,'marsim':5}[sys.argv[4]]
config['mapping']['bias_freeze_en']=False
profile=json.loads(Path(sys.argv[3]).read_text())
config['preprocess']['scan_line']=profile['lidar']['vertical_samples']
config['preprocess']['scan_rate']=profile['lidar']['rate_hz']
config['preprocess']['timestamp_unit']=0
Path(sys.argv[2]).write_text(yaml.safe_dump(config,sort_keys=False))
PY
    capture+=(--launch-arg "annotate_lidar:=true")
    capture+=(--fastglio-launch \
      "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_live_loose.launch" \
      --fastglio-config "$fastglio_config" \
      --fastglio-point-filter-num "$point_filter_num" \
      --fastglio-cube-side-length "$cube_side_length" \
      --fastglio-rviz-config \
      "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_live_loose.rviz")
    if [[ -n "$local_map_demo" ]]; then
      capture+=(--fastglio-rviz-config "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/local_map_video.rviz")
    fi
    if $wait_for_recording; then capture+=(--wait-for-recording); fi
    if ! $headless; then
      capture+=(--tile-windows-script \
        "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/tile_live_windows.py")
    fi
    if $teleop; then
      capture+=(--interactive --mapping-only \
        --map-script "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/lio_map_recorder.py")
      capture+=(--fastglio-rviz-config \
        "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_teleop_mapping.rviz")
    fi
  fi
  if $teleop; then
    controller_script="$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_teleop.py"
    if $autonomy; then controller_script="$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_autonomy.py"; fi
    capture+=(--controller-script "$controller_script")
  elif ! $manual; then
    export MSIM_PX4_FLIGHT_PATTERN="$flight_pattern"
    export MSIM_ROUTE_CLOCK="$route_clock"
    capture+=(--controller-script \
      "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/px4_patrol.py")
  fi
  if $gui; then capture+=(--gui); fi
  echo "PX4 SITL 城市无人机采集：$run"
  if $web_console; then echo "网页控制台：http://127.0.0.1:$web_port/"; fi
  if $teleop; then
    if $autonomy; then
      if [[ -n "$goal_enu" ]]; then
        echo "目标任务：相对起飞点 E/N/U=$goal_enu m，在线探索规划，到达后返航；自动起飞。"
      else
        echo '自主模式：自动起飞、出航并沿记录路线返航；使用独立 LIO 定位，GPS 不进入飞控定位。'
      fi
    elif $headless; then
      echo '后台手动采集：通过 /msim/teleop/command 控制；Ctrl+C 结束并保存。'
    else
      echo '手动采集：请在控制窗口起飞和操纵；点击“结束并保存”或 Ctrl+C 结束。'
    fi
    trap 'echo "正在结束采集并保存数据…"' INT
  fi
  python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/ros1_bag.py" capture "${capture[@]}"
  if $teleop && [[ ! -s "$run/sensors.bag" ]]; then
    echo '已结束启动，尚未生成传感器录包。'
    exit 0
  fi
  if $teleop; then trap - INT; duration=0; fi
  python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/ros1_bag.py" trajectory \
    --run "$run" --duration-s "$duration" --antenna-offset 0.05 0 0.04
  if $teleop; then
    duration=$(/usr/bin/python3 - "$run/capture-report.json" <<'PY'
import json,sys
print(json.load(open(sys.argv[1]))['duration_s'])
PY
)
  fi
  if ! $manual; then
    verify=(--run "$run")
    if $online_gnss; then verify+=(--online-gnss --require-lidar-sync); fi
    if ! python "$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav/verify_px4_flight.py" "${verify[@]}"; then
      verification_ok=false
      echo "飞行或同步检查未通过；继续导出定位图表，请检查 px4-flight-report.json。" >&2
    fi
  fi
  if $fastglio_live; then
    export MPLCONFIGDIR="$run/matplotlib-cache"
    mkdir -p "$MPLCONFIGDIR"
    if $teleop; then
      python "$MSIM_ROOT/tools/plot-teleop-trajectory.py" --run "$run"
      if [[ -n "$goal_enu" ]]; then python "$MSIM_ROOT/tools/plot-goal-mission.py" --run "$run"; fi
    else
      python "$MSIM_ROOT/tools/plot-fastglio-live.py" --run "$run"
      python "$MSIM_ROOT/tools/plot-fastglio-enu-error.py" --run "$run"
    fi
    python "$MSIM_ROOT/tools/evaluate-fastglio-replay.py" \
      --bag "$run/sensors.bag" \
      --output "$run/fastglio-live-lio-error.json"
  fi
  if $autonomy; then
    if ! python "$MSIM_ROOT/tools/evaluate-autonomy.py" --run "$run"; then
      verification_ok=false
      echo '自主返航核验未通过，请检查 autonomy-verification.json 和 navigation-report.json。' >&2
    fi
    if [[ -n "$goal_enu" && "$scene" == ksql_airport && -f "$run/navigation-report.json" ]]; then
      if ! python "$MSIM_ROOT/tools/evaluate-mesh-clearance.py" --run "$run"; then
        verification_ok=false
        echo '轨迹与场景网格的离线间距核验未通过，请检查 mesh-clearance-verification.json。' >&2
      fi
    fi
  fi
  if ! $verification_ok; then exit 1; fi
fi
if $skip_signalsim; then exit 0; fi
"$MSIM_ROOT/tools/fastglio-uav-capture.sh" --postprocess-only \
  --run "$run" --duration-s "$duration"
