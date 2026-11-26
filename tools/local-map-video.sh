#!/usr/bin/env bash
# Isolated, paused LiDAR/IMU replay for filming before/after map parameters.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODE=${1:-}
if [[ "$MODE" != before && "$MODE" != after ]]; then
  echo "Usage: bash $0 before|after [--source-run DIR] [--rate 0.5] [--port 11346] [--output DIR] [--binary FILE] [--headless] [--auto-play] [--duration-s N]" >&2
  exit 2
fi
shift
SOURCE="$ROOT/runs/lidar-comparison-20261122-01/vlp16"
FAST=${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}
BINARY="$FAST/devel/lib/fast_lio/fastlio_mapping"
PORT=11346 RATE=0.5 OUT='' DURATION='' HEADLESS=false AUTO=false
while (($#)); do
  case "$1" in
    --source-run) SOURCE=$2; shift 2;;
    --rate) RATE=$2; shift 2;;
    --port) PORT=$2; shift 2;;
    --output) OUT=$2; shift 2;;
    --binary) BINARY=$2; shift 2;;
    --duration-s) DURATION=$2; shift 2;;
    --headless) HEADLESS=true; shift;;
    --auto-play) AUTO=true; shift;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
SOURCE=$(realpath "$SOURCE")
[[ -f "$SOURCE/sensors.bag" && -f "$SOURCE/parameters.yaml" ]] || { echo 'Source bag or archived parameters missing; mount the recording disk.' >&2; exit 2; }
[[ -x "$BINARY" ]] || { echo "FAST-LIO binary not found: $BINARY" >&2; exit 2; }
[[ -n "$OUT" ]] || OUT="$ROOT/build/local-map-video/$MODE-$(date +%Y%m%d-%H%M%S)"
[[ ! -e "$OUT" ]] || { echo "Refusing to overwrite: $OUT" >&2; exit 2; }
export PATH="$FAST/.pixi/envs/default/bin:$PATH" CONDA_PREFIX="$FAST/.pixi/envs/default"
set +u; source "$FAST/devel/setup.bash"; set -u
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_IP=127.0.0.1
python - "$PORT" "$RATE" "$DURATION" <<'PY'
import math,socket,sys
port=int(sys.argv[1]);rate=float(sys.argv[2]);duration=float(sys.argv[3]) if sys.argv[3] else None
if not 1024<=port<=65535 or not math.isfinite(rate) or not .1<=rate<=2 or duration is not None and (not math.isfinite(duration) or duration<=0):
    raise SystemExit('Invalid port, rate or duration')
with socket.socket() as s:
    try:s.bind(('127.0.0.1',port))
    except OSError:raise SystemExit('ROS port occupied; choose another --port. No existing ROS processes were stopped.')
PY
mkdir -p "$OUT/ros-home" "$OUT/roslogs"
OUT=$(realpath "$OUT")
export ROS_HOME="$OUT/ros-home" ROS_LOG_DIR="$OUT/roslogs"
# Optional diagnostic binaries must not inherit previous point-sampling trials.
export MSIM_SAMPLE_OFFSET=0 MSIM_SAMPLE_MODE=stride MSIM_SAMPLE_SEED=17 MSIM_PLANE_MIN_SPREAD=0
unset MSIM_LIO_DIAGNOSTICS || true
MASTER='' MAPPING='' RVIZ=''
cleanup() {
  for pid in "$RVIZ" "$MAPPING" "$MASTER"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      for _ in {1..30}; do kill -0 "$pid" 2>/dev/null || break; sleep .1; done
      if kill -0 "$pid" 2>/dev/null; then kill -TERM "$pid" 2>/dev/null || true; fi
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
roscore -p "$PORT" > "$OUT/roscore.log" 2>&1 & MASTER=$!
for _ in {1..100}; do rosparam list >/dev/null 2>&1 && break; sleep .1; done
rosparam list >/dev/null
rosparam load "$SOURCE/parameters.yaml"
rosparam set /use_sim_time true
# Deliberately reproduce the old effective value without changing good launch files.
CUBE=200; [[ "$MODE" == after ]] && CUBE=1000
rosparam set /cube_side_length "$CUBE"
rosparam set /laserMapping/cube_side_length 1000
rosparam set /mapping/det_range 100.0
rosparam set /point_filter_num 1
rosparam set /laserMapping/point_filter_num 1
rosparam set /mapping/bias_freeze_en false
rosparam set /publish/path_en true
rosparam set /publish/scan_publish_en true
rosparam set /publish/scan_bodyframe_pub_en false
rosparam set /pcd_save/pcd_save_en false
for type in loose tight raw_tight; do rosparam set "/laserMapping/gnss_$type/enable" false; done
rosparam dump "$OUT/parameters.yaml"
cp "$ROOT/adapters/ros1_noetic/fast_glio_uav/local_map_video.rviz" "$OUT/view.rviz"
python - "$SOURCE" "$OUT" "$MODE" "$RATE" "$BINARY" <<'PY'
import hashlib,json,sys,yaml
from pathlib import Path
source,out,mode,rate,binary=sys.argv[1:];out=Path(out)
p=yaml.safe_load((out/'parameters.yaml').read_text())
if p.get('preprocess',{}).get('lidar_type')!=2:raise SystemExit('This viewer currently requires an archived Velodyne-format run')
topic=p['common']['lid_topic'];imu=p['common']['imu_topic']
(out/'topics.txt').write_text(topic+'\n'+imu+'\n')
(out/'experiment.json').write_text(json.dumps(dict(source_run=source,mode=mode,playback_rate=float(rate),
    effective_cube_side_length_m=p['cube_side_length'],det_range_m=p['mapping']['det_range'],point_filter_num=1,
    binary=binary,binary_sha256=hashlib.sha256(Path(binary).read_bytes()).hexdigest(),
    gnss=False,method='recompute archived sensors; before uses explicit global 200 to reproduce the historical missing-global fallback; not original video',
    display='current scan and RViz display history; not the internal ikd-Tree map'),indent=2)+'\n')
PY
mapfile -t TOPICS < "$OUT/topics.txt"
"$BINARY" __name:=laserMapping > "$OUT/algorithm.log" 2>&1 & MAPPING=$!
READY=false
for _ in {1..100}; do
  if rosnode list 2>/dev/null | /bin/grep -Fx /laserMapping >/dev/null; then READY=true; break; fi
  kill -0 "$MAPPING" 2>/dev/null || { echo "Algorithm exited; see $OUT/algorithm.log" >&2; exit 1; }
  sleep .1
done
$READY || { echo 'Algorithm did not register in time' >&2; exit 1; }
if ! $HEADLESS; then rviz -d "$OUT/view.rviz" __name:=msim_local_map_video > "$OUT/rviz.log" 2>&1 & RVIZ=$!; fi
printf 'Mode: %s | cube: %s m | det_range: 100 m | rate: %s | logs: %s\n' "$MODE" "$CUBE" "$RATE" "$OUT"
echo '纯 LiDAR/IMU 重新解算；红色为当前帧，紫色为 RViz 累计显示，青色为估计轨迹。'
PLAY=(--clock --hz=100 -r "$RATE")
if $HEADLESS; then PLAY+=(--quiet); fi
if ! $AUTO; then PLAY+=(--pause); echo '先摆好 RViz 并开始录屏，再回此终端按空格播放；空格暂停，s 单条消息步进。'; fi
[[ -z "$DURATION" ]] || PLAY+=(--duration "$DURATION")
rosbag play "${PLAY[@]}" "$SOURCE/sensors.bag" --topics "${TOPICS[@]}"
if ! $HEADLESS; then read -r -p '回放完成，画面保留。录屏结束后按 Enter 关闭。' _; fi
