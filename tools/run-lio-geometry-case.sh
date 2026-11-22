#!/usr/bin/env bash
# One pure LIO diagnostic replay using an archived per-sensor parameter file.
set -euo pipefail
if [[ $# -lt 5 || $# -gt 9 ]]; then
  echo "Usage: $0 GROUP INPUT_BAG LABEL STRIDE OFFSET [MODE] [COVARIANCE] [SURF_VOXEL] [RATE]" >&2
  exit 2
fi
ROOT=$(cd "$(dirname "$0")/.." && pwd)
GROUP=$(realpath "$1")
BAG=$(realpath "$2")
LABEL=$3
STRIDE=$4
OFFSET=$5
MODE=${6:-stride}
COV=${7:-0.001}
VOXEL=${8:-0.5}
MAP_VOXEL=${MSIM_MAP_VOXEL:-0.5}
RATE=${9:-0.5}
OUT="$GROUP/$LABEL"
BINARY=${MSIM_DIAGNOSTIC_BINARY:-$GROUP/build/fastlio_mapping_diagnostic}
if [[ -e "$OUT" ]]; then echo "Case already exists: $OUT" >&2; exit 2; fi
mkdir -p "$OUT/ros-home" "$OUT/roslogs"
FAST=${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}
export PATH="$FAST/.pixi/envs/default/bin:$PATH"
export CONDA_PREFIX="$FAST/.pixi/envs/default"
set +u
source "$FAST/devel/setup.bash"
set -u
PORT=${MSIM_LIO_PORT:-11335}
export ROS_MASTER_URI="http://127.0.0.1:$PORT" ROS_IP=127.0.0.1
export ROS_HOME="$OUT/ros-home" ROS_LOG_DIR="$OUT/roslogs"
export MSIM_LIO_DIAGNOSTICS="$OUT/matching.csv"
export MSIM_SAMPLE_OFFSET="$OFFSET" MSIM_SAMPLE_MODE="$MODE" MSIM_SAMPLE_SEED=17
master_pid='' mapping_pid='' record_pid=''
cleanup() {
  for pid in "$record_pid" "$mapping_pid" "$master_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      for _ in {1..30}; do
        if ! kill -0 "$pid" 2>/dev/null; then break; fi
        sleep .1
      done
      if kill -0 "$pid" 2>/dev/null; then kill -TERM "$pid" 2>/dev/null || true; fi
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM
python - "$OUT" "$BAG" "$LABEL" "$STRIDE" "$OFFSET" "$MODE" "$COV" "$VOXEL" "$RATE" "$BINARY" <<'PY'
import json,sys,os
import hashlib
from pathlib import Path
out,bag,label,stride,offset,mode,cov,voxel,rate,binary=sys.argv[1:]
(Path(out)/'experiment.json').write_text(json.dumps(dict(input_bag=bag,label=label,
 input_parameters=os.environ['MSIM_CASE_PARAMETERS'],
 input_parameters_sha256=hashlib.sha256(Path(os.environ['MSIM_CASE_PARAMETERS']).read_bytes()).hexdigest(),
 stride=int(stride),offset=int(offset),mode=mode,seed=17,lidar_meas_cov=float(cov),
 surface_voxel_m=float(voxel),map_voxel_m=float(os.environ.get('MSIM_MAP_VOXEL','.5')),playback_rate=float(rate),gnss=False,
 min_plane_spread=float(os.environ.get('MSIM_PLANE_MIN_SPREAD','0')),
 binary=binary,binary_sha256=hashlib.sha256(Path(binary).read_bytes()).hexdigest()),indent=2)+'\n')
PY
roscore -p "$PORT" > "$OUT/roscore.log" 2>&1 & master_pid=$!
for _ in {1..100}; do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep .1
done
rosparam load "${MSIM_CASE_PARAMETERS:?Set archived parameters.yaml}"
rosparam set /use_sim_time true
rosparam set /point_filter_num "$STRIDE"
rosparam set /laserMapping/point_filter_num "$STRIDE"
rosparam set /feature_extract_enable false
rosparam set /mapping/lidar_meas_cov "$COV"
rosparam set /filter_size_surf "$VOXEL"
rosparam set /filter_size_map "$MAP_VOXEL"
rosparam set /laserMapping/filter_size_surf "$VOXEL"
rosparam set /laserMapping/filter_size_map "$MAP_VOXEL"
rosparam set /mapping/bias_freeze_en false
rosparam set /laserMapping/gnss_loose/enable false
rosparam set /laserMapping/gnss_tight/enable false
rosparam set /laserMapping/gnss_raw_tight/enable false
rosparam set /publish/scan_publish_en false
rosparam set /publish/path_en false
rosparam set /pcd_save/pcd_save_en false
rosparam dump "$OUT/parameters.yaml"
"$BINARY" __name:=laserMapping > "$OUT/algorithm.log" 2>&1 & mapping_pid=$!
for _ in {1..100}; do
  if rosnode list 2>/dev/null | /bin/grep -Fx /laserMapping >/dev/null; then break; fi
  if ! kill -0 "$mapping_pid" 2>/dev/null; then echo "Algorithm exited" >&2; exit 1; fi
  sleep .1
done
rosbag record -O "$OUT/algorithm.bag" /Odometry /ground_truth/state > "$OUT/record.log" 2>&1 & record_pid=$!
sleep 1.5
rosbag play -q --clock -r "$RATE" "$BAG" --topics /imu/data /msim/velodyne_points_annotated /ground_truth/state > "$OUT/play.log" 2>&1
sleep 2
kill -INT "$record_pid"
wait "$record_pid" || true
record_pid=''
python "$ROOT/tools/evaluate-fastglio-replay.py" --bag "$OUT/algorithm.bag" --output "$OUT/summary.json"
echo "Completed: $LABEL"
