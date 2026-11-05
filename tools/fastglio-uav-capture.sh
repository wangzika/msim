#!/usr/bin/env bash
set -euo pipefail

MSIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_GLIO_ROOT="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
ROS1_DIR="$MSIM_ROOT/adapters/ros1_noetic/fast_glio_uav"
duration=300
flight_mode=patrol_s
gui=false
postprocess=false
run=""
scenario=""
retain_nlos=false
while (($#)); do
  case "$1" in
    --duration-s) duration="$2"; shift 2;;
    --flight-mode) flight_mode="$2"; shift 2;;
    --gui) gui=true; shift;;
    --run) run="$2"; shift 2;;
    --postprocess-only) postprocess=true; shift;;
    --scenario) scenario="$2"; shift 2;;
    --retain-nlos) retain_nlos=true; shift;;
    *) echo "Unknown argument: $1" >&2; exit 2;;
  esac
done
if [[ -z "$run" ]]; then
  run="$MSIM_ROOT/runs/uav-fastglio-$(date +%Y%m%d-%H%M%S)"
fi
run="$(realpath -m "$run")"
if [[ ! -f "$FAST_GLIO_ROOT/devel/setup.bash" ]]; then
  echo "FAST_GLIO ROS1 workspace not built: $FAST_GLIO_ROOT/devel/setup.bash" >&2
  exit 2
fi
export PATH="$FAST_GLIO_ROOT/.pixi/envs/default/bin:$PATH"
set +u
source "$FAST_GLIO_ROOT/devel/setup.bash"
set -u
export ROS_LOG_DIR="$run/roslogs"
if ! $postprocess; then
  capture=(--run "$run" --duration-s "$duration" --flight-mode "$flight_mode")
  if $gui; then capture+=(--gui); fi
  echo "FAST_GLIO 无人机采集：$run"
  python "$ROS1_DIR/ros1_bag.py" capture "${capture[@]}"
fi
if [[ ! -f "$run/sensors.bag" ]]; then
  echo "Missing $run/sensors.bag" >&2
  exit 2
fi
if [[ ! -f "$run/trajectory.json" || ! -f "$run/capture-report.json" ]]; then
  python "$ROS1_DIR/ros1_bag.py" trajectory --run "$run" --duration-s "$duration"
fi
prepare=(--run "$run" --duration-s "$duration")
if [[ -n "$scenario" ]]; then prepare+=(--scenario "$scenario"); fi
scenario=$(/usr/bin/python3 "$ROS1_DIR/prepare_run.py" "${prepare[@]}")
if [[ ! -x "$MSIM_ROOT/build/signalsim/signalsim_observables" ]]; then
  "$MSIM_ROOT/tools/build-signalsim-adapter.sh"
fi
signal_dir="$run/signalsim"
if [[ ! -f "$signal_dir/manifest.json" ]]; then
  generation=(--run "$run" --scenario "$scenario" --output-dir "$signal_dir")
  if $retain_nlos; then generation+=(--retain-nlos); fi
  /usr/bin/python3 "$MSIM_ROOT/apps/fastglio/run_signalsim.py" "${generation[@]}"
else
  /usr/bin/python3 - "$signal_dir/manifest.json" "$scenario" "$retain_nlos" <<'PY'
import hashlib, json, sys
from pathlib import Path
manifest = json.loads(Path(sys.argv[1]).read_text())
digest = hashlib.sha256(Path(sys.argv[2]).read_bytes()).hexdigest()
if manifest.get('scenario_sha256') != digest or manifest.get('retain_nlos') != (sys.argv[3] == 'true'):
    raise SystemExit('existing SignalSim output uses a different scenario or NLOS policy; use a new run directory')
PY
fi
epochs="$signal_dir/epochs.jsonl"
bag="$run/combined-signalsim.bag"
report="$run/merge-signalsim-report.json"
verification="$run/verification-signalsim.json"
if [[ ! -f "$bag" ]]; then
  python "$ROS1_DIR/ros1_bag.py" merge --run "$run" --duration-s "$duration" \
    --scenario "$scenario" --epochs "$epochs" --output-bag "$bag" \
    --report "$report"
fi
python "$ROS1_DIR/ros1_bag.py" verify --run "$run" --duration-s "$duration" \
  --epochs "$epochs" --bag "$bag" --report "$verification"
echo "完成：$bag"
echo "RTKLIB 输入：$signal_dir/rover.obs $signal_dir/base.obs $signal_dir/navigation.nav"
