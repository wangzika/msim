#!/usr/bin/env bash
# Six fresh captures: a simple reference and two complex worlds, each with/without spoofing.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
GROUP=$(realpath -m "${1:-$ROOT/runs/scene-comparison-20261116-01}")
DURATION=${2:-60}
mkdir -p "$GROUP"
mkdir -p "$GROUP/worlds"
for scene in simple complex_urban industrial_yard; do
  if [[ ! -f "$GROUP/worlds/$scene.world" ]]; then
    cp "$ROOT/scenes/$scene.world" "$GROUP/worlds/$scene.world"
  fi
done
if [[ ! -f "$GROUP/scene-manifest.json" ]]; then
  cp "$ROOT/scenes/scene-manifest.json" "$GROUP/scene-manifest.json"
fi
if [[ ! -f "$GROUP/sensor_profile.json" ]]; then
  cp "$ROOT/adapters/ros1_noetic/msim_sensors/config/sensor_profile.json" "$GROUP/sensor_profile.json"
fi
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
for scene in complex_urban industrial_yard simple; do
  for scenario in clean spoofed; do
    run="$GROUP/$scene/$scenario"
    if [[ -f "$run/fastglio-live-report.json" && -f "$run/px4-flight-report.json" ]]; then
      echo "Existing completed capture: $scene/$scenario"
      continue
    fi
    extra=()
    if [[ "$scenario" == spoofed ]]; then
      extra=(--spoof-start-s 20 --spoof-east-rate-mps 0.18)
    fi
    echo "Starting $scene/$scenario"
    if ! bash "$ROOT/tools/px4-uav-capture.sh" --duration-s "$DURATION" \
      --fastglio-live --headless --skip-signalsim --flight-pattern route --route-clock sim \
      --scene "$scene" --world "$GROUP/worlds/$scene.world" \
      --fastglio-frontend velodyne --point-filter-num 4 \
      --sensor-profile "$GROUP/sensor_profile.json" --run "$run" "${extra[@]}" \
      > "$GROUP/$scene-$scenario-console.log" 2>&1; then
      # A pairing failure stays recorded; retain usable captures only when flight and fusion succeeded.
      python3 - "$run" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]);f=json.loads((p/'px4-flight-report.json').read_text())
l=json.loads((p/'fastglio-live-report.json').read_text())
assert f['armed'] and f['offboard'] and f['max_displacement_m']>=5 and f['max_climb_m']>=3
assert f['injection_match_fraction']>=.8 and l['pass_live_loose_gate']
assert f['pass_lidar_sync_gate'] is False
print('Captured flight and fusion; original LiDAR/GPS pairing gate remains failed:',p)
PY
    fi
    echo "Completed $scene/$scenario"
  done
done
echo "All six scene captures completed: $GROUP"
