#!/usr/bin/env bash
# Four fresh, sequential captures. Keep every report, including failed gates.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
group="$(realpath -m "${1:-$root/runs/lidar-comparison-$(date +%Y%m%d-%H%M%S)}")"
duration="${2:-60}"
mkdir -p "$group/worlds"
if [[ ! -f "$group/worlds/simple.world" ]]; then
  cp "${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}/simulation/uav_simulator/worlds/urban_canyon.world" "$group/worlds/simple.world"
fi
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
for lidar in vlp16 hdl32 os1-64 mid360; do
  run="$group/$lidar"
  if [[ -f "$run/px4-flight-report.json" && -f "$run/fastglio-live-report.json" ]]; then
    echo "已有结果，保留原报告：$lidar"
    continue
  fi
  if [[ -e "$run" ]]; then
    echo "运行目录已有未完成采集，请使用新的比较目录：$run" >&2; exit 2
  fi
  echo "开始测试 $lidar：${duration}s，无欺骗，全部保留点云"
  # Same input sampling policy, IMU/GPS noise, route clock, and scene. Sensor
  # geometry, point rate and frontend format retain their declared presets.
  result=0
  bash "$root/tools/px4-uav-capture.sh" --duration-s "$duration" \
    --fastglio-live --headless --skip-signalsim --flight-pattern route --route-clock sim \
    --world "$group/worlds/simple.world" --lidar-model "$lidar" \
    --imu-model mems-baseline --gps-model standalone --lidar-timing rolling \
    --point-filter-num 1 --fusion-delay-s 0.65 --run "$run" \
    > "$group/$lidar-console.log" 2>&1 || result=$?
  echo "$result" > "$group/$lidar-exit-code.txt"
  python3 - "$run" "$result" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1])
required=('sensors.bag','px4-flight-report.json','fastglio-live-report.json')
if not all((p/f).is_file() for f in required):
    raise SystemExit('采集未完成，请检查 '+str(p.parent/(p.name+'-console.log')))
f=json.loads((p/'px4-flight-report.json').read_text());l=json.loads((p/'fastglio-live-report.json').read_text())
print(json.dumps(dict(lidar=p.name,exit_code=int(sys.argv[2]),flight_gate=f['pass_flight_gate'],sync_gate=f['pass_lidar_sync_gate'],fusion_gate=l['pass_live_loose_gate']),ensure_ascii=False),flush=True)
PY
done
echo "四种 LiDAR 采集完成：$group"
