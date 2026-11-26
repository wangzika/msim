#!/usr/bin/env bash
# Real Gazebo/PX4 flight with simultaneous RViz and display-only truth overlay.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MODE=${1:-}
if [[ "$MODE" != before && "$MODE" != after ]]; then
  echo "Usage: bash $0 before|after [--auto-start] [px4-uav-capture options]" >&2; exit 2
fi
shift
AUTO=false
EXTRA=()
while (($#)); do
  case "$1" in
    --auto-start) AUTO=true; shift;;
    --web|--manual|--teleop|--autonomy|--goal-enu|--local-map-demo)
      echo '此入口用于 Gazebo/RViz 双窗口的自动航线录制；这些模式请使用常规采集入口。' >&2; exit 2;;
    *) EXTRA+=("$1"); shift;;
  esac
done
WAIT=()
if ! $AUTO; then WAIT+=(--wait-for-recording); fi
echo "局部地图 $MODE 现场复现：Gazebo + PX4 + FAST-LIO + RViz，真值仅用于显示。"
echo 'Gazebo：白=真值，青=三维 LIO，蓝=GPS，绿=二维检测器。'
echo 'RViz：绿=对齐真值轨迹，青=LIO 轨迹，红=当前帧，紫=显示历史。'
exec bash "$ROOT/tools/px4-uav-capture.sh" \
  --fastglio-live --scene simple --duration-s 60 --flight-pattern route \
  --route-clock sim --skip-signalsim --lidar-model vlp16 --point-filter-num 1 \
  --local-map-demo "$MODE" "${WAIT[@]}" "${EXTRA[@]}"
