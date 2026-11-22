#!/usr/bin/env bash
# Replay one algorithm on both source flights; each algorithm uses an isolated ROS master.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
GROUP=$(realpath "$1")
ALGORITHM=$2
case "$ALGORITHM" in
  fast-lio-velodyne) CONFIG="$GROUP/configs/fast-lio.yaml";;
  lio-sam-patched) CONFIG="$GROUP/configs/lio-sam.yaml";;
  glio-front-end) CONFIG="$GROUP/configs/glio.yaml";;
  ligo) CONFIG="$GROUP/configs/ligo.yaml";;
  *) echo "Unknown algorithm: $ALGORITHM" >&2; exit 2;;
esac
export MSIM_LIO_PLAY_RATE=0.5 MSIM_LIO_POINT_FILTER=4
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
for condition in clean spoofed; do
  RUN="$GROUP/$condition"
  OUT="$RUN/comparisons/$ALGORITHM"
  if [[ -f "$OUT/summary.json" && -f "$OUT/algorithm.bag" ]]; then
    echo "Existing completed result: $condition/$ALGORITHM"
    continue
  fi
  echo "Starting $condition/$ALGORITHM"
  bash "$ROOT/tools/compare-fastglio-lio.sh" "$RUN" "$ALGORITHM" "$CONFIG"
  echo "Completed $condition/$ALGORITHM"
done
