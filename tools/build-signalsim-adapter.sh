#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="${SIGNALSIM_SOURCE:-$ROOT/build/deps/SignalSim}"
EXPECTED="baba9deb68ed193efb9fd9a362b431ae8c491fa4"
if [[ ! -d "$SOURCE/.git" ]]; then
  echo "SignalSim source missing: $SOURCE" >&2
  echo "git clone https://github.com/globsky/SignalSim.git '$SOURCE'" >&2
  exit 2
fi
ACTUAL="$(git -C "$SOURCE" rev-parse HEAD)"
if [[ "$ACTUAL" != "$EXPECTED" ]]; then
  echo "SignalSim revision differs from pinned $EXPECTED: $ACTUAL" >&2
  exit 2
fi
mkdir -p "$ROOT/build/signalsim"
g++ -std=c++17 -O2 -w -include cstring \
  -I "$SOURCE/inc" -I "$ROOT/core/include" \
  "$ROOT/adapters/signalsim/observables.cpp" "$SOURCE"/src/*.cpp \
  "$ROOT/core/src/frames/geodesy.cpp" "$ROOT/core/src/gnss/urban.cpp" \
  -o "$ROOT/build/signalsim/signalsim_observables"
printf '%s\n' "$ROOT/build/signalsim/signalsim_observables"
