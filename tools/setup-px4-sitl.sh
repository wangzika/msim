#!/usr/bin/env bash
set -euo pipefail

MSIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_GLIO_ROOT="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
PX4_ROOT="${PX4_ROOT:-$MSIM_ROOT/build/deps/PX4-Autopilot}"
if [[ ! -d "$PX4_ROOT/.git" ]]; then
  mkdir -p "$(dirname "$PX4_ROOT")"
  git clone --depth 1 --branch v1.16.1 https://github.com/PX4/PX4-Autopilot.git "$PX4_ROOT"
fi
if [[ "$(git -C "$PX4_ROOT" describe --tags --exact-match 2>/dev/null)" != v1.16.1 ]]; then
  echo "Expected PX4 v1.16.1 checkout: $PX4_ROOT" >&2
  exit 2
fi
git -C "$PX4_ROOT" submodule update --init --recursive --depth 1 \
  Tools/simulation/gazebo-classic/sitl_gazebo-classic \
  src/modules/mavlink/mavlink
# Current Pixi Boost removed the legacy io_service alias used by this pinned
# Gazebo Classic bridge. io_context is its supported replacement.
bridge_header="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/include/mavlink_interface.h"
sed -i 's/boost::asio::io_service io_service_/boost::asio::io_context io_service_/' "$bridge_header"
sed -i 's/io_service_\.post(/boost::asio::post(io_service_, /g' \
  "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/mavlink_interface.cpp"
mkdir -p "$MSIM_ROOT/build/px4-bin"
ln -sfn /usr/bin/cmake "$MSIM_ROOT/build/px4-bin/cmake"
export PATH="$MSIM_ROOT/build/px4-bin:$FAST_GLIO_ROOT/.pixi/envs/default/bin:$PATH"
export TinyXML_ROOT_DIR="$FAST_GLIO_ROOT/.pixi/envs/default"
export CMAKE_PREFIX_PATH="$FAST_GLIO_ROOT/.pixi/envs/default${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="$FAST_GLIO_ROOT/.pixi/envs/default/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export PYTHONPATH="$MSIM_ROOT/build/px4-pydeps${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$MSIM_ROOT/build/px4-pydeps"
if ! python -c 'import jinja2, kconfiglib, serial, future, jsonschema; from pymavlink import mavutil'; then
  python -m pip install --target "$MSIM_ROOT/build/px4-pydeps" \
    'jinja2<4' 'kconfiglib<15' 'pyserial<4' 'future<2' \
    'jsonschema<5' 'pymavlink<3'
fi
if ! python -c 'import pyulog'; then
  python -m pip install --no-deps --target "$MSIM_ROOT/build/px4-pydeps" 'pyulog==1.3.0'
fi
cd "$PX4_ROOT"
DONT_RUN=1 make px4_sitl_default gazebo-classic
test -x build/px4_sitl_default/bin/px4
echo "PX4 SITL ready: $PX4_ROOT/build/px4_sitl_default/bin/px4"
