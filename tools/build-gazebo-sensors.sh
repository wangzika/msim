#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fast="${FAST_GLIO_ROOT:-/home/wzb/FAST_GLIO}"
px4="${PX4_ROOT:-$root/build/deps/PX4-Autopilot}"
export PATH="$fast/.pixi/envs/default/bin:$PATH"
export CONDA_PREFIX="$fast/.pixi/envs/default"
set +u
source "$fast/devel/setup.bash"
set -u
cmake -S "$root/adapters/gazebo_classic" -B "$root/build/gazebo-sensors" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$fast/devel;$CONDA_PREFIX" \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_CXX_COMPILER="$CONDA_PREFIX/bin/c++" \
  -DPX4_GAZEBO_BUILD="$px4/build/px4_sitl_default/build_gazebo-classic"
cmake --build "$root/build/gazebo-sensors" -j2
