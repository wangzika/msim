#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
root=$PWD
docker build -f containers/core-ubuntu24/Dockerfile -t msim-core:p0 .
docker build -f containers/ros2-jazzy-harmonic/Dockerfile -t msim-ros2:p0 .
docker build -f containers/ros1-noetic-focal/Dockerfile -t msim-ros1:p0 .
docker run --rm --network none -v "$root:/input:ro" msim-core:p0 bash -c '
  cp -a /input /tmp/msim && cd /tmp/msim && rm -rf build &&
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build &&
  ctest --test-dir build --output-on-failure && tools/ci/check-format.sh && tools/ci/check-tidy.sh'
docker run --rm --network none -v "$root/adapters/ros2_jazzy/msim_msgs:/input:ro" msim-ros2:p0 bash -c '
  gz sim --versions && mkdir -p /tmp/ws/src && cp -r /input /tmp/ws/src/msim_msgs && cd /tmp/ws && colcon build'
docker run --rm --network none -v "$root/adapters/ros1_noetic/msim_msgs:/input:ro" msim-ros1:p0 bash -c '
  mkdir -p /tmp/ws/src && cp -r /input /tmp/ws/src/msim_msgs && cd /tmp/ws && catkin_make'
