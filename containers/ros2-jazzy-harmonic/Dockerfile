FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
# Candidate package version is fixed; transitive apt dependencies are not yet snapshotted.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim=1.0.24-1noble.20261006.035056 \
    && rm -rf /var/lib/apt/lists/*
COPY containers/ros2.packages.lock /tmp/msim-packages.lock
RUN dpkg-query -W '-f=${binary:Package} ${Version}\n' > /tmp/msim-packages.actual \
    && diff -u /tmp/msim-packages.lock /tmp/msim-packages.actual \
    && rm /tmp/msim-packages.lock /tmp/msim-packages.actual
