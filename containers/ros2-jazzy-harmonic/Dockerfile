FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617
COPY containers/ros2-jazzy-harmonic/ros-snapshot-key.asc /usr/share/keyrings/msim-ros-snapshot.asc
RUN rm /etc/apt/sources.list.d/ros2.sources && \
    printf 'Types: deb\nURIs: http://snapshots.ros.org/jazzy/2026-10-05/ubuntu\nSuites: noble\nComponents: main\nSigned-By: /usr/share/keyrings/msim-ros-snapshot.asc\n' > /etc/apt/sources.list.d/ros2-snapshot.sources && \
    apt-get -o APT::Update::Error-Mode=any --snapshot 20261029T040000Z update && \
    DEBIAN_FRONTEND=noninteractive apt-get --snapshot 20261029T040000Z install -y --no-install-recommends ros-jazzy-ros-gz-sim=1.0.24-1noble.20261006.035056 && \
    rm -rf /var/lib/apt/lists/*
COPY containers/ros2.packages.lock /tmp/msim-packages.lock
RUN dpkg-query -W '-f=${binary:Package} ${Version}\n' > /tmp/msim-packages.actual && diff -u /tmp/msim-packages.lock /tmp/msim-packages.actual && rm /tmp/msim-packages.lock /tmp/msim-packages.actual
