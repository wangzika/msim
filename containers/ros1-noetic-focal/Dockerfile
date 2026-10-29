FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d
COPY containers/ros1.packages.lock /tmp/msim-packages.lock
RUN dpkg-query -W '-f=${binary:Package} ${Version}\n' > /tmp/msim-packages.actual \
    && diff -u /tmp/msim-packages.lock /tmp/msim-packages.actual \
    && rm /tmp/msim-packages.lock /tmp/msim-packages.actual
