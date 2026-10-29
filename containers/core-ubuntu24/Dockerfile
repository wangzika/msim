FROM ros:jazzy-ros-base-noble@sha256:8eed0a4669bbf43ee331878a4c39bd8ebddea8e3312a58b4fae8edb14526b617 AS certs
FROM ubuntu:24.04@sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852
COPY --from=certs /etc/ssl/certs/ca-certificates.crt /etc/ssl/certs/ca-certificates.crt
RUN apt-get -o APT::Update::Error-Mode=any --snapshot 20261029T040000Z update && DEBIAN_FRONTEND=noninteractive apt-get --snapshot 20261029T040000Z install -y --no-install-recommends build-essential cmake python3 git clang-format clang-tidy ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY containers/core.packages.lock /tmp/msim-packages.lock
RUN dpkg-query -W '-f=${binary:Package} ${Version}\n' > /tmp/msim-packages.actual && diff -u /tmp/msim-packages.lock /tmp/msim-packages.actual && rm /tmp/msim-packages.lock /tmp/msim-packages.actual
