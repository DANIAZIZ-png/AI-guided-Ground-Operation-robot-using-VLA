# syntax=docker/dockerfile:1
#
# The ROS side of the stack: Nav2, SLAM Toolbox, the TurtleBot 4 packages, the
# Ignition simulation bridge, the agent and the operator console.
#
#   podman build -f docker/ros.Dockerfile -t vla-ros:1.0 .
#   docker build  -f docker/ros.Dockerfile -t vla-ros:1.0 .
#
# Build context is the REPOSITORY ROOT, because it copies env/ and docker/.
#
# WHAT THIS REPLACES
#   env/MANIFEST.md records that the ROS container ran from a local Podman
#   snapshot (localhost/ubuntu22-snapshot:latest, 9.88 GB) that nobody else can
#   pull. That was the biggest reproducibility hole in the project. This image
#   is built from a public base plus an exact, verified package list.
#
# WHY ubuntu:22.04 AND NOT ros:humble
#   The ros:humble images carry ROS packages from whenever the image was built,
#   which is newer than the versions that produced the recorded results. Mixing
#   them with pinned versions means either apt downgrades or a silent mismatch
#   on anything not in the pin list. Starting from plain Ubuntu and adding only
#   the 2026-05-14 snapshot means nothing in the image can be newer than the
#   snapshot.

FROM ubuntu:22.04

# ---------------------------------------------------------------------------
# Base system
# ---------------------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        locales \
        tzdata \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# The ROS apt source, pinned to a dated snapshot
# ---------------------------------------------------------------------------
# All 399 packages in docker/ros-packages.txt match this snapshot EXACTLY --
# verified package by package; see env/ROS_SNAPSHOT.md for the method and the
# command to re-check it.
#
# http, not https: snapshots.ros.org resolves to CloudFront and its certificate
# does not cover the hostname ("no alternative certificate subject name matches
# target host name"). [trusted=yes] is acceptable here precisely because every
# package version is pinned below and verified against the recorded manifest --
# a tampered mirror could not satisfy the pins.
ARG ROS_SNAPSHOT=2026-05-14
ENV ROS_DISTRO=humble

RUN echo "deb [trusted=yes] http://snapshots.ros.org/${ROS_DISTRO}/${ROS_SNAPSHOT}/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2-snapshot.list

# ---------------------------------------------------------------------------
# ROS, at the recorded versions
# ---------------------------------------------------------------------------
# One source of truth: the list is generated from
# env/ros-humble-packages_ubuntu22-gpu.txt. All 399 are pinned, including
# transitive dependencies, because dependencies resolved at image build time are
# exactly what drifts. The build FAILS if any pin cannot be satisfied, which is
# the point -- a silent substitution would be worse than a broken build.
COPY docker/ros-packages.txt /tmp/ros-packages.txt

RUN apt-get update \
    && xargs -a /tmp/ros-packages.txt apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ros-packages.txt

# ---------------------------------------------------------------------------
# Non-ROS packages the project actually uses
# ---------------------------------------------------------------------------
#   xvfb     headless Gazebo and RViz. The TurtleBot 4 ignition launch builds
#            its own ign_args and offers no way to pass -s or
#            --headless-rendering, so headless has to come from outside the
#            launch system: `xvfb-run -a ros2 launch ...`. Installed here so the
#            setup does not depend on one machine having it.
#   xterm    the agent and voice launchers need a real tty. A pipe silently puts
#            the agent back into headless mode and manual override (#60) stops
#            working, so `xterm -e` is not cosmetic.
#   iproute2 the Wi-Fi throughput check (awk over /proc/net/dev needs no tools,
#            but `ip` is used by the hotspot guard)
#
# NOTE: ping is deliberately NOT relied upon. It exists but exits 2 with no
# output for every address inside a container, because containers are not
# granted the raw socket ICMP needs -- identically whether the robot is up or
# unplugged. Reachability is tested with bash's /dev/tcp instead.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-opencv \
        python3-requests \
        python3-pil \
        xvfb \
        xterm \
        x11-utils \
        iproute2 \
        less \
        nano \
    && rm -rf /var/lib/apt/lists/*

# Python packages that are not packaged for 22.04.
RUN python3 -m pip install --no-cache-dir "setuptools<66" "pytest>=7,<9" ruff

# ---------------------------------------------------------------------------
# The project
# ---------------------------------------------------------------------------
WORKDIR /opt/vla
COPY src/ /opt/vla/src/
COPY config/ /opt/vla/config/
COPY launch/ /opt/vla/launch/
COPY maps/ /opt/vla/maps/
COPY tools/ /opt/vla/tools/
COPY scripts/ /opt/vla/scripts/
COPY vla_bringup/ /opt/vla/vla_bringup/
COPY tests/ /opt/vla/tests/
COPY env/ /opt/vla/env/

# VLA_ROOT is set explicitly so config/paths.sh does not have to guess, and so
# the two output directories land on mounted volumes rather than inside the
# image layer.
ENV VLA_ROOT=/opt/vla \
    VLA_LOG_DIR=/opt/vla/logs \
    VLA_MODEL_DIR=/opt/vla/models \
    PYTHONPATH=/opt/vla/src

RUN mkdir -p /opt/vla/logs /opt/vla/models

# Install the five packages in editable mode so the mounted source wins at run
# time while the entry points (vla-agent, vla-brain, ...) are on PATH.
RUN python3 -m pip install --no-cache-dir -e /opt/vla/src

# Build vla_bringup so `ros2 launch vla_bringup robot.launch.py` works.
# The symlink matters: CMakeLists resolves the repository root by realpath'ing
# its own directory and going up, which is how the package finds launch/,
# config/ and maps/ at the repo root.
RUN . /opt/ros/humble/setup.sh \
    && mkdir -p /opt/ros2_ws/src \
    && ln -s /opt/vla/vla_bringup /opt/ros2_ws/src/vla_bringup \
    && cd /opt/ros2_ws \
    && colcon build --packages-select vla_bringup \
    && rm -rf build log

COPY docker/ros-entrypoint.sh /usr/local/bin/ros-entrypoint.sh
RUN chmod +x /usr/local/bin/ros-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ros-entrypoint.sh"]
CMD ["bash"]
