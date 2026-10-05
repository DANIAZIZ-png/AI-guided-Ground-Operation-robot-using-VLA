#!/usr/bin/env bash
# Entrypoint for the ROS image.
#
# Sources ROS and the vla_bringup workspace, then applies the simulation or
# hardware environment according to VLA_MODE, then runs whatever was asked for.
#
#   VLA_MODE=sim    -> config/sim.env
#   VLA_MODE=robot  -> config/robot.env
#   unset           -> ROS only, neither profile
#
# WHY A MODE IS REQUIRED RATHER THAN DEFAULTED
#   Simulation and hardware need OPPOSITE discovery settings and each fails
#   SILENTLY under the other's -- no error, just an empty topic list, or a
#   Gazebo world that loads with no robot in it and a log repeating "Waiting
#   messages on topic [robot_description]" forever. Guessing would reintroduce
#   the single biggest time-waster in this project, so an unset VLA_MODE applies
#   neither and says so.

set -euo pipefail

# ROS's own setup scripts reference unbound variables, so -u has to come off
# around them.
set +u
# shellcheck disable=SC1091
. /opt/ros/humble/setup.bash
if [ -f /opt/ros2_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    . /opt/ros2_ws/install/setup.bash
fi
set -u

VLA_ROOT="${VLA_ROOT:-/opt/vla}"
export VLA_ROOT

case "${VLA_MODE:-}" in
    sim)
        set +u
        # shellcheck disable=SC1091
        . "$VLA_ROOT/config/sim.env"
        set -u
        echo "[entrypoint] simulation mode: discovery server unset, VLA_NS empty" >&2
        ;;
    robot)
        set +u
        # shellcheck disable=SC1091
        . "$VLA_ROOT/config/robot.env"
        set -u
        echo "[entrypoint] hardware mode: discovery server ${ROS_DISCOVERY_SERVER:-unset}," \
             "profile ${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}" >&2
        ;;
    "")
        echo "[entrypoint] VLA_MODE is not set: ROS is sourced but neither the" >&2
        echo "             simulation nor the hardware profile is applied. Set" >&2
        echo "             VLA_MODE=sim or VLA_MODE=robot -- the two use opposite" >&2
        echo "             discovery settings and each fails silently under the" >&2
        echo "             other's." >&2
        ;;
    *)
        echo "[entrypoint] VLA_MODE='${VLA_MODE}' is not valid (expected sim or robot)" >&2
        exit 2
        ;;
esac

mkdir -p "${VLA_LOG_DIR:-$VLA_ROOT/logs}"

# VLA_HEADLESS=1 wraps the command in a virtual X display. Needed because the
# TurtleBot 4 ignition launch builds its own ign_args and gives no way to ask
# Gazebo for headless mode, so it has to come from outside the launch system.
if [ "${VLA_HEADLESS:-0}" = "1" ]; then
    if ! command -v xvfb-run >/dev/null 2>&1; then
        echo "[entrypoint] VLA_HEADLESS=1 but xvfb-run is missing" >&2
        exit 2
    fi
    echo "[entrypoint] headless: running under xvfb-run" >&2
    exec xvfb-run -a --server-args="-screen 0 1280x1024x24" "$@"
fi

exec "$@"
