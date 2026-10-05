# config/paths.sh -- resolve where everything lives. Source it, never execute it.
#
#   source "$VLA_ROOT/config/paths.sh"
#
# Every path in this project derives from VLA_ROOT. If VLA_ROOT is already set
# (by config/.env, by the Makefile, or by a container) it is respected;
# otherwise it is resolved from the location of THIS file, which is one level
# below the repository root. That resolution works regardless of the caller's
# working directory, which is what the old hard-coded absolute host paths
# were really standing in for.
#
# Nothing here has side effects beyond creating the log directory: no ROS
# sourcing, no daemon restart, no banner. Safe in a bash -c wrapper.

if [ -z "${VLA_ROOT:-}" ]; then
    _vla_this="${BASH_SOURCE[0]:-$0}"
    VLA_ROOT="$(cd "$(dirname "$_vla_this")/.." && pwd)"
    unset _vla_this
fi
export VLA_ROOT

# ---- derived locations -------------------------------------------------------
export VLA_CONFIG_DIR="${VLA_CONFIG_DIR:-$VLA_ROOT/config}"
export VLA_SRC_DIR="${VLA_SRC_DIR:-$VLA_ROOT/src}"
export VLA_TOOLS_DIR="${VLA_TOOLS_DIR:-$VLA_ROOT/tools}"
export VLA_LAUNCH_DIR="${VLA_LAUNCH_DIR:-$VLA_ROOT/launch}"
export VLA_MAP_DIR="${VLA_MAP_DIR:-$VLA_ROOT/maps}"

# ---- the two overridable output locations ------------------------------------
# Defaults are inside the repo and are git-ignored. Point them elsewhere to keep
# logs and multi-gigabyte weights out of the working tree.
export VLA_LOG_DIR="${VLA_LOG_DIR:-$VLA_ROOT/logs}"
export VLA_MODEL_DIR="${VLA_MODEL_DIR:-$VLA_ROOT/models}"

# ---- user overrides ----------------------------------------------------------
# config/.env is git-ignored and optional; see config/.env.example.
if [ -f "$VLA_CONFIG_DIR/.env" ]; then
    set -a
    . "$VLA_CONFIG_DIR/.env"
    set +a
fi

mkdir -p "$VLA_LOG_DIR" 2>/dev/null || true

# ---- helper: source ROS, loudly ----------------------------------------------
# Sourcing ROS silently was a real bug: when /opt/ros/humble/setup.bash is
# missing there is no `ros2` on PATH, every later command fails, and a success
# banner still prints. Warn instead.
vla_ros_setup() {
    local setup="${VLA_ROS_SETUP:-/opt/ros/humble/setup.bash}"
    if [ -f "$setup" ]; then
        # shellcheck disable=SC1090
        . "$setup"
    else
        echo "WARNING: $setup not found -- ros2 will NOT work" >&2
        echo "         (are you on the host instead of inside the ROS container?)" >&2
        return 1
    fi
}

# ---- helper: clean dead DDS shared memory ------------------------------------
# `fastdds shm clean` removes ONLY segments whose owner process is dead. The
# `rm -rf /dev/shm/fastrtps_*` this replaced also deleted the segments of
# RUNNING nodes, which cut them off from everything started afterwards on the
# same PC: RViz map 0x0, "Frame [map] does not exist", map_saver "Failed to spin
# map subscription", save_map hanging (CHANGELOG_2026-09-08.md).
vla_shm_clean() {
    if command -v fastdds >/dev/null 2>&1; then
        fastdds shm clean
    else
        echo "  fastdds not on PATH -- stale shm not cleaned"
    fi
}
