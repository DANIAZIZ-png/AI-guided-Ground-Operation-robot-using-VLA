# robot_env.sh -- HARDWARE environment for NON-INTERACTIVE shells. Source it.
#
#   bash -c "source scripts/robot_env.sh; <command>"
#
# Thin wrapper kept for compatibility: this is now just config/robot.env.
#
# Same variables as robot_mode.sh, WITHOUT its side effects: no shared-memory
# clean, no ROS daemon restart, no banner. Use robot_mode.sh in a terminal you
# type into; use this file inside scripts, `xterm -e` launchers and `bash -c`
# wrappers, where restarting the daemon on every call would leave every other
# terminal blind for ~10 s each time (CHANGELOG_2026-09-08.md §2).
#
# Hardware only. The simulation never sources this file.

# shellcheck source=../config/robot.env
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/../config/robot.env"
