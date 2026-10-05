# sim_mode.sh -- put THIS terminal into SIMULATION mode.
#
#   RUN IT AS:   source scripts/sim_mode.sh        (or with a leading dot)
#   NOT AS:      ./scripts/sim_mode.sh             (child shell; sets nothing)
#
# Thin wrapper kept for compatibility: the variables now live in config/sim.env
# and the side effects in scripts/vla_mode.sh.
#
# WHY THIS FILE EXISTS: a terminal left in robot mode will not see the
# simulation, and it fails SILENTLY -- Gazebo loads the world, the robot never
# spawns, and the log just repeats "Waiting messages on topic
# [robot_description]" forever. That cost an hour to find once.

# shellcheck source=./vla_mode.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/vla_mode.sh" sim
