# robot_mode.sh -- put THIS terminal into REAL-ROBOT mode.
#
#   RUN IT AS:   source scripts/robot_mode.sh      (or with a leading dot)
#   NOT AS:      ./scripts/robot_mode.sh           (child shell; sets nothing)
#
# Thin wrapper kept for compatibility: the variables now live in
# config/robot.env and the side effects in scripts/vla_mode.sh.
#
# BEFORE YOU USE IT the robot must be ON and reachable. `ping` does not work
# inside the ROS container -- it exits 2 with no output for every address,
# identically whether the robot is up or unplugged. Use bash's /dev/tcp:
#
#   timeout 5 bash -c "cat < /dev/null > /dev/tcp/${VLA_ROBOT_IP:-10.42.0.169}/22"
#
# Exit 0 means reachable. The discovery server lives on the robot; if it is off,
# every node here registers with nothing and discovers nothing, silently.

# shellcheck source=./vla_mode.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)/vla_mode.sh" robot
