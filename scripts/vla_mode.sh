# scripts/vla_mode.sh -- put THIS terminal into robot or simulation mode.
#
#   source scripts/vla_mode.sh robot
#   source scripts/vla_mode.sh sim
#
# MUST BE SOURCED, never executed. Executed, it runs in a child shell, prints
# the banner, and sets NOTHING in the shell you are in. The banner prints either
# way -- it is NOT evidence. After sourcing, verify:
#
#   env | grep -E "^(ROS_DISCOVERY|FASTRTPS|VLA_NS|ROS_DOMAIN)"
#
# No output means the shell is not configured, whatever the banner said.
#
# Replaces robot_mode.sh and sim_mode.sh, which are now thin wrappers around it.
# The variables live in config/robot.env and config/sim.env; this file adds the
# three side effects that only make sense in a terminal you type into:
# shared-memory clean, ROS daemon restart, and the banner.
#
# Do NOT source this inside a `bash -c` wrapper or an `xterm -e` launcher: the
# daemon restart leaves every other terminal blind for ~10 s. Source
# config/robot.env or config/sim.env directly there instead.

_vla_mode="${1:-}"
_vla_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

case "$_vla_mode" in
    robot|sim) : ;;
    *)
        echo "usage: source ${BASH_SOURCE[0]:-$0} robot|sim" >&2
        unset _vla_mode _vla_dir
        return 2 2>/dev/null || exit 2
        ;;
esac

# shellcheck source=../config/robot.env
. "$_vla_dir/../config/$_vla_mode.env"

# ---- side effects, terminal only --------------------------------------------
vla_shm_clean

ros2 daemon stop
sleep 2
ros2 daemon start
# The daemon caches the node/topic graph PER DOMAIN and lies after a switch.
# NOT silenced: these used to be `>/dev/null 2>&1`, which hid
# "ros2: command not found" completely, so a daemon restart that did nothing
# looked identical to one that worked.

# ---- banner ------------------------------------------------------------------
echo "──────────────────────────────────────────"
if [ "$_vla_mode" = robot ]; then
    echo " ROBOT MODE   domain=$ROS_DOMAIN_ID   namespace=$VLA_NS"
    echo "   discovery : server $ROS_DISCOVERY_SERVER"
    echo "   transports: compressed RGB + compressed depth"
    echo "   profile   : $FASTRTPS_DEFAULT_PROFILES_FILE"
    echo "──────────────────────────────────────────"
    echo "Startup order (never skip, never reorder):"
    echo "  1. Robot: sudo systemctl stop turtlebot4 ; sleep 10 ;"
    echo "            sudo systemctl restart discovery ; sleep 15 ;"
    echo "            sudo systemctl start turtlebot4"
    echo "  2. CHECK THE CLOCK -- a 118 s drift makes SLAM drop every scan:"
    echo "       ssh $VLA_ROBOT_USER@$VLA_ROBOT_IP 'chronyc tracking | grep \"System time\"'"
    echo "       if it is more than ~1 s out:  sudo chronyc makestep"
    echo "  3. verify: base_link in \$VLA_NS/tf, dock_status publisher = 1"
    echo "  4. camera: ros2 service call \$VLA_NS/oakd/start_camera std_srvs/srv/Trigger"
    echo "  5. SLAM, then Nav2, then the agent"
else
    echo " SIM MODE   domain=(default, unset)   namespace=(empty)"
    echo "   discovery : multicast (no server)"
    echo "   transports: raw RGB + raw depth"
    echo "──────────────────────────────────────────"
    echo "Launch sim  :  make sim"
    echo "Launch agent:  scripts/run_agent_sim.sh"
fi
echo "──────────────────────────────────────────"
echo "VLA_ROOT=$VLA_ROOT"
echo "VLA_LOG_DIR=$VLA_LOG_DIR"

unset _vla_mode _vla_dir
