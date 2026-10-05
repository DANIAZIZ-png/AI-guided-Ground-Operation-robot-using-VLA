#!/bin/bash
# Resolve the repository root from this script's own location, so the script
# works from any working directory and from a clone anywhere on disk.
VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
source "$VLA_ROOT/config/paths.sh"
VLA_YOLO_ENV="${VLA_YOLO_ENV:-$HOME/yolo-env}"
VLA_GUI_CONFIG="${VLA_GUI_CONFIG:-$HOME/.vla_gui.json}"
# ─────────────────────────────────────────────────────────────────
#  run_agent_sim.sh  (v2) — start the VLA agent against the SIMULATION
#
#  RUN:  scripts/run_agent_sim.sh          (in ubuntu22-gpu)
#
#  WHAT CHANGED FROM v1
#    v1 set ROS_DOMAIN_ID=42 to isolate sim from hardware. That BROKE the
#    simulation: gz_ros2_control runs INSIDE the Gazebo process and does not
#    inherit the domain, so controller_manager came up on domain 0 while the
#    spawner looked for it on 42. Symptom: "Could not contact service
#    /controller_manager/list_controllers" forever, and a robot that spawns
#    but cannot move. Domain isolation is REMOVED.
#
#    Isolation now comes from the DISCOVERY VARS being unset (below), plus
#    not running sim and hardware at the same time. That is what mattered.
# ─────────────────────────────────────────────────────────────────

AGENT="$VLA_SRC_DIR/vla_agent_v28.py"
# change this ONE line when a new agent version is delivered

unset ROS_DOMAIN_ID
# default domain 0 — the same one Gazebo's internal nodes use

unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_DISCOVERY_SERVER
# .bashrc points these at the Pi's discovery server. With the robot off,
# nodes register with a server that isn't there and discover NOTHING —
# no error, just silence. This is the line that actually matters.

export VLA_NS=""
# EMPTY, not unset. The agent reads os.environ.get("VLA_NS", "/robot1"),
# so unsetting hands back the /robot1 default — the opposite of what we want.

export VLA_RAW_RGB=1
export VLA_RAW_DEPTH=1
# Gazebo publishes plain sensor_msgs/Image. The compressed topics (#50/#51)
# do not exist here, so the agent would subscribe to nothing and report
# "I see: nothing" while the camera streamed fine.

if [ ! -f "$AGENT" ]; then
    echo "STOP: $AGENT not found."
    exit 1
fi

echo "Starting $(basename $AGENT) against the simulation..."

python3 "$AGENT" --ros-args \
  -r /oakd/stereo/image_raw:=/oakd/rgb/preview/depth \
  -r /oakd/stereo/camera_info:=/oakd/rgb/preview/camera_info
# -r = remap. The agent still THINKS it uses the stereo topics; ROS rewires
# them to Gazebo's names underneath. One agent file for sim AND hardware.
