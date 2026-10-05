#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  sim_mode.sh  —  put THIS terminal into SIMULATION mode
#
#  RUN IT AS:   . ~/sim_mode.sh      (with the leading dot!)
#  NOT AS:      ./sim_mode.sh        (that runs in a child shell and the
#                                     settings die when it exits)
#
#  WHY THIS FILE EXISTS
#    ~/.bashrc USED TO set the discovery-server variables for the real robot
#    in every new terminal. With the Pi switched off, every node then tried
#    to register with a discovery server that was not there, discovered
#    nothing, and NOTHING REPORTED AN ERROR: Gazebo loads the world, the
#    robot never spawns, and the log just repeats "Waiting messages on topic
#    [robot_description]" forever. That cost an hour to find.
#    Since 19 Aug the .bashrc guard (lines 125-130) unsets both variables
#    unless VLA_MODE=robot is exported first, so a FRESH shell is already
#    simulation-safe. This file is still how you switch a terminal that has
#    been put into robot mode.
#
#  ISOLATION -- HOW IT IS ACTUALLY ACHIEVED
#    NOT by ROS_DOMAIN_ID. Domain 42 was tried and REJECTED (handout 5.1):
#    gz_ros2_control runs INSIDE the Gazebo process and does not inherit the
#    domain, so controller_manager came up on 0 while the spawner looked for
#    it on 42 -- the robot spawned and could not move. The domain is left at
#    the default, matching vla_sim.sh and run_agent_sim.sh.
#    Isolation comes instead from: unsetting the two discovery-server
#    variables below, the mode lock (/tmp/vla_mode.lock) that stops both
#    modes running at once, and separate GUI configs.
#    The problem this file solves is real and was observed: /robot1/oakd/...
#    topics from the morning's hardware session were still visible in the
#    simulation and the agent bound to them instead of the simulated camera.
# ─────────────────────────────────────────────────────────────────

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "WARNING: /opt/ros/humble/setup.bash not found -- ros2 will NOT work" >&2
    echo "         (are you on the host instead of inside ubuntu22-gpu?)" >&2
fi
# ROS itself, FIRST. Without this there is no ros2 on PATH, and the daemon
# restart below fails silently -- the banner still prints, so nothing warns
# you. Sourcing it twice is harmless.

unset ROS_DOMAIN_ID
# Domain 42 was tried and REJECTED -- handout §5.1. gz_ros2_control runs
# INSIDE the Gazebo process and does not inherit the domain, so
# controller_manager came up on 0 while the spawner looked for it on 42: the
# robot spawned and could not move. vla_sim.sh (line 46) and run_agent_sim.sh
# (line 22) both unset it; this file now matches them. Isolation comes from
# the discovery-server unsets below, the mode lock and separate GUI configs.

unset FASTRTPS_DEFAULT_PROFILES_FILE
# the super-client XML points FastDDS at the Pi's discovery server

unset ROS_DISCOVERY_SERVER
# 10.42.0.169:11811 -- the Pi. Unreachable when the robot is off.

export VLA_NS=""
# EMPTY, not unset. vla_agent reads os.environ.get("VLA_NS", "/robot1"),
# so UNSETTING it hands back the /robot1 default -- the exact opposite of
# what is wanted. The simulation publishes bare /scan, /odom, /cmd_vel.

export VLA_RAW_RGB=1
# turn OFF compressed RGB (#50). Gazebo publishes plain sensor_msgs/Image;
# the /compressed topic does not exist here, so the agent would subscribe
# to nothing and report "I see: nothing" while the camera ran fine.

export VLA_RAW_DEPTH=1
# same for compressedDepth (#51). Both transports exist only to survive the
# Wi-Fi link, which simulation does not have.

# fastdds shm clean removes ONLY shared-memory segments whose owner process
# is dead. The rm -rf that used to be here also deleted the segments of
# RUNNING nodes, which cut them off from everything started afterwards on
# this PC: RViz map 0x0, "Frame [map] does not exist", map_saver "Failed to
# spin map subscription", save_map service hanging (2026-09-08).
if command -v fastdds >/dev/null 2>&1; then
    fastdds shm clean
else
    echo "  fastdds not on PATH -- stale shm not cleaned"
fi
# stale DDS shared memory from a previous session reports nodes that are
# already dead, which produces false health checks

ros2 daemon stop
sleep 2
ros2 daemon start
# the daemon caches the node/topic graph PER DOMAIN and lies after a switch.
# NOT silenced: these used to be >/dev/null 2>&1, which hid "ros2: command
# not found" completely. A daemon restart that did nothing looked identical
# to one that worked.

echo "──────────────────────────────────────────"
echo " SIM MODE   domain=(default, unset)   namespace=(empty)"
echo "   discovery : multicast (no server)"
echo "   transports: raw RGB + raw depth"
echo "──────────────────────────────────────────"
echo "Launch sim :  ros2 launch turtlebot4_ignition_bringup \\"
echo "                turtlebot4_ignition.launch.py model:=lite \\"
echo "                slam:=true nav2:=true rviz:=false"
echo "Launch agent: ~/run_agent_sim.sh"
