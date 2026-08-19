#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  sim_mode.sh  —  put THIS terminal into SIMULATION mode
#
#  RUN IT AS:   . ~/sim_mode.sh      (with the leading dot!)
#  NOT AS:      ./sim_mode.sh        (that runs in a child shell and the
#                                     settings die when it exits)
#
#  WHY THIS FILE EXISTS
#    ~/.bashrc sets the discovery-server variables for the REAL ROBOT in
#    every new terminal. With the Pi switched off, every node then tries to
#    register with a discovery server that is not there, discovers nothing,
#    and NOTHING REPORTS AN ERROR: Gazebo loads the world, the robot never
#    spawns, and the log just repeats "Waiting messages on topic
#    [robot_description]" forever. That cost an hour to find.
#
#  THE ISOLATION GUARANTEE
#    ROS_DOMAIN_ID is the important line below. Two ROS 2 systems on
#    DIFFERENT domain IDs cannot see each other at all -- not their topics,
#    not their nodes, not their TF. Domain 42 is the simulation; domain 0
#    is the real robot. This is what makes the simulation a SAFE BACKUP
#    DEMO: powering the robot on, or leaving stale hardware topics in the
#    network, can no longer reach into the simulation and break it.
#    Observed before this was added: /robot1/oakd/... topics from the
#    morning's hardware session were still visible in the simulation and
#    the agent bound to them instead of the simulated camera.
# ─────────────────────────────────────────────────────────────────

export ROS_DOMAIN_ID=42
# THE ISOLATION LINE. 42 is the simulation's own universe; the robot uses 0.
# Any value 0-101 is safe; it only has to differ from the robot's.

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

rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
# stale DDS shared memory from a previous session reports nodes that are
# already dead, which produces false health checks

ros2 daemon stop >/dev/null 2>&1
sleep 2
ros2 daemon start >/dev/null 2>&1
# the daemon caches the node/topic graph PER DOMAIN and lies after a switch

echo "──────────────────────────────────────────"
echo " SIM MODE   domain=$ROS_DOMAIN_ID   namespace=(empty)"
echo "   discovery : multicast (no server)"
echo "   transports: raw RGB + raw depth"
echo "──────────────────────────────────────────"
echo "Launch sim :  ros2 launch turtlebot4_ignition_bringup \\"
echo "                turtlebot4_ignition.launch.py model:=lite \\"
echo "                slam:=true nav2:=true rviz:=false"
echo "Launch agent: ~/run_agent_sim.sh"
