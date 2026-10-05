#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  run_sim_stack.sh — launch Gazebo + SLAM + Nav2 for the SIMULATION
#
#  RUN:  ~/run_sim_stack.sh          (in ubuntu22-gpu)
#
#  Everything the simulation needs, with the environment fixed inside the
#  script so it does not matter which terminal you start it from. That was
#  the single biggest time-waster: ~/.bashrc re-sets the discovery-server
#  variables in EVERY new terminal, so one forgotten terminal silently
#  broke the whole stack with no error message anywhere.
# ─────────────────────────────────────────────────────────────────

unset ROS_DOMAIN_ID
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_DISCOVERY_SERVER
# the three that must be clear for simulation

pkill -f "ign gazebo"      2>/dev/null
pkill -f gzserver          2>/dev/null
pkill -f parameter_bridge  2>/dev/null
pkill -f slam_toolbox      2>/dev/null
pkill -f controller_server 2>/dev/null
pkill -f bt_navigator      2>/dev/null
# kill any previous stack FIRST. Two Gazebo stacks publish conflicting TF,
# which produced "Tf has two or more unconnected trees", starved
# controller_server, and made the lifecycle manager shoot Nav2 dead 16 s
# after it came up. Always start from nothing.
sleep 4

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
# stale DDS shared memory reports nodes that are already dead

ros2 daemon stop  >/dev/null 2>&1
sleep 2
ros2 daemon start >/dev/null 2>&1
# the CLI daemon caches the graph and lies after a restart

echo "Launching simulation (robot takes ~30 s to appear)..."

ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
     model:=lite slam:=true nav2:=true rviz:=false
# rviz:=false — RViz segfaults in this container (exit -11, "egl: failed to
# create dri2 screen") and takes the whole launch down with it. The GUI's
# own feed panel replaces it.
