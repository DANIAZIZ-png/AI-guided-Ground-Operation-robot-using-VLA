#!/usr/bin/env bash
# =====================================================================
#  PROJECT COMMAND CHEAT SHEET
#  AI-Guided Ground Operations Robot (VLA)  -  Danyal Aziz
#
#  THIS IS A REFERENCE, NOT A SCRIPT TO RUN ALL AT ONCE.
#  Find the action you want, copy THAT ONE command, and paste it into
#  the right container's terminal. Each block says which box to use.
#
#  CONTAINERS:
#    ubuntu22-gpu  ->  ROS 2, Gazebo, Nav2, all robot nodes   (GPU)
#    vla-box       ->  YOLO-World server, OpenVLA server       (GPU)
#  Confirm which box you are in:   echo $CONTAINER_ID   (empty = host)
# =====================================================================


# ---------------------------------------------------------------------
#  0. ENTER A CONTAINER
# ---------------------------------------------------------------------
# Enter the ROS / Gazebo box (everything robot-side):
distrobox enter ubuntu22-gpu

# Enter the model-server box (YOLO / OpenVLA):
distrobox enter vla-box

# Which container am I in?  (empty output = you are on the host)
echo $CONTAINER_ID

# List all containers  (run this on the HOST):
podman ps -a


# ---------------------------------------------------------------------
#  1. SOURCE ROS   (do this FIRST in every new ubuntu22-gpu terminal)
# ---------------------------------------------------------------------
source /opt/ros/humble/setup.bash


# ---------------------------------------------------------------------
#  2. PRE-FLIGHT CLEANUP   (before launching Gazebo)
# ---------------------------------------------------------------------
# Free the GPU if the OpenVLA server is still running:
pkill -9 -f vla_server.py

# Check the GPU is free  (a python process using ~15 GB = OpenVLA):
nvidia-smi

# Clear stale FastDDS shared-memory locks  (fixes TF "two unconnected
# trees" / transport errors). Do this with NO ros nodes running:
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*


# ---------------------------------------------------------------------
#  3. TURTLEBOT BRING-UP IN GAZEBO + NAV2     [ubuntu22-gpu]
# ---------------------------------------------------------------------
# (A) LIVE SLAM mode - builds the map as it drives (use for exploring):
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  model:=lite slam:=true nav2:=true rviz:=true

# (B) LOCALIZATION mode - uses a SAVED map (the reliable demo). After it
#     loads, set the start pose with "2D Pose Estimate" in RViz:
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  model:=lite slam:=false localization:=true map:=$HOME/warehouse_map.yaml \
  nav2:=true rviz:=true

# See all valid launch arguments  (if an arg above is rejected):
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py --show-args


# ---------------------------------------------------------------------
#  4. START THE YOLO-WORLD DETECTION SERVER     [vla-box]
# ---------------------------------------------------------------------
source ~/yolo-env/bin/activate
python ~/yolo_server.py          # serves detections on port 5001


# ---------------------------------------------------------------------
#  5. START THE OPENVLA SERVER  (BASELINE ONLY)     [vla-box]
# ---------------------------------------------------------------------
# WARNING: OpenVLA uses ~15 GB and CANNOT run at the same time as
# Gazebo. Use it only for the baseline test, then kill it (step 2)
# BEFORE launching Gazebo.
source ~/openvla-env/bin/activate
python ~/vla_server.py           # serves on port 5000


# ---------------------------------------------------------------------
#  6. GIVE A LANGUAGE COMMAND   [ubuntu22-gpu]  (Gazebo + YOLO must be up)
# ---------------------------------------------------------------------
# Drive to a named object (reliable, single-goal navigator):
python3 ~/object_navigator.py go to the chair

# Search + map + drive (active explorer that looks for the target):
python3 ~/object_explorer.py go to the red box


# ---------------------------------------------------------------------
#  7. PUT OBJECTS IN THE WORLD   [ubuntu22-gpu]  (after Gazebo is up)
# ---------------------------------------------------------------------
# One-time: make the folder for the object models:
mkdir -p ~/sim_objects

# Check the model files are present:
ls ~/sim_objects

# Spawn the objects (cup, red box, door) into the running world:
python3 ~/spawn_objects.py


# ---------------------------------------------------------------------
#  8. RESET / TELEPORT THE ROBOT   [ubuntu22-gpu]
# ---------------------------------------------------------------------
# Move robot to x, y, yaw(radians) in BOTH Gazebo and RViz:
python3 ~/reset_robot.py 1.5 2.0 1.57

# Send it back to the origin:
python3 ~/reset_robot.py


# ---------------------------------------------------------------------
#  9. SAVE THE MAP   [ubuntu22-gpu]   (after exploring in SLAM mode)
# ---------------------------------------------------------------------
# Saves ~/warehouse_map.yaml + ~/warehouse_map.pgm :
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_map


# ---------------------------------------------------------------------
#  10. DIAGNOSTICS & FIXES   [ubuntu22-gpu unless noted]
# ---------------------------------------------------------------------
# Is the camera publishing frames?  (Ctrl+C to stop)
ros2 topic hz /oakd/rgb/preview/image_raw

# Did the robot actually spawn?  (prints odometry if yes)
ros2 topic echo /odom --once

# What camera frame is used?  (should end in _optical_frame)
ros2 topic echo /oakd/rgb/preview/camera_info --field header.frame_id --once

# Is Gazebo rendering on the GPU?  (should say NVIDIA, NOT llvmpipe)
glxinfo | grep "OpenGL renderer"

# List all running nodes / all topics:
ros2 node list
ros2 topic list

# Is the robot in localization mode?  (AMCL listed = yes)
ros2 node list | grep amcl

# Remove the Create 3 backward-drive limit (keeps forward cliff safety):
ros2 param set /motion_control safety_override backup_only

# Install a missing tf2 package (if the object nodes error on import):
sudo apt install ros-humble-tf2-geometry-msgs


# =====================================================================
#  TYPICAL FULL DEMO  (3 terminals)
# =====================================================================
#  TERMINAL 1  [ubuntu22-gpu]  - simulator + Nav2
#     distrobox enter ubuntu22-gpu
#     source /opt/ros/humble/setup.bash
#     ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
#       model:=lite slam:=false localization:=true map:=$HOME/warehouse_map.yaml \
#       nav2:=true rviz:=true
#
#  TERMINAL 2  [vla-box]  - detection server
#     distrobox enter vla-box
#     source ~/yolo-env/bin/activate
#     python ~/yolo_server.py
#
#  TERMINAL 3  [ubuntu22-gpu]  - put objects in + give the command
#     distrobox enter ubuntu22-gpu
#     source /opt/ros/humble/setup.bash
#     python3 ~/spawn_objects.py
#     python3 ~/object_navigator.py go to the chair
# =====================================================================
