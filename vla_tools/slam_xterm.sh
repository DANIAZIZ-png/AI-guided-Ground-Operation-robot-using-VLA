#!/bin/bash
source /home/danyalaziz/robot_env.sh
ros2 launch /home/danyalaziz/slam_hw.launch.py namespace:=/robot1 params:=/home/danyalaziz/slam_vla.yaml   # 14 Sep: wrapper adds the parameter_events/rosout remaps
echo; echo "[SLAM exited -- window kept open; close it when done]"; exec bash
