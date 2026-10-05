#!/bin/bash
VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
source "$VLA_ROOT/config/robot.env"
ros2 launch $VLA_LAUNCH_DIR/slam_hw.launch.py namespace:=/robot1 params:=$VLA_CONFIG_DIR/slam_vla.yaml   # 14 Sep: wrapper adds the parameter_events/rosout remaps
echo; echo "[SLAM exited -- window kept open; close it when done]"; exec bash
