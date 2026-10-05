#!/bin/bash
# Runs from inside ubuntu22-gpu. Sourcing robot_mode.sh first is what lets
# vla_kill.sh restart the ROS daemon in ROBOT mode instead of leaving it dead.
source /home/danyalaziz/robot_mode.sh
echo "---- now stopping the stack ----"
/home/danyalaziz/vla_kill.sh
