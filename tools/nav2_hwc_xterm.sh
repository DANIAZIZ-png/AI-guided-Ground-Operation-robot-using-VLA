#!/bin/bash
source /home/danyalaziz/robot_env.sh
ros2 launch /home/danyalaziz/nav2_hw_composed.launch.py 2>&1 | tee -i -a /home/danyalaziz/vla_logs/nav2_hwc_$(date +%Y%m%d).log
echo; echo "[Nav2 exited -- window kept open]"; exec bash
