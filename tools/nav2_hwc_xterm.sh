#!/bin/bash
VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
source "$VLA_ROOT/config/robot.env"
ros2 launch $VLA_LAUNCH_DIR/nav2_hw_composed.launch.py 2>&1 | tee -i -a $VLA_LOG_DIR/nav2_hwc_$(date +%Y%m%d).log
echo; echo "[Nav2 exited -- window kept open]"; exec bash
