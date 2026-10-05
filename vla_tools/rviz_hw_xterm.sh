#!/bin/bash
source /home/danyalaziz/robot_env.sh
# 14 Sep: /parameter_events and /rosout remapped off the Wi-Fi link, same reason
# as Nav2 (CHANGELOG_2026-09-11.md §2.4): the Pi's camera node stormed RViz's
# /parameter_events reader at 4,700 heartbeats/s. With the PC-side name changed
# the two never match.
ros2 run rviz2 rviz2 -d /home/danyalaziz/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static -r /parameter_events:=/pc/parameter_events -r /rosout:=/pc/rosout
echo; echo "[RViz exited -- window kept open]"; exec bash
