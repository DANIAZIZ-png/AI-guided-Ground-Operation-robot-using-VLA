# robot_env.sh -- HARDWARE environment for NON-INTERACTIVE shells. Source it.
#
#   distrobox enter ubuntu22-gpu -- bash -c "source ~/robot_env.sh; <command>"
#
# Same variables as robot_mode.sh, WITHOUT its side effects: no shared-memory
# clean, no ROS daemon restart, no banner. Use robot_mode.sh in a terminal
# you type into; use this file inside scripts, xterm -e launchers and
# bash -c wrappers, where restarting the daemon on every call would leave
# every other terminal blind for ~10 s each time (CHANGELOG_2026-09-08.md §2).
# Hardware only. The simulation never sources this file.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/danyalaziz/.ros/fastdds_hw_wifi_only.xml   # 11 Sep: + interface whitelist (CHANGELOG_2026-09-11.md)
export ROS_DISCOVERY_SERVER="10.42.0.169:11811;"
export VLA_NS=/robot1
unset VLA_RAW_RGB VLA_RAW_DEPTH
