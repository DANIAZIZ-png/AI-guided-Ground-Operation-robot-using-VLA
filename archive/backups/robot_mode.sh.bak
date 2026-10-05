#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  robot_mode.sh  —  put THIS terminal into REAL-ROBOT mode
#
#  RUN IT AS:   . ~/robot_mode.sh     (with the leading dot!)
#
#  This restores exactly what ~/.bashrc normally sets. Its job is to undo
#  sim_mode.sh in a terminal that has been switched, so you never have to
#  remember which variables were changed.
#
#  BEFORE YOU USE IT, the robot must be ON and reachable:
#      ping -c 3 10.42.0.169
#  The discovery server lives on the Pi. If the Pi is off, every node here
#  will register with nothing and silently discover nothing.
# ─────────────────────────────────────────────────────────────────

export ROS_DOMAIN_ID=0
# the real robot's domain. The Pi's own nodes are on 0, so this must be 0.
# Simulation sits on 42 and is now invisible from here -- that is the point.

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/danyalaziz/.ros/fastdds_super_client.xml
# the permanent .bashrc fix: tells FastDDS to act as a SUPER CLIENT, i.e.
# to get its whole view of the network from the discovery server

export ROS_DISCOVERY_SERVER="10.42.0.169:11811;"
# the Pi. The trailing semicolon is required -- it terminates the server
# list, and FastDDS mis-parses the entry without it.

export VLA_NS=/robot1
# the real robot runs everything under /robot1 (#37)

unset VLA_RAW_RGB
# re-enable compressed RGB (#50): 187 kB -> ~10 kB per frame. Raw over the
# Wi-Fi link is what made depth collapse to one frame every twenty seconds.

unset VLA_RAW_DEPTH
# re-enable compressedDepth (#51), same reason

rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
# stale shared memory causes false health positives

ros2 daemon stop >/dev/null 2>&1
sleep 2
ros2 daemon start >/dev/null 2>&1
# the daemon caches per domain and must be restarted after a switch

echo "──────────────────────────────────────────"
echo " ROBOT MODE   domain=$ROS_DOMAIN_ID   namespace=$VLA_NS"
echo "   discovery : server $ROS_DISCOVERY_SERVER"
echo "   transports: compressed RGB + compressed depth"
echo "──────────────────────────────────────────"
echo "Startup order (never skip, never reorder):"
echo "  1. Pi:  sudo systemctl stop turtlebot4 ; sleep 10 ;"
echo "          sudo systemctl restart discovery ; sleep 15 ;"
echo "          sudo systemctl start turtlebot4"
echo "  2. CHECK THE CLOCK -- a 118 s drift makes SLAM drop every scan:"
echo "       ssh ubuntu@10.42.0.169 'chronyc tracking | grep \"System time\"'"
echo "       if it is more than ~1 s out:  sudo chronyc makestep"
echo "  3. verify: base_link in /robot1/tf, dock_status publisher = 1"
echo "  4. camera: ros2 service call /robot1/oakd/start_camera std_srvs/srv/Trigger"
echo "  5. SLAM, then Nav2, then the agent"
