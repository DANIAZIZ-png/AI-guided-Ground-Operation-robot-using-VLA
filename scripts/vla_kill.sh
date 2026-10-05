#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  vla_kill.sh — stop EVERYTHING this project starts, reliably
#
#  RUN:  ~/vla_kill.sh        (in ubuntu22-gpu)
#
#  WHY THIS FILE EXISTS
#    The first version of vla_sim.sh killed a hand-written list of ~10
#    node names. That list was wrong: `ros2 launch turtlebot4_ignition`
#    starts about 50 processes, and robot_state_publisher was not on the
#    list. After a few relaunches there were EIGHT robot_state_publisher
#    processes alive at once, all advertising the same service and all
#    publishing TF. Consequences:
#       gz_ros2_control: "robot_state_publisher service not available"
#                         forever -> controller_manager never appears
#                         -> the robot spawns but cannot move
#       controller_server: "Tf has two or more unconnected trees"
#    Neither error names the real cause. Both look like Gazebo faults.
#
#    A hand-written list can never be complete. This kills by PROCESS
#    GROUP instead: every child of a `ros2 launch` shares its parent's
#    process-group id, so killing the group takes the whole tree at once,
#    including nodes nobody remembered to list.
# ─────────────────────────────────────────────────────────────────

echo "Stopping everything..."

# ── 1. kill whole launch trees by process group ──────────────────
for PGID in $(ps -eo pgid,cmd --no-headers \
              | grep -E "ros2 launch|ign gazebo|vla_agent|voice_command|vla_gui|rviz2" \
              | grep -v grep \
              | awk '{print $1}' | sort -u); do
    [ "$PGID" = "$$" ] && continue          # never kill ourselves
    kill -TERM -"$PGID" 2>/dev/null
    # the MINUS before $PGID is what makes this a process-GROUP kill.
    # TERM first so nodes get a chance to shut down cleanly.
done
sleep 4

# ── 2. anything that ignored TERM gets KILL ──────────────────────
for PGID in $(ps -eo pgid,cmd --no-headers \
              | grep -E "ros2 launch|ign gazebo|vla_agent|voice_command|vla_gui|rviz2" \
              | grep -v grep \
              | awk '{print $1}' | sort -u); do
    [ "$PGID" = "$$" ] && continue
    kill -KILL -"$PGID" 2>/dev/null
done
sleep 2

# ── 3. sweep any orphan left behind ──────────────────────────────
# A node whose parent died gets re-parented to init and leaves its old
# process group, so step 1 can miss it. This is the safety net, not the
# main mechanism.
for P in ign gazebo gzserver parameter_bridge robot_state_publisher \
         joint_state_publisher static_transform_publisher turtlebot4_node \
         ros_gz_sim gz_ros2_control slam_toolbox controller_server \
         planner_server behavior_server bt_navigator waypoint_follower \
         velocity_smoother smoother_server lifecycle_manager rviz2 \
         hazards_vector ir_intensity motion_control wheel_status \
         mock_publisher kidnap_estimator ui_mgr pose_republisher \
         sensors_node interface_buttons vla_agent voice_command vla_gui ; do
    pkill -9 -f "$P" 2>/dev/null
done
sleep 2

# ── 4. clear the DDS state that outlives the processes ───────────
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
# stale shared memory makes dead nodes look alive to health checks
# the CLI daemon caches the node/topic graph and reports nodes that are gone,
# so it is stopped here. It is RESTARTED only if this terminal is in robot
# mode. A daemon inherits the environment of whatever shell starts it, and
# this script used to restart it silently from ANY terminal. From a fresh
# terminal -- where ~/.bashrc strips ROS_DISCOVERY_SERVER and
# FASTRTPS_DEFAULT_PROFILES_FILE -- that produced a daemon that cannot see
# the robot at all: `ros2 node list` empty and `topic info` "Unknown topic"
# in EVERY terminal, until the next `source ~/robot_mode.sh` (2026-09-08).
if ! command -v ros2 >/dev/null 2>&1; then
    echo "  ros2 not on PATH here (host?) -- daemon untouched"
else
    ros2 daemon stop
    sleep 2
    if [ -n "$ROS_DISCOVERY_SERVER" ] && [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        ros2 daemon start
        echo "  daemon restarted in ROBOT mode (server $ROS_DISCOVERY_SERVER)"
    else
        echo "  daemon STOPPED, not restarted: this terminal has no discovery-server"
        echo "  config. The next 'source ~/robot_mode.sh' (robot) or"
        echo "  'source ~/sim_mode.sh' (sim) restarts it correctly."
    fi
fi

# ── 5. PROVE it worked instead of assuming ───────────────────────
SURVIVORS="ign gazebo|robot_state_publisher|slam_toolbox|vla_agent"
# one pattern for both the count and the listing. It used to be undefined
# at the pgrep -af below, so a survivor listed EVERY process (2026-09-08).
LEFT=$(pgrep -c -f "$SURVIVORS" 2>/dev/null)
RSP=$(pgrep -c -f robot_state_publisher 2>/dev/null)
echo "  robot_state_publisher processes: $RSP   (must be 0)"
if [ "$LEFT" -gt 0 ] 2>/dev/null; then
    echo "  !! $LEFT process(es) survived:"
    pgrep -af "$SURVIVORS"
    echo "  Kill them by PID before relaunching, or the next run WILL fail."
    exit 1
fi
echo "  clean"
exit 0
