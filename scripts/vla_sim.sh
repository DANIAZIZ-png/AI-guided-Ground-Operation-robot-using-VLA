#!/bin/bash
# Resolve the repository root from this script's own location, so the script
# works from any working directory and from a clone anywhere on disk.
VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
source "$VLA_ROOT/config/paths.sh"
VLA_YOLO_ENV="${VLA_YOLO_ENV:-$HOME/yolo-env}"
VLA_GUI_CONFIG="${VLA_GUI_CONFIG:-$HOME/.vla_gui.json}"
# ─────────────────────────────────────────────────────────────────
#  vla_sim.sh — ONE COMMAND to bring up the whole SIMULATION demo
#
#  RUN:  scripts/vla_sim.sh          (in ubuntu22-gpu, from any terminal)
#
#  This is the BACKUP DEMO. It must work when the robot is in pieces on
#  the bench, so it owns its whole world and assumes nothing about the
#  terminal it was started from.
#
#  WHAT IT DOES, in the order that was proven to work by hand:
#     1. kill everything (by process GROUP, and verify)
#     2. fix the environment
#     3. start Gazebo + SLAM + Nav2   -> WAIT for /clock to actually TICK
#     4. start the VLA agent          -> WAIT for /vla/status to actually TICK
#     5. start voice + the GUI last
#
#  THE RULE THIS SCRIPT ENCODES
#    Wait for DATA, never for a topic name. A topic can exist while
#    nothing publishes on it -- that is exactly how the GUI's START ALL
#    reported "ready (0s)" three times while nothing worked. Every wait
#    below uses `ros2 topic echo --once`, which returns only when a real
#    message arrives.
#
#  WHY THE GUI IS STARTED LAST AND DOES NOT LAUNCH ANYTHING
#    vla_gui_v2.py binds its subscriptions once. If the agent restarts
#    underneath it, it keeps a dead subscription and shows stale status
#    forever. Starting it last, against an already-running agent, is the
#    only ordering that works. DO NOT PRESS "START ALL" -- it would kill
#    the very stack this script just built.
# ─────────────────────────────────────────────────────────────────

LOCK=/tmp/vla_mode.lock
LOG=/tmp/vla_sim_logs
mkdir -p "$LOG"

# ── 0. refuse to run if HARDWARE mode is active ──────────────────
if [ -f "$LOCK" ] && [ "$(cat $LOCK)" = "robot" ]; then
    echo "STOP: hardware mode is active. Close it, or: rm $LOCK"
    exit 1
fi

# ── 1. environment ───────────────────────────────────────────────
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset ROS_DOMAIN_ID
# ~/.bashrc sets the first two for the Pi in EVERY new terminal. With the
# robot off, nodes register with a discovery server that is not there and
# discover nothing -- no error, just silence. This cost hours.
# ROS_DOMAIN_ID must stay at the default: setting it to 42 broke
# gz_ros2_control, which runs inside Gazebo and does not inherit it, so
# controller_manager never appeared and the robot spawned but could not move.

export VLA_NS=""
# EMPTY, not unset -- the agent's fallback is "/robot1" when it is absent
export VLA_RAW_RGB=1
export VLA_RAW_DEPTH=1
# Gazebo publishes plain sensor_msgs/Image. The compressed topics (#50/#51)
# exist only on the real OAK-D; subscribing to them here silently gets
# nothing and the agent reports "I see: nothing" while the camera is fine.

# ── 2. clear everything, and PROVE it ────────────────────────────
if [ -x "$VLA_ROOT/scripts/vla_kill.sh" ]; then
    "$VLA_ROOT/scripts/vla_kill.sh" || { echo "STOP: old stack survived. Kill those PIDs first."; exit 1; }
else
    echo "STOP: $VLA_ROOT/scripts/vla_kill.sh missing or not executable (chmod +x it)."; exit 1
fi

echo "sim" > "$LOCK"

# ── 3. the two servers this script CANNOT start ──────────────────
# distrobox-host-exec returns exit 127 from inside ubuntu22-gpu, so there
# is no way to reach vla-box or the host from here. Check and say so.
if ! curl -s --max-time 3 http://localhost:5001/ >/dev/null 2>&1; then
    echo ""
    echo "  !! YOLO-World is NOT running — detection will not work."
    echo "     In a vla-box terminal:"
    echo "       distrobox enter vla-box"
    echo "       source $VLA_YOLO_ENV/bin/activate && python $VLA_SRC_DIR/yolo_server.py"
    echo ""
    read -p "  Press ENTER once YOLO is up (or Ctrl-C to abort)... " _
fi
if ! curl -s --max-time 3 http://localhost:11434/api/tags >/dev/null 2>&1; then
    echo "  !! Ollama is NOT running — on the HOST run:  ollama serve"
    read -p "  Press ENTER once Ollama is up (or Ctrl-C to abort)... " _
fi

# ── helper: wait until a topic delivers a REAL message ───────────
wait_for_topic () {            # $1 = topic, $2 = seconds, $3 = label
    local topic="$1" limit="$2" label="$3" waited=0
    printf "  waiting for %s " "$label"
    while [ $waited -lt $limit ]; do
        if timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1; then
            printf " OK (%ss)\n" "$waited"; return 0
        fi
        printf "."; sleep 5; waited=$((waited+5))
    done
    printf " TIMED OUT after %ss\n" "$limit"; return 1
}
# `echo --once` blocks until a message ARRIVES. `topic list` would return
# instantly for a topic that exists but is silent -- the exact false
# positive that made every step go green in 0 seconds.

# ── helper: wait until a topic has a PUBLISHER (event-driven topics) ──
wait_for_publisher () {        # $1 = topic, $2 = seconds, $3 = label
    local topic="$1" limit="$2" label="$3" waited=0 n
    printf "  waiting for %s " "$label"
    while [ $waited -lt $limit ]; do
        n=$(timeout 5 ros2 topic info "$topic" 2>/dev/null \
            | awk -F': *' '/^Publisher count:/ {print $2}')
        if [ -n "$n" ] && [ "$n" -gt 0 ] 2>/dev/null; then
            printf " OK (%ss)\n" "$waited"; return 0
        fi
        printf "."; sleep 5; waited=$((waited+5))
    done
    printf " TIMED OUT after %ss\n" "$limit"; return 1
}
# THE BOUNDARY CONDITION ON §3.2's RULE -- read before "simplifying" these two
# helpers into one. "Wait for DATA, never a topic name" is right for CONTINUOUS
# topics (/clock, /scan, /vla/status): they publish on a timer, so a message is
# always coming and wait_for_topic returns the moment one does.
#
# It is WRONG for EVENT-DRIVEN topics. /vla/voice/state publishes only on a
# STATE CHANGE: it says "ready" once -- possibly before this gate even started
# listening -- and then says nothing until the mic button is pressed. There is
# no next message to wait for, so `echo --once` blocked the full 240 s on a
# node that was already up and healthy (Publisher count: 1, Subscription
# count: 0, process idling at 5.2% CPU). It looked exactly like "Whisper is
# slow to load". It was not.
#
# So: ask what the topic's publishing model is. Continuous -> wait for a
# MESSAGE. Event-driven -> wait for a PUBLISHER, because a message may never
# come. Only the voice gate below is event-driven; every other gate in this
# script is continuous and correctly uses wait_for_topic.

# ── 4. Gazebo + SLAM + Nav2 ──────────────────────────────────────
echo ""
echo "[1/4] Starting Gazebo + SLAM + Nav2  (log: $LOG/gazebo.log)"
setsid ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
       model:=lite slam:=true nav2:=true rviz:=false \
       > "$LOG/gazebo.log" 2>&1 &
# setsid puts it in its OWN process group so vla_kill.sh can take the whole
# tree next time. rviz:=false because RViz segfaults in this container
# (exit -11, "egl: failed to create dri2 screen") and takes the launch with it.

if ! wait_for_topic /clock 180 "the simulator (/clock)"; then
    echo "  Gazebo did not come alive. Last 30 lines:"
    tail -30 "$LOG/gazebo.log"
    rm -f "$LOCK"; exit 1
fi
# /clock is the simulator's heartbeat. If it ticks, physics is running and
# every sensor downstream will work. If it does not, nothing else can.

if ! wait_for_topic /scan 60 "the LiDAR (/scan)"; then
    echo "  The robot spawned but its sensors are silent — usually"
    echo "  gz_ros2_control failing. Last 30 lines:"
    tail -30 "$LOG/gazebo.log"
    rm -f "$LOCK"; exit 1
fi

wait_for_topic /oakd/rgb/preview/image_raw 60 "the camera" || \
    echo "  (camera slow — continuing; check with: ros2 topic hz /oakd/rgb/preview/image_raw)"

# ── 5. the VLA agent ─────────────────────────────────────────────
echo ""
echo "[2/4] Starting the VLA agent  (log: $LOG/agent.log)"
AGENT="$VLA_SRC_DIR/vla_agent_v28.py"                 # change this ONE line for a new version
[ -f "$AGENT" ] || { echo "STOP: $AGENT not found."; rm -f "$LOCK"; exit 1; }

# #60 MANUAL OVERRIDE NEEDS A REAL TTY -- see the long note in vla_robot.sh.
# Redirecting to a log file made the agent headless, so override was
# unreachable in the SIMULATION too, not just on hardware. Do NOT pipe this to
# tee to get a log back: a pipe is not a tty either, and it breaks the same
# way. The agent writes its own mission log to ~/vla_logs/ regardless.
if [ -n "$DISPLAY" ] && command -v xterm >/dev/null 2>&1; then
    setsid xterm -hold -sb -sl 5000 \
           -T "VLA AGENT (sim) — type 'manual override' HERE" \
           -e python3 -u "$AGENT" --ros-args \
           -r /oakd/stereo/image_raw:=/oakd/rgb/preview/depth \
           -r /oakd/stereo/camera_info:=/oakd/rgb/preview/camera_info &
    echo "  Agent window opened. Manual override is typed in THAT window."
else
    echo "  !! No DISPLAY (or no xterm) — HEADLESS, manual override unavailable."
    setsid python3 -u "$AGENT" --ros-args \
        -r /oakd/stereo/image_raw:=/oakd/rgb/preview/depth \
        -r /oakd/stereo/camera_info:=/oakd/rgb/preview/camera_info \
        > "$LOG/agent.log" 2>&1 &
fi
# The remaps let ONE agent file serve both sim and hardware. The agent still
# thinks it uses the stereo topics; ROS rewires them underneath. Without the
# depth remap its synchronizer never forms an (RGB, depth) pair, so the
# callback that runs YOLO never fires and it reports "I see: nothing".
# #63 keeps its prints from aborting a command if its output goes away.

if ! wait_for_topic /vla/status 90 "the agent (/vla/status)"; then
    echo "  Agent did not come up. Errors are in the AGENT WINDOW."
    [ -s "$LOG/agent.log" ] && { echo "  (headless-fallback log:)"; tail -30 "$LOG/agent.log"; }
    echo "  Mission log: $VLA_LOG_DIR/"
    rm -f "$LOCK"; exit 1
fi

# ── 6. voice ─────────────────────────────────────────────────────
echo ""
echo "[3/4] Starting voice input  (log: $LOG/voice.log)"
if [ -f "$VLA_SRC_DIR/voice_command.py" ]; then
    setsid python3 -u "$VLA_SRC_DIR/voice_command.py" --mode ptt --mic pulse \
           --device cpu --compute int8 > "$LOG/voice.log" 2>&1 &
    # -u: without it Python buffers stdout into the log file and voice.log stays
    # EMPTY while it runs, so a hung step cannot be diagnosed (§8.6).
    #
    # wait_for_publisher, NOT wait_for_topic -- /vla/voice/state is event-driven.
    # See the long note on the helper above. 60 s is ample for a node to create
    # a publisher; the old 240 s only existed to cover a slow model load that
    # was never actually the problem.
    wait_for_publisher /vla/voice/state 60 "the voice node" || \
        echo "  (voice not ready — text commands still work)"
else
    echo "  $VLA_SRC_DIR/voice_command.py not found — skipping voice."
fi

# ── 7. the GUI, last ─────────────────────────────────────────────
echo ""
echo "[4/4] Starting the operator console"
[ -f "$VLA_CONFIG_DIR/vla_gui.sim.json" ] && cp "$VLA_CONFIG_DIR/vla_gui.sim.json" "$VLA_GUI_CONFIG"
# the GUI reads VLA_GUI_CONFIG (default ~/.vla_gui.json); the mode's config is copied in
# ── 7b. RViz — the map view for the audience ─────────────────────
RVIZ_CFG=/opt/ros/humble/share/turtlebot4_viz/rviz/robot.rviz
# the stock TurtleBot 4 config: map, laser scan, robot model and TF are
# already set up, so nothing has to be added by hand in front of people

if [ -f "$RVIZ_CFG" ]; then
    LIBGL_ALWAYS_SOFTWARE=1 setsid rviz2 -d "$RVIZ_CFG" \
        > "$LOG/rviz.log" 2>&1 &
    # LIBGL_ALWAYS_SOFTWARE=1 forces Mesa's CPU renderer (llvmpipe) instead
    # of the GPU's EGL/DRI path. That path is what fails in this container
    # ("egl: failed to create dri2 screen") and caused the exit -11 SIGSEGV.
    # Bypassing it entirely is what makes RViz survive here.
    #
    # setsid gives RViz its OWN process group. This is the important half:
    # launched with rviz:=true it shares Gazebo's group, so a segfault takes
    # SLAM and Nav2 down with it. Decoupled, a crash costs only the display.
    #
    # No wait_for_topic gate: RViz is a VIEWER. It publishes nothing the
    # stack depends on, so nothing downstream should ever block on it.
    echo "  RViz map view starting (log: $LOG/rviz.log)"
    echo "  ...it renders on the CPU, so it is slow to draw. This is normal."
else
    echo "  (RViz config not found at $RVIZ_CFG — skipping the map view)"
fi
echo ""
echo "═══════════════════════════════════════════════════"
echo "  SIMULATION READY"
echo ""
echo "  DO NOT PRESS 'START ALL' — everything is already"
echo "  running. Pressing it would kill this stack."
echo ""
echo "  Try:  what do you see  |  go to the chair"
echo "  Feed: press 'Show feed'"
echo "  Map:  the RViz window — the robot's live map of the room"
echo "  Logs: $LOG/"
echo "  Stop: $VLA_ROOT/scripts/vla_kill.sh"
echo "═══════════════════════════════════════════════════"
echo ""

python3 "$VLA_SRC_DIR/vla_gui_v2.py"
# runs in the foreground: closing the GUI returns you to this prompt.
# The stack keeps running (each part is in its own session via setsid),
# so you can reopen the GUI without rebuilding anything.

echo ""
echo "GUI closed. The simulation is STILL RUNNING."
echo "  reopen the console:  python3 $VLA_SRC_DIR/vla_gui_v2.py"
echo "  stop everything:     $VLA_ROOT/scripts/vla_kill.sh"
