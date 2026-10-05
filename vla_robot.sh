#!/bin/bash
# ─────────────────────────────────────────────────────────────────
#  vla_robot.sh — ONE COMMAND to run the REAL ROBOT demo
#
#  RUN:  ~/vla_robot.sh          (in ubuntu22-gpu)
#
#  Mirror image of vla_sim.sh. Same guarantee: it owns its whole world, so
#  nothing the simulation left behind can leak into a hardware session.
# ─────────────────────────────────────────────────────────────────

LOCK=/tmp/vla_mode.lock
PI=10.42.0.169
LOG=/tmp/vla_robot_logs
mkdir -p "$LOG"

# ── 1. refuse to run if SIMULATION mode is active ────────────────
if [ -f "$LOCK" ] && [ "$(cat $LOCK)" = "sim" ]; then
    echo "STOP: simulation mode is currently active."
    echo "Close it first, or clear the lock:  rm $LOCK"
    exit 1
fi

# ── 2. is the robot even reachable? ──────────────────────────────
if ! timeout 5 bash -c "cat < /dev/null > /dev/tcp/$PI/22" 2>/dev/null; then
    echo "STOP: cannot reach the robot at $PI."
    echo "Power it on and wait for Wi-Fi, or run ~/vla_sim.sh instead."
    exit 1
fi
# DO NOT put `ping` back here. /usr/bin/ping EXISTS inside ubuntu22-gpu but
# exits 2 with NO output at all -- no header, no statistics -- for EVERY
# address, reachable or not: ICMP needs a raw-socket capability the container
# is not granted. So the old `ping -c 2 -W 2` check failed with the robot up
# and healthy, and the script refused to start. A container permission
# problem wearing the mask of a network fault. The giveaway is the silence:
# a real unreachable host still prints a header and a 100%-loss summary.
#
# bash's /dev/tcp opens an ordinary TCP socket -- no special permission -- and
# port 22 is the port this script actually depends on: section 3 ssh's to the
# Pi for the clock check. Testing the port we use beats testing ICMP we don't.
#
# checking FIRST turns a confusing 20-minute silent failure into one line.
# Every node would otherwise register with a discovery server that is not
# there and discover nothing, with no error printed anywhere.

# ── 3. CLOCK CHECK — the one that cost two hours ─────────────────
echo "Checking the Pi's clock..."
OFFSET=$(ssh -o ConnectTimeout=5 ubuntu@$PI \
         "chronyc tracking | grep 'System time'" 2>/dev/null)
echo "  $OFFSET"
echo "  ^ if that is more than ~1 second, run:"
echo "      ssh ubuntu@$PI 'sudo chronyc makestep'"
echo ""
# A 118 s drift makes every scan arrive stamped in the past. SLAM then drops
# ALL of them -- "Message Filter dropping message ... queue is full" -- which
# looks exactly like the silent-Create-3 fault and is not. The tell: compare
# the scan's timestamp against the log line's own time. Seconds apart = TF.
# Two minutes apart = the clock.

# ── 4. environment: real robot ───────────────────────────────────
unset ROS_DOMAIN_ID
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_super_client.xml
export ROS_DISCOVERY_SERVER="$PI:11811;"
# the trailing semicolon is REQUIRED -- FastDDS mis-parses the list without it
export VLA_NS=/robot1
unset VLA_RAW_RGB
unset VLA_RAW_DEPTH
# compressed transports back ON (#50/#51): 187 kB -> ~10 kB per frame.
# Raw over the Wi-Fi link is what made depth collapse to one frame per 20 s.

# ── 5. clear anything left running ───────────────────────────────
# Same delegation as the sim path: kill by PROCESS GROUP, and refuse to
# continue if anything survived. A leftover node from a previous run is
# the single most common cause of a session that looks healthy and is not.
if [ -x ~/vla_kill.sh ]; then
    ~/vla_kill.sh || {
        echo "STOP: could not clear the old stack. Kill the listed PIDs first."
        exit 1
    }
else
    echo "STOP: ~/vla_kill.sh not found (chmod +x it)."
    exit 1
fi

# ── 6. install the HARDWARE gui config ───────────────────────────
if [ ! -f ~/.vla_gui.robot.json ]; then
    echo "STOP: ~/.vla_gui.robot.json not found."
    exit 1
fi
cp ~/.vla_gui.robot.json ~/.vla_gui.json

echo "robot" > "$LOCK"
trap 'rm -f "$LOCK"' EXIT

# ── 6b. the two servers this script CANNOT start ─────────────────
# distrobox-host-exec returns exit 127 from inside ubuntu22-gpu, so there
# is no way to reach vla-box or the host from here. Check and say so.
if ! curl -s --max-time 3 http://localhost:5001/ >/dev/null 2>&1; then
    echo ""
    echo "  !! YOLO-World is NOT running — detection will not work."
    echo "     In a vla-box terminal:"
    echo "       distrobox enter vla-box"
    echo "       source ~/yolo-env/bin/activate && python ~/yolo_server.py"
    echo ""
    read -p "  Press ENTER once YOLO is up (or Ctrl-C to abort)... " _
fi
if ! curl -s --max-time 3 http://localhost:11434/api/tags >/dev/null 2>&1; then
    echo "  !! Ollama is NOT running — on the HOST run:  ollama serve"
    read -p "  Press ENTER once Ollama is up (or Ctrl-C to abort)... " _
fi

# ── 7. the health checks that decide whether a session is worth it ──
echo "Verifying the robot..."
timeout 20 ros2 topic echo /robot1/tf 2>/dev/null | grep -m10 "child_frame_id" \
    | sort | uniq -c | grep base_link \
    && echo "  OK  odom -> base_link is publishing" \
    || echo "  !!  base_link MISSING -> Create 3 is silent. POWER-CYCLE IT:
      dock it, hold power until the ring goes dark, lift off,
      wait 15 s, put it back. No software fix for this one."

timeout 15 ros2 topic echo /robot1/battery_state --field percentage --once 2>/dev/null \
    | head -1 | awk '{printf "  battery %.0f%%\n", $1*100}'
# below ~40% the base starts going silent mid-session; charge before demoing


# ── 7b. start the camera — docking turns it off ──────────────────
echo ""
echo "Starting the camera (needed after EVERY dock or service restart)."
echo "  This can take up to 2 minutes. Do not interrupt it."
ros2 service call /robot1/oakd/start_camera std_srvs/srv/Trigger \
    && echo "  OK  camera started" \
    || echo "  !!  start_camera FAILED — the agent will report 'I see: nothing'.
      Retry by hand:
        ros2 service call /robot1/oakd/start_camera std_srvs/srv/Trigger"
# NEVER wrap this call in `timeout`. SIGTERM kills the ros2 CLI node while it
# is still discovering, and what you get is "rcl node's context is invalid" --
# an error about the CLI being killed, not about the camera. It sends you
# debugging the wrong subsystem entirely. Let it run as long as it needs.

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
# instantly for a topic that exists but is silent -- the false positive that
# made the GUI's launcher report "ready (0s)" three times while nothing worked.

# ── 8. the VLA agent ─────────────────────────────────────────────
echo ""
echo "Starting the VLA agent in its OWN TERMINAL window"
AGENT=~/vla_agent_v28.py                 # change this ONE line for a new version
[ -f "$AGENT" ] || { echo "STOP: $AGENT not found."; exit 1; }

# #60 MANUAL OVERRIDE NEEDS A REAL TTY. `setsid python3 ... > log 2>&1 &` gave
# the agent no terminal, so it ran HEADLESS (#33) and refused override with
# "(ignoring manual override from remote)". Override is the documented fallback
# for the unreliable stop (§8.1), so losing it in the GUI -- the only interface
# a demo actually uses -- is a SAFETY gap, not a convenience one.
#
# DO NOT "keep a log" by piping this: `| tee agent.log` makes stdout a PIPE,
# isatty() goes false, and the agent is silently headless again with override
# dead. Nothing is lost -- it writes its own mission log to ~/vla_logs/.
if [ -n "$DISPLAY" ] && command -v xterm >/dev/null 2>&1; then
    setsid xterm -hold -sb -sl 5000 \
           -T "VLA AGENT — type 'manual override' HERE" \
           -e python3 -u "$AGENT" &
    echo "  Agent window opened. Manual override is typed in THAT window."
else
    echo "  !! No DISPLAY (or no xterm) — starting the agent HEADLESS."
    echo "     MANUAL OVERRIDE WILL NOT BE AVAILABLE (#60 needs a real tty)."
    setsid python3 -u "$AGENT" > "$LOG/agent.log" 2>&1 &
fi
# NO --ros-args remaps here. Those exist only to point the SIM's Gazebo topic
# names at what the agent expects; the real OAK-D already publishes the stereo
# topics, under VLA_NS=/robot1 exported in section 4.
# -u keeps output flowing promptly (§8.6). setsid gives the whole thing its own
# process group, so vla_kill.sh takes the tree next time. #63 keeps the agent's
# prints from aborting a command if its output ever goes away.

if ! wait_for_topic /vla/status 90 "the agent (/vla/status)"; then
    echo "  Agent did not come up."
    echo "  Its errors are in the AGENT WINDOW now, not in a file."
    [ -s "$LOG/agent.log" ] && { echo "  (headless-fallback log:)"; tail -30 "$LOG/agent.log"; }
    echo "  Mission log: ~/vla_logs/"
    exit 1
fi
# /vla/status is ABSOLUTE in the agent (STATUS_TOPIC): VLA_NS namespaces the
# robot's topics, not the agent's own. Same topic name in both modes.
# The lock is released by the EXIT trap above, so a bare exit is enough.

echo ""
echo "──────────────────────────────────────────"
echo " REAL ROBOT MODE — starting operator console"
echo "──────────────────────────────────────────"

python3 ~/vla_gui_v2.py
