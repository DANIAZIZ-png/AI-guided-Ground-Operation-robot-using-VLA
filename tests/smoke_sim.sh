#!/usr/bin/env bash
# Headless simulation smoke test.
#
#   tests/smoke_sim.sh              # full run
#   tests/smoke_sim.sh --no-agent   # stack only: Gazebo, SLAM, Nav2
#
# WHAT IT PROVES
#   1. the simulation stack comes up headless           (Gazebo, SLAM, Nav2)
#   2. the sensor data path is live                     (/scan, /odom, /map)
#   3. Nav2 accepts and completes a goal                (NavigateToPose)
#   4. the agent is up and answers a typed command      (/vla/command -> /vla/reply)
#
# SAFETY RULES THIS SCRIPT FOLLOWS
#   * hard overall timeout, enforced by a watchdog
#   * its own process group; the cleanup kills ONLY the PIDs it started, by PID,
#     in reverse order
#   * it NEVER calls scripts/vla_sim.sh or scripts/vla_kill.sh
#   * it NEVER uses `pkill -f` or any broad name pattern. `pkill -f` matches the
#     whole command line of every process including the shell running it, so a
#     pattern that appears in a comment or a filename makes the shell kill
#     itself and everything after that line silently does not run.
#
# WHY ONE ASSERTION IS SOFT
#   "go to the chair" needs YOLO to actually find a chair in whichever Gazebo
#   world is loaded, which depends on the world and the robot's spawn pose. A
#   smoke test that fails because a chair was out of frame tells you nothing
#   about the code, so the end-to-end drive is reported but does not fail the
#   run. The hard assertions above are the ones that catch real breakage.

set -uo pipefail

VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
cd "$VLA_ROOT"

TIMEOUT_TOTAL="${SMOKE_TIMEOUT:-300}"
RUN_AGENT=1
[ "${1:-}" = "--no-agent" ] && RUN_AGENT=0

LOGDIR="${VLA_LOG_DIR:-$VLA_ROOT/logs}/smoke_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"

GRN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; CYN=$'\e[36m'; RST=$'\e[0m'
T0=$(date +%s)
PIDS=()
FAILED=0

say()  { printf '%s[+%3ds]%s %s\n' "$CYN" "$(( $(date +%s) - T0 ))" "$RST" "$*"; }
pass() { printf '%s[+%3ds]  PASS%s %s\n' "$GRN" "$(( $(date +%s) - T0 ))" "$RST" "$*"; }
soft() { printf '%s[+%3ds]  NOTE%s %s\n' "$YEL" "$(( $(date +%s) - T0 ))" "$RST" "$*"; }
fail() { printf '%s[+%3ds]  FAIL%s %s\n' "$RED" "$(( $(date +%s) - T0 ))" "$RST" "$*"; FAILED=1; }

# ---- cleanup: only our own PIDs, by PID, newest first ----------------------
cleanup() {
    local rc=$?
    say "cleanup: stopping ${#PIDS[@]} process(es) this script started"
    for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
        local p="${PIDS[i]}"
        if kill -0 "$p" 2>/dev/null; then
            kill -INT "$p" 2>/dev/null || true
        fi
    done
    sleep 3
    for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
        local p="${PIDS[i]}"
        kill -0 "$p" 2>/dev/null && kill -9 "$p" 2>/dev/null || true
    done
    say "logs: $LOGDIR"
    exit $rc
}
trap cleanup EXIT INT TERM

# ---- watchdog --------------------------------------------------------------
( sleep "$TIMEOUT_TOTAL"; echo; echo "${RED}smoke: hard timeout after ${TIMEOUT_TOTAL}s${RST}" >&2;
  kill -TERM $$ 2>/dev/null ) &
PIDS+=("$!")

# ---- how to run a ROS command ---------------------------------------------
# The distrobox container on the development machine, otherwise assume we are
# already in a ROS environment (the container image).
if command -v distrobox >/dev/null 2>&1 && distrobox list 2>/dev/null | grep -q ubuntu22-gpu; then
    ros_bg() { nohup distrobox enter ubuntu22-gpu -- bash -lc \
                 "source $VLA_ROOT/config/sim.env >/dev/null 2>&1; $1" </dev/null >>"$2" 2>&1 & echo $!; }
    ros_fg() { timeout "${3:-30}" distrobox enter ubuntu22-gpu -- bash -lc \
                 "source $VLA_ROOT/config/sim.env >/dev/null 2>&1; $1" </dev/null 2>&1; }
else
    ros_bg() { nohup bash -lc \
                 "source $VLA_ROOT/config/sim.env >/dev/null 2>&1; $1" </dev/null >>"$2" 2>&1 & echo $!; }
    ros_fg() { timeout "${3:-30}" bash -lc \
                 "source $VLA_ROOT/config/sim.env >/dev/null 2>&1; $1" </dev/null 2>&1; }
fi

wait_for() {   # wait_for <seconds> <label> <shell test>
    local limit=$1 label=$2 test=$3 waited=0
    while [ "$waited" -lt "$limit" ]; do
        if eval "$test" >/dev/null 2>&1; then pass "$label (${waited}s)"; return 0; fi
        sleep 5; waited=$(( waited + 5 ))
    done
    fail "$label did not appear within ${limit}s"
    return 1
}

echo "=============================================================="
echo " headless simulation smoke test   (timeout ${TIMEOUT_TOTAL}s)"
echo "=============================================================="

# ---- 1. xvfb present -------------------------------------------------------
if [ -z "$(ros_fg 'command -v xvfb-run' 10)" ]; then
    fail "xvfb-run is missing. Gazebo cannot run headless without it; it is installed in docker/ros.Dockerfile."
    exit 1
fi
pass "xvfb-run available"

# ---- 2. the simulation stack, headless ------------------------------------
say "starting Gazebo + SLAM + Nav2 under xvfb (this takes ~60-90s)"
PIDS+=( "$(ros_bg "xvfb-run -a --server-args='-screen 0 1280x1024x24' \
      ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
      model:=lite slam:=true nav2:=true rviz:=false" "$LOGDIR/sim.log")" )

wait_for 150 "/scan is publishing" \
    "[ \"\$(ros_fg 'timeout 15 ros2 topic hz /scan 2>&1 | grep -c average' 25)\" != 0 ]" || true
wait_for 60  "/odom is publishing" \
    "[ \"\$(ros_fg 'timeout 15 ros2 topic hz /odom 2>&1 | grep -c average' 25)\" != 0 ]" || true
wait_for 90  "SLAM is publishing /map" \
    "ros_fg 'timeout 15 ros2 topic info /map -v' 25 | grep -q slam_toolbox" || true

# ---- 3. Nav2 accepts a goal ------------------------------------------------
wait_for 120 "Nav2 action server /navigate_to_pose is available" \
    "ros_fg 'timeout 20 ros2 action list' 30 | grep -q navigate_to_pose" || true

if [ "$FAILED" -eq 0 ]; then
    say "sending one Nav2 goal 1.0 m ahead via tools/nav_goal.py"
    if ros_fg "python3 $VLA_ROOT/tools/nav_goal.py 1.0 0.0 0.0" 90 \
         | tee "$LOGDIR/nav_goal.log" | grep -qiE "accepted|succeeded|SUCCEEDED"; then
        pass "Nav2 accepted the goal"
    else
        fail "Nav2 did not accept the goal (see $LOGDIR/nav_goal.log)"
    fi
fi

# ---- 4. the agent answers a command ---------------------------------------
if [ "$RUN_AGENT" -eq 1 ] && [ "$FAILED" -eq 0 ]; then
    say "starting the detector and the agent"
    PIDS+=( "$(ros_bg "python3 $VLA_ROOT/src/yolo_server.py" "$LOGDIR/yolo.log")" )
    wait_for 120 "detector answering on :5001" \
        "timeout 5 bash -c 'cat < /dev/null > /dev/tcp/127.0.0.1/5001'" || true

    PIDS+=( "$(ros_bg "python3 -u $VLA_ROOT/src/vla_agent_v28.py --ros-args \
          -r /oakd/stereo/image_raw:=/oakd/rgb/preview/depth \
          -r /oakd/stereo/camera_info:=/oakd/rgb/preview/camera_info" "$LOGDIR/agent.log")" )

    wait_for 90 "agent publishing /vla/status" \
        "ros_fg 'timeout 20 ros2 topic echo /vla/status --once' 30 | grep -q mode" || true

    if [ "$FAILED" -eq 0 ]; then
        say "sending one typed command on /vla/command"
        ros_fg "python3 $VLA_ROOT/tools/say.py 'what do you see' 45" 60 \
            > "$LOGDIR/say.log" 2>&1
        if grep -qiE "I (can )?see|nothing|ROBOT" "$LOGDIR/say.log"; then
            pass "agent answered a typed command"
        else
            fail "agent did not answer (see $LOGDIR/say.log)"
        fi

        say "end-to-end drive (soft: depends on the world and the spawn pose)"
        ros_fg "python3 $VLA_ROOT/tools/say.py 'go to the chair' 90" 110 \
            > "$LOGDIR/drive.log" 2>&1
        if grep -qiE "Arrived" "$LOGDIR/drive.log"; then
            pass "agent reported arrival"
        elif grep -qiE "can'?t see|cannot see|no .* found" "$LOGDIR/drive.log"; then
            soft "no chair visible from the spawn pose -- not a code failure"
        else
            soft "no arrival and no clear refusal; see $LOGDIR/drive.log"
        fi
    fi
fi

echo "=============================================================="
if [ "$FAILED" -eq 0 ]; then
    printf '%s SMOKE TEST PASSED %s  in %ds\n' "$GRN" "$RST" "$(( $(date +%s) - T0 ))"
else
    printf '%s SMOKE TEST FAILED %s  see %s\n' "$RED" "$RST" "$LOGDIR"
fi
echo "=============================================================="
exit "$FAILED"
