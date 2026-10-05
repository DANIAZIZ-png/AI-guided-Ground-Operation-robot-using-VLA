#!/bin/bash
# =============================================================================
#  vla_demo_stop.sh  --  shut the hardware demo down and put the robot back
#  on its charger.  Written 11 Sep 2026.  HARDWARE ONLY.  Run on the HOST.
#
#  Order matters:
#    1  stop the agent, voice node and GUI first -- nothing may publish
#       velocities while the Dock action drives the robot
#    2  report dock status and remind you to DOCK BY HAND (auto-redock removed
#       21 Sep: the Create 3 Dock action aborted ~half the time).
#    3  vla_tools/shutdown_stack.sh (robot_mode.sh + vla_kill.sh, by path):
#       kills SLAM, Nav2, RViz and restarts the ROS daemon in robot mode
#    4  the YOLO server (vla_kill.sh does not cover it), leftover xterms
#
#  RUN IT AS  ~/vla_demo_stop.sh  from a plain terminal. Never inline it in a
#  command whose text names a ROS process (rviz2, ros2 launch, vla_agent ...):
#  vla_kill.sh group-kills any shell whose command line contains those words,
#  and that includes the shell running this script (CLAUDE.md).
#
#  Options:   --no-dock   skip step 2 (robot already docked by hand, or you
#                         want it left where it is)
# =============================================================================
set -u
VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
source "$VLA_ROOT/config/paths.sh"
T="$VLA_TOOLS_DIR"
LOGDIR="$VLA_LOG_DIR"
PI=${VLA_ROBOT_IP:-10.42.0.169}
NODOCK=0; [ "${1:-}" = "--no-dock" ] && NODOCK=1
BOLD=$'\e[1m'; GRN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; CYN=$'\e[36m'; RST=$'\e[0m'
say()  { printf '%s[%s]%s %s\n' "$CYN" "$(date +%T)" "$RST" "$*"; }
warn() { printf '%s[%s] WARNING:%s %s\n' "$YEL" "$(date +%T)" "$RST" "$*"; }
ros()  { distrobox enter ubuntu22-gpu -- bash -c "source $VLA_ROOT/config/robot.env; $1" < /dev/null 2>&1; }   # < /dev/null: no pty, see vla_demo.sh
# kill every process whose command line matches PATTERN (bracket trick keeps
# this script's own line out of the match), INT first, then KILL.
stop_pat() {
  local pat=$1 p
  for p in $(pgrep -f "$pat"); do kill -INT "$p" 2>/dev/null; done
  sleep 3
  for p in $(pgrep -f "$pat"); do kill -9 "$p" 2>/dev/null; done
}

say "${BOLD}VLA demo shutdown${RST}"

# ---- 1. operator-side processes first -----------------------------------
say "1/4  stopping the agent, voice node and GUI"
stop_pat "[v]la_agent_v28.py"
stop_pat "[v]oice_command.py"
stop_pat "[v]la_gui_v2.py"
for p in $(pgrep -f "[x]term .*-T (VLA AGENT|VOICE hw)"); do kill "$p" 2>/dev/null; done
say "     agent=$(pgrep -c -f '[v]la_agent_v28') voice=$(pgrep -c -f '[v]oice_command') gui=$(pgrep -c -f '[v]la_gui_v2')  (all should be 0)"

# ---- 2. dock by hand -----------------------------------------------------
# 21 Sep: automatic re-dock REMOVED. The Create 3 Dock action aborted on
# 10/17/21 Sep (about half its attempts), and after a SLAM pose drift it drove
# Nav2 to a wrong staging pose. Docking by hand is more reliable for the demo.
# (The --no-dock flag is now the only behaviour; redock.py is still on disk if
# ever wanted manually.)
DOCKED=$(ros "timeout 15 ros2 topic echo /robot1/dock_status --once" | awk '/is_docked/{print $2}')
CUR=$(ros "timeout 15 ros2 topic echo /robot1/battery_state --once" | awk '/^current/{print $2}')
if [ "$DOCKED" = "true" ]; then
  printf '%s2/4  robot is DOCKED (current %s A) — good.%s\n' "$GRN" "${CUR:-?}" "$RST"
else
  printf '\n%s%s2/4  robot is OFF the charger (is_docked=%s). Please DOCK IT BY HAND now.%s\n\n' "$YEL" "$BOLD" "${DOCKED:-?}" "$RST"
fi

# ---- 3. the ROS stack ----------------------------------------------------
say "3/4  stopping SLAM, Nav2, RViz; restarting the daemon in robot mode"
distrobox enter ubuntu22-gpu -- bash "$T/shutdown_stack.sh" < /dev/null 2>&1 | tail -3 | sed 's/^/     /'   # the daemon vla_kill.sh restarts must outlive this call
for p in $(pgrep -f "[c]omponent_container_isolated"); do kill -9 "$p" 2>/dev/null; done

# ---- 4. YOLO and windows -------------------------------------------------
say "4/4  stopping the YOLO server and leftover windows"
for p in $(pgrep -f "[y]olo_server.py"); do kill -9 "$p" 2>/dev/null; done
sleep 1
for p in $(pgrep -f "[x]term .*-T (SLAM hw|NAV2 hw|NAV2 hw composed|RVIZ hw|VLA AGENT|VOICE hw)"); do kill "$p" 2>/dev/null; done

echo
say "left running: slam=$(pgrep -c -f '[s]lam_toolbox') nav2=$(pgrep -c -f '[c]omponent_container_isolated|[b]t_navigator') rviz=$(pgrep -c -f '[r]viz2 -d') agent=$(pgrep -c -f '[v]la_agent') yolo=$(pgrep -c -f '[y]olo_server') gui=$(pgrep -c -f '[v]la_gui') voice=$(pgrep -c -f '[v]oice_command')   (all 0 = clean)"
say "ollama left running on purpose (model stays pinned)."
B=$(ros "timeout 15 ros2 topic echo /robot1/battery_state --once" | awk '/^percentage/{printf "%d %%", $2*100} /^current/{c=$2} END{printf "  current %s A", c}')
say "robot: battery $B"
exit 0
