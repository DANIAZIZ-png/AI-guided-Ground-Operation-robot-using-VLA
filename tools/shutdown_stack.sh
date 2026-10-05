#!/bin/bash
# Runs from inside ubuntu22-gpu. Sourcing robot_mode.sh first is what lets
# vla_kill.sh restart the ROS daemon in ROBOT mode instead of leaving it dead.
VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
source "$VLA_ROOT/scripts/robot_mode.sh"
echo "---- now stopping the stack ----"
$VLA_ROOT/scripts/vla_kill.sh
