#!/bin/bash
for p in $(pgrep -f "[v]la_agent_v28.py"); do kill -INT $p 2>/dev/null; done; sleep 4
for p in $(pgrep -f "[v]la_agent_v28.py"); do kill -9 $p 2>/dev/null; done
for p in $(pgrep -f "[x]term .*-T VLA AGENT"); do kill $p 2>/dev/null; done; sleep 1
( cd "$VLA_ROOT" && nohup distrobox enter ubuntu22-gpu -- setsid xterm -iconic -hold -geometry 120x40 -T "VLA AGENT" -e bash $VLA_TOOLS_DIR/agent_xterm.sh >/dev/null 2>&1 & )
sleep 35
echo "agent running: $(pgrep -c -f '[v]la_agent_v28.py')"
