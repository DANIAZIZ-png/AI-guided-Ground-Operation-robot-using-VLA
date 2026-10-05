#!/bin/bash
p=$(pgrep -f "[r]os2-daemon" | head -1)
if [ -z "$p" ]; then echo "NO DAEMON"; exit 1; fi
echo "daemon pid: $p"
tr '\0' '\n' < /proc/$p/environ | grep -E "^(ROS_DISCOVERY_SERVER|FASTRTPS_DEFAULT_PROFILES_FILE|ROS_DOMAIN_ID)" \
  && echo "==> daemon OK (robot mode)" || echo "==> DAEMON BLIND"
