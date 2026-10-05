#!/bin/bash
# hotspot_guard.sh -- keep the robot's 2.4 GHz hotspot channel for the robot.
# 22 Sep 2026. HOST tool (not a container). Nothing here touches the sim.
#
# Why: on 22 Sep a laptop on the PC's hotspot downloaded 131 MB through the
# PC's internet sharing. That hogs the 2.4 GHz channel; the camera's UDP frames
# are dropped and the GUI picture froze for 5-12 s at a time (agent log
# vla_run_20260922_081537.log: five stalls 08:17:25-08:18:10, ~40 s in total,
# with the Pi at 60 C and no DDS storm). The hotspot exists for the robot only.
#
#   bash tools/hotspot_guard.sh status       who is on the hotspot + 5 s throughput (no root)
#   sudo bash tools/hotspot_guard.sh apply   block internet for every client except the robot
#                                                  and kick the others off now (they may rejoin: harmless, no internet)
#   sudo bash tools/hotspot_guard.sh install apply + re-apply automatically whenever the hotspot
#                                                  comes up (NetworkManager dispatcher; survives reboots)
#   sudo bash tools/hotspot_guard.sh off     remove the block and the auto re-apply
#
# The block is an nftables table of its own (inet vla_hotspot) and leaves
# NetworkManager's own sharing rules alone. Only the robot (10.42.0.169,
# d8:3a:dd:1b:52:53) keeps internet through the PC -- it needs it for NTP.
# Other clients can still reach the PC itself (DHCP, DNS), they just get no
# internet, so they cannot download anything over the robot's channel.
set -u
IF=wlp0s20f3
ROBOT_IP=10.42.0.169
ROBOT_MAC=d8:3a:dd:1b:52:53
MARK=/run/vla_hotspot_guard.active          # tmpfs: gone after a reboot until the dispatcher re-applies
DISP=/etc/NetworkManager/dispatcher.d/90-vla-hotspot-guard

need_root() { [ "$EUID" -eq 0 ] || { echo "needs root:  sudo bash $0 $1"; exit 1; }; }

apply() {
  nft delete table inet vla_hotspot 2>/dev/null
  nft -f - <<NFT
table inet vla_hotspot {
  chain fwd {
    type filter hook forward priority 10; policy accept;
    iifname "$IF" ip saddr != $ROBOT_IP drop
    oifname "$IF" ip daddr != $ROBOT_IP drop
    iifname "$IF" meta nfproto ipv6 drop
    oifname "$IF" meta nfproto ipv6 drop
  }
}
NFT
  local m n=0
  for m in $(iw dev "$IF" station dump 2>/dev/null | awk -v r="$ROBOT_MAC" '/^Station/ && $2!=r {print $2}'); do
    iw dev "$IF" station del "$m" 2>/dev/null && n=$((n+1))
  done
  touch "$MARK"
  echo "hotspot guard ACTIVE on $IF: internet only for $ROBOT_IP; kicked $n other device(s)."
}

status() {
  local ips; ips=$(ip neigh show dev "$IF" 2>/dev/null | awk '{print $3"="$1}' | tr '\n' ' ')
  s() { iw dev "$IF" station dump 2>/dev/null | awk '/^Station/{m=$2} /signal:/{sg[m]=$2} /rx bytes/{r[m]=$3} /tx bytes/{t[m]=$3} END{for(k in r) print k, r[k], t[k], sg[k]}' | sort; }
  local A B; A=$(s); sleep 5; B=$(s)
  echo "devices on the hotspot ($IF), 5 s average:"
  [ -n "$A" ] || echo "  (none)"
  join <(echo "$A") <(echo "$B") | awk -v r="$ROBOT_MAC" -v ips="$ips" '
    BEGIN{n=split(ips,a," "); for(i=1;i<=n;i++){split(a[i],kv,"="); ip[kv[1]]=kv[2]}}
    {tag=($1==r)?"ROBOT   ":"STRANGER"; printf "  %s %s %-12s %4s dBm  from-device %6.1f kB/s  to-device %6.1f kB/s\n", tag, $1, ip[$1], $4, ($5-$2)/5/1024, ($6-$3)/5/1024}'
  if [ -f "$MARK" ]; then echo "guard: ACTIVE (strangers get no internet through the PC)"
  else echo "guard: NOT active  ->  sudo bash $0 install"; fi
}

# --- NetworkManager dispatcher mode: called as  <script> <interface> <action> ---
if [ $# -ge 2 ]; then
  [ "$1" = "$IF" ] || exit 0
  case "$2" in
    up)   apply ;;
    down) rm -f "$MARK" ;;
  esac
  exit 0
fi

case "${1:-status}" in
  status)  status ;;
  apply)   need_root apply; apply ;;
  install) need_root install
           install -m 755 -o root -g root "$(readlink -f "$0")" "$DISP"
           apply
           echo "installed $DISP: re-applied every time the hotspot comes up." ;;
  off)     need_root off
           nft delete table inet vla_hotspot 2>/dev/null; rm -f "$MARK" "$DISP"
           echo "hotspot guard OFF (block removed, auto re-apply removed)." ;;
  *)       sed -n '2,20p' "$0"; exit 1 ;;
esac
