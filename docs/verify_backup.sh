#!/usr/bin/env bash
# Re-check the two items that block the wipe. Read-only: changes nothing,
# anywhere, including on the robot.
#
#   bash docs/verify_backup.sh [/path/to/backup]
#
# Default destination: /media/$USER/USB/vla-backup
# See docs/BACKUP_CHECKLIST.md for the full audit.

set -uo pipefail

DEST="${1:-/media/$USER/USB/vla-backup}"
PI_HOST="${VLA_ROBOT_IP:-10.42.0.169}"
PI_USER="${VLA_ROBOT_USER:-ubuntu}"

GRN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; RST=$'\e[0m'
fail=0

ok()   { printf '  %sOK%s      %s\n' "$GRN" "$RST" "$*"; }
bad()  { printf '  %sMISSING%s %s\n' "$RED" "$RST" "$*"; fail=1; }
warn() { printf '  %sNOTE%s    %s\n' "$YEL" "$RST" "$*"; }

echo "=============================================================="
echo " pre-wipe backup re-check"
echo " destination: $DEST"
echo "=============================================================="

# ---------------------------------------------------------------------------
echo
echo "1. demo footage and deliverables copied off?"
# basename|source path -- the 28 Jul recording is deduplicated on purpose
ITEMS=(
  "open_house_final_v1.mp4|$HOME/Downloads/open_house_final_v1.mp4"
  "Demo_Explainer_Danyal_Aziz.mp4|$HOME/Downloads/Demo_Explainer_Danyal_Aziz.mp4"
  "recording-2026-07-28_21.21.52.mp4|$HOME/Videos/recording-2026-07-28_21.21.52.mp4"
  "recording-2026-09-30_13.03.49.mp4|$HOME/Videos/recording-2026-09-30_13.03.49.mp4"
  "recording-2026-09-30_13.01.37.mp4|$HOME/Videos/recording-2026-09-30_13.01.37.mp4"
  "FYDP_Report_Overleaf__2_.pdf|$HOME/Downloads/FYDP_Report_Overleaf__2_.pdf"
  "FYDP_Research_Paper_Overleaf.pdf|$HOME/Downloads/FYDP_Research_Paper_Overleaf.pdf"
  "FYDP_Poster_Danyal_Aziz.pptx|$HOME/Downloads/FYDP_Poster_Danyal_Aziz.pptx"
  "Open_House-99EC.zip|$HOME/Downloads/Open_House-99EC.zip"
)

if [ ! -d "$DEST" ]; then
    bad "the destination $DEST does not exist -- nothing has been copied yet"
else
    for entry in "${ITEMS[@]}"; do
        name="${entry%%|*}"; src="${entry#*|}"
        found=$(find "$DEST" -type f -name "$name" -print -quit 2>/dev/null)
        if [ -z "$found" ]; then
            bad "$name"
            continue
        fi
        # compare by content, not by name: a truncated copy is worse than none
        if [ -f "$src" ]; then
            if [ "$(sha256sum "$src" | cut -d' ' -f1)" = "$(sha256sum "$found" | cut -d' ' -f1)" ]; then
                ok "$name (sha256 matches the original)"
            else
                bad "$name is present but its sha256 DIFFERS from the original -- copy it again"
            fi
        else
            warn "$name present in the backup; the original is gone, cannot compare"
        fi
    done
fi

# ---------------------------------------------------------------------------
echo
echo "2. robot reachable, so the Pi configuration can be captured?"
# /dev/tcp, not ping: ping exits 2 with no output inside a container and is
# useless as a reachability test either way.
if timeout 8 bash -c "cat < /dev/null > /dev/tcp/$PI_HOST/22" 2>/dev/null; then
    ok "$PI_HOST:22 reachable -- run the capture block in docs/BACKUP_CHECKLIST.md §6"
    if [ -d "$(dirname "$0")/../robot_pi" ]; then
        ok "robot_pi/ already exists -- the capture appears to have been done"
    else
        bad "robot_pi/ does not exist yet; the Pi configuration has NOT been captured"
    fi
else
    bad "$PI_HOST:22 not reachable; the Pi configuration cannot be captured right now"
    iface_ip=$(ip -brief addr show 2>/dev/null | awk '/10\.42\.0\./ {print $3}' | head -1)
    if [ -z "$iface_ip" ]; then
        warn "no interface is on the 10.42.0.x subnet -- the hotspot is probably not running."
        warn "Current wireless address: $(ip -brief addr show wlp0s20f3 2>/dev/null | awk '{print $3}')"
        warn "Start the hotspot (2.4 GHz, channel 6) and power the robot, then re-run."
    fi
fi

# ---------------------------------------------------------------------------
echo
echo "3. repository has nothing project-related left uncommitted?"
R="$(cd "$(dirname "$0")/.." && pwd)"
if [ -n "$(git -C "$R" status --porcelain 2>/dev/null)" ]; then
    warn "the working tree is not clean:"
    git -C "$R" status --short 2>/dev/null | sed 's/^/          /' | head -10
else
    ok "working tree clean"
fi
unpushed=$(git -C "$R" log --oneline @{u}.. 2>/dev/null | wc -l)
if [ "$unpushed" -gt 0 ]; then
    bad "$unpushed commit(s) not pushed -- push before wiping"
else
    ok "all commits pushed"
fi

# ---------------------------------------------------------------------------
echo
echo "=============================================================="
if [ "$fail" -eq 0 ]; then
    printf '%s SAFE TO WIPE %s\n' "$GRN" "$RST"
else
    printf '%s NOT SAFE TO WIPE %s -- see the MISSING lines above\n' "$RED" "$RST"
fi
echo "=============================================================="
exit "$fail"
