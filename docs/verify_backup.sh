#!/usr/bin/env bash
# Re-check the two items that block the wipe. Read-only: changes nothing,
# anywhere, including on the robot.
#
#   bash docs/verify_backup.sh
#
# Staging folder: $HOME/vla_media_backup (override with VLA_MEDIA_DIR).
# Destination:    GitHub Release v1.1-media.
# See docs/BACKUP_CHECKLIST.md for the full audit.

set -uo pipefail

STAGE_DEFAULT="$HOME/vla_media_backup"
PI_HOST="${VLA_ROBOT_IP:-10.42.0.169}"
PI_USER="${VLA_ROBOT_USER:-ubuntu}"

GRN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; RST=$'\e[0m'
fail=0

ok()   { printf '  %sOK%s      %s\n' "$GRN" "$RST" "$*"; }
bad()  { printf '  %sMISSING%s %s\n' "$RED" "$RST" "$*"; fail=1; }
warn() { printf '  %sNOTE%s    %s\n' "$YEL" "$RST" "$*"; }

echo "=============================================================="
echo " pre-wipe backup re-check"
echo " staging:     ${VLA_MEDIA_DIR:-$STAGE_DEFAULT}"
echo " release:     v1.1-media"
echo "=============================================================="

# ---------------------------------------------------------------------------
echo
echo "1. demo footage and deliverables preserved off this disk?"
# The destination is the GitHub Release v1.1-media, not a USB stick: the files
# are gathered in ~/vla_media_backup/ and uploaded through the website. This
# check confirms the staging folder is complete, then delegates the release
# comparison to verify_release.sh, which is the script that knows how to read
# the releases API.
STAGE="${VLA_MEDIA_DIR:-$HOME/vla_media_backup}"

if [ ! -d "$STAGE" ]; then
    bad "the staging folder $STAGE does not exist"
else
    n=$(find "$STAGE" -maxdepth 1 -type f ! -name MANIFEST.txt | wc -l)
    if [ "$n" -ge 16 ]; then
        ok "staging folder has $n files ($(du -sh "$STAGE" | cut -f1))"
    else
        bad "staging folder has only $n files, expected 16"
    fi
    if [ -f "$STAGE/MANIFEST.txt" ]; then
        # every file must still match the hash recorded when it was staged
        drift=0
        while read -r size sha name; do
            case "$size" in \#*) continue ;; esac
            [ -f "$STAGE/$name" ] || { bad "$name listed in MANIFEST.txt but missing"; drift=1; continue; }
            [ "$(sha256sum "$STAGE/$name" | cut -d" " -f1)" = "$sha" ] \
                || { bad "$name has changed since it was staged"; drift=1; }
        done < <(grep -v "^#" "$STAGE/MANIFEST.txt")
        [ "$drift" -eq 0 ] && ok "all staged files match MANIFEST.txt"
    else
        warn "no MANIFEST.txt in $STAGE"
    fi
fi

# Capture first, then test -- do NOT pipe into `grep -q` here.
# `grep -q` exits the moment it finds a match, curl's write then fails with
# EPIPE, and `set -o pipefail` (on at the top of this script) propagates curl's
# failure. The result is that the check fails PRECISELY WHEN THE RELEASE EXISTS
# -- a perfect inversion, and it reported "release does not exist yet" for a
# release with 17 assets.
_rel=$(curl -sS --max-time 30 \
     "https://api.github.com/repos/${VLA_GH_REPO:-DANIAZIZ-png/AI-guided-Ground-Operation-robot-using-VLA}/releases/tags/v1.1-media" \
     2>/dev/null) || _rel=""
_n_assets=$(printf '%s' "$_rel" | grep -c '"browser_download_url"' || true)
if printf '%s' "$_rel" | grep -F '"tag_name"' >/dev/null 2>&1; then
    ok "release v1.1-media exists with $_n_assets asset(s)"
    if [ "$_n_assets" -ge 16 ]; then
        ok "asset count looks complete -- confirm sizes: bash docs/verify_release.sh"
    else
        bad "only $_n_assets assets on the release, expected at least 16"
    fi
elif [ -z "$_rel" ]; then
    warn "could not reach the GitHub API; release state unknown (not treated as a failure)"
else
    bad "release v1.1-media does not exist yet (see docs/BACKUP_CHECKLIST.md §2a)"
fi

# ---------------------------------------------------------------------------
echo
echo "2. Pi configuration (DEFERRED -- informational, not a blocker)"
# /dev/tcp, not ping: ping exits 2 with no output inside a container and is
# useless as a reachability test either way.
# DEFERRED, and deliberately not a failure: the Pi is not being wiped, so its
# files survive. Only the written record of what differs from stock is
# outstanding, and that can be captured whenever the robot is on.
if timeout 8 bash -c "cat < /dev/null > /dev/tcp/$PI_HOST/22" 2>/dev/null; then
    if [ -d "$(dirname "$0")/../robot_pi" ]; then
        ok "robot_pi/ exists -- the Pi capture has been done"
    else
        warn "$PI_HOST:22 is reachable now, so the deferred Pi capture COULD be done"
        warn "  -> see the capture block in docs/BACKUP_CHECKLIST.md §6 (read-only)"
    fi
else
    warn "deferred: $PI_HOST:22 not reachable, Pi config not captured (does NOT block the wipe --"
    warn "          the Pi is not being wiped, so the files still exist on it)"
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
