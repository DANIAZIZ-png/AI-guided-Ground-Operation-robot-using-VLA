#!/usr/bin/env bash
# Verify a GitHub Release's assets against the local backup folder.
#
#   bash docs/verify_release.sh [tag] [local-folder]
#
# Defaults: tag v1.1-media, folder ~/vla_media_backup
#
# Compares the SIZE GitHub reports for each asset against the local file, using
# the public releases API -- no token needed for a public repo, and it uploads
# nothing. Read-only.
#
# WHY SIZE AND NOT sha256
#   GitHub does not publish a checksum for release assets, so size is the only
#   field that can be compared without downloading. A size match catches the
#   failure that actually happens: a browser upload that was interrupted and
#   left a truncated asset. To check content as well, pass --download, which
#   fetches each asset and compares its sha256 against MANIFEST.txt.

set -uo pipefail

TAG="${1:-v1.1-media}"
DIR="${2:-$HOME/vla_media_backup}"
REPO="${VLA_GH_REPO:-DANIAZIZ-png/AI-guided-Ground-Operation-robot-using-VLA}"
DOWNLOAD=0
for a in "$@"; do [ "$a" = "--download" ] && DOWNLOAD=1; done

GRN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; RST=$'\e[0m'
fail=0

echo "=============================================================="
echo " release:      $REPO @ $TAG"
echo " local folder: $DIR"
echo "=============================================================="

[ -d "$DIR" ] || { echo "${RED}local folder $DIR does not exist${RST}"; exit 1; }

API="https://api.github.com/repos/$REPO/releases/tags/$TAG"
json=$(curl -sS --max-time 30 "$API" 2>/dev/null) || {
    echo "${RED}could not reach the GitHub API${RST}"; exit 1; }

if printf '%s' "$json" | grep -q '"message": *"Not Found"'; then
    echo "${RED}no release found with tag $TAG${RST}"
    echo "Create it first -- see docs/BACKUP_CHECKLIST.md §2."
    exit 1
fi

# name<TAB>size<TAB>url, one asset per line.
# The parser is a separate file on purpose: the first version of this inlined a
# Python f-string inside a single-quoted shell argument, the nested quoting broke,
# python exited non-zero, and the script reported "0 assets" for a release that
# actually had 17. A silent parse failure that reads as "nothing was uploaded" is
# the worst possible way for this to fail.
PARSER="$(dirname "$0")/release_assets.py"
[ -f "$PARSER" ] || { echo "${RED}missing $PARSER${RST}"; exit 1; }
assets=$(printf '%s' "$json" | python3 "$PARSER") || {
    echo "${RED}could not parse the release JSON${RST}"; exit 1; }

n_assets=$(printf '%s' "$assets" | grep -c . || true)
echo "assets on the release: $n_assets"
echo

# ---- every local file must be on the release, at the right size ----
while IFS= read -r f; do
    [ -f "$DIR/$f" ] || continue
    want=$(stat -c%s "$DIR/$f")
    line=$(printf '%s' "$assets" | awk -F'\t' -v n="$f" '$1==n {print; exit}')
    if [ -z "$line" ]; then
        printf '  %sMISSING%s %-52s not uploaded\n' "$RED" "$RST" "$f"
        fail=1
        continue
    fi
    got=$(printf '%s' "$line" | cut -f2)
    if [ "$got" = "$want" ]; then
        printf '  %sOK%s      %-52s %s bytes\n' "$GRN" "$RST" "$f" "$got"
    else
        printf '  %sSIZE%s    %-52s release %s, local %s\n' "$RED" "$RST" "$f" "$got" "$want"
        printf '          a truncated upload -- delete that asset and re-upload it\n'
        fail=1
        continue
    fi

    if [ "$DOWNLOAD" = 1 ]; then
        url=$(printf '%s' "$line" | cut -f3)
        tmp=$(mktemp)
        if curl -fL --no-progress-meter --max-time 900 -o "$tmp" "$url" 2>/dev/null; then
            a=$(sha256sum "$tmp" | cut -d' ' -f1)
            b=$(sha256sum "$DIR/$f" | cut -d' ' -f1)
            if [ "$a" = "$b" ]; then
                printf '          %ssha256 matches%s\n' "$GRN" "$RST"
            else
                printf '          %ssha256 DIFFERS%s\n' "$RED" "$RST"; fail=1
            fi
        else
            printf '          %scould not download to verify%s\n' "$YEL" "$RST"
        fi
        rm -f "$tmp"
    fi
done < <(find "$DIR" -maxdepth 1 -type f -printf '%f\n' | grep -v '^MANIFEST.txt$' | sort)

# ---- anything on the release that is not local ----
extra=$(printf '%s' "$assets" | cut -f1 | while IFS= read -r n; do
            [ -n "$n" ] && [ ! -f "$DIR/$n" ] && echo "$n"; done)
if [ -n "$extra" ]; then
    echo
    echo "on the release but not in the local folder (fine, just noting):"
    printf '%s\n' "$extra" | sed 's/^/    /'
fi

echo
echo "=============================================================="
if [ "$fail" -eq 0 ]; then
    printf '%s ALL FILES PRESENT AND THE RIGHT SIZE %s\n' "$GRN" "$RST"
    echo " Tick the release row in docs/BACKUP_CHECKLIST.md §2."
    [ "$DOWNLOAD" = 0 ] && echo " For a content check too: bash docs/verify_release.sh $TAG $DIR --download"
else
    printf '%s SOME FILES ARE MISSING OR THE WRONG SIZE %s\n' "$RED" "$RST"
fi
echo "=============================================================="
exit "$fail"
