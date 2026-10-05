#!/usr/bin/env bash
# Fetch the model weights into $VLA_MODEL_DIR and verify every sha256.
#
#   scripts/download_models.sh              # fetch what is missing, verify all
#   scripts/download_models.sh --verify     # verify only, download nothing
#   scripts/download_models.sh --force      # re-download even if present
#
# The hashes are the ones in env/MANIFEST.md, i.e. the exact weights that
# produced the results in results/. A file that downloads successfully but
# hashes differently is NOT accepted: it is moved aside and the script fails.
# Silently running on different weights would make every recorded number
# unreproducible without anything appearing to be wrong.
#
# The Ollama model is NOT downloaded here -- it lives in Ollama's own store, not
# a file. `--verify` checks its digest through the API if Ollama is reachable.

set -uo pipefail

VLA_ROOT="${VLA_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)}"
# shellcheck source=../config/paths.sh
. "$VLA_ROOT/config/paths.sh"

MODE=fetch
case "${1:-}" in
    --verify) MODE=verify ;;
    --force)  MODE=force ;;
    "")       ;;
    *) echo "usage: $0 [--verify|--force]" >&2; exit 2 ;;
esac

mkdir -p "$VLA_MODEL_DIR"

GRN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; RST=$'\e[0m'
fail=0

# Show a progress bar when a human is watching; stay silent when the output is
# redirected. A 1.5 GB download with curl's default meter writes thousands of
# carriage-return lines into a log file and makes it unreadable.
if [ -t 1 ]; then CURL_QUIET=(--progress-bar); else CURL_QUIET=(--no-progress-meter); fi

# name|sha256|url
# Whisper models are fetched by faster-whisper into its own HuggingFace cache on
# first use, so they are verified rather than placed here; see the note below.
MODELS=(
  "yolov8s-world.pt|095f5266bb9b654bd5ad9e21e9cdeda78e0f2c8460f5d652eaf04bab7ee251cf|https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-world.pt"
  "ViT-B-32.pt|40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af|https://openaipublic.azureedge.net/clip/models/40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt"
)

sha_of() { sha256sum "$1" | awk '{print $1}'; }

for entry in "${MODELS[@]}"; do
    IFS='|' read -r name want url <<< "$entry"
    dest="$VLA_MODEL_DIR/$name"

    if [ -f "$dest" ] && [ "$MODE" != force ]; then
        got=$(sha_of "$dest")
        if [ "$got" = "$want" ]; then
            printf '  %sOK%s      %-22s sha256 verified\n' "$GRN" "$RST" "$name"
            continue
        fi
        printf '  %sBAD%s     %-22s sha256 MISMATCH\n' "$RED" "$RST" "$name"
        printf '            want %s\n            got  %s\n' "$want" "$got"
        mv -f "$dest" "$dest.bad-$(date +%Y%m%d%H%M%S)"
        printf '            moved aside; re-run without --verify to fetch again\n'
        fail=1
        [ "$MODE" = verify ] && continue
    fi

    if [ "$MODE" = verify ]; then
        [ -f "$dest" ] || { printf '  %sMISSING%s %-22s (run without --verify)\n' "$YEL" "$RST" "$name"; fail=1; }
        continue
    fi

    printf '  fetching %s ...\n' "$name"
    if ! curl -fL "${CURL_QUIET[@]}" --retry 3 --retry-delay 2 -o "$dest.part" "$url"; then
        printf '  %sFAIL%s    %-22s download failed\n' "$RED" "$RST" "$name"
        rm -f "$dest.part"; fail=1; continue
    fi
    got=$(sha_of "$dest.part")
    if [ "$got" != "$want" ]; then
        printf '  %sFAIL%s    %-22s sha256 mismatch after download\n' "$RED" "$RST" "$name"
        printf '            want %s\n            got  %s\n' "$want" "$got"
        mv -f "$dest.part" "$dest.bad-$(date +%Y%m%d%H%M%S)"; fail=1; continue
    fi
    mv -f "$dest.part" "$dest"
    printf '  %sOK%s      %-22s downloaded and verified\n' "$GRN" "$RST" "$name"
done

# ---------------------------------------------------------------------------
# faster-whisper, pinned to an exact HuggingFace revision
# ---------------------------------------------------------------------------
# A model TAG is not a pin. faster-whisper resolves
# "Systran/faster-whisper-medium.en" to whatever main points at TODAY, so a
# rebuild after this machine is wiped would silently fetch a different model and
# every transcription measurement would stop being reproducible with nothing
# appearing to be wrong.
#
# So the revisions below are the commits that were actually used, read from
# ~/.cache/huggingface/hub/models--Systran--faster-whisper-*/refs/main, and every
# file is fetched BY COMMIT and hash-checked against env/MANIFEST.md.
#
# Downloaded into $VLA_MODEL_DIR/whisper/<model>/ rather than the HuggingFace
# cache, so the pin is visible and portable. Point VLA_WHISPER_DIR at it, or set
# HF_HUB_OFFLINE=1 with a populated cache.
echo
echo "faster-whisper (pinned revisions):"

WHISPER_DIR="${VLA_WHISPER_DIR:-$VLA_MODEL_DIR/whisper}"

# model|revision|file:sha256|file:sha256...
WHISPER=(
  "medium.en|a29b04bd15381511a9af671baec01072039215e3|model.bin:11b220779aea4c6f3ce9d2549c8a95ea869ed84066864b999531ef53e594fe5b|config.json:4a1848ebabe7938d9797c15a2e8e4ce1d36e6fd4a43d096ae5955257c67c7962|tokenizer.json:929c5252409436dce1b38a75d1abbcb5e132d170d8e324e4e04ed915fa2d22df|vocabulary.txt:ff77588746d3a2595d32ab5b69ffd7b95ce2441ac57533cb66fc3eb575a115cf"
  "small.en|d1d751a5f8271d482d14ca55d9e2deeebbae577f|model.bin:62b2a45b05ee59acb4a5341b33ee35e041395d378d418a18acfe4c9e768ee37a|config.json:666a9605530ac1f61fa8177f3702b4dacec9966749e42610839fcc32661d5fae|tokenizer.json:929c5252409436dce1b38a75d1abbcb5e132d170d8e324e4e04ed915fa2d22df|vocabulary.txt:ff77588746d3a2595d32ab5b69ffd7b95ce2441ac57533cb66fc3eb575a115cf"
)

for spec in "${WHISPER[@]}"; do
    IFS='|' read -r -a parts <<< "$spec"
    model="${parts[0]}"; rev="${parts[1]}"
    dest="$WHISPER_DIR/$model"
    mkdir -p "$dest"
    printf '  %s  (revision %s)\n' "$model" "${rev:0:12}"

    for fh in "${parts[@]:2}"; do
        name="${fh%%:*}"; want="${fh#*:}"
        out="$dest/$name"

        if [ -f "$out" ] && [ "$MODE" != force ]; then
            got=$(sha_of "$out")
            if [ "$got" = "$want" ]; then
                printf '    %sOK%s      %-16s sha256 verified\n' "$GRN" "$RST" "$name"
                continue
            fi
            printf '    %sBAD%s     %-16s sha256 MISMATCH -- wrong revision?\n' "$RED" "$RST" "$name"
            printf '              want %s\n              got  %s\n' "$want" "$got"
            mv -f "$out" "$out.bad-$(date +%Y%m%d%H%M%S)"
            fail=1
            [ "$MODE" = verify ] && continue
        fi

        if [ "$MODE" = verify ]; then
            [ -f "$out" ] || { printf '    %sMISSING%s %-16s (run without --verify)\n' "$YEL" "$RST" "$name"; fail=1; }
            continue
        fi

        # resolve/<revision>/ pins the commit. resolve/main would not.
        url="https://huggingface.co/Systran/faster-whisper-$model/resolve/$rev/$name"
        printf '    fetching %s ...\n' "$name"
        if ! curl -fL "${CURL_QUIET[@]}" --retry 3 --retry-delay 2 -o "$out.part" "$url"; then
            printf '    %sFAIL%s    %-16s download failed\n' "$RED" "$RST" "$name"
            rm -f "$out.part"; fail=1; continue
        fi
        got=$(sha_of "$out.part")
        if [ "$got" != "$want" ]; then
            printf '    %sFAIL%s    %-16s sha256 mismatch after download\n' "$RED" "$RST" "$name"
            printf '              want %s\n              got  %s\n' "$want" "$got"
            mv -f "$out.part" "$out.bad-$(date +%Y%m%d%H%M%S)"; fail=1; continue
        fi
        mv -f "$out.part" "$out"
        printf '    %sOK%s      %-16s downloaded and verified\n' "$GRN" "$RST" "$name"
    done
done

# The HuggingFace cache, if faster-whisper has already populated it. Blobs there
# are content-addressed, so a blob's filename IS its sha256 -- its presence under
# the expected name is the verification.
HF="${HF_HOME:-$HOME/.cache/huggingface}/hub"
for spec in "medium.en|11b220779aea4c6f3ce9d2549c8a95ea869ed84066864b999531ef53e594fe5b|a29b04bd15381511a9af671baec01072039215e3" \
            "small.en|62b2a45b05ee59acb4a5341b33ee35e041395d378d418a18acfe4c9e768ee37a|d1d751a5f8271d482d14ca55d9e2deeebbae577f"; do
    IFS='|' read -r model blob rev <<< "$spec"
    base="$HF/models--Systran--faster-whisper-$model"
    if [ -f "$base/blobs/$blob" ]; then
        cached_rev=$(cat "$base/refs/main" 2>/dev/null || echo "?")
        if [ "$cached_rev" = "$rev" ]; then
            printf '  %sOK%s      HF cache %-10s revision matches\n' "$GRN" "$RST" "$model"
        else
            printf '  %sWARN%s    HF cache %-10s is at revision %s, expected %s\n' \
                   "$YEL" "$RST" "$model" "${cached_rev:0:12}" "${rev:0:12}"
            printf '            The model.bin still hashes correctly, so the weights are\n'
            printf '            right; only the ref moved. Not treated as a failure.\n'
        fi
    fi
done

# ---------------------------------------------------------------------------
# Ollama
# ---------------------------------------------------------------------------
echo
echo "Ollama model:"
OLLAMA_HOST="${VLA_OLLAMA_HOST:-http://127.0.0.1:11434}"
WANT_MODEL="${VLA_OLLAMA_MODEL:-qwen2.5:7b}"
WANT_DIGEST=845dbda0ea48

if tags=$(curl -fsS --max-time 8 "$OLLAMA_HOST/api/tags" 2>/dev/null); then
    got=$(printf '%s' "$tags" | python3 -c '
import json,sys
want = sys.argv[1]
for m in json.load(sys.stdin).get("models", []):
    if m.get("name") == want:
        print(m.get("digest","")[:12]); break
' "$WANT_MODEL" 2>/dev/null)
    if [ -z "$got" ]; then
        printf '  %sMISSING%s %-22s not pulled. Run: ollama pull %s\n' \
               "$YEL" "$RST" "$WANT_MODEL" "$WANT_MODEL"
        fail=1
    elif [ "$got" = "$WANT_DIGEST" ]; then
        printf '  %sOK%s      %-22s digest %s matches\n' "$GRN" "$RST" "$WANT_MODEL" "$got"
    else
        printf '  %sBAD%s     %-22s digest %s, expected %s\n' \
               "$RED" "$RST" "$WANT_MODEL" "$got" "$WANT_DIGEST"
        printf '            The recorded results used %s. A different build of the\n' "$WANT_DIGEST"
        printf '            same tag can answer differently.\n'
        fail=1
    fi
else
    printf '  %s--%s      Ollama not reachable at %s; skipped\n' "$YEL" "$RST" "$OLLAMA_HOST"
fi

echo
if [ "$fail" -eq 0 ]; then
    printf '%sAll model checks passed.%s  %s\n' "$GRN" "$RST" "$VLA_MODEL_DIR"
else
    printf '%sSome model checks failed (see above).%s\n' "$RED" "$RST"
fi
exit "$fail"
