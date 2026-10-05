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
    if ! curl -fL --retry 3 --retry-delay 2 -o "$dest.part" "$url"; then
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
# faster-whisper models
# ---------------------------------------------------------------------------
# These are pulled by faster-whisper into ~/.cache/huggingface on first use, as
# repository snapshots rather than single files, so they are verified where they
# land instead of being placed in VLA_MODEL_DIR.
echo
echo "faster-whisper (HuggingFace cache):"
HF="${HF_HOME:-$HOME/.cache/huggingface}/hub"
for spec in "medium.en|11b220779aea4c6f3ce9d2549c8a95ea869ed84066864b999531ef53e594fe5b" \
            "small.en|62b2a45b05ee59acb4a5341b33ee35e041395d378d418a18acfe4c9e768ee37a"; do
    IFS='|' read -r model want <<< "$spec"
    blob="$HF/models--Systran--faster-whisper-$model/blobs/$want"
    if [ -f "$blob" ]; then
        printf '  %sOK%s      faster-whisper-%-10s present, content-addressed as its sha256\n' \
               "$GRN" "$RST" "$model"
    else
        printf '  %s--%s      faster-whisper-%-10s not cached; it downloads on first transcription\n' \
               "$YEL" "$RST" "$model"
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
