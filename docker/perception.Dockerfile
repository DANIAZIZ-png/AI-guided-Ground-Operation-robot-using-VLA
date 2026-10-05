# syntax=docker/dockerfile:1
#
# The open-vocabulary detector: YOLOv8s-World + CLIP, served over HTTP.
#
#   podman build -f docker/perception.Dockerfile -t vla-perception:1.0 .
#   docker build  -f docker/perception.Dockerfile -t vla-perception:1.0 .
#
# Build context is the REPOSITORY ROOT.
#
# WHAT THIS FIXES
#   env/MANIFEST.md documents the most fragile thing in the whole setup:
#   yolo_server.py ran INSIDE the Ubuntu 22.04 vla-box container but activated
#   the HOST virtualenv ~/yolo-env, whose python3 symlinks to
#   /usr/bin/python3.12 -- a binary that does not exist in 22.04. It worked only
#   because $HOME was bind-mounted into the container, so the host interpreter
#   was reachable from inside. Nothing about that survives being rebuilt, and it
#   would not survive this PC being wiped.
#
#   This image is Python 3.12 in its own right, with every dependency pinned
#   from env/pip-freeze_yolo-env.txt. No host venv, no bind mount, no shared
#   interpreter.
#
# GPU
#   Needs the NVIDIA Container Toolkit. Under Podman that means CDI:
#       podman run --device nvidia.com/gpu=all ...
#   Under Docker:
#       docker run --gpus all ...
#   compose.yaml sets this up. Without a GPU the server still starts and still
#   answers, on the CPU, slowly -- useful for a smoke test, useless for a demo.

FROM python:3.12-slim

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# git: the CLIP pin is a git reference, see the requirements file.
# libgl1 / libglib2.0-0: opencv-python's runtime shared libraries. Without them
# `import cv2` fails with "libGL.so.1: cannot open shared object file", which
# looks like a Python problem and is not.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        libgl1 \
        libglib2.0-0 \
        curl \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Dependencies, exactly as recorded
# ---------------------------------------------------------------------------
# One source of truth: this is env/pip-freeze_yolo-env.txt verbatim -- 64
# packages including torch 2.12.1, torchvision 0.27.1, ultralytics 8.4.82,
# numpy 2.5.0, opencv-python 4.13.0.92, Flask 3.1.3 and CLIP pinned to commit
# 16be45c7062240d445cce764f2afd9454a91ef7e.
#
# It is NOT re-pinned here. A second copy of the versions would drift from the
# manifest, and the manifest is what ties the recorded results to the code.
COPY env/pip-freeze_yolo-env.txt /tmp/requirements.txt

RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm -f /tmp/requirements.txt

# ---------------------------------------------------------------------------
# The detector
# ---------------------------------------------------------------------------
WORKDIR /opt/vla
COPY src/ /opt/vla/src/

ENV VLA_ROOT=/opt/vla \
    VLA_MODEL_DIR=/opt/vla/models \
    VLA_LOG_DIR=/opt/vla/logs \
    PYTHONPATH=/opt/vla/src

RUN mkdir -p /opt/vla/models /opt/vla/logs

# Installs the vla-perception entry point. --no-deps because every dependency is
# already pinned above and resolving them again could pull a different version.
RUN python3 -m pip install --no-cache-dir --no-deps -e /opt/vla/src

# The weights are NOT baked in: yolov8s-world.pt is 27 MB and CLIP ViT-B-32 is
# 354 MB, and both are mounted from ./models so the image stays rebuildable
# without re-downloading them. scripts/download_models.sh fetches them and
# verifies every sha256 against env/MANIFEST.md.
#
# If VLA_MODEL_DIR turns out to be empty, vla_paths.model_path() falls back to
# the bare filename and ultralytics downloads the weights itself -- which works,
# but gets whatever is current rather than the recorded version. Mount ./models.
VOLUME ["/opt/vla/models", "/opt/vla/logs"]

EXPOSE 5001

# The liveness route is "/" (there is no /health). start-period is 90 s because
# loading the model and paying the first-inference CUDA cost takes ~30 s on a
# warm GPU and longer cold; a shorter period reports the container unhealthy
# while it is simply still starting.
HEALTHCHECK --interval=15s --timeout=5s --start-period=90s --retries=5 \
    CMD curl -fsS "http://127.0.0.1:${YOLO_PORT:-5001}/" || exit 1

CMD ["python3", "-u", "/opt/vla/src/yolo_server.py"]
