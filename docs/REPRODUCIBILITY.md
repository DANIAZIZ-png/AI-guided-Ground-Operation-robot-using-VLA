# Reproducibility run

A from-scratch verification: clone the repository to a path it has never seen,
follow **only** the README, and record what passed, what did **not** run, and how
long each step took.

| | |
|---|---|
| Date | 5 October 2026 |
| Clone path | `/tmp/vla-verify` (deliberately not under `~/repos`) |
| Commit | `796543f` on branch `reorg` |
| Host | Ubuntu 24.04.4, kernel 6.17.0-14, RTX A4000, driver 595.71.05 |
| Engine | Podman 4.9.3 |
| Network | shared 2.4 GHz link, ~1–7 MB/s and highly variable |

**On timings:** download figures are bandwidth, not properties of the
repository. They are recorded to set expectations, not as a benchmark. The steps
that do no network I/O are the meaningful ones, and they all complete in
single-digit seconds.

---

## Summary

| Step | Result | Time |
|---|---|---|
| `git clone -b reorg …` | ✅ 43 MB | 12 s |
| `make help` | ✅ | <1 s |
| `make tools` (venv + podman-compose 1.6.0) | ✅ | 5 s |
| **`make parity`** | ✅ **`PARITY OK`** | 2 s |
| `make test` | ✅ **140 passed** | 1 s |
| `make lint` | ✅ `All checks passed!` | 1 s |
| `make models` → `yolov8s-world.pt` | ✅ downloaded, **sha256 verified** | 24 s (27 MB) |
| `make models` → `ViT-B-32.pt` | ✅ downloaded, **sha256 verified** | 99 s (354 MB) |
| `make models` → faster-whisper | ⏸ **not completed** — bounded at 300 s | ~11 min est. |
| `make images` → `vla-ros:1.0` | ✅ **built, 4.39 GB, tested inside** | ~50 min over 4 attempts |
| `make images` → `vla-perception:1.0` | ❌ **NOT BUILT** — see below | — |
| `make sim` | ❌ **NOT RUN** — needs the perception image | — |
| `tests/smoke_sim.sh` | ❌ **NOT RUN** — needs the perception image | — |
| `make robot` | ⛔ **NOT VERIFIED ON HARDWARE** | — |

**What this run does and does not license you to say.** It shows the repository
is self-contained, path-independent and that the ROS half of the stack rebuilds
from pinned sources on a machine-independent basis. It does **not** show the
simulation runs end to end, and it says nothing at all about the robot.

---

## ✅ What passed

### The parity gate from a different path

This is the property the review asked to be proven. Run from `/tmp/vla-verify`,
a path the baseline has never seen:

```
$ cd /tmp/vla-verify && make parity
==> refactor parity gate
PARITY OK  (160 baseline names all present and unchanged; now 161)
```

Before the fix it failed here, naming exactly the two constants predicted:

```
PARITY FAIL  (2 problems)
  CHANGED constant: LOG_DIR         was /home/danyalaziz/... now $VLA_ROOT/logs
  CHANGED constant: SAVE_MAP_PATH   was /home/danyalaziz/... now $VLA_ROOT/maps/...
```

A second, independent version of the same bug was then found by running the gate
**inside the container image**: the agent derives its topic names from `VLA_NS`
at import time, so with `VLA_MODE=sim` about ten constants read as CHANGED while
nothing in the code had moved. `make test` would have **passed in robot mode and
failed in sim mode** — worse than failing everywhere, because it looks like a
regression exactly when someone is working on the simulation. The snapshot now
neutralises the environment first, verified under four:

| Environment | Result |
|---|---|
| no mode set | `PARITY OK` |
| `source config/sim.env` | `PARITY OK` |
| `source config/robot.env` | `PARITY OK` |
| hostile (`VLA_NS=/wat VLA_RAW_RGB=1 VLA_LOG_DIR=/tmp/x YOLO_PORT=9999`) | `PARITY OK` |

### Tests and lint from the clone

```
$ make test   ->  140 passed in 0.50s
$ make lint   ->  All checks passed!
```

91 of the 140 need no ROS at all, which is what CI runs.

### Model downloads

Against an **empty** model directory, through the script's own code path:

```
$ VLA_MODEL_DIR=/tmp/vla-verify-models ./scripts/download_models.sh
  OK  yolov8s-world.pt   downloaded and verified     27 MB,  24 s
  OK  ViT-B-32.pt        downloaded and verified    354 MB,  99 s
```

381 MB at ~3.1 MB/s, both hashes checked against `env/MANIFEST.md`.

### The ROS image

Built, 4.39 GB, 30 steps. Verified **inside** it against `env/MANIFEST.md`:

| Checked | Found | Expected |
|---|---|---|
| numpy | **1.26.4** | 1.26.4 (`pip-freeze_ubuntu22-gpu.txt`) |
| cv2 | **4.5.4** | apt `python3-opencv`, not a pip build |
| requests | **2.25.1** | 2.25.1 |
| rclpy | imports | — |
| colcon | `/usr/bin/colcon` | — |
| `xvfb-run`, `xterm`, `fastdds` | present | — |
| `vla_bringup` | discoverable, 5 launch files | — |
| entry points | all 5 on PATH | `vla-agent` … `vla-gui` |
| tests inside the image | **140 passed**, `PARITY OK` | — |

ROS is pinned to the `snapshots.ros.org` snapshot of **2026-05-14**, verified
package-by-package against all 399 recorded versions (`env/ROS_SNAPSHOT.md`).

---

## ❌ What was NOT run

### The perception image was not built

The build was **interrupted deliberately** after repeated network failures, with
the last attempt stopped part-way through the dependency download. It is not a
code or pin problem: every failure was a transfer failure on a 2.4 GHz link
shared with the robot's hotspot, pulling CUDA wheels of 200–423 MB each.

```
pip._vendor.urllib3.exceptions.ReadTimeoutError:
  HTTPSConnectionPool(host='files.pythonhosted.org', port=443): Read timed out.
```

`docker/perception.Dockerfile` now wraps pip in a three-attempt loop with
`--timeout 120 --retries 10` and asserts the imports afterwards, so the next run
has a much better chance — but **it has never completed, and no claim is made
that it works.**

### Therefore `make sim` and the smoke test were not run

Both need `vla-perception:1.0`. `tests/smoke_sim.sh` has been written and
syntax-checked, and verified to contain no `pkill`, no `vla_kill.sh` and no
`vla_sim.sh` call — but it **has not been executed**, so the simulation has not
been shown to come up end to end in this environment.

### The robot profile is not verified on hardware

The robot was not powered or reachable: `10.42.0.169:22` refused, and
`wlp0s20f3` was on `192.168.137.156/24` — the PC was joined to another network
rather than hosting the hotspot, so the robot's subnet did not exist. **No claim
is made about `make robot`.**

---

## Exact commands to finish both later

### 1. Build the perception image

```bash
cd ~/repos/AI-guided-Ground-Operation-robot-using-VLA   # or a fresh clone
make image-perception                                   # ~5 GB of wheels
```

Do it on a wired or 5 GHz connection if you can. It needs no GPU to build; the
GPU is only needed to run it. Check it afterwards:

```bash
podman run --rm vla-perception:1.0 python3 -c \
  "import torch, ultralytics, cv2, numpy; print(torch.__version__, ultralytics.__version__)"
# expect: 2.12.1 8.4.82
```

If pip times out again, re-run the same command — the completed layers are
cached, so it resumes rather than starting over.

### 2. Finish the model downloads

```bash
make models          # fetches the ~2 GB of faster-whisper, then verifies everything
make verify-models   # expect every line OK, exit 0
```

The pinned-revision mechanism is already proven: the small `config.json` from
revision `a29b04bd1538` was fetched by commit and its sha256 matched exactly.
Only the 2 GB transfer was not waited out.

### 3. Run the simulation and the smoke test

```bash
make sim                 # Gazebo + SLAM + Nav2 + agent + GUI, via compose
# in another terminal:
tests/smoke_sim.sh       # hard timeout 300 s; override with SMOKE_TIMEOUT
```

Hard assertions: `xvfb` present, `/scan`, `/odom`, SLAM `/map`, the Nav2 action
server, Nav2 accepting a goal, and the agent answering a typed command. The
end-to-end "go to the chair" drive is reported but **does not fail the run**,
because it depends on YOLO finding a chair from whatever pose the Gazebo world
spawns the robot at — a smoke test that fails on that tells you nothing about
the code.

### 4. Verify the robot profile on hardware

Robot docked, charged, on the 2.4 GHz channel-6 hotspot:

```bash
timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22" && echo reachable
make robot               # or scripts/vla_demo.sh for the original 13-stage bring-up
```

---

## Build failures worth recording

Four, across the two images. **One was a real defect; three were the network.**
None would have been caught by any test, and all are now defended against.

| # | Failure | Cause | Fix |
|---|---|---|---|
| 1 | `pip install -e` → "build backend is missing the `build_editable` hook" | `setuptools<66` predates PEP 660 | upgrade pip/setuptools first; require `setuptools>=64` |
| 2 | `colcon: not found` | **a gap in the project's own manifest** — `env/ros-humble-packages_ubuntu22-gpu.txt` was generated with `grep ros-humble`, so all 24 `python3-colcon-*` were excluded *by construction* | `docker/ros-tooling-packages.txt`; all 24 match the same snapshot |
| 3 | apt: "File has unexpected size (134 != 3074). Mirror sync in progress?" | transient mirror | `Acquire::Retries` + a retry loop that clears partials; **recovered on attempt 2 of the next build** |
| 4 | pip `ReadTimeoutError` on a 423 MB CUDA wheel | transient, slow link | `--timeout 120 --retries 10` + retry loop |

A fifth issue was silent rather than fatal: the editable install **replaced
apt's numpy 1.21.5 with numpy 2.2.6** and `opencv-python 5.0.0.93` over
`python3-opencv`. ROS Humble's Python extensions are built against numpy 1.x, so
a numpy 2 ABI under `rclpy` and `cv_bridge` is a real hazard. Fixed with
`--no-deps` and numpy pinned to the recorded 1.26.4.

Both Dockerfiles now assert after installing — `dpkg -l | grep navigation2` and
`command -v colcon` for ROS, an import check for perception — because without
them a run where every retry failed would still exit 0 and produce an image
missing the thing it exists to provide.

---

## One correction this run forced on the README

The quick start said `git clone <url>`, which fetches the default branch. While
this work lived on `reorg`, following the README literally could not work. It
was changed to `git clone -b reorg …` for the duration, and reverted to a plain
`git clone` once `reorg` was merged into `main`.

Worth stating plainly: the instruction to follow **only** the README is what
exposed that. Reading the repository and inferring the right command would have
hidden it.
