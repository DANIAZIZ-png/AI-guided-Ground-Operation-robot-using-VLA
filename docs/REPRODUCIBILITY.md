# Reproducibility run

A from-scratch verification: clone the repository to a path it has never seen,
follow **only** the README, and record what passed, what failed and how long each
step took.

| | |
|---|---|
| Date | 5 October 2026 |
| Clone path | `/tmp/vla-verify` (deliberately not under `~/repos`) |
| Commit | `796543f`, branch `reorg` |
| Host | Ubuntu 24.04.4, kernel 6.17.0-14, RTX A4000, driver 595.71.05 |
| Engine | Podman 4.9.3 |
| Network | shared 2.4 GHz link, ~1–7 MB/s and variable — see the note on timings |

**On timings:** the download figures are bandwidth, not properties of the
repository. They are recorded because they set expectations, not as a benchmark.
The `make` steps that do no network I/O are the meaningful ones, and they are all
measured in single-digit seconds.

---

## Results

| Step | Result | Time |
|---|---|---|
| `git clone -b reorg …` | ✅ 43 MB | 12 s |
| `make help` | ✅ | <1 s |
| `make tools` (venv + podman-compose 1.6.0) | ✅ | 5 s |
| **`make parity`** | ✅ **`PARITY OK`** | 2 s |
| `make test` | ✅ **140 passed** | 1 s |
| `make lint` | ✅ `All checks passed!` | 1 s |
| `make models` — `yolov8s-world.pt` | ✅ downloaded, **sha256 verified** | 24 s (27 MB) |
| `make models` — `ViT-B-32.pt` | ✅ downloaded, **sha256 verified** | 99 s (354 MB) |
| `make models` — faster-whisper | ⏸ bounded, see below | ~11 min est. |
| `make images` — `vla-ros:1.0` | ✅ built, 4.39 GB | see below |
| `make images` — `vla-perception:1.0` | ⏳ in progress | — |
| `make sim` | ⏳ pending the perception image | — |
| `tests/smoke_sim.sh` | ⏳ pending | — |
| Robot profile (`make robot`) | ⛔ **not verified on hardware** | — |

---

## The parity gate from a different path — the thing under test

This is what the review asked to be proven. Run from `/tmp/vla-verify`, a path
the baseline has never seen:

```
$ cd /tmp/vla-verify && make parity
==> refactor parity gate
PARITY OK  (160 baseline names all present and unchanged; now 161)
```

Before the fix it failed here, naming exactly the two constants predicted:

```
PARITY FAIL  (2 problems)
  CHANGED constant: LOG_DIR
    was: /home/danyalaziz/repos/.../logs     now: $VLA_ROOT/logs
  CHANGED constant: SAVE_MAP_PATH
    was: /home/danyalaziz/repos/.../maps/... now: $VLA_ROOT/maps/warehouse_map
```

A second, independent version of the same bug was then found by running the gate
**inside the container image**: the agent derives its topic names from `VLA_NS`
at import time, so with `VLA_MODE=sim` about ten constants read as CHANGED while
nothing in the code had moved. `make test` would have **passed in robot mode and
failed in sim mode**, which is worse than failing everywhere. The snapshot now
neutralises the environment first, and is verified under four:

| Environment | Result |
|---|---|
| no mode set | `PARITY OK` |
| `source config/sim.env` | `PARITY OK` |
| `source config/robot.env` | `PARITY OK` |
| hostile (`VLA_NS=/wat VLA_RAW_RGB=1 VLA_LOG_DIR=/tmp/x YOLO_PORT=9999`) | `PARITY OK` |

---

## Model downloads

Run against an **empty** model directory, through the script's own code path —
not a hand-written `curl`:

```
$ VLA_MODEL_DIR=/tmp/vla-verify-models ./scripts/download_models.sh
  fetching yolov8s-world.pt ...
  OK      yolov8s-world.pt       downloaded and verified      27 MB,  24 s
  fetching ViT-B-32.pt ...
  OK      ViT-B-32.pt            downloaded and verified     354 MB,  99 s
  faster-whisper (pinned revisions):
    medium.en  (revision a29b04bd1538)
```

381 MB at ~3.1 MB/s average, both hashes verified against `env/MANIFEST.md`.

**The Whisper download was bounded, not skipped.** At the observed rate the
remaining ~2.0 GB needs about 11 minutes, and the link is shared with the
container builds. The fetch-by-commit path it uses was verified separately and
decisively, with the small file from the same revision:

```
$ curl -fL .../resolve/a29b04bd15381511a9af671baec01072039215e3/config.json
  sha256 got : 4a1848ebabe7938d9797c15a2e8e4ce1d36e6fd4a43d096ae5955257c67c7962
  sha256 want: 4a1848ebabe7938d9797c15a2e8e4ce1d36e6fd4a43d096ae5955257c67c7962
  MATCH
```

So the mechanism is proven; only the 2 GB transfer was not waited out. To
complete it: `make models` and leave it running.

---

## The ROS image

Built, 4.39 GB, 30 steps. Verified **inside** it against `env/MANIFEST.md`:

| Checked | Found | Expected |
|---|---|---|
| numpy | **1.26.4** | 1.26.4 (`pip-freeze_ubuntu22-gpu.txt`) |
| cv2 | **4.5.4** | apt `python3-opencv`, not a pip build |
| requests | **2.25.1** | 2.25.1 |
| rclpy | imports | — |
| colcon | `/usr/bin/colcon` | — |
| `xvfb-run`, `xterm`, `fastdds` | all present | — |
| `vla_bringup` | discoverable, 5 launch files | — |
| entry points | all 5 on PATH | `vla-agent` … `vla-gui` |
| tests inside the image | **140 passed**, `PARITY OK` | — |

ROS itself is pinned to the `snapshots.ros.org` snapshot of **2026-05-14**, which
was verified package-by-package to match all 399 recorded versions exactly
(`env/ROS_SNAPSHOT.md`).

### Three build failures, all fixed, all worth recording

The image did not build first time, and none of these would have been caught by
any test:

1. **`pip install -e` failed** — "build backend is missing the `build_editable`
   hook", because `setuptools<66` predates PEP 660. Fixed by upgrading pip and
   setuptools first and requiring `setuptools>=64`.
2. **`colcon: not found`** — a gap in the project's own manifest.
   `env/ros-humble-packages_ubuntu22-gpu.txt` was generated with
   `grep ros-humble`, so all 24 `python3-colcon-*` packages were excluded *by
   construction*. Now captured in `docker/ros-tooling-packages.txt`; all 24 match
   the same snapshot exactly.
3. **The editable install silently replaced apt's numpy 1.21.5 with numpy
   2.2.6**, and `opencv-python 5.0.0.93` over `python3-opencv`. ROS Humble's
   Python extensions are built against numpy 1.x, so a numpy 2 ABI under `rclpy`
   and `cv_bridge` is a real hazard. Fixed with `--no-deps` and numpy pinned to
   the recorded 1.26.4.

A fourth failure was **not** a code problem: apt fetched 134 bytes instead of
3074 for `libomp-dev` ("Mirror sync in progress?"). The Dockerfile now sets
`Acquire::Retries` and wraps the install in a retry loop that clears partials
between attempts — and it earned itself immediately, recovering on attempt 2 of
the next build. A post-install assertion was added too, because without it a run
where all attempts failed would still have exited 0 and produced an image with no
ROS in it.

---

## Not verified

- **The robot profile is NOT VERIFIED ON HARDWARE.** The robot was not powered
  or reachable during this run: `10.42.0.169:22` refused, and `wlp0s20f3` was on
  `192.168.137.156/24`, i.e. the PC was joined to another network rather than
  hosting the hotspot, so the robot's subnet did not exist. No claim is made
  about `make robot`.
- `make sim` and `tests/smoke_sim.sh` are pending the perception image.
- The simulation smoke test's end-to-end "go to the chair" assertion is **soft
  by design**: it depends on YOLO finding a chair from whatever pose the Gazebo
  world spawns the robot at. The hard assertions are `xvfb`, `/scan`, `/odom`,
  SLAM `/map`, the Nav2 action server, Nav2 accepting a goal, and the agent
  answering a typed command.

## One correction the run forced on the README

The README's quick start said `git clone <url>`, which fetches **`main`** — and
`main` still has the flat as-run layout with absolute `/home/danyalaziz` paths
and no `make setup`. Following the README literally therefore could not work.
It now says `git clone -b reorg …`, with a note to drop the flag once the branch
merges. Worth stating plainly: the instruction to follow only the README is what
exposed this.
