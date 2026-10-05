# AI-guided Ground Operation robot using VLA

> This repository contains the codding for a ground robot which uses OpenVLA model for Military operations

*(original README line, kept verbatim)*

A TurtleBot 4 driven by spoken natural language, running **entirely on local
hardware** — no cloud APIs. You say *"go to the chair"*; the robot transcribes
it, plans with a local LLM, finds the chair with an open-vocabulary detector,
ranges it with the laser, and drives to a stand-off pose in front of it.

Final-year design project. The hardware demo ran end to end on **25 September
2026**.

---

## Quick start

```bash
git clone -b reorg https://github.com/DANIAZIZ-png/AI-guided-Ground-Operation-robot-using-VLA.git
cd AI-guided-Ground-Operation-robot-using-VLA
make setup && make sim
```

> **`-b reorg` is required for now.** The containers, the Makefile, the tests and
> the path-independent layout live on the `reorg` branch until it is merged.
> `main` still has the flat as-run layout with absolute `/home/danyalaziz` paths
> and no `make setup`. Drop the flag once `reorg` lands on `main`.

`make setup` creates a tool venv, downloads the model weights **and verifies
every sha256** against `env/MANIFEST.md`, then builds both container images.
`make sim` starts the simulation. `make help` lists everything else.

For the real robot — powered, docked, on the PC's hotspot:

```bash
make robot
```

| | |
|---|---|
| `make test` | 133 tests (91 of them need no ROS) |
| `make parity` | the refactor parity gate |
| `make smoke` | headless simulation smoke test |
| `make stop` | everything down |

### Requirements

| | |
|---|---|
| GPU | NVIDIA, **driver ≥ 595**. Developed on an RTX A4000 16 GB |
| Container engine | Podman ≥ 4.9 **or** Docker. `ENGINE=docker make sim` to force |
| GPU in containers | NVIDIA Container Toolkit. Podman needs a CDI spec: `sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml` |
| Disk | ~25 GB for both images plus ~2.4 GB of weights |
| Robot (hardware only) | TurtleBot 4 — iRobot Create 3 + Raspberry Pi 4, OAK-D camera, RPLIDAR |

Nothing else is needed on the host: ROS, Gazebo, Nav2, torch and Whisper all
live inside the images.

---

## How it works

```
  microphone ──► vla_voice ──────────► /vla/command
                 faster-whisper            │
                 medium.en, local          ▼
                                      vla_agent ──────► vla_brain ──► Ollama
                                      state machine     prompt and     qwen2.5:7b
                                      and ALL guards    parse plan     (local)
                                            │
                         ┌──────────────────┼──────────────────┐
                         ▼                  ▼                  ▼
                   vla_perception       /scan (LiDAR)        Nav2
                   YOLOv8s-World        range to target      drive to pose
                   open vocabulary
```

**The LLM proposes; deterministic Python disposes.** The model never actuates
anything — it can only return a plan. `vla_brain` validates that plan against a
schema and drops any action it invented; `vla_agent` then decides whether the
plan is allowed, through guards for front clearance, arrival tolerance and
stand-off distance. In recorded runs the guards intervened and the robot still
did the right thing. That separation is the project's central claim, and it is
what `tests/test_llm_brain.py` exercises.

| Package | Entry point | Role |
|---|---|---|
| `src/vla_agent/` | `vla-agent` | state machine and every safety guard |
| `src/vla_brain/` | `vla-brain` | Ollama client, plan parsing, guardrails |
| `src/vla_perception/` | `vla-perception` | YOLOv8s-World + CLIP over HTTP |
| `src/vla_voice/` | `vla-voice` | push-to-talk, faster-whisper |
| `src/vla_gui/` | `vla-gui` | operator console |
| `vla_bringup/` | — | ROS 2 package: `ros2 launch vla_bringup robot.launch.py` |

---

## Repository map

| Path | Contents |
|---|---|
| `src/` | the five Python packages, plus thin shims at the old filenames |
| `vla_bringup/` | ROS 2 bring-up package (`sim.launch.py`, `robot.launch.py`) |
| `launch/` | the Nav2 and SLAM launch files, **with the Fast DDS storm fixes** |
| `config/` | `paths.sh`, `sim.env`, `robot.env`, params, DDS profiles, RViz, GUI configs |
| `scripts/` | bring-up, shutdown, mode switching, `download_models.sh` |
| `tools/` | 30 diagnostic probes and launchers |
| `maps/` | the saved SLAM map |
| `docker/` | both Dockerfiles, the pinned ROS package list, the entrypoint |
| `tests/` | 133 tests, the parity gate, the headless smoke test |
| `docs/` | handout, handovers, changelogs, `REORG_PROGRESS.md` |
| `env/` | environment manifest and `ROS_SNAPSHOT.md` |
| `results/` | data pack, packet captures, measurements, 241 logs |
| `archive/` | superseded code, kept for provenance. **Not live.** |
| `CLAUDE.md` | the operating manual for the physical machine |

Every path derives from `VLA_ROOT`, resolved from `config/paths.sh` or
`src/vla_paths.py`, so a clone works anywhere. `config/.env.example` lists every
override; copy it to `config/.env`.

---

## Reproducing the results

`env/MANIFEST.md` records the machine. Two things make it more than a list:

**The ROS versions are exactly recoverable.** All **399** installed ROS Humble
packages match the `snapshots.ros.org` Humble snapshot of **2026-05-14**,
exactly — zero version differences, zero missing. `docker/ros.Dockerfile` pins
the apt source to that date and installs all 399 at their recorded versions.
`env/ROS_SNAPSHOT.md` has the verification method and a command to re-check it.
This replaces what used to be the biggest hole: the ROS container ran from a
local Podman image nobody else could pull.

**The weights are hash-verified.** `scripts/download_models.sh` fetches them and
checks every sha256 against the manifest. A file that downloads but hashes
differently is moved aside and the run fails, rather than silently making every
recorded number unreproducible.

| Model | Size | sha256 |
|---|---|---|
| `yolov8s-world.pt` | 27.2 MB | `095f5266…ee251cf` |
| `ViT-B-32.pt` (CLIP) | 354 MB | `40d36571…ba950af` |
| faster-whisper `medium.en` | 1.53 GB | `11b22077…594fe5b` |
| faster-whisper `small.en` | 484 MB | `62b2a45b…8ee37a` |
| Ollama `qwen2.5:7b` | 4.68 GB | digest `845dbda0ea48` |

`docker/perception.Dockerfile` also removes the setup's most fragile part: the
detector used to run inside an Ubuntu 22.04 container while activating a **host**
virtualenv whose `python3` pointed at a 3.12 binary that does not exist in
22.04. It worked only because `$HOME` was bind-mounted. It is now Python 3.12 in
its own image with all 64 dependencies pinned.

See `docs/REPRODUCIBILITY.md` for a from-scratch verification run.

---

## What was actually hard

Worth stating plainly, because it is the project's main engineering lesson:
**the difficult problems were in the middleware and the radio link, not the AI.**
Detection, planning and the LLM behaved much as in simulation. The days that got
lost went to:

- **A Fast DDS heartbeat storm of our own making.** Nav2 launched without the
  `/parameter_events` and `/rosout` remaps flooded the Raspberry Pi at ~7,400
  packets/s (2 MB/s). The Pi hit 84 °C, throttled, and the camera stopped. For
  two days this looked like a thermally weak Pi, then like a camera bug. Packet
  captures in `results/evidence/` settled it. The remaps are mandatory and now
  live in `launch/`, in `vla_bringup`, and in the operator console's buttons.
- **A "4.3 s camera lag" that was neither a clock nor a camera fault.** It was
  the depth pipeline plus the same flood. Colour-only, a frame arrives in
  47–85 ms. Ranging moved to the LiDAR.
- **Bluetooth earbuds killing the hotspot.** The PC's Intel AX201 shares one
  antenna between Wi-Fi and Bluetooth. Earbuds connected in mic mode: 100 %
  packet loss to the robot. Disconnected, same spot: 0 %.
- **A 2-D laser seeing straight through a chair** — the beam passes between the
  legs and reports the wall 7 m away. Fixed with a stand-off and an "arrived if
  seen within 1.2 m" rule.

Two of these were first "fixed" by adjusting thresholds, which did nothing. Each
only yielded to a measurement. **Measure before changing** is the honest
conclusion of the hardware phase.

---

## Branches

| Ref | What |
|---|---|
| `main` | organised layout |
| `reorg` | this work: paths, packages, containers, tests, CI |
| `snapshot-as-run` | the original flat layout, byte-for-byte as it ran |
| `v1.0-fydp-snapshot` | tag pinning that as-run state |

## Notes

- The project is named after OpenVLA, which was **evaluated and dropped** — it is
  in `archive/openvla/`. The shipped system is a local LLM for reasoning plus an
  open-vocabulary detector for grounding, which ran on the available hardware
  where OpenVLA did not.
- `scripts/vla_demo.sh` and `scripts/vla_demo_stop.sh` are the original 13-stage
  hardware bring-up and shutdown, kept working and unchanged.
- Three MAC addresses are hard-coded in `scripts/vla_demo.sh` and
  `tools/hotspot_guard.sh` — the robot's Wi-Fi and two Bluetooth headsets. They
  are load-bearing: the hotspot guard tells the robot from stray clients, and
  the demo disconnects the earbuds because of the shared-antenna problem above.

## License

MIT — see `LICENSE`. Please cite via `CITATION.cff` if you use this work.
