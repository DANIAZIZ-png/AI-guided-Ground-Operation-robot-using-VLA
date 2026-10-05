# AI-guided Ground Operation robot using VLA

> This repository contains the codding for a ground robot which uses OpenVLA model for Military operations

*(original README line, kept verbatim)*

A TurtleBot 4 driven by spoken natural language. You say *"go to the chair"*; the
robot transcribes it locally, plans with a local LLM, finds the chair with an
open-vocabulary detector, ranges it with the laser, and drives to a stand-off
pose in front of it. Nothing leaves the machine — no cloud APIs.

Final-year design project. The hardware demo ran end to end on **25 September 2026**.

---

## How it works

```
  microphone ──► voice_command.py ──► /vla/command
                 faster-whisper          │
                 medium.en, local        ▼
                                   vla_agent_v28.py ──► llm_brain.py ──► Ollama
                                   state machine &      prompt/parse      qwen2.5:7b
                                   ALL safety guards                      (local)
                                         │
                      ┌──────────────────┼──────────────────┐
                      ▼                  ▼                  ▼
                 yolo_server.py      /scan (LiDAR)       Nav2
                 YOLOv8s-World       range to target     drive to pose
                 open vocabulary
```

The design rule: **the LLM proposes, deterministic Python disposes.** The model
never actuates anything. Every motion passes a guard in `vla_agent_v28.py` —
front-clearance minimum, arrival tolerance, stand-off distance — and the guards
fired in recorded runs while the robot still did the right thing.

| Layer | Program | Notes |
|---|---|---|
| Speech | `src/voice_command.py` | faster-whisper `medium.en`, push-to-talk |
| Reasoning | `src/llm_brain.py` | Ollama `qwen2.5:7b`, local |
| Perception | `src/yolo_server.py` | YOLOv8s-World + CLIP, open vocabulary |
| Agent / guards | `src/vla_agent_v28.py` | the state machine; **this is the core of the project** |
| Navigation | `launch/nav2_hw_composed.launch.py` | Nav2, 7/7 lifecycle bonds in ~30 s |
| Mapping | `launch/slam_hw.launch.py` | SLAM Toolbox |
| Operator UI | `src/vla_gui_v2.py` | plus RViz (`config/vla_hardware.rviz`) |

---

## Repository layout

| Folder | What's in it |
|---|---|
| `src/` | the five live programs. Start with `vla_agent_v28.py` |
| `launch/` | ROS 2 launch files for Nav2 and SLAM |
| `config/` | params, DDS profiles, RViz layout, GUI configs |
| `scripts/` | bring-up, shutdown and mode-switching shell scripts |
| `tools/` | 30 diagnostic probes and xterm launchers |
| `maps/` | the saved SLAM map |
| `docs/` | project handout, handovers, changelogs, report section |
| `env/` | environment manifest — exact versions of everything |
| `results/` | data pack, packet captures, measurements, screenshots, 241 logs |
| `archive/` | superseded code kept for provenance. **Not live.** |
| `CLAUDE.md` | the operating manual for the physical machine — read this first |

**`CLAUDE.md` is the most useful document here.** It is the accumulated list of
every trap this hardware sets, written while debugging it.

### ⚠ Paths inside the scripts are absolute

Every script refers to `/home/danyalaziz/...`, because that is where it ran.
This repository is a **record of a working system, not a package you can clone
and launch.** To actually run it, the files go back to `$HOME` on a machine set
up as `env/MANIFEST.md` describes:

| Repo path | Host path |
|---|---|
| `src/*`, `scripts/*`, `launch/*`, `maps/*` | `~/` (flat) |
| `tools/*` | `~/vla_tools/` |
| `config/fastdds_*.xml` | `~/.ros/` |
| `config/vla_gui.sim.json` | `~/.vla_gui.sim.json` |
| `config/vla_gui.robot.json` | `~/.vla_gui.robot.json` |
| `config/slam_vla.yaml`, `config/nav2_hw_slow.yaml`, `config/vla_hardware.rviz` | `~/` |

The flat as-run layout is preserved exactly on branch **`snapshot-as-run`** and
at tag **`v1.0-fydp-snapshot`**, if you need to see where each file really sat.

---

## Running it

Hardware, from a plain host terminal with the robot docked and charged:

```bash
~/vla_demo.sh            # 13 gated stages, ~4 min; stops loudly at the first failure
~/vla_demo_stop.sh       # shutdown and re-dock
```

Rehearse everything short of the undock with `VLA_DEMO_STOP_AFTER=8 ~/vla_demo.sh`.
Simulation is `~/vla_sim.sh`.

Simulation and hardware use **opposite** DDS discovery settings and each fails
*silently* under the other's — no error, just an empty topic list. `CLAUDE.md`
explains the switch; `config/README.md` explains the two DDS profiles.

---

## What was actually hard

Worth stating plainly, because it is the main engineering lesson of the project:
**the difficult problems were in the middleware and the radio link, not the AI.**
Detection, planning and the LLM behaved much as they had in simulation. The days
that got lost went to:

- **A Fast DDS heartbeat storm of our own making.** Nav2 launched without
  `/parameter_events` and `/rosout` remaps flooded the Raspberry Pi at ~7,400
  packets/s (2 MB/s). The Pi hit 84 °C, throttled, and the camera stopped. For
  two days this looked like a thermally weak Pi and then like a camera bug.
  Packet captures in `results/evidence/` settled it. The remaps are now in both
  Nav2 launch files and are mandatory.
- **A "4.3 s camera lag" that was neither a clock nor a camera fault.** It was
  the depth pipeline plus that same Wi-Fi flood. Colour-only, a fresh frame
  arrives in 47–85 ms. The camera has run colour-only ever since; ranging moved
  to the LiDAR.
- **Bluetooth earbuds killing the hotspot.** The PC's Intel AX201 shares one
  antenna between Wi-Fi and Bluetooth. With the earbuds connected in mic mode:
  100 % packet loss to the robot. Disconnected, same spot: 0 %.
- **A 2-D laser seeing straight through a chair** — the beam passes between the
  legs and reports the wall 7 m away. Fixed with a stand-off and an
  "arrived if seen within 1.2 m" rule.

Two of these were first "fixed" by adjusting thresholds, which did nothing. Each
only yielded to a measurement. That pattern — **measure before changing** — is
the honest conclusion of the hardware phase.

---

## Reproducibility

`env/MANIFEST.md` records the exact machine: Ubuntu 24.04.4, kernel 6.17.0-14,
RTX A4000, driver 595.71.05, CUDA 13.2 runtime, Podman 4.9.3, 399 pinned ROS
Humble packages, and `pip freeze` for all three Python environments.

Two caveats stated deliberately rather than hidden:

1. **The ROS container cannot be pulled.** `ubuntu22-gpu` runs from a local
   Podman snapshot (`localhost/ubuntu22-snapshot:latest`, 9.88 GB). The
   399-package pinned list is the only recoverable record of its contents.
2. **The detector's virtualenv crosses distributions.** `yolo_server.py` runs
   inside the Ubuntu 22.04 `vla-box` container but activates the host venv
   `~/yolo-env`, whose `python3` points at `/usr/bin/python3.12` — a binary
   that does not exist in 22.04. It works only because `$HOME` is bind-mounted
   into the container. Fragile, and worth knowing before rebuilding anything.

### Model weights are not committed

Hashes so you can confirm you have the same ones:

| File | Size | sha256 |
|---|---|---|
| `yolov8s-world.pt` | 27.2 MB | `095f5266bb9b654bd5ad9e21e9cdeda78e0f2c8460f5d652eaf04bab7ee251cf` |
| `ViT-B-32.pt` (CLIP) | 354 MB | `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af` |
| faster-whisper `medium.en` | 1.53 GB | `11b220779aea4c6f3ce9d2549c8a95ea869ed84066864b999531ef53e594fe5b` |
| faster-whisper `small.en` | 484 MB | `62b2a45b05ee59acb4a5341b33ee35e041395d378d418a18acfe4c9e768ee37a` |
| Ollama `qwen2.5:7b` | 4.68 GB | digest `845dbda0ea48ed749caa` |

---

## Branches and tags

| Ref | What it is |
|---|---|
| `main` | organised layout — **read this one** |
| `snapshot-as-run` | the original flat layout, byte-for-byte as it ran |
| `v1.0-fydp-snapshot` | tag pinning that same as-run state |

Nothing was lost in the reorganisation: the 98 files removed were all exact
duplicates whose content still exists elsewhere in the tree, and every other
change was a rename that git records as such.

## A note on hard-coded MAC addresses

`scripts/vla_demo.sh` and `tools/hotspot_guard.sh` match on three MAC addresses
— the robot's Wi-Fi interface and two Bluetooth headsets. They are load-bearing:
the hotspot guard uses the robot's address to tell it apart from stray clients,
and the demo disconnects the earbuds because of the shared-antenna problem
above. Left as they ran, on purpose.
