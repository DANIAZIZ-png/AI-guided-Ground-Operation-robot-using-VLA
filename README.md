# AI-guided Ground Operation robot using VLA

> This repository contains the codding for a ground robot which uses OpenVLA model for Military operations

*(original README line, kept verbatim)*

---

## Frozen FYDP snapshot

**This branch (`main`) is an as-run snapshot, not a tidy codebase.** It is the
working tree exactly as it stood on **5 October 2026**, when the hardware demo
was running end to end. Filenames, flat layout, superseded versions and all.
The point is that the results in the report can be traced to the code that
produced them.

**A cleaned-up, reorganised version will live on branch `reorg`.** Use that one
if you want to read the project; use this one if you want to reproduce it.

Tagged **`v1.0-fydp-snapshot`**.

### What the system does

A TurtleBot 4 (Create 3 base + Raspberry Pi 4) driven by natural language.
Speech is transcribed locally, a local LLM turns the sentence into a plan, an
open-vocabulary detector finds the named object in the camera image, the 2-D
laser gives its range, and Nav2 drives to a stand-off pose in front of it.
The LLM proposes; deterministic Python disposes — every motion passes a guard.

| Layer | What runs |
|---|---|
| Speech | `voice_command.py` — faster-whisper `medium.en`, push-to-talk |
| Reasoning | `llm_brain.py` → Ollama `qwen2.5:7b`, local, no cloud |
| Perception | `yolo_server.py` — YOLOv8s-World (open vocabulary) + CLIP |
| Agent / guards | `vla_agent_v28.py` — the state machine and all safety gates |
| Navigation | Nav2 (`nav2_hw_composed.launch.py`), SLAM Toolbox (`slam_hw.launch.py`) |
| Operator UI | `vla_gui_v2.py` + RViz (`vla_hardware.rviz`) |

### Running it

Hardware demo, from a plain host terminal with the robot docked:

```bash
~/vla_demo.sh          # 13 gated stages, ~4 min
~/vla_demo_stop.sh     # shutdown + re-dock
```

Simulation: `~/vla_sim.sh`. The two use **opposite** DDS discovery settings and
each fails silently under the other's — see `CLAUDE.md`, which is the operating
manual for this machine and the first thing to read.

### Layout

| Path | Contents |
|---|---|
| *(top level)* | live scripts, agent, GUI, launch files, params, saved map |
| `vla_tools/` | diagnostic probes and xterm launchers |
| `config/` | DDS profiles and GUI configs — these live in dotfiles on the host |
| `docs/` | handout, handovers, changelogs, report section |
| `archive/` | superseded versions, kept for provenance — **not** the live code |
| `results/` | `report_pack/`, Wi-Fi packet captures, Nav2 and demo logs |
| `env/` | environment manifest: see **Reproducibility** below |

The live agent is `vla_agent_v28.py`. Everything named `vla_agent_v9` … `v27`
is history, not an alternative.

### Reproducibility

`env/` records the machine this ran on. Two honest caveats:

1. **`ubuntu22-gpu` is a local Podman snapshot image** (`localhost/ubuntu22-snapshot:latest`,
   9.88 GB) and cannot be pulled by anyone else.
   `env/ros-humble-packages_ubuntu22-gpu.txt` (399 packages, pinned versions) is
   the only recoverable record of its contents.
2. **`yolo_server.py` runs inside the `vla-box` container but activates the host
   venv `~/yolo-env`**, whose `python3` symlinks to `/usr/bin/python3.12` — a
   binary that does not exist in Ubuntu 22.04. It works only because the home
   folder is shared between host and container. Fragile; noted deliberately.

### Model weights — deliberately not committed

Fetch these separately; hashes are here so you can confirm you have the same ones.

| File | Size | sha256 |
|---|---|---|
| `yolov8s-world.pt` | 27.2 MB | `095f5266bb9b654bd5ad9e21e9cdeda78e0f2c8460f5d652eaf04bab7ee251cf` |
| `weights/clip/ViT-B-32.pt` | 354 MB | `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af` |
| faster-whisper `medium.en` | 1.53 GB | `11b220779aea4c6f3ce9d2549c8a95ea869ed84066864b999531ef53e594fe5b` |
| faster-whisper `small.en` | 484 MB | `62b2a45b05ee59acb4a5341b33ee35e041395d378d418a18acfe4c9e768ee37a` |
| Ollama `qwen2.5:7b` | 4.68 GB | digest `845dbda0ea48ed749caa` |

### Note on hard-coded MAC addresses

`vla_demo.sh` and `vla_tools/hotspot_guard.sh` match on three MAC addresses —
the robot's Wi-Fi interface and two Bluetooth headsets. They are load-bearing:
the hotspot guard distinguishes the robot from stray clients, and the demo
disconnects the earbuds because they share the PC's single antenna with the
hotspot. Left as-run on purpose.
