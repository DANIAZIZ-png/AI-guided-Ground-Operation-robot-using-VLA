# Environment manifest

Captured **5 October 2026** on the machine that produced every result in the
report. This is what makes the numbers reproducible.

## Host

| | |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| Kernel | 6.17.0-14-generic, x86_64 |
| GPU | NVIDIA RTX A4000, 16376 MiB |
| NVIDIA driver | **595.71.05** |
| CUDA (runtime, via driver) | **13.2** |
| CUDA toolkit | **not installed** — no `nvcc`; everything uses pip-shipped CUDA libraries |
| Container engine | **Podman 4.9.3** — Docker is *not* installed |
| distrobox | 1.8.2.5 |
| NVIDIA Container Toolkit | **not installed** — distrobox provides GPU access with its own `--nvidia` host integration |
| Ollama | 0.30.8, serving on `127.0.0.1:11434` |

## Containers

`distrobox list`:

| Name | Image | Role |
|---|---|---|
| `ubuntu22-gpu` | **`localhost/ubuntu22-snapshot:latest`** (9.88 GB, image id `a0c41e00ba5a`) | ROS 2 Humble, Gazebo, Nav2, SLAM Toolbox, the agent |
| `vla-box` | `docker.io/library/ubuntu:22.04` (161 MB, image id `86f1a8d7b38e`) | YOLO server |
| `ubuntu22` | `docker.io/library/ubuntu:22.04` | created, unused |

| | `ubuntu22-gpu` | `vla-box` |
|---|---|---|
| Python | 3.10.12 | 3.10.12 |
| ROS Humble packages | **399** (`ros-humble-packages_ubuntu22-gpu.txt`) | 0 |
| pip packages | 147 (`pip-freeze_ubuntu22-gpu.txt`) | 32 (`pip-freeze_vla-box.txt`) |

### ⚠ `ubuntu22-gpu` cannot be pulled

It runs from a **local Podman snapshot**, not a registry image. Nobody else can
obtain it. `ros-humble-packages_ubuntu22-gpu.txt` lists all 399 packages with
exact Debian versions and is the only recoverable record of what is inside.

## Host virtualenv `~/yolo-env`

This is where the detector's dependencies actually live.

| | |
|---|---|
| Python | 3.12.3 (symlink to `/usr/bin/python3`) |
| pip packages | 64 (`pip-freeze_yolo-env.txt`) |
| torch | 2.12.1 |
| torchvision | 0.27.1 |
| ultralytics | 8.4.82 |
| ultralytics-thop | 2.0.20 |
| opencv-python | 4.13.0.92 |
| CLIP | git+https://github.com/ultralytics/CLIP.git@`16be45c7062240d445cce764f2afd9454a91ef7e` |
| numpy | 2.5.0 |

### ⚠ A cross-distro quirk that works by accident

`vla_demo.sh` stage 4 starts the detector like this:

```
distrobox enter vla-box -- bash -c "source $HOME/yolo-env/bin/activate && python -u $HOME/yolo_server.py"
```

So the venv is **created on the Ubuntu 24.04 host (Python 3.12) but activated
inside an Ubuntu 22.04 container (Python 3.10)**, and `yolo-env/bin/python3`
is a symlink to `/usr/bin/python3.12`, which does not exist in 22.04. It works
only because `$HOME` is bind-mounted into the container, so the host's
interpreter is reachable from inside. Recorded because it is load-bearing and
would not survive being rebuilt casually.

## Local LLM

| | |
|---|---|
| Model | `qwen2.5:7b` |
| Digest | `845dbda0ea48ed749caa` |
| Size | 4 683 087 332 B (4.68 GB) |
| Pulled | ~3 months before capture |

Pin it before a demo or the first command after a 5-minute idle costs **34.2 s**
instead of **0.15 s**.

## Model weights — not committed

| File | Size (B) | sha256 |
|---|---|---|
| `yolov8s-world.pt` | 27 169 314 | `095f5266bb9b654bd5ad9e21e9cdeda78e0f2c8460f5d652eaf04bab7ee251cf` |
| `weights/clip/ViT-B-32.pt` | 353 976 522 | `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af` |
| faster-whisper `medium.en` `model.bin` | 1 527 904 330 | `11b220779aea4c6f3ce9d2549c8a95ea869ed84066864b999531ef53e594fe5b` |
| faster-whisper `small.en` `model.bin` | 483 545 366 | `62b2a45b05ee59acb4a5341b33ee35e041395d378d418a18acfe4c9e768ee37a` |

Whisper models are cached at
`~/.cache/huggingface/hub/models--Systran--faster-whisper-{medium,small}.en/`.

No rosbags (`.db3`, `.mcap`) exist anywhere in the project.

## Robot side

| | |
|---|---|
| Platform | TurtleBot 4 — iRobot Create 3 base + Raspberry Pi 4 |
| Pi address | `10.42.0.169` (PC hotspot `10.42.0.1`, 2.4 GHz channel 6) |
| Camera | Luxonis OAK-D, run **colour-only** at 30 fps / JPEG 75 / 512 px |
| Ranging | 2-D LiDAR (RPLIDAR) |
| Discovery | Fast DDS discovery server on the Pi, `10.42.0.169:11811` |

5 GHz hotspot is impossible on this PC: the Intel AX201's LAR firmware owns the
regulatory domain (`phy#0 self-managed`), so the card can join 5 GHz but not
create it.

## Files in this directory

| File | Lines |
|---|---|
| `ros-humble-packages_ubuntu22-gpu.txt` | 399 |
| `pip-freeze_ubuntu22-gpu.txt` | 147 |
| `pip-freeze_vla-box.txt` | 32 |
| `pip-freeze_yolo-env.txt` | 64 |
| `bashrc_lines_118-130.txt` | the ROS/DDS block from the host `~/.bashrc`, copied verbatim — the live file was **not** modified |
