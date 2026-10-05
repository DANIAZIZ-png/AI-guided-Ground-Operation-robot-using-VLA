# Pre-wipe backup checklist

**Audited 5 October 2026.** This PC is being wiped. This file is the record of
what was found in `~`, where each item must go, and whether it is safe to wipe
yet.

Everything was sorted by **content hash**, not by filename, against the
repository's `main` and `snapshot-as-run` branches — so "already in git" means
the bytes are in git, not that something with a similar name is.

---

## VERDICT

> ## ❌ NO — not safe to wipe yet
>
> Three things are outstanding, all of them actions only you can take:
>
> 1. **~100 MB of academic deliverables and ~71 MB of demo video are in `~` only**
>    and are not in git by design. They must be copied off. **The open house is
>    20 October**, and `open_house_final_v1.mp4` exists in one place on one disk.
> 2. **Phase 6 (the portability proof) has not finished.** Until a fresh clone has
>    been shown to rebuild and run, "the repo is the only copy" is a claim rather
>    than a verified fact.
> 3. **The robot's own configuration has not been collected** — the Pi was not
>    reachable during this audit (see §6).
>
> Everything in the **git** bucket is done: 38 files were added during this audit
> and nothing project-related remains uncommitted in `~`.
>
> Re-run `bash docs/verify_backup.sh` (§8) to re-check items 1 and 3.

---

## 1. git — done ✅

Added to the repository during this audit, 1.28 MB total. These existed only in
`~` and are now committed:

| Added | What | Why it mattered |
|---|---|---|
| `sim_objects/cup.sdf`, `red_box.sdf`, `door.sdf` | Gazebo models | **`tools/spawn_objects.py` spawns these.** Without them that tool does nothing, and they were in no branch. `/sim_objects/` had also been wrongly git-ignored in Phase 1 — they are source assets, not runtime output |
| `archive/backups/` (25 files) | pre-edit `.bak-*` copies of the agent, GUI, voice, demo scripts, SLAM params, RViz layout | the only record of several intermediate states |
| `archive/configs/` (5 files) | `vla_gui.json`, `vla_gui_sim.json` and three `.vla_gui.json.*` variants | config variants in no branch |
| `archive/notes/` (3 files) | `phase3_report.txt`, `phase3_arm2.txt`, `openvla_working_setup.txt` | working notes |
| `results/measurements/frames_2026-06-29_09.14.27.gv` | TF tree dump | a measurement artefact |
| `env/bashrc_full.txt` | the complete host `~/.bashrc` | only lines 118-130 were captured before |
| `docs/deliverables/Optimizations.docx` | 66 KB | **existed nowhere else.** Found untracked in a second, forgotten clone — see §7 |

**81 of 90** top-level project files in `~` were already preserved on
`snapshot-as-run` byte-for-byte. The live `~` copies of `vla_agent_v28.py` and
friends differ from `main` **on purpose** — `main` holds the refactored package
and `snapshot-as-run` holds the as-run original.

---

## 2. USB / Google Drive — ACTION REQUIRED ⬜

Too large or too personal for git. **This is the bucket that blocks the wipe.**

### Demo footage — needed for the open house on 20 Oct

| File | Size | Note |
|---|---|---|
| `~/Downloads/open_house_final_v1.mp4` | 26.9 MB | **the open-house video** |
| `~/Downloads/Demo_Explainer_Danyal_Aziz.mp4` | 3.8 MB | explainer |
| `~/Videos/recording-2026-07-28_21.21.52.mp4` | 22.8 MB | byte-identical to the copy in `~/Downloads` (sha256 `3c90e6f3…`) — **one copy is enough** |
| `~/Videos/recording-2026-09-30_13.03.49.mp4` | 8.5 MB | |
| `~/Videos/recording-2026-09-30_13.01.37.mp4` | 5.5 MB | |

**67.5 MB deduplicated.** `~/Videos/Screencasts/` is empty.

### Academic deliverables

| File | Size |
|---|---|
| `~/Downloads/FYDP_Report_Overleaf__2_.pdf` | 5.7 MB |
| `~/Downloads/FYDP_Research_Paper_Overleaf.pdf` | 1.9 MB |
| `~/Downloads/Project_Report.pdf` | 3.2 MB |
| `~/Downloads/FYDP_Poster_Danyal_Aziz.pptx` | 862 KB |
| `~/Downloads/8th_sem_final 2.pptx` | 17.5 MB |
| `~/Downloads/Danyal_Mid_Semester_8thSem.pptx` | 18.2 MB |
| `~/Downloads/Danyal_Aziz_7thSem_FYP_presentaion.pptx` | 17.3 MB |
| `~/Downloads/One_Slide_Summary_Danyal_Aziz.pptx` | 347 KB |
| `~/Downloads/80_Word_Summary_and_Elevator_Pitch_Danyal_Aziz.docx` | 12 KB |
| `~/Downloads/Phase2_Proposals_101B.docx` | 12 KB |
| `~/Downloads/Open_House-99EC.zip` | 6.5 MB |

**~71 MB.** These were **not** committed: they are your personal academic
documents and that is your call, not mine. If you want the report and paper in
the repository they are small enough — say so and I will add them under
`docs/deliverables/`.

### Screenshots not in git

11 files, **2.6 MB** — 9 in `~/VLA /Screenshots/` (note: that directory name
ends in a **space**, which breaks naive shell globbing), 1 in
`~/Pictures/Screenshots/`, plus `~/bus.jpg` and `~/detection_result.jpg`.

These are small enough to commit if you want them as evidence; you asked for
images in this bucket, so that is where they are. Say the word and I will add
them to `results/screenshots/`.

### The exact copy command

```bash
DEST=/media/$USER/USB/vla-backup          # or ~/GoogleDrive/vla-backup
mkdir -p "$DEST"/{video,deliverables,screenshots}

# video (deduplicated: the Downloads copy of the 28 Jul recording is skipped)
cp -v ~/Downloads/open_house_final_v1.mp4 \
      ~/Downloads/Demo_Explainer_Danyal_Aziz.mp4 \
      ~/Videos/recording-2026-07-28_21.21.52.mp4 \
      ~/Videos/recording-2026-09-30_13.03.49.mp4 \
      ~/Videos/recording-2026-09-30_13.01.37.mp4 \
      "$DEST/video/"

# deliverables
cp -v ~/Downloads/*.pdf ~/Downloads/*.pptx ~/Downloads/*.docx \
      ~/Downloads/Open_House-99EC.zip "$DEST/deliverables/"

# screenshots (quote the path: the directory name ends in a space)
cp -rv "$HOME/VLA /Screenshots" "$HOME/VLA /Screenshot from "*.png \
       ~/Pictures/Screenshots ~/bus.jpg ~/detection_result.jpg \
       "$DEST/screenshots/"

# verify before trusting it
du -sh "$DEST"                       # expect ~141 MB
find "$DEST" -type f | wc -l         # expect ~30
```

`~/Downloads/Kerberos_Conceptual_Guide.pdf` and `KerberOS.pdf` are other
coursework, not this project. Your call.

---

## 3. re-downloadable ✅

Nothing to copy. Every one of these is recoverable **by hash or by commit**, and
`scripts/download_models.sh` verifies each against `env/MANIFEST.md`.

| Item | Size | How it is recovered |
|---|---|---|
| `yolov8s-world.pt` | 27 MB | URL + sha256 in the script |
| `ViT-B-32.pt` (CLIP) | 354 MB | URL + sha256 |
| faster-whisper `medium.en` | 1.53 GB | **pinned to commit `a29b04bd1538…`**, all 4 files hashed |
| faster-whisper `small.en` | 484 MB | **pinned to commit `d1d751a5f827…`** |
| Ollama `qwen2.5:7b` | 4.68 GB | digest `845dbda0ea48` + the GGUF fallback in §4 |
| `~/Downloads/turtlebot4_humble_lite_1.0.5.zip` | 2.59 GB | vendor SD image, re-downloadable from Clearpath |
| ROS Humble, all 399 packages | — | **snapshot `2026-05-14`, verified to match exactly** (`env/ROS_SNAPSHOT.md`) |
| the 64 perception pip packages | — | `env/pip-freeze_yolo-env.txt`, installed by `docker/perception.Dockerfile` |

### Superseded — explicitly do NOT back these up

| Item | Size | Why not |
|---|---|---|
| `localhost/ubuntu22-snapshot:latest` | 9.88 GB | `docker/ros.Dockerfile` rebuilds it from the verified snapshot. This was the project's biggest reproducibility hole and Phase 3 closed it. |
| `~/yolo-env/` | — | `docker/perception.Dockerfile` replaces it, from the recorded pins. It also removes the host-venv-inside-a-22.04-container trick that would not have survived a rebuild anyway. |
| `~/.local/share/containers` | 9.8 GB | both distrobox containers, rebuildable from `docker/`. Checked for files written inside that are not in `$HOME`: none project-related. |

---

## 4. Ollama fallback — if the tag stops matching

`env/MANIFEST.md` pins `qwen2.5:7b` to digest `845dbda0ea48`, but a *tag* can be
re-pointed upstream at any time. Confirmed from the running Ollama: the model is
**Qwen2.5-7B-Instruct, Q4_K_M, GGUF, 7.6B parameters**.

The equivalent weights, addressed by content rather than by tag:

**`Qwen/Qwen2.5-7B-Instruct-GGUF`**, repo revision
`bb5d59e06d9551d752d08b292a50eb208b07ab1f`. The Q4_K_M build is split in two:

| File | Size (B) | sha256 |
|---|---|---|
| `qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf` | 3 993 201 344 | `dfce12e3862a5283ccfb88221b48480e58745165de856439950d0f22590580db` |
| `qwen2.5-7b-instruct-q4_k_m-00002-of-00002.gguf` | 689 872 288 | `539cf93f78e887edea1c04e2d7d8cdaca9d01dae9c9025bcb8accbe29df3d72a` |

The two sum to **4 683 073 632 B** against Ollama's reported **4 683 087 332 B** —
a 13.7 KB difference, which is Ollama's own manifest overhead. That corroborates
that these are the same weights.

```bash
# only if `ollama pull qwen2.5:7b` stops giving digest 845dbda0ea48
REV=bb5d59e06d9551d752d08b292a50eb208b07ab1f
BASE=https://huggingface.co/Qwen/Qwen2.5-7B-Instruct-GGUF/resolve/$REV
for n in 00001-of-00002 00002-of-00002; do
  f=qwen2.5-7b-instruct-q4_k_m-$n.gguf
  curl -fL --retry 3 -o "$f" "$BASE/$f"
done
sha256sum -c <<'SUMS'
dfce12e3862a5283ccfb88221b48480e58745165de856439950d0f22590580db  qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf
539cf93f78e887edea1c04e2d7d8cdaca9d01dae9c9025bcb8accbe29df3d72a  qwen2.5-7b-instruct-q4_k_m-00002-of-00002.gguf
SUMS

# then register it with Ollama
printf 'FROM ./qwen2.5-7b-instruct-q4_k_m-00001-of-00002.gguf\n' > Modelfile
ollama create qwen2.5-7b-pinned -f Modelfile
# and point the project at it
echo 'VLA_OLLAMA_MODEL=qwen2.5-7b-pinned' >> config/.env
```

---

## 5. secret — NOT committed 🔑

Recorded here as existence and location only. Nothing below is in the repository
and nothing below should be.

| Item | Where | What to do after the rebuild |
|---|---|---|
| SSH key `id_ed25519` | `~/.ssh/` | fingerprint `SHA256:y07BjLHH5/FFKAs728MTf8/ZvFqB2K7TnY6CQ7N4574`, comment `danyalaziz@workstation-vla`. **Generate a NEW key** and add it at github.com/settings/keys. Do not copy the old one to a new machine. Also note GitHub SSH needs port 443 here — port 22 is blocked (see §9). |
| Robot SSH access | key-based to `ubuntu@10.42.0.169` | the Pi holds the matching `authorized_keys`. A new PC key must be appended there — **and the Pi is not being wiped, so that is the only step** |
| Wi-Fi / hotspot | NetworkManager profiles `Hotspot` and `raspi` | recreate. **2.4 GHz channel 6**: 5 GHz hotspot is impossible on this card (Intel AX201 LAR firmware owns the regulatory domain) |
| Bluetooth pairings | `Airbud 595` `BB:86:57:3D:AF:79`, `realme Buds Air 2` `18:95:52:6B:5B:FD` | re-pair. **These MACs are hard-coded** in `scripts/vla_demo.sh` and `tools/hotspot_guard.sh`; different earbuds mean editing those |

A full secrets scan of everything committed found **nothing** — no passwords,
tokens, API keys, PSKs or key material. The four pattern matches were all false
positives ("passwordless sudo" in an error string, "text tokens" in a CLIP
comment, `tokenizers==0.19.1` as a package name, and a comment saying "contains
no secrets").

---

## 6. The robot (Raspberry Pi) — NOT COLLECTED ⬜

**The Pi is in scope as a read-only source. It was not reachable during this
audit, so nothing was collected.**

```
$ timeout 8 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22"
  10.42.0.169:22 NOT reachable
$ ip -brief addr show wlp0s20f3
  wlp0s20f3  UP  192.168.137.156/24
```

The cause is visible in that second line: the PC's Wi-Fi is **joined to another
network** (`192.168.137.x`) rather than hosting the hotspot, so the robot's
`10.42.0.x` subnet does not exist right now. The robot may well be fine.

### What must be collected when it is reachable

Read-only — **change nothing on the Pi.** Run from the host:

```bash
PI=ubuntu@10.42.0.169
D=robot_pi && mkdir -p $D/{etc,systemd,ros,info}

# the discovery server setup -- this is the one that matters most
scp $PI:/etc/turtlebot4_discovery/setup.bash        $D/etc/
scp $PI:/etc/turtlebot4_discovery/*.xml             $D/etc/ 2>/dev/null
# the turtlebot4 configuration
scp $PI:/etc/turtlebot4/*                           $D/etc/ 2>/dev/null
# the units we restart in vla_demo.sh stage 2
ssh $PI 'systemctl cat discovery.service turtlebot4.service' > $D/systemd/units.txt
# apt timers were masked to stop unattended-upgrades hitting load 9.4
ssh $PI 'systemctl list-unit-files --state=masked' > $D/systemd/masked.txt
# what is actually installed and running, for the record
ssh $PI 'dpkg -l | grep -E "ros-humble|turtlebot4" | awk "{print \$2, \$3}"' > $D/info/packages.txt
ssh $PI 'uname -a; vcgencmd measure_temp; vcgencmd get_throttled; uptime' > $D/info/state.txt
ssh $PI 'cat /etc/netplan/*.yaml 2>/dev/null' > $D/etc/netplan.txt
ssh $PI 'ls -la ~/ ; cat ~/.bashrc' > $D/info/home.txt

# then diff against stock and commit only what differs
git add robot_pi && git commit -m "robot_pi: Pi-side configuration, read-only capture"
```

**If it is never collected before the wipe, this is what is lost:** the Pi-side
discovery-server configuration and the record of which units were masked. The Pi
itself is not being wiped, so the files still exist on it — the risk is only that
nobody records *what* was changed from stock. `CLAUDE.md` and
`docs/changelogs/CHANGELOG_2026-09-08.md` §6 already describe the discovery
setup in prose, which is a partial mitigation.

---

## 7. A second, forgotten clone

`~/Documents/AI-guided-Ground-Operation-robot-using-VLA/` is **another clone of
this repository**, sitting at the June commit `2818cea` and 4 files long. It had
uncommitted work:

- **`Optimizations.docx`, 66 KB, untracked — in no branch and nowhere else.**
  Rescued to `docs/deliverables/Optimizations.docx`. This is the single item the
  audit found that was genuinely at risk of being lost.
- `openvla_pictest.py`, modified: the change is one comment reword
  ("CHANGE THESE TWO LINES" → "TWO MORE LINES"). Not worth keeping. It also
  references `~/Pictures/VLA /table.jpeg`, a path that no longer exists.

Delete that clone, or leave it — nothing in it is needed now.

---

## 8. Re-check script

```bash
bash docs/verify_backup.sh
```

Checks the two outstanding items: that the USB/Drive destination holds the
expected video and deliverable files, and whether the robot is reachable so the
Pi capture can run. It changes nothing.

---

## 9. After the rebuild — in this order

1. `git clone git@github.com:DANIAZIZ-png/AI-guided-Ground-Operation-robot-using-VLA.git`
   — **generate a new SSH key first**, add it to GitHub, and note **port 22 is
   blocked** on this network. Put this in `~/.ssh/config`:
   ```
   Host github.com
     HostName ssh.github.com
     Port 443
     User git
   ```
2. `make setup` — installs the tool venv, downloads and **hash-verifies** every
   weight, builds both images.
3. `make test` — expect `133 passed` and `PARITY OK`.
4. Recreate the 2.4 GHz hotspot on **channel 6**, re-pair the earbuds, append the
   new public key to the Pi's `authorized_keys`.
5. GPU in containers needs the NVIDIA Container Toolkit and a CDI spec:
   `sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml`.
6. `make robot` with the robot docked, or `scripts/vla_demo.sh` for the original
   13-stage bring-up.

---

## Scope note

The instruction for this phase was "exactly as I specified earlier". **No earlier
specification exists on this disk** — all 11 Claude transcripts for this project
(~43 MB) were searched for `BACKUP_CHECKLIST`, "safe to wipe", "pre-wipe",
"re-downloadable" and "release asset", and the only match is the message that
asked for the phase. This checklist is built from that message plus the two
follow-up answers about the Pi and the media, and nothing is inferred beyond
them. The **release asset** bucket ended up empty: everything large is either
re-downloadable by hash or belongs on USB/Drive, so nothing needs a GitHub
release.
