# Reorg progress

Branch **`reorg`**, off `main`. Advisor deadline: **Tuesday**.

Goal: make the repository reproducible, modular and portable without changing
behaviour — same models, same parameters, same Nav2/SLAM configs.

**Never touch** branch `snapshot-as-run` or tag `v1.0-fydp-snapshot`.
**Never edit anything in `~` outside this repository** — the live `~/vla_sim.sh`
demo must keep working. It does: every host script still refers to its own
absolute paths, and nothing outside the repo has been modified.

| Phase | What | Status |
|---|---|---|
| 1 | Remove hard-coded paths | **DONE** |
| 2 | Modular code (packages + entry points) | **NEXT** |
| 3 | Containers (Podman + Docker) | not started |
| 4 | One-command use (Makefile) | not started |
| 5 | Tests and CI | not started |
| 6 | Prove portability (fresh clone) | not started |
| 7 | Docs | not started |

Report to the user after phases **1**, **3** and **6**.

---

## Phase 1 — DONE

Acceptance test passes:

```
$ grep -rn "/home/danyalaziz" src scripts launch config tools
(no output)
```

Verified: `bash -n` on 29 shell files, `python3 -m py_compile` on 73 Python
files, path resolution tested from an unrelated working directory, and
`VLA_LOG_DIR` / `VLA_MODEL_DIR` overrides confirmed to be honoured.

### What was added

| File | Purpose |
|---|---|
| `config/paths.sh` | single source of truth. Resolves `VLA_ROOT` from its own location, then derives `VLA_CONFIG_DIR`, `VLA_SRC_DIR`, `VLA_TOOLS_DIR`, `VLA_LAUNCH_DIR`, `VLA_MAP_DIR`, `VLA_LOG_DIR`, `VLA_MODEL_DIR`. Also provides `vla_ros_setup` and `vla_shm_clean`. No side effects beyond creating the log directory. |
| `config/robot.env` | hardware variables only — safe in a `bash -c` wrapper |
| `config/sim.env` | simulation variables only |
| `config/.env.example` | documented template; `config/.env` is git-ignored and read automatically by `paths.sh` |
| `scripts/vla_mode.sh` | `source scripts/vla_mode.sh robot|sim` — env **plus** the three terminal-only side effects (shm clean, daemon restart, banner) |
| `src/vla_paths.py` | the Python counterpart of `paths.sh` |

### Defaults

`VLA_LOG_DIR` → `<repo>/logs`, `VLA_MODEL_DIR` → `<repo>/models`, both
git-ignored. Everything else defaults to its previous value, so no behaviour
changed.

### Why the env files are sourceable shell, not plain `KEY=value`

`sim.env` has to **unset** `ROS_DOMAIN_ID`, `FASTRTPS_DEFAULT_PROFILES_FILE` and
`ROS_DISCOVERY_SERVER`. A dotenv file cannot express an unset, and leaving those
three set is exactly the failure that makes the simulation hang silently. They
are therefore `. `-able shell files.

### Why the side effects are separate from the variables

`robot_mode.sh` did two different jobs: set variables, and restart the ROS
daemon. The daemon restart blinds every other terminal for ~10 s, so it must
never run inside a wrapper. Phase 1 splits them:

- **variables only** → `config/robot.env` / `config/sim.env` (use in scripts,
  `xterm -e` launchers, `bash -c`)
- **variables + side effects + banner** → `scripts/vla_mode.sh` (use in a
  terminal you type into)

`robot_mode.sh`, `sim_mode.sh` and `robot_env.sh` are kept as thin wrappers, so
every existing caller still works.

### One genuine portability fix, not just a path move

`tools/voice_xterm.sh` hard-coded
`/home/danyalaziz/.local/lib/python3.10/site-packages` for the pip cuBLAS/cuDNN
libraries. That path is wrong on any other machine and in any other container,
and getting it wrong makes **every transcription fail** with
`libcublas.so.12 not found`. It now asks Python where its `nvidia/` package
actually is, warns clearly if it cannot find it, and accepts
`VLA_SITE_PACKAGES` as an override.

### Behaviour preserved deliberately

- `src/yolo_server.py` uses `vla_paths.model_path("yolov8s-world.pt")`, which
  returns `$VLA_MODEL_DIR/yolov8s-world.pt` **only if that file exists** and the
  bare filename otherwise. A checkout with no `models/` directory therefore
  behaves exactly as before (ultralytics resolves the bare name against the
  working directory).
- `src/vla_gui_v2.py` reads `VLA_GUI_CONFIG`, defaulting to `~/.vla_gui.json`,
  so an existing installation keeps the config it already has.
- `config/vla_gui.*.json` had their paths updated but **not** which launch files
  their steps run. See the open issue below.

---

## Open issues found in Phase 1 (not fixed — they are logic, not structure)

1. **`config/vla_gui.robot.json` launches the stock Nav2 and SLAM.** Its SLAM
   and Nav2 steps run `turtlebot4_navigation slam.launch.py` and
   `nav2.launch.py`, which have **no `/parameter_events` or `/rosout` remaps**
   and a 4 s bond timeout. `CLAUDE.md` is explicit that this floods the robot's
   Wi-Fi at ~7,400 packets/s and aborts its own bring-up. The repository's own
   `launch/nav2_hw_composed.launch.py` and `launch/slam_hw.launch.py` are the
   correct ones, and `vla_demo.sh` uses them. Those GUI buttons look like a trap
   left over from before the storm was diagnosed.
   **Decide in Phase 2:** point the GUI steps at the repo's launch files, or
   disable them as `vla_demo.sh` already supersedes them.
2. **The same config still names `fastdds_super_client.xml`**, the profile
   *without* the interface whitelist, where `robot.env` now uses
   `fastdds_hw_wifi_only.xml`. Only the path was updated, not the choice.
3. **`nav2.yaml.today` / `slam.yaml.today` are not tuned.** They are
   byte-identical to the stock backups. `CLAUDE.md` and the handout both call
   them "the tuned versions". Phase 7 must correct that claim.
4. **`slam_vla.yaml` sets `minimum_travel_distance: 0.2`** while `CLAUDE.md`
   says it must stay at `0.0`. Unreconciled upstream; left alone.

---

## Phase 2 — NEXT

Split `src/vla_agent_v28.py` (**4,730 lines**, 4 classes, 128 functions) into
`ros_io`, `perception_client`, `navigation`, `guards`, `state_machine` and a thin
main node; turn `src/` into packages with `pyproject.toml` entry points; move
`launch/`, `config/` and `maps/` into a `vla_bringup` ROS 2 package with
`sim.launch.py` and `robot.launch.py`.

### ⚠ Blocker to raise before starting Phase 2

The task says *"After each step, run the sim and check that it still behaves the
same."* **I cannot run the simulation.** `CLAUDE.md` says:

> `~/vla_sim.sh` — ends with the GUI in the foreground; it will hang you
> forever. I run this one myself.

So each agent-splitting step needs the user to run the sim and report back, or
an agreed alternative (a headless smoke test, which is what Phase 5's
`tests/smoke_sim.sh` is for). **Ask before splitting the agent.** Splitting a
4,730-line state machine with no behavioural check between steps is the one part
of this plan that could silently break the demo.

Suggested order, smallest risk first: `guards` (pure functions) → `navigation`
→ `perception_client` → `ros_io` → `state_machine`.
