# Changelog — 5 October 2026 (repository reorg, branch `reorg`)

Work on making the repository reproducible, modular and portable. No robot
session: nothing here was run on the hardware. Everything is verified by the
tests named against each item.

---

## 1. config fix: GUI robot buttons now use the storm-fixed launch files

**`config/vla_gui.robot.json`** — the SLAM and Nav2 buttons in the operator
console pointed at the **stock** `turtlebot4_navigation` launch files:

```
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot1
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
```

Those have **no `/parameter_events` or `/rosout` remap** and a **4 s bond
timeout**. Pressing either button reproduces the 8 and 10 September failure
exactly: Fast DDS floods the Raspberry Pi at ~7,400 packets/s (2 MB/s), every Pi
node burns CPU, the Pi throttles at 84 °C, camera frames stop, and Nav2's
lifecycle manager shoots its own bring-up dead. Diagnosed 11 September; packet
captures in `results/evidence/`.

They now run this repository's own launch files:

| Button | Now runs | Carries |
|---|---|---|
| SLAM | `launch/slam_hw.launch.py` | `/parameter_events` → `/pc/parameter_events`, `/rosout` → `/pc/rosout` |
| Nav2 | `launch/nav2_hw_composed.launch.py` | both remaps, `bond_timeout` 30 s, 20 s delayed STARTUP |

**Why it survived this long:** the bring-up script `scripts/vla_demo.sh` was
fixed on 11 September to use the repository's launch files. The GUI buttons were
left behind, and a JSON config is not covered by any test that imports code, so
nothing complained. On this machine the buttons were simply never used, because
the demo script supersedes them.

It is fixed now rather than noted because this PC is being wiped and the
repository is the only copy — shipping it would hand the storm to whoever
rebuilds the system.

### Also changed in the same file

- **Every `where: here` step now sources `config/robot.env`** instead of
  exporting a few variables by hand. That is what selects
  `fastdds_hw_wifi_only.xml` (super client **plus** the interface whitelist)
  over the older `fastdds_super_client.xml` the steps had been naming, and it
  brings the namespace and compressed-transport settings with it. Each step has
  to stand alone, because the GUI hands its own environment to every child —
  and after a rebuild there is no `~/.bashrc` setting any of this.
- **The agent and voice steps now exec the same launchers the demo uses**
  (`tools/agent_xterm.sh`, `tools/voice_xterm.sh`) rather than invoking Python
  directly. So the hardware-only `VLA_*` flags and the cuBLAS/cuDNN
  `LD_LIBRARY_PATH` fix apply here too, instead of being duplicated in a config
  file that then drifts. Without that `LD_LIBRARY_PATH`, every transcription
  fails with `libcublas.so.12 not found` — 10 September's voice bug.

### Verified

`tests/test_gui_config.py`, 11 tests, no ROS or GPU needed:

- the hardware config never names the stock `nav2.launch.py` or
  `slam.launch.py` — **checked against the previous version of the file, where
  it fails**, so the test is known to catch the real thing
- the Nav2 button uses the composed launch file, the SLAM button the remapped one
- every `here` step sources `config/robot.env`
- the launch files it names exist in the repository
- those launch files still carry **both** remaps, and the composed Nav2 keeps a
  `bond_timeout` default well above the stock 4 s — a launch file that quietly
  lost the remaps would otherwise pass every other check while still storming
  the robot
- no config hard-codes a `/home/danyalaziz` path

The four commands were also expanded with the `VLA_*` environment **cleared**,
to imitate a freshly rebuilt machine, and every file each one names was
confirmed to exist.

### One supporting change outside the config

**`src/vla_gui/app.py`** exports the `VLA_*` paths into its own environment at
startup (via `vla_paths`, with `setdefault` so an explicit value still wins).
`Popen` inherits that environment, which is what lets the config refer to
`$VLA_ROOT` and `$VLA_LAUNCH_DIR` and still work when the GUI is started
directly rather than through `vla_demo.sh`.

---

## 2. Hard-coded paths removed (reorg Phase 1)

`grep -rn "/home/danyalaziz" src scripts launch config tools` returns nothing.

Everything derives from `VLA_ROOT`, resolved from the location of
`config/paths.sh` (shell) or `src/vla_paths.py` (Python), so it does not depend
on the caller's working directory and a clone works anywhere.

New: `config/paths.sh`, `config/robot.env`, `config/sim.env`,
`config/.env.example`, `scripts/vla_mode.sh`, `src/vla_paths.py`.
`robot_mode.sh`, `sim_mode.sh` and `robot_env.sh` are kept as thin wrappers.

Two notes worth keeping:

- The env files are **sourceable shell, not `KEY=value`**, because `sim.env` has
  to *unset* `ROS_DOMAIN_ID`, `FASTRTPS_DEFAULT_PROFILES_FILE` and
  `ROS_DISCOVERY_SERVER`. A dotenv file cannot express an unset, and leaving
  those set is what makes the simulation hang with no error.
- Variables and side effects are now separate. `config/*.env` sets variables
  only (safe in a `bash -c` wrapper); `scripts/vla_mode.sh` adds the
  shared-memory clean, the ROS daemon restart and the banner, which belong only
  in a terminal you type into — the daemon restart blinds every other terminal
  for ~10 s.

**`tools/voice_xterm.sh`** had `/home/danyalaziz/.local/lib/python3.10/site-packages`
hard-coded for the cuBLAS/cuDNN libraries. That path is wrong on any other
machine and in any other container, and getting it wrong makes every
transcription fail. It now asks Python where its `nvidia/` package is, warns
clearly if it cannot find it, and accepts `VLA_SITE_PACKAGES`.

---

## 3. The agent split into modules (reorg Phase 2)

`src/vla_agent_v28.py`, 4,730 lines, is now 9 files with the largest at 1,023.
All 111 methods moved into five mixins; `__init__` stays on the node; the source
was moved byte-for-byte with its comments. `src/vla_agent_v28.py` remains as a
thin shim because every launcher names it.

`llm_brain.py`, `yolo_server.py`, `voice_command.py` and `vla_gui_v2.py` are
packages too, each with a shim at its old path and a console entry point.

### Two latent bugs found by the parity gate

The split was checked with `tests/snapshot_agent_api.py`, which snapshots the
module's public surface — constants **with their values**, signatures, method
lists, MRO — against a baseline taken from the monolith before anything moved.

1. **`AGENT_VERSION` was dropped** when the class and `main()` moved out of the
   shim. `py_compile` passed it: a missing name is a runtime error.
2. Chasing that exposed an older instance of the same bug: `core.py`'s
   `MissionLog.log()` referenced `AGENT_VERSION` while the constant was still in
   the shim, so it was never in `core`'s globals. The gate said OK, because the
   name *was* on the shim. **Nothing would have complained until the agent wrote
   its first mission log line — on the robot, mid-demo.**

`pyflakes` catches neither: the modules use `from .core import *`, and a star
import makes it abandon undefined-name analysis. `tests/test_package_globals.py`
now closes that gap dynamically.

---

## 4. Tests

65 passing, all of them new.

| File | Tests | Needs ROS? |
|---|---|---|
| `test_agent_helpers.py` | 32 | yes |
| `test_parity_gate.py` | 12 | no |
| `test_package_globals.py` | 10 | yes |
| `test_gui_config.py` | 11 | no |

`test_parity_gate.py` tests the gate itself, including that it **does** fail on
a lost constant, a changed constant *value*, a lost method and a broken
inheritance chain. A gate that cannot fail proves nothing.

`test_62_guard_direction.py` and `test_63_broken_pipe.py` do not exist anywhere
on this filesystem — searched with `find / -xdev`. The new tests cover the same
behaviour and say in their headers that they are new; they are not presented as
the originals.

---

## 5. Still wrong elsewhere in the documentation

- **`nav2.yaml.today` and `slam.yaml.today` are not tuned.** They are
  byte-identical to the stock backups (`nav2.yaml.pre_v25`, `slam.yaml.bak`),
  while `CLAUDE.md` and `PROJECT_HANDOUT_v5.md` both call them "the tuned
  versions". Either the tuning was reverted or it was never written to those
  files. To be corrected in the handout.
- `slam_vla.yaml` sets `minimum_travel_distance: 0.2` while `CLAUDE.md` says it
  must stay at `0.0`. Unreconciled upstream; left alone.
