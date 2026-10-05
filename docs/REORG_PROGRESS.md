# Reorg progress

Branch **`reorg`**, off `main`. Advisor deadline: **Tuesday**.

Goal: make the repository reproducible, modular and portable without changing
behaviour — same models, same parameters, same Nav2/SLAM configs.

**Never touch** branch `snapshot-as-run` or tag `v1.0-fydp-snapshot`.
**Never edit anything in `~` outside this repository.** Nothing outside the repo
has been modified at any point; the live `~/vla_sim.sh` demo is untouched.

| Phase | What | Status |
|---|---|---|
| 1 | Remove hard-coded paths | **DONE** |
| 2 | Modular code | **DONE except `vla_bringup`** |
| 3 | Containers (Podman + Docker) | **NEXT** |
| 4 | One-command use (Makefile) | not started |
| 5 | Tests and CI | partly done (54 tests; no ruff/CI yet) |
| 6 | Prove portability (fresh clone) | not started |
| 7 | Docs | not started |

Report to the user after phases **1**, **3** and **6**. Phase 1 reported.

## How to verify anything, from a cold start

```bash
distrobox enter ubuntu22-gpu -- bash -c '
  source /opt/ros/humble/setup.bash
  cd ~/repos/AI-guided-Ground-Operation-robot-using-VLA
  python3 tests/snapshot_agent_api.py vla_agent_v28 --compare tests/baseline_agent_api.json
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
'
```

Expect `PARITY OK` and `54 passed`. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is
required: the container's pytest is 6.2.5 and a user-site `anyio` plugin wants a
newer `_pytest.scope`, which otherwise aborts collection before any test runs.

---

## Phase 1 — DONE

`grep -rn "/home/danyalaziz" src scripts launch config tools` returns nothing.

| File | Purpose |
|---|---|
| `config/paths.sh` | resolves `VLA_ROOT` from its own location, derives `VLA_CONFIG_DIR`, `VLA_SRC_DIR`, `VLA_TOOLS_DIR`, `VLA_LAUNCH_DIR`, `VLA_MAP_DIR`, `VLA_LOG_DIR`, `VLA_MODEL_DIR`; provides `vla_ros_setup`, `vla_shm_clean` |
| `config/robot.env`, `config/sim.env` | variables only — safe in a `bash -c` wrapper |
| `config/.env.example` | documented template; `config/.env` is git-ignored, read automatically |
| `scripts/vla_mode.sh` | `source scripts/vla_mode.sh robot⎮sim` — variables **plus** shm clean, daemon restart, banner |
| `src/vla_paths.py` | the Python counterpart |

`robot_mode.sh`, `sim_mode.sh`, `robot_env.sh` are thin wrappers, so all
existing callers still work. Defaults: `VLA_LOG_DIR` → `<repo>/logs`,
`VLA_MODEL_DIR` → `<repo>/models`, both git-ignored.

The env files are sourceable shell, not `KEY=value`, because `sim.env` must
**unset** `ROS_DOMAIN_ID`, `FASTRTPS_DEFAULT_PROFILES_FILE` and
`ROS_DISCOVERY_SERVER` — a dotenv file cannot express an unset, and leaving them
set is what makes the simulation hang silently.

---

## Phase 2 — DONE except `vla_bringup`

`src/vla_agent_v28.py` went from **4,730 lines to 9 files, largest 1,023**:

| Module | Lines | Contents |
|---|---|---|
| `vla_agent/core.py` | 890 | 147 constants, `AGENT_VERSION`, headless-safe `print`, module helpers |
| `vla_agent/ros_io.py` | 274 | 22 methods — callbacks, publishers, replies, status |
| `vla_agent/perception_client.py` | 948 | 28 methods — camera pairing, YOLO client, ranging, annotated feed |
| `vla_agent/navigation.py` | 857 | 32 methods — grid, goal choice, Nav2 + Dock clients, frontiers |
| `vla_agent/guards.py` | 162 | 6 methods — clearance, collision braking, stuck detection |
| `vla_agent/state_machine.py` | 1,023 | 23 methods — intake, dispatch, think steps |
| `vla_agent/agent.py` | 391 | `VLAAgent`: `__init__` plus the five mixins |
| `vla_agent/main.py` | 47 | entry point |
| `vla_agent_v28.py` | 280 | thin shim |

MRO: `VLAAgent → StateMachineMixin → NavigationMixin → PerceptionMixin →
RosIoMixin → GuardsMixin → Node`. All 111 methods accounted for; `__init__`
stays on the node. Source moved byte-for-byte with its comments.

The other four modules are packages too, each with a shim at the old path:

| Package | From | Entry point |
|---|---|---|
| `vla_brain/brain.py` | `llm_brain.py` | `vla-brain` |
| `vla_perception/server.py` | `yolo_server.py` | `vla-perception` |
| `vla_voice/listener.py` | `voice_command.py` | `vla-voice` |
| `vla_gui/app.py` | `vla_gui_v2.py` | `vla-gui` |

`src/pyproject.toml` installs all five with `pip install -e src`. **One
pyproject, not five** — five distributions would mean five near-identical files,
five versions to keep in step, and a re-pinned dependency between `vla_agent`
and `vla_brain` on every change, for packages that are always installed and
deployed together. Same import boundaries, same entry points. Split later if any
package ever ships alone.

### The shims are load-bearing

Every launcher names the old paths: `scripts/vla_demo.sh`,
`tools/agent_xterm.sh`, `tools/restart_agent_hw.sh`, `scripts/run_agent_sim.sh`,
`scripts/vla_sim.sh`, `scripts/vla_robot.sh`. They were deliberately not
rewritten, so Phase 2 cannot have broken the bring-up.

### Still to do in Phase 2

`vla_bringup`: a ROS 2 package wrapping `launch/`, `config/` and `maps/`, with
`sim.launch.py` and `robot.launch.py`. Not started. The launch files work as
they are (`ros2 launch launch/nav2_hw_composed.launch.py`), so this is tidying
rather than a blocker — but it is what makes `ros2 launch vla_bringup ...` work,
and Phase 3's containers will want it.

---

## How the split was verified, and what that caught

The brief said to run the sim after each step. `CLAUDE.md` forbids running
`~/vla_sim.sh`, and a full headless sim needs Gazebo + Nav2 + SLAM + YOLO +
Ollama, minutes per attempt, with the TurtleBot 4 ignition launch hard-coding
its own `ign_args` so true headless would need `xvfb` (an apt install). Instead:

**`tests/snapshot_agent_api.py` — the parity gate.** Snapshots the module's
public surface (constants *with their values*, function signatures, class
method lists, MRO) and diffs it against `tests/baseline_agent_api.json`, taken
from the monolith before anything moved. ~1 s, no Gazebo, no GPU, no robot.
Additions pass; losses and changes fail by name.

**`tests/extract_mixin.py`** did the moves mechanically. Hand-editing 112
methods across 3,900 lines is where a decorator or trailing comment gets
dropped.

**It caught two real regressions.**

1. `AGENT_VERSION` was dropped when the class and `main()` moved out of the
   shim. `py_compile` passed it — a missing name is a runtime error.
2. Chasing that exposed an older instance of the same bug, present since step 1:
   `core.py`'s `MissionLog.log()` referenced `AGENT_VERSION` while the constant
   was still in the shim, so it was never in `core`'s globals. The parity gate
   said OK, because the name *was* on the shim. Nothing would have complained
   until the agent wrote its first mission log line — **on the robot,
   mid-demo.**

`pyflakes` cannot catch either: these modules use `from .core import *`, and a
star import makes it abandon undefined-name analysis.

So the blind spot is now closed by **`tests/test_package_globals.py`** (10
tests): for each module, import it and assert every ALL_CAPS name it *reads*
resolves there; plus a check that every module calling `print()` has `core`'s
`print` and not the builtin — losing that silently reinstates the broken-pipe
crash fix #63 exists to prevent.

### Test inventory — 54 passing

| File | Tests | Needs ROS? |
|---|---|---|
| `test_agent_helpers.py` | 32 | yes |
| `test_parity_gate.py` | 12 | **no** — CI-ready |
| `test_package_globals.py` | 10 | yes |

`test_parity_gate.py` tests the gate itself, including that it *does* fail on a
lost constant, a changed constant value, a lost method and a broken inheritance
chain. A gate that cannot fail proves nothing.

---

## Decisions taken

- **LICENSE: MIT** (chosen by the user). Not yet written — Phase 5.
- **`test_62_guard_direction.py` / `test_63_broken_pipe.py` do not exist
  anywhere on disk.** Searched the whole filesystem (`find / -xdev`). New tests
  covering the same behaviour are labelled as new in their file headers, as
  instructed; they are not presented as the originals. Fix #63's behaviour is
  covered by `test_headless_safe_print_still_shadows_builtin` and
  `test_the_headless_safe_print_is_present_in_every_module_that_prints`; fix
  #62's direction-aware guard by `test_off_front_is_zero_dead_ahead_and_symmetric`.

---

## Open issues found, NOT fixed (logic, not structure)

1. **`config/vla_gui.robot.json` launches the stock Nav2 and SLAM.** Its SLAM
   and Nav2 steps run `turtlebot4_navigation slam.launch.py` and
   `nav2.launch.py` — no `/parameter_events` or `/rosout` remaps, 4 s bond
   timeout. `CLAUDE.md` is explicit that this floods the robot's Wi-Fi at
   ~7,400 packets/s and aborts its own bring-up. The repo's own
   `launch/nav2_hw_composed.launch.py` and `launch/slam_hw.launch.py` are
   correct, and `vla_demo.sh` uses them. **Those GUI buttons look like a trap
   left from before the storm was diagnosed.** Decide: repoint them, or disable
   them since `vla_demo.sh` supersedes them.
2. The same config still names `fastdds_super_client.xml` (no interface
   whitelist) where `robot.env` uses `fastdds_hw_wifi_only.xml`.
3. **`nav2.yaml.today` and `slam.yaml.today` are not tuned** — byte-identical to
   the stock backups, while `CLAUDE.md` and the handout call them "the tuned
   versions". Phase 7 must correct that claim.
4. `slam_vla.yaml` sets `minimum_travel_distance: 0.2` while `CLAUDE.md` says it
   must stay at `0.0`. Unreconciled upstream; left alone.

---

## Phase 3 — NEXT

- `docker/ros.Dockerfile`: `ros:humble` + only the apt packages used, derived
  from `env/ros-humble-packages_ubuntu22-gpu.txt` (399 pinned). Check whether
  `snapshots.ros.org` has a Humble snapshot matching those versions; if not,
  record what differs in `env/`.
- `docker/perception.Dockerfile`: Python 3.12 + pins from
  `env/pip-freeze_yolo-env.txt` (torch 2.12.1, ultralytics 8.4.82, CLIP at
  `16be45c7062240d445cce764f2afd9454a91ef7e`). **This removes the
  host-venv-inside-a-22.04-container trick** documented in `env/MANIFEST.md`,
  which is the single most fragile thing in the current setup.
- Ollama: official image, pull `qwen2.5:7b`, script checks digest
  `845dbda0ea48`.
- `compose.yaml`: services `ollama`, `perception`, `sim`, `agent`, `gui`,
  `voice`; profiles `sim` and `robot`; robot profile uses host networking and
  mounts the Fast DDS XML.
- **GPU under Podman needs the NVIDIA Container Toolkit (CDI), which is not
  installed** (`nvidia-ctk` is absent; confirmed in Phase 1's survey). The exact
  `sudo` commands go to the user — do not run them.
- `scripts/download_models.sh`: fetch the four weights into `./models` and verify
  every sha256 in `env/MANIFEST.md`.
