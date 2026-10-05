# archive/ — superseded, kept for provenance

Nothing in here is live code. It is kept so that intermediate results in the
report can be traced to the version that produced them.

## Why the old agents are *not* in here

`vla_agent.py`, `vla_agent_improved.py` and `vla_agent_v9` … `v27` were already
committed at the top level of this repository in August and September 2026.
Moving them would show up as deletions, so they have been **left exactly where
they were**. They are history, not alternatives — the live agent is
`vla_agent_v28.py`.

The `reorg` branch will tidy this.

## Contents

| File | Superseded by | Note |
|---|---|---|
| `vla_agent_unified_intelligent.py` | `vla_agent_v28.py` | June 2026 prototype |
| `llm_brain_unified.py` | `llm_brain.py` | early single-file brain |
| `frontier_explorer.py` | — | autonomous exploration experiment, not used in the demo |
| `vla_server.py` | `vla_agent_v28.py` | early client/server split |
| `yolo_camera_node.py` | `yolo_server.py` | ROS-node detector, replaced by the socket server |
| `yolo_test.py` | `vla_tools/yolo_live_test.py` | manual probe |
| `ros_stubs.py` | — | stubs used before ROS was available |
| `sd_flash.sh` | — | one-off SD-card flashing helper |
| `nav2_fixed.yaml` | stock `/opt` `nav2.yaml` | early hand-tuned params |
| `nav2.yaml.pre_v25` | stock `/opt` `nav2.yaml` | backup taken before the v25 experiment |
| `nav2.yaml.today` | — | tuned params, **never put into service** |
| `slam.yaml.today` | — | tuned params, **never put into service** |
| `CLAUDE.md.bak-*` (3) | `CLAUDE.md` | operating-manual history |
| `PROJECT_HANDOUT_v5.md.bak-*` (7) | `docs/PROJECT_HANDOUT_v5.md` | handout history |
| `vla_tools/*.bak-*` (4) | the live scripts in `vla_tools/` | pre-edit copies |

## On the Nav2 and SLAM params

`/opt`'s `nav2.yaml` and `slam.yaml` were deliberately **restored to stock**. The
tuned versions here were not in use when the demo was recorded. The live SLAM
parameters are `slam_vla.yaml` at the top level, and the gentler-speed profile
that *is* usable is `nav2_hw_slow.yaml`, also at the top level.
