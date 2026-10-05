# archive/

**Nothing in here is live code.** It is kept so that intermediate results in the
report can be traced to the version that produced them. The live programs are in
`../src/`.

| Folder | Files | What |
|---|---|---|
| `agents/` | 23 | every earlier agent: `vla_agent.py`, `vla_agent_improved.py`, `vla_agent_unified_intelligent.py`, `vla_agent_v9` … `v27`, and `llm_brain_unified.py`. The live agent is `../src/vla_agent_v28.py`. |
| `openvla/` | 5 | the **abandoned OpenVLA baseline** — `Openvla_baseline.py`, the action translator, two interactive harnesses and `test_openvla.py`. The project is named after it, but the shipped system does not use it. |
| `experiments/` | 10 | things tried and dropped: `frontier_explorer.py` (autonomous exploration), `object_explorer/locator/navigator.py` (pre-agent navigation attempts), `vla_server.py` (early client/server split), `yolo_camera_node.py` (ROS-node detector, replaced by the socket server), `ros_stubs.py` (stubs used before ROS was available), `sd_flash.sh`. |
| `params/` | 3 | `nav2_fixed.yaml` (early hand-tuned Nav2), `nav2.yaml.pre_v25` and `slam.yaml.bak` (stock backups). |
| `docs/` | 10 | `CLAUDE.md.bak-*` and `PROJECT_HANDOUT_v5.md.bak-*` — the history of the two living documents. |
| `tools/` | 4 | pre-edit copies of scripts now in `../tools/`. |
| `report_snapshot_20260912/` | 11 | the frozen copies of the live code and docs taken on **12 September 2026** for the report bundle. These are *older versions*, not duplicates — e.g. `vla_agent_v28.py` here is 253,893 bytes against 255,465 live. |

## Two files were removed as exact duplicates

- `nav2.yaml.today` was **byte-identical** to `params/nav2.yaml.pre_v25`
- `slam.yaml.today` was **byte-identical** to `params/slam.yaml.bak`

`CLAUDE.md` describes those two as "the tuned versions, not in use". They are not
tuned: they are unmodified copies of the stock backups. Either the tuning was
reverted or it was never written to those files. Worth correcting in the handout.

## On the Nav2 and SLAM parameters

`/opt`'s `nav2.yaml` and `slam.yaml` were deliberately **restored to stock** before
the demo, so the stock files are what produced the recorded results. The live
SLAM parameters are `../config/slam_vla.yaml`, and the gentler-speed Nav2 profile
that *is* usable is `../config/nav2_hw_slow.yaml`.

One unreconciled oddity, recorded rather than quietly fixed: `slam_vla.yaml` sets
`minimum_travel_distance: 0.2`, which contradicts the rule in `CLAUDE.md` that it
must stay at `0.0` or the `map->odom` transform goes stale while the robot is
stationary. In practice the map frame stayed valid through 45+ minutes docked.
Neither was changed without a test.
