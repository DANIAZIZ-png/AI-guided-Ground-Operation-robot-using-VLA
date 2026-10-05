# config/

| File | Host path | What it does |
|---|---|---|
| `fastdds_hw_wifi_only.xml` | `~/.ros/fastdds_hw_wifi_only.xml` | **the hardware DDS profile in use.** Fast DDS super-client plus an interface whitelist pinned to `10.42.0.1`, so DDS traffic cannot leak onto another interface. Set by `scripts/robot_env.sh` and `scripts/robot_mode.sh`. |
| `fastdds_super_client.xml` | `~/.ros/fastdds_super_client.xml` | the earlier profile, no whitelist. Superseded, but still referenced by `scripts/vla_robot.sh` and by `~/.bashrc`, so it was deliberately left in place on the host. |
| `vla_gui.sim.json` | `~/.vla_gui.sim.json` | simulation GUI config. `vla_sim.sh` copies it over `~/.vla_gui.json`. |
| `vla_gui.robot.json` | `~/.vla_gui.robot.json` | hardware GUI config. `vla_robot.sh` and `vla_demo.sh` copy it over `~/.vla_gui.json`. |
| `slam_vla.yaml` | `~/slam_vla.yaml` | the live SLAM Toolbox parameters. |
| `nav2_hw_slow.yaml` | `~/nav2_hw_slow.yaml` | gentler speeds and accelerations (0.15 m/s, 0.4 rad/s). Pass with `params_file:=`. |
| `vla_hardware.rviz` | `~/vla_hardware.rviz` | the hardware RViz layout. |

`~/.vla_gui.json` itself is not kept here: it is whichever of the two configs was
copied in last, and it was byte-identical to `vla_gui.robot.json`.

## Why there are two DDS profiles

Simulation and hardware need **opposite** discovery settings, and each fails
*silently* under the other's — no error message, just an empty topic list. On
hardware, both of these must be set, together:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_hw_wifi_only.xml
export ROS_DISCOVERY_SERVER="10.42.0.169:11811;"
```

Sourcing `/etc/turtlebot4_discovery/setup.bash` on its own is **not enough** in a
non-interactive shell. It sets `ROS_SUPER_CLIENT=True` only when a terminal is
attached, so inside any `bash -c` wrapper you get a plain client: it receives
topic data but no discovery directory, which shows up as an empty topic list
while everything is actually fine. The XML profile forces super-client mode
regardless of that flag, which is why both are needed.

See `../CLAUDE.md` and `../docs/changelogs/CHANGELOG_2026-09-08.md` §6.

## On the Nav2 remaps

Neither Nav2 launch file in `../launch/` may be replaced by the stock
`turtlebot4_navigation nav2.launch.py`. The stock one has no `/parameter_events`
or `/rosout` remaps and a 4-second bond timeout: it floods the Pi's Wi-Fi link
at ~7,400 packets/s and then aborts its own bring-up. Packet captures are in
`../results/evidence/`.

Do not try to solve that with XML QoS overrides — `rmw_fastrtps` on Humble
ignores them. This was tested; the attempt is kept as
`../results/evidence/qos_override_attempt.xml`.
