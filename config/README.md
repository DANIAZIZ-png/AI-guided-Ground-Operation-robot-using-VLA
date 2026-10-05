# config/ — where these files actually live on the host

These are dotfiles in `$HOME` on the running machine. They are copied here with
their original names so the snapshot is verbatim. Nothing reads them from this
directory.

| File here | Host path | What it does |
|---|---|---|
| `fastdds_hw_wifi_only.xml` | `~/.ros/fastdds_hw_wifi_only.xml` | **the hardware profile in use.** Fast DDS super-client + interface whitelist pinned to `10.42.0.1`, so DDS traffic cannot leak onto another interface. Set by `robot_env.sh` and `robot_mode.sh`. |
| `fastdds_super_client.xml` | `~/.ros/fastdds_super_client.xml` | the earlier profile, no whitelist. Superseded but still referenced by `vla_robot.sh` and `.bashrc`; left untouched on the host. |
| `.vla_gui.json` | `~/.vla_gui.json` | the GUI's live config — whichever mode was last used. |
| `.vla_gui.sim.json` | `~/.vla_gui.sim.json` | simulation GUI config; copied over `.vla_gui.json` by `vla_sim.sh`. |
| `.vla_gui.robot.json` | `~/.vla_gui.robot.json` | hardware GUI config; copied over `.vla_gui.json` by `vla_robot.sh` and `vla_demo.sh`. |

## Why there are two DDS profiles

Simulation and hardware need **opposite** discovery settings and each fails
*silently* under the other's — no error, just an empty topic list. On hardware
both of these must be set together:

```
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_hw_wifi_only.xml
export ROS_DISCOVERY_SERVER="10.42.0.169:11811;"
```

Sourcing `/etc/turtlebot4_discovery/setup.bash` alone is **not** enough in a
non-interactive shell: it sets `ROS_SUPER_CLIENT=False` when no terminal is
attached, which yields a plain client that receives topic data but no discovery
directory. The XML profile forces super-client mode regardless of that flag.
See `CLAUDE.md` and `docs/CHANGELOG_2026-09-08.md` §6.
