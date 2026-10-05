# Change log — 8 September 2026 — discovery-server visibility fix

> **Resuming? Read `~/RESUME_NEXT_SESSION.md` first** (written 8 Sep 13:20):
> where we stopped, the thermal gate, the exact bring-up sequence, and
> Task 3 order. This file is the detailed record behind it.

Written for the project evaluation. Every claim below was verified on the
live system today; nothing is taken from memory or documentation.

## 1. Files modified or created

### Modified

**`/home/danyalaziz/vla_kill.sh`** — three changes. Original saved as
`~/vla_kill.sh.bak-20260908`.

Change A, line 67. Before:

```
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
```

Now (lines 67–76):

```
if command -v fastdds >/dev/null 2>&1; then
    fastdds shm clean
else
    echo "  fastdds not on PATH -- stale shm not cleaned"
fi
```

Why. Two ROS programs on the same computer do not talk over the network.
Fast DDS gives each one a file under `/dev/shm` that works as its mailbox.
The old line deleted every mailbox, including those of programs still
running. A running program keeps its own copy, but nothing started afterwards
can find it any more. `fastdds shm clean` is the tool eProsima ships for
exactly this job: it deletes only mailboxes whose owner has died, which is the
"stale shared memory" the original comment was worried about, and leaves live
programs alone.

Change B, lines 69–72. Before:

```
ros2 daemon stop  >/dev/null 2>&1
sleep 2
ros2 daemon start >/dev/null 2>&1
```

Now (lines 78–99):

```
if ! command -v ros2 >/dev/null 2>&1; then
    echo "  ros2 not on PATH here (host?) -- daemon untouched"
else
    ros2 daemon stop
    sleep 2
    if [ -n "$ROS_DISCOVERY_SERVER" ] && [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        ros2 daemon start
        echo "  daemon restarted in ROBOT mode (server $ROS_DISCOVERY_SERVER)"
    else
        echo "  daemon STOPPED, not restarted: this terminal has no discovery-server"
        echo "  config. The next 'source ~/robot_mode.sh' (robot) or"
        echo "  'source ~/sim_mode.sh' (sim) restarts it correctly."
    fi
fi
```

Why. The ROS daemon is a background helper that answers `ros2 node list`,
`ros2 topic list` and `ros2 topic info`. It inherits the environment of
whichever terminal starts it, and there is only one of them, shared by every
terminal. The old lines restarted it from any terminal, silently. From a
terminal that had no discovery-server settings, that produced a daemon that
could not see the robot, and every other terminal then got empty answers. The
new version stops the daemon (to clear its stale cache, the original purpose)
but starts it again only when the terminal is verifiably in robot mode, and
says which case happened.

Change C, line 75. Before:

```
LEFT=$(pgrep -c -f "ign gazebo|robot_state_publisher|slam_toolbox|vla_agent" 2>/dev/null)
```

Now (lines 102–105):

```
SURVIVORS="ign gazebo|robot_state_publisher|slam_toolbox|vla_agent"
LEFT=$(pgrep -c -f "$SURVIVORS" 2>/dev/null)
```

Why. The failure branch a few lines later runs `pgrep -af "$SURVIVORS"`, but
`SURVIVORS` was never defined. An empty pattern matches everything, so if a
process ever survived the kill, the script would have printed every process on
the machine instead of the survivors. The pattern now lives in one variable
used by both the count and the listing.

**`/home/danyalaziz/robot_mode.sh`** — line 49. Original saved as
`~/robot_mode.sh.bak-20260908`. Same replacement as Change A above
(`rm -rf /dev/shm/...` → guarded `fastdds shm clean`), same reason. This was
the copy that did the damage today: it ran at about 11:08:51, after SLAM
(started 11:03:57) and RViz (started 11:07:16) were already running.

**`/home/danyalaziz/sim_mode.sh`** — line 73. Original saved as
`~/sim_mode.sh.bak-20260908`. Same replacement, same reason.

**`/home/danyalaziz/run_sim_stack.sh`** — line 31. Original saved as
`~/run_sim_stack.sh.bak-20260908`. Same replacement, same reason.

### Created

- `~/vla_kill.sh.bak-20260908`, `~/robot_mode.sh.bak-20260908`,
  `~/sim_mode.sh.bak-20260908`, `~/run_sim_stack.sh.bak-20260908` — exact
  copies of the four scripts before today's edits.
- `~/.claude/jobs/557b175b/tmp/robot_env.sh`, `slam_xterm.sh`,
  `rviz_xterm.sh` — temporary launchers used to reopen SLAM and RViz in
  xterm windows with the robot environment set explicitly. They are deleted
  when this Claude job is cleaned up. The SLAM and RViz windows keep running
  regardless; they have already read the scripts.
- `~/CHANGELOG_2026-09-08.md` — this document.
- A private note under `~/.claude/projects/.../memory/` — Claude's own
  reminder of today's findings, not a project file.

### Not modified

- `~/.bashrc` — untouched (last modified 12 Aug). See section 3.
- `~/.ros/fastdds_super_client.xml` — untouched (last modified 9 Aug). It is
  correct and was never the problem.
- `~/slam_vla.yaml` — not modified by me. Its timestamp is 11:04:52 today,
  one minute after you launched SLAM, so that change was yours.
- `CLAUDE.md`, `PROJECT_HANDOUT_v5.md` — untouched. Corrections are listed in
  section 6 for you to apply.
- Nothing under `/opt`.

## 2. The daemon root cause

The daemon that was running when I looked (process 51071, started at 10:30:25
today) had `ROS_SUPER_CLIENT=True`, `ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=0`
in its environment, and nothing else — no `ROS_DISCOVERY_SERVER`, no
`FASTRTPS_DEFAULT_PROFILES_FILE`. That exact combination is what `~/.bashrc`
leaves behind in a fresh interactive terminal inside `ubuntu22-gpu`: the
turtlebot4 setup file gives it the super-client flag and the domain, then the
`VLA_MODE` guard strips the two discovery variables. Its parent process was an
interactive `distrobox enter` session opened at 10:28:47. So the daemon was
started from a fresh terminal in which `robot_mode.sh` had not been sourced.
Two code paths can do that from such a terminal, and the evidence cannot
separate them: the old `vla_kill.sh` restarted the daemon silently from
wherever it was run, and the `ros2` command-line tool starts a daemon by itself
the first time any `ros2 topic` / `ros2 node` command runs while no daemon is
alive. Either way the result is a daemon that knows nothing about the discovery
server and therefore sees nothing on domain 0, while `ros2 topic echo` and
`ros2 topic hz` keep working because they subscribe directly and never ask the
daemon. The `ros2` tool only checks that the running daemon matches the
terminal's domain and middleware, never its discovery settings, so a correctly
configured terminal happily uses a blind daemon. What prevents it now:
`vla_kill.sh` will not start a daemon from a terminal without the robot
settings, and `robot_mode.sh` restarts the daemon correctly every time it is
sourced. The auto-start path still exists; section 5 gives the one-line check
that catches it.

## 3. The `.bashrc` `VLA_MODE` logic

Left exactly as it was. Lines 118–130 of `~/.bashrc` do this in order: source
ROS, source `/etc/turtlebot4_discovery/setup.bash` (which sets
`RMW_IMPLEMENTATION`, `ROS_SUPER_CLIENT`, `ROS_DOMAIN_ID=0` and
`ROS_DISCOVERY_SERVER`), set `FASTRTPS_DEFAULT_PROFILES_FILE`, and then, unless
`VLA_MODE=robot` was exported before the terminal opened, unset
`ROS_DISCOVERY_SERVER` and `FASTRTPS_DEFAULT_PROFILES_FILE` again.

The correct behaviour, which is what you have now, is: a fresh terminal is
simulation-safe and blind to the robot by default. Hardware mode is entered
only by `source ~/robot_mode.sh`, which exports both variables explicitly
after `.bashrc` has run, so the guard does not matter in that terminal.
Nothing in the workflow sets `VLA_MODE`, and nothing needs to. The guard has
two side effects you must know about, both covered in section 5: a daemon
started from a guarded terminal is blind, and an interactive shell opened from
a robot terminal (a bare `xterm`, or typing `bash`) runs `.bashrc` again and
strips the variables it inherited.

## 4. Start-of-session checklist for hardware

Do these in order. Each step has a check; do not go on until the check passes.

Step 1 — robot powered on and reachable. From the host or the container:

```
timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22" && echo REACHABLE || echo UNREACHABLE
```

Check: prints `REACHABLE`. This opens a real TCP connection to the Pi's ssh
port. `ping` cannot be used inside the container.

Step 2 — Pi-side service order and clock, your existing procedure:

```
ssh ubuntu@10.42.0.169 'sudo systemctl stop turtlebot4; sleep 10; sudo systemctl restart discovery; sleep 15; sudo systemctl start turtlebot4'
ssh ubuntu@10.42.0.169 'uptime; chronyc tracking | grep "System time"'
```

Check: load average under 1.5 (the handout §14.15 gate; "about 4" was
written here in the morning and is wrong), system time within 1 s, and —
added in Part 2 — `vcgencmd measure_temp` under 80 °C with
`vcgencmd get_throttled` ending in 0. If the time is out,
`sudo chronyc makestep` on the Pi.

Step 3 — open a terminal and enter the container:

```
distrobox enter ubuntu22-gpu
[ -f /run/.containerenv ] && echo CONTAINER || echo HOST
```

Check: prints `CONTAINER`.

Step 4 — switch this terminal to robot mode. The word `source` is essential;
running the file without it prints the banner and sets nothing.

```
source ~/robot_mode.sh
env | grep -E "^(ROS_DISCOVERY|FASTRTPS)"
```

Check: exactly two lines, `FASTRTPS_DEFAULT_PROFILES_FILE=...` and
`ROS_DISCOVERY_SERVER=10.42.0.169:11811;`. Above the banner you should also
have seen `The daemon has been stopped` and `The daemon has been started`.

Step 5 — confirm the daemon can see the robot. Wait 10 s first; that is how
long the daemon takes to download the directory from the Pi.

```
sleep 10; ros2 node list
```

Check: a list containing `/robot1/turtlebot4_node` and
`/robot1/rplidar_composition`. An empty list here means the daemon is blind;
re-run step 4.

Step 6 — confirm the base and the LIDAR are alive:

```
timeout 5 ros2 topic hz /robot1/odom
timeout 5 ros2 topic hz /robot1/scan
```

Check: about 20 Hz and about 7–8 Hz. No odom means the Create 3 has gone
silent; only a power-cycle fixes that.

Step 7 — start SLAM from this terminal:

```
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot1 params:=/home/danyalaziz/slam_vla.yaml
```

For every further terminal you need, repeat steps 3 and 4 in it. Sourcing
`robot_mode.sh` again is now safe while SLAM runs; it restarts the daemon,
so wait 10 s before the next graph command.

Check, after about 20 s, from a robot-mode terminal:

```
ros2 node list | grep slam
ros2 topic info /robot1/map -v | grep "Node name"
timeout 12 ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static | tail -2
```

Expect `/robot1/slam_toolbox`; the first `Node name` line reading
`slam_toolbox`, not `_NODE_NAME_UNKNOWN_`; and a `Translation:` line from
`tf2_echo`. Read the last lines of `tf2_echo`, never the first: the first line
always says the frame does not exist, because the listener has not filled yet.

Step 8 — RViz, from a robot-mode terminal. **SUPERSEDED the same afternoon
by Part 2 §8 step 5** (the pre-configured `~/vla_hardware.rviz` with
`-r __ns:=/robot1`); kept for the record:

```
ros2 run rviz2 rviz2 --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
```

Set Fixed Frame to `map`; add a Map display on `/robot1/map`.

Step 9 — Nav2, then the agent. **SUPERSEDED by Part 2 §8 step 6**: the
stock Nav2 launch aborts on a bond timeout here; use
`ros2 launch ~/nav2_hw.launch.py` and the `velocity_smoother` check. The
original text, kept for the record:

```
ros2 node list | grep -E "bt_navigator|controller_server|planner_server"
ros2 service list | grep -c change_state
```

Expect the three nodes and a count above zero. Those are the lifecycle
services Nav2 needs; they are discovered the same way `save_map` was today.

Step 10 — end of session. Run the kill script from a robot-mode terminal
inside the container, not from the host:

```
~/vla_kill.sh
```

Check: the line `daemon restarted in ROBOT mode`. If it says `daemon STOPPED,
not restarted`, the terminal was not in robot mode; that is harmless now, but
the next terminal must source `robot_mode.sh` before any `ros2` command.

## 5. What will silently break it again

Each item: the mistake, then the one-line check that catches it.

**Running any `ros2` command in a fresh terminal while no daemon is alive.**
The tool auto-starts a blind daemon and every terminal inherits it. Check:

```
tr '\0' '\n' < /proc/$(pgrep -f ros2-daemon | head -1)/environ | grep -q ROS_DISCOVERY_SERVER && echo "daemon OK" || echo "DAEMON BLIND - source ~/robot_mode.sh"
```

**Sourcing `sim_mode.sh` in any terminal while robot work is going on.** It
restarts the daemon with no discovery server, which is correct for simulation
and blind for the robot. Same check as above.

**Executing `robot_mode.sh` instead of sourcing it** (`./robot_mode.sh` or
`bash robot_mode.sh`). The banner prints, nothing is set. Check:

```
env | grep -E "^(ROS_DISCOVERY|FASTRTPS)"
```

Two lines or it is not configured.

**Launching RViz, map_saver, Nav2 or a `ros2 service call` from a terminal
that never sourced `robot_mode.sh`.** Each of those creates its own
participant with whatever the terminal has. Same `env | grep` check, run in
that terminal before launching.

**Opening an interactive shell from a robot terminal** — a bare `xterm`, or
typing `bash`. `.bashrc` runs again and the guard strips the two variables.
Same `env | grep` check, run inside the new window. (A window started with
`xterm -e bash somescript.sh` is not interactive and is safe.)

**Deleting `/dev/shm/fastrtps_*` by hand, or restoring an old script copy.**
These copies still contain the wipe: `robot_mode.sh.bak`,
`sim_mode.sh.WORKING`, and all four `*.bak-20260908` files. Restore from them
only if you re-apply Change A. Check that SLAM is still reachable:

```
ros2 topic info /robot1/map -v | grep "Node name"
```

First line `slam_toolbox` is good. `_NODE_NAME_UNKNOWN_`, or no `/robot1/map`
at all while the SLAM process is running, means it has been cut off. The only
fix is to restart SLAM.

**Running `vla_kill.sh` on the host.** It still kills the processes, but it
now prints `ros2 not on PATH here (host?) -- daemon untouched` and
`fastdds not on PATH -- stale shm not cleaned`, and does neither. Not a
breakage, but the daemon is then whatever it was before.

## 6. Statements in CLAUDE.md and the handout that are now wrong

**CLAUDE.md, line 28.** Says: "Ollama, ~/vla_kill.sh, ssh to the robot —
directly on the host." Corrected: "Ollama and ssh to the robot — directly on
the host. `~/vla_kill.sh` — inside `ubuntu22-gpu`, from a terminal that has
sourced `robot_mode.sh`, so the daemon is restarted in robot mode. On the host
it kills processes but cannot touch the daemon or shared memory, and says so."

**CLAUDE.md, lines 58–62.** Says: "`/etc/turtlebot4_discovery/setup.bash`
sets ONLY `ROS_DISCOVERY_SERVER`. It does NOT set
`FASTRTPS_DEFAULT_PROFILES_FILE`. Source it on its own and the robot is
undiscoverable, silently — no error, just an empty topic list." Corrected:
"`/etc/turtlebot4_discovery/setup.bash` sets four variables:
`RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID=0`, `ROS_DISCOVERY_SERVER`, and
`ROS_SUPER_CLIENT`, which is `True` only when the shell has a terminal attached
and `False` in any `bash -c` wrapper. It does NOT set
`FASTRTPS_DEFAULT_PROFILES_FILE`. In a wrapper shell, sourcing it alone makes a
plain client that receives topic data but no directory, which shows as an
empty topic list. The XML profile forces super-client mode regardless of the
flag, so hardware needs both variables together."

**CLAUDE.md, lines 63–66, and PROJECT_HANDOUT_v5.md §14.6 (line 1061).**
Both say `ros2 node list` / `node info` / `param get` "are unreliable through
the discovery server — they time out, return partial lists ... and they do it
inconsistently." Corrected: "These commands are answered by the ROS daemon,
one shared background process. They are complete and stable when the daemon
was started from a robot-mode terminal (verified repeatedly on 8 Sep), and
empty or partial when it was started from a terminal without the discovery
variables. Inconsistency between runs means the daemon was restarted in
between from a different terminal. Before trusting or distrusting `node list`,
check the daemon's environment with the one-liner in `CHANGELOG_2026-09-08.md`
§5. A clean `node list` still says nothing about the data path; `topic hz` and
`tf2_echo` remain the ground truth for that."

**PROJECT_HANDOUT_v5.md, lines 834–841.** Says: "`FASTRTPS_DEFAULT_PROFILES_FILE`
is NOT set by `/etc/turtlebot4_discovery/setup.bash`. That file sets
`ROS_DISCOVERY_SERVER` and only that." Corrected: same text as the CLAUDE.md
lines 58–62 correction above. The rest of that bullet (hardware needs both
variables together) stands.

**PROJECT_HANDOUT_v5.md, lines 1282–1300 (the `.bashrc` guard).** Not wrong,
but incomplete. Add: "Two consequences found 8 Sep: any ROS daemon started
from a guarded terminal is blind to the robot and is then used by every
terminal, and any interactive shell opened from a robot terminal re-runs the
guard and loses both variables. Section 5 of `CHANGELOG_2026-09-08.md` has the
checks."

**PROJECT_HANDOUT_v5.md, line 120 of CLAUDE.md and line 159 of the handout**
("Stop everything with `~/vla_kill.sh`" / "Kill everything: `~/vla_kill.sh`").
Add: "from a robot-mode terminal inside the container".

**`robot_mode.sh` lines 36–37 (comment, not changed).** Says the trailing
semicolon in `ROS_DISCOVERY_SERVER` "is required -- it terminates the server
list, and FastDDS mis-parses the entry without it." Not verified and almost
certainly folklore: the semicolon separates entries in a list of several
servers, and the ROS 2 discovery-server tutorial uses the form without it. It
comes from TurtleBot 4's own generated setup file, and it is harmless either
way. Suggested comment: "The trailing semicolon is the form TurtleBot 4's
setup script writes. It is a list separator and is harmless; the form without
it is equivalent."

## What was verified after the changes

- `bash -n` passes on all four edited scripts.
- With a daemon started from a robot-mode terminal: `ros2 node list` shows
  all 12 Pi nodes plus `/robot1/slam_toolbox`; `ros2 service list` shows
  `/robot1/slam_toolbox/save_map`; `ros2 topic info /robot1/map -v` names
  `slam_toolbox` as the publisher.
- After relaunching SLAM and RViz: a fresh subscriber receives `/robot1/map`
  at 2.0 Hz; `map → base_link` resolves; `/robot1/odom` stayed at 20 Hz
  through the SLAM restart.
- Two in-place rotations commanded on `/robot1/cmd_vel` (0.3 rad/s, 22 s and
  10 s) were executed by the base, with heading tracked in odometry, and the
  zero-velocity stop brought angular velocity to 0.0 both times.

---

# Part 2 — afternoon of 8 September 2026: RViz layout and Nav2 on the real robot

Done between 11:45 and 13:00 with the robot on its dock, SLAM running from
the morning session, and the workstation in robot mode. Every statement
below was checked on the live system; nothing is taken from memory. Log
timestamps are converted to local time (UTC+5). Software versions in play:
Fast DDS 2.6.11 and rmw_fastrtps 6.2.10 on the PC, Fast DDS 2.6.10 on the
Pi, Nav2 1.1.20, RViz 11.2.26, turtlebot4_navigation 1.0.5.

## 7. Files created or modified

### Created

**`/home/danyalaziz/vla_hardware.rviz`** (11:52, 411 lines). The RViz layout
for the real robot. Load it with `-d`. Contents: Grid; RobotModel from
`/robot1/robot_description`; TF showing only `map`, `odom`, `base_footprint`,
`base_link`, `rplidar_link`, `oakd_link` (the robot has about 40 frames, the
rest are hidden); LaserScan on `/robot1/scan`; Odometry on `/robot1/odom`;
Map on `/robot1/map`; a "Nav2" group with the global and local costmaps,
global plan, local plan and footprint; and an OAK-D image display that is
switched off by default because the camera loads the Pi. Fixed Frame is
`map`. Tools: 2D Pose Estimate on `/robot1/initialpose`, 2D Goal Pose on
`/robot1/goal_pose`, Publish Point on `/robot1/clicked_point`, and the Nav2
Goal tool with its Navigation 2 panel.

Why every topic is written in full as `/robot1/...`: the displays then work
no matter how RViz was started. The QoS (quality-of-service, the delivery
rules a subscriber asks for) was set from what the robot actually publishes,
read with `ros2 topic info -v`: the map, the odometry and the robot
description are all published "transient local", meaning the last message is
kept for late joiners, so those displays ask for transient local or they
never receive the message that was sent before RViz opened. The laser scan
is reliable/volatile; the display asks for best effort, which is compatible.

**`/home/danyalaziz/nav2_hw.launch.py`** (12:15, 7.4 kB). Nav2 bring-up for
the real robot, hardware only. It starts exactly the eight processes the
stock command starts (controller, smoother, planner, behavior server,
bt_navigator, waypoint follower, velocity smoother, lifecycle manager), with
the same namespace, the same `/tf` remaps, the same scan remaps into both
costmaps, the same stock parameter file from `/opt`, `use_sim_time` false
and no composition. It differs in two settings, both explained in section 9:

- `bond_timeout` is a launch argument, default 30 s (stock 4 s, and the
  stock launch file offers no way to change it).
- `autostart` is off, and a timer sends the lifecycle manager its STARTUP
  command `startup_delay` seconds after launch, default 20 s.

**`/home/danyalaziz/vla_logs/nav2_hw_20260908.log`** — full Nav2 output of
the run that worked, including the two navigation goals.
**`nav2_hw_20260908.log.attempt1`** — output of the second (failed) attempt.
The first attempt used the stock launch and was captured only as a
screenshot of its terminal; its two decisive lines are quoted in section 9.

Temporary files under `~/.claude/jobs/557b175b/tmp/` (xterm launchers for
RViz and Nav2, a planner dry-run script, an XWD-to-PNG converter for
screenshots) are deleted when the job is cleaned up. Nothing depends on
them; the RViz and Nav2 windows have already read their launchers.

### Modified

Nothing in this part. `CLAUDE.md` and `PROJECT_HANDOUT_v5.md` are updated
separately (section 14). `slam_vla.yaml` was not touched: `debug_logging`
already reads `false` in the file, and the running SLAM node reports
`False` when asked.

### Simulation path: untouched, confirmed by timestamps

| File | Last modified |
|---|---|
| `~/vla_sim.sh` | 13 Aug 2026 |
| `~/run_agent_sim.sh` | 12 Aug 2026 |
| `~/.vla_gui.sim.json` | 12 Aug 2026 |
| `~/sim_mode.sh.WORKING` | 12 Aug 2026 |
| `~/sim_mode.sh`, `~/run_sim_stack.sh` | 8 Sep 11:15, the morning shm change (section 1), before this work |
| `/opt/ros/humble/share/turtlebot4_viz/rviz/robot.rviz` (the sim's RViz layout) | 22 Feb 2023, stock |
| anything under `/opt` | untouched |

The sim gets its RViz layout from that stock `robot.rviz`, loaded by
`vla_sim.sh` line 227. It is not copied, edited or referenced by the new
hardware file.

## 8. Bringing Nav2 up on hardware, in order

Every step has a check. Do not go on until the check passes. "Robot-mode
terminal" means a terminal inside `ubuntu22-gpu` in which
`source ~/robot_mode.sh` has been run and `env | grep -E
"^(ROS_DISCOVERY|FASTRTPS)"` prints two lines.

**Step 0 — the Pi gate, now with temperature.** From the host:

```
ssh ubuntu@10.42.0.169 'uptime; pgrep -af "[u]nattended-upgrade|[a]pt\.systemd\.daily|[a]pt-check|[d]pkg|[a]pt-get" || echo APT CLEAR; vcgencmd measure_temp; vcgencmd get_throttled'
```

Healthy: load under 1.5, `APT CLEAR`, temperature under 80 °C, and the
last hex digit of `throttled=` is 0. On 8 Sep it read load 6.9, `APT
CLEAR`, 84.7 °C, `throttled=0xe0008`; see section 9.5 for what that meant.

**Step 1 — reachable.** `timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22" && echo REACHABLE`.

**Step 2 — robot-mode terminal**, as in section 4 steps 3 and 4.

**Step 3 — base alive.** `timeout 30 ros2 topic hz /robot1/odom` shows about
20 Hz, and
`ros2 run tf2_ros tf2_echo odom base_link --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`
prints a `Translation:` line (read the last lines, never the first).

**Step 4 — SLAM**, exactly as in section 4 step 7. Check that
`ros2 run tf2_ros tf2_echo map base_link` (same remaps) prints a
translation.

**Step 5 — RViz**, from a robot-mode terminal:

```
ros2 run rviz2 rviz2 -d ~/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
```

Healthy: the window opens already showing the map, the laser scan and the
robot model, Fixed Frame `map`, "Global Status: Ok", and
`ros2 topic list | grep goal_pose` includes `/robot1/goal_pose`.

**Step 6 — Nav2**, from a robot-mode terminal:

```
ros2 launch ~/nav2_hw.launch.py
```

Wait about 35 s. Healthy: the terminal prints `Managed nodes are active`
(on 8 Sep at 31 s after launch, all seven "connected with bond" lines
present), then from another robot-mode terminal:

```
ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother     # expect 1
timeout 30 ros2 topic hz /robot1/local_costmap/costmap             # about 1.7 Hz
ros2 action list | grep navigate_to_pose                           # /robot1/navigate_to_pose
```

The `velocity_smoother` line is the one that matters: it is the last server
to be activated and the one that publishes `/robot1/cmd_vel`. If it is not
publishing, Nav2 accepts goals and the robot never moves.

**Step 7 — undock**, then the goal. The undock action is
`ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"`.
On 8 Sep the goal was sent while still docked and Nav2 simply drove off the
dock; it worked, but undocking first is the correct order. Send the goal
from RViz: click "Nav2 Goal", click a point on the map, drag for the
heading. Do not send goals from a freshly started terminal command; see
section 9.4.

**Step 8 — confirm it drove.** Section 10 lists what to look at.

**Step 9 — end of session.** Dock with
`ros2 action send_goal /robot1/dock irobot_create_msgs/action/Dock "{}"`,
then `~/vla_kill.sh` from a robot-mode terminal inside the container.

## 9. What was broken, and how each was fixed

None of the classic Nav2 faults occurred: the stock parameters were right,
no remap was missing (the stock launch already remaps `/tf` and the costmap
scan topics), the costmaps built, and the `map`, `odom` and `base_link`
frames were all present and connected throughout. Everything that went wrong
was about timing between programs, and all of it traces to one cause.

### 9.1 RViz opened empty on hardware, and its goal tools were in the wrong namespace

Symptom: every display had to be added by hand, Fixed Frame set, and the
map's QoS changed, every session.

Root cause: on hardware RViz was being started with no `-d` layout file at
all. The sim's layout, stock `robot.rviz`, uses relative topic names
(`map`, `scan`) that only resolve correctly when RViz is in the robot's
namespace, which the sim (no namespace) satisfies trivially. A second,
invisible problem: the old hardware RViz published its goal and initial-pose
on `/goal_pose` and `/initialpose`, in the root namespace, because nothing
told RViz it belonged to `/robot1`. The Nav2 Goal tool and Navigation 2
panel ignore the layout file entirely and use whatever namespace RViz's own
node has.

Fix: `vla_hardware.rviz` (section 7) plus `-r __ns:=/robot1` on the command
line. Verified: `/robot1/goal_pose`, `/robot1/initialpose`,
`/robot1/clicked_point` present; RViz's subscriptions on `/robot1/map` and
`/robot1/robot_description` are transient local; the panel's client node is
`/robot1/rviz_navigation_dialog_action_client`.

### 9.2 Attempt 1, stock launch: lifecycle bring-up aborted on a bond timeout

Symptom, 11:59, in the Nav2 terminal:

```
Server bt_navigator was unable to be reached after 4.00s by bond. This server may be misconfigured.
Failed to bring up all requested nodes. Aborting bringup.
```

Controller and smoother active, planner and behavior server active,
bt_navigator active but unbonded, waypoint follower and velocity smoother
never activated. Because the velocity smoother is what publishes
`/robot1/cmd_vel`, this state accepts goals and never moves.

What a bond is: after the lifecycle manager activates a server, the two open
a heartbeat link ("bond") over the `/robot1/bond` topic so the manager can
tell if the server dies. The manager waits half of `bond_timeout`, so 2 s,
for the link to form. On this system that link has to be matched through the
discovery server on the Pi, and on 8 Sep that took longer than 2 s for the
fifth server.

Fix: `bond_timeout` raised to 30 s in `nav2_hw.launch.py`. The stock launch
file passes the lifecycle manager only three parameters and offers no way to
set this one, which is why a separate hardware launch file was needed.

### 9.3 Attempt 2, timeout raised: the manager hung on its first configure reply

Symptom, 12:06, in `nav2_hw_20260908.log.attempt1`: the manager configured
the controller, asked the smoother to configure, the smoother did so, and
then:

```
[robot1.smoother_server.rclcpp]: failed to send response to /robot1/smoother_server/change_state (timeout): client will not receive response
```

The manager then sat forever; the five remaining servers stayed
unconfigured.

Root cause: the manager sent that request less than a second after it
started. A request and its reply travel on two separate channels. The
request's channel was matched, so the smoother received it; the reply
channel was not yet matched at the smoother's side, and rmw_fastrtps drops a
reply if the caller's reply channel is not matched within about 0.1 s. The
manager's call has no timeout of its own, so it waits indefinitely. In
normal multicast discovery both channels match within milliseconds and this
race never shows; through a slow discovery server it does.

Fix: `autostart` off and a 20 s delayed STARTUP in `nav2_hw.launch.py`, so
every request/reply channel between the manager and its seven servers is
matched before the first request is sent. Attempt 3 at 12:15 came up clean:
`Managed nodes are active` at 31 s, all seven bonds formed in 0.1 to 1.1 s.
Honest caveat: attempt 3 also changed a second thing at the same time. In
attempts 1 and 2 the bring-up was being polled every 5 s with `ros2
lifecycle get`, and each of those is a new program that the discovery server
has to serve; attempt 3 was watched from its log file only. Which of the two
changes did the work was not separated. Both are cheap and both stay.

### 9.4 Goals from freshly started terminal commands get no reply

Symptom: three planner-only dry runs (`compute_path_to_pose`, which plans
without moving) from the command line, including one that waited 8 s after
creating its client, all timed out with no reply. Meanwhile the goal sent
from RViz worked first time.

Root cause, confirmed in the planner's log:

```
[robot1.planner_server.rclcpp_action]: Failed to send goal response ... (timeout): client will not receive
```

one line per attempt. So the requests arrived and the planner answered; each
answer was thrown away because the caller's reply channel was still
unmatched. Measured the same afternoon: a brand-new program on this PC
receives data from an existing publisher after 2.0 to 2.4 s, but an existing
program receives data from a brand-new publisher only after 18.7 s, and a
fresh service call to a Nav2 node succeeded once in 4.5 s and got no reply
the next time in 30 s. A new program is fully known to everyone else only
after roughly 20 s.

Fix: none needed in code. RViz, the lifecycle manager after its delay, and
the VLA agent all keep their clients alive from start-up, so their first call
comes long after matching has completed. The rule for command-line tools:
expect `ros2 action send_goal`, `ros2 service call` and `ros2 param get`
from a fresh terminal to hang or need a retry on this system, and never use
one as the only evidence that something is broken.

### 9.5 The Pi is saturated and thermally throttled, the probable common cause

Measured at 12:45 with the camera on (robot off the dock): load 6.9 on 4
cores, 84.7 °C, `throttled=0xe0008`, clock 1.68 GHz of 1.8, CPU 29 % user /
44 % kernel / 10 % soft-interrupt. Apt clear, timers masked, clock synced,
memory fine, no swap. Measured again at 12:52 with the robot docked and the
camera off: load 5.3, still 84.7 °C, clock now 1.53 GHz.

What `0xe0008` means: the low digit 8 says the soft temperature limit is
active right now; the high digits say the CPU has been frequency-capped and
throttled at some point since boot. A Pi 4 hard-throttles at 85 °C. The Pi
has no fan and no thermal overrides in `config.txt`.

Where the CPU goes, 3-second sample, camera on: diagnostics updater 37 %,
Create 3 republisher 32 %, two kernel Wi-Fi/USB worker threads 27 % and
26 %, OAK-D driver 27 %, turtlebot4_node 23 %, diagnostics aggregator 22 %,
robot_state_publisher 18 %, the two joystick nodes 14 % and 13 % with no
joystick attached. With the camera off the discovery server itself was at
27 %. Every ROS node on the Pi is burning CPU, which points at DDS
housekeeping for a graph of 74 topics and 233 services rather than at any
one node's real work.

Status: this is a strong correlation, not a proven cause. Nav2 worked on the
same architecture in August, and sections 9.2 to 9.4 are exactly what a slow
discovery server produces. What would settle it: let the Pi cool (or add a
fan), confirm `get_throttled` ends in 0, then re-run the three timing tests
from section 9.4. If the numbers drop to well under a second, the thermal
reading was the cause.

### 9.6 A command killed its own shell (a new form of handout §14.9)

`pkill -INT -f "[n]av2.launch.py namespace"` was run in the same command
that had just written `nav2_hw.launch.py` through a heredoc. The bracket
trick protects against the `pkill` line matching itself, but the launch
file's comment contained the text `nav2.launch.py namespace`, and the whole
heredoc was part of the shell's command line, so `pkill` matched and killed
the shell running it. The stock Nav2 was stopped as intended; everything
after that line in the command never ran. Rule: the bracket trick only
protects the `pkill` line. If the pattern text appears anywhere else in the
same command, kill by PID instead: find it with `pgrep -f` in one command,
then `kill <pid>` in the next.

## 10. How the goal was sent and what proved the robot planned and drove

The goal was sent by clicking "Nav2 Goal" in the new RViz window. That tool
hands the pose to the Navigation 2 panel, whose action client had been
connected since RViz started at 11:52, and the panel sends it to the
`/robot1/navigate_to_pose` action. Two goals were sent in a row.

Evidence, from `nav2_hw_20260908.log`:

```
[robot1.bt_navigator]: Begin navigating from current location (0.04, -0.05) to (-0.29, 3.20)     12:37:45
[robot1.controller_server]: Reached the goal!                                                    12:38:02
[robot1.bt_navigator]: Goal succeeded
[robot1.bt_navigator]: Begin navigating from current location (-0.24, 3.05) to (-0.61, -0.84)    12:38:05
[robot1.controller_server]: Reached the goal!                                                    12:38:23
[robot1.bt_navigator]: Goal succeeded
```

The first goal started from the dock position and reached a point 3.3 m
away in 17 s; the second covered 3.9 m in 18 s. "Recoveries: 0" in the RViz
panel and no spin/backup lines in the log. Between goals the controller
logged "Passing new path to controller" once per second, which is the
planner replanning as the map grew.

Independent confirmation after the runs: `map -> base_link` from TF put the
robot at (-0.36, 0.36) heading 70°, so it had moved from the dock;
`/robot1/odom` still at 20 Hz; `/robot1/dock_status` `is_docked: false`;
`/robot1/cmd_vel` silent once idle; battery 54 %. The RViz panel showed
Feedback "reached".

Two warnings in the log worth knowing for the viva: 417 lines of "Behavior
Tree tick rate 100.00 was exceeded" and two of "Control loop missed its
desired rate of 20 Hz". These say the workstation was busy (software-drawn
RViz, SLAM and Nav2 together) and Nav2's loops occasionally ran late. The
paths were still tracked. They will get worse when the agent and YOLO are
added, and they are the first thing to check if navigation becomes jerky.

## 11. Hardware differences from simulation, for the evaluation

1. **Namespace.** The real robot puts everything under `/robot1`; the
   simulation uses none. Same code, different names, which is why the RViz
   layout, the `/tf` remaps and the `__ns` option exist only on hardware.
2. **Discovery.** In simulation every program finds every other by
   multicast on one machine, in milliseconds. On hardware every program
   registers with a discovery server on the Pi and receives the full
   directory from it. Consequences: new programs take seconds to be fully
   known (section 9.4), the ROS daemon must be started from a robot-mode
   terminal (section 2), and the Pi's CPU state affects the PC's software.
3. **Lifecycle timing.** Nav2's bond timeout and its start-up race never
   show in simulation. `nav2_hw.launch.py` exists only because of hardware
   discovery latency.
4. **Clock.** Simulation runs on `/clock` from Gazebo with `use_sim_time`
   true; hardware uses wall time with `use_sim_time` false, and the Pi and
   PC clocks must agree (chrony, section 4).
5. **Compute budget.** The Pi is a 4-core computer with no fan carrying the
   Create 3 republisher, LiDAR, camera, diagnostics and the discovery
   server. The simulation has the whole workstation. Section 9.5 is what
   that looks like when it runs out.
6. **Real QoS.** The real odometry and map are transient local; the layout
   must ask for that. In simulation the defaults happen to work.
7. **The dock.** Nav2 will drive the robot straight off the dock. The
   correct sequence is undock first, and the dock action, not Nav2, puts it
   back.
8. **Rendering.** RViz draws on the CPU in the container (handout §8.4), so
   it is slow and adds to the tick-rate warnings.

## 12. What will silently break it again, with the check that catches each

**RViz started without `-d ~/vla_hardware.rviz` or without `-r __ns:=/robot1`.**
Displays empty, or goals go to the root namespace and Nav2 never sees them.
Check: `ros2 topic list | grep -c /robot1/goal_pose` must be 1.

**Nav2 started with the stock `ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1`.**
Bond abort or a silent hang; the robot accepts goals and does not move.
Check: `ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother` must be 1.

**A goal, service call or parameter read sent from a freshly started terminal command.**
Hangs or times out while everything is actually healthy.
Check: `grep -c "Failed to send goal response" ~/vla_logs/nav2_hw_*.log`
rising means the replies were dropped, not the request. Send goals from RViz
or the agent.

**Polling Nav2 with `ros2 lifecycle get` or `ros2 node list` every few seconds while it is coming up.**
Each poll is a new program the Pi's discovery server must serve; it can
tip the bond timing over. Watch the launch log instead.

**The Pi hot or throttled.**
Every timing failure in section 9 gets more likely.
Check: `ssh ubuntu@10.42.0.169 'vcgencmd measure_temp; vcgencmd get_throttled'`, want under 80 °C and a `throttled=` value whose last digit is 0.

**The camera left running when no step needs it.**
Adds about 1.5 to the Pi's load and a full CPU core.
Check: `timeout 20 ros2 topic hz /robot1/oakd/rgb/preview/image_raw` should print nothing when the camera should be off.

**`pkill -f` in a command that also contains the pattern text.** Kills the
shell, silently. Check before running: `pgrep -fc "<pattern>"` in a separate
command shows how many processes match; if it is one more than you expect,
one of them is you.

**The Navigation 2 panel's "Navigation: inactive" label.** It polled once
during the 20 s start-up delay and never re-polls. Not a fault. The
Feedback line and the launch log are the truth.

**`minimum_travel_distance` in `~/slam_vla.yaml` is 0.2.** CLAUDE.md and
handout §14.4 say it must stay 0.0 or the `map` frame vanishes while the
robot is parked. On 8 Sep the robot sat docked for over 45 minutes with
0.2 and the map frame stayed valid throughout, with `transform_publish_period`
0.05 in the same file. The two observations are not reconciled. Not changed
today; do not change either the file or the rule without a test.

## 13. Open questions after this part

- Is the Pi's heat the cause of the discovery latency (section 9.5)? Test
  described there.
- Would the stock Nav2 launch succeed on a cool Pi? It was never retried
  cleanly; `nav2_hw.launch.py` is safe either way.
- Which of the two changes in attempt 3 mattered (section 9.3)?
- The diagnostics updater at 30 to 37 % of a core and the two joystick
  nodes are candidates for switching off on the Pi to buy back CPU. Not
  done; it changes the robot's own bring-up.
- `minimum_travel_distance` 0.2 versus §14.4 (section 12).

## 14. Documentation updated in this part

- `CLAUDE.md`: the section 6 corrections applied (`vla_kill.sh` location,
  what `setup.bash` sets, `node list` reliability); the Pi check now
  includes temperature and throttling; the hardware RViz and Nav2 commands,
  the fresh-client rule and the `pkill` rule added.
- `PROJECT_HANDOUT_v5.md`: section 6 corrections applied to §13.2, §14.6
  and §14.13; `vla_kill.sh` note in §3.1; status table and §10 file list
  updated; new §15 recording this session.
- `~/slam_vla.yaml`: not modified; `debug_logging` was already `false`.

## 15. End of day, 13:05–13:27

- **Re-dock.** At 13:05 the robot reported `is_docked: false` and a falling
  battery while sitting at the dock position it had been docked at since
  12:52, with no goal sent and no velocity published in between: the
  contacts had let go. Re-docked at 13:14 by a long-lived script client
  (`redock.py`, job tmp): Nav2 goal to a staging pose 0.6 m in front of the
  dock, `SUCCEEDED` in 36 s, then the Dock action, `is_docked=True` in 15 s.
  The client waited 30 s after creation before its first call and both
  replies arrived immediately — the fresh-client rule of §9.4, applied.
- **Map not saved.** `save_map` from a fresh command-line client at 13:16
  got no reply in 90 s (§9.4 again); SLAM was then stopped. `~/maps/` is
  empty.
- **Shutdown.** Nav2, RViz and SLAM stopped by SIGINT to their launch
  processes, the three xterms closed, then `vla_kill.sh` from a robot-mode
  terminal inside the container. It printed `daemon restarted in ROBOT mode
  (server 10.42.0.169:11811;)`, `robot_state_publisher processes: 0`,
  `clean`. Daemon environment verified to carry both discovery variables;
  it lists 11 `/robot1` nodes.
- **`vla_kill.sh` killed the wrapper shell that ran it.** Its process-group
  kill matches any command line containing `rviz2` (among others), and the
  `bash -c` wrapper that invoked it also contained the text
  `lib/rviz2/[r]viz2` from a verification step. The script itself completed
  (the daemon restart proves it); the wrapper died and its remaining
  checks never ran, so the script was re-run cleanly from a script file to
  capture its output. Same class as §9.6 and handout §14.9; recorded in
  CLAUDE.md.
- **The Pi cooled as soon as the PC-side stack stopped.** With the robot
  docked and camera off both times: 13:00, PC stack running, load 5.3,
  84.7 °C, clock 1.53 GHz; 13:27, ten minutes after SLAM/Nav2/RViz and all
  command-line tools were gone, load 1.39 (5-min average 3.58 and falling),
  70.6 °C, `throttled=0xe0000` (soft limit no longer active). The Pi's own
  nodes were identical in both readings. So a large share of the Pi's
  load, and therefore its heat, was generated by serving the PC-side graph
  through the discovery server — not by the Pi's own sensor work.
- **Battery.** Reading 47 % at 13:10 (undocked) and still 47 % at 13:27,
  12 min after re-docking; earlier in the day it rose 48 → 54 % in 27 min
  docked. Check in the morning that it charged.
