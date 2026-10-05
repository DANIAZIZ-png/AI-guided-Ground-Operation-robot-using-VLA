# HANDOVER — VLA ground robot, hardware bring-up, written 8 Sep 2026 ~13:40

For the model that continues this work tomorrow. You have no memory of
today. This file is self-contained: everything you need to reach today's
working state and continue is written out here, not referenced. Where the
older documents disagree with this file, this file is newer and wins.
Companion documents: `~/CLAUDE.md` (working rules, read first),
`~/PROJECT_HANDOUT_v5.md` (the project record, §15 is today),
`~/CHANGELOG_2026-09-08.md` (today's detailed log with log excerpts),
`~/RESUME_NEXT_SESSION.md` (short version of the restart sequence).

The user is an avionics student, not a software engineer, preparing an
evaluation. Plain language. One step at a time: give the command, run it,
report pass/fail, stop and wait. Say plainly when something failed.

--------------------------------------------------------------------------

## 0. The one hard constraint

**`~/vla_sim.sh` and the entire simulation path must never be touched.**
That is `vla_sim.sh`, `sim_mode.sh`, `sim_mode.sh.WORKING`,
`run_sim_stack.sh`, `run_agent_sim.sh`, `~/.vla_gui.sim.json`, anything
under `/opt`, and the stock RViz layout
`/opt/ros/humble/share/turtlebot4_viz/rviz/robot.rviz`. It is the user's
working demo and the fallback if hardware fails in front of the faculty.
Hardware-only files are fine to change. If a fix would touch something
shared by both, stop and tell the user before doing it. Timestamps on
8 Sep 13:00 for the record: `vla_sim.sh` 13 Aug, `run_agent_sim.sh` 12 Aug,
`.vla_gui.sim.json` 12 Aug, `sim_mode.sh.WORKING` 12 Aug, `sim_mode.sh` and
`run_sim_stack.sh` 8 Sep 11:15 (a one-line shared-memory fix, section 5.6,
made before the constraint was stated). Nothing sim-side was touched after
11:15.

## 1. The system in one page

- **Host:** Ubuntu 24.04 workstation, user `danyalaziz`, `DISPLAY=:0`.
  Ollama and ssh to the robot run here. Check where you are with
  `[ -f /run/.containerenv ] && echo CONTAINER || echo HOST`.
- **Container `ubuntu22-gpu`** (distrobox): ROS 2 Humble, Nav2 1.1.20,
  slam_toolbox 2.6.10, RViz 11.2.26, Fast DDS 2.6.11 / rmw_fastrtps 6.2.10.
  Everything ROS runs here. Every `distrobox enter ubuntu22-gpu -- bash -c
  "..."` call is a fresh shell.
- **Container `vla-box`:** YOLO-World detection server, `~/yolo_server.py`,
  HTTP on port 5001 (`/detect`, `/set_classes`). Started by hand:
  `distrobox enter vla-box`, then
  `source ~/yolo-env/bin/activate && python ~/yolo_server.py`.
  (The handout sometimes calls it `yolo_server_v2.py`; no such file exists,
  the fixes #20–#24 are inside `yolo_server.py`.)
- **Ollama** on the host, port 11434, model `qwen2.5:7b`, used by
  `~/llm_brain.py`. Was running all day as `ollama serve`.
- **Robot:** TurtleBot 4 Lite. Raspberry Pi 4 (Rev 1.5, no fan) at
  `10.42.0.169`, user `ubuntu`, key-based ssh from the host works. iRobot
  Create 3 base underneath. RPLIDAR, OAK-D camera. All robot topics under
  the namespace `/robot1`. The Pi runs a Fast DDS **discovery server**
  (`fast-discovery-server -i 0 -p 11811`); every ROS program on the PC must
  register with it. `iw` is not installed on the Pi.
- **Agent:** `~/vla_agent_v28.py` (one file for sim and hardware; hardware
  needs `VLA_NS=/robot1`, which is its default, and compressed images,
  which is also its default). Operator console `~/vla_gui_v2.py`. Mission
  logs in `~/vla_logs/`.
- **The home folder is shared** between host and containers; edit files
  from anywhere.

### 1.1 Two ways to get the hardware environment, and when to use each

Simulation and hardware use opposite settings and each fails silently in
the other's. Hardware needs BOTH of these set, together:

```
FASTRTPS_DEFAULT_PROFILES_FILE=/home/danyalaziz/.ros/fastdds_super_client.xml
ROS_DISCOVERY_SERVER=10.42.0.169:11811;
```

- **In a terminal you type into:** `source ~/robot_mode.sh` (must be
  *sourced*; executed it prints its banner and sets nothing). It sources
  ROS, exports the variables, runs `fastdds shm clean`, and **restarts the
  ROS daemon** in robot mode. Verify every time with
  `env | grep -E "^(ROS_DISCOVERY|FASTRTPS)"` — two lines, or it is not set.
- **Inside scripts, `bash -c` wrappers and `xterm -e` launchers:**
  `source ~/robot_env.sh` (created 8 Sep). Same variables, no side effects.
  Never source `robot_mode.sh` in a wrapper: restarting the daemon on every
  call blinds every other terminal for ~10 s each time.

A fresh interactive terminal in the container is simulation-safe by
default (`~/.bashrc` lines 125–130 strip the two variables unless
`VLA_MODE=robot` was exported first; nothing in the workflow sets that).

## 2. State at the end of 8 Sep

- Robot **docked and charging**: `is_docked: true`, `power_supply` current
  +0.67 A, percentage 49 % and rising (it read 47 % at 13:10). Left powered
  on the dock.
- Nothing ROS-related running on the PC. `~/vla_kill.sh` was run from a
  robot-mode terminal and printed `daemon restarted in ROBOT mode (server
  10.42.0.169:11811;)`, `robot_state_publisher processes: 0`, `clean`. The
  ROS daemon is alive with both discovery variables in its environment and
  lists 11 `/robot1` nodes.
- Pi at 13:35: load 0.82, 67.6 °C, `throttled=0xe0000` (the low digit 0 =
  not throttled now; the `e` = throttling *has occurred* since boot, clears
  only on reboot). At 12:45 it had been load 6.9 and 84.7 °C, section 7.2.
- Today's SLAM map was **not** saved (the save call got no reply; SLAM was
  then stopped). `~/maps/` is empty. Every session starts SLAM fresh.

## 3. What is verified working on the real robot, with evidence

| Item | Evidence, 8 Sep |
|---|---|
| Robot-mode terminal and daemon | `ros2 node list` from a robot-mode daemon shows all Pi nodes plus PC nodes, repeatedly; daemon env checked via `/proc/<pid>/environ` |
| Base alive | `/robot1/odom` 20.0 Hz at 11:55, 12:40, 13:27, 13:35; `odom -> base_link` from `tf2_echo` every time |
| SLAM (`~/slam_vla.yaml`) | `/robot1/map` published by `slam_toolbox` at 2 Hz; `map -> base_link` resolved from 11:07 to 13:15 continuously, including 45+ min stationary on the dock; Create 3 never went silent in 6 h |
| RViz hardware layout | opens pre-configured; subscriptions on `/robot1/map` and `/robot1/robot_description` are TRANSIENT_LOCAL; `/robot1/goal_pose`, `/robot1/initialpose`, `/robot1/clicked_point` exist; panel node `/robot1/rviz_navigation_dialog_action_client` |
| Nav2 (`~/nav2_hw.launch.py`) | `Managed nodes are active` 31 s after launch (12:16:25); all seven `connected with bond` lines, 0.10–1.10 s each; `/robot1/cmd_vel` published by `velocity_smoother`; local costmap 1.67 Hz; global costmap 181×303 cells at 0.05 m |
| Navigation goals | from RViz "Nav2 Goal": `Begin navigating from (0.04,-0.05) to (-0.29,3.20)` 12:37:45 → `Reached the goal!` 12:38:02 → `Goal succeeded`; second goal `(-0.24,3.05) to (-0.61,-0.84)` 12:38:05 → 12:38:23. 3.3 m in 17 s, 3.9 m in 18 s, 0 recoveries. Afterwards TF put the robot at (-0.36,0.36) heading 70°, `is_docked: false` |
| Nav2 goal + Dock action from a script client | `~/redock.py` 13:12–13:14: Nav2 goal to (-0.56,-0.05) `SUCCEEDED` in 36 s, Dock `is_docked=True` in 15 s |
| Pi pre-flight over ssh | key works; `uptime`, apt check, `chronyc`, `vcgencmd` all answered |

Not verified today: the OAK-D under the agent, YOLO on live frames, the
agent and GUI on hardware, `go to <object>` on hardware (Task 3, section 9),
and **manual override typed into the agent's xterm** (handout §13.3 step 1,
still never done on hardware; it is the operator's only fallback for the
unreliable stop, handout §8.1).

## 4. Restart sequence to today's working state

Every step has a check. Do not go on until it passes. Run ROS commands
inside `ubuntu22-gpu`. Robot stays docked until step 9.

**Step 0 — Pi gate.** From the host:

```
ssh ubuntu@10.42.0.169 'uptime; pgrep -af "[u]nattended-upgrade|[a]pt\.systemd\.daily|[a]pt-check|[d]pkg|[a]pt-get" || echo APT CLEAR; vcgencmd measure_temp; vcgencmd get_throttled; chronyc tracking | grep "System time"'
```

Pass: load under 1.5, `APT CLEAR`, temperature under 80 °C (ideally under
60 at idle), `throttled=` ending in `0`, system time within 1 s. Fail: wait
or fix; a measurement under load is void (handout §14.15). If the time is
out: `ssh ubuntu@10.42.0.169 'sudo chronyc makestep'`. To clear the
since-boot throttle bits and get a clean baseline: `sudo reboot` on the
Pi, wait 2 min, re-run.

**Step 1 — reachable.**
`timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22" && echo REACHABLE`.
(`ping` does not work inside the container: exits 2 with no output for
every address.)

**Step 2 — a robot-mode terminal.** `distrobox enter ubuntu22-gpu`, then
`source ~/robot_mode.sh`, then `env | grep -E "^(ROS_DISCOVERY|FASTRTPS)"`
must print two lines. Above the banner you should have seen `The daemon has
been stopped` / `The daemon has been started`.

**Step 3 — daemon sees the robot.** Wait 10 s. `ros2 node list` shows
`/robot1/turtlebot4_node` and `/robot1/rplidar_composition` among ~12
`/robot1/...` nodes. Empty = blind daemon: redo step 2.

**Step 4 — base alive.**
`timeout 30 ros2 topic hz /robot1/odom` → about 20 Hz.
`timeout 20 ros2 run tf2_ros tf2_echo odom base_link --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`
→ a `Translation:` line (read the LAST lines; the first always says the
frame does not exist). No odom = Create 3 silent (handout §14.3): only a
physical power-cycle of the base fixes it; tell the user.

**Step 5 — SLAM**, in its own window. From a robot-mode terminal:

```
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot1 params:=/home/danyalaziz/slam_vla.yaml
```

Or, from a non-interactive session, the launcher pattern in section 8.1.
Check after ~20 s, from another robot-mode terminal:
`timeout 20 ros2 run tf2_ros tf2_echo map base_link --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`
→ a translation; `ros2 topic info /robot1/map -v | grep "Node name"` →
first line `slam_toolbox` (not `_NODE_NAME_UNKNOWN_`).

**Step 6 — RViz**, in its own window, from a robot-mode terminal:

```
ros2 run rviz2 rviz2 -d ~/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
```

Check: window opens already showing map, laser scan and robot model, Fixed
Frame `map`, "Global Status: Ok"; `ros2 topic list | grep -c /robot1/goal_pose`
prints 1. Slow to draw is normal (CPU rendering).

**Step 7 — Nav2**, in its own window, from a robot-mode terminal:

```
ros2 launch ~/nav2_hw.launch.py
```

Wait ~35 s. **Do not poll it with `ros2 lifecycle get` or `ros2 node list`
while it comes up** (section 5.3). Check: its output prints
`Managed nodes are active` (today: 31 s). Then from another robot-mode
terminal:

```
ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother     # must print 1
timeout 30 ros2 topic hz /robot1/local_costmap/costmap             # about 1.7 Hz
ros2 action list | grep navigate_to_pose                           # /robot1/navigate_to_pose
```

The full output is also written to `~/vla_logs/nav2_hw_<date>.log`.

**Step 8 — RViz Nav2 panel** will say "Navigation: inactive". That label is
stale (it polled once during the 20 s start-up delay and never re-polls).
Ignore it; the panel's Feedback line and the Nav2 log are live.

**Step 9 — undock.** Only when a step needs the robot off the dock.
`ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"`
from a robot-mode terminal. Today's caveat: a freshly started command-line
client may hang at "Waiting for an action server" or never print a result
even though the robot undocks (section 5.2). Confirm with
`timeout 30 ros2 topic echo /robot1/dock_status irobot_create_msgs/msg/DockStatus --once`
→ `is_docked: false`. Nav2 will also drive straight off the dock if given a
goal while docked (it did today); undocking first is the correct order.

**Step 10 — a goal.** Click "Nav2 Goal" in RViz, click the map, drag for
heading. Confirm in the Nav2 window: `Begin navigating from ... to ...`,
then `Reached the goal!` and `Goal succeeded`; RViz panel Feedback
"reached". Or, from a script, `~/redock.py` shows the long-lived-client
pattern (section 5.2) with a 30 s settle before the first call.

**Step 11 — end of session.** Dock:
`ros2 action send_goal /robot1/dock irobot_create_msgs/action/Dock "{}"`
(or `python3 ~/redock.py` after `source ~/robot_env.sh`, which first drives
to a staging pose 0.6 m in front of the dock). Confirm `is_docked: true`
AND that the battery current is positive
(`ros2 topic echo /robot1/battery_state sensor_msgs/msg/BatteryState --once | grep -E "^(percentage|current):"`).
Stop SLAM, Nav2 and RViz with Ctrl+C in their windows. Then
`~/vla_kill.sh` from a robot-mode terminal inside the container (never from
the host, never from a wrapper whose text names a ROS process, section 5.4).
It must print `daemon restarted in ROBOT mode`.

## 5. The two new hardware files — do not simplify them back to stock

### 5.1 `~/vla_hardware.rviz` (11:52, 411 lines, YAML)

RViz layout for the real robot. Displays: Grid; RobotModel from
`/robot1/robot_description`; TF showing only `map`, `odom`, `base_footprint`,
`base_link`, `rplidar_link`, `oakd_link` (of ~40 frames); LaserScan
`/robot1/scan`; Odometry `/robot1/odom`; Map `/robot1/map` (+ `map_updates`);
group "Nav2": Global Costmap `/robot1/global_costmap/costmap`, Global Plan
`/robot1/plan`, Local Costmap `/robot1/local_costmap/costmap`, Local Plan
`/robot1/local_plan`, Footprint `/robot1/local_costmap/published_footprint`;
Image `/robot1/oakd/rgb/preview/image_raw`, **disabled by default**. Fixed
Frame `map`. Tools: 2D Pose Estimate → `/robot1/initialpose`, 2D Goal Pose
→ `/robot1/goal_pose`, Publish Point → `/robot1/clicked_point`, Nav2 Goal
tool + Navigation 2 panel. View TopDownOrtho.

Non-stock settings and why:

- **Absolute `/robot1/...` topic names** (stock uses relative `map`,
  `scan`). Reverting to relative names makes every display depend on RViz
  being started inside the `/robot1` namespace; started any other way it
  shows nothing, which is exactly the "add every display by hand" state the
  user had before.
- **Durability "Transient Local" on Map, RobotModel description and both
  costmaps**, measured from the publishers with `ros2 topic info -v` (map,
  odom, robot_description are RELIABLE/TRANSIENT_LOCAL; scan is
  RELIABLE/VOLATILE). Reverting to Volatile means RViz opened after SLAM or
  the robot never receives the map / the robot model, and the user has to
  fix QoS by hand again.
- **LaserScan Best Effort**: compatible with the reliable publisher, and
  with best-effort ones in the sim. Leave it.
- **Camera display off by default**: the OAK-D loads the Pi (section 7.2).
- **The launch command's `-r __ns:=/robot1`** is not in the file and cannot
  be: RViz's Nav2 Goal tool and Navigation 2 panel ignore the layout and use
  RViz's own node namespace. Without it goals go to `/goal_pose` in the root
  namespace and `/navigate_to_pose`, and Nav2 (which listens under
  `/robot1`) never sees them. Check: `ros2 topic list | grep -c /robot1/goal_pose` = 1.

The simulation's RViz is the stock file loaded by `vla_sim.sh` line 227; the
two files share nothing.

### 5.2 `~/nav2_hw.launch.py` (12:15, ~7.4 kB, Python launch file)

Nav2 bring-up for the real robot. Starts the same eight processes as
`ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1`:
controller_server, smoother_server, planner_server, behavior_server,
bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager
(`lifecycle_manager_navigation`), in namespace `/robot1`, with the stock
parameter file `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`
(rewritten by `RewrittenYaml` under root key `/robot1` with
`use_sim_time: false`, `autostart: true` for the servers), `/tf`→`tf` and
`/tf_static`→`tf_static` remaps, `cmd_vel`→`cmd_vel_nav` on the controller,
`cmd_vel`→`cmd_vel_nav` and `cmd_vel_smoothed`→`cmd_vel` on the velocity
smoother, `SetRemap` of `/robot1/global_costmap/scan` and
`/robot1/local_costmap/scan` to `/robot1/scan`, no composition,
`RCUTILS_LOGGING_BUFFERED_STREAM=1`. Launch arguments: `params_file`
(default the stock file), `bond_timeout` (default `30.0`), `startup_delay`
(default `20.0`), `log_level`.

Non-stock settings and why — **both are required, together**:

1. **`bond_timeout: 30.0`** on the lifecycle manager (stock 4.0, and the
   stock launch files give no way to set it). After activating each server
   the manager opens a heartbeat "bond" to it and waits half the timeout
   for it to form. Through the discovery server on the Pi the fifth bond
   took longer than 2 s on 8 Sep, the manager printed
   `Server bt_navigator was unable to be reached after 4.00s by bond. This
   server may be misconfigured.` and `Failed to bring up all requested
   nodes. Aborting bringup.`, and waypoint_follower and velocity_smoother
   were never activated. velocity_smoother is what publishes
   `/robot1/cmd_vel`, so in that state Nav2 accepts goals and the robot
   never moves. Revert this and that failure returns.
2. **`autostart: False` on the manager plus a `TimerAction` that runs
   `timeout 300 ros2 service call /robot1/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'`
   after `startup_delay` seconds.** With autostart, the manager sent its
   first configure request under a second after starting; the smoother
   configured, but its reply was discarded (`failed to send response to
   /robot1/smoother_server/change_state (timeout): client will not receive
   response`) because the manager's reply channel was not yet matched at
   the smoother's side — rmw_fastrtps waits only ~0.1 s for that — and the
   manager, whose call has no timeout, hung forever with five servers
   unconfigured. The delay lets every request/reply channel match first.
   Revert this and the hang returns.

With both: third attempt of the day, `Managed nodes are active` at 31 s,
seven bonds in 0.10–1.10 s. Caveat, stated honestly in section 7.3: the
successful attempt also stopped the 5-second command-line polling of the
bring-up, so the delay and the absence of polling were not separated.

Health check after any change: `ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother`
must print 1.

## 6. Traps, as rules, each with its one-line check

**5.1 The blind daemon.** `ros2 node list`, `topic list`, `topic info`,
`lifecycle get` are answered by one shared background daemon that inherits
the environment of whichever terminal started it. Started from a terminal
without the two discovery variables (any fresh terminal, or `sim_mode.sh`)
it sees nothing on the robot, and every terminal then gets empty answers
while `topic echo` / `topic hz` keep working. Rule: only `robot_mode.sh`
(interactive) or `vla_kill.sh` from a robot-mode terminal may restart it.
Check:
`tr '\0' '\n' < /proc/$(pgrep -f ros2-daemon | head -1)/environ | grep -q ROS_DISCOVERY_SERVER && echo "daemon OK" || echo "DAEMON BLIND - source ~/robot_mode.sh"`

**5.2 Fresh clients: ~20 s to match, replies dropped meanwhile.** A newly
started ROS program on the PC becomes fully known to existing ones only
after 2–20 s (measured: new subscriber gets data after 2.0–2.4 s; existing
subscriber gets data from a new publisher after 18.7 s). Any service or
action reply to a client whose reply channel is not yet matched is dropped
within ~0.1 s and the client waits forever. So `ros2 action send_goal`,
`ros2 service call`, `ros2 param get`, `ros2 lifecycle get` from a fresh
terminal may hang while everything is healthy; three planner dry runs did
exactly that, each leaving `Failed to send goal response ... (timeout)` in
the planner's log. Rules: send goals from RViz or the agent (long-lived
clients); in a script, create the client and spin for 30 s before the first
call (`~/redock.py` does this and both its calls succeeded immediately);
put `timeout` on every ROS CLI command; never treat one hung CLI call as
evidence of a fault; never poll a bring-up with CLI tools every few seconds
(each is a new participant the Pi's discovery server must serve, and it
adds to the Pi's load). Check that it is this and not a real fault:
`grep -c "Failed to send goal response\|failed to send response" ~/vla_logs/nav2_hw_*.log`
rising = replies dropped, requests arrived.

**5.3 Polling the bring-up.** Watch the Nav2 log file, never
`ros2 lifecycle get` in a loop. Check: `grep "Managed nodes are active" ~/vla_logs/nav2_hw_<date>.log`.

**5.4 `pkill -f` and `vla_kill.sh` kill the shell that runs them.**
`pkill -f <pattern>` matches every process's full command line, including
the `bash -c "..."` wrapper running the command if the pattern text appears
anywhere in that text — a heredoc, a comment, a filename, an unrelated
`grep`. The `[n]ame` bracket trick protects only the `pkill` line itself.
`~/vla_kill.sh` group-kills anything matching `ros2 launch|ign gazebo|
vla_agent|voice_command|vla_gui|rviz2` and then `pkill -9 -f`s ~30 names
(`slam_toolbox`, `robot_state_publisher`, `controller_server`, `rviz2`,
`ign`, ...). It killed my wrapper shell three times today, and once killed a
`grep` in the same pipeline whose filter text named a process. Rules: kill
by PID (`pgrep -f` in one command, `kill <pid>` in the next); run
`vla_kill.sh` only from a plain interactive terminal, or as
`bash <scriptfile>` where the script file was created in an *earlier*
command, the running command's text is just that path, and nothing in the
script's own pipeline (a `grep`, a `tee` filename) names a ROS process;
filter its output with `tail`, not `grep`. Check before a `pkill -f`:
`pgrep -fc "<pattern>"` in a separate command; one more than expected means
one of them is you.

**5.5 `__ns:=/robot1` on the RViz command line.** Without it the goal
tools publish in the root namespace and Nav2 never sees a goal. Check:
`ros2 topic list | grep -c /robot1/goal_pose` = 1.

**5.6 `velocity_smoother` is the Nav2 health check.** It is the last server
the lifecycle manager activates and the only publisher of `/robot1/cmd_vel`
besides the Pi's joystick node. Check:
`ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother` = 1.

**5.7 `rm -rf /dev/shm/fastrtps_*` — fixed this morning, do not restore.**
Same-host Fast DDS traffic uses shared-memory segments under `/dev/shm`.
`robot_mode.sh`, `sim_mode.sh`, `vla_kill.sh` and `run_sim_stack.sh` used to
delete all of them, cutting running local nodes (SLAM, RViz) off from
anything started afterwards: map 0×0, "Frame [map] does not exist",
`_NODE_NAME_UNKNOWN_` as the map publisher, `save_map` hanging. All four now
run `fastdds shm clean` (removes only segments whose owner died). The old
line still exists in `robot_mode.sh.bak`, `sim_mode.sh.WORKING` and the four
`*.bak-20260908` copies: **never restore those without re-applying the
fix.** Check that SLAM is still reachable:
`ros2 topic info /robot1/map -v | grep "Node name"` → first line `slam_toolbox`.

**5.8 `robot_mode.sh` / `sim_mode.sh` must be sourced.** Executed, they set
nothing and still print the success banner. Check: `env | grep -E "^(ROS_DISCOVERY|FASTRTPS)"`.
And never `source ~/robot_mode.sh | tail` or `$(...)`: a pipe is a subshell
and the exports die with it.

**5.9 `/etc/turtlebot4_discovery/setup.bash` alone is not enough.** It sets
`RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID=0`, `ROS_DISCOVERY_SERVER` and
`ROS_SUPER_CLIENT` (True only with a terminal attached, False in `bash -c`),
never `FASTRTPS_DEFAULT_PROFILES_FILE`; the XML profile is what forces
super-client mode. Both variables, together, always.

**5.10 The Create 3 goes silent with no warning** (turtlebot4 issue #554):
`/robot1/odom` stops, `odom -> base_link` disappears, `create3_republisher`
still running. Only a physical power-cycle of the base fixes it. Not seen
on 8 Sep in 6 h. Check: `timeout 30 ros2 topic hz /robot1/odom`.

**5.11 The camera.** The OAK-D shuts down on the dock and starts after
undocking; `ros2 service call /robot1/oakd/start_camera` returns success
even when the camera is off. Running, it costs the Pi ~1.5 load and a full
core. Check: `timeout 30 ros2 topic hz /robot1/oakd/rgb/preview/image_raw`
(about 12.7 Hz when on; nothing when off).

**5.12 `ros2 topic echo` may fail on type lookup, not on data.** Pass the
type explicitly: `ros2 topic echo /robot1/battery_state sensor_msgs/msg/BatteryState --once`.

**5.13 The agent's xterm.** Both launch scripts start the agent in its own
`xterm -hold` so it has a real tty; manual override is typed there and
works nowhere else. Never close that window, never pipe or redirect the
agent's output (`| tee` makes it headless with override dead).

## 7. Unresolved, honestly

**7.1 `minimum_travel_distance`: 0.2 in the file, 0.0 in the rule.**
`~/slam_vla.yaml` (the live SLAM parameters, edited by the user 8 Sep
11:04) has `minimum_travel_distance: 0.2`, `minimum_travel_heading: 0.2`,
`transform_publish_period: 0.05`, `debug_logging: false`. CLAUDE.md and
handout §14.4 say it must stay 0.0 because at 0.1 the `map -> odom`
transform went stale while the robot stood still and the `map` frame
vanished (observed 19–21 Aug). On 8 Sep the robot stood on the dock for
45+ minutes at 0.2 and the map frame stayed valid throughout, and Nav2
navigated from that state. Both observations are real; they are not
reconciled (a plausible difference is `transform_publish_period`, which
re-stamps the transform on a timer, but that is a guess). Do not change the
file to 0.0 and do not delete the rule until a test settles it: park the
robot with SLAM running for 15 min at each value and watch
`tf2_echo map base_link`.

**7.2 The Pi's heat: probable cause of today's discovery latency, not
proven.** 12:45, robot off the dock, camera on: load 6.9/4 cores, 84.7 °C,
`throttled=0xe0008` (soft thermal limit active, throttling and frequency
capping have occurred), clock 1.68 GHz of 1.8, CPU 29 % user / 44 % kernel
/ 10 % soft-interrupt, apt clear, timers masked, clock synced, memory fine.
13:00, docked, camera off, PC stack still running: load 5.3, 84.7 °C, clock
1.53 GHz; 3-s sample: diagnostics updater 27–37 %, Create 3 republisher
30 %, fast-discovery-server 27 %, kernel Wi-Fi/USB threads ~27 % each,
turtlebot4_node 19–23 %, the two joystick nodes 13–14 % with no joystick.
13:27, ten minutes after SLAM/Nav2/RViz and all CLI tools on the PC were
stopped, Pi nodes unchanged: load 1.39, 70.6 °C; 13:35: load 0.82, 67.6 °C.
So most of the Pi's load and heat came from serving the PC-side graph
(74 topics, 233 services) through the discovery server, not from its own
sensors. Whether the heat *caused* the slow matching (sections 5.2, 5.2 of
the launch file) is a correlation: Nav2 worked on this architecture in
August. The settling test: on a cool Pi (`get_throttled` ending in 0),
before starting anything, run the three timing tests — a fresh subscriber
against a running `ros2 topic pub`, a fresh publisher against a running
`ros2 topic echo`, a fresh `ros2 service call` to an existing service — and
compare with 2.3 s / 18.7 s / no-reply. Then repeat with the full PC stack
up. My recommendation to the user, given: cut demand in software first
(disable the diagnostics updater/aggregator and the joystick nodes on the
Pi, keep the camera off until needed, no CLI churn, one RViz), and add a
5 V fan regardless because the evaluation-day state is the hottest state
and throttling feeds back into more load. Do not raise the firmware
temperature limit.

**7.3 Which change fixed Nav2's third attempt.** The 20 s delayed STARTUP
and the end of 5-second CLI polling were introduced together. Both are kept.
To separate them: on a cool Pi, launch `nav2_hw.launch.py` with
`startup_delay:=0.0` and no polling; if it activates cleanly, the polling
was the culprit and the delay is belt-and-braces.

**7.4 Would the stock `nav2.launch.py` work on a cool Pi?** Never retried
cleanly. Worth one try for the evaluation narrative; `nav2_hw.launch.py`
is safe either way.

**7.5 The dock let go once.** 12:52 docked; 13:05 `is_docked: false`,
battery falling, robot position unchanged (0.02, 0.02 vs docked 0.04, -0.05),
no goal sent, no velocity published. Re-docked 13:14. At 13:35 it was
charging (+0.67 A). If the battery is not well above 49 % in the morning,
the dock contacts or the dock's power supply need looking at before any
test. Battery readings today were non-monotonic (48 → 54 → 47 → 49 %); the
Create 3's estimate is coarse, so use the `current` sign, not the
percentage, to know if it is charging.

**7.6 Manual override on hardware** (handout §13.3 step 1) has never been
typed into the agent's xterm. It is the only operator fallback for the
unreliable stop (handout §8.1). Test it stationary before the first
autonomous drive under the agent.

## 8. Working in this environment as a model

**8.1 Launch pattern for SLAM, RViz and Nav2 from a non-interactive
session.** Write a launcher script into a scratch directory (not the home
folder), then open it in an xterm from the host. The scripts used today
(they lived in the job's temp dir and are gone; recreate as needed):

```
#!/bin/bash                                   # slam_xterm.sh
source /home/danyalaziz/robot_env.sh
ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot1 params:=/home/danyalaziz/slam_vla.yaml
echo; echo "[SLAM exited -- window kept open; close it when done]"; exec bash
```
```
#!/bin/bash                                   # rviz_hw_xterm.sh
source /home/danyalaziz/robot_env.sh
ros2 run rviz2 rviz2 -d /home/danyalaziz/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
echo; echo "[RViz exited -- window kept open]"; exec bash
```
```
#!/bin/bash                                   # nav2_hw_xterm.sh
source /home/danyalaziz/robot_env.sh
ros2 launch /home/danyalaziz/nav2_hw.launch.py 2>&1 | tee -i -a /home/danyalaziz/vla_logs/nav2_hw_$(date +%Y%m%d).log
echo; echo "[Nav2 exited -- window kept open]"; exec bash
```

Open each with (from the host, `DISPLAY=:0` is set):
`( cd ~ && nohup distrobox enter ubuntu22-gpu -- setsid xterm -T "NAV2 hw" -geometry 130x35 -e bash /path/nav2_hw_xterm.sh >/dev/null 2>&1 & )`.
RViz takes 30–60 s to appear. `tee -i` ignores Ctrl+C so the log survives
the shutdown. Never `tee` the *agent* (section 5.13).

**8.2 Reading windows.** No ImageMagick, xdotool or scrot on the host, and
GNOME's screenshot D-Bus call is denied. What works: find the window id
with `xwininfo -root -tree | grep '<title>'`, capture with
`xwd -id <id> -silent -out f.xwd`, convert with Pillow inside the container
(Pillow 9.0.1 is there): read the 100-byte big-endian header (fields 4,5 =
width,height; 7 = byte order; 11 = bits per pixel; 12 = bytes per line;
19 = ncolors), skip `header_size + ncolors*12` bytes, then
`Image.frombytes("RGB",(w,h),data,"raw",mode,bpl)` with mode `BGR` for
24 bpp / `BGRX` for 32 bpp when byte order is 0. The Nav2 xterm at 130×35
shows the last 35 log lines; the tee'd log is easier.

**8.3 Other environment facts.** `~/.ros/log/<date>/launch.log` contains
only "process started" lines, not node output; use the tee'd log. Bare
`sleep` as a top-level command is blocked by the harness; `sleep` inside a
loop or a script file is fine. Every ROS CLI call needs `timeout` (30–45 s
is realistic today). `who -b` on the Pi reports a stale boot date; use
`uptime`. SSH to the Pi from the container has no key; use the host. The
ROS daemon's environment is readable from the host via `/proc/<pid>/environ`
(shared PID namespace). `iw` is absent on the Pi, so handout §13.2's Wi-Fi
power-save check (`iw dev wlan0 set power_save off`) cannot be re-run as
written and the current power-save state is unknown. `ros2 service list`
also lists names that only have a *client* (RViz's panel creates clients
for `/robot1/lifecycle_manager_localization/is_active` and `manage_nodes`);
there is no localization lifecycle manager on hardware, SLAM provides the
`map` frame. A fresh `ros2 topic hz` can under-read for its first seconds
while it is being matched (odom once read 13.6 Hz during heavy discovery
churn, 20 Hz every other time); repeat before concluding anything. Two goals in a row from RViz are fine; the agent
and RViz can both be up.

## 9. Task 3 — the next job: the full VLA stack on hardware

Bring up in this order, confirming each stage before the next. Camera only
after undocking and only after the Pi gate passed. Stop and re-read the Pi
temperature after the camera comes on; if it climbs past 80 °C, record it
and tell the user — that is the fan decision.

1. **Sections 4 steps 0–7**: gate, robot-mode terminal, base alive, SLAM,
   RViz, Nav2 active.
2. **Undock** (step 9). `is_docked: false`.
3. **OAK-D camera.** It should start by itself after undocking; if not,
   `ros2 service call /robot1/oakd/start_camera std_srvs/srv/Trigger` (its
   "success" means nothing). The only check is data:
   `timeout 30 ros2 topic hz /robot1/oakd/rgb/preview/image_raw` → 10–13 Hz.
   The agent uses compressed RGB (`/robot1/oakd/rgb/preview/image_raw/compressed`)
   and depth from `/robot1/oakd/stereo/image_raw` by default on hardware
   (no remaps; remaps are for the sim only). Then
   `ssh ubuntu@10.42.0.169 'uptime; vcgencmd measure_temp'`.
4. **YOLO** in a `vla-box` terminal:
   `distrobox enter vla-box`, `source ~/yolo-env/bin/activate && python ~/yolo_server.py`.
   Check: `curl -s -m 5 -o /dev/null -w "%{http_code}\n" http://127.0.0.1:5001/detect`
   answers (any HTTP code = server up; the agent's "I see: ..." reply is the
   real check). Vocabulary changes only work at startup via
   `YOLO_CLASSES="a,b,c"`; `/set_classes` at runtime fails on this machine
   (handout §8.6).
5. **Ollama** on the host: `pgrep -af "ollama serve"`; if absent,
   `ollama serve` in a host terminal. Check:
   `curl -s -m 5 localhost:11434/api/tags | grep -o 'qwen2.5:7b'`.
6. **Agent**, in its own xterm with a real tty, from a robot-mode terminal
   (this is what `vla_robot.sh` section 8 does):
   `setsid xterm -hold -sb -sl 5000 -T "VLA AGENT — type 'manual override' HERE" -e python3 -u ~/vla_agent_v28.py &`
   No `--ros-args` remaps on hardware. Check after ~30 s:
   `timeout 25 ros2 topic echo /vla/status --once --full-length` shows
   `camera: True, map: True`. `/vla/status`, `/vla/command`, `/vla/reply`
   are absolute (not under `/robot1`). The agent creates its Nav2 action
   client (`/robot1/navigate_to_pose`) and dock/undock clients at start-up,
   so section 5.2 does not affect it; give it 30 s before the first command.
7. **Manual override test, stationary** (section 7.6): type
   `manual override` in the agent window, confirm it enters teleop and
   `q` exits. Do this before any autonomous drive.
8. **GUI last**, from a robot-mode terminal: `cp ~/.vla_gui.robot.json ~/.vla_gui.json`
   (what `vla_robot.sh` section 6 does), then `python3 ~/vla_gui_v2.py`.
   Do not press START ALL (it kills the running stack and cannot start YOLO
   or Ollama). If the status bar shows stale `camera no / map no`, close and
   reopen the GUI; the agent's own `/vla/status` is the truth.
9. **Operator command** in the GUI: `go to <object>` for something YOLO
   can see (`chair` is the best-tested class). Chain to confirm: reply
   appears in the GUI conversation panel → agent prints "I see: <object>"
   → Nav2 window prints `Begin navigating from ... to ...` → robot moves
   → `Goal succeeded` → agent reports arrival. `what do you see` is a safe
   first command that moves nothing.
10. **End**: dock (`~/redock.py` or the dock action), Ctrl+C the agent
    window last-but-one (the GUI first), then `~/vla_kill.sh` from a
    robot-mode terminal.

`~/vla_robot.sh` exists as a one-command bring-up (reachability, clock,
env, kill, GUI config, camera, agent in xterm, GUI) but it does **not**
start SLAM, Nav2 or RViz, calls `start_camera` while still docked (handout
§8.8), and has not been run since today's discovery findings. Prefer the
manual stages above for Task 3.

## 10. Files touched today (8 Sep), all under `/home/danyalaziz`

Created: `vla_hardware.rviz`, `nav2_hw.launch.py`, `robot_env.sh`,
`redock.py`, `HANDOVER_TO_OPUS.md`, `RESUME_NEXT_SESSION.md`,
`CHANGELOG_2026-09-08.md`, `vla_logs/nav2_hw_20260908.log` (+`.attempt1`),
`maps/` (empty), backups `CLAUDE.md.bak-20260908`,
`PROJECT_HANDOUT_v5.md.bak-20260908`, `vla_kill.sh.bak-20260908`,
`robot_mode.sh.bak-20260908`, `sim_mode.sh.bak-20260908`,
`run_sim_stack.sh.bak-20260908`.
Modified: `vla_kill.sh`, `robot_mode.sh`, `sim_mode.sh`, `run_sim_stack.sh`
(morning, shared-memory fix + daemon guard, section 5.7), `CLAUDE.md`,
`PROJECT_HANDOUT_v5.md`. Not modified: `slam_vla.yaml` (user's edit at
11:04), `.bashrc`, `.ros/fastdds_super_client.xml`, anything under `/opt`,
anything in the simulation path.

## 11. Reference numbers

Topics (all under `/robot1`): `odom` 20 Hz RELIABLE/TRANSIENT_LOCAL;
`scan` 8 Hz RELIABLE/VOLATILE; `map` 2 Hz RELIABLE/TRANSIENT_LOCAL;
`robot_description` TRANSIENT_LOCAL; `oakd/rgb/preview/image_raw` ~12.7 Hz
when on; `cmd_vel` publishers: `velocity_smoother`, `behavior_server` (×5),
`teleop_twist_joy_node` (Pi); `dock_status` (`is_docked`, `dock_visible` is
false even when docked); `battery_state` (`percentage`, `current` > 0 =
charging). Actions: `/robot1/navigate_to_pose`, `/robot1/compute_path_to_pose`,
`/robot1/dock`, `/robot1/undock`. Frames: `map` → `odom` (slam_toolbox) →
`base_footprint` → `base_link` → `rplidar_link`, `oakd_link`, ~35 more.
Dock position in today's map frame: robot docked at (0.04, -0.05) heading
-3° (facing +x); staging pose used for docking (-0.56, -0.05) heading 0.
Nav2 goal tolerance 0.25 m / 0.25 rad, max speed 0.26 m/s (stock). Global
costmap 181×303 at 0.05 m, origin (-3.47, -7.60). Pi healthy idle load
0.6–0.8 (Aug), 0.82 at 67.6 °C on 8 Sep after the PC stack stopped.
Workstation Wi-Fi to the Pi: 1.1 ms RTT, 0 % loss, -40 dBm, 72 Mbit/s,
retries 4.4 % (three other stations share the hotspot).
