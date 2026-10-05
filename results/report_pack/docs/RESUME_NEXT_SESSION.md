# Resume note — written 8 Sep 2026, 13:20, for the next hardware session

Read this first. Then `CHANGELOG_2026-09-08.md` (Part 1 = morning daemon /
shared-memory fixes, Part 2 = RViz layout and Nav2 on hardware) and
`PROJECT_HANDOUT_v5.md` §15.

## Where we stopped

Stopped deliberately at 13:00 because the Pi was thermally throttled
(84.7 °C, `throttled=0xe0008`, clock capped to 1.5 GHz) and Task 3 would only
add load. Everything through Nav2 is verified on hardware; Task 3 (camera,
YOLO, Ollama, agent, GUI) has not been started on hardware today.

**Verified working on the real robot, 8 Sep:**

- Robot-mode terminal, daemon in robot mode, `node list` complete (morning).
- SLAM from `~/slam_vla.yaml`, map published, `map -> base_link` live for
  the whole session (11:07 to 13:15), Create 3 never went silent.
- RViz with the new hardware layout `~/vla_hardware.rviz`: opens
  pre-configured, correct QoS, goal tools in `/robot1`.
- Nav2 with the new hardware launch `~/nav2_hw.launch.py`: all seven servers
  active in 31 s; two goals (3.3 m in 17 s, 3.9 m in 18 s) driven from RViz;
  a third Nav2 goal plus the dock action driven from a script at the end.
- Dock / undock actions from a long-lived client.
- (Today's map was NOT saved: the `save_map` call at 13:16 got no reply
  within 90 s and SLAM was then stopped. `~/maps/` is empty. Tomorrow's SLAM
  starts fresh, as every session has.)

**End state at shutdown (13:27):** robot docked (`is_docked: true`), battery
reading 47 % and NOT yet rising 12 min after docking (see section 3), SLAM /
Nav2 / RViz stopped, xterms closed, `vla_kill.sh` run from a robot-mode
terminal and printed `daemon restarted in ROBOT mode`, `robot_state_publisher
processes: 0`, `clean`; daemon sees 11 `/robot1` nodes; Pi left powered on
the dock. Ten minutes after the PC-side stack was stopped the Pi read load
1.39 and 70.6 °C (from 5.3 and 84.7 °C), `throttled=0xe0000` (soft limit no
longer active; only the since-boot history bits remain).

## 0. THE THERMAL GATE — run this before anything else

```
ssh ubuntu@10.42.0.169 'uptime; pgrep -af "[u]nattended-upgrade|[a]pt\.systemd\.daily|[a]pt-check|[d]pkg|[a]pt-get" || echo APT CLEAR; vcgencmd measure_temp; vcgencmd get_throttled; chronyc tracking | grep "System time"'
```

Pass = load under 1.5, `APT CLEAR`, temperature under 80 °C, `throttled=`
ending in 0, system time within 1 s. If the temperature is already above
70 °C at idle, the Pi has not cooled and the enclosure is the problem: do not
start the camera until it is fixed or the reading is under 60 °C.

The "has occurred" bits (the `e` in `0xe0008`) only clear on a reboot. A
reboot of the Pi on a cool morning gives a clean baseline:
`ssh ubuntu@10.42.0.169 'sudo reboot'`, wait 2 min, re-run the gate.

Also worth doing once, on the cool Pi, before anything else is started: the
three timing tests from the changelog §9.4 (fresh subscriber, fresh
publisher, fresh service call). If they come back well under a second, heat
was the cause of today's discovery latency and the fan question in §15.5 is
answered.

## 1. Getting back to today's working state, in order

Each step has a check. Do not go on until it passes. "Robot-mode terminal"
= a terminal inside `ubuntu22-gpu` after `source ~/robot_mode.sh`, in which
`env | grep -E "^(ROS_DISCOVERY|FASTRTPS)"` prints two lines.

1. **Gate** (section 0).
2. **Reachable:** `timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22" && echo REACHABLE`.
3. **Robot-mode terminal:** `distrobox enter ubuntu22-gpu`, then
   `source ~/robot_mode.sh`, then the `env | grep` check. Expect "The daemon
   has been started" above the banner.
4. **Daemon sees the robot:** wait 10 s, `ros2 node list` shows
   `/robot1/turtlebot4_node` and `/robot1/rplidar_composition`. Empty list =
   blind daemon, redo step 3.
5. **Base alive:** `timeout 30 ros2 topic hz /robot1/odom` about 20 Hz;
   `ros2 run tf2_ros tf2_echo odom base_link --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`
   prints a `Translation:` line (read the last lines). No odom = Create 3
   silent, power-cycle the base.
6. **SLAM**, in its own robot-mode terminal:
   `ros2 launch turtlebot4_navigation slam.launch.py namespace:=/robot1 params:=/home/danyalaziz/slam_vla.yaml`
   Check after 20 s: `tf2_echo map base_link` (same remaps) prints a
   translation; `ros2 topic info /robot1/map -v | grep "Node name"` says
   `slam_toolbox`.
7. **RViz**, in its own robot-mode terminal:
   `ros2 run rviz2 rviz2 -d ~/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`
   Check: map, scan and robot model visible on open, "Global Status: Ok",
   `ros2 topic list | grep -c /robot1/goal_pose` prints 1.
8. **Nav2**, in its own robot-mode terminal:
   `ros2 launch ~/nav2_hw.launch.py`
   Wait ~35 s for `Managed nodes are active`. Do NOT poll it with
   `ros2 lifecycle get` while it comes up. Check from another terminal:
   `ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother` prints 1;
   `timeout 30 ros2 topic hz /robot1/local_costmap/costmap` about 1.7 Hz.
9. **Undock:** `ros2 action send_goal /robot1/undock irobot_create_msgs/action/Undock "{}"`.
   From a fresh terminal this may hang without printing a result even though
   the robot undocks (reply dropped, changelog §9.4). Watch
   `ros2 topic echo /robot1/dock_status irobot_create_msgs/msg/DockStatus --once`.
10. **Goal:** click "Nav2 Goal" in RViz. Confirm with `Begin navigating` /
    `Reached the goal!` / `Goal succeeded` in the Nav2 terminal and the
    panel's Feedback "reached".

## 2. Task 3 — the full VLA stack, the remaining item

Bring up in this order, confirming each before the next. The camera is the
step that loads the Pi (+1.5 load, a full core), so it goes on only when the
robot is undocked and only once the gate has passed.

1. **Steps 1–8 above** (SLAM, RViz, Nav2 running, robot still docked).
2. **Undock** (step 9). Check `is_docked: false`.
3. **OAK-D camera.** The camera shuts down on the dock and starts after
   undocking; `start_camera` returns success even when it is off (§8.8), so
   the only check is data:
   `timeout 30 ros2 topic hz /robot1/oakd/rgb/preview/image_raw` about 10–13 Hz.
   Then re-read the Pi: `ssh ubuntu@10.42.0.169 'uptime; vcgencmd measure_temp'`.
   If it climbs past 80 °C, stop and note it; that is the fan decision.
4. **YOLO**, in a `vla-box` terminal:
   `distrobox enter vla-box`, `source ~/yolo-env/bin/activate && python ~/yolo_server.py`
   (or `yolo_server_v2.py`, the version with per-class floors — check §9 of
   the handout for which one is current). Check: it prints its listening
   port and answers a request; the agent's "I see:" reply is the end-to-end
   check.
5. **Ollama**, on the host: `pgrep -af ollama` (it was running all day as
   `ollama serve`); `curl -s localhost:11434/api/tags | grep -o qwen2.5[^\"]*`
   lists the model.
6. **Agent**, in its own xterm with a real tty (manual override lives
   there): from a robot-mode terminal,
   `xterm -T "VLA AGENT" -hold -e bash -c 'source ~/robot_mode.sh; python3 ~/vla_agent_v28.py' &`
   Check: `timeout 25 ros2 topic echo /vla/status --once --full-length`
   shows `camera: True, map: True`. Never pipe or redirect its output. The
   agent creates its Nav2 and dock clients at start-up, so the fresh-client
   problem does not affect it; give it 30 s before the first command.
7. **GUI last**, from a robot-mode terminal: `python3 ~/vla_gui_v2.py`.
   Do not press START ALL. If it shows stale status, close and reopen it.
8. **Operator command** through the GUI: `go to <object>`. Chain to confirm:
   command → LLM reply in the conversation panel → "I see:" with a
   detection → `Begin navigating` in the Nav2 terminal → robot moves →
   `Goal succeeded`.
9. **Before step 8**, handout §13.3 step 1 still stands: manual override
   has never been typed into the agent's xterm on hardware. Test it while
   the robot is stationary before the first autonomous drive.
10. End: dock, then `~/vla_kill.sh` from a robot-mode terminal.

## 3. Things to remember tomorrow

- **Fresh-client rule.** `ros2 action send_goal`, `ros2 service call`,
  `ros2 param get` from a new terminal may hang while everything is healthy.
  A long-lived client that waits 30 s after creation worked every time
  today (RViz, the lifecycle manager, the re-dock script). Never conclude a
  fault from one hung CLI call.
- **`~/nav2_hw.launch.py` is hardware only.** The sim never reads it.
  Stock `nav2.launch.py` was never retried on a cool Pi; doing so once
  would tell the evaluation whether the file is a heat workaround or a
  permanent need.
- **`~/slam_vla.yaml` has `minimum_travel_distance: 0.2`.** CLAUDE.md and
  §14.4 say 0.0. The map frame stayed valid through 45 min docked today at
  0.2. Not reconciled; do not change either without a test.
- **The dock reported "not docked" at 13:05** with the robot sitting on the
  dock (position unchanged since docking at 12:52) and the battery
  discharging. Re-docked at 13:14 via Nav2 + dock action. Check
  `is_docked` AND that the percentage is rising, not just the flag.
- **Never `pkill -f` a pattern that appears anywhere else in the same
  command** (heredoc, comment, filename): it kills the shell running it.
  Kill by PID.
- **SSH key to the Pi is installed again** (8 Sep, `ssh-copy-id`); the
  gate runs unattended.
- **Battery: check it actually charged overnight.** At shutdown the flag
  said docked but the percentage sat at 47 % for 12 min. Earlier in the day
  it rose 48 → 54 % in 27 min on the dock. If it is not well above 47 % in
  the morning, the dock contacts or the dock's power need looking at before
  anything else.
- **`vla_kill.sh` kills by process group anything whose command line
  contains `ros2 launch`, `ign gazebo`, `vla_agent`, `voice_command`,
  `vla_gui` or `rviz2`.** From a plain interactive terminal that is fine.
  From a `bash -c "..."` wrapper (a script, a Claude session, an xterm
  `-e` line) whose command text contains one of those words, it kills the
  wrapper itself, mid-run, silently (happened at 13:17). Run it from a plain
  terminal or from a script file whose name contains none of those words.
- **Backups made today:** `~/CLAUDE.md.bak-20260908`,
  `~/PROJECT_HANDOUT_v5.md.bak-20260908`, and the four `*.bak-20260908`
  script copies from the morning.
