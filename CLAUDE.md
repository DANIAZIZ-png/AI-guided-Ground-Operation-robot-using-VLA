# VLA Ground Robot — FYDP context

Read `PROJECT_HANDOUT_v5.md` in this folder before doing anything (§17 is the latest, 11 Sep).
It is the authoritative record of this project. If something I say
conflicts with it, say so rather than guessing.
**Then read `CHANGELOG_2026-09-11.md`** (11 Sep 2026, the LAST hardware
session): it is the NEWEST document and wins wherever anything disagrees
with it. The demo works end to end; the bring-up is now ONE script,
`~/vla_demo.sh` (shutdown: `~/vla_demo_stop.sh`), which replaces the manual
restart sequence in `HANDOVER_2026-09-10.md` §4. That handover is still
right about the traps and the tools, but WRONG on three things that 11 Sep
settled: (1) the camera is run COLOUR-ONLY now (no RGBD switch; the agent's
depth gate is bypassed by `VLA_RGB_ONLY=1`); (2) the "4.3 s camera lag" was
the depth pipeline plus a Wi-Fi flood, not a clock problem — fresh colour
frames arrive in ~50 ms; (3) the Pi was never thermally weak — our own
Nav2 was flooding its Wi-Fi link (Fast DDS heartbeat storm on
`/parameter_events`), fixed by remapping in the hardware Nav2 launches.
`HANDOVER_TO_OPUS.md` (8 Sep) is still correct on the discovery server, the
blind daemon and the shared-memory rules. Report/viva text is in
`REPORT_SECTION_HARDWARE.md`.

## Where commands must run

You are on the Ubuntu 24.04 HOST. `distrobox` works from here.
Verify with: `[ -f /run/.containerenv ] && echo CONTAINER || echo HOST`

**ROS / Gazebo / Nav2 / the agent** — inside `ubuntu22-gpu`. Each
call is a FRESH shell, so the unsets and the ROS source must be
repeated every single time:

    distrobox enter ubuntu22-gpu -- bash -c \
      "unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID; \
       source /opt/ros/humble/setup.bash; <command>"

Simulation and hardware use OPPOSITE settings, and both fail
SILENTLY in the other's — no error, just nothing works. Never skip
the unsets.

**The same wrapper for HARDWARE** must SET the discovery variables
instead. Source `~/robot_env.sh` (env only, no side effects):

    distrobox enter ubuntu22-gpu -- bash -c \
      "source ~/robot_env.sh; <command>"

Do NOT source `~/robot_mode.sh` inside wrappers: it restarts the ROS
daemon every time, which blinds every other terminal for ~10 s. Use
`robot_mode.sh` only in a terminal you type into. Start SLAM, RViz and
Nav2 from `xterm -e bash <script>` launchers whose script sources
`~/robot_env.sh` first (pattern in `HANDOVER_TO_OPUS.md`).

**YOLO** — inside `vla-box`:

    distrobox enter vla-box -- bash -c "<command>"

**Ollama and ssh to the robot** — directly on the host.

**`~/vla_kill.sh`** — inside `ubuntu22-gpu`, from a terminal that has
sourced `robot_mode.sh`, so the ROS daemon is restarted in robot mode.
On the host it kills processes but cannot touch the daemon or shared
memory, and says so (changelog 8 Sep §1).

The home folder is SHARED between host and containers, so any file
edit works from here with no wrapping at all.

**`~/robot_mode.sh` and `~/sim_mode.sh` must be SOURCED, never executed.**
Executed, they run in a child shell, print a success banner, and
set nothing in the shell you are in. The banner is printed either
way — it is NOT evidence. After every source, verify:

    source ~/robot_mode.sh
    env | grep -E "^(ROS_DISCOVERY|FASTRTPS|VLA_NS)"

No output means the shell is not configured, whatever the banner said.
Do this in every new shell before any ROS command.

## Container gotchas

- **`ping` does not work inside `ubuntu22-gpu`.** It exists at
  `/usr/bin/ping` but exits 2 with ZERO output for every address —
  containers are not granted the raw socket ICMP needs. It fails
  identically whether the robot is up or unplugged, so it can never
  be used as a reachability test. Test reachability with bash's
  `/dev/tcp` instead — no privilege needed, and it tests a real
  connection, which is what ROS actually depends on:

      timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22"

  Exit 0 = reachable.

- **`/etc/turtlebot4_discovery/setup.bash` sets four variables:**
  `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID=0`, `ROS_DISCOVERY_SERVER`, and
  `ROS_SUPER_CLIENT` (True only when the shell has a terminal
  attached, False in any `bash -c` wrapper). It does NOT set
  `FASTRTPS_DEFAULT_PROFILES_FILE`. In a wrapper shell, sourcing it
  alone makes a plain client that receives topic data but no
  directory, which shows as an empty topic list. The XML profile
  forces super-client mode regardless of the flag, so hardware needs
  BOTH, together (corrected 8 Sep, changelog §6):

      export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_hw_wifi_only.xml
      export ROS_DISCOVERY_SERVER="10.42.0.169:11811;"

  (`robot_env.sh` does exactly this; the 11 Sep profile adds an interface
  whitelist to the original `fastdds_super_client.xml`, which is untouched.)

- **`ros2 node list` / `node info` / `topic info` are answered by the
  ROS daemon**, one shared background process. They are complete and
  stable when the daemon was started from a robot-mode terminal
  (verified repeatedly 8 Sep) and empty or partial when it was started
  from a terminal without the discovery variables. Inconsistency
  between runs means the daemon was restarted in between from a
  different terminal. Check its environment before trusting or
  distrusting it (changelog 8 Sep §5). A clean `node list` still says
  nothing about the data path: `topic hz` and
  `ros2 run tf2_ros tf2_echo <a> <b>` remain the ground truth, and
  `pgrep -af <name>` for "is it running".

- **A freshly started program takes up to ~20 s to be fully matched
  through the discovery server** (measured 8 Sep, changelog §9.4).
  `ros2 action send_goal`, `ros2 service call` and `ros2 param get`
  from a new terminal may hang or get no reply while everything is
  healthy: the reply is dropped if the caller's reply channel is not
  matched within ~0.1 s. Long-lived clients (RViz, the agent, Nav2's
  lifecycle manager after its delay) are unaffected. Never use one
  hung CLI call as the only evidence of a fault, and never poll a
  bring-up with CLI tools every few seconds: each poll is a new
  program the Pi's discovery server has to serve.

- **The Create 3 goes silent with no warning** (turtlebot4 #554):
  `/robot1/odom` stops, `/robot1/battery_state` empties,
  `odom->base_link` disappears — while `create3_republisher` is still
  running, so every process check passes. NOT a low-battery symptom
  (happened at 63%). Only fix is a physical power-cycle; tell me, do
  not try to restart nodes.

- **After a robot power-cycle the Pi's discovery server can come up
  half-deaf** (12 Sep): the PC sees all 118 `/robot1` topics but ZERO node
  names and no data, `topic echo` gets nothing, while a super-client ON THE
  PI sees every node and the laser streams. It did not recover in 25 min.
  Fix: `ssh ubuntu@10.42.0.169 'sudo systemctl restart discovery.service
  turtlebot4.service'`, wait ~75 s, restart the PC daemon. `vla_demo.sh`
  stage 2 does this automatically once. (A Pi reboot also works, 2 min.)

- **Check the Pi's load AND temperature first.**
  `ssh ubuntu@10.42.0.169 'uptime; vcgencmd measure_temp; vcgencmd get_throttled'`.
  Want load under 1.5, under 80 °C, and a `throttled=` value ending
  in 0. unattended-upgrades used to hit load 9.4 on 4 cores and look
  exactly like a network fault; the apt TIMERS are now masked (not
  just the service), still check. On 8 Sep the Pi sat at 84.7 °C,
  `throttled=0xe0008` (soft thermal limit active, clock capped to
  1.5 GHz) and load 5–7 with apt clear; the discovery server slowed
  down and Nav2's bring-up failed twice on timing (changelog §9).
  The OAK-D alone adds ~1.5 to the load and a full core — keep the
  camera stopped unless a step needs it.

- **`pkill -f <pattern>` matches the whole command line of every
  process, including the shell running it.** The `[n]ame` bracket
  trick protects only the pkill line itself. If the pattern text
  appears anywhere else in the same command (a heredoc, a comment, a
  filename) the shell kills itself silently and nothing after that
  line runs (happened 8 Sep, changelog §9.6). Find the PID with
  `pgrep -f` in one command and `kill <pid>` in the next.
  **`~/vla_kill.sh` has the same property**: it group-kills anything
  whose command line contains `ros2 launch`, `ign gazebo`, `vla_agent`,
  `voice_command`, `vla_gui` or `rviz2` — including a `bash -c` wrapper
  that merely mentions one of those words. Run it from a plain terminal
  or via a script file, never inline in a wrapper that names them.

- **slam_toolbox `minimum_travel_distance` must stay at 0.0.** At 0.1
  the `map->odom` transform goes stale while the robot is stationary
  and the map frame vanishes.

- **THE OAK-D RUNS COLOUR-ONLY. Do NOT switch it to RGBD any more** (11 Sep).
  The depth pipeline costs the Pi +7 °C, halves colour to 15 Hz and, under
  load, delayed frames by seconds (that was 10 Sep's "4.3 s lag"). Colour-only
  frames arrive in 47–85 ms at 29 Hz. The agent's camera gate used to need a
  depth frame; on hardware `vla_tools/agent_xterm.sh` exports `VLA_RGB_ONLY=1`
  (agent fix #64, default off, sim unchanged) and `/vla/status` reads
  `camera: true` with no `stereo/*` topic at all. Ranging is LiDAR (#54).
  If the camera ever comes up wedged (node at 80 % CPU, no frames),
  `python3 ~/vla_tools/oakd_pipeline.py RGB` cycles it.

- **NEVER launch Nav2 on hardware without the `/parameter_events` and
  `/rosout` remaps** (they are in `nav2_hw_composed.launch.py` and
  `nav2_hw.launch.py`; `vla_demo.sh` uses the composed one). Without them
  Fast DDS floods the Pi at ~7,400 packets/s (2 MB/s), every Pi node burns
  CPU, the Pi throttles at 84 °C and camera frames stop — the 8 Sep and
  10 Sep mysteries. Health check at any time, on the host:
  `awk '/wlp0s20f3/{print $10}' /proc/net/dev` twice 5 s apart; PC→Pi above
  ~200 kB/s with the robot idle = a storm. Evidence: `CHANGELOG_2026-09-11.md`
  §2–3, captures in `~/vla_evidence_20260911/`. Do not "fix" it with XML QoS
  overrides — rmw_fastrtps Humble ignores them (tested).

- **Short-lived ROS clients leave ghosts.** A probe killed by `timeout` or
  exiting without `rclpy.shutdown()` stays in every node's view until the
  daemon is restarted (the daemon listed the pre-reboot Pi nodes next to the
  new ones). Probes must shut down cleanly (`vla_tools/say.py` does); keep
  CLI probing to a minimum on hardware.

- **`vla-box` has no `xterm`.** An `xterm -e` launcher there opens no
  window and fails silently. Run YOLO with `nohup ... > logfile` instead;
  only the agent needs a real tty.

- **`~/vla_kill.sh` does NOT stop `yolo_server`.** It is in none of its
  patterns and it runs in `vla-box`. Kill it by PID afterwards:
  `pgrep -f "[y]olo_server"` in one command, `kill -9` in the next.

- **A SHORT `ros2 node list` is not proof a node is dead.** While docked
  it omitted `turtlebot4_node` and `create3_repub` while those same nodes
  were publishing `dock_status` and `battery_state` in the same minute.
  Confirm with `topic hz` / `topic echo` before concluding anything.

- **The Dock action can ABORT.** On 10 Sep `redock.py` drove to the
  staging pose fine (12 s) and the Dock action then failed after 79 s
  (`status=6`, `is_docked=False`), leaving the robot off the charger.
  Watch it finish; verify `is_docked: true` AND `current > 0`.

- **Moving the robot by hand invalidates SLAM.** Odometry never sees it,
  so `map->odom` is wrong and Nav2 goals go to the wrong place. Restart
  SLAM, or set a 2D Pose Estimate in RViz, before navigating.

## Do not run these

- `~/vla_sim.sh` — ends with the GUI in the foreground; it will hang
  you forever. I run this one myself.
- Anything that makes the physical robot MOVE. The stop command is
  known to be unreliable (handout section 8.1). Propose the command,
  let me run it.
- Do NOT close the **VLA AGENT** xterm window. Both launch scripts
  now start the agent in its own terminal so it has a real tty —
  manual override (#60) is typed in THAT window and works nowhere
  else. Closing it kills the agent. Never pipe or redirect the
  agent's output either (`| tee`, `> log`): a pipe is not a tty, and
  it silently puts the agent back in headless mode with override
  dead. It writes its own log to `~/vla_logs/` regardless.

## How I want you to work

- One step at a time. Give the command, run it, tell me the result,
  then stop and wait for me.
- Say plainly whether each step passed or failed.
- Short answers. I am a student, not a systems engineer — explain
  terms I might not know.
- Surgical edits only. Do not rewrite files that work.
- **`vla_agent_v28.py`, `yolo_server.py`, `llm_brain.py` and
  `voice_command.py` are SHARED by the simulation and the hardware.** They
  are not in the sim-path list below, but changing them changes my demo.
  Any change must sit behind a flag or environment variable that DEFAULTS
  to the current behaviour, so only hardware takes the new branch — and
  show me the diff before applying it.
- Verify every script with `bash -n` and every Python file with
  `python3 -m py_compile` before saying it is done.

## Useful facts

- Robot (Raspberry Pi): 10.42.0.169, user `ubuntu`. Key-based ssh works
  from the host (re-installed 8 Sep). `iw` is not installed on the Pi.
- Containers: `ubuntu22-gpu` (ROS), `vla-box` (YOLO)
- Current agent: `vla_agent_v28.py`. Hardware-only env flags, all set in
  `vla_tools/agent_xterm.sh`, defaults = sim behaviour: `VLA_RGB_ONLY` (#64),
  `VLA_ARRIVE_TOL` 0.50 (#64b; must exceed largest stand-off + 0.25 − stop),
  `VLA_MIN_FRONT_CLEAR` 0.35 m guard (#64c), `VLA_STOP_DISTANCE` 0.60 /
  `VLA_STANDOFFS` 0.60,0.80 (#64d; 0.45 FAILED — Nav2 rejects goals inside its
  0.45 m inflation and the chair leaves the camera view under 1 m),
  `VLA_ARRIVE_IF_SEEN_M` 1.2 (#64e: target in frame within 1.2 m = arrived, any
  class). Restart only the agent: `~/vla_tools/restart_agent_hw.sh`.
  **Never tick the RViz "Image" display on hardware** (removed from the layout
  15 Sep): raw frames = 2–4 MB/s over Wi-Fi, camera drops to 12 Hz, everything
  lags. The Create 3 went silent a second time on 15 Sep (power-cycle fixed it).
  Camera stream default is **30 fps / JPEG 75 / 512 px** since 21 Sep (user-tested at
  distance, no lag; `VLA_CAM_FPS=10` to dial back). GUI feed = agent's annotated
  stream at 30 Hz via `VLA_ANNOT_PERIOD=0.033` (#64f, default 0.2). Browser raw feed:
  `python3 ~/vla_tools/feed_http.py raw 8081` → http://127.0.0.1:8081/. It is
  set by `vla_demo.sh` stage 10 via `vla_tools/oakd_bandwidth.py` (`VLA_CAM_FPS/JPEG/PX`,
  `VLA_CAM_FPS=0` to skip); 512 px helps far/occluded chairs, do not lower below 416.
  A resolution change invalidates the agent's cached calibration — the demo
  applies it (stage 10) BEFORE the agent starts (stage 11); if changed live,
  run `~/vla_tools/restart_agent_hw.sh`. Camera is only ~90 kB/s of a ~475 kB/s
  Wi-Fi link (rest = scan/TF/odom/IMU, uncuttable); distance lag past ~5 m is
  the 2.4 GHz link, not software — keep the graded run within ~4-5 m.
  GUI (`vla_gui_v2.py`, 21 Sep): 18 px conversation font, per-role colours
  (YOU=blue, ROBOT=green, SYSTEM=orange, error=red), Clear button. Voice: `VLA_VOICE_PROMPT` in `voice_command.py`,
  set by `vla_tools/voice_xterm.sh` with `medium.en` and `--min-logprob -0.75`.
- **DEMO DAY: `~/vla_demo.sh`** from a plain host terminal (robot docked and
  charged). 13 gated stages, ~4 min; only the GUI and RViz are visible; the
  rest are iconified xterms. It stops loudly at the first failed gate.
  `VLA_DEMO_STOP_AFTER=8 ~/vla_demo.sh` rehearses everything before the
  undock. Shutdown + re-dock: `~/vla_demo_stop.sh` (`--no-dock` to skip).
  Neither may be run inline in a command whose text names a ROS process
  (`vla_kill.sh` group-kills that shell — it happened on 11 Sep).
- Kill everything: `~/vla_kill.sh`, from a robot-mode terminal inside
  the container (it does not stop YOLO; `vla_demo_stop.sh` does).
- **RViz on hardware** (robot-mode terminal in the container):
  `ros2 run rviz2 rviz2 -d ~/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`
  The `__ns` is what puts the Nav2 Goal tool in `/robot1`; the layout
  file alone does not. Hardware only; the sim keeps the stock
  `turtlebot4_viz/rviz/robot.rviz` via `vla_sim.sh`.
- **Nav2 on hardware**: `ros2 launch ~/nav2_hw_composed.launch.py` (all
  servers in one container, `bond_timeout` 30 s, 20 s delayed STARTUP, and
  the parameter-event/rosout remaps). Activates in ~30 s with 7/7 bonds
  every time since the remap (11 Sep). `nav2_hw.launch.py` (one process per
  server) also carries the remaps and still works. Confirm
  `ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother` prints 1.
  The stock `turtlebot4_navigation nav2.launch.py` has no remaps and a 4 s
  bond timeout: it will storm and abort. Send goals from RViz's "Nav2 Goal"
  or the agent, not from a fresh CLI.
- Nav2 logs: `~/vla_logs/nav2_hwc_<date>.log` (composed) /
  `nav2_hw_<date>.log`. Demo bring-up logs: `~/vla_logs/demo_<date>_*.log`.
- DDS profile for hardware: `~/.ros/fastdds_hw_wifi_only.xml` (super client
  + interface whitelist 10.42.0.1), set by `robot_env.sh` and
  `robot_mode.sh`. The old `fastdds_super_client.xml` is untouched.
- `/opt` nav2.yaml and slam.yaml are back to STOCK (restored from
  `nav2.yaml.pre_v25` and `slam.yaml.bak`, now in `archive/params/`).
  `nav2_hw.launch.py` uses the stock `/opt` nav2.yaml.
- **CORRECTED 5 Oct 2026: `nav2.yaml.today` and `slam.yaml.today` were NOT
  tuned.** They are byte-identical to the stock backups — `nav2.yaml.today`
  to `nav2.yaml.pre_v25`, `slam.yaml.today` to `slam.yaml.bak` (verified by
  sha256). Either the tuning was reverted before being saved, or it was never
  written to those files. There is no preserved tuning. Both duplicates were
  deleted during the repository reorg; the originals they copied are in
  `archive/params/`. The gentler-speed profile that IS real and usable is
  `config/nav2_hw_slow.yaml` (0.15 m/s, 0.4 rad/s), still untested on hardware.
- `~/slam_vla.yaml` (the live SLAM params) has
  `minimum_travel_distance: 0.2`, which contradicts the 0.0 rule
  above, yet the map frame stayed valid through 45+ min docked on
  8 Sep. Not reconciled; do not change either without a test.
- `~/vla_tools/` (10–11 Sep): diagnostic and launcher scripts, all verified.
  `oakd_pipeline.py RGB|RGBD` (cycle the camera; RGB is the mode we use —
  `oakd_rgbd.py` is its RGBD-only predecessor, no longer run), `say.py`
  (send one command, print replies), `voice_xterm.sh`, `nav2_hwc_xterm.sh`,
  `undock_hw.py`,
  `nav_goal.py X Y YAW`, `range_probe.py` (YOLO box -> laser distance),
  `latency_probe.py` / `drift_probe.py` (camera lag), `scan_look.py`
  (clearance by bearing), `is_moving.py`, `yolo_live_test.py`,
  `daemoncheck.sh`, and the xterm launchers. Table in changelog §13.
- **Voice**: the GUI only publishes push-to-talk on `/vla/voice/trigger`;
  transcription is `voice_command.py`, started by
  `~/vla_tools/voice_xterm.sh` — which also sets `LD_LIBRARY_PATH` to the
  pip cuBLAS/cuDNN libraries; without that every transcription fails with
  `libcublas.so.12 not found` (10 Sep's real voice bug, fixed 11 Sep).
  The PC has NO built-in microphone: the input is Bluetooth earbuds in
  headset mode (paired: `Airbud 595` BB:86:57:3D:AF:79, `realme Buds Air 2`
  18:95:52:6B:5B:FD). `pactl get-default-source` must not be a `.monitor`.
  `small.en` mishears accented English; *stop* phrases are always forwarded.
- **THE EARBUDS KILL THE HOTSPOT (22 Sep).** The PC's Intel AX201 is Wi-Fi and
  Bluetooth on one antenna: with the earbuds connected in mic mode, ping to the
  robot was 100 % loss / 6.5 Mbit/s; disconnected, 0 % / 72 Mbit/s (A/B, same
  spot). Worse the farther the earbuds are from the PC. Before any latency
  measurement check `bluetoothctl devices Connected`. Workaround:
  `bluetoothctl disconnect BB:86:57:3D:AF:79` while driving; real fix = wired
  mic or a lazy-mic flag in `voice_command.py` (not done). Changelog §19.
  Camera defaults are back to 10 fps / JPEG 75 / 416 px, console feed 5 Hz.
- **Pin the Ollama model before a demo** or the first command after a
  5-minute pause costs 34 s (cold 34.2 s vs warm 0.15 s):
  `curl -s http://127.0.0.1:11434/api/chat -d '{"model":"qwen2.5:7b","messages":[{"role":"user","content":"hi"}],"stream":false,"keep_alive":-1}'`
- `~/nav2_hw_slow.yaml` (10 Sep, UNTESTED): stock nav2.yaml with gentler
  speeds and accelerations (0.15 m/s, 0.4 rad/s). Use with
  `ros2 launch ~/nav2_hw_composed.launch.py params_file:=/home/danyalaziz/nav2_hw_slow.yaml`.
- **Detection needs standoff.** The same chairs scored 0.174 (below the
  0.30 floor, invisible) at 13 cm from the dock and 0.53 at 2.3 m. Give
  the robot 2-3 m before asking it to find furniture; `person` scores
  ~0.70 even up close.
- Never publish to `/robot1/cmd_vel` without warning me first.
- `~/vla_tools/say.py "go to the chair" 60` sends one command on
  `/vla/command` (exactly what the GUI or voice sends) and prints the agent's
  replies and status for 60 s. Long-lived client, clean shutdown.
- **A 2-D laser sees through a chair**: from 4–5 m the agent reported 7.1 m
  (beam between the legs → wall). Recovers near the object; person/box/shelf
  are fine. Arrival ("Arrived at the chair") needs the robot within 1.35 m of
  its belief; with the 1.30 m stand-off + Nav2's 0.25 m tolerance it is
  marginal (2 of 3 drives parked correctly without announcing). Fixed 12 Sep:
  `VLA_ARRIVE_TOL=0.60` in the hardware launcher (agent #64b, default 0.35).
- 5 GHz hotspot: **impossible with this card** (14 Sep). Intel LAR firmware owns
  the regulatory domain (`phy#0 (self-managed)`); `iw reg set` is ignored,
  `lar_disable` no longer exists, BIOS has no WRDD, no 5 GHz beacon in range
  to learn from. Joining 5 GHz works, creating it does not. Fix = a MediaTek
  USB adapter. Stay on 2.4 GHz channel 6.
- SLAM on hardware: `ros2 launch ~/slam_hw.launch.py` (stock slam.launch.py
  + parameter_events/rosout remaps, 14 Sep); RViz launcher carries the same
  remaps. Voice launcher uses Whisper `medium.en` (`VLA_WHISPER` overrides).
- OPEN, unsettled: whether starting SLAM triggers the Create 3 silence
  or the base dies on its own. Do not assume either way.
- RESOLVED 11 Sep: the "4.3 s camera lag" (10 Sep) was measured in RGBD
  mode under the Wi-Fi flood. A fresh colour-only camera stamps frames
  47–85 ms old (laser 130–180 ms) — measured before and after the fixes.
  `go to <object>` works; three drives out of three reached the chair.
