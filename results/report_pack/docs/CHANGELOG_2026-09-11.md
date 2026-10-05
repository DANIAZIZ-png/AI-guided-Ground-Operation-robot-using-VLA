# CHANGELOG — 11 September 2026 (last hardware session)

Companion to `PROJECT_HANDOUT_v5.md` §17 and `CLAUDE.md`. Evidence files in
`~/vla_evidence_20260911/` (packet captures + decoder), logs in `~/vla_logs/`
(`demo_20260911_*.log`, `nav2_hwc_20260911.log`, `vla_run_20260911_095239.log`,
`voice_20260911.log`).

## 0. Headline

1. **`go to the chair` works on hardware, end to end, by voice and by text**:
   three drives out of three reached the chair's stand-off point (4 s, 6 s and
   ~18 s including a memory-seeded approach); the third also announced
   "Arrived at the chair". §7.
2. **The robot's Wi-Fi link was being flooded by our own navigation stack** —
   a Fast DDS HEARTBEAT storm on the RELIABLE `/parameter_events` topic:
   7,400 packets/s PC→Pi from Nav2, 9,700 packets/s Pi→PC from the camera
   node, 1.7–2.6 MB/s each way, Pi at load 7 / 84 °C / throttled, camera
   frames stuck. Proven by packet capture on the Pi and decoded to the RTPS
   submessage level. **Fixed** by remapping `/parameter_events` and `/rosout`
   off the link in the Nav2 launch: PC→Pi fell from 1,757 kB/s to **24 kB/s**,
   Pi load 7 → 1.6, temperature 84 → 64 °C. §2–§4. This also explains the
   8 Sep thermal spike and the bond timeouts that forced `nav2_hw.launch.py`.
3. **Yesterday's "4.3 s camera lag" is gone**: colour-only frames arrive in
   **47–85 ms** (laser 130–180 ms). The lag was the stereo-depth pipeline
   plus the link storm, not the driver's clock. §5.
4. The agent's depth gate is bypassed on hardware by a new flag
   `VLA_RGB_ONLY=1` (default off; simulation byte-identical in effect). §5.
5. **Voice works**: yesterday every transcription failed with
   `libcublas.so.12 not found`; fixed with `LD_LIBRARY_PATH` in a hardware-only
   launcher. Whisper transcribes in 0.1–0.4 s on the GPU. §6.
6. **One-command demo**: `~/vla_demo.sh` (13 gated stages, ~4 min from cold,
   stages 0–8 rehearsed twice) and `~/vla_demo_stop.sh` (re-docks, stops
   everything incl. YOLO). §8.

## 1. Morning: gates and the first bring-up (08:00–08:24)

Pi after the user's reboot: load 1.00, 55.5 °C, `throttled=0x0`, clock
0.2 µs off NTP. Daemon started in robot mode (`daemoncheck.sh` OK, 12 nodes).
Odom 20.0 Hz, `odom→base_link` present, battery 41 %, docked +0.63 A. SLAM,
RViz and Nav2 (`nav2_hw.launch.py`, per-process) came up: `Managed nodes are
active`, 7/7 bonds, 0 errors, `velocity_smoother` on `/robot1/cmd_vel`.
YOLO-World up (9 classes incl. chair, person), Ollama pinned (1.9 s load).

## 2. THE STORM — discovery

### 2.1 Symptoms
After the RGBD camera switch (08:21) no colour or depth frames reached the
PC at all. Pi: load 7.11, 80.8 °C, `throttled=0x80000`, every ROS node at
17–41 % CPU including `joy_linux_node` (which has nothing to do), and two
kernel threads `brcmf_wq` (the Broadcom Wi-Fi driver) at 35 % + 24 %.
`ssh` took >120 s to answer. The link carried **1.5 MB/s in each direction**
(`/proc/net/dev` on both ends: PC tx 1,757 kB/s, rx 1,387 kB/s).

### 2.2 Source
`tcpdump` on the Pi, 6 s: 118,289 packets ≈ 20,000/s. 50,000 of them from
ONE PC socket, `10.42.0.1:54193`, to six Pi ports (7415–7429), ~50–190 B
each. `ss -uanp` on the PC: port 54193 belongs to **`bt_navigator`**
(pid 462537), Send-Q 105–112 kB. Stopping Nav2 with SIGINT dropped PC→Pi
from 1,757 to **105 kB/s** within 12 s — the clean causal test.

### 2.3 What the packets are (capture `01_pc_to_pi_nav2_storm.pcap`, 3.9 s)
Decoded with `rtps_decode.py` (RTPS header + submessage parser):

    packets 29062 in 3.93 s = 7404 pkt/s
    25416  HEARTBEAT                      (87 %)
     2501  INFO_DST + INFO_TS + DATA
     1004  INFO_DST + ACKNACK
    per Pi participant: ~845-1217 HEARTBEAT/s, i.e. one every 0.8-1.2 ms

The DATA payloads are `rcl_interfaces/ParameterEvent` messages in clear
text: `/robot1/controller_server … general_goal_checker.plugin …
nav2_controller::SimpleGoalChecker`, `/robot1/global_costmap …
static_layer.map_topic`, `/robot1/bt_navigator_navigate_to_pose_rclcpp_node
… wait_for_service_timeout`. One Pi reader received the same 89 samples
**19 times each** (1,679 DATA for 89 distinct sequence numbers).

Why every Pi node: rclcpp's time source subscribes every node to
`/parameter_events` (RELIABLE, keep-last 1000). Nav2 declares ~90 parameters
per node at configure time and publishes one event per declaration.

### 2.4 The reverse direction (capture `02_pi_to_pc_camera_storm.pcap`, 2.9 s)
With Nav2 DOWN, the Pi still sent **9,658 pkt/s**: 26,762 HEARTBEATs from a
single writer, entity `0x00001803` in participant prefix `010f3af92e04d7ab`
(= Pi pid 1070, the **OAK-D camera container**, then at 82–88 % CPU), to
SLAM (PC port 7413) and RViz (7415), 4,700/s each, holding 178 unacknowledged
samples (126..303 — the camera driver's 125+ parameters plus the runtime
pipeline switches). Its HEARTBEAT `count` field advanced by one per packet
(2,729,331 → 2,736,018 in 2.9 s): the writer's own logic firing every
0.22 ms, not the transport retrying. Each heartbeat was sent **twice**
(6,688 distinct counts for 13,374 packets) — once to each PC address.

### 2.5 The acknowledgements ARE arriving
ACKNACK from SLAM's reader to the camera writer, 330× in 4 s:
`base=304 numBits=0 missing=0 final=True` — "I have everything through 303".
Addressed to the right participant GUID and port (7421). Kernel UDP counters
on the Pi: `UdpInErrors +0, UdpRcvbufErrors +0`. Yet `ss` showed port 7421
with a constant **42 kB** of unread data: the process is too busy sending
heartbeats to drain its socket. Self-sustaining once started.

### 2.6 What it was NOT (each tested)
* **Not the behaviour-tree loop rate**: the storm persisted with Nav2
  deactivated (BT does not tick without a goal): 1,753 kB/s.
* **Not eight processes**: Nav2 composed into ONE container stormed at the
  same rate (1.8–2.2 MB/s).
* **Not stale "ghost" participants**: after a Pi reboot with a fresh daemon
  (zero ghosts), the storm returned within 20 s of Nav2 launching. (Ghosts
  are real, though: the daemon listed the pre-reboot Pi nodes alongside the
  new ones — `/launch_ros_1028` and `/launch_ros_1015` — until restarted.)
* **Not the PC's XML profile**: it sets nothing but the discovery server.
* **Not the Pi's profile**: `/etc/turtlebot4/fastdds_rpi.xml` is empty.
* Fast DDS versions: PC 2.6.11, Pi 2.6.10 (rmw_fastrtps 6.2.10 / 6.2.8).

### 2.7 Things that could NOT be used as a fix (tested, ruled out)
* **Per-topic QoS from XML** (`RMW_FASTRTPS_USE_QOS_FROM_XML=1`, profile named
  `rt/parameter_events` set BEST_EFFORT): rmw_fastrtps in Humble keeps the
  ROS-specified reliability; `ros2 topic info -v` still showed RELIABLE.
  (Without a reallocating history memory policy it also crashes nodes with
  `NotEnoughMemoryException`.)
* **DDS partition from XML** on the topic profile: not applied either — a
  plain subscriber still received the "partitioned" data.
* **5 GHz Wi-Fi**: the PC's card supports AP mode on 5 GHz (80 MHz), but
  the regulatory domain is unset (`country 00`), which marks every 5 GHz
  channel "no IR" (no access point allowed). Needs `sudo iw reg set PK`
  and then `nmcli con modify Hotspot 802-11-wireless.band a channel 36`.
  Not done (needs the user's password); reversible from the PC alone.
  The link is 2.4 GHz channel 6, −46 dBm, 72/43 Mbit/s, 13,007 retries.

## 3. THE FIX

`/parameter_events` and `/rosout` are ordinary ROS topics and **can be
remapped per process** (verified: `-r /parameter_events:=/pc/parameter_events
-r /rosout:=/pc/rosout` moved both publisher and subscription). If the PC
side uses different names, PC and Pi endpoints never match, so the reliable
relationship that storms never exists — in either direction. Nothing on the
robot needs the PC's parameter events or log stream.

Applied in `nav2_hw_composed.launch.py` (every component, including the
lifecycle manager — components do NOT inherit the container's remaps; the
first attempt missed the lifecycle manager and ~1.0 MB/s remained, capture
`03_…pcap`) and in `nav2_hw.launch.py` (`SetRemap`).

Measured with SLAM + RViz + Nav2 composed, robot docked:

| condition                       | PC→Pi     | Pi→PC   | Pi load | Pi temp |
|---------------------------------|-----------|---------|---------|---------|
| daemon only                     | 1 kB/s    | 0       | 0.6     | 65 °C   |
| + SLAM                          | 4         | 84      | 0.6     | 64      |
| + RViz                          | 10        | 183     | 0.6     | 63      |
| + Nav2 composed, NOT remapped   | **2,057–2,653** | 450–1,150 | 7 | 84 (throttled) |
| + Nav2 composed, fully remapped | **24**    | 280     | 1.6     | 64      |
| + agent (camera at 29 Hz)       | 29        | 925     | 1.9     | 68      |

Nav2 composed also activates faster and more reliably (all 7 bonds in ~30 s,
0 errors, every launch today once remapped).

Second, smaller fix: `~/.ros/fastdds_hw_wifi_only.xml` — the PC's DDS profile
now whitelists the hotspot interface (10.42.0.1) so PC participants advertise
ONE address. Before, every Pi sample and heartbeat went out twice (once to
10.42.0.1, once to the LAN address 192.168.24.189 routed back through the
hotspot). `robot_env.sh` and `robot_mode.sh` point at the new file; the
shared `fastdds_super_client.xml` is untouched; the sim never reads either.

## 4. Why the Pi looked thermally weak on 8 Sep

8 Sep: 84.7 °C, `throttled=0xe0008`, "every Pi node burning CPU",
fast-discovery-server busy, Nav2 bond timeouts (4 s stock) and dropped
service replies — recorded then as "probable thermal cause, not proven".
Today's identical signature had a measured cause: the storm. Yesterday
(10 Sep) the Pi sat at load 2.3 / 57 °C with the same stack because the
storm did not trigger that day (it needs a burst of unacknowledged reliable
samples; the trigger conditions are timing-dependent). A cool Pi was
correlation; the flooded link was the mechanism.

## 5. CAMERA: colour-only, and the lag that was not a clock error

* 08:20, camera fresh after undock, colour-only: RGB **85 ms** median
  (n=529), laser 175 ms, 29.9 Hz. → the driver's stamps are honest.
* 08:21 RGBD switch: **no frames at all** for 20 min (storm + depth pipeline;
  camera node at 83 % CPU). Switched back with `oakd_pipeline.py RGB`:
  28.1 Hz, **53 ms**. After the Pi reboot: 28.9 Hz, **47 ms**.
* Yesterday's 4.3 s was measured in RGBD mode (colour halves to 15.8 Hz, +7 °C
  on the Pi). Decision: run colour-only; ranging is by LiDAR (#54) anyway.
* `vla_agent_v28.py` #64 — `VLA_RGB_ONLY=1` makes `current_pair()` return
  `(rgb, None)` when no depth frame ever arrived; every depth consumer
  (`depth_at`, `robust_box_depth`, `project_pixel`, `project_box`,
  `range_for_box`) already returns None safely; the annotated-feed label uses
  the LiDAR range when the flag is on. 13 changed lines, all guarded; default
  `"0"` is the previous behaviour. Backup `vla_agent_v28.py.bak-20260911`.
  Set only in `vla_tools/agent_xterm.sh`. Result: `/vla/status camera: true`
  with no `stereo/*` topic in existence.
* `oakd_rgbd.py` is superseded by `oakd_pipeline.py RGB|RGBD` (kept).

## 6. VOICE

Yesterday's log (`voice_20260910.log`): four clean recordings (peak ~1.0)
and four `transcription failed: Library libcublas.so.12 is not found or
cannot be loaded`. faster-whisper loads cuBLAS/cuDNN lazily at the FIRST
transcription; the libraries are pip packages in
`~/.local/lib/python3.10/site-packages/nvidia/{cublas,cudnn}/lib`, not on the
loader path. `vla_tools/voice_xterm.sh` exports `LD_LIBRARY_PATH` with both
and runs `voice_command.py --mode ptt --mic pulse` headless (stdin from
/dev/null, output visible in its window and in `voice_$DAY.log`).
`voice_command.py` unchanged.

Microphone: the PC has NO built-in mic (only a headset jack, empty); the
default input was the speaker monitor. Bluetooth earbuds `Airbud 595`
connected in `headset-head-unit-msbc` (16 kHz mono) became the default
source. Results: `what do you see` heard in 0.39 s (logprob −0.46), `go to
the chair` heard as "go about that" (the LLM still resolved it to navigate →
chair), later "stop looking for a gene" — accented English through `small.en`
mishears, but every phrase starting with *stop* was forwarded as a stop, as
designed. Push-to-talk is the GUI's gold button; `vla_demo.sh` tries to
connect both paired earbuds and warns if the input is still a monitor.

## 7. THE DEMO RUNS (agent log `vla_run_20260911_095239.log`)

| time  | command (source)                | result |
|-------|---------------------------------|--------|
| 09:54 | `what do you see` (text)        | "shelf (0.7 m)" at the dock; after `turn right 60`: "person (2.7 m)" |
| 09:59 | voice "go about that"           | chair seen at 2.77 m; 1st goal replanned (status 6), 3rd goal 6 s drive, reached stand-off; no "Arrived" |
| 10:00 | `go to the chair` (voice/text)  | chair at 2.33 m; goal, **4 s drive, reached**; no "Arrived" |
| 10:00 | "start patrolling the area"     | explore mode, two frontiers skipped ("stuck at a frontier") |
| 10:02 | `go to the chair` (text, from 3.3 m away after the patrol) | memory-seeded ("I remember seeing a chair about 3.3 m away"), camera re-confirmed on the way, **"Arrived at the chair." at +18 s** |

Distances: the user placed the chair at 4–5 m and the GUI showed 7.1 m at
one point; a 2-D laser at 15 cm height looks between chair legs and ranges
the wall behind. The agent's `[lidar] not used — only 0 valid beams within
5.8° of bearing −74.9°` line shows the #55 failure reporting working; the
projection fallback then jittered 2.1–4.4 m before the approach converged.

Why runs 1–2 stalled at the stand-off without "Arrived": arrival needs the
robot within `STOP_DISTANCE + ARRIVE_TOL` = 1.35 m of the belief; the 1.30 m
stand-off plus Nav2's 0.25 m goal tolerance can leave it at 1.55 m, so the
goal was re-sent every second (each "succeeded" in 60 ms) until the next
command. **Applied 12 Sep** (approved): agent #64b,
`ARRIVE_TOL = float(os.environ.get("VLA_ARRIVE_TOL", "0.35"))`; the hardware
launcher sets `VLA_ARRIVE_TOL=0.60`. Default unchanged; sim unaffected.
Not yet exercised in a drive — the cold-start run is the test.

Also seen: "I sent the stop but the robot is STILL MOVING (0.24 m/s) — take
manual override" during a normal ROTATE→NAVIGATING transition while Nav2 was
legitimately driving — the known stop-verification false alarm (handout
§8.1), not a runaway.

Docking: `redock.py` Nav2 leg 10 s, Dock action 21 s, `is_docked: true`,
+0.59 A. (Yesterday's abort did not recur.)

## 8. THE DEMO LAUNCHER

`~/vla_demo.sh` (host): stages 0–13 with gates; backend windows start
iconified (`xterm -iconic`), GUI and RViz visible; `VLA_DEMO_STOP_AFTER=n`
test hook. Rehearsed twice through stage 8 (docked): 83 s and 77 s.
Two bugs found by the rehearsal and fixed: a healthy Pi shows only 7
`__node:=` processes (threshold was 10); `/robot1/goal_pose` exists as soon
as Nav2 is up, so the RViz gate now looks for the RViz window.
`~/vla_demo_stop.sh`: tested (exit 0, all counts 0, daemon in robot mode).
**Run both from a plain terminal** — `vla_kill.sh` group-kills any shell whose
command line contains `rviz2`, `ros2 launch`, `vla_agent`, … (it killed my
wrapper shell once today, as CLAUDE.md predicts).

## 9. Files created or changed today (all under `/home/danyalaziz`)

Created: `vla_demo.sh`, `vla_demo_stop.sh`, `nav2_hw_composed.launch.py`,
`.ros/fastdds_hw_wifi_only.xml`, `vla_tools/{voice_xterm.sh, nav2_hwc_xterm.sh,
oakd_pipeline.py, say.py}`, `vla_evidence_20260911/`, this file,
`REPORT_SECTION_HARDWARE.md`.
Changed: `vla_agent_v28.py` (#64 flag; backup `.bak-20260911`),
`nav2_hw.launch.py` (SetRemap ×2), `robot_env.sh`, `robot_mode.sh` (DDS
profile path; backups `.bak-20260911`), `vla_tools/agent_xterm.sh`
(`VLA_RGB_ONLY=1`), `CLAUDE.md`, `PROJECT_HANDOUT_v5.md`.
On the Pi: nothing edited; rebooted twice (08:00 by the user, 09:30 by me).
NOT touched: `vla_sim.sh`, `sim_mode.sh`, `run_sim_stack.sh`,
`run_agent_sim.sh`, `.vla_gui.sim.json`, `/opt` on either machine,
`fastdds_super_client.xml`, `yolo_server.py`, `llm_brain.py`,
`voice_command.py`, `slam_vla.yaml`.

## 10. Still open

1. `VLA_ARRIVE_TOL` (§7) — applied 12 Sep, awaiting its first drive.
2. 5 GHz hotspot (§2.7) — needs `sudo`; would cut retransmissions further.
3. Why Fast DDS's writer ignores valid ACKNACKs once its socket backs up —
   the mechanism is characterised, the internal cause is not. Worth an
   eProsima issue with `01_…pcap` and `02_…pcap` attached.
4. `turtlebot4_diagnostics` subscribes to the RAW camera image at 30 Hz just
   to count frames — a standing 30 % of a Pi core. Stock behaviour; could be
   disabled on the Pi.
5. No map saved yet.

## 11. Addendum, 12 Sep morning

The robot had been power-cycled overnight. 25 min after boot the PC saw all
118 `/robot1` topics but no node names and no data (`ros2 node list` empty,
`topic echo` silent, even from a fresh super-client probe, with either DDS
profile), while a super-client run ON THE PI listed every node incl. the
Create 3's and read the laser at 7.7 Hz. tcpdump on the Pi during a PC
subscription: only the server's heartbeats crossed — the Pi's writers had
never matched the PC's readers, i.e. the discovery server was not relaying
PC endpoints to its local clients. `sudo systemctl restart discovery.service
turtlebot4.service` on the Pi, 75 s, PC daemon restart → 11 nodes, battery
99 %, docked, charging. `vla_demo.sh` stage 2 now performs that recovery
automatically (once) if the robot's nodes are not listed; rehearsed
stages 0–3 afterwards: pass in 29 s.

## 12. Addendum, 14 Sep — three fixes without the robot

1. **Camera-node storm toward SLAM/RViz (frame loss late in the 12 Sep
   session): remap applied.** `vla_tools/rviz_hw_xterm.sh` now adds
   `-r /parameter_events:=/pc/parameter_events -r /rosout:=/pc/rosout`;
   SLAM runs through a new wrapper `~/slam_hw.launch.py` (stock
   `turtlebot4_navigation slam.launch.py` inside a `SetRemap` group), used by
   `vla_tools/slam_xterm.sh`. Verified offline on an isolated domain:
   slam_toolbox has 2 endpoints on `/pc/parameter_events` and 1 on `/pc/rosout`,
   none on the old names; RViz 4 endpoints on `/pc/parameter_events`, none on
   `/parameter_events`. Effect on the frame loss to be confirmed on the robot.
   Still unremapped PC readers of `/parameter_events`: the ROS daemon and the
   `ros2 launch` processes themselves (small; no launcher-level switch).
2. **Speech: `medium.en` in the hardware voice launcher** (`VLA_WHISPER` to
   override). Test with the earbuds: the four demo phrases ("go to the chair",
   "what do you see", "stop looking for a chair", "go to the person")
   transcribed exactly by both `small.en` and `medium.en`; `medium.en`
   confidence −0.22 vs −0.25, 0.38–0.45 s per clip once warm, 1.5 GB cached.
   The earbuds' audio peaks at full scale (1.000) at both 100 % and 60 % PC
   gain — clipping happens inside the headset, PulseAudio cannot change it;
   still transcribed correctly. Practical rule: hold the button until the
   phrase is finished (the 11 Sep mishearings were short presses during motion).
3. **5 GHz hotspot: not possible with this card.** `iw reg get` shows
   `phy#0 (self-managed)` — Intel LAR: the firmware owns the regulatory domain,
   `iw reg set` is ignored, the `lar_disable` module parameter no longer exists
   in kernel 6.17, the BIOS provides no WRDD country (`dmesg`), and no 5 GHz
   beacon is in range for the card to learn from (two networks, both 2.4 GHz).
   That is exactly why the PC can *join* 5 GHz networks (it learns the country
   from them) but cannot *create* one. Also tried: a laptop broadcasting a
   5 GHz hotspot (`wlan-ap`, ch 149, 40 %) next to the PC -- its beacon carries
   no Country element (Windows hotspots send none), so the card learned
   nothing; the only Country element in range was a weak third-party AP with
   a malformed `CN` element, ignored by the firmware. Windows on the same
   dual-boot machine can create a 5 GHz hotspot because the Intel Windows
   driver takes the country from the OS region setting; Linux has no such input. Recommendation: a USB adapter with a
   non-Intel chipset (MediaTek MT7921AU / MT7612U) for a 5 GHz AP. Hotspot left
   on 2.4 GHz channel 6.

## 13. Addendum, 14 Sep afternoon — robot session (three cold starts)

* **Cold starts:** 228 s (Ollama/YOLO cold), 185 s, 161 s — 13/13 stages each.
  `go to the chair` reached the chair and announced arrival at 08:59 (8 s, after
  a `turn right 45` to clear the dock) and 10:58 (with guard 0.35 m, no stop).
* **SLAM/RViz remap verified in use:** link 25 kB/s PC→Pi with the full stack,
  Pi 63–70 °C, load 1.3–1.6, and **no camera-frame gaps in 2+ hours** — the
  12 Sep frame loss did not recur.
* **Voice, applied (approved "fix it"):** `voice_command.py` gained
  `VLA_VOICE_PROMPT` (faster-whisper `initial_prompt`; empty = old behaviour;
  backup `voice_command.py.bak-20260914`); launcher sets a command
  vocabulary, `--model medium.en`, `--min-logprob -0.75`. Evidence for the gate:
  correct phrases scored −0.25…−0.56, junk ("welcome to our dressing room",
  "go to the beach for a test drive") −0.92…−1.08. All recordings peak at
  1.0 (headset-side clipping). **User-confirmed later on 14 Sep: voice
  commands transcribe correctly with the prompt + `medium.en` + gate.**
* **Agent flags, all hardware-only, defaults unchanged, backup
  `vla_agent_v28.py.bak-20260914`:** #64c `VLA_MIN_FRONT_CLEAR` (guard
  distance; hardware 0.35 m — at 0.70 every drive/move that started near the
  dock or near people was stopped "something solid is 0.55 m ahead"); #64d
  `VLA_STOP_DISTANCE` / `VLA_STANDOFFS` (stand-off; hardware 0.60 / 0.60,0.80,
  `VLA_ARRIVE_TOL` 0.35 → arrival radius 0.95 m). **0.45 m stand-off failed:**
  robot reached 0.75 m, then all ring goals were rejected ("can't find a
  reachable path") — inside Nav2's 0.45 m inflation — and the chair left the
  camera's view (<1 m, #57) so the agent drove off to search. 0.60 was set but
  the user's manual driving overlapped the test; not cleanly verified.
* **Guard message is misleading near clutter:** with the robot beside the
  chair it had just reached, `go to the person` was stopped at 0.28 m
  (laser: 0.25–0.30 m at +30…+90°) with "my distance estimate was wrong" —
  the person estimate was correct; the near object was the chair. A
  `turn right` / `move back` clears it.
* **Docking:** the Nav2 leg to the staging pose worked (27 s); the Dock
  action aborted twice (status 6, 15 s and 59 s) with `dock_visible: false`;
  docked by hand. Total: Dock action 2 successes / 3 aborts over 10–14 Sep;
  **user-confirmed later on 14 Sep that docking works for them.** Open:
  move the staging pose so the IR receiver sees the dock from `redock.py`.
* **Sign edge case:** "turn righ t" (typo) → executed as a LEFT turn; the
  brain-guard only forces the sign for the whole word "right".
* Daemon stale after a robot power-on (0 topics in the container): a daemon
  restart is all it needs; `vla_demo.sh` stage 2 does it.

## 14. Addendum, 15 Sep — four cold starts, latency cause found, Create 3 died once more

* **Cold starts:** 206 s, 187 s, 174 s (13/13 each); one aborted at stage 7
  because the Create 3 base went silent mid-configure (see below). One run was
  refused at stage 0 because Claude Code had been started INSIDE the
  container — the script must run from a host terminal.
* **"Very high latency" was RViz's raw camera display.** Twice the robot was
  sending 2.6–3.8 MB/s (1,394 B packets at 1,900 pkt/s) and the camera
  dropped to 12 Hz: the RViz "Image" display (raw 250×250 frames) had been
  ticked on. With it off: 0.97–1.08 MB/s, camera 30–38 Hz. Removed the Image
  display from `vla_hardware.rviz` (backup `.bak-20260915`). The three other
  hotspot clients were idle (0–3 kB/s) throughout; cycling the hotspot drops
  them but they rejoin automatically — no root, no MAC filtering possible.
* **Create 3 silence, second occurrence** (first 8 Sep): odometry, battery
  and dock_status stopped, `odom→base_link` vanished from `/robot1/tf` while
  the laser, the republisher process and the base's ping all kept working.
  Battery 100 %. Fixed by the power button, ~2 min. Not battery, not load.
* **Agent #64e `VLA_ARRIVE_IF_SEEN_M`** (default 0 = off; hardware 1.2 m):
  if the target is in the current camera frame and the LiDAR puts it at
  ≤ 1.2 m, stop and announce arrival immediately — no stand-off goal. Reason:
  a person near furniture left every ring goal inside Nav2's inflation
  (20 refusals in 12 s) and the agent rescanned instead of stopping. Applies
  to every object class. Under ~1 m a CHAIR is no longer detected (#57), so
  chairs still arrive by the pose test — whose radius was too small:
  `VLA_ARRIVE_TOL` 0.35 → **0.50** (radius 1.10 m > largest stand-off 0.80 +
  Nav2 tolerance 0.25). Two drives reached the chair in 9 s and 13 s without
  announcing before the fix.
* `~/vla_tools/restart_agent_hw.sh` — restart only the agent (kill + relaunch
  in its xterm) without touching the rest of the stack.
* Docking: `redock.py` succeeded (17 s) once, robot docked by the user otherwise.
* Voice with earbuds: user reports commands transcribe well with `medium.en`
  + vocabulary prompt + gate.
