# DATA PACK — VLA ground robot (FYDP), compiled 12 Sep 2026

Every table is ready to paste into LaTeX (Markdown pipe tables; `pandoc -t latex`
converts them directly). Sources are given per table: a log or capture in this
folder, a live query made on 12 Sep 2026 with the stack up and the robot docked,
or a changelog section. Nothing here is estimated unless marked *approx.*

Contents of this folder:

| folder | what |
|---|---|
| `docs/` | CLAUDE.md, PROJECT_HANDOUT_v5.md (§13–§17 = session records), three changelogs, two handovers, REPORT_SECTION_HARDWARE.md (report/viva text), RESUME_NEXT_SESSION.md, PROJECT_HANDOFF(2).md. **`VLA_EVALUATION_KNOWLEDGE_PACK.md` does not exist anywhere on the workstation** — it was requested but was never written. |
| `code/` | `vla_agent_v28.py` (current agent), `llm_brain.py` (the brain the agent imports), `llm_brain_unified.py`, `yolo_server.py`, `vla_gui_v2.py`, `voice_command.py`, `frontier_explorer.py`, `redock.py`, `vla_demo.sh`, `vla_demo_stop.sh`, `vla_tools/` (launchers and probes) |
| `configs/` | `slam_vla.yaml`, `nav2_hw.launch.py`, `nav2_hw_composed.launch.py` (the one in use), `nav2_hw_slow.yaml`, `vla_hardware.rviz`, `robot_env.sh`, `robot_mode.sh`, `vla_kill.sh`, both Fast DDS profiles, the robot GUI config |
| `logs/` | Nav2 logs 8/10/11 Sep, agent run logs with `go to the chair` (28 Aug, 3 Sep, 10 Sep, 11 Sep), voice/YOLO/demo logs, `ros_graph_dump_20260912.txt` (nodes, topics, QoS), `ros_rates_tf_20260912.txt`, `frames_2026-09-12_13.02.11.pdf/.gv` (TF tree), `dds_ports_pc_20260912.txt` |
| `evidence/` | three packet captures from the Pi (`.pcap`), the RTPS decoder `rtps_decode.py`, `cam_probe.py`, `SUMMARY.txt` |
| `screenshots/` | **empty on purpose** — hardware screenshots are to be taken fresh during the cold-start demo run (RViz with path, GUI with detection, YOLO frame, robot at the chair); simulation screenshots come from the presentation files; the photo of the robot needs a phone. |

---

## 1. Software stack — exact versions (queried 12 Sep 2026)

| layer | component | version | where |
|---|---|---|---|
| OS | Ubuntu (workstation host) | 24.04.4 LTS, kernel 6.17.0-14-generic | host |
| OS | Ubuntu (ROS container `ubuntu22-gpu`, distrobox) | 22.04.5 LTS | container |
| OS | Ubuntu (YOLO container `vla-box`, distrobox) | 22.04 (Python 3.10.12) | container |
| OS | Ubuntu (robot, Raspberry Pi) | 22.04.5 LTS, kernel 5.15.0-1080-raspi | Pi |
| ROS | ROS 2 Humble `ros-humble-ros-core` | 0.10.0 (PC build 20260423; Pi build 20250722) | both |
| ROS | `rclpy` | 3.3.21 | PC |
| DDS | Fast DDS `ros-humble-fastrtps` | **2.6.11** (PC) / **2.6.10** (Pi) | both |
| DDS | `rmw_fastrtps_cpp` | 6.2.10 (PC) / 6.2.8 (Pi) | both |
| DDS | Discovery | Fast DDS Discovery Server on the Pi, `-i 0 -p 11811`; PC participants SUPER_CLIENT via XML | both |
| Nav | Nav2 `navigation2` / `nav2-bringup` / `nav2-bt-navigator` | 1.1.20 | PC |
| SLAM | `slam_toolbox` (sync) | 2.6.10 | PC |
| Viz | RViz2 | 11.2.26 | PC |
| Robot pkgs | `turtlebot4-navigation` 1.0.5, `turtlebot4-desktop` 1.0.0, `turtlebot4-simulator` 1.0.3 | PC |
| Robot pkgs | `turtlebot4-bringup` 1.0.3, `turtlebot4-node` 1.0.5, `depthai-ros-driver` 2.11.2, `rplidar-ros` 2.1.4, `irobot-create-msgs` 2.1.0 | Pi |
| Sim | Gazebo Sim (Ignition Fortress) | 6.16.0, `ros-ign-gazebo` 0.244.24 | PC |
| LLM | Ollama | 0.30.8 | host |
| LLM | model tag `qwen2.5:7b` | Qwen2 family, 7.6 B parameters, **Q4_K_M** GGUF, 4.68 GB, digest `845dbda0ea48` | host |
| Detector | YOLO-World via Ultralytics | weights **`yolov8s-world.pt`** (YOLOv8-s World), `ultralytics` 8.4.82 | vla-box |
| Detector deps | PyTorch 2.12.1 (CUDA 13.0 build), torchvision 0.27.1, Flask 3.1.3, OpenCV 4.11.0 | vla-box |
| Speech | `faster-whisper` 1.2.1 (model `small.en`, float16 on GPU), CTranslate2 4.8.1, `sounddevice` 0.5.5, cuBLAS 12.9.2.10 + cuDNN 9.24.0.43 (pip) | container |
| GUI | PyQt5 5.15.6 / Qt 5.15.3 | container |
| Python | 3.12.3 (host) / 3.10.12 (both containers) / 3.10 (Pi) | — |
| GPU | NVIDIA driver 595.71.05, CUDA 13.2 (driver API) | host |
| Vision libs | OpenCV 4.5.4, NumPy 1.26.4 (container) | container |

## 2. Hardware inventory

| item | detail | price (*approx., list, USD — verify before citing*) |
|---|---|---|
| Robot platform | **TurtleBot 4 Lite** (Clearpath Robotics / iRobot) | ~1,200–1,600 (kit) |
| Base | iRobot **Create 3** (differential drive, cliff/bump/IR/optical-flow sensors, dock; connected to the Pi over USB-Ethernet 192.168.186.2/3) | ~300 standalone |
| Computer | **Raspberry Pi 4 Model B Rev 1.5, 4 GB RAM** (3,789 MB usable), Broadcom `brcmfmac` Wi-Fi (2.4/5 GHz, SDIO), firmware `c72ad6b2` | ~55–75 |
| LiDAR | **RPLIDAR A1M8** (Slamtec), 2-D, 360°, ~7.5–7.8 Hz scan rate measured, ~1,080 beams/scan, mounted ~15 cm above the floor, over CP210x UART bridge | ~100 |
| Camera | **Luxonis OAK-D Lite** (Intel Movidius Myriad X, IMX214 colour sensor, stereo pair), USB 3 (USB SPEED: SUPER); run **colour-only** at 250×250 preview, 29–30 Hz | ~150 |
| Battery | Create 3 Li-ion; measured drain 37 %/h with the camera on (10 Sep) | in kit |
| Workstation | **Dell Precision 3660** tower | ~2,500–3,500 configured |
| CPU | Intel Core **i9-12900** (12th gen, 16 cores / 24 threads) | — |
| RAM | 31 GB usable (32 GB) | — |
| GPU | **NVIDIA RTX A4000, 16 GB** (GA104GL) | ~1,000–1,200 |
| Storage | Samsung PM9A1 NVMe 512 GB + WD 2 TB HDD | — |
| Network | Intel Alder Lake CNVi Wi-Fi (AP mode = the robot's hotspot, **2.4 GHz ch 6, 20 MHz**), Intel I219-LM + I225-LM Ethernet (LAN 192.168.24.189) | — |
| Audio in | Bluetooth earbuds `Airbud 595` (HFP headset profile, mSBC 16 kHz mono); the PC has no built-in mic | — |

## 3. YOLO-World vocabulary and per-class confidence floors (`yolo_server.py`)

Model `yolov8s-world.pt`, input 640 px, NMS IoU 0.50, max 50 detections, global floor 0.05 (the per-class floor is the real filter). Vocabulary is fixed at start-up (`YOLO_CLASSES` env var); runtime `set_classes` is unreliable on this machine (#21).

| class | floor | note |
|---|---|---|
| person | 0.35 | a printed figure on a poster scored 0.10 and was reported before the floor was raised; real people score 0.60–0.90 |
| box | 0.35 | catch-all for rectangular things; strictest object floor |
| cardboard box | 0.15 | |
| shelf | 0.25 | counters/partitions land here at ~0.10 |
| door | 0.20 | |
| chair | 0.30 | the main false-positive offender; real chairs 0.53 at 2.3 m, 0.174 at 0.13 m (too close, cut off) |
| pillar | 0.30 | |
| docking station | 0.15 | kept easy — the agent geo-fences the dock so a false dock is harmless (#19/#22) |
| charging dock | 0.15 | same |
| computer monitor / computer tower / poster (optional, not in the default list) | 0.30 / 0.30 / 0.25 | |

Measured inference: ~88–90 ms per 250×250 frame (HTTP round trip, GPU), warm-up 0.6–3.7 s.

## 4. LLM brain — command → action mapping (`llm_brain.py`, model `qwen2.5:7b`, temperature-free JSON plan)

Output schema: `{"steps":[{"action":…, "target":…, optional fields, "speech":…}]}`. The verb decides the action, never the article ("go to **a** chair" = navigate).

| action | meaning | trigger words | fields |
|---|---|---|---|
| `navigate` | drive to ONE named object, stop in front of it | go to / drive to / move to / head to / approach / navigate to / come to X | `target`; optional `qualifier` ∈ nearest, farthest, leftmost, rightmost |
| `locate` | look for an object and REPORT only, no driving | find / locate / search for / look for / is there / can you see / scan for X | `target` |
| `find_another` | look for one MORE instance of an already-found object | only if the operator says another / one more / a different / the other / else / additional / second one | `target` |
| `move` | precise relative motion, no object | move forward/back N m, turn left/right N° | `distance_m`, `rotation_deg` |
| `count` | count one object now | count the chairs | `target` |
| `describe` | report what the camera currently sees | what do you see | — |
| `explore` | map an UNKNOWN area autonomously (frontier exploration) | map the area, explore | — |
| `patrol` | move around a KNOWN area | patrol the area | — |
| `feed` | open the live camera window | show camera, live feed | — |
| `dock` / `undock` | Create 3 dock actions | dock, go home, go charge / undock | — |
| `forget` | erase remembered object positions | forget the chair, clear your memory | `target` or null |
| `answer` | talk only | what can you do, are you ok | `speech` |
| `reject` | refuse the impossible, offer an alternative | fly, fetch, go outdoors | `speech` |
| `clarify` | ask one follow-up | no object and no clear task | `speech` |

**Sign conventions** (ROS right-hand rule):

| field | positive | negative | unit help in the prompt |
|---|---|---|---|
| `distance_m` | forward | backward | 1 "block" ≈ 1 m, 1 "step" ≈ 0.5 m, "a little" ≈ 0.3 m, "a lot" ≈ 2 m |
| `rotation_deg` | **LEFT / anticlockwise** | **RIGHT / clockwise** | "turn around" = 180 |

Deterministic guards applied *after* the LLM (documented in the brain header): #15 verb-overrides-article (a motion verb forces `navigate` even if the model said `locate`); the rotation sign is re-derived from the operator's own words (left/right/clockwise) because the 7 B model flips it ("[brain-guard] command says RIGHT — forcing rotation_deg to −90", seen 11 Sep); `find_another` only with an explicit "another"-type word; invalid steps dropped, and `clarify` if nothing valid remains. Control words (`cancel`, `stop`, `halt`, `abort`) bypass the LLM entirely (#1, #25 typo-tolerant); bare "yes"/"no" with nothing pending is ignored (#26); conversation context expires (#27).

Voice front-end (`voice_command.py`): 16 kHz, push-to-talk from the GUI over `/vla/voice/trigger`; utterances < 0.35 s or < 3 characters ignored; PRIORITY words (cancel/stop/halt/abort) forwarded before any filtering; BLOCKED words (quit/exit/shutdown) never forwarded; wake word "robot" in hands-free mode only.

## 5. Agent fixes #1 – #64 (one line each; from the header of `vla_agent_v28.py`)

| # | what broke | what fixed it |
|---|---|---|
| 1 | "stop"/"cancel" could be swallowed by politeness or a pending question | control words preempt everything, matched before the LLM |
| 2 | *(number never used — the series skips from #1 to #3 in every version)* | — |
| 3 | "go to the chair" with several chairs picked arbitrarily | nearest instance by default; farthest/leftmost/rightmost on request |
| 4 | patrol revisited the same places | coverage memory + forward-biased frontier scoring |
| 5 | goals sent into walls / unknown space | Nav2 paths around obstacles; goals validated against the map |
| 6 | "go to X" could search forever | give-up-and-report timeout |
| 7 | "find X" drove to it | find/look for = locate and report only; go to = drive |
| 8 | TF looked up at "now", not when the frame was taken | TF at the frame's capture time (paired frames) |
| 9 | no precise relative moves | "move forward 2 m", "turn right 30°" implemented as odometry-checked moves |
| 10 | motion attempted while docked | managed dock/undock, auto-undock before any motion |
| 11 | "now go to it" had no referent | conversation context handed to the brain (last action/target) |
| 12 | colour and depth frames used from different instants | ApproximateTimeSynchronizer pairs RGB and depth |
| 13 | one hallucinated frame sent the robot chasing a ghost | target must be seen SIGHT_CONFIRM (2) times before commitment; hold still between looks |
| 14 | no record of runs | timestamped mission log per run in `~/vla_logs/` |
| 15 | YOLO client opened a new HTTP session per call and could hang | one pooled session, throttled error message, logging |
| 16 | dock state read with the wrong QoS; dock result unchecked | correct /dock_status QoS + result check |
| 17 | "find a chair" forgot chairs already seen | spatial memory reuse for find/count |
| 18 | projected goal off-map → silent spin to timeout | reported honestly instead of spinning |
| 19 | "can't reach it" while the object was reachable (three causes) | (a) progress-based timeout instead of a flat 60 s, (b) footprint-aware goal check (0.25 m disc), (c) ring of approach goals around the object |
| 20 | no visual evidence of what YOLO saw | annotated live feed with boxes and distances; detections cached per frame |
| 21 | false "Arrived at the person" metres away (occluder at the box centre) | (a) median depth over a grid across the whole box, (b) teleport gate: a big jump needs a second look |
| 22 | the dock was detected as a chair (cascade) | dock no-detect geo-fence |
| 23 | "I remember…" spam and stale memory | live-first memory: current sighting beats memory |
| 24 | no way to erase memory | "forget" action |
| 25 | "cancle" sailed past the control-word check | typo-tolerant control words |
| 26 | a lone "yes"/"no" triggered actions | bare yes/no with nothing pending is ignored |
| 27 | old context misrouted new commands | conversation context expires |
| 28 | "I can see 2 chairs" without positions | every instance reported with distance |
| 29 | Create 3 cliff sensors refuse reverse near edges → endless retries | backup-limit ratchet |
| 30 | goals re-sent on every jitter; one Nav2 refusal blacklisted neighbours | (a) sticky goals with GOAL_REISSUE 0.9 m, (b) range-scaled acceptance gate, (c) blacklist radius 0.22 m |
| 31 | "go to the chair" failed when the chair was beyond the mapped area | incremental "hop" goals toward off-map targets |
| 32 | live feed froze (four causes) | (a) stale pair dropped after PAIR_STALE_S, (b) no head-of-line blocking, (c) feed TTL, (d) YOLO timeout (2 s, 5 s) |
| 33 | commands only from the keyboard | remote command bridge: `/vla/command`, `/vla/reply`, `/vla/status` (GUI + voice) |
| 34 | feed only when asked | feed publishes to any subscriber |
| 35 | "cancel" did not stop the robot | action-level cancel service + zero-velocity barrage |
| 36 | patrol asked "do you want a live feed?" | prompt removed |
| 37 | Nav2 action server not found on hardware | `/robot1` namespace everywhere |
| 38 | depth topic name wrong on the real OAK-D | `stereo/` depth topic |
| 39 | sensor subscriptions never matched the driver's publishers | best-effort sensor QoS |
| 40 | depth read with colour pixel coordinates (different resolution and aspect) | un-project through colour K, re-project through depth K |
| 41 | depth units guessed from magnitude | encoding-based mm/m conversion (16UC1 vs 32FC1) |
| 42 | TF listener on bare `/tf` on hardware | explicit namespaced `/robot1/tf` |
| 43 | action-server discovery timed out through the discovery server | longer cached action-server wait |
| 44 | undock retried forever | retry-loop breaker |
| 45 | 1 Hz depth never paired with 15 Hz colour | sync slop 0.6 s |
| 46 | duplicate camera subscriptions | one subscription per camera stream |
| 47 | goals pointed the wrong way while turning | TF at the COLOUR frame's stamp (bearing and pose from the same instant) |
| 48 | goals computed from seconds-old depth | reject depth older than 2 s relative to colour |
| 49 | feed built and published every cycle | annotated feed gated per topic |
| 50 | raw RGB over Wi-Fi (~18× larger) | compressed RGB transport |
| 51 | raw depth over Wi-Fi | compressedDepth transport |
| 52 | depth intrinsics assumed 1280×720 while frames were smaller | intrinsics rescaled to the received frame size |
| 53 | stereo intrinsics arrived late | seeded depth intrinsics |
| 54 | depth ranging unreliable | **LiDAR ranging of detections** along the camera bearing (depth becomes the fallback) |
| 55 | LiDAR failures silent | failure reasons reported once each ("0 valid beams within 5.8° of bearing…") |
| 56 | continuous rotation blurred the laggy camera | step-and-stare scan, SCAN_DWELL_S 3.5 s |
| 57 | robot parked so close the chair left the field of view | STOP_DISTANCE 1.0 m, stand-offs 1.00/1.30 m |
| 58 | every re-send cancelled the Nav2 path → jerky motion | smoother approach (GOAL_REISSUE raised from 0.5 to 0.9 m) |
| 59 | collision check only at 1 Hz and only when "close" | 10 Hz collision guard, MIN_FRONT_CLEAR 0.70 m |
| 60 | no way to take over by hand | manual override teleop in the agent's terminal (dead-man 0.35 s) |
| 61 | LiDAR range picked the wall behind thin objects | nearest-cluster LiDAR ranging |
| 62 | guard ignored direction of travel | direction-aware collision guard |
| 63 | console printing crashed without a tty | headless-safe console output |
| 64 | camera gate required a depth frame; the depth pipeline overloads the Pi | `VLA_RGB_ONLY=1` (hardware only): colour alone opens the gate, LiDAR ranges |
| 64b | arrival never announced at the 1.30 m stand-off + 0.25 m Nav2 tolerance | `VLA_ARRIVE_TOL` (hardware 0.35–0.60; default 0.35) |
| 64c | collision guard at 0.70 m stopped every drive/move near the dock or people | `VLA_MIN_FRONT_CLEAR` (hardware 0.35 m; default 0.70) |
| 64d | robot parked 1 m short of the object | `VLA_STOP_DISTANCE` / `VLA_STANDOFFS` (hardware 0.60 / 0.60,0.80; 0.45 fails inside Nav2 inflation; default 1.0 / 1.0,1.3) |
| voice | rare phrases misheard as common ones through clipped earbud audio | `VLA_VOICE_PROMPT` vocabulary hint + `medium.en` + confidence gate −0.75 (hardware launcher) |

Key constants (v28): THINK_PERIOD 1.0 s, STOP_DISTANCE 1.0 m, ARRIVE_TOL 0.35 (0.60 hw), APPROACH_STANDOFFS (1.00, 1.30) m, GOAL_REISSUE 0.9 m, ROBOT_CLEARANCE 0.25 m, MIN_FRONT_CLEAR 0.70 m, SCAN_DWELL_S 3.5 s, SIGHT_CONFIRM 2, SEARCH_TIMEOUT 120 s, NO_PROGRESS_LIMIT 60 s, SYNC_SLOP_S 0.6, PAIR_STALE_S 1.5, MAX_PAIR_SKEW_S 2.0, LiDAR half-span 1°–12°.

## 6. Every measured number

### 6.1 Sensor and graph rates (12 Sep, stack up, robot docked; `logs/ros_rates_tf_20260912.txt`)

| topic | rate | notes |
|---|---|---|
| `/robot1/scan` (RPLIDAR A1) | **7.80 Hz** (7.3–7.8 across sessions) | ~1,080 beams, ~8 kB/scan |
| `/robot1/odom` | **19.96 Hz** | 20.0–20.4 measured on 8/10/11 Sep |
| `/robot1/tf` | 50.5 Hz | odom→base_link from the Create 3 + map→odom from SLAM |
| `/robot1/imu` | 33.9 Hz | |
| `/robot1/map` (slam_toolbox) | 2.0 Hz | |
| `/robot1/global_costmap/costmap` | 1.00 Hz | 0.5–0.6 Hz under the 11 Sep storm |
| `/robot1/local_costmap/costmap` | 1.67 Hz | |
| `/robot1/battery_state` | 0.2 Hz | |
| `/robot1/tf_static` | latched | |

### 6.2 Camera latency and rate (colour vs RGBD)

| pipeline | date | colour rate | depth rate | stamp-to-arrival latency (median) | laser latency same moment |
|---|---|---|---|---|---|
| RGB only, fresh after undock, under the Nav2 storm | 11 Sep 08:20 | 29.9 Hz | — | **85 ms** (n=529, p90 174) | 175 ms |
| RGBD (colour + stereo depth) | 10 Sep | 15.8 Hz | 4.52 Hz (`compressedDepth`), stereo `camera_info` 29.5 Hz | **4,300 ms** (spread 4,254–4,372, stable over 60 s) | 178 ms |
| RGBD, under the storm | 11 Sep 08:22–08:40 | **0** (no frames for 20 min, camera node 83 % CPU) | 0 | — | 199 ms |
| RGB only, after `oakd_pipeline.py RGB` | 11 Sep 08:43 | 28.1 Hz | — | **53 ms** (p90 70) | 143 ms |
| RGB only, fresh Pi, storm fixed | 11 Sep 09:49 | 28.9 Hz | — | **47 ms** (p90 50) | 129 ms |

Depth pipeline cost on the Pi: +7.3 °C (66.2 → 73.5 °C, 10 Sep). Laser latency itself: 129–200 ms depending on link load.

### 6.3 Network — the Fast DDS heartbeat storm (11 Sep; `evidence/`)

| condition (PC ↔ Pi over Wi-Fi) | PC→Pi | Pi→PC | source |
|---|---|---|---|
| daemon only | 1 kB/s | 0–1 kB/s | `/proc/net/dev` |
| + SLAM | 4 | 84 | |
| + SLAM + RViz | 10 | 183 | |
| + Nav2 (per-process launch, stock topics) | **1,757** | 1,387 | 08:24 |
| + Nav2 composed, stock topics | 2,057–2,653 | 450–1,150 | 09:37 |
| + Nav2 composed, remapped except lifecycle manager | ~1,000 | 390 | 09:41 |
| + Nav2 composed, fully remapped | **24** | 280 | 09:45 |
| + agent (camera at 29 Hz) | 29 | 925 | 09:53 |
| Nav2 stopped (SIGINT) while storming | 1,757 → **105** within 12 s | — | causal test |

Packet-level (Pi `tcpdump`, decoded with `rtps_decode.py`):

| capture | direction | duration | packets | rate | composition |
|---|---|---|---|---|---|
| `01_pc_to_pi_nav2_storm.pcap` | PC → Pi | 3.93 s | 29,062 | **7,404 pkt/s** (804 kB/s payload) | 25,416 HEARTBEAT (87 %), 2,501 DATA, 1,004 ACKNACK; per Pi participant 845–1,217 HEARTBEAT/s |
| `02_pi_to_pc_camera_storm.pcap` | Pi → PC | 2.89 s | 27,899 | **9,658 pkt/s** (1,216 kB/s) | 26,762 HEARTBEAT from one writer (camera node, entity 0x1803, 178 unacked samples), 4,700/s to each of SLAM and RViz, HEARTBEAT count advancing 2,729,331→2,736,018 |
| `03_pc_to_pi_after_partial_remap.pcap` | PC → Pi | 2.85 s | 22,124 | 7,772 pkt/s | 19,247 HEARTBEAT from the (unremapped) lifecycle manager's parameter-events writer |
| first `tcpdump` (6 s, not saved) | both | 6 s | 118,289 | ~20,000 pkt/s | 50,000 from `bt_navigator`'s socket 10.42.0.1:54193 |

ACKNACK content from PC readers to the storming Pi writer: `base=304, numBits=0, missing=0, final=True`, 330× in 4 s (valid full acknowledgements, ignored). Pi kernel UDP drops during the storm: 0. Pi port 7421 receive queue: constant 42 kB. Each Pi heartbeat sent twice before the interface whitelist (6,688 distinct counts → 13,374 packets).

### 6.4 Pi temperature and load

| moment | load (1 min) | temp | throttled | context |
|---|---|---|---|---|
| 10 Sep idle | 0.72 | 51.1 °C | 0x0 | nothing on the PC |
| 10 Sep + SLAM + RViz + Nav2 | 2.34 | 57.4 °C | 0x0 | no storm that day |
| 10 Sep + OAK-D colour | 2.61 | 66.2 °C | 0x0 | |
| 10 Sep + OAK-D depth | 2.36 | 73.5 °C | 0x0 | peak of the day |
| 8 Sep, Nav2 up (storm, unrecognised) | 5–7 | 84.7 °C | **0xe0008** (soft limit active, 1.5 GHz cap) | bond timeouts |
| 11 Sep 08:00 after reboot | 1.00 | 55.5 °C | 0x0 | |
| 11 Sep 08:24 storm | **7.11** | 80.8 °C | 0x80000 | camera stuck |
| 11 Sep 08:44 storm | 6.18 | 83.7 °C | **0xe0008** | |
| 11 Sep 09:45 storm fixed, Nav2 composed + remapped | 1.60 | 64.2 °C | 0x0 | |
| 11 Sep 09:53 + agent + camera | 1.88 | 67.6 °C | 0x0 | |
| 12 Sep idle | 0.51 | 51.6 °C | 0x0 | |

Per-process CPU under the storm (Pi, 11 Sep): camera container 82–88 %, `create3_republisher` 41 %, `turtlebot4_diagnostics` 29 %, `turtlebot4_node` 23–29 %, `rplidar` 23 %, `joy_linux_node` 18 %, kernel `brcmf_wq` (Wi-Fi driver) 35 % + 24 %. Normal: diagnostics 29 % (it subscribes to the raw camera image), create3 12–18 %, everything else < 6 %.

### 6.5 Nav2 timings

| launch | date | result | activation time | bonds | notes |
|---|---|---|---|---|---|
| stock `turtlebot4_navigation nav2.launch.py` | 8 Sep | **aborted** ×2 | — | bond timeout 4 s "bt_navigator unable to be reached" | storm + 4 s timeout |
| `nav2_hw.launch.py` (bond 30 s, STARTUP +20 s) | 8 Sep | active on 3rd attempt | 31 s | 7/7 | |
| `nav2_hw.launch.py` | 10 Sep | active first attempt | ~30 s | 7/7 | |
| `nav2_hw.launch.py` | 11 Sep 08:05 | active | ~55 s | 7/7 | storm began |
| `nav2_hw.launch.py` 2nd launch | 11 Sep 08:53 | active then **"controller_server IS DOWN"** → reset | 48 s (controller bond 22 s) | 7/7 then lost | storm |
| `nav2_hw_composed.launch.py`, stock topics | 11 Sep 09:16 | active | ~35 s | 7/7 | storm |
| `nav2_hw_composed.launch.py`, remapped | 11 Sep 09:45 and every launch since (4×) | active | ~30 s | 7/7, 0 errors | no storm |

Fresh-client matching delay through the discovery server: 2.0–2.4 s (subscriber), up to 18.7 s (publisher) measured 8 Sep; service replies dropped if the reply channel is not matched within ~0.1 s → all robot-side tools settle 30 s before their first call.

### 6.6 Navigation runs

| date | command | distance | time | outcome |
|---|---|---|---|---|
| 8 Sep | RViz Nav2 Goal | 3.3 m | 17 s | reached, 0 recoveries |
| 8 Sep | RViz Nav2 Goal | 3.9 m | 18 s | reached, 0 recoveries |
| 10 Sep | goal (−2.0, 0.0) | 1.9 m | 75 s | reached after wobbling into unexplored space (retreated 0.22 → 0.91 m) |
| 11 Sep 09:59 | voice "go about that" → navigate chair | chair at 2.77 m | 1st goal replanned (status 6), 3rd goal 6 s | stand-off reached, no "Arrived" |
| 11 Sep 10:00 | `go to the chair` | chair at 2.33 m | **4 s** | stand-off reached, no "Arrived" |
| 11 Sep 10:02 | `go to the chair` (from 3.3 m, memory-seeded, camera re-confirmed en route) | 3.3 m | **18 s** | **"Arrived at the chair."** |
| 11 Sep 10:24 | `redock.py` | staging pose + Dock action | 10 s + 21 s | docked, +0.59 A |
| 10 Sep | `redock.py` | Nav2 leg 12 s | Dock action 79 s | **Dock ABORTED** (status 6), docked by hand |
| 11 Sep 09:48 | `undock_hw.py` | — | 6 s | undocked |

Perception numbers: YOLO HTTP round trip 88 ms; chair 0.53 confidence at 2.3 m, 0.174 at 0.13 m; person 0.70–0.74; laser range at the chairs' bearings 2.34 m / 2.30 m vs the agent's "2 m" (10 Sep); a chair at 4–5 m reported as 7.1 m (11 Sep, laser through the legs). Whisper `small.en` on the A4000: model load 4.1–5.5 s, transcription 0.08–0.39 s per utterance. Ollama `qwen2.5:7b`: cold load 34.2–37.6 s, warm reply 0.15–0.3 s. Battery: 66 → 29 % over ~1 h undocked with the camera on (10 Sep); 41 → 33 % over the 11 Sep session's ~50 min of driving and idling.

### 6.7 Demo bring-up (`vla_demo.sh`, stages 0–8 with the robot docked)

| run | time to stage 8 | note |
|---|---|---|
| 11 Sep 10:32 | 83 s | |
| 11 Sep 10:43 | 77 s | |
| 12 Sep 12:44 (to stage 3) | 29 s | |
| 12 Sep 12:58 | 125 s | Ollama cold load 37.6 s included |
| 12 Sep 13:22 (full, 13 stages) | 151 s | first full cold start; go to the chair 5 s and 11 s, go to the person arrived |
| 14 Sep 08:59 / 10:51 / 11:12 (full) | 228 / 185 / 161 s | 13/13 each; SLAM+RViz remap in use, no frame loss in 2 h |

## 7. Topics, QoS, TF frames, ports

### 7.1 Key topics and QoS (12 Sep, `logs/ros_graph_dump_20260912.txt` has all 111 topics with every endpoint)

| topic | type | publisher (P) / subscribers (S) | P reliability / durability | S reliability / durability |
|---|---|---|---|---|
| `/robot1/scan` | sensor_msgs/LaserScan | P rplidar_composition (Pi); S slam_toolbox, global_costmap, local_costmap, rviz, turtlebot4_diagnostics | RELIABLE / VOLATILE | BEST_EFFORT / VOLATILE (all) |
| `/robot1/odom` | nav_msgs/Odometry | P create3_repub; S bt_navigator, controller_server, rviz | RELIABLE / TRANSIENT_LOCAL | BEST_EFFORT (Nav2), RELIABLE (rviz) / VOLATILE |
| `/robot1/tf` | tf2_msgs/TFMessage | P create3_repub, robot_state_publisher, slam_toolbox; S every TF listener | RELIABLE / TRANSIENT_LOCAL (create3), RELIABLE / VOLATILE (others) | RELIABLE / VOLATILE |
| `/robot1/tf_static` | tf2_msgs/TFMessage | P robot_state_publisher, create3_repub (camera frames via oakd) | RELIABLE / TRANSIENT_LOCAL | RELIABLE / TRANSIENT_LOCAL |
| `/robot1/map` | nav_msgs/OccupancyGrid | P slam_toolbox; S global/local costmap, rviz | RELIABLE / TRANSIENT_LOCAL | RELIABLE / TRANSIENT_LOCAL |
| `/robot1/cmd_vel` | geometry_msgs/Twist | P velocity_smoother (Nav2), behavior_server, teleop_twist_joy; S create3_repub | RELIABLE / VOLATILE | BEST_EFFORT / VOLATILE |
| `/robot1/battery_state`, `/robot1/dock_status`, `/robot1/imu` | BatteryState / DockStatus / Imu | P create3_repub; S turtlebot4_node, turtlebot4_diagnostics | RELIABLE / TRANSIENT_LOCAL | BEST_EFFORT / VOLATILE |
| `/robot1/oakd/rgb/preview/image_raw/compressed` | sensor_msgs/CompressedImage | P oakd (image_transport); S agent | BEST_EFFORT / VOLATILE (sensor-data QoS) | BEST_EFFORT / VOLATILE (#39) |
| `/robot1/global_costmap/costmap`, `/robot1/local_costmap/costmap` | nav_msgs/OccupancyGrid | P Nav2 costmaps; S rviz | RELIABLE / TRANSIENT_LOCAL | RELIABLE / TRANSIENT_LOCAL |
| `/robot1/plan` | nav_msgs/Path | P planner_server; S rviz | RELIABLE / VOLATILE | RELIABLE / VOLATILE |
| `/vla/command`, `/vla/reply`, `/vla/status`, `/vla/voice/trigger`, `/vla/voice/transcript`, `/vla/voice/state` | std_msgs/String (Bool for trigger) | GUI / voice node / agent | RELIABLE / VOLATILE, depth 10 (rclpy default) | same |
| `/vla/annotated/compressed` | sensor_msgs/CompressedImage | P agent; S GUI | RELIABLE / VOLATILE | same |
| `/parameter_events` | rcl_interfaces/ParameterEvent | P and S: **every** rclcpp node (21 endpoints on the Pi side alone) | RELIABLE / VOLATILE, keep-last 1000 | RELIABLE / VOLATILE |
| `/pc/parameter_events`, `/pc/rosout` | ParameterEvent / Log | PC Nav2 nodes (remapped 11 Sep) | RELIABLE / VOLATILE; rosout TRANSIENT_LOCAL | — |

(Per-endpoint values for all 111 topics: `logs/ros_graph_dump_20260912.txt`. Note the mismatch pattern that matters for the storm: the Pi publishes its sensor topics RELIABLE and every rclcpp node reads `/parameter_events` RELIABLE.)


### 7.2 TF frame tree (12 Sep, `logs/frames_2026-09-12_13.02.11.pdf`)

`map → odom` (slam_toolbox) → `base_link` and `base_footprint` (Create 3 odometry) → `rplidar_link`, `imu_link`, `oakd_camera_bracket → oakd_link → {oakd_rgb_camera_frame → oakd_rgb_camera_optical_frame, oakd_left/right_camera_frame → …_optical_frame, oakd_imu_frame}`, wheels (`wheel_drop_left/right → left/right_wheel`), `front_caster_link`, bump/cliff/IR/button frames (`bump_*`, `cliff_*`, `ir_intensity_*`, `ir_omni`, `button_*`, `mouse`, `bumper`). 41 edges in total.

### 7.3 Network addresses and ports

| what | value |
|---|---|
| PC hotspot / robot | 10.42.0.1 (PC, `wlp0s20f3`) ↔ 10.42.0.169 (Pi, `wlan0`); ssh key-based as `ubuntu` |
| PC LAN (must NOT be advertised to DDS — whitelist) | 192.168.24.189 (`enp0s31f6`) |
| Pi ↔ Create 3 | 192.168.186.3 (Pi `usb0`) ↔ 192.168.186.2 (Create 3) |
| Fast DDS Discovery Server | 10.42.0.169:11811 UDP (`fast-discovery-server -i 0 -p 11811`, GUID prefix `44.53.00.5f.45.50.52.4f.53.49.4d.41`), `discovery.service` on the Pi |
| DDS participant ports (domain 0) | metatraffic unicast 7410 + 2·id, user unicast 7411 + 2·id: Pi participants seen on 7413–7435; PC participants 7410–7435 (`logs/dds_ports_pc_20260912.txt`) |
| YOLO server | http://127.0.0.1:5001/detect (POST JSON, base64 JPEG) |
| Ollama | http://127.0.0.1:11434/api/chat |
| Chrony/NTP on the Pi | in sync (0.2 µs); the PC is **not** NTP-synced |
| Wi-Fi link | 2.4 GHz channel 6, 20 MHz, −46 dBm at the PC, 72 Mbit/s (PC→Pi) / 43 Mbit/s (Pi→PC) negotiated, 13,007 retries logged |
