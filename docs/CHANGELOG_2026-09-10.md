# CHANGELOG — 10 Sep 2026. Task 3: full VLA stack on hardware

Session ran ~09:35–10:50 PC time (Pi clock is 5 h behind: PKT vs UTC).
Robot: TurtleBot 4 Lite, Pi at 10.42.0.169. Operator: the user.
Companion documents: `HANDOVER_2026-09-10.md` (what to DO next session,
self-contained), `CLAUDE.md` (working rules), `PROJECT_HANDOUT_v5.md` §16.
The simulation path was NOT touched at any point (§9).

--------------------------------------------------------------------------

## 0. Headline

The whole stack came up on hardware and every stage passed its own check,
but the end-to-end `go to <object>` drive was NOT completed. Two things
blocked it, one now fixed and one now understood but unfixed:

1. **The OAK-D was publishing colour only.** The agent needs a depth frame
   before it will run YOLO at all. Fixed at runtime, no robot restart (§2).
2. **Camera frames carry timestamps ~4.3 s older than the laser's.** This
   is the cause of the "moves abruptly / loses the target" behaviour. Root
   cause not yet isolated; the two candidate causes need opposite fixes,
   so nothing was changed in the agent (§3).

Also fixed today: voice input (§5), Ollama demo stall (§6). Also verified
for the first time on hardware: manual override (§4).

--------------------------------------------------------------------------

## 1. Pi thermal gate — the best state yet

The Pi had been rebooted (up 5 min at first contact).

| time  | load | temp | throttled | context |
|-------|------|------|-----------|---------|
| 09:37 | 0.72 | 51.1 C | `0x0` | idle, nothing on the PC |
| 10:47 | 2.34 | 57.4 C | `0x0` | + SLAM, RViz, Nav2 |
| 09:51 | 2.61 | 66.2 C | `0x0` | + OAK-D colour |
| 10:02 | 2.36 | 73.5 C | `0x0` | + OAK-D depth pipeline |
| 10:47 | 0.19 | 58.9 C | `0x0` | everything stopped |

**Peak 73.5 C, `throttled=0x0` all day — the Pi never throttled once.**
Compare 8 Sep: 84.7 C and `throttled=0xe0008` (soft limit active, clock
capped to 1.5 GHz). The `0x0` (rather than `0xe0000`) is because the reboot
cleared the since-boot history bits.

**Conclusion: a reboot before the session is worth doing, and no fan is
needed at this workload.** The depth pipeline costs +7.3 C (66.2 -> 73.5),
which is affordable. Do NOT read this as "the fan question is closed" —
evaluation day may add a warmer room and a longer run.

Battery: 66 % at 09:44 (charged overnight on the dock, so the 8 Sep §7.5
"dock let go" worry did not repeat), 29 % at 10:45 after ~1 h undocked with
the camera on. **The camera + depth is the expensive part; budget for it.**

--------------------------------------------------------------------------

## 2. THE OAK-D WAS RGB-ONLY — found, and fixed at runtime

### 2.1 The symptom that would have hit

`/robot1/oakd/stereo/*` did not exist. Only these were published:

    /robot1/oakd/rgb/preview/image_raw            (+ /compressed, /theora,
    /robot1/oakd/rgb/preview/camera_info             /compressedDepth)
    /robot1/oakd/imu/data

The agent's fixes #38/#51 expect `/robot1/oakd/stereo/image_raw/compressedDepth`.

### 2.2 Why that is fatal, exactly

`vla_agent_v28.py::current_pair()` (~line 1668) returns, in order:

1. `self.pair` — written ONLY by the RGB+depth ApproximateTimeSynchronizer
2. else `(self.rgb_raw, self.depth_raw)` — **only if BOTH are non-None**
3. else `None`

With no depth topic, `self.depth_raw` is `None` forever, so `current_pair()`
returns `None`, `camera_ready()` is False and `yolo_detect()` returns `None`
before it ever calls the server. `/vla/status` would read `"camera": false`.

**The #54 LiDAR ranging cannot rescue this** — it runs downstream of that
gate. Depth is required to OPEN the gate even though depth is not what
measures the distance (§7).

### 2.3 Root cause

`/opt/ros/humble/share/turtlebot4_bringup/config/oakd_lite.yaml` on the Pi:

    camera:
      i_pipeline_type: RGB          <-- colour only, no stereo depth

File is dated 2024-07-02, owned by `ros-humble-turtlebot4-bringup`, and is
the **untouched upstream default** — it has never been edited on this robot.
So depth has never worked on this install, which is consistent with the
8 Sep handover listing "the OAK-D under the agent" as never verified.
(The agent's #51 comment describes a `stereo: {i_low_bandwidth: true}`
section. No such section exists in this file. That comment was written
against a configuration this robot does not have.)

### 2.4 THE FIX — runtime parameter, no restart. USE THIS.

The camera is started by `turtlebot4.service` -> `lite.launch.py` ->
`oakd.launch.py`, so there is no way to restart only the camera; editing
the yaml would mean restarting every robot node and losing the SLAM map.
**That is not necessary.** `camera.i_pipeline_type` is settable at runtime:

    python3 ~/vla_tools/oakd_rgbd.py     # inside ubuntu22-gpu, robot_env sourced

It sets `camera.i_pipeline_type=RGBD` via `/robot1/oakd/set_parameters`,
then calls `stop_camera` and `start_camera`. Result 10:00:50:

    set i_pipeline_type=RGBD -> successful=True reason=''
    stop_camera success=True / start_camera success=True

and immediately afterwards:

| topic | rate |
|---|---|
| `/robot1/oakd/rgb/preview/image_raw/compressed` | 15.8 Hz |
| `/robot1/oakd/stereo/image_raw/compressedDepth` | **4.52 Hz** |
| `/robot1/oakd/stereo/camera_info` | **29.5 Hz** |

Both better than the agent's comments assume (#51 expected ~2.4 Hz depth;
#53 worried the stereo intrinsics arrive too late to be useful — at 29.5 Hz
they arrive immediately, so the #53 fallback should never be needed).

After this, `/vla/status` read **`"camera": true, "map": true`**.

### 2.5 THE CATCH — it is not persistent

This is a live parameter. **A Pi reboot or a `turtlebot4.service` restart
puts it back to `RGB` and the agent's camera gate closes again.**
Re-run `oakd_rgbd.py` every session, after undocking, before starting the
agent. The camera must be ON for it to work (it is off while docked), so
the order is: undock -> confirm colour frames -> `oakd_rgbd.py` -> confirm
`stereo/...` topics -> start the agent.

A permanent fix would be editing the Pi's `oakd_lite.yaml` and restarting
`turtlebot4.service`; it was deliberately NOT done, because the runtime
switch is free and the yaml is a package file that an apt upgrade can
revert without warning.

--------------------------------------------------------------------------

## 3. THE REAL BUG — camera frames are ~4.3 s stale. NOT FIXED.

### 3.1 The measurement

`~/vla_tools/latency_probe.py` and `~/vla_tools/drift_probe.py` compare each
message's `header.stamp` with the PC's ROS clock on arrival. The PC is
**not** NTP-synced (`NTPSynchronized=no`), so absolute values include an
unknown clock offset — **but both streams come from the same Pi over the
same link, so the DIFFERENCE between them is real and offset-free.**

    LASER  /robot1/scan                      median  178 ms   (spread 174-180)
    RGB    .../rgb/preview/image_raw/compressed median 4300 ms  (spread 4254-4372)

Six 10-second buckets over 60 s, all six within 120 ms of each other:
**the offset is stable within a minute, not accumulating.**
(The "first 20 samples = 5966 ms" is a subscription-startup artefact.)

**So the camera's frames are timestamped ~4.1 s older than the laser's.**

### 3.2 Why this breaks `go to <object>`

Fix #47 deliberately looks up TF **at the colour frame's timestamp** —
correct reasoning, but it assumes the stamp is honest. The chain:

1. frame stamp is ~4.3 s in the past
2. `#47` reads the robot's pose at that stamp -> a 4.3 s-old pose
3. the bearing to the chair is rotated by the wrong yaw -> the object is
   projected to the **wrong map position**
4. that false position differs from the live goal by more than
   `GOAL_REISSUE` (0.9 m) -> the agent **cancels the Nav2 goal and re-sends**
5. per the agent's own #58 comment: *"Every trigger CANCELS the path Nav2
   is driving and starts a new one — the robot decelerates, replans,
   accelerates again. That stop-start IS the jerky motion"*

That is precisely the operator's report: *"when I tell it to go to the chair
it moves abruptly ... the target is missed due to the latency."*
And #47's own comment already names the symptom: *"the 'it backs up and
turns away from the chair that is right in front of it' symptom."*

**While stationary the bug is invisible** — a 4.3 s-old pose equals the
current pose — which is why detection and distance test perfectly (§7) and
only the drive misbehaves.

### 3.3 The step-and-stare dwell is too short for this lag

    SCAN_DWELL_S = 3.5     # s spent stationary looking, after every step

The author sized this against a laggy camera on purpose (#56: *"the pause
gives the laggy camera time to deliver a sharp frame"*). **3.5 s < 4.3 s**,
so every frame examined during the "stare" was captured before or during
the preceding turn. The step-and-stare design is defeated by a lag slightly
larger than its own pause. If the lag cannot be removed, `SCAN_DWELL_S`
must exceed it (6.0 s would give ~1.7 s of genuinely stationary frames).

### 3.4 Operator observation (first-hand, 10:45)

> "the frames arrive late and sometimes are not received by the agent"

Frames arriving late **and being dropped** points at genuine buffering plus
loss rather than a pure clock-conversion error, but this was not measured.

### 3.5 WHY NOTHING WAS CHANGED IN THE AGENT — read before "fixing" this

There are two candidate causes and **they need opposite fixes**:

| cause | correct fix | the other fix makes it WORSE |
|---|---|---|
| stamps are wrong (driver clock conversion), frames are actually fresh | stamp frames on ARRIVAL and use that for TF | slowing the robot buys nothing |
| frames are genuinely 4.3 s stale (buffering) | cut real latency: camera fps / queue size | re-stamping would LIE about freshness and worsen the projection |

Guessing would risk making the demo worse. **Do the discriminating test in
`HANDOVER_2026-09-10.md` §3 first.** It is free: the camera restarts on
undock, so measuring the offset immediately after undocking separates the
two (near-zero then growing = clock conversion; 4.3 s at once = buffering).

`vla_agent_v28.py` is SHARED with the simulation demo. The operator's
standing instruction: any change to it must sit behind a flag or
environment variable that defaults to today's behaviour so the sim path is
byte-identical in effect, and **the diff must be shown before it is applied**.

--------------------------------------------------------------------------

## 4. VERIFIED FIRST TIME ON HARDWARE — manual override

Closes the 8 Sep handover §7.6, which had never been tested on the robot.
Operator typed `manual override` in the agent's xterm and confirmed it
worked.

Reading of the implementation (`vla_agent_v28.py` ~2805-2875), worth having
in the report because it contradicts a known weakness:

* it refuses unless `sys.stdin.isatty()` — hence the xterm, hence the
  standing rule never to pipe or redirect the agent
* it is a **dead-man switch**: `select()` with a 0.05 s timeout, and if no
  key has arrived for `MANUAL_HOLD_S = 0.35 s` it publishes a zero `Twist`
  by itself. Releasing the key stops the robot; you never need to press `k`
* on exit (`q`, Ctrl+C, or ANY exception) the `finally:` block restores the
  terminal and publishes **five** consecutive zero `Twist` messages
* speeds: starts at 0.15 m/s / 0.50 rad/s, capped at
  `MANUAL_LIN_MAX = 0.26` (the Create 3's own limit) and 1.00 rad/s

**Therefore `manual override` -> `q` is a MORE reliable stop than the normal
stop command that handout §8.1 calls unreliable.** Five stop publishes from
a `finally:` block beats one best-effort publish. This is the operator's
fallback and it is now proven on hardware.

--------------------------------------------------------------------------

## 5. VOICE — it was simply never started

Symptom: switching the GUI to "Voice command" transcribed nothing.

Cause: the GUI does **not** transcribe. `vla_gui_v2.py` only publishes
push-to-talk on `/vla/voice/trigger`; the transcription is a separate
program, `~/voice_command.py`, which was not running. The GUI's own hint
says so: *"(The 'Voice input' step must be running.)"*

Fix — start it (inside `ubuntu22-gpu`, `robot_env.sh` sourced):

    python3 -u ~/voice_command.py --mode ptt --mic pulse

    [voice] loading faster-whisper 'small.en' on cuda (float16) ...
    [voice] faster-whisper ready in 4.1s
    [voice] microphone: [11] pulse  @ 16000 Hz
    [voice] No terminal attached — running HEADLESS.
            Waiting for hold-to-talk on /vla/voice/trigger.

Headless is CORRECT here: the GUI's gold button is the trigger, so the node
needs no terminal of its own (unlike the agent). `--mic pulse` follows
whatever the system input device is, including a Bluetooth headset — the
three inputs seen were `hw:1,0`, `hw:1,2` and `pulse`.
Topics then present: `/vla/voice/trigger`, `/vla/voice/state`,
`/vla/voice/transcript`.

--------------------------------------------------------------------------

## 6. OLLAMA — a 34-second stall removed

    first call after idle : 34.16 s   (cold: 4.7 GB model load)
    second call           :  0.149 s  (warm)

Default `keep_alive` is 5 minutes, so any pause longer than that during a
demo costs 34 s on the next command. Pinned with one API call, no file
edited:

    curl -s http://127.0.0.1:11434/api/chat -d '{"model":"qwen2.5:7b",
      "messages":[{"role":"user","content":"hi"}],"stream":false,"keep_alive":-1}'

`/api/ps` then shows the model expiring in the year 2318. **Do this before
any demo.** It does not survive an `ollama serve` restart.

--------------------------------------------------------------------------

## 7. NOT A BUG — the reported distance is correct

The operator suspected the "chair at 2 m" reading was wrong. It is right,
and it was confirmed with a sensor that has nothing to do with the camera.

`~/vla_tools/range_probe.py` converts each YOLO box centre to a bearing
(`bearing = -atan((u_c - c_x)/f_x)`, with `f_x=197.74, c_x=126.94` from
`/robot1/oakd/rgb/preview/camera_info`) and reads the RPLIDAR with the same
nearest-coherent-cluster logic the agent uses (#61):

    chair  conf 0.532  bearing +13.4 deg   LASER 2.34 m
    chair  conf 0.523  bearing -23.2 deg   LASER 2.30 m

Raw forward sweep agreed: -20 deg 2.04 m, -10 deg 1.80 m, 0 deg 1.93 m,
+10 deg 2.36 m, +20 deg 3.67 m. **The chair really was ~2.3 m away.**

### 7.1 Ranging precedence, verified in the source (asked for the report)

* `range_for_box()` docstring: *"Distance in metres to a detection: LiDAR
  first, depth as fallback."*
* `project_box()` #54 comment: *"LiDAR first - it is the sensor that
  measures this robot's world correctly. Only if it cannot see the target
  does the depth path run."*

So, precisely: **depth does not normally produce the distance — the RPLIDAR
does.** Depth's two remaining jobs are (a) the fallback when the laser
cannot see the target, and (b) **gating `camera_ready()`** via
`current_pair()`, which is what made §2 fatal. Both statements are needed;
"depth is unused" would be wrong.

--------------------------------------------------------------------------

## 8. Detection depends on standoff — a demo-planning fact

Same chairs, same camera, two positions:

| robot position | chair confidence | above the 0.30 floor? |
|---|---|---|
| 13 cm off the dock, under a desk | **0.174** | no — invisible to the agent |
| 2.3 m back, same heading | **0.532 / 0.523** | yes |

The OAK-D on the Lite sits low; up close it sees chair legs and desk
underside, cropped, and YOLO-World scores them poorly. **Give the robot
2-3 m of standoff before asking it to find furniture.** The agent already
has #57 ("stand off far enough to keep the object in view") for the same
reason. `person` scored 0.698/0.522 at the close position, so `person` is
the more robust demo class if space is tight.

Default vocabulary (startup only, `YOLO_CLASSES=` to change; `/set_classes`
at runtime is broken on this machine, #21):
`person, box, cardboard box, shelf, door, chair, pillar, docking station,
charging dock` — the last two exist to absorb dock detections away from
the others.

--------------------------------------------------------------------------

## 9. Nav2 — worked first try, but the drive was not clean

`ros2 launch ~/nav2_hw.launch.py` (bond_timeout 30 s + 20 s delayed
STARTUP) activated on the **first attempt**: `Managed nodes are active`,
**7 of 7 bonds connected, 0 errors**, `velocity_smoother` publishing
`/robot1/cmd_vel`, local costmap 1.667 Hz. On 8 Sep this took three
attempts. A cool Pi is the plausible difference; not isolated.

**But the one real drive wobbled.** Goal (-2.0, 0.0), 1.9 m, on a costmap
that was almost entirely unexplored:

    10:09:42  goal accepted, 1.84 m remaining
    10:09:53  0.22 m remaining      <- nearly there
    10:10:03  0.65 m remaining      <- went BACKWARDS
    10:10:06  0.91 m remaining
    10:10:27  status=4 SUCCEEDED

75 s for a 1.9 m drive that should take ~10 s, retreating from 0.22 m to
0.91 m in between — Nav2 recovery behaviours firing, most likely because
the goal lay in unknown space and/or the desk was 0.34 m off the left side.
**Build some map before sending goals into unexplored space.**

### 9.1 A slower motion profile is prepared but NOT tested

`~/nav2_hw_slow.yaml` — a copy of the stock
`/opt/.../turtlebot4_navigation/config/nav2.yaml` with 14 motion lines
changed and nothing else (`/opt` untouched, so the simulation is unaffected):

| setting | stock | new |
|---|---|---|
| `max_vel_x`, `max_speed_xy` | 0.26 | 0.15 m/s |
| `max_vel_theta` | 1.0 | 0.4 rad/s |
| `acc_lim_x` / `decel_lim_x` | 2.5 / -2.5 | 0.5 / -1.0 |
| `acc_lim_theta` / `decel_lim_theta` | 3.2 / -3.2 | 0.8 / -1.2 |
| behaviour server `max_rotational_vel` / `rotational_acc_lim` | 1.0 / 3.2 | 0.4 / 0.8 |
| smoother `max_velocity` / `max_accel` / `max_decel` | [0.26,0,1.0] / [2.5,0,3.2] / [-2.5,0,-3.2] | [0.15,0,0.4] / [0.5,0,0.8] / [-1.0,0,-1.2] |

Use it with:

    ros2 launch ~/nav2_hw.launch.py params_file:=/home/danyalaziz/nav2_hw_slow.yaml

**Honest limit: this cannot fix §3.** At the stock 1.0 rad/s a 4.3 s stale
stamp is 246 deg of bearing error; at 0.4 rad/s it is still ~99 deg.
Slowing down scales the error, it does not remove it. Its real value is
that gentler acceleration reduces motion blur and makes cancel/replan
cycles less violent. **Fix the timestamp first; keep this as polish.**

--------------------------------------------------------------------------

## 10. New traps found today

**10.1 `vla-box` has no `xterm`.** An `xterm -e` launcher silently opens no
window there. Run the detector with `nohup ... > logfile` instead — it has
no tty requirement (only the agent does).

**10.2 `vla_kill.sh` does NOT stop `yolo_server`.** It is in none of the
group-kill patterns nor the orphan sweep, and it runs in `vla-box`. Kill it
by PID: `pgrep -f "[y]olo_server"` in one command, `kill` in the next; it
needed `-9` today.

**10.3 `ros2 node list` returned a PARTIAL graph and looked alarming.**
While docked it omitted `turtlebot4_node`, `create3_repub`, `oakd*` and
others — yet `dock_status` and `battery_state` were publishing normally
from those very nodes, in the same minute. A short node list is not
evidence that a node is dead. Confirm with `topic hz` / `topic echo`.

**10.4 The Dock action ABORTED.** `~/redock.py`'s Nav2 leg reached the
staging pose in 12 s (`status=4`), then the Dock action ran 79 s and
returned `status=6 (ABORTED) is_docked=False`, leaving the robot off the
charger. The operator docked it by hand. Cause not investigated. **Watch
`redock.py` to completion — do not assume it docked; check `is_docked` and
`current > 0`.**

**10.5 Placing the robot on the dock by hand invalidates SLAM's pose.**
Odometry never saw the motion, so `map -> odom` is now wrong and any Nav2
goal would drive to the wrong place. After a manual move: restart SLAM, or
set a 2D Pose Estimate in RViz, before navigating.

**10.6 The PC is not NTP-synced** (`NTPSynchronized=no`). Absolute
PC-vs-Pi timestamp comparisons are therefore untrustworthy on their own.
Use `/robot1/scan` (178 ms) as the reference baseline and compare other
Pi-sourced streams against it — a common clock offset cancels out.

--------------------------------------------------------------------------

## 11. What was NOT done

* **The end-to-end `go to <object>` drive** — blocked by §3. `what do you
  see` worked; the chain GUI -> agent -> YOLO -> LLM -> reply was exercised
  and the operator confirmed the GUI status bar read all-yes.
* The discriminating latency test (§3.5) — needs the camera on, i.e. the
  robot undocked; battery was down to 29 %.
* `~/nav2_hw_slow.yaml` has never been loaded by a running Nav2.
* No map was saved. `~/maps/` is still empty.

## 12. Files created or changed today, all under `/home/danyalaziz`

Created:
* `nav2_hw_slow.yaml` — slower motion profile (§9.1), untested
* `vla_tools/` — 16 diagnostic and launcher scripts, all verified with
  `python3 -m py_compile` / `bash -n` (§13)
* `CHANGELOG_2026-09-10.md`, `HANDOVER_2026-09-10.md` (this and its companion)
* `.vla_gui.json.bak-20260910-preswitch` — the sim GUI config as it was
* `vla_logs/nav2_hw_20260910.log`, `vla_logs/yolo_20260910.log`,
  `vla_logs/voice_20260910.log`, `vla_logs/gui_20260910.log`

Changed:
* `.vla_gui.json` — now the ROBOT config (was the sim's). **Safe**:
  `vla_sim.sh` line 224 copies `.vla_gui.sim.json` over it on every sim run,
  so the simulation restores itself automatically.
* `CLAUDE.md`, `PROJECT_HANDOUT_v5.md` — updated with today.

On the ROBOT: `camera.i_pipeline_type` set to `RGBD` at runtime (§2.4).
**Not persistent** — reverts on reboot or service restart. No file on the
Pi was edited.

NOT touched, deliberately: `vla_agent_v28.py`, `vla_sim.sh`, `sim_mode.sh`,
`run_sim_stack.sh`, `run_agent_sim.sh`, `.vla_gui.sim.json`, `slam_vla.yaml`
(md5 `fc4265602444a83d8cd7a3a72f14a147`, unchanged), anything under `/opt`
on either machine, `yolo_server.py`, `llm_brain.py`, `voice_command.py`.

## 13. `~/vla_tools/` — what each script is for

| script | purpose |
|---|---|
| `oakd_rgbd.py` | **switch the camera to RGBD at runtime (§2.4) — needed every session** |
| `oakd_params.py` | list/inspect the OAK-D's 125 parameters via a long-lived client |
| `undock_hw.py` | undock from a long-lived action client (a fresh CLI client hangs) |
| `nav_goal.py` | `nav_goal.py X Y YAW_DEG` — one Nav2 goal, with feedback |
| `range_probe.py` | YOLO boxes -> bearing -> laser range; ground-truth distances (§7) |
| `latency_probe.py` | per-stream `header.stamp` vs arrival latency (§3.1) |
| `drift_probe.py` | is that offset constant or growing? 60 s in 10 s buckets |
| `scan_look.py` | laser clearance by bearing — check a direction is free before moving |
| `is_moving.py` | read-only: is the robot actually stationary? is a goal active? |
| `yolo_live_test.py` | one live frame -> YOLO -> detections, end to end |
| `yolo_floor_probe.py` | same frame at default vs very low confidence floors (§8) |
| `slam_xterm.sh`, `rviz_hw_xterm.sh`, `nav2_hw_xterm.sh`, `agent_xterm.sh` | window launchers (source `robot_env.sh` first) |
| `shutdown_stack.sh` | `robot_mode.sh` + `vla_kill.sh`, safe to call by path |

Every Python tool uses the long-lived-client pattern from `~/redock.py`:
create the client, spin ~30 s, then call — because a fresh client's replies
are dropped for up to ~20 s on this system (8 Sep §9.4).
