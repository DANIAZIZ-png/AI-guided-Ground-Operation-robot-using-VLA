# PROJECT HANDOUT v5 — SIMULATION AS BACKUP DEMO, AND TWO AGENT FIXES

**AVN CDT Danyal Aziz** · CMS 432606 · 99 EC Bravo
College of Aeronautical Engineering, NUST · PAF Academy Risalpur

**Covers:** sessions of 11–13 Aug 2026 (v26 → v28, plus the launcher scripts),
extended with the hardware sessions of **19–21 Aug 2026 in §14** and the
**8 Sep 2026** hardware session in **§15** (RViz layout and Nav2 on hardware).
The 13 Aug session is recorded in **§13**, whose priority list supersedes §11.
§14 adds hardware findings and retracts the battery theory in §8.5.
**Supersedes:** `PROJECT_HANDOUT_v4.md`. Everything in v4 still stands except
where corrected below — in particular **§3.4 of v4 was wrong** and is retracted
in §7 here.

---

## 1. STATUS AT A GLANCE

| Capability | Hardware | Simulation |
|---|---|---|
| Camera RGB stream | working | **working** |
| Camera depth stream | working, not trusted for ranging | working |
| SLAM Toolbox | working | **working** |
| Nav2 stack | working — re-verified 8 Sep with `nav2_hw.launch.py`, §15 | **working** |
| RViz, pre-configured layout | **working — `vla_hardware.rviz`, §15** | **working** |
| YOLO-World detection | working | **working** |
| LLM reasoning (Qwen2.5-7B) | working | **working** |
| `go to <object>` end-to-end | **working** | **working** |
| Relative motion (turn N°, move N m) | **working** — see #62 | working |
| Dock / undock | working | working (see #63) |
| Operator console GUI | untested this phase | **working** |
| Voice input (Whisper, push-to-talk) | untested this phase | **working** |
| Live annotated feed in the GUI | untested this phase | **working** |
| Object ranging via LiDAR (#61) | untested | **broken — bearing ~90° off** |
| Collision guard | fixed in #62, partly tested | fires; stop is unreliable |
| Manual override / teleop | working **from a terminal only — §8.7** | working **from a terminal only — §8.7** |
| One-command cold start | script written, untested | **working** |

**The headline:** the simulation now runs the complete demo from a single
command — `~/vla_sim.sh` — including the GUI, the live feed and voice. This is
the guaranteed fallback if hardware fails in front of the faculty.

---

## 2. THE TWO AGENT FIXES

Current file: **`vla_agent_v28.py`**. `llm_brain.py` unchanged.

| # | Version | Problem it solved |
|---|---|---|
| 62 | v27 | Direction-aware collision guard — the robot could not turn away from an obstacle |
| 63 | v28 | Headless-safe console output — a closed pipe killed the command being executed |

### 2.1 #62 — the guard trapped the robot it was protecting

**Observed on hardware.** The robot came to rest ~0.68 m from a wall, inside
`MIN_FRONT_CLEAR` (0.70 m). From then on every command died instantly:

```
[move] Turning ninety degrees to the right.
[robot] Stopping — an obstacle is 0.69 m ahead.
```

It could not turn away and it could not reverse away. The guard had trapped the
robot in exactly the situation it existed to prevent.

**Root cause.** v26 did exempt rotation, but with the wrong test:

```python
if self.search_state == "ROTATE":
    return
```

`search_state` is set **only** by the search state-machine inside
`navigate`/`find_more`. A `turn right 90` is a *relative move* — `mode ==
"move"`, `search_state` is `None` — so the exemption never applied.

The exemption was keyed on **which state machine was running** instead of on
**which way the robot was being commanded to move**.

**Fix.** A front-facing guard is only meaningful against forward motion.
Rotation in place sweeps no new ground ahead; reversing moves *away* from the
thing being measured. `driving_forward()` now decides:

| Situation | Guard |
|---|---|
| `turn right 90` in place | **off** ← the bug |
| reverse | **off** — a front sensor cannot see behind |
| move forward | on |
| search spin | off (already was) |
| **Nav2 driving** | **on, unconditional** |

**The subtlety that made this non-trivial.** When Nav2 drives, the agent does
not own `self.cmd` — Nav2 publishes to `/cmd_vel` directly and `self.cmd` sits
at a stale zero. A naive `self.cmd.linear.x > 0` test would have silently
**disabled the guard during real navigation**, the case it matters most for.
`agent_owns_cmd()` mirrors `publish_cmd()` exactly, and anything unknown is
treated as forward: an unreadable velocity must be assumed dangerous, never safe.

**#59's principle is untouched.** The guard still consults no belief about where
objects are. Direction of travel is not a belief — it is the command the agent
itself issued one tick ago.

Verified by `test_62_guard_direction.py`, 10/10 cases including the Nav2 rows.
**Confirmed on hardware:** the robot turns again.

### 2.2 #63 — a closed pipe killed the command

**Observed through the GUI.**

```
ROBOT [undock] Undocking.
ROBOT Something went wrong handling that command: [Errno 32] Broken pipe
```

Undock announced itself and then died. Same for other commands.

**Root cause.** `notify()` printed on every robot message with no guard:

```python
def notify(self, msg):
    self.mlog.log("ROBOT", msg)
    print(f"\n[robot] {msg}\nCommand> ", end="", flush=True)
```

When the GUI launches the agent there is no terminal; the GUI reads stdout into
its log pane. The moment that pipe closes — GUI restarted, step stopped, buffer
full — every `print()` raises `BrokenPipeError`. The exception was raised from
*inside* the command handler and propagated up, killing the command.

**#33 had already handled `input()`** — the agent does not read a keyboard when
there is no tty — but it left all **55** `print()` calls unguarded.

**Fix.** Console output is cosmetic and must never abort a robot action. Rather
than wrap 55 call sites and every future one, `print` itself is replaced:

```python
_real_print = print
def print(*args, **kwargs):
    try:
        _real_print(*args, **kwargs)
    except (BrokenPipeError, ValueError, OSError):
        pass
```

The real output paths are `/vla/reply` and `/vla/status` — DDS topics,
unaffected by a closed pipe. Verified by `test_63_broken_pipe.py`, 4/4.

---

## 3. THE SIMULATION IS NOW THE BACKUP DEMO

### 3.1 One command

```bash
~/vla_sim.sh
```

Brings up, in order, waiting for each before starting the next: Gazebo + SLAM +
Nav2 → the VLA agent → voice → the operator console. Logs land in
`/tmp/vla_sim_logs/`. Stop everything with `~/vla_kill.sh`, run inside the
container: from the host it kills the processes but cannot restart the ROS
daemon or clean shared memory, and says so (8 Sep, `CHANGELOG_2026-09-08.md` §1).

**Do not press START ALL in the GUI** — everything is already running, and
pressing it kills the stack the script just built.

### 3.2 The rule the script encodes: wait for DATA, never for a topic name

The GUI's own launcher reported *"ready (0s)"* for Gazebo three times in a row
while nothing worked. Its probe checked whether a **topic existed**. A topic can
exist while nothing publishes on it — a leftover node from a previous run is
enough to satisfy the check.

Every wait in `vla_sim.sh` uses `ros2 topic echo --once`, which blocks until a
real message **arrives**:

| Stage | Waits for | Timeout |
|---|---|---|
| Gazebo | `/clock` ticking | 180 s |
| Gazebo | `/scan` ticking | 60 s |
| Gazebo | camera ticking | 60 s |
| Agent | `/vla/status` ticking | 90 s |
| Voice | a **publisher** on `/vla/voice/state` — see below | 60 s (optional) |

On failure it prints the last 30 lines of the relevant log rather than leaving
you to find it.

`/clock` is the single most useful check in the whole system: it is the
simulator's heartbeat. If it ticks, physics is running and every sensor
downstream will work. If it does not, nothing else can, and no amount of ROS
debugging will help.

### 3.2a THE BOUNDARY CONDITION ON THAT RULE (13 Aug — PARTLY UNRESOLVED)

The rule above is right, but it is **not universal**, and the exception cost a
session's worth of confusion.

"Wait for data, never for a topic name" holds for **continuous** topics —
`/clock`, `/scan`, the camera and `/vla/status` all publish on a timer, so a
message is always on its way and `echo --once` returns as soon as one lands.

It is **wrong for event-driven topics**, and `/vla/voice/state` is a candidate
for being one.

**What was observed, 13 Aug**, with the voice node running:

```
Publisher count:    1      <- a publisher exists
Subscription count: 0      <- the gate was not connected at the moment sampled
```

The gate ran its full 240 s and reported the stage not-ready. *If* the topic
publishes only on a state change — announcing "ready" once, possibly before the
gate began listening, then staying silent until the mic button is pressed —
then there is no next message for `echo --once` to return on, and the timeout
is measuring the gate's own assumption rather than the node's health.

**UNRESOLVED — do not cite this as settled.** The competing explanation is the
original one: Whisper genuinely was still loading. The process was measured at
**5.2 % CPU**, first read as "loaded and idle" — but **that reading does not
discriminate.** With `--device cpu --compute int8` the model load is largely
disk I/O plus quantisation, so a *loading* process can sit at single-digit CPU
and look identical to an idle one. Sustained high CPU would have meant
something; low CPU means nothing either way.

Both can also be true simultaneously: the topic can be event-driven **and** the
model slow to load. Which of them produced the 240 s timeout is not yet
established.

**The test that settles it:** watch two timestamps against one clock — when a
publisher first appears on `/vla/voice/state`, and when the voice node actually
reports ready. Far apart ⇒ the publisher gate is premature and 5.2 % was
loading. Close together ⇒ the event-driven reading holds.

**The corrected rule: ask what the topic's publishing model is.**

| Topic type | Correct gate | Why |
|---|---|---|
| continuous (`/clock`, `/scan`, `/vla/status`) | wait for a **message** | one is always coming; mere existence proves nothing |
| event-driven (`/vla/voice/state`) | wait for a **publisher** | there may be no next message at all |

`vla_sim.sh` now has two helpers — `wait_for_topic` and `wait_for_publisher` —
and uses the second **only** for the voice gate. Every other gate is continuous
and correctly keeps the first. **Do not merge the two helpers back into one.**

**The honest limit of the new gate.** A publisher appears when the node creates
it, which may be before Whisper has finished loading. It proves the node
started, not that it is ready — genuinely weaker than the continuous-topic
check, not equivalent to it. It is the strongest signal available for a topic
with no next message, and voice is optional, so the trade is acceptable. It
should not be copied to a gate that something else depends on.

**The general lesson**, and it generalises past ROS: a health check encodes an
assumption about how the thing it watches behaves. §3.2's original failure was
checking existence when it should have checked liveness. This one is the mirror
image — checking liveness where liveness is not continuously observable. Both
report a healthy system as broken, or a broken one as healthy, without ever
being wrong about the fact they actually measured.

### 3.3 Simulation vs hardware — the settings are mutually exclusive

| | Hardware | Simulation |
|---|---|---|
| `ROS_DISCOVERY_SERVER` | `10.42.0.169:11811;` | **unset** |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | the super-client XML | **unset** |
| `ROS_DOMAIN_ID` | default | **default — see §5.1** |
| `VLA_NS` | `/robot1` | `""` — empty, **not unset** |
| `VLA_RAW_RGB` / `VLA_RAW_DEPTH` | unset (compressed on) | `1` (raw) |
| Camera topics | `/robot1/oakd/...` | remapped, see §3.4 |

Neither mode fails loudly in the other's settings. Both fail **silently**.

### 3.4 One agent file, two worlds

`vla_agent_v28.py` is used unmodified for both. ROS remaps the topic names at
launch:

```bash
python3 ~/vla_agent_v28.py --ros-args \
  -r /oakd/stereo/image_raw:=/oakd/rgb/preview/depth \
  -r /oakd/stereo/camera_info:=/oakd/rgb/preview/camera_info
```

Two copies of the agent would drift apart, and that is exactly how a backup demo
rots. **Without the depth remap** the agent's `ApproximateTimeSynchronizer`
never forms an (RGB, depth) pair, the callback that runs YOLO never fires, and
it reports *"I see: nothing"* while RGB streams at 29 Hz — with no error.

---

## 4. THE BIG ONE: EIGHT `robot_state_publisher` PROCESSES

This was behind most of what looked like "Gazebo is broken" across two sessions.

**Symptoms, none of which name the cause:**

```
[gz_ros2_control] robot_state_publisher service not available, waiting again...
[controller_server] Tf has two or more unconnected trees.
[spawner] Could not contact service /controller_manager/list_controllers
```

The world loads, the robot spawns, and it cannot move. Or the robot never
appears at all.

**Root cause.** Every relaunch left the previous stack's nodes alive. After a
few cycles there were **eight** `robot_state_publisher` processes, all
advertising the same service and all publishing TF. `gz_ros2_control` could not
resolve which to talk to; TF split into unconnected trees.

The kill lists we were using named ~10 processes. `ros2 launch
turtlebot4_ignition` starts about **50**. A hand-written list can never be
complete.

**Fix — `vla_kill.sh`.** Kills by **process group**: every child of a
`ros2 launch` shares its parent's PGID, so one signal takes the whole tree,
including nodes nobody thought to list. It then **verifies** and exits non-zero
if anything survived. Both launchers refuse to start on a dirty system.

```bash
kill -TERM -"$PGID"    # the MINUS makes it a process-GROUP kill
```

**The general lesson:** *prove the kill worked; do not assume it.* That
verification step is what was missing.

### 4.1 Related: closing a terminal does not stop a launch

Closing the window orphans every process in it. They keep running, keep
publishing, and are invisible in that terminal's history. Always Ctrl+C
**inside** the terminal, or use `~/vla_kill.sh`.

Three agents were found running simultaneously in one session — all publishing
`/vla/status` and all responding to the same `/vla/command`.

---

## 5. THINGS THAT LOOKED LIKE THE FAULT AND WERE NOT

Recorded because each cost real time, and because a wrong hypothesis that
survives into a viva is worse than an open question.

### 5.1 `ROS_DOMAIN_ID` for isolation — tried and REJECTED

Setting `ROS_DOMAIN_ID=42` for the simulation is the textbook way to isolate two
ROS 2 systems. **It broke the simulation.**

`gz_ros2_control` runs *inside* the Gazebo process and does not inherit the
domain. `controller_manager` came up on domain 0 while the spawner looked for it
on 42, producing "Could not contact service /controller_manager/
list_controllers" forever. The robot spawned and could not move.

Isolation is now achieved by: clearing the discovery-server variables, a mode
lock (`/tmp/vla_mode.lock`) that stops both modes running at once, and separate
GUI configs. **Do not reintroduce the domain ID** without solving the
`gz_ros2_control` inheritance problem first.

### 5.2 "SLAM queue is full" has TWO distinct causes

v4 §5.1 documented this as the silent-Create-3 signature. It is also produced by
**clock drift**, and the two need different fixes.

On 11 Aug the Pi was found **118 seconds slow**. Every scan arrived stamped ~2
minutes in the past, so SLAM dropped all of them:

```
now:        1786442934
scan stamp: 1786442815     ← 119 s in the past
```

`chronyc tracking` reported `Last offset: 0.000239 s` — chrony believed it was
locked, because it was correcting the 118 s by **slewing** (nudging the clock
rate so time never jumps backwards). At the default slew rate that takes hours.
It had also fallen off the workstation as its time source and picked up an
internet NTP server instead.

**The tell that separates the two causes:** compare the scan's timestamp against
the log line's own time.

| Gap | Cause | Fix |
|---|---|---|
| < 1 s | TF / silent Create 3 | power-cycle the base |
| **~2 minutes** | **clock drift** | `sudo chronyc makestep` |

`makestep` forces an immediate jump instead of slewing. Add a clock check to
every hardware startup:

```bash
ssh ubuntu@10.42.0.169 'chronyc tracking | grep "System time"'
```

### 5.3 USB 3.0 radio interference — hypothesis, then contradicted

Moving the OAK-D from a black (USB 2.0) to a blue (USB 3.0) port on the Pi
suggested USB 3.0's well-documented 2.4 GHz RF noise as the latency cause. It
matched the measurements — strong signal, zero loss, high retries.

**It was contradicted by evidence.** Latency *improved* to 1.4 ms after the move
to USB 3.0. The direction is wrong for that mechanism. Recorded so the theory is
not resurrected.

---

## 6. THE GUI'S LAUNCHER DOES NOT WORK — USE IT AS A VIEWER

`vla_gui_v2.py` is excellent as an operator console: live annotated feed,
conversation panel, status bar, voice button, command box. All of that works.

**Its START ALL does not.** Three independent reasons:

1. **Probes check topic existence, not data** — leftovers make every step go
   green in 0 seconds while nothing works (§3.2).
2. **`distrobox-host-exec` returns exit 127** from inside `ubuntu22-gpu`, so it
   can reach neither the host (Ollama) nor `vla-box` (YOLO). Those two must be
   started by hand.
3. **It cannot re-latch subscriptions.** It binds `/vla/status` and the feed
   once at startup. Restart the agent underneath it and the GUI shows the last
   packet it ever received — `map no`, `camera no` — indefinitely, while the
   agent is perfectly healthy.

**Working order:** Gazebo → agent → **GUI last**. That is what `vla_sim.sh`
does. If the GUI ever shows stale status, close and reopen it; leave the stack
running.

**Diagnostic worth knowing.** The GUI's status bar can lie; the agent's own
report cannot:

```bash
timeout 25 ros2 topic echo /vla/status --once --full-length
```

In one session this showed `camera: True, map: True, feed: True` while the GUI
displayed `camera no, map no, docked None`. That single check distinguishes "the
robot is broken" from "the display is stale" — two very different problems.

---

## 7. RETRACTION: §3.4 OF v4 WAS WRONG

v4 listed link latency of 200–280 ms as an unexplained hardware fault, with
*"tx retries in the 10,000–14,000 range"* as the smoking gun.

**`tx retries` is a cumulative counter since association, not a rate.** A large
number only means the link has been up a while. Measured properly on 11 Aug:

```
tx packets: 203512
tx retries:   4643        →  2.3%
```

Under ~5% is a healthy 2.4 GHz link. The measurement that drove the entire
latency investigation was a misread counter.

**Measured the same day:**

```
rtt min/avg/max/mdev = 0.934/1.437/3.510/0.666 ms
20 packets transmitted, 20 received, 0% packet loss
signal: -43 dBm, tx bitrate: 72.2 Mbit/s
```

**1.4 ms average — 150× better than the 200–280 ms recorded in v4**, on the same
hardware, same channel, same room.

**The probable real cause of the historical latency: duplicated ROS nodes.**
Two camera drivers publish two copies of every frame; RGB alone is ~2.2 MB/s.
Doubled, plus depth, plus duplicate TF, that saturates a 72 Mbit/s link and
produces exactly the observed signature — high latency, high jitter, zero loss,
clean RF metrics. Every RF-side measurement in v4 §6 came back clean **because
the problem was never RF.**

This reclassifies §3.4 from "unexplained hardware fault" to "measurement
artefact plus an already-documented software fault." It also predicts the
latency will not return now that `vla_kill.sh` prevents duplicate stacks — a
falsifiable claim, worth testing.

---

## 8. OPEN ISSUES, IN PRIORITY ORDER

### 8.1 The stop command does not always stop the base — SAFETY

```
[robot] Stopping — an obstacle is 0.69 m ahead.
[robot] I sent the stop but the robot is STILL MOVING (0.26 m/s, 0.07 rad/s).
        Take manual override now.
```

The guard fired, the stop was issued, and the base kept moving. The agent
detected and reported it honestly — #35's barrage did not win against whatever
else was publishing to `/cmd_vel`.

**Fix this before any live hardware demonstration.** Nothing else on this list
matters as much.

**UNTESTED HYPOTHESIS — it may be the dock action, not a race on `/cmd_vel`.**
In the 13 Aug hardware log, all three "STILL MOVING" events occurred **during
docking**, and all three read `0.00 m/s` with `~0.6 rad/s` — pure rotation, and
the magnitude barely moved between them (0.61, 0.63, 0.65 rad/s). That
consistency argues against a random publisher race, which would not repeat to
two decimal places.

During docking the Create 3's own dock **action** owns the base. #35's barrage
publishes to `/cmd_vel`, but an action server driving the base keeps
re-asserting its own velocity — so the agent may not be losing to a stray
publisher at all. It may be losing to an action it never cancelled.

If that is right, the fix is to **cancel the dock/undock action goal** on stop,
and #35's barrage is simply the wrong mechanism for this case, however loud it
gets.

**This is a hypothesis drawn from one log. It has NOT been tested.** What would
test it: issue a stop mid-dock while watching who publishes to `/cmd_vel`, then
separately try cancelling the action goal and see whether the base stops.

**Caveat that keeps it honest:** the original failure quoted above was
`0.26 m/s, 0.07 rad/s` during forward motion with the obstacle guard firing —
**not** a docking case. So the dock-action theory cannot explain every
occurrence. Docking may be an aggravator rather than the whole cause, or there
may be two distinct faults wearing one error message. Do not let this
hypothesis close the investigation early.

### 8.2 LiDAR bearing is ~90° wrong in simulation

```
[lidar] not used — only 0 valid beams within 1.4 deg of bearing -102.8 deg
```

A chair the camera can plainly see is placed 103° to the robot's right. The
camera→LiDAR bearing transform is wrong in the simulated frames, so #61
nearest-cluster ranging finds no beams and falls back to depth (14.5 m for a
chair a few metres away).

Navigation still works — the goal is set in the map frame — but the ranging
number is meaningless in sim. **#61 therefore remains unverified in both
worlds.**

### 8.3 #61 nearest-cluster ranging still unverified on hardware

Unchanged from v4 §3.1. Chair directly ahead, tape-measured, nothing else within
2 m, then `what do you see`. Repeat at ~1.5 m and ~3 m.

### 8.4 RViz segfaults inside `ubuntu22-gpu` — RESOLVED

```
[ERROR] [rviz2]: process has died [exit code -11]
libEGL warning: egl: failed to create dri2 screen
```

Exit −11 is SIGSEGV: the container's EGL/DRI path, not ROS. **RViz now runs.**
Two changes, and both are needed:

* `LIBGL_ALWAYS_SOFTWARE=1` forces Mesa's CPU renderer (llvmpipe) and bypasses
  the failing GPU path entirely.
* `setsid` gives RViz its **own process group** — the half that actually
  mattered. Launched with `rviz:=true` it shared Gazebo's group, so a segfault
  took SLAM and Nav2 down with it. Decoupled, a crash costs only the display.

Launches still pass `rviz:=false`; `vla_sim.sh` starts RViz separately (§7b).
It renders on the CPU, so it is slow to draw — expected, not a fault.
**Not yet applied to `vla_robot.sh`**, which has no RViz step. **8 Sep:** on
hardware RViz ran for two hours started from an `xterm -e` script inside the
container **without** `LIBGL_ALWAYS_SOFTWARE=1`; the variable is harmless
either way. The hardware command is in §15.1.

### 8.5 The Create 3 goes silent — SEE §14.3, BATTERY THEORY RETRACTED

Unchanged from v4 §3.2, now **eight** occurrences. The battery suspicion below
is **retracted**: on 19–21 Aug it happened at 63 % (§14.3), and §14.3 identifies
it as turtlebot4 issue #554, recoverable only by a physical power-cycle.

~~Battery is a strong suspect: it fell 97% → 86% → 55% within one session, and
the silences cluster at the low end.~~

### 8.6 Smaller items

* **PyQt5 was not installed** in `ubuntu22-gpu` — `sudo apt install -y
  python3-pyqt5`. Add to the container prerequisites.
* **`yolo_server.py` `/set_classes` is broken at runtime.** `model.set_classes()`
  raises "Expected all tensors to be on the same device" because the CLIP text
  tokens are built on the CPU while the encoder has moved to cuda:0. The
  *startup* call works (nothing has touched the GPU yet). `yolo_server_v2.py`
  fails safely and returns 503 with the workaround. Change the vocabulary by
  restarting with `YOLO_CLASSES="..."`.
* **Voice logs are empty while running** — Python buffers stdout to a file. Add
  `python3 -u` in `vla_sim.sh` so a hung step can be diagnosed.
* ~~**`~/.bashrc` guard not yet applied.** Every new terminal still points at
  the Pi's discovery server.~~ **CORRECTED — the guard IS applied, see §14.13.**
  A fresh terminal is simulation-safe by default. This was the single largest
  time-sink of the phase, and it is closed.

### 8.7 Manual override is terminal-only — and was unreachable in a demo

`#60` manual override is the documented fallback for §8.1, and until this
session **neither launch script could reach it.** Both started the agent with
`setsid ... > log 2>&1 &`, so it had no tty and ran headless. The GUI printed:

```
(ignoring manual override from remote — it has to be taken from the
 agent's own terminal.)
```

**The refusal is correct, not a bug.** Inside `manual_override()`, stdin is
simultaneously the throttle, the dead-man switch (`select()` timeout → publish
stop) and the only exit ('q'). Entering the mode also sets `manual_mode = True`,
which switches the collision guard **off**. Granting override to a source with
no keyboard would give: guard disabled, no dead-man switch, and no way out —
strictly worse than having no override at all.

**Never "fix" this by deleting the `source != "keyboard"` test.** It would not
even work — headless, nothing consumes `manual_requested` — and if it did, it
would build exactly the hazard above.

**Fix.** Both scripts now start the agent in its own `xterm -hold`, so it owns
a real tty and override works as designed. The GUI opens afterwards as before.
Over a display-less SSH session both fall back to headless and say so loudly;
override is genuinely unavailable there.

**The trap.** Do NOT pipe the agent's output to keep a log file: `| tee` makes
stdout a pipe, `isatty()` goes false, and the agent is silently headless again
with override dead. Nothing is lost — it writes `~/vla_logs/` regardless.

This restores the fallback. It does **not** fix §8.1 itself.

### 8.8 `start_camera` returns success while docked — and means nothing

**Already known:** the OAK-D shuts down when the robot is on the dock. That part
is not new and is not the finding.

**The new finding:** `ros2 service call /robot1/oakd/start_camera` returns
**success while docked anyway**. The call is accepted, reports OK, and the
camera is dead by the time the operator console opens. It streamed only after
undocking.

That distinction is the useful part. A known shutdown can be planned around; a
success return that does not mean the camera is running is a **false positive
in a health check** — the same failure mode as §3.2, where a topic that existed
but was silent made every launch step go green in 0 s. Here the service answers
"yes" on behalf of a camera that is off.

`vla_robot.sh` calls `start_camera` during bring-up, i.e. while the robot is
still docked, so the call sits in the wrong place and its OK tells you nothing.
The camera start must happen **after** undocking, which the script does not do.
Until that is restructured, call it by hand once the robot is off the dock.

In priority this sits just below §8.1; it is numbered last only to avoid
renumbering the section.

---

## 9. THE DETECTION VOCABULARY PROBLEM

Two real misdetections were observed on hardware: a **printed figure on a wall
poster** reported as a person at 10% confidence, and a **computer tower**
reported as a box.

These are different problems.

**The poster.** `yolo_server.py` deliberately left `person`, `box` and `pillar`
on the 5% global floor:

```python
GLOBAL_CONF = 0.05
PER_CLASS_CONF = { "chair": 0.30, "shelf": 0.25, ... }
# "person" / "box" / "pillar" -> use GLOBAL_CONF (we want them easily)
```

That comment was written for Gazebo, where flat-shaded primitives score low. On
real hardware it is the entire source of the junk detections. A real person at
working range scores 0.6–0.9; the poster scored 0.10. A floor of 0.35 sits in
the empty space between them. `yolo_server_v2.py` gives **every** class an
explicit floor.

**The computer tower is not a misdetection.** YOLO-World is open-vocabulary: it
can only answer with words it has been given, and it always picks a winner. Seen
a dark rectangular object with only `box` available, `box` is the closest
correct answer. The model is not wrong; the dictionary is incomplete.

**A failed experiment worth recording.** Adding `"computer monitor"` and
`"computer tower"` to the vocabulary appeared to make things dramatically worse
— a counter and a doorway both became towers. The real cause was a **label/index
desync**: the old `/set_classes` assigned the Python list *before* calling
`model.set_classes()`, so when the model call raised, the server named
detections by index against a vocabulary the model was not using. Every label
was shifted. `yolo_server_v2.py` fix #20 updates the model **first** and commits
the list only on success.

The lesson holds regardless: in an open vocabulary, adding near-synonyms adds
**competition**, not precision. A vague-but-correct label beats a confident
wrong one.

---

## 10. CURRENT FILES

| File | Purpose |
|---|---|
| `vla_agent_v28.py` | the agent — sim and hardware, unmodified |
| `vla_gui_v2.py` | operator console (viewer; do not use START ALL) |
| `yolo_server.py` | detection server, carries fixes #20–#24 in its header (there is NO file called `yolo_server_v2.py`; §8.6/§9 use that name for the same file — corrected 8 Sep) |
| `llm_brain.py` | unchanged |
| `voice_command.py` | Whisper push-to-talk |
| `vla_sim.sh` | **one-command simulation demo** |
| `vla_robot.sh` | one-command hardware bring-up (first successful run 13 Aug; agent-in-xterm path not yet executed) |
| `vla_kill.sh` | process-group kill + verification |
| `vla_hardware.rviz` | RViz layout for the real robot: `/robot1` topics, correct QoS, hardware only (8 Sep, §15) |
| `nav2_hw.launch.py` | Nav2 bring-up for the real robot: stock nodes and params, plus `bond_timeout` 30 s and a delayed STARTUP (8 Sep, §15) |
| `~/.vla_gui.sim.json` | GUI config, simulation |
| `~/.vla_gui.robot.json` | GUI config, hardware |
| `test_62_guard_direction.py` | 10/10 |
| `test_63_broken_pipe.py` | 4/4 |

### Cold start, simulation

```bash
# 1. YOLO, in a vla-box terminal
distrobox enter vla-box
source ~/yolo-env/bin/activate && python ~/yolo_server.py

# 2. Ollama, on the host (if not already running)
ollama serve

# 3. everything else, in ubuntu22-gpu
~/vla_sim.sh
```

### Manual sequence, if the script fails

Each in its **own** terminal, each starting with the three unsets:

```bash
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID

# terminal 1 — Gazebo
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
    model:=lite slam:=true nav2:=true rviz:=false
# WAIT: ros2 topic hz /clock   must show ~300 Hz before continuing

# terminal 2 — agent
export VLA_NS="" VLA_RAW_RGB=1 VLA_RAW_DEPTH=1
python3 ~/vla_agent_v28.py --ros-args \
  -r /oakd/stereo/image_raw:=/oakd/rgb/preview/depth \
  -r /oakd/stereo/camera_info:=/oakd/rgb/preview/camera_info

# terminal 3 — GUI, LAST. Do not press START ALL.
python3 ~/vla_gui_v2.py
```

---

## 11. NEXT SESSION — SUPERSEDED BY §13.3

Kept for the record. Items 3 and 5 were completed on 13 Aug; the current
priority order is **§13.3**.

1. **Fix the stop command** (§8.1). Safety, and it blocks live demonstration.
2. Apply the `~/.bashrc` guard so new terminals default to simulation-safe.
3. ~~Test `vla_robot.sh` on hardware — it has never been run.~~ **DONE 13 Aug** (§13.1).
4. Verify #61 ranging on hardware (§8.3) and diagnose the sim bearing (§8.2).
5. ~~Add `python3 -u` to `vla_sim.sh` so logs flush live.~~ **DONE 13 Aug** (§13.1).
6. Charge the Create 3 fully before any hardware session and watch the battery.

---

## 12. WHAT IS DEFENSIBLE IN A VIVA

Four items from this phase are engineering, not debugging — each predicts a
failure from first principles or corrects a claim against evidence:

1. **#62's principle** — a safety layer must not depend on the estimate it
   protects against, *and* must know the direction of travel. The Nav2 case,
   where the guard must stay unconditional precisely because the agent cannot
   see the commanded velocity, is the interesting half.
2. **#63's principle** — cosmetic output must never be able to abort a control
   action. Fixing the mechanism rather than 55 call sites is the defensible
   choice.
3. **Wait for data, not for names.** A topic that exists but is silent is the
   most common false positive in a ROS 2 health check, and it is exactly what
   made three consecutive launches report success while failing.
4. **The §7 retraction.** Finding that a documented "unexplained hardware fault"
   was a misread cumulative counter — and saying so — is stronger evidence of
   method than leaving it in the report.

**Presentation note.** The simulation is no longer a fallback of last resort. It
runs the identical agent binary, over the same ROS 2 interfaces, with only topic
names remapped. That is a direct consequence of the modular VLA design: the
three pretrained modules do not know or care whether the wheels are real.

---

## 13. SESSION RECORD — 13 AUG 2026

Work not already written up above. Where a finding has its own section, this
points there rather than repeating it.

### 13.1 BUILT

* **RViz recovered and decoupled.** Written up in §8.4.

* **`vla_gui_v2.py` — the launch panel is hidden without being destroyed.**
  `left.hide()` must come **after** `root.addWidget(left)`, never before. The
  widget has to acquire its parent first: hide it while it is still unparented
  and Qt destroys it, after which `_tick()` crashes every 300 ms dereferencing
  the deleted status lamps. The *ordering* is the fix — this is not cosmetic,
  and swapping the two lines back reintroduces a repeating crash.

* **`vla_kill.sh` — `rviz2` added, and the two lists merged.** `rviz2` now
  appears in **both** process-group patterns and in the survivors list, and the
  verification reads a single `SURVIVORS` variable. The kill list and the
  verify list had drifted apart, so the script could kill one set and check a
  different one — reporting a clean system while a process it never looked for
  was still running. Worth noting what that means: §4's lesson is "prove the
  kill worked," and here the proof itself was incomplete. A verification step
  is only as good as its list.

* **`vla_robot.sh` — first successful run, ever.** Previous §11 item 3 closed.
  Added this session: the agent launch (§8.7), the YOLO and Ollama checks, a
  real `start_camera` call in place of printed advice (§8.8), and the TCP
  reachability check (§13.2). Clock, kill/verify, camera, agent and GUI all
  came up clean end to end.

### 13.2 FOUND

* **`ping` is useless inside `ubuntu22-gpu`.** It exists at `/usr/bin/ping` and
  exits **2 with zero output** — no header, no statistics — for *every*
  address: the container is not granted the raw-socket capability ICMP needs.
  The reachability check therefore failed with the robot up and healthy, and
  blocked the script entirely. It could never have passed on any hardware, so
  this was not a regression; the check had simply never worked.

  Replaced with bash's `/dev/tcp`, which needs no privilege and tests a real
  connection rather than ICMP — which is what ROS actually depends on:

  ```bash
  timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22"
  ```

  The tell that separates this from a genuine outage is the **silence**: an
  unreachable host still prints a header and a 100 %-loss summary.

* **`FASTRTPS_DEFAULT_PROFILES_FILE` is NOT set by
  `/etc/turtlebot4_discovery/setup.bash`.** ~~That file sets `ROS_DISCOVERY_SERVER`
  and only that.~~ **CORRECTED 8 Sep 2026:** that file sets four variables —
  `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID=0`, `ROS_DISCOVERY_SERVER`, and
  `ROS_SUPER_CLIENT`, which is `True` only when the shell has a terminal
  attached and `False` in any `bash -c` wrapper. In a wrapper shell, sourcing
  it alone makes a plain client that receives topic data but no directory,
  which shows as an empty topic list. The XML profile forces super-client
  mode regardless of the flag, so hardware needs **both** variables set
  together, which is what §4 of `vla_robot.sh` does. The rest of the original
  finding stands: this is the same silent-failure class as §3.3 — half the
  hardware settings is not half working, it is not working at all.

* **SLAM eliminated as a camera bottleneck.** Dropouts occur with SLAM running
  and with SLAM absent, and the feed held 0.0–0.1 s either way. **Nav2 remains
  the untested load** — but see §8.8: the message means the agent's RGB
  callback did not run, which can be internal starvation rather than anything
  on the wire, and neither SLAM nor Wi-Fi could have shown that.

* **The voice gate was never waiting on Whisper.** `/vla/voice/state` is
  event-driven, so the "wait for data" rule could not work on it and burned the
  full 240 s against a healthy node. Written up as the boundary condition on
  §3.2, in **§3.2a**. `vla_sim.sh` now waits for a publisher there instead.

* **Wi-Fi power save eliminated — and it does not survive a reboot.** It was
  ON; `sudo iw dev wlan0 set power_save off` confirmed it off; the stalls
  continued. Eliminated as the cause. **The setting is lost on every Pi
  reboot**, so anyone re-testing must confirm it is still off first or the
  negative result is worthless.

### 13.3 NEXT SESSION, in order

> **NOTHING IN THIS SESSION'S SAFETY STORY IS VERIFIED YET.**
>
> The manual-override fix is written, `bash -n` clean and `py_compile` clean —
> but **no `xterm` has ever been spawned by either script, and nobody has typed
> `manual override` into one.** Until both of those have actually happened,
> §8.7 is a claim and not a result, and the §8.1 fallback is assumed rather
> than known.
>
> **Step 3 is blocked until step 1 passes.** Do not drive the robot under
> autonomy on the strength of a fix that has only ever been syntax-checked.

1. **Verify manual override actually works in the new `xterm`** (§8.7). The fix
   is written and syntax-checked but has never been run. Until it is confirmed,
   the §8.1 fallback is assumed, not known.
2. **Add SLAM + Nav2 to the hardware stack.**
3. **Test `go to <object>` on hardware — not before step 1**, and with a
   physical fallback within reach. This is the first step that drives the base
   under autonomy with §8.1 still open.
4. **The four-cell camera stall test.** `ros2 topic hz` on the raw RGB while
   the agent reports a stall, with the GUI feed panel open and again closed. It
   separates a starved callback from a genuinely dead camera — the distinction
   neither the SLAM nor the Wi-Fi test could make.
5. **§8.1**, the stop command.

Steps 1 and 3 are ordered deliberately: 3 is the first thing that moves the
robot autonomously, and 1 is what gives the operator a way to stop it.

---

## 14. SESSION RECORD — 19–22 AUG 2026

Hardware sessions. Everything below is **confirmed by observation** unless the
section says otherwise. The one open question is isolated in §14.8.
§§14.1–14.8 are the 19–21 Aug hardware findings; **§§14.9–14.11 were found on
22 Aug** during a clean bring-up, and are all workstation-side traps — none of
them was the robot.

### 14.1 `robot_mode.sh` / `sim_mode.sh` MUST be sourced, never executed

```bash
source ~/robot_mode.sh      # correct
~/robot_mode.sh             # WRONG — sets nothing
```

Executed, the script runs in a **child shell**. It exports the variables into
that child, prints its success banner, and the child exits taking every export
with it. The parent shell — the one you then run ROS in — is untouched.

**The banner is printed either way.** It is not evidence of anything: it proves
the script reached its last line, not that your shell has the variables. This
is the same false-positive class as §3.2 and §8.8 — a success message issued on
behalf of something that did not happen.

**After every source, verify:**

```bash
env | grep -E "^(ROS_DISCOVERY|FASTRTPS|VLA_NS)"
```

Three lines back = the shell is configured. No output = it is not, whatever the
banner said. Because both modes fail **silently** in the other's settings
(§3.3), an unverified shell is indistinguishable from a broken robot.

### 14.2 The Pi runs `unattended-upgrades` — AND MASKING THE SERVICE DOES NOT STOP IT

**Measured 19 Aug:** 75 % CPU on one core, load average **9.4** on a 4-core Pi.
**Measured again 22 Aug, after it was supposedly masked:** 76 % CPU, load 3.2.

The symptom set is identical to a network fault — topics arrive late or not at
all, SLAM drops scans, the camera stalls — and nothing in ROS names the cause.

**THE CORRECTION (22 Aug).** This section previously said "now masked, so it
should not recur." **That was wrong**, and it cost most of an afternoon.
Masking `unattended-upgrades.service` masks *that unit only*. The upgrade also
runs from a completely different path:

```
apt-daily.timer  /  apt-daily-upgrade.timer
        -> /usr/lib/apt/apt.systemd.daily
                -> /usr/bin/unattended-upgrade
```

Caught in the act on 22 Aug, with the service reporting itself inactive:

```
$ systemctl is-active unattended-upgrades
inactive                       <- the service really is masked

$ ps -eo pcpu,pid,comm --sort=-pcpu | head -5
76.2  1731 unattended-upgr    <- running anyway, via the timer
38.0  6927 systemd
24.1  1108 diagnostics_upd
18.4  6726 apt-check
```

**So the obvious check is a false negative.** `systemctl is-active
unattended-upgrades` answers about a unit that is not the one doing the work.
It returns `inactive` while 76 % of a core is gone. This is the same failure
class as §8.8 and §14.1 — a health check that truthfully answers a question
nobody asked.

**THE WORKING CHECK — look for the processes, never the service state:**

```bash
ssh ubuntu@10.42.0.169 'uptime; pgrep -af "[u]nattended-upgrade|[a]pt\.systemd\.daily|[a]pt-check|[d]pkg|[a]pt-get"'
```

No process lines back = genuinely clear. Note the **bracketed** patterns: the
ssh command line itself appears in the Pi's process table, so an unbracketed
pattern matches itself and the check can never come back clean (§14.9).

**Do not kill apt mid-transaction** — an interrupted dpkg run can leave the
package database needing `sudo dpkg --configure -a` before anything else works.
Wait for it to finish, or stop the units cleanly:

```bash
sudo systemctl stop apt-daily.service apt-daily-upgrade.service
```

**The permanent fix is to mask the TIMERS, not the service:**

```bash
sudo systemctl mask apt-daily.timer apt-daily-upgrade.timer
```

**The general lesson.** "I disabled it" is a claim about a configuration, not an
observation of a system. The only thing that settles whether a process is
running is looking for the process. A masked unit and a running workload are
perfectly compatible when more than one unit can start the same work.

### 14.3 The Create 3 stops publishing with no warning — NOT a low-battery symptom

Supersedes the battery suspicion in **§8.5**.

**Signature, all at once and with no error anywhere:**

* `/robot1/odom` stops publishing
* `/robot1/battery_state` goes empty
* the `odom -> base_link` transform disappears
* **`create3_republisher` keeps running** — the process is alive, `pgrep` finds
  it, and it is publishing nothing

That last point is what makes it hard: every process-level check passes. The
node is up. The data is gone.

**This is turtlebot4 issue #554.** It is a known upstream fault, not something
in this project.

**The only fix is a physical power-cycle of the base.** Restarting nodes,
relaunching the stack and `vla_kill.sh` all fail to recover it.

**It happened twice on 19–21 Aug, once at 63 % battery.** §8.5 recorded the
silences clustering at low battery and named battery a strong suspect — that
suspicion is now **retracted**. 63 % is not a low-battery condition, and the
correlation in §8.5 was a small-sample coincidence.

**Detection.** `ros2 node list` will still show the node (§14.6), so do not use
it. Use the transform:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Silence there with `create3_republisher` alive = this fault. Power-cycle.

### 14.4 slam_toolbox `minimum_travel_distance: 0.1` kills `map -> odom` when stationary

**Keep it at `0.0`.** — **CONTRADICTED 8 Sep 2026, see §15.7 item 4 before
acting on this:** `~/slam_vla.yaml` ran all day with `0.2` and the `map`
frame stayed valid through 45+ min stationary on the dock. Do not "fix" the
file to 0.0, and do not delete this section, until the two are reconciled by
a test.

`minimum_travel_distance` tells slam_toolbox to skip processing a scan unless
the robot has moved at least that far. The published `map -> odom` transform is
**stamped from the last scan it processed**. Park the robot and no scan is
processed, so the transform is never re-stamped, its timestamp ages past the TF
buffer's tolerance, and consumers report the `map` frame as simply **not
existing**.

The robot looks lost while sitting perfectly still and doing nothing wrong. It
recovers the moment it is pushed 0.1 m — which reads as "movement fixed it" and
sends you looking in the wrong place.

At `0.0` every scan is processed and the transform is continuously re-stamped.
The cost is CPU on redundant scans; on this system that is affordable and a
vanishing `map` frame is not.

### 14.5 `/opt` configs restored to stock

`nav2.yaml` and `slam.yaml` in `/opt` are now **stock**, restored from:

| Restored from | Into |
|---|---|
| `~/nav2.yaml.pre_v25` | `/opt` nav2 config |
| `~/slam.yaml.bak` | `/opt` slam config |

The tuned versions from these sessions are preserved as `~/nav2.yaml.today` and
`~/slam.yaml.today` — **not in use**, kept so the tuning is not lost.

Anything that behaved differently before 21 Aug because of a `/opt` edit will
now behave as stock. The `minimum_travel_distance` finding in §14.4 applies to
whichever slam config is live — check it after any restore.

### 14.6 `ros2 node list` / `node info` / `param get` are unreliable through the discovery server

They time out, return partial lists, or list nodes that are publishing nothing
(§14.3) — and they do it inconsistently, so a passing run does not mean the
next one passes. Introspection traffic goes through the discovery server and is
not the same path the data takes: a clean `node list` is not evidence the data
path works, and an empty one is not evidence it is broken.

**CORRECTED 8 Sep 2026.** These commands are answered by the ROS daemon, one
shared background process. They are complete and stable when the daemon was
started from a robot-mode terminal (verified repeatedly on 8 Sep), and empty
or partial when it was started from a terminal without the discovery
variables. Inconsistency between runs means the daemon was restarted in
between from a different terminal. Before trusting or distrusting
`node list`, check the daemon's environment with the one-liner in
`CHANGELOG_2026-09-08.md` §5. A clean `node list` still says nothing about the
data path; `topic hz` and `tf2_echo` remain the ground truth for that. A
second, separate effect found the same day: a freshly started program takes up
to ~20 s to be fully matched through the discovery server, so `param get`,
`service call` and `action send_goal` from a new terminal can hang while
everything is healthy (§15.4).

**Use these as ground truth instead:**

| Question | Use |
|---|---|
| Is the process running? | `pgrep -af <name>` |
| Is the transform live? | `ros2 run tf2_ros tf2_echo <a> <b>` |
| Is the topic carrying data? | `ros2 topic hz` / `echo --once` (§3.2) |

`pgrep` reads the local process table and `tf2_echo` subscribes to real data —
neither depends on discovery-server introspection.

### 14.7 With the OAK-D running, Pi load hits 8.0

**Keep the camera stopped unless it is needed.** Load 8.0 on 4 cores means
everything else on the Pi — odometry, the LiDAR, the republisher — is competing
for time. Start the camera when a step actually needs vision (and remember
§8.8: start it *after* undocking, and the success return means nothing).

This also gives §13.3 step 4 (the four-cell camera stall test) a plausible
mechanism that neither the SLAM nor the Wi-Fi test could have exposed: the
stall may be the Pi running out of CPU, not the link and not a dead camera.

### 14.8 NOT ESTABLISHED — do not assume either way

**Whether starting SLAM triggers the Create 3 failure (§14.3), or whether the
base dies on its own.**

Both silences on 19–21 Aug are consistent with either reading, and there is not
enough data to separate them. Do not record a cause in either direction, and do
not let §14.3's "known upstream fault" framing quietly close the question —
issue #554 says the base goes silent, not what provokes it.

What would settle it: run the base with SLAM absent for as long as a session
that normally produces a silence, and separately start SLAM on a base that has
been idle and healthy, watching `tf2_echo odom base_link` across the start.
A silence only in the SLAM case ⇒ SLAM is implicated; silences in both ⇒ it is
not.

### 14.9 `pkill -9 -f <pattern>` KILLS ITS OWN SHELL

**22 Aug.** A cleanup one-liner exited **1 with no output whatsoever** — not
even the banner of a script it sourced later. Nothing in it had run.

```bash
bash -lc 'pkill -9 -f slam_toolbox; pkill -9 -f nav2; ... ; source ~/robot_mode.sh && ros2 daemon stop'
```

`pkill -f` matches against the **full command line** of every process. The
`bash -lc '...'` shell running that one-liner has the whole string — including
the text `slam_toolbox` — in *its* command line. `pkill` exempts itself but
**not its parent shell**, so the very first `pkill` killed the shell executing
it. Everything after that point never ran: the `rm`, the `source`, the daemon
restart. The `distrobox enter` wrapper on the host carries the same string and
is equally exposed.

**This can never work, and it fails silently** — an exit 1 with no output looks
like "nothing matched", which is exactly what a *successful* cleanup on a clean
system looks like.

**Confirmed both ways**, with a token matching no real process, so the only
process containing it was the shell itself:

```bash
bash -lc 'pkill -9 -f zqx_test_token_1234; echo SURVIVED'      # prints nothing
bash -lc 'pkill -9 -f "[z]qx_test_token_1234"; echo SURVIVED'  # prints SURVIVED
```

**The fix is the bracket trick.** `[s]lam_toolbox` is a regular expression that
matches the string `slam_toolbox`, but the shell's own command line contains
the literal text `[s]lam_toolbox`, which the expression does **not** match.

**Working form:**

```bash
pkill -9 -f "[s]lam_toolbox"; pkill -9 -f "[n]av2"
pkill -9 -f "[c]ontroller_server"; pkill -9 -f "[p]lanner_server"
pkill -9 -f "[b]t_navigator"; pkill -9 -f "[l]ifecycle_manager"
```

`~/vla_kill.sh` is unaffected — it kills by process **group** (§4), not by
pattern, which is a further argument for that design.

### 14.10 `robot_mode.sh` DID NOT SOURCE ROS — and hid the failure — FIXED 22 Aug

**Found:** `source ~/robot_mode.sh` set all three discovery variables correctly
and then `ros2 daemon stop` returned `ros2: command not found`. The script
exported the environment but never sourced `/opt/ros/humble/setup.bash`, so
there was no `ros2` on `PATH` at all.

**The part that matters:** the script's *own* daemon restart, lines 42–44, was
written as

```bash
ros2 daemon stop  >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
```

so **it had been failing invisibly for the entire life of the file.** The
banner printed, the environment verified clean under §14.1's `env | grep`, and
the daemon was never actually restarted. §14.1 says the banner is not evidence;
this is the sharper version — *the env check is not evidence either*, because it
only proves the exports ran, not the commands between them.

**FIXED 22 Aug.** `robot_mode.sh` now sources ROS before the discovery exports
(guarded, so it warns instead of failing mutely when run on the host), and the
two daemon calls are no longer silenced. Backup of the original:
`~/robot_mode.sh.bak`. Verified — sourcing it alone now yields all three
variables *and* `/opt/ros/humble/bin/ros2`.

**`sim_mode.sh` HAD THE SAME DEFECT** — no ROS source, and the same two
silenced daemon calls at lines 57/59. **Fixed the same day — see §14.12**,
which also removed a domain-42 line that contradicted §5.1.

**A second trap found while testing the fix.** This does not work either:

```bash
source ~/robot_mode.sh | tail -6      # WRONG — exports are lost
```

A pipeline runs its commands in a **subshell**, so the exports die with it,
exactly as if the script had been executed instead of sourced (§14.1). Never
pipe, redirect into, or `$( )` a `source`. To quieten it, redirect the *script's*
output instead — `source ~/robot_mode.sh >/dev/null 2>&1` — which keeps the
source in the current shell.

### 14.11 `ros2 topic echo` FAILS ON TYPE LOOKUP, NOT ON DATA

```
WARNING: topic [/robot1/battery_state] does not appear to be published yet
Could not determine the type for the passed topic
```

Read literally this says the base is silent — the §14.3 signature. **It was
not.** At that same moment `/robot1/odom` was publishing at a steady 20.0 Hz and
`odom -> base_link` was live. The battery topic was fine; only the *type
lookup* failed, and that lookup is introspection through the discovery server —
precisely what §14.6 says is unreliable.

**Working form — pass the message type explicitly and the lookup is skipped:**

```bash
ros2 topic echo /robot1/battery_state sensor_msgs/msg/BatteryState \
     --field percentage --once
```

That returned `0.93` first try and every time after.

**`ros2 topic list` was measured lying in both directions on 22 Aug**: 27 topics
(24 under `/robot1`), then **2 topics**, from identical back-to-back commands in
the same shell seconds apart. Treat it as unusable for hardware diagnosis.

**The rule this adds to §14.6:** an introspection failure and a dead base
produce *the same error text*. Never conclude §14.3 from a `topic echo` or
`topic list` failure alone — confirm with `topic hz` and `tf2_echo`, which
subscribe to real data. Two of those three passing means the base is alive
whatever the third says.

### 14.12 `sim_mode.sh` BROUGHT INTO LINE — FIXED 22 Aug

Closes the "not yet fixed" note in §14.10. Three changes, all verified from a
fresh shell.

**1. The §14.10 defect, identical to `robot_mode.sh`.** No
`source /opt/ros/humble/setup.bash`, and its own `ros2 daemon stop/start` at
lines 57/59 wrapped in `>/dev/null 2>&1` — so those had been failing invisibly
for the life of the file. ROS is now sourced first (guarded, warns instead of
failing mutely on the host) and the daemon calls are no longer silenced. The
fix proved itself on the first run: `The daemon has been stopped` /
`The daemon has been started` now appear where previously there was nothing,
and nothing is what a total failure also looked like.

**2. `export ROS_DOMAIN_ID=42` REMOVED — it contradicted §5.1.** The file set
the domain to the value §5.1 records as *tried and rejected*: `gz_ros2_control`
runs **inside** the Gazebo process and does not inherit the domain, so
`controller_manager` comes up on 0 while the spawner looks for it on 42. The
robot spawns and cannot move. Meanwhile the scripts that actually run the demo
both unset it — `vla_sim.sh` line 46 and `run_agent_sim.sh` line 22, each with a
comment saying 42 broke the simulation. `sim_mode.sh` was the odd one out.

The danger was specific: sourcing `sim_mode.sh` and then launching by hand — the
§10 "manual sequence, if the script fails" path, i.e. the fallback *to* the
fallback — put you straight onto the broken configuration, and §5.1's failure
mode is silent-ish and slow to diagnose. `sim_mode.sh` now unsets the domain,
matching both scripts. The banner, which printed `domain=$ROS_DOMAIN_ID`, would
have rendered as an empty `domain=`; it now reads `domain=(default, unset)`.

**3. The header at lines 17–27 was stale and contradicted the code.** It was
titled "THE ISOLATION GUARANTEE" and said `ROS_DOMAIN_ID` "is the important line
below" — arguing for exactly what §5.1 rejected. Rewritten to state how
isolation is actually achieved: the discovery-server unsets, the mode lock
(`/tmp/vla_mode.lock`), and separate GUI configs. The genuine observation it
recorded is kept — stale `/robot1/oakd/...` topics from a morning hardware
session were visible in the simulation and the agent bound to them instead of
the simulated camera — because that is a real event; only the attribution of
the fix was wrong.

**Verified from a fresh shell after the change:** `ROS_DOMAIN_ID` absent (unset,
not empty), `VLA_NS` set to the empty string as §3.3 requires, both discovery
variables absent, raw RGB/depth both `1`, and `ros2` resolving to
`/opt/ros/humble/bin/ros2`.

**Reference copy: `~/sim_mode.sh.WORKING`**, byte-identical to the
pre-change file (md5 `8d105229137e14cd355513a89baaebd9`), kept deliberately
untouched so the evaluation fallback can always be restored.

**The lesson, and it is §14.10's generalised.** Three files encoded the same
decision — `vla_sim.sh`, `run_agent_sim.sh` and `sim_mode.sh` — and one of them
never got the correction. A rejected approach does not stay rejected unless
every copy of it is removed; §5.1 said "do not reintroduce the domain ID" while
a file in the same directory was reintroducing it on every source.

### 14.13 CORRECTION TO §8.6 — THE `.bashrc` GUARD IS APPLIED

§8.6 lists as an open item: *"`~/.bashrc` guard not yet applied. Every new
terminal still points at the Pi's discovery server."* **That is out of date.**
The guard was applied on **19 Aug** and is present at **`~/.bashrc` lines
125–130**:

```bash
# VLA: hardware discovery vars only when VLA_MODE=robot is exported first.
if [ "$VLA_MODE" != "robot" ]; then
    unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE
fi
```

Lines 122–123 still source `/etc/turtlebot4_discovery/setup.bash` and set the
super-client profile; the guard then strips **both** variables again unless
`VLA_MODE=robot` was exported first.

**Consequence: a fresh terminal is simulation-safe by default,** and hardware
mode is opt-in. Confirmed by observation on 22 Aug — a fresh `bash -lc` shell
inside `ubuntu22-gpu` had `ROS_DISCOVERY_SERVER` and
`FASTRTPS_DEFAULT_PROFILES_FILE` both **absent** before anything was sourced.

This is a **documentation fix, not an open question.** §11 item 2 ("apply the
`~/.bashrc` guard so new terminals default to simulation-safe") is closed. Note
that the guard unsets the two discovery variables only — it does not touch
`ROS_DOMAIN_ID`, which nothing sets by default and which §5.1 requires to stay
that way.

Two consequences worth carrying forward:

* The old habit of assuming a new terminal is in **robot** mode is now wrong,
  and wrong in the safe direction. §14.1's `env | grep` check still settles it
  in one line, and is still the only thing that does.
* `sim_mode.sh` is no longer needed to make a *fresh* terminal safe. It is now
  for switching a terminal that has been put into robot mode — which is what
  its header says after the §14.12 rewrite.
* **Found 8 Sep 2026:** any ROS daemon started from a guarded terminal is
  blind to the robot and is then used by every terminal, and any interactive
  shell opened from a robot terminal (a bare `xterm`, or typing `bash`)
  re-runs the guard and loses both variables. Section 5 of
  `CHANGELOG_2026-09-08.md` has the checks.

### 14.14 WORKING RULES CONFIRMED THIS PHASE

* One step at a time — run one command, read the output, stop. No batching.
* Always state **which container** a command runs in.
* Every new shell: `source ~/robot_mode.sh`, then verify with `env | grep`
  (§14.1) before any ROS command.
* Never publish to `/robot1/cmd_vel` without warning first — the robot moves.
* Do not theorise when a cheap check would settle it.

### 14.15 PRE-FLIGHT, HARDWARE — the current short list

#### THE GATE — do not start any hardware test until BOTH are true

Added 22 Aug, after an afternoon was lost to measuring a Pi that was busy doing
something else entirely (§14.2).

1. **Pi load average is under 1.5**, and
2. **no apt process is running.**

One command answers both:

```bash
ssh ubuntu@10.42.0.169 'uptime; pgrep -af "[u]nattended-upgrade|[a]pt\.systemd\.daily|[a]pt-check|[d]pkg|[a]pt-get" || echo "APT CLEAR"; vcgencmd measure_temp; vcgencmd get_throttled'
```

Pass = load under 1.5 **and** `APT CLEAR` **and** (added 8 Sep, §15.5)
temperature under 80 °C with `throttled=` ending in `0`. Anything else,
**wait** — do not start, and do not kill apt mid-transaction (§14.2).

**Why a gate and not a guideline.** A Pi under unrelated load produces the exact
symptom set of a network fault, a dying base, and a starved camera — the three
things most hardware tests here are trying to tell apart. A measurement taken
under that load cannot distinguish its own subject from the noise, so it is not
a weak measurement, it is a **void** one.

**The old threshold was wrong, too.** This section previously said load "< ~4".
That was carried over from §14.2's observation that the *failure* case reached
9.4, which makes 4 a threshold for catastrophe rather than for a clean
measurement. A healthy idle Pi in this system sits at **0.6–0.8** (measured at
Phase 2 start, 22 Aug). Anything above 1.5 means something unaccounted for is
running.

#### Then, in this order

1. **THE GATE above** — load < 1.5 and apt clear (§14.2)
2. `ssh ubuntu@10.42.0.169 'chronyc tracking | grep "System time"'` — clock (§5.2)
3. `timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22"` — reachable (§13.2)
4. `source ~/robot_mode.sh` then `env | grep -E "^(ROS_DISCOVERY|FASTRTPS|VLA_NS)"` (§14.1)
5. `ros2 run tf2_ros tf2_echo odom base_link` — the base is actually alive (§14.3)

### 14.16 NEXT SESSION

§13.3 still stands and is unchanged — manual override (step 1) is still
unverified and still blocks step 3. Added by this phase:

* Settle §14.8 (does SLAM trigger the Create 3 silence).
* Re-check `minimum_travel_distance` in whichever slam config is live after the
  §14.5 restore.
* ~~Apply the §14.10 fix to `sim_mode.sh`.~~ **DONE 22 Aug — §14.12**, which
  also removed the domain-42 line and corrected the stale header.
* Key-based SSH to the Pi is now set up (22 Aug), so the §14.15 pre-flight runs
  unattended.
* ~~**Mask the apt timers on the Pi**~~ **DONE before 8 Sep** — both timers
  and the service report `masked`, and the Pi was apt-clear all day on 8 Sep.
* **8 Sep 2026:** see §15. Nav2 is verified on hardware; the current
  next-session list is **§15.7**.

---

## 15. SESSION RECORD — 8 SEP 2026

Hardware session. The morning found and fixed the blind-daemon and
shared-memory faults (`CHANGELOG_2026-09-08.md` Part 1, §§1–6); the afternoon
gave RViz a hardware layout and brought Nav2 up on the real robot for the
first time since the `/opt` configs were restored to stock (Part 2, §§7–14).
Everything below was **observed on the live system**; the changelog carries
the full detail, log excerpts and measurements, and this section is the
summary an evaluator needs.

### 15.1 BUILT

* **`~/vla_hardware.rviz`** — the RViz layout for the real robot. Every topic
  written in full as `/robot1/...`, Fixed Frame `map`, the six frames that
  matter, laser, odometry, map, both Nav2 costmaps and plans, camera display
  off by default. QoS set from what the robot actually publishes (`ros2 topic
  info -v`): map, odometry and robot description are **transient local** and
  the displays ask for that, which is the setting that had to be fixed by
  hand every session before. Launch, from a robot-mode terminal in the
  container:

  ```bash
  ros2 run rviz2 rviz2 -d ~/vla_hardware.rviz --ros-args -r __ns:=/robot1 -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
  ```

  The `__ns:=/robot1` is not decoration. RViz's "Nav2 Goal" tool and its
  Navigation 2 panel ignore the layout file and use RViz's own namespace, so
  without it goals go to `/goal_pose` in the root namespace and Nav2 never
  sees them — which is what the old hardware RViz was doing. The simulation
  is untouched: `vla_sim.sh` still loads the stock
  `turtlebot4_viz/rviz/robot.rviz`, whose *relative* topic names (`map`,
  `scan`) only work because the sim has no namespace.

* **`~/nav2_hw.launch.py`** — Nav2 bring-up for the real robot, hardware
  only. The same eight processes, remaps and stock `/opt` parameters as
  `ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1`, plus
  two settings that launch file cannot express: `bond_timeout` 30 s (stock
  4 s) and a 20 s delayed STARTUP instead of autostart. Why both are needed is
  §15.3. Launch, from a robot-mode terminal:

  ```bash
  ros2 launch ~/nav2_hw.launch.py
  ```

  Healthy: `Managed nodes are active` after about 30 s, and
  `ros2 topic info /robot1/cmd_vel -v | grep -c velocity_smoother` prints 1.
  Full output is tee'd to `~/vla_logs/nav2_hw_<date>.log`.

### 15.2 THE HARDWARE NAV2 PROCEDURE

The gate first (§14.15, now with temperature: `vcgencmd measure_temp;
vcgencmd get_throttled` — want under 80 °C and a value ending in 0), then
reachability, robot-mode terminal, base alive (`odom` at 20 Hz,
`tf2_echo odom base_link`), SLAM (§14 procedure, then `tf2_echo map
base_link`), RViz (15.1), Nav2 (15.1), undock, goal from RViz's "Nav2 Goal",
and at the end dock and `~/vla_kill.sh` from a robot-mode terminal. Each
step's check and healthy result are written out in `CHANGELOG_2026-09-08.md`
§8.

### 15.3 FOUND — the stock Nav2 launch fails through the discovery server, in two different ways

Both are timing, not configuration. No parameter was wrong, no remap was
missing, the costmaps built and the `map`/`odom`/`base_link` frames were
connected throughout.

1. **Bond timeout.** After the lifecycle manager activates each server, the
   two open a heartbeat link (a "bond") and the manager allows 2 s (half of
   `bond_timeout`) for it to form. Through the discovery server on the Pi the
   fifth link took longer, the manager aborted, and the last two servers —
   including `velocity_smoother`, the one that publishes `/robot1/cmd_vel` —
   were never activated. Goals are accepted and the robot does not move.
   Fix: `bond_timeout` 30 s.
2. **Dropped reply.** With the timeout raised, the manager sent its first
   configure request within a second of starting. The server acted on it,
   but the reply was discarded because the manager's reply channel was not
   yet matched — rmw_fastrtps waits only ~0.1 s for that — and the manager
   waits forever for a reply. Fix: start-up delayed 20 s so every channel is
   matched first. Third attempt: active in 31 s, all seven bonds in under
   1.2 s.

   Caveat, recorded honestly: the third attempt also stopped polling the
   bring-up with `ros2 lifecycle get` every 5 s. Each poll is a new program
   the Pi's discovery server must serve. Which change did the work was not
   separated; both stay.

### 15.4 FOUND — new programs are matched slowly, and replies to them are dropped

Measured on 8 Sep: a brand-new program receives from an existing publisher
after 2.0–2.4 s; an existing program receives from a brand-new publisher after
**18.7 s**; a fresh service call to a Nav2 node worked once in 4.5 s and got
no reply in 30 s the next time; three planner-only goals from the command
line got no reply at all, and the planner's log shows why:
`Failed to send goal response ... (timeout): client will not receive`, once
per attempt. The request arrived, the answer was thrown away.

**The rule.** RViz, the agent and Nav2's own lifecycle manager keep their
clients alive from start-up and are unaffected. `ros2 action send_goal`,
`ros2 service call` and `ros2 param get` from a fresh terminal may hang or
need a retry on this system, and a hung one is never by itself evidence of a
fault. Send goals from RViz or through the agent.

### 15.5 FOUND — the Pi is thermally throttled and saturated

With the camera on: load 6.9 on 4 cores, **84.7 °C**, `throttled=0xe0008`
(soft thermal limit active now; clock-capped and throttled at some point
since boot), clock 1.68 GHz of 1.8. With the robot docked and the camera
off: load 5.3, still 84.7 °C, clock 1.53 GHz. Apt clear, timers masked, clock
synced, memory fine. The CPU is spread across nearly every ROS node on the
Pi — diagnostics updater 37 %, Create 3 republisher 32 %, OAK-D 27 %, two
kernel Wi-Fi/USB threads ~27 % each, even the two joystick nodes 13–14 %
with no joystick attached — and with the camera off the discovery server
itself was at 27 %. That pattern says DDS housekeeping for a graph of 74
topics and 233 services, not any one node's real work.

**Measured at shutdown:** ten minutes after SLAM, Nav2, RViz and every
command-line tool on the PC were stopped — robot docked and camera off in
both readings, the Pi's own nodes unchanged — the Pi fell from load 5.3 and
84.7 °C to load 1.39 and 70.6 °C, and the soft thermal limit cleared. A
large share of the Pi's load and heat was the work of serving the PC-side
graph through the discovery server, not the Pi's own sensor work.

This is the probable common cause of 15.3 and 15.4, and it is a
**correlation, not yet a proof**: Nav2 ran on this architecture in August.
The test that settles it is in the changelog §9.5 — cool the Pi, confirm
`get_throttled` ends in 0, re-run the three timing tests. The Pi has no fan
and no thermal overrides in `config.txt`.

### 15.6 VERIFIED — two Nav2 goals driven on hardware

Sent from RViz's "Nav2 Goal", 12:37–12:38, no recoveries:

| Goal | From (map) | To (map) | Distance | Time |
|---|---|---|---|---|
| 1 | (0.04, -0.05), on the dock | (-0.29, 3.20) | 3.3 m | 17 s |
| 2 | (-0.24, 3.05) | (-0.61, -0.84) | 3.9 m | 18 s |

Evidence: `Begin navigating ...`, `Reached the goal!` and `Goal succeeded` in
the Nav2 log; RViz panel Feedback "reached"; afterwards `map -> base_link` at
(-0.36, 0.36) heading 70°, `odom` still 20 Hz, `is_docked: false`, `cmd_vel`
silent when idle, battery 54 %. The first goal was sent while still docked
and Nav2 drove straight off the dock; that worked, but undocking first is the
correct order. The log also holds 417 "Behavior Tree tick rate exceeded"
warnings and two "Control loop missed its desired rate of 20 Hz": the
workstation was busy with CPU-drawn RViz, SLAM and Nav2 together. Paths were
still tracked; expect it to get worse when the agent and YOLO are added.

A data point for **§14.8**: SLAM ran from 11:07 to at least 12:52 with the
Create 3 alive throughout — one session with SLAM and no silence. It does not
settle the question (the SLAM-absent run is still needed) but it is one
observation against "SLAM triggers it".

### 15.7 NEXT SESSION, in order

1. **Task 3 — the full VLA stack on hardware** (camera → YOLO → Ollama →
   agent → GUI → `go to <object>`). Start the camera only when the step
   needs it and only after undocking (§8.8, §14.7); the agent creates its
   clients at start-up, so 15.4 does not affect it. §13.3 step 1 (manual
   override in the xterm) is **still unverified** and still the operator's
   only fallback for §8.1.
2. **Cool the Pi**, then re-run the 15.4 timing tests to settle 15.5. A
   fan on the Pi 4 is the obvious hardware fix.
3. Retry the stock `nav2.launch.py` once on a cool Pi, so the evaluation can
   say whether `nav2_hw.launch.py` is a workaround for heat or a permanent
   need.
4. Reconcile `minimum_travel_distance`: `~/slam_vla.yaml` has 0.2 and the
   map frame stayed valid through 45+ min docked on 8 Sep; §14.4 says 0.1
   breaks it. Do not change either without a test.
5. Consider disabling the diagnostics updater and the joystick nodes on the
   Pi to buy back CPU; that changes the robot's own bring-up, so test it
   separately.
6. Never run `pkill -f` in a command that also contains the pattern text —
   the shell kills itself (§14.9 generalised; changelog §9.6). Kill by PID.

### 15.8 WHAT IS DEFENSIBLE IN A VIVA, from this session

* **Timing budgets, not configuration.** Nav2's bond handshake and the
  request/reply channels are designed for millisecond local discovery; through
  a discovery server on a throttled computer those budgets are exceeded, and
  the failures look like misconfiguration while every configuration line is
  correct. Reading the actual log lines and measuring the actual matching
  times is what separated the two.
* **Names are the interface.** The stock RViz layout is not wrong; it is
  written for a robot with no namespace. Absolute names and an explicit
  namespace made one layout work on hardware without touching the sim.
* **The observer is part of the load.** Polling the bring-up with new
  command-line programs added work to the very component that was
  struggling. The clean run was watched from a log file.
* **Correlation stated as correlation.** The thermal reading explains
  everything and has not been proven to; §15.5 says which experiment would.

--------------------------------------------------------------------------

## 16. SESSION RECORD — 10 SEP 2026 (Task 3: the full VLA stack on hardware)

Detailed evidence: `CHANGELOG_2026-09-10.md`.
What to do next, self-contained: `HANDOVER_2026-09-10.md`.

### 16.1 OUTCOME IN ONE PARAGRAPH

Every component of the VLA stack was brought up on the real robot
simultaneously and each passed its own check: robot-mode daemon, SLAM,
RViz, Nav2, the OAK-D with depth, YOLO-World on live frames, Ollama /
Qwen2.5, the agent, the voice node and the operator GUI. An operator
command issued through the GUI (`what do you see`) returned a correct
answer, exercising GUI -> agent -> camera -> YOLO -> LLM -> reply.
**The autonomous `go to <object>` drive was not completed.** One blocker
was found and fixed (the camera was publishing no depth at all); a second
was found, measured and diagnosed but deliberately left unfixed, because
its two candidate causes require opposite remedies and guessing would risk
the working simulation demo.

### 16.2 BUILT / CHANGED

* `~/nav2_hw_slow.yaml` — the stock Nav2 parameter file with 14 motion
  lines softened (0.15 m/s, 0.4 rad/s, accelerations cut 4-5x). `/opt`
  untouched, so the simulation is unaffected. **Never loaded; untested.**
* `~/vla_tools/` — 17 diagnostic and launcher scripts, each verified with
  `python3 -m py_compile` or `bash -n`. Index in changelog §13.
* `CHANGELOG_2026-09-10.md`, `HANDOVER_2026-09-10.md`; `CLAUDE.md` updated.
* On the robot only, and NOT persistent: the OAK-D pipeline switched from
  `RGB` to `RGBD` as a live parameter.
* Nothing shared with the simulation was modified. `vla_agent_v28.py`,
  `yolo_server.py`, `llm_brain.py`, `voice_command.py`, `slam_vla.yaml`
  (md5 `fc4265602444a83d8cd7a3a72f14a147`) and everything under `/opt` are
  byte-identical to how the session found them.

### 16.3 FOUND — the OAK-D was publishing colour only, and that alone
###        would have stopped the whole demonstration

The Pi's `oakd_lite.yaml` is the untouched upstream default and sets
`i_pipeline_type: RGB`. No `/robot1/oakd/stereo/*` topic therefore existed.
The agent's `current_pair()` returns a frame only when BOTH colour and
depth are present, so `camera_ready()` would have been false and YOLO would
never have been called — `/vla/status` would have read `camera: false`.
Notably the #54 LiDAR ranging cannot compensate, because it runs downstream
of that gate.

The fix does **not** require touching the robot's filesystem or restarting
its software: `camera.i_pipeline_type` is settable at runtime, and after
`set_parameters` + `stop_camera` + `start_camera` the depth topics appeared
at 4.52 Hz (`compressedDepth`) and 29.5 Hz (`stereo/camera_info`), both
better than the agent's own comments assume. `/vla/status` then read
`camera: true, map: true`. The switch is lost on any reboot or service
restart and must be re-applied each session.

### 16.4 FOUND, MEASURED, NOT FIXED — camera frames are ~4.3 s stale,
###        and this is what makes `go to <object>` fail

Comparing each message's `header.stamp` with arrival time, for two streams
from the same Pi over the same link (so any clock offset cancels):

    /robot1/scan                              median  178 ms
    /robot1/oakd/.../image_raw/compressed     median 4300 ms

stable across six consecutive 10-second buckets. Fix #47 deliberately looks
up TF at the colour frame's timestamp — correct in principle, but it
assumes the stamp is honest. With a 4.3 s-old stamp the robot's pose is
read 4.3 s late, the object is projected to the wrong map position, the
error exceeds `GOAL_REISSUE` (0.9 m), and the agent cancels and re-issues
the Nav2 goal. The agent's own #58 comment describes the consequence:
*"Every trigger CANCELS the path Nav2 is driving and starts a new one — the
robot decelerates, replans, accelerates again. That stop-start IS the jerky
motion."* This matches the operator's report exactly, and #47's comment
already names the symptom: *"it backs up and turns away from the chair that
is right in front of it."*

A second consequence: `SCAN_DWELL_S = 3.5 s`, the stationary pause in the
step-and-stare search (#56), is SHORTER than the lag, so every frame
examined during the "stare" was captured before or during the preceding
turn. The design is defeated by a lag slightly larger than its own pause.

**Why it was left unfixed.** Either the timestamps are wrong (a driver
clock-conversion error, frames actually fresh) or the frames are genuinely
4.3 s old (buffering). The first is fixed by re-stamping frames on arrival;
the second by cutting real latency — and applying the wrong one makes the
behaviour worse. The discriminating test is free and is written out in the
handover. The operator additionally observed that frames "sometimes are not
received by the agent", which suggests buffering and loss rather than a
pure clock error, but that was not measured.

### 16.5 VERIFIED — manual override works on hardware, and is the most
###        reliable stop the system has

Never tested on the robot before (8 Sep handover §7.6 listed it as the
operator's untested fallback). The operator typed `manual override` in the
agent's terminal and confirmed it took control. Reading the implementation:
it refuses unless `sys.stdin.isatty()`; it is a dead-man switch that
publishes a zero velocity by itself if no key has arrived for 0.35 s; and
on exit — by `q`, Ctrl+C or any exception — a `finally:` block restores the
terminal and publishes **five** consecutive stop commands.

**This is a stronger guarantee than the normal stop command that §8.1 of
this handout calls unreliable**, and it is worth stating in the report: the
operator's emergency path is the better-engineered one.

### 16.6 NOT A BUG — the reported object distance is correct

The operator suspected the "chair at 2 m" reading was wrong. Converting
each YOLO box centre to a bearing using the published intrinsics
(`f_x = 197.74`, `c_x = 126.94`) and reading the RPLIDAR at that bearing
gave **2.34 m and 2.30 m** for the two chairs, with the raw forward sweep
agreeing (1.8-2.4 m across the forward arc). The camera-independent sensor
confirms the number.

This also settles how ranging works, which is worth stating precisely in
the report: `range_for_box()` is documented *"LiDAR first, depth as
fallback"*, and #54 states *"LiDAR first — it is the sensor that measures
this robot's world correctly."* **Depth does not normally produce the
distance; the laser does.** Depth's remaining roles are the fallback when
the laser cannot see the target, and gating `camera_ready()` — which is
precisely why §16.3 was fatal.

### 16.7 QUANTIFIED — detection depends on standoff

| robot position | chair confidence | usable? |
|---|---|---|
| 13 cm from the dock, under a desk | 0.174 | no — below the 0.30 floor |
| 2.3 m back, same heading | 0.532 / 0.523 | yes |

The OAK-D on the Lite sits low; close up it sees cropped chair legs and
desk underside. `person` scored 0.698 even at the close position. Design
the demonstration with 2-3 m of standoff, which is also what the agent's
#57 already tries to enforce.

### 16.8 THERMAL — the 8 Sep throttling did not recur

With the Pi rebooted before the session: 51.1 C idle, 57.4 C with
SLAM/RViz/Nav2, 66.2 C with the camera, **73.5 C peak with the depth
pipeline**, and `throttled=0x0` at every reading — the Pi never throttled.
Compare 8 Sep: 84.7 C and `throttled=0xe0008`, with the discovery server
slowing and Nav2 failing twice on timing. Nav2 also activated on the
**first** attempt today (7/7 bonds, 0 errors) against three attempts on
8 Sep. A cool Pi is the plausible difference, though it was not isolated.
**Recommendation for evaluation day: reboot the Pi first.** The depth
pipeline costs +7.3 C, which is affordable; no fan was needed at this
workload.

### 16.9 FIXED — voice input and an LLM stall

* **Voice** transcribed nothing because `voice_command.py` was never
  started. The GUI only publishes push-to-talk on `/vla/voice/trigger`;
  transcription is a separate program. Started with
  `--mode ptt --mic pulse`, Whisper `small.en` loads on CUDA in 4.1 s and
  runs headless waiting for the trigger.
* **Ollama** takes 34.2 s on a cold call and 0.149 s warm, and unloads the
  model after 5 idle minutes — so any pause in a demonstration costs 34 s
  on the next command. Pinned with a single `keep_alive: -1` API call.

### 16.10 WHAT IS DEFENSIBLE IN A VIVA, from this session

1. **The full stack runs on real hardware.** Ten components up at once,
   each with an independent check, and an operator command answered
   correctly end to end through the GUI.
2. **A silent, total perception failure was found by reasoning about the
   code rather than by trial and error** — the missing depth stream would
   have presented only as "the robot never sees anything", and was traced
   to a single upstream default parameter, then fixed at runtime without
   restarting the robot or losing the map.
3. **The remaining failure is measured, not guessed.** 4.3 s of camera
   staleness against a 178 ms laser baseline, with the causal chain traced
   through three documented fixes in the agent (#47, #58, #56) to the
   observed behaviour.
4. **A wrong "fix" was correctly refused.** Two candidate causes require
   opposite remedies; the discriminating experiment is specified and cheap.
   Choosing to measure rather than guess is the defensible engineering
   position, especially with a working demonstration to protect.
5. **A suspected bug was disproved with an independent sensor.** The
   distance reading was checked against the laser and found correct.
6. **The emergency stop was validated** and shown to be more robust than
   the nominal one.
7. **Thermal management is characterised**: a pre-session reboot keeps the
   Pi unthrottled through the heaviest workload the project has run.

### 16.11 NEXT SESSION, in order

1. Charge the robot (it ended at 29 %).
2. Reboot the Pi; run the thermal gate.
3. Bring the stack up per `HANDOVER_2026-09-10.md` §4 — **including the
   RGBD switch at step 7**, which is easy to forget and fatal to omit.
4. Run the camera-latency discriminating test (handover §3) BEFORE changing
   any code, then apply the matching fix.
5. Only then attempt `go to chair`, with 2-3 m of standoff.
6. Save a map. One has never been saved.

## 17. SESSION RECORD — 11 SEP 2026 (last hardware session: the demo works)

Full evidence: `CHANGELOG_2026-09-11.md`. Report/viva text, ready to use:
`REPORT_SECTION_HARDWARE.md`. Packet captures: `~/vla_evidence_20260911/`.

### 17.1 Outcome

* **`go to the chair` runs end to end on the real robot** — by voice and by
  text — three drives out of three reached the chair; one announced
  "Arrived at the chair" (the other two stopped at the right place without
  the announcement, §17.5). `what do you see` works by voice ("shelf
  (0.7 m)", "person (2.7 m)").
* **One command brings the whole stack up from cold:** `~/vla_demo.sh`
  (13 gated stages, ~4 min; only the GUI and RViz are visible).
  `~/vla_demo_stop.sh` re-docks and stops everything including YOLO.
* **The "thermal problem" and the "camera lag" had one cause — our own
  navigation stack was flooding the robot's Wi-Fi link** (§17.2). Fixed.

### 17.2 The Fast DDS heartbeat storm (the finding of the hardware phase)

Every ROS 2 node subscribes RELIABLY to `/parameter_events`. Nav2's
configure-time burst of ~90 parameter events per node, sent to the eight
nodes on the Pi over Wi-Fi, put the Fast DDS writers into a HEARTBEAT storm:
7,400 packets/s PC→Pi (87 % `HEARTBEAT`, one per Pi node every ~1 ms), and
the camera node stormed back at 9,700 packets/s. Valid ACKNACKs ("have
everything, missing nothing") were arriving and being ignored while the
sender's receive socket sat 42 kB deep. Pi: load 7, 84 °C, throttled, camera
frames stopped. Stopping Nav2 dropped PC→Pi from 1,757 to 105 kB/s — the
causal test. Ruled out by experiment: the behaviour-tree tick rate (storm
persisted with Nav2 deactivated), process count (composed Nav2 stormed the
same), stale participants (fresh Pi, fresh daemon, storm back in 20 s), XML
QoS overrides (rmw_fastrtps Humble ignores them — tested), 5 GHz (regulatory
domain unset, AP not allowed).

**Fix:** remap `/parameter_events` → `/pc/parameter_events` and `/rosout` →
`/pc/rosout` in the hardware Nav2 launches (every component, including the
lifecycle manager). PC→Pi with SLAM + RViz + Nav2: **24 kB/s**; Pi load 1.6,
64 °C. Plus an interface whitelist in a new hardware DDS profile so the PC
advertises one address (before, every Pi sample went out twice).

This retro-explains 8 Sep (84.7 °C, bond timeouts, dropped replies, "every
Pi node burning CPU") and 10 Sep's 4.3 s frame lag (measured in RGBD mode
under load; a fresh colour-only camera is at 47–85 ms).

### 17.3 Camera: colour-only, flag `VLA_RGB_ONLY=1`

The depth pipeline (+7 °C, colour halved to 15 Hz, seconds of lag) is no
longer used. `vla_agent_v28.py` #64: with `VLA_RGB_ONLY=1` (set only in
`vla_tools/agent_xterm.sh`) the camera gate opens on colour alone; every
depth consumer already returns None safely; ranging is LiDAR (#54). Default
"0" = previous behaviour; the simulation is unaffected. Diff shown and
approved before applying; backup `vla_agent_v28.py.bak-20260911`.

### 17.4 Voice: fixed

10 Sep's failure was `libcublas.so.12 not found` at the first transcription
(the libraries are pip packages under `~/.local`, not on the loader path).
`vla_tools/voice_xterm.sh` sets `LD_LIBRARY_PATH`; `voice_command.py` is
unchanged. The PC has no built-in microphone — Bluetooth earbuds in headset
mode are the input; the demo script connects them. `small.en` mishears
accented English ("go about that"), but *stop* phrases are always forwarded.

### 17.5 Limits recorded honestly

* Thin-legged chair ranged at 7.1 m from 4–5 m: the 2-D laser looks between
  the legs to the wall. Recovers near the object.
* "Arrived" not announced in 2 of 3 drives: arrival needs ≤ 1.35 m to the
  belief, stand-off 1.30 m + Nav2 tolerance 0.25 m. One-line flag
  `VLA_ARRIVE_TOL` (hardware 0.60) applied 12 Sep, to be proven on the
  cold-start run.
* One false "STILL MOVING — take manual override" during a normal state
  change while Nav2 was driving.
* No map saved; 5 GHz not enabled; Fast DDS internal cause not explained.
* 12 Sep: after an overnight power-cycle the Pi's discovery server came up
  half-deaf to the PC (topics visible, no nodes, no data) and stayed so for
  25 min; `systemctl restart discovery.service turtlebot4.service` on the
  Pi fixed it in 90 s. `vla_demo.sh` stage 2 now does this automatically.

### 17.6 Files

Created: `vla_demo.sh`, `vla_demo_stop.sh`, `nav2_hw_composed.launch.py`,
`.ros/fastdds_hw_wifi_only.xml`, `vla_tools/{voice_xterm.sh, nav2_hwc_xterm.sh,
oakd_pipeline.py, say.py}`, `vla_evidence_20260911/`,
`CHANGELOG_2026-09-11.md`, `REPORT_SECTION_HARDWARE.md`.
Changed: `vla_agent_v28.py` (#64), `nav2_hw.launch.py`, `robot_env.sh`,
`robot_mode.sh`, `vla_tools/agent_xterm.sh`, `CLAUDE.md`, this file.
Untouched: everything in the simulation path, `/opt`, the Pi's files,
`yolo_server.py`, `llm_brain.py`, `voice_command.py`.

### 17.7 Demo day

    ~/vla_demo.sh          # from a plain terminal on the host, robot docked and charged
    ...                    # "what do you see", then "go to the chair" (chair 2-3 m ahead)
    ~/vla_demo_stop.sh     # re-docks and stops everything

Never run either inline in a command that names a ROS process (`vla_kill.sh`
group-kills that shell). Do not close the VLA AGENT window. Do not press
START ALL in the GUI.
