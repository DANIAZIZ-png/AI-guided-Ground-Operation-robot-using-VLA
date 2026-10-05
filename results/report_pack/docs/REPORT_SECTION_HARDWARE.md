# Hardware results — what works, what does not, and why

*Written 11 September 2026 at the end of the last hardware session, for the
project report and the viva. Every number below is from a log or a packet
capture in `~/vla_logs/` and `~/vla_evidence_20260911/`; the dates say which
session produced it.*

## 1. The system as demonstrated

A TurtleBot 4 Lite (Raspberry Pi 4, Create 3 base, RPLIDAR A1, OAK-D Lite
camera) driven over a 2.4 GHz Wi-Fi hotspot by a workstation running
ROS 2 Humble, slam_toolbox, Nav2, a YOLO-World open-vocabulary detector, a
local 7-billion-parameter language model (qwen2.5:7b via Ollama), a speech
front-end (faster-whisper `small.en`) and the VLA agent (`vla_agent_v28.py`,
64 numbered fixes). One command, `~/vla_demo.sh`, brings all of it up from
cold in about four minutes, with thirteen gated stages that refuse to start
a broken stack. The same agent code runs the Gazebo simulation unchanged;
the two hardware-specific behaviours are selected by environment variables
that default off.

## 2. What works on hardware

| capability | evidence | reason it works |
|---|---|---|
| **Spoken or typed command → LLM plan → detection → navigation → arrival**, the full chain | 11 Sep: three `go to the chair` drives reached the chair's stand-off (4 s, 6 s, 18 s); the third announced "Arrived at the chair". Spoken "what do you see" transcribed in 0.39 s and answered "I see: shelf (0.7 m)" / "person (2.7 m)" | every stage was verified separately first, and the agent treats the LLM as a planner whose output is checked by deterministic code (the "brain-guard" overrode a wrong verb sign and a wrong action class in these very runs) |
| **Open-vocabulary detection on live frames** | YOLO-World answers in ~90 ms; chair 0.53 at 2.3 m, person 0.70–0.74 (10–11 Sep) | GPU detector on the workstation; the camera runs colour-only at 29 Hz with ~50 ms frame latency |
| **Object ranging without depth** | laser 2.34 m / 2.30 m against the agent's "2 m" (10 Sep) | the 2-D LiDAR is a calibrated range sensor; fix #54 ranges each detection along its camera bearing |
| **Mapping and autonomous navigation** | Nav2 activates in ~30 s, 7/7 bonds, 0 errors on every launch once the link was fixed; goals of 1.9–3.9 m reached in 10–18 s; autonomous docking 31 s | stock slam_toolbox and Nav2, launched composed with a 30 s bond timeout and a delayed start |
| **Push-to-talk speech input** | Whisper on the GPU, 0.1–0.4 s per utterance; stop words always forwarded | the speech node is separate from the agent, so every safeguard in the agent applies to spoken text for free |
| **Manual override / emergency stop** | typed in the agent's terminal, robot stopped (10 Sep) | dead-man timer at 0.35 s plus a burst of zero-velocity commands from a `finally:` block |
| **Repeatable bring-up** | `vla_demo.sh` rehearsed twice through the pre-drive stages: 83 s and 77 s | each stage has a measurable pass condition and the script stops at the first failure |

## 3. What does not work, or works with limits — and the honest reason

**3.1 The navigation stack was flooding the robot's Wi-Fi link (found and
fixed 11 Sep; the finding is the most important engineering result of the
hardware phase).**
Every ROS 2 node publishes and subscribes to `/parameter_events` with
RELIABLE quality of service. When Nav2 configures, each of its nodes emits
~90 parameter events in a burst, addressed reliably to the eight nodes on the
robot. Over the wireless link the Fast DDS writer then entered a heartbeat
storm — a packet capture on the robot decoded to the RTPS level shows 87 %
of 7,400 packets/s being `HEARTBEAT` submessages, one per Pi node every
~1 ms, with valid acknowledgements from the readers being ignored while the
sending process's own receive socket sat 42 kB deep. The camera node on the
robot did the same in the other direction (9,700 packets/s toward the
workstation). The robot's CPU load rose to 7, its temperature to 84 °C, it
throttled, and camera frames stopped. This is what had been recorded on
8 September as "the Pi is thermally weak" and on 10 September as "camera
frames arrive 4.3 s late". The root cause is a Fast DDS reliable-protocol
pathology triggered by a burst of unacknowledged samples across a lossy
link; its trigger is timing-dependent, which is why one day was clean and
the next was not. *Fix:* the workstation's Nav2 processes now publish and
subscribe to `/pc/parameter_events` and `/pc/rosout` instead — a remap, no
code change — so the two machines' endpoints never match. Workstation-to-
robot traffic fell from 1,757 kB/s to 24 kB/s; the Pi runs at load 1.6 and
64 °C with the whole stack up. *What remains open:* why the writer ignores
acknowledgements once its socket backs up is characterised but not
explained; it deserves a report to the DDS vendor with the captures.

**3.2 Stereo depth is not used.** The OAK-D Lite ships colour-only; enabling
its depth pipeline costs the Pi +7 °C, halves the colour rate to 15 Hz and,
combined with the link problem above, delayed frames by seconds. The agent
had required a depth frame before it would run detection at all. On
hardware that gate is now bypassed (`VLA_RGB_ONLY=1`) and distance comes
from the LiDAR, which was the primary range source in any case. *Consequence:*
objects above or below the laser plane cannot be ranged.

**3.3 A 2-D laser sees through a chair.** With the chair placed 4–5 m away
the agent reported 7.1 m: at 15 cm height the beam passes between the legs
and ranges the wall behind. Near the object the nearest-cluster logic and
the projection fallback recover (the third drive arrived), but the reported
distance to thin-legged furniture can be wrong at range. A depth camera or a
higher laser would fix this; a person, box or shelf does not have the
problem.

**3.4 "Arrived" is not always announced.** Two of three drives stopped at
the correct stand-off point but the agent kept re-sending the same goal
every second instead of declaring arrival. The arrival test requires the
robot to be within 1.35 m of its belief of the object; the 1.30 m stand-off
plus Nav2's 0.25 m goal tolerance can leave it at 1.55 m. A one-line
tolerance flag (`VLA_ARRIVE_TOL`, hardware only) was applied on 12 Sep; the
drive itself was never affected.

**3.5 Speech recognition mishears accented English.** `small.en` turned
"go to the chair" into "go about that" and "stop looking for a chair" into
"stop looking for a gene". The design tolerates this in two ways: the LLM
still resolved the first to *navigate → chair*, and any phrase beginning
with *stop* is forwarded before any filtering, so a misheard stop is still a
stop. On 14 Sep the speech front-end was switched to `medium.en` with a
command-vocabulary prompt and a confidence gate (−0.75); the four demo phrases
then transcribed exactly in a controlled test, and the operator confirmed
spoken commands work in use. Audio from the earbuds clips at the source
regardless of PC gain; short button presses during robot motion remain the
main cause of misrecognition.

**3.6 The stop-verification warning can be a false alarm.** During a normal
transition between the agent's ROTATE and NAVIGATING states it printed "I
sent the stop but the robot is STILL MOVING — take manual override" while
Nav2 was legitimately driving. The message is conservative by design; the
true emergency stop (manual override) was verified separately.

**3.7 Things that are environment-specific and had to be discovered.**
A freshly started ROS client on this network can wait up to ~20 s before
its service replies arrive, so every robot-side tool is a long-lived client
with a 30 s settle. The ROS daemon goes "blind" if restarted from a terminal
without the discovery variables. Docking by the Dock action aborted once
(10 Sep) and succeeded on 11 Sep; the shutdown script verifies charging
current rather than trusting the action's result. The workstation's
hotspot is on 2.4 GHz; its card supports 5 GHz but the regulatory domain
is unset, which forbids running an access point there — a free improvement
left undone.

## 4. What this shows (viva framing)

1. **A language model can direct a real robot safely when it proposes and
   deterministic code disposes.** The guards intervened in the recorded runs
   and the robot still did the right thing.
2. **The hard problems on hardware were in the middleware and the link, not
   in the AI.** Detection, planning and the language model behaved as in
   simulation; the days lost were to DDS discovery, Wi-Fi flooding and a
   camera pipeline setting.
3. **Measure before changing.** The 4.3 s "camera lag" had two candidate
   causes with opposite fixes; the discriminating measurement (fresh camera:
   85 ms) showed neither — the link was the cause. The Pi "thermal problem"
   was likewise a symptom. Both were settled with packet captures, not
   guesses.
4. **Hardware-specific behaviour lives behind flags that default off**, so
   the simulation demo remained byte-for-byte the fallback throughout.
