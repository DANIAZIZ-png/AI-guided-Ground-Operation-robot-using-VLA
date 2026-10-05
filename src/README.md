# src/

The five live programs. On the host these sit flat in `$HOME`.

| File | Lines of responsibility |
|---|---|
| `vla_agent_v28.py` | the state machine and **every safety guard**. This is the core of the project. |
| `llm_brain.py` | builds the prompt, calls Ollama `qwen2.5:7b`, parses the reply into a plan |
| `yolo_server.py` | YOLOv8s-World + CLIP behind a socket; open-vocabulary, so the target class comes from the sentence |
| `voice_command.py` | faster-whisper `medium.en`, push-to-talk, publishes on `/vla/command` |
| `vla_gui_v2.py` | operator UI — conversation, annotated camera feed, push-to-talk button |

## These files are shared by the simulation and the hardware

That is the single most important thing to know before editing any of them. The
simulation demo is the fallback if the hardware fails, so **every
hardware-specific behaviour sits behind an environment variable that defaults to
the simulation's behaviour.** The hardware launcher `tools/agent_xterm.sh` sets
them; nothing else does.

| Flag | Default | Hardware | Why |
|---|---|---|---|
| `VLA_RGB_ONLY` | off | `1` | the camera is colour-only; bypasses the agent's depth gate |
| `VLA_ARRIVE_TOL` | 0.35 | `0.50` | must exceed the largest stand-off + 0.25 − stop |
| `VLA_MIN_FRONT_CLEAR` | — | `0.35` m | front-clearance guard |
| `VLA_STOP_DISTANCE` | — | `0.60` | 0.45 failed: Nav2 rejects goals inside its 0.45 m inflation |
| `VLA_STANDOFFS` | — | `0.60,0.80` | tried in order |
| `VLA_ARRIVE_IF_SEEN_M` | — | `1.2` | target in frame within 1.2 m counts as arrived, any class |
| `VLA_ANNOT_PERIOD` | 0.2 | `0.033` | 30 Hz annotated stream for the GUI |
| `VLA_CAM_FPS` / `_JPEG` / `_PX` | — | 30 / 75 / 512 | set by `tools/oakd_bandwidth.py` |

Restart only the agent with `tools/restart_agent_hw.sh`.

A resolution change invalidates the agent's cached calibration, so
`scripts/vla_demo.sh` applies the camera settings at stage 10, **before** the
agent starts at stage 11.

## The design rule

The language model proposes; deterministic Python disposes. `llm_brain.py` can
only return a plan — it never actuates anything. `vla_agent_v28.py` decides
whether that plan is allowed, and the guards above are what makes it safe to let
a 7-billion-parameter model direct a 20 kg robot. In recorded runs the guards
intervened and the robot still did the right thing, which is the result worth
defending in the viva.

## Why the project is named after OpenVLA but does not use it

The OpenVLA baseline was evaluated and dropped — it is in `../archive/openvla/`.
The shipped system is a local LLM for reasoning plus an open-vocabulary detector
for grounding, which ran on the available hardware where OpenVLA did not.
