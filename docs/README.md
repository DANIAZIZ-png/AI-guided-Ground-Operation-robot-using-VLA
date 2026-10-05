# docs/

Read in this order.

| Document | What it is |
|---|---|
| `../CLAUDE.md` | **start here.** The operating manual for the physical machine: every trap, in the order they were found. Kept at the repository root because it is the single most useful file. |
| `PROJECT_HANDOUT_v5.md` | the authoritative project record — architecture, decisions, results. §17 is the most recent section. |
| `REPORT_SECTION_HARDWARE.md` | the hardware chapter as written for the report and viva. |
| `HANDOVER_2026-09-10.md` | state of the system on 10 Sep. Still correct on the traps and the tools, but **superseded on three points** by the 11 Sep changelog — see below. |
| `HANDOVER_TO_OPUS.md` | 8 Sep. Still correct on the discovery server, the blind daemon and the shared-memory rules. |
| `RESUME_NEXT_SESSION.md` | working notes picking up from 8 Sep. |
| `PROJECT_HANDOFF_2.md` | the earliest handoff, June 2026. Mostly historical. |
| `changelogs/` | the three hardware-session changelogs. |

## Precedence

These documents were written over three weeks of debugging, and **later ones
correct earlier ones.** Where they disagree, the newest wins:

```
CHANGELOG_2026-09-11.md   ← newest, wins
HANDOVER_2026-09-10.md
CHANGELOG_2026-09-10.md
CHANGELOG_2026-09-08.md
HANDOVER_TO_OPUS.md
PROJECT_HANDOFF_2.md      ← oldest
```

Three specific reversals, because they are the ones most likely to mislead:

1. **The camera is colour-only.** Earlier documents describe an RGBD switch.
   Depth is no longer used at all; the agent's depth gate is bypassed with
   `VLA_RGB_ONLY=1` and ranging is done with the LiDAR.
2. **The "4.3 s camera lag" was not a clock problem.** It was the depth pipeline
   plus a Wi-Fi flood. Fresh colour frames arrive in 47–85 ms.
3. **The Raspberry Pi was never thermally weak.** Our own Nav2, launched without
   the `/parameter_events` and `/rosout` remaps, was flooding its Wi-Fi link
   with a Fast DDS heartbeat storm. The heat and the throttling were symptoms.

Bring-up is also no longer the manual sequence in `HANDOVER_2026-09-10.md` §4 —
it is one script, `scripts/vla_demo.sh`.

## Still open

Recorded honestly rather than resolved: whether starting SLAM triggers the
Create 3 base going silent, or whether the base fails on its own. The silence is
a known iRobot firmware issue (`turtlebot4` #554) with no software fix — only a
physical power-cycle — and it happened twice at healthy battery levels. Not
assumed either way.
