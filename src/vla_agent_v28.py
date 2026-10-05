# ─────────────────────────────────────────────────────────────────
#  vla_agent.py  —  interactive VLA agent  (UPDATED v3)
#
#  NEW IN THIS VERSION (on top of #1–#10 below):
#    #11 CONVERSATION CONTEXT: the agent tells the brain what the last
#        action/target was, so "find a chair" ... "now go to IT" works.
#    #12 RGB–DEPTH SYNC (completes #8): the colour and depth frames are
#        now PAIRED by timestamp with message_filters'
#        ApproximateTimeSynchronizer, so YOLO and the depth projection
#        look at the SAME instant. In sim both streams tick together so
#        the old code got away with grabbing "latest of each"; on the
#        real OAK-D they drift, and a box from frame t matched with depth
#        from t±0.2 s while turning = a goal metres off target. If no
#        synced pair ever arrives, it falls back to the old behaviour
#        with a one-time warning (so nothing is worse than before).
#    #13 TARGET CONFIRMATION: "navigate" now needs the target seen in
#        SIGHT_CONFIRM (2) think-cycles before it commits and announces
#        — a single-frame YOLO hallucination no longer sends the robot
#        chasing a ghost. Set SIGHT_CONFIRM = 1 to disable.
#    #14 MISSION LOG: every run writes a timestamped log to ~/vla_logs/
#        (commands, brain decisions, goals, arrivals, timeouts, stucks,
#        dock events). Free, honest evidence for the thesis/viva.
#    #15 YOLO CLIENT HARDENING: one shared HTTP session (faster — reuses
#        the TCP connection instead of reconnecting every second) and
#        the "server down" error prints at most every 10 s instead of
#        spamming once per think cycle.
#    #17 SPATIAL MEMORY REUSE: 'find a chair' / 'count' now REMEMBER the
#        map position of what they saw, and 'go to the chair' starts by
#        driving to the remembered spot instead of searching from scratch.
#        Sightings are re-confirmed live on the way (honest behaviour:
#        if the object moved, the robot says it can't see it anymore).
#    #18 NO MORE SILENT SPIN-TO-TIMEOUT: if the target's projected map
#        point is in unmapped/occupied space (e.g. seen 14 m away, beyond
#        the SLAM'd area), the robot now drives to the FARTHEST reachable
#        point along the line to it (the edge of the known map) so SLAM
#        can extend the map, instead of looping on a rejected goal with a
#        stale spin command until the 60 s timeout. Repeated unreachable
#        goals (GOAL_FAIL_LIMIT) drop the sighting and resume a real scan,
#        and the timeout message now distinguishes "never saw it" from
#        "saw it but couldn't reach it".
#    #19 REACHING PAST OBSTACLES (three real causes, one symptom):
#        (a) PROGRESS-BASED TIMEOUT. The old flat 60 s deadline killed
#            Nav2 mid-journey: a 14 m drive at ~0.25 m/s needs ~60 s with
#            NO obstacles, so any detour guaranteed a false "couldn't
#            reach it". Now: a short deadline only while SEARCHING, and
#            once we're committed the robot gets unlimited time AS LONG AS
#            it keeps getting closer. It only quits after NO_PROGRESS_LIMIT
#            seconds of no net progress (check_stuck still catches frozen).
#        (b) FOOTPRINT-AWARE GOALS. is_goal_free() checked ONE cell, but
#            the robot is ~0.35 m wide — a goal 5 cm from a wall passed our
#            check and was then rejected by Nav2's inflation layer. Now we
#            check a disc of ROBOT_CLEARANCE around the goal, so our idea
#            of "free" matches Nav2's.
#        (c) RING OF APPROACH GOALS. The stand-off goal used to be the ONE
#            point on the robot->object line. A box near the chair blocked
#            that single point and the task died, even though the chair's
#            left/right/far side was wide open. Now we generate a RING of
#            candidate goals around the object, keep the cheapest reachable
#            one, and on a Nav2 rejection we try the NEXT candidate instead
#            of dropping the sighting. Nav2 still does the actual path
#            planning around obstacles — we just stop handing it illegal
#            goals and stop cutting it off early.
#    #20 ANNOTATED LIVE FEED: the live camera window now shows the YOLO
#        bounding boxes, class names, confidences and the DEPTH distance to
#        each object, plus a caption bar with the current mode/target and a
#        timestamp — i.e. a single screenshot that evidences the whole
#        perception pipeline (YOLO-World + OAK-D depth + agent state) for
#        the thesis/slides. The agent republishes an annotated stream on
#        /vla/annotated and points rqt_image_view at it. Detections are
#        REUSED from the think-loop when they're for the same frame, so
#        this does not double the GPU load. Small preview frames (the sim's
#        250x250) are upscaled so the text is legible in a screenshot.
#        Set ANNOTATED_FEED = False for the old raw feed.
#    #21 OCCLUSION-PROOF DEPTH + TELEPORT GATE (fixes the false
#        "Arrived at the person" while still metres away):
#        (a) Depth was read from a single 9x9 patch at the BOUNDING-BOX
#            CENTRE. When a target stands partly behind a shelf/rack, the
#            box centre can land on the OCCLUDER, so the target's map
#            position collapsed onto an object right next to the robot —
#            and the pose-based arrival check then honestly (and wrongly)
#            fired. Depth is now the median over a grid of samples across
#            the whole box (robust_box_depth): the occluder must cover
#            most of the box to steal the reading, instead of one pixel.
#        (b) TELEPORT GATE: once committed, a new sighting that moves the
#            target by > TELEPORT_JUMP metres is held as "suspicious"
#            until a SECOND look agrees with it (objects don't teleport;
#            depth noise does). One bad frame can no longer hijack the
#            goal or fake an arrival.
#        The annotated feed's distance labels use the same robust depth,
#        so what you screenshot is what the robot acted on.
#    #28 REPORT EVERY INSTANCE, WITH DISTANCE: "I can see 2 chairs" used
#        to be followed by a single distance, because locate/count picked
#        ONE instance (nearest, or the qualifier) and reported only that.
#        Now locate_candidates() ranges EVERY visible instance and the
#        answer lists them all ("one about 6.3 m away and another about
#        12.4 m away"). 'count' includes the distances, so the follow-up
#        question isn't needed, and 'describe' ("what do you see") gives a
#        distance per object. Duplicate boxes stacked on one physical
#        object are merged (REPORT_MERGE_RADIUS) so counts stay honest,
#        and every reported number comes from the SAME robust depth the
#        navigation goals use — a report that disagreed with the goal
#        would be useless as thesis evidence.
#    #29 CREATE 3 BACKUP-LIMIT RATCHET: the Create 3 has cliff sensors
#        only at the FRONT, so with the factory safety setting the
#        firmware refuses to reverse more than a few centimetres, and
#        "move back 1 m" silently stalled. iRobot's documentation gives
#        the reset condition: driving FORWARD re-enables reverse motion.
#        So a reverse move is now a RATCHET — back up until the firmware
#        cuts us off (detected as no net progress for BACKUP_STALL_TIME),
#        creep BACKUP_NUDGE_DIST forward to clear the limit, reverse
#        again, repeat until the NET displacement is what was asked for.
#        Progress is measured as SIGNED distance along the starting
#        heading, so the forward shuffles are accounted for correctly.
#        If the robot is blocked front AND rear it says so instead of
#        grinding, and MAX_BACKUP_NUDGES caps the loop. This keeps the
#        rear cliff protection ARMED — the alternative (safety_override =
#        backup_only) simply switches it off.
#    #25 TYPO-TOLERANT CONTROL WORDS: "cancle" used to sail past the
#        stop-word check (exact match only) and reach the LLM, which
#        answered with a reject. Control words are the ONE place a typo
#        is dangerous — every other command is handled by an LLM, and
#        LLMs read through misspellings fine ("forget the last cahir
#        postion" already worked). Now cancel/stop/halt/abort are matched
#        with a similarity score, and the correction is printed so the
#        operator sees what was understood. 'quit'/'exit' stay EXACT on
#        purpose — auto-correcting into a program exit is destructive and
#        unrecoverable, while an over-eager cancel just stops the robot.
#    #26 BARE "yes"/"no" WITH NOTHING PENDING: a lone confirmation word
#        is meaningless unless a question is open. It used to be sent to
#        the brain, which had no question to attach it to and invented a
#        task out of stale conversation context ("yes" -> "Checking if
#        the shelf is still there"). Now it is intercepted and answered
#        honestly. A "yes" that ANSWERS a real pending question still
#        works exactly as before (and now tolerates typos too).
#    #27 CONVERSATION CONTEXT EXPIRES: the previous action/target handed
#        to the brain for pronoun resolution ("go to IT") is the raw
#        material stale commands are built from — it is the reason a
#        long-forgotten "shelf" resurfaced. Context older than
#        CONTEXT_TTL seconds is no longer sent; "go to it" straight after
#        a locate is unaffected.
#    #22 DOCK NO-DETECT ZONE (kills the "dock = chair" cascade): the
#        robot RECORDS the dock's map position (the moment dock_status
#        says it's docked, or when a dock/undock action succeeds) and
#        then IGNORES any detection whose projected position lands within
#        DOCK_EXCLUDE_RADIUS of it — in live targeting, in locate, and in
#        spatial memory. Existing memory entries sitting on the dock are
#        purged the moment the dock position becomes known. One YOLO
#        mislabel of the dock used to poison everything downstream:
#        memory-seeded navigation to a "0.5 m chair", Nav2 goals inside
#        the dock's costmap footprint ("the way looks blocked"), and a
#        false "Arrived at the chair". The vocabulary fix (yolo_server
#        #19) treats the cause; this geo-fence guarantees the symptom
#        can't return. Asking about the dock itself still works (targets
#        containing "dock" bypass the filter; plain "dock" remains a
#        deterministic action anyway).
#    #23 LIVE-FIRST MEMORY (ends the "I remember..." spam): "go to X" now
#        checks the CAMERA first — if X is visible right now, it searches
#        fresh and locks on within a couple of think-cycles, with no
#        memory monologue and no risk of driving to a stale spot while
#        the real object sits in plain view. Memory seeding only kicks in
#        when the target is NOT currently visible (its actual job:
#        returning to things seen earlier). Also: "find a chair" ...
#        "go to it" now recalls the chair that was JUST located, not
#        whichever remembered chair happens to be nearest (the log showed
#        a locate at 13.3 m followed by a recall of 0.9 m).
#    #24 "forget" ACTION: "forget the chair" erases every remembered
#        chair position; "forget everything / clear your memory" wipes
#        the whole spatial memory. Paired with llm_brain #17 so the
#        command can never again be misrouted to navigate.
#    #16 CORRECT DOCK QoS + RESULT CHECK: /dock_status is subscribed
#        with sensor-data (best-effort) QoS — the Create 3 publishes
#        best-effort, and a reliable subscription silently receives
#        NOTHING. Dock/undock results are also checked for real success
#        instead of assumed.
#
#  STILL HERE FROM BEFORE:
#    #1  "cancel"/"stop" preempt everything, even with politeness.
#    #3  Nearest instance by default (or farthest/leftmost/rightmost).
#    #4  Patrol coverage memory + forward-bias frontier scoring.
#    #5  Nav2 paths AROUND obstacles; goals validated against the map.
#    #6  Give-up-and-report timeout for "go to X".
#    #7  "find/look for X" locates and REPORTS only; "go to X" drives.
#    #8  TF at the frame's capture time (now with truly paired frames).
#    #9  Precise relative motion ("move forward 2 m", "turn right 30°").
#    #10 Managed dock/undock + auto-undock before any motion.
#
#
#  ── NEW IN v9 (hardware-implementation phase) ────────────────────
#    #31 INCREMENTAL ("HOP") GOALS FOR OFF-MAP TARGETS. "go to the chair"
#        failed whenever the chair was seen BEYOND the SLAM'd area. Cause:
#        every goal we can send has to survive is_goal_free(), which demands
#        a disc of KNOWN-FREE cells. Around an off-map object every cell is
#        UNKNOWN (-1), so approach_candidates() returned [] and
#        farthest_free_along() could only reach the last known-free cell —
#        i.e. the robot stopped ON the frontier and never crossed it. After
#        GOAL_FAIL_LIMIT cycles the sighting was dropped and the robot
#        rescanned. It looked like "it refuses to go there"; it was really
#        "every goal it is allowed to send is behind it".
#        Now navigation runs in TWO regimes, chosen automatically:
#          MAPPED   -> unchanged. The ring of approach goals exists, so we
#                      send ONE full Nav2 goal and let Nav2 plan the path.
#          UNMAPPED -> HOP MODE. We send a SHORT Nav2 goal (STEP_GOAL_DIST,
#                      default 1.5 m) along the bearing to the object, which
#                      is allowed to end in unknown space (unknown is not
#                      occupied — Nav2's planner already accepts it, and the
#                      LiDAR local costmap still guards the robot). On
#                      arrival SLAM has extended the map, we re-range the
#                      object and hop again. As soon as the ring becomes
#                      reachable we switch back to a single full goal.
#        A hop is STICKY (kept until reached, ~35 deg off-bearing, blocked
#        or timed out) so we do not preempt Nav2 every second — the same
#        stutter #30a cured for full goals. Blocked hops sidestep through a
#        fan of angles and then shorten before giving up.
#        EXPLORE/PATROL IS UNTOUCHED: hop mode lives entirely inside
#        navigate_think(); explore_think() still sends full frontier goals.
#    #32 LIVE FEED THAT DOESN'T FREEZE. Four independent causes, all real:
#        (a) STALE PAIR — the big one. current_pair() returned self.pair
#            forever once set, and self.pair is only refreshed when the
#            ApproximateTimeSynchronizer MATCHES a frame. On the real OAK-D
#            the two streams drift and matching stops for seconds at a time;
#            the agent then republished the SAME frame indefinitely (feed
#            "freezes") and — far worse — kept NAVIGATING on a stale image.
#            A pair older than PAIR_STALE_S is now discarded and the raw
#            latest-of-each fallback takes over, with a logged warning.
#        (b) HEAD-OF-LINE BLOCKING — think(), publish_cmd() and
#            annotate_think() shared one mutually-exclusive callback group
#            on a single-threaded spin. A slow YOLO reply (timeout was 15 s!)
#            froze the feed AND the 10 Hz velocity publisher. Each timer now
#            has its own callback group and main() uses a MultiThreadedExecutor.
#            The YOLO timeout is also cut to (2 s connect, 5 s read).
#        (c) DOUBLE INFERENCE — annotate_think() ran its OWN YOLO pass
#            whenever the frame stamp differed from the think loop's, which
#            is most of the time. The feed now never calls YOLO: it reuses
#            the cached detections while they are fresher than ANNOT_DET_TTL
#            and prints their age in the caption bar (honest, and it frees
#            the GPU). Because the feed no longer waits on inference it can
#            run at 5 Hz instead of 2 Hz.
#        (d) TRANSPORT — a 750x750 BGR frame is ~1.7 MB. Queued 10 deep over
#            the default QoS this jams DDS. History is now KEEP_LAST depth 1
#            (drop old frames, never build a backlog) and a JPEG stream is
#            published alongside on /vla/annotated/compressed (~50 kB) —
#            that is the one to use over Wi-Fi and in the GUI.
#        Plus a watchdog: a dead rqt_image_view is detected and reported,
#        and a camera that stops delivering frames is announced once.
#    #33 REMOTE COMMAND BRIDGE (foundation for voice + GUI). The agent no
#        longer only listens to the keyboard:
#          /vla/command  (std_msgs/String, in)  - a command from anywhere
#          /vla/reply    (std_msgs/String, out) - everything the agent says
#          /vla/status   (std_msgs/String, out) - JSON state at 2 Hz
#        Commands from every source go through ONE queue served by a worker
#        thread, so a slow LLM call can never block the ROS executor, and
#        the terminal keeps working exactly as before. cancel/stop is still
#        handled instantly at intake so it can preempt a queued command.
#        voice_command.py and vla_gui.py are pure clients of these topics —
#        the agent has no idea whether a command was typed or spoken.
#
#  Companion file: the updated llm_brain.py (adds context + validation).
#  Use them together.
# ─────────────────────────────────────────────────────────────────
#
# PHASE 2: this file is now a THIN SHIM. The implementation lives in the
# vla_agent package next to it (see vla_agent/__init__.py for the map).
# It is kept because every launcher refers to it by name:
#   scripts/vla_demo.sh, tools/agent_xterm.sh, tools/restart_agent_hw.sh,
#   scripts/run_agent_sim.sh, scripts/vla_sim.sh, scripts/vla_robot.sh
# and because `python3 src/vla_agent_v28.py` must keep working.
#
# The three underscore-prefixed names are imported explicitly: `import *`
# deliberately skips them, and leaving them out imports cleanly and then
# fails at runtime.

from vla_agent import *                                      # noqa: F401,F403
from vla_agent import print                                  # noqa: A001 - fix #63
from vla_agent.core import _real_print, _off_front, _DecodedFilter   # noqa: F401
from vla_agent.agent import VLAAgent                         # noqa: F401
from vla_agent.main import main                              # noqa: F401


if __name__ == "__main__":
    main()
