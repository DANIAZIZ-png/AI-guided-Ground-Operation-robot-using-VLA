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
#  Companion file: the updated llm_brain.py (adds context + validation).
#  Use them together.
# ─────────────────────────────────────────────────────────────────
AGENT_VERSION = ("vla_agent v8  (#22 dock no-detect zone, #23 live-first memory, "
                 "#24 forget, #25 typo-tolerant control words, #26 bare yes/no, "
                 "#27 context expiry, #28 all-instance distances, "
                 "#29 Create 3 backup ratchet, #30 sticky goals + range-scaled "
                 "gate + ring-safe blacklist)")

import base64
import datetime
import difflib
import hashlib
import math
import os
import subprocess
import threading
import time

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point
import message_filters                      # #12: RGB-depth pairing

from llm_brain import decide

# Create 3 dock/undock are optional — guard the import so the agent still
# runs in setups that don't have irobot_create_msgs.
try:
    from irobot_create_msgs.action import Dock, Undock
    from irobot_create_msgs.msg import DockStatus
    HAVE_CREATE = True
except Exception:
    Dock = Undock = DockStatus = None
    HAVE_CREATE = False

YOLO_URL    = "http://127.0.0.1:5001/detect"
RGB_TOPIC   = "/oakd/rgb/preview/image_raw"
DEPTH_TOPIC = "/oakd/rgb/preview/depth"
INFO_TOPIC  = "/oakd/rgb/preview/camera_info"
SCAN_TOPIC  = "/scan"
ODOM_TOPIC  = "/odom"
CMD_TOPIC   = "/cmd_vel"
MAP_TOPIC   = "/map"
ANNOT_TOPIC = "/vla/annotated"          # #20: RGB + YOLO boxes, for the live window
MAP_FRAME   = "map"
ROBOT_FRAME = "base_link"

STOP_DISTANCE     = 0.6
ARRIVE_TOL        = 0.35
SEARCH_TURN_SPEED = 0.5
ADVANCE_SPEED     = 0.20
ADVANCE_DISTANCE  = 1.5
OBSTACLE_STOP     = 0.6
ENABLE_ADVANCE    = True

GOAL_REISSUE        = 0.5    # re-send a nav goal only if the target moved > this (m)
NEW_INSTANCE_RADIUS = 0.8    # a detection this far from known ones counts as a NEW object (m)
# #28: when REPORTING what's in frame, two projected positions closer than
# this are the same physical object seen as two boxes (YOLO-World stacks
# boxes on plain shapes). Deliberately smaller than NEW_INSTANCE_RADIUS so
# two genuinely adjacent chairs are still reported separately.
REPORT_MERGE_RADIUS = 0.40
FIND_TIMEOUT        = 45.0   # give up "find another" after this long (s)
LOCATE_TIMEOUT      = 25.0   # #7: give up "find/locate X" (report-only) after this long (s)

# #19a: "go to X" is no longer on a flat stopwatch. While we have NOT seen
# the target we give up after SEARCH_TIMEOUT. Once we're committed to a
# target the robot may take as long as it needs, provided it keeps closing
# the gap: we only quit after NO_PROGRESS_LIMIT seconds without getting at
# least PROGRESS_EPS metres nearer. (A long detour around an obstacle makes
# no progress for a while, hence the generous 60 s.)
SEARCH_TIMEOUT      = 45.0   # never saw it -> give up (s)
NO_PROGRESS_LIMIT   = 60.0   # committed but not closing the gap -> give up (s)
PROGRESS_EPS        = 0.25   # this much closer counts as real progress (m)

# #19b: the robot is ~0.35 m wide, so a goal needs clear space AROUND it,
# not just one free cell. Keep this a bit under Nav2's inflation_radius.
ROBOT_CLEARANCE     = 0.25   # m of required free space around a goal point

# #19c: candidate approach directions around the object, in degrees, offset
# from "the side the robot is already on" (0 = straight-line approach).
APPROACH_RING       = (0, 25, -25, 50, -50, 75, -75, 100, -100,
                       130, -130, 160, -160, 180)
# Stand-off distances to try. Both are < STOP_DISTANCE + ARRIVE_TOL so that
# reaching any of them counts as an arrival.
APPROACH_STANDOFFS  = (0.60, 0.90)
# #30c: was 0.40, which is WIDER than the spacing between adjacent ring
# spots (~0.26 m at a 0.6 m stand-off), so ONE Nav2 refusal also wiped out
# its neighbours. In open ground that cascaded into a false "I can't find a
# reachable path". 0.22 m rules out only the refused spot itself.
GOAL_TRIED_RADIUS   = 0.22

# #30a STICKY GOALS. The goal block below re-ranks all 28 ring candidates
# EVERY think-cycle and re-sends whenever the winner shifts more than
# GOAL_REISSUE. Because the believed object position jitters a little each
# frame (depth noise) and the robot is moving, the winner flips between
# near-tied spots — so Nav2 was being PREEMPTED roughly once a second. It
# cancels, replans, starts accelerating, gets preempted again: the goal
# marker dances in RViz and the robot stutters or looks stuck. Fix: once a
# goal is accepted, KEEP it while it is still a sane approach, and only
# re-choose when it stops being valid or the object genuinely moved.
GOAL_STICK_MIN      = 0.35   # keep the goal while it sits between MIN and MAX
GOAL_STICK_MAX      = 1.25   #   metres of the (jittering) object position
OBJ_DRIFT_REISSUE   = 0.80   # re-choose only if the object moved this far (m)
                             #   since the current goal was picked

# #21b teleport gate: a committed target's position may move by at most this
# much per sighting without confirmation. A bigger jump needs a SECOND look
# that lands within JUMP_AGREE of the first before it is believed.
TELEPORT_JUMP       = 3.0    # m — larger jumps are suspicious
JUMP_AGREE          = 1.0    # m — MINIMUM agreement radius for a confirming look
# #30b: a FIXED 1.0 m agreement was impossible to satisfy at long range.
# Stereo depth error grows with distance, so two honest looks at a chair
# 10 m away routinely disagree by >1 m. The gate therefore never confirmed
# a legitimate RE-TARGET (the closest chair entering view once the robot had
# already committed to a far one) and the robot drove to the wrong chair.
# Agreement now scales with range: max(JUMP_AGREE, JUMP_AGREE_FRAC * d).
JUMP_AGREE_FRAC     = 0.25
JUMP_PENDING_TTL    = 6.0    # #30b: a pending jump older than this is stale
# #21a robust depth: sample grid density across the bounding box
BOX_DEPTH_GRID      = 5      # 5x5 = 25 samples over the central box region

# #13: how many think-cycles must SEE the target before we commit to it.
# 2 kills single-frame hallucinations at the cost of ~1 s. Set 1 to disable.
# (The scan now PAUSES between confirmation looks, so the camera doesn't
# rotate off the target while it's trying to confirm it.)
SIGHT_CONFIRM = 2

# #17: "go to X" seeds from remembered instance positions (from find/count/
# previous navigations) instead of always searching from scratch.
# (#23: seeding now happens ONLY when the target is not currently visible.)
MEMORY_SEED = True

# #22: detections projected within this radius of the recorded dock position
# are treated as the DOCK, whatever YOLO called them. 0.8 m covers the dock
# plus the ~0.5 m the Create 3 backs off during undock (the recorded point
# can be either the docked pose or the just-undocked pose).
DOCK_EXCLUDE_RADIUS = 0.8

# #18/#19c: after this many consecutive failed approach goals for the same
# sighting, forget that sighting and go back to scanning. With the approach
# RING (#19c) each failure now costs only one angle, so allow a few more.
GOAL_FAIL_LIMIT = 5

# #12: RGB-depth pairing tolerance. In sim both streams share a clock so
# 0.1 s is generous; on the real OAK-D keep <= 0.1 s.
SYNC_SLOP_S    = 0.1
SYNC_WARN_S    = 6.0        # warn if raw frames flow but no synced pair after this long

# #15: don't spam "server down" more than once per this many seconds
YOLO_ERR_PERIOD = 10.0

# #14: mission logs (thesis evidence) live here, one file per run
LOG_DIR = os.path.expanduser("~/vla_logs")

# #9 relative move
BLOCK_SIZE   = 1.0          # metres per "block" (brain already converts; here for reference)
MOVE_SPEED   = 0.18         # m/s for relative translate
ROTATE_SPEED = 0.5          # rad/s for relative turn

# ── #29: CREATE 3 BACKUP-LIMIT RATCHET ──────────────────────────────
# The Create 3's cliff sensors are all at the FRONT, so with the factory
# safety setting (safety_override = "none") the firmware refuses to reverse
# more than a few centimetres — it can't rule out a drop behind it. iRobot's
# own documentation states the reset condition explicitly: driving FORWARD
# re-enables backward motion. So "move back 1 m" becomes a ratchet: reverse
# until the firmware stops us, creep forward a little to clear the limit,
# reverse again, repeat until the NET displacement is 1 m.
#   Why not just set safety_override = backup_only / full?  You can, and it
#   is the faster fix (ros2 param set /motion_control safety_override
#   backup_only) — but it turns OFF rear cliff protection. The ratchet keeps
#   the safety system armed and still delivers the commanded motion, which is
#   the defensible choice for an indoor ISR platform working near stairs,
#   loading bays and ramps. Both paths are available; the ratchet is default.
BACKUP_STALL_EPS  = 0.02    # m — net reverse progress that still counts as moving
BACKUP_STALL_TIME = 2.0     # s — no reverse progress for this long = limit hit
BACKUP_NUDGE_DIST = 0.15    # m — creep forward this far to clear the limit
# Net reverse gained per cycle = (firmware allowance) - BACKUP_NUDGE_DIST.
# With a ~0.30 m allowance and a 0.15 m nudge that's 0.15 m per shuffle, so
# 1 m of reverse needs ~6 shuffles. 25 therefore covers ~3.7 m — beyond that
# it is faster and safer to turn around and drive forward.
#   TUNING: a SMALLER nudge gains more per cycle but may not fully clear the
#   limit; measure the real allowance on the robot (watch the printed
#   "reverse limit reached at X m") and set the nudge to about half of it.
MAX_BACKUP_NUDGES = 25      # give up after this many shuffles (safety valve)

# #4 patrol coverage
VISITED_RADIUS = 1.0        # frontiers within this of a covered goal are skipped (m)
TURN_WEIGHT    = 1.5        # how strongly to prefer frontiers ahead (m of cost per rad of turn)

BIN_SIZE         = 0.5
MIN_FRONTIER     = 4
BLACKLIST_RADIUS = 0.8
SAVE_MAP_PATH    = os.path.expanduser("~/warehouse_map")

STUCK_DIST = 0.10      # moved less than this...
STUCK_TIME = 15.0      # ...for this many seconds while driving = stuck
MAX_STUCKS = 3         # give up exploring after this many stucks in a row

THINK_PERIOD = 1.0
PUB_PERIOD   = 0.1

# #20 annotated feed
ANNOTATED_FEED  = True     # False -> open the plain raw camera topic instead
ANNOT_PERIOD    = 0.5      # s between annotated frames (only while the feed is open)
ANNOT_MIN_WIDTH = 750      # upscale narrow preview frames to at least this wide (px)

QUIT_WORDS = ("quit", "exit")
STOP_WORDS = ("cancel", "stop", "halt", "abort")
YES_WORDS  = ("yes", "y", "yeah", "yep", "sure", "ok", "okay", "affirmative")
# #26: the other half of a yes/no answer — meaningless with no question open
NO_WORDS   = ("no", "nope", "nah", "negative")
POLITE_PREFIXES = ("please ", "can you ", "could you ", "would you ", "kindly ", "now ", "just ")

# #25: how similar a typed word must be to a control word before we act on it.
# 0.82 accepts "cancle"->"cancel" (0.83) while rejecting "about"->"abort"
# (0.80) — "about face" is a real turn command and must NOT cancel the task.
FUZZY_CONTROL = 0.82
FUZZY_MIN_LEN = 4          # words shorter than this are never fuzzy-matched
# #25: real commands that happen to look like control words — never corrected.
NEVER_CONTROL = ("about", "start", "scan", "stand", "stay", "back", "count")

# #27: conversation context (last action/target) older than this is stale and
# is no longer offered to the brain for pronoun resolution. Long enough for
# "find a chair" ... "now go to it"; short enough that a target from ten
# commands ago can't be resurrected into a nonsense task.
CONTEXT_TTL = 180.0        # seconds

# Actions that physically move the robot (used for auto-undock).
MOVE_ACTIONS = ("navigate", "find_another", "explore", "patrol", "move", "locate")


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def strip_politeness(low):
    """Remove leading 'please ', 'can you ', etc. so short commands like
    'please stop' / 'can you dock' are recognised."""
    changed = True
    while changed:
        changed = False
        for p in POLITE_PREFIXES:
            if low.startswith(p):
                low = low[len(p):]
                changed = True
    return low.strip()


# ── #25: typo-tolerant matching for the few words we handle WITHOUT the LLM ──
def fuzzy_word(word, options, cutoff=FUZZY_CONTROL):
    """Return the option the operator most likely MEANT, or None.

    Only used for control words (cancel/stop, yes/no). Everything else goes
    to the LLM, which reads through typos on its own. Deliberately cautious:
    very short words and known-good commands are never corrected, because a
    false 'cancel' is annoying and a false 'quit' would be unrecoverable."""
    if not word or len(word) < FUZZY_MIN_LEN:
        return None
    if word in NEVER_CONTROL:
        return None
    if word in options:
        return word
    m = difflib.get_close_matches(word, [o for o in options if len(o) >= FUZZY_MIN_LEN],
                                  n=1, cutoff=cutoff)
    return m[0] if m else None


def first_word(low):
    parts = low.split()
    return parts[0].strip(".,!?;:") if parts else ""


def is_yes(low):
    """#25: 'yes' / 'yeah' / 'ok' — and near-misses like 'yse', 'yeh'."""
    w = first_word(strip_politeness(low))
    return w in YES_WORDS or fuzzy_word(w, YES_WORDS) is not None


class MissionLog:
    """#14: append-only, timestamped run log — evidence for the thesis.
    Every line: '2026-07-08 14:03:22.512 [TAG] message'. Never crashes
    the agent: if the disk misbehaves, logging silently stops."""

    def __init__(self):
        self.f = None
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            name = datetime.datetime.now().strftime("vla_run_%Y%m%d_%H%M%S.log")
            self.path = os.path.join(LOG_DIR, name)
            self.f = open(self.path, "a", buffering=1)   # line-buffered
            self.log("RUN", f"{AGENT_VERSION}")
            self.log("RUN", f"agent started (SIGHT_CONFIRM={SIGHT_CONFIRM}, "
                            f"SEARCH_TIMEOUT={SEARCH_TIMEOUT}s, "
                            f"NO_PROGRESS_LIMIT={NO_PROGRESS_LIMIT}s, "
                            f"ROBOT_CLEARANCE={ROBOT_CLEARANCE}m)")
        except Exception:
            self.f = None

    def log(self, tag, msg):
        if self.f is None:
            return
        try:
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.f.write(f"{ts} [{tag}] {msg}\n")
        except Exception:
            self.f = None                                # disk problem -> stop logging


class VLAAgent(Node):
    def __init__(self):
        super().__init__("vla_agent")
        self.bridge = CvBridge()
        self.K = self.cam_frame = None

        # #12: the primary camera state is a timestamp-matched (rgb, depth)
        # PAIR. The raw single-topic copies below are only a fallback.
        self.pair = None                 # (rgb_msg, depth_msg), same instant
        self.rgb_raw = None
        self.depth_raw = None
        self.sync_seen = False           # ever received a matched pair?
        self.warned_no_sync = False
        self.start_t = time.monotonic()

        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.front_min = float("inf")

        self.mode = None
        self.target = None
        self.target_qualifier = None     # #3: nearest / farthest / leftmost / rightmost
        self.task_id = 0
        self.goal_handle = None
        self.cmd = Twist()
        self.queue = []                  # remaining steps in a multi-step command

        self.search_state = None
        self.turn_accum = 0.0
        self.last_yaw = None
        self.advance_start = None
        self.navigate_start = None       # #6: when the current "go to X" started

        # navigation arrival / tracking state
        self.last_obj_xy = None          # last seen map position of the current target
        self.target_announced = False    # have we already said "found it"?
        self.nav_goal_xy = None          # the goal we last sent for this target
        self.sight_count = 0             # #13: consecutive-ish sightings before committing
        self.seen_live = False           # #17: seen with the CAMERA during THIS task (not just memory)
        self.goal_fail_count = 0         # #18: consecutive unreachable/rejected goals
        self.best_dist = None            # #19a: closest we've ever been to the target
        self.last_progress_t = None      # #19a: when best_dist last improved
        self.goal_tried = []             # #19c: approach goals Nav2 refused this task
        self.jump_pending = None         # #21b: suspicious sighting awaiting confirmation
        self.jump_pending_t = 0.0        # #30b: when it was raised (for expiry)
        self.goal_obj_xy = None          # #30a: object position when goal was picked

        # spatial instance memory (for counting / "find another")
        self.seen_instances = {}         # class name -> list of distinct (x, y) world positions
        self.find_baseline = 0
        self.find_start = 0.0
        self.locate_start = None         # #7
        self.dock_xy = None              # #22: dock position in the map frame, once known
        self.last_located = None         # #23: (target, (x, y)) of the most recent locate hit

        # #11: what we last acted on, for pronoun resolution in the brain
        self.ctx_last_action = None
        self.ctx_last_target = None
        self.ctx_time = 0.0              # #27: when that context was set

        # #9 relative move
        self.move_spec = None
        self.move_ref_yaw = None
        self.move_turned = 0.0
        self.move_ref_pos = None
        self.move_moved = 0.0
        # #29: backward-ratchet state
        self.move_axis = None            # unit heading vector when the translate started
        self.backup_best = 0.0           # best net reverse distance achieved so far
        self.backup_stall_t = None       # when reverse progress last improved
        self.backup_nudges = 0           # how many forward shuffles we've used
        self.nudge_from = None           # position where the current shuffle began

        # pending yes/no + whether this command already has a feed step
        self.pending = None
        self.has_feed_step = False

        self.map = None
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.cur_goal = (0.0, 0.0)
        self.blacklist = []
        self.visited_goals = []          # #4: areas we've already covered while patrolling
        self.explore_start_t = None
        self.warned_no_map = False
        self.warned_no_frontier = False

        # stuck detection
        self.last_pos = None
        self.last_move_t = None
        self.consecutive_stucks = 0

        # #10 dock state
        self.is_docked = False
        self.dock_known = False
        self.dock_goal_handle = None

        self.feed_proc = None
        self.shutdown = False

        # #14 mission log + #15 pooled HTTP session for YOLO
        self.mlog = MissionLog()
        self.http = requests.Session()
        self.last_yolo_err_t = 0.0
        # #20: cache of the most recent YOLO result, keyed by the frame's
        # timestamp, so the annotated feed can REUSE the think-loop's
        # inference instead of paying for a second pass on the same image.
        self.last_det = None             # (stamp_key, detections)

        # #12: synchronized RGB + depth (primary path)
        rgb_sub   = message_filters.Subscriber(self, Image, RGB_TOPIC)
        depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=SYNC_SLOP_S)
        self.sync.registerCallback(self.on_rgb_depth)

        # raw copies (fallback only — used if pairing never succeeds)
        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, 1)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        # #20: annotated stream for the live window. The publisher exists from
        # startup so rqt_image_view can always find the topic; frames are only
        # produced while the feed is actually open.
        self.annot_pub = self.create_publisher(Image, ANNOT_TOPIC, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # #10 dock/undock clients + dock status (all optional)
        if HAVE_CREATE:
            self.dock_client = ActionClient(self, Dock, "dock")
            self.undock_client = ActionClient(self, Undock, "undock")
            # #16: the Create 3 publishes dock_status BEST-EFFORT. A default
            # (reliable) subscription silently gets NOTHING -> is_docked would
            # never update on the real robot. Sensor-data QoS matches it.
            self.create_subscription(DockStatus, "dock_status",
                                     self.on_dock_status, qos_profile_sensor_data)
        else:
            self.dock_client = self.undock_client = None

        self.create_timer(THINK_PERIOD, self.think)
        self.create_timer(PUB_PERIOD, self.publish_cmd)
        self.create_timer(ANNOT_PERIOD, self.annotate_think)      # #20

    # ── sensor callbacks ──
    def on_rgb_depth(self, rgb_msg, depth_msg):        # #12: matched pair
        self.pair = (rgb_msg, depth_msg)
        self.sync_seen = True
    def on_rgb(self, m):   self.rgb_raw = m
    def on_depth(self, m): self.depth_raw = m
    def on_info(self, m):  self.K = m.k; self.cam_frame = m.header.frame_id
    def on_map(self, m):   self.map = m
    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.have_odom = True
    def on_scan(self, m):
        if len(m.ranges) == 0: return
        cone = math.radians(15); best = float("inf")
        for i, r in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if -cone <= ang <= cone and math.isfinite(r) and r > 0.0:
                best = min(best, r)
        self.front_min = best
    def on_dock_status(self, m):
        self.is_docked = bool(m.is_docked)
        self.dock_known = True
        # #22: the first time we learn we're ON the dock, the robot's own map
        # pose IS the dock's position. (TF may not be up yet at boot — then
        # robot_xy() is None and we simply try again on the next status msg.)
        if self.is_docked and self.dock_xy is None:
            xy = self.robot_xy()
            if xy is not None:
                self.set_dock_position(*xy)

    def notify(self, msg):
        self.mlog.log("ROBOT", msg)                    # #14
        print(f"\n[robot] {msg}\nCommand> ", end="", flush=True)

    # ── #12: one place decides which frames the whole pipeline uses ──
    def current_pair(self):
        """Return a matched (rgb_msg, depth_msg) pair. Prefers the
        timestamp-synchronized pair; falls back to latest-of-each with a
        one-time warning if pairing has never worked (e.g. a driver that
        stamps the two streams from different clocks)."""
        if self.pair is not None:
            return self.pair
        if self.rgb_raw is not None and self.depth_raw is not None:
            if (not self.warned_no_sync and not self.sync_seen
                    and time.monotonic() - self.start_t > SYNC_WARN_S):
                self.warned_no_sync = True
                self.notify("RGB and depth timestamps never match (slop "
                            f"{SYNC_SLOP_S}s) — using unsynchronized frames. "
                            "Check the camera driver's stamps.")
                self.mlog.log("WARN", "RGB-depth sync failed; raw fallback in use")
            return (self.rgb_raw, self.depth_raw)
        return None

    def camera_ready(self):
        return self.current_pair() is not None and self.K is not None

    # ── YOLO ──
    def yolo_detect(self, rgb_msg=None):
        """Run detection on rgb_msg (or the current pair's RGB). #15: pooled
        session + throttled error message + logging."""
        if rgb_msg is None:
            pair = self.current_pair()
            if pair is None: return None
            rgb_msg = pair[0]
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_rgb)
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        try:
            resp = self.http.post(YOLO_URL, json={"image": img_b64}, timeout=15)
            dets = resp.json()["detections"]
            # #20: remember WHICH frame these belong to so the annotated feed
            # can reuse them instead of running YOLO twice on one image.
            self.last_det = (self._stamp_key(rgb_msg), dets)
            return dets
        except Exception as e:
            now = time.monotonic()
            self.mlog.log("WARN", f"YOLO request failed: {e}")
            if now - self.last_yolo_err_t > YOLO_ERR_PERIOD:
                self.last_yolo_err_t = now
                print(f"\n[robot] YOLO server error (is yolo_server.py running?): {e}")
            return None

    # ── project one pixel (u, v) to a map (x, y) using depth + TF ──
    #    #8/#12: the depth frame is the one PAIRED with the RGB YOLO saw, the
    #    TF lookup uses its CAPTURE time (with a 'latest' fallback), and depth
    #    is sampled as a median patch to shrug off noise/holes.
    # ── #20: median depth (METRES) around a pixel. Factored out of
    #    project_pixel so the annotated feed's distance labels and the
    #    navigation goals come from the SAME number — a label that disagreed
    #    with the goal would be worthless as evidence.
    def depth_at(self, u, v, depth_msg):
        """Return (depth_m, u_clamped, v_clamped), or None if the patch has
        too few valid pixels (OAK-D depth has holes)."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return None
        h, w = depth_img.shape[:2]
        u = max(0, min(int(u), w - 1)); v = max(0, min(int(v), h - 1))
        patch = depth_img[max(0, v-4):v+5, max(0, u-4):u+5].astype(float)
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size < 5:                       # too few valid pixels -> unreliable
            return None
        d = float(np.median(patch))
        if not math.isfinite(d) or d <= 0:
            return None
        if d > 100: d = d / 1000.0               # mm -> m if needed
        return d, u, v

    def project_pixel(self, u, v, depth_msg=None):
        if depth_msg is None:
            pair = self.current_pair()
            if pair is None: return None
            depth_msg = pair[1]
        if self.K is None or self.cam_frame is None:
            return None
        got = self.depth_at(u, v, depth_msg)
        if got is None:
            return None
        d, u, v = got
        return self._project_uvd(u, v, d, depth_msg.header.stamp)

    # ── #21a: depth for a whole BOUNDING BOX, occlusion-robust ──
    def robust_box_depth(self, box, depth_msg):
        """Median depth over a grid of samples across the central region of
        the box. The old single centre-patch read could land on a shelf/rack
        IN FRONT of a partially occluded target and report the occluder's
        distance — which is how 'Arrived at the person' fired metres away
        from the person. With a grid, the occluder must cover most of the
        box to steal the reading. Returns depth in metres, or None."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return None
        h, w = depth_img.shape[:2]
        x1, y1, x2, y2 = box
        # central 70% of the box: skip the outer rim, which is mostly
        # background pixels bleeding into the detection
        mx, my = 0.15 * (x2 - x1), 0.15 * (y2 - y1)
        xa, xb = x1 + mx, x2 - mx
        ya, yb = y1 + my, y2 - my
        n = max(2, BOX_DEPTH_GRID)
        vals = []
        for i in range(n):
            for j in range(n):
                u = int(round(xa + (xb - xa) * (i + 0.5) / n))
                v = int(round(ya + (yb - ya) * (j + 0.5) / n))
                if 0 <= u < w and 0 <= v < h:
                    d = float(depth_img[v, u])
                    if math.isfinite(d) and d > 0:
                        vals.append(d)
        if len(vals) < 5:                        # box mostly holes -> unreliable
            return None
        d = float(np.median(vals))
        if not math.isfinite(d) or d <= 0:
            return None
        if d > 100: d = d / 1000.0               # mm -> m if needed
        return d

    def project_box(self, box, depth_msg):
        """Map (x, y) of a detection: bearing from the box centre, depth from
        the occlusion-robust grid over the whole box (#21a)."""
        if self.K is None or self.cam_frame is None or depth_msg is None:
            return None
        d = self.robust_box_depth(box, depth_msg)
        if d is None:
            # fall back to the old centre read rather than dropping the target
            x1, y1, x2, y2 = box
            got = self.depth_at((x1 + x2) / 2, (y1 + y2) / 2, depth_msg)
            if got is None:
                return None
            d = got[0]
        x1, y1, x2, y2 = box
        return self._project_uvd((x1 + x2) / 2, (y1 + y2) / 2, d,
                                 depth_msg.header.stamp)

    def _project_uvd(self, u, v, d, stamp):
        """Pixel (u, v) at depth d (m) -> map (x, y) via camera ray + TF at
        the frame's capture time (#8)."""
        fx, fy = self.K[0], self.K[4]; cx, cy = self.K[2], self.K[5]
        pt = PointStamped()
        pt.header.frame_id = self.cam_frame
        pt.header.stamp = stamp                  # the time the frame was actually taken
        pt.point.x = (u - cx) * d / fx
        pt.point.y = (v - cy) * d / fy
        pt.point.z = d
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, self.cam_frame, rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.1))
            obj = do_transform_point(pt, tf)
        except Exception:
            try:                                  # fallback: latest TF (old behaviour)
                pt.header.stamp = rclpy.time.Time().to_msg()
                tf = self.tf_buffer.lookup_transform(MAP_FRAME, self.cam_frame, rclpy.time.Time())
                obj = do_transform_point(pt, tf)
            except Exception:
                return None
        return obj.point.x, obj.point.y

    # ── #21b: TELEPORT GATE for committed targets ──
    def accept_sighting(self, ox, oy, meas_dist=None):
        """Objects don't teleport; depth noise and occluder-stolen depth do.
        Once committed to a target, a new sighting that moves it by more than
        TELEPORT_JUMP metres is held as 'pending' and only believed when a
        SECOND look agrees with it. One bad frame can no longer hijack the
        goal or fake an arrival.

        #30b: 'agrees' now SCALES WITH RANGE. Stereo depth error grows with
        distance, so two honest looks at a target 10 m away can legitimately
        disagree by more than a metre. The old fixed 1.0 m radius made a
        genuine RE-TARGET (the closest chair coming into view after the robot
        had committed to a far one) impossible to confirm — the robot kept
        driving to the wrong chair. A stale pending also expires, so a single
        odd frame can't block updates indefinitely."""
        if self.last_obj_xy is None:
            self.jump_pending = None
            return True
        now = time.monotonic()
        jump = math.hypot(ox - self.last_obj_xy[0], oy - self.last_obj_xy[1])
        if jump <= TELEPORT_JUMP:
            self.jump_pending = None
            return True
        agree = max(JUMP_AGREE,
                    JUMP_AGREE_FRAC * (meas_dist if meas_dist else jump))
        if (self.jump_pending is not None
                and now - self.jump_pending_t <= JUMP_PENDING_TTL
                and math.hypot(ox - self.jump_pending[0],
                               oy - self.jump_pending[1]) <= agree):
            self.mlog.log("SIGHT", f"large {self.target} jump ({jump:.1f} m) "
                                   f"confirmed by a second look — accepting")
            self.jump_pending = None
            return True
        self.mlog.log("WARN", f"suspicious {self.target} jump of {jump:.1f} m "
                              f"(occlusion/depth noise?) — waiting for a confirming "
                              f"look (agree radius {agree:.1f} m)")
        self.jump_pending = (ox, oy)
        self.jump_pending_t = now
        return False

    # ── #28: EVERY visible instance of the target, with distances ──
    def locate_candidates(self):
        """Return (cands, rx, ry) where cands is a list of
        (ox, oy, dist_m, pixel_u) for every instance of self.target the camera
        can see AND range right now, nearest first. Returns ([], None, None)
        if we can't see or can't range any.

        Factored out of locate_target so that reporting ("how far are the
        chairs") and acting ("go to the chair") use the SAME numbers — a
        report that disagreed with the goal would be worthless as evidence."""
        pair = self.current_pair()
        if pair is None: return [], None, None
        rgb_msg, depth_msg = pair
        detections = self.yolo_detect(rgb_msg)
        if detections is None: return [], None, None
        matches = [d for d in detections if self.target and self.target in d["name"].lower()]
        if not matches: return [], None, None
        pose = self.robot_pose_map()
        if pose is None: return [], None, None
        rx, ry, _ = pose
        cands = []                               # (ox, oy, dist, pixel_u)
        for d in matches:
            x1, y1, x2, y2 = d["box"]
            proj = self.project_box(d["box"], depth_msg)          # #21a
            if proj is None: continue
            ox, oy = proj
            # #22: anything sitting ON the dock IS the dock (YOLO-World kept
            # labelling it "chair") — skip it unless the dock was the target.
            if "dock" not in self.target and self.near_dock(ox, oy):
                continue
            # #28: two boxes on the SAME physical object (YOLO-World stacks
            # them on plain shapes) must not be reported as two objects.
            if any(math.hypot(ox - cx, oy - cy) < REPORT_MERGE_RADIUS
                   for cx, cy, _, _ in cands):
                continue
            cands.append((ox, oy, math.hypot(ox - rx, oy - ry), (x1 + x2) / 2))
        cands.sort(key=lambda c: c[2])           # nearest first
        return cands, rx, ry

    # ── locate the target; #3: choose nearest (or qualifier) among instances ──
    def locate_target(self):
        cands, rx, ry = self.locate_candidates()
        if not cands: return None
        q = self.target_qualifier
        if q == "farthest":
            ox, oy, dist, _ = max(cands, key=lambda c: c[2])
        elif q == "leftmost":
            ox, oy, dist, _ = min(cands, key=lambda c: c[3])   # smaller u = left in image
        elif q == "rightmost":
            ox, oy, dist, _ = max(cands, key=lambda c: c[3])
        else:                                                  # nearest (default)
            ox, oy, dist, _ = min(cands, key=lambda c: c[2])
        return ox, oy, rx, ry, dist

    # ── #28: turn a candidate list into one natural sentence ──
    def describe_distances(self, cands):
        """'about 6.3 m away' / 'one 6.3 m away and another 12.4 m away' /
        '2.1 m, 5.4 m and 9.8 m away'. Nearest first."""
        ds = [c[2] for c in cands]
        if len(ds) == 1:
            return f"about {ds[0]:.1f} m away"
        if len(ds) == 2:
            return f"one about {ds[0]:.1f} m away and another about {ds[1]:.1f} m away"
        head = ", ".join(f"{d:.1f} m" for d in ds[:-1])
        return f"at about {head} and {ds[-1]:.1f} m away"

    # ── project EVERY detection of a class to map (x, y) ──
    def locate_all(self, target, detections=None, depth_msg=None):
        if detections is None:
            pair = self.current_pair()
            if pair is None: return []
            detections = self.yolo_detect(pair[0]) or []
            depth_msg = pair[1]
        out = []
        for d in detections:
            if target and target in d["name"].lower():
                proj = self.project_box(d["box"], depth_msg)      # #21a
                if proj is not None:
                    if "dock" not in target and self.near_dock(proj[0], proj[1]):
                        continue                                   # #22
                    out.append(proj)
        return out

    # ── remember distinct object positions; return how many were NEW ──
    def register_instances(self, target, positions):
        known = self.seen_instances.setdefault(target, [])
        added = 0
        for (x, y) in positions:
            if "dock" not in target and self.near_dock(x, y):
                continue                         # #22: never memorise the dock as an object
            if all(math.hypot(x - kx, y - ky) > NEW_INSTANCE_RADIUS for (kx, ky) in known):
                known.append((x, y)); added += 1
        return added

    # ── #22: dock no-detect zone ──
    def set_dock_position(self, x, y):
        """Remember where the dock is, and purge any 'objects' the memory is
        holding at that spot — they were the dock wearing a costume."""
        self.dock_xy = (x, y)
        self.mlog.log("DOCK", f"dock position recorded at ({x:.2f}, {y:.2f}); "
                              f"detections within {DOCK_EXCLUDE_RADIUS} m are ignored")
        for cls in list(self.seen_instances.keys()):
            if "dock" in cls:
                continue                         # remembering the dock AS a dock is fine
            kept = [p for p in self.seen_instances[cls]
                    if not self.near_dock(p[0], p[1])]
            dropped = len(self.seen_instances[cls]) - len(kept)
            if dropped:
                self.seen_instances[cls] = kept
                self.mlog.log("MEMORY", f"purged {dropped} remembered {cls} "
                                        f"position(s) that sat on the dock")
        if (self.last_located is not None
                and self.near_dock(self.last_located[1][0], self.last_located[1][1])):
            self.last_located = None

    def near_dock(self, x, y):
        """True if (x, y) falls inside the dock's exclusion zone."""
        return (self.dock_xy is not None and
                math.hypot(x - self.dock_xy[0], y - self.dock_xy[1]) < DOCK_EXCLUDE_RADIUS)

    def robot_xy(self):
        pose = self.robot_pose_map()
        return None if pose is None else (pose[0], pose[1])

    def robot_pose_map(self):
        """Robot (x, y, yaw) in the MAP frame from TF (correct for goals)."""
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return (tf.transform.translation.x,
                    tf.transform.translation.y,
                    yaw_from_quat(tf.transform.rotation))
        except Exception:
            return None

    # ── occupancy-grid helpers (#5: keep goals out of walls) ──
    def grid_value(self, x, y):
        m = self.map
        if m is None: return None
        res = m.info.resolution
        ox = m.info.origin.position.x; oy = m.info.origin.position.y
        cx = int((x - ox) / res); cy = int((y - oy) / res)
        if cx < 0 or cy < 0 or cx >= m.info.width or cy >= m.info.height:
            return None
        return m.data[cy * m.info.width + cx]

    def cell_free(self, x, y):
        v = self.grid_value(x, y)
        return v is not None and 0 <= v < 50     # known + not (near-)occupied

    # #19b: a single free CELL is not enough — the robot has a body. Check a
    # disc of ROBOT_CLEARANCE around the point so our "free" agrees with
    # Nav2's inflation layer. Without this we happily sent goals 5 cm from a
    # wall and Nav2 (correctly) refused them, which looked like "it can't
    # avoid obstacles".
    def is_goal_free(self, x, y, clearance=ROBOT_CLEARANCE):
        m = self.map
        if m is None:
            return False
        if not self.cell_free(x, y):
            return False
        res = m.info.resolution
        r = max(1, int(math.ceil(clearance / res)))
        r2 = r * r
        for iy in range(-r, r + 1):
            for ix in range(-r, r + 1):
                if ix * ix + iy * iy > r2:
                    continue
                if not self.cell_free(x + ix * res, y + iy * res):
                    return False
        return True

    # ── #19c: candidate approach goals in a RING around the object ──
    def is_goal_tried(self, x, y):
        return any(math.hypot(x - tx, y - ty) < GOAL_TRIED_RADIUS
                   for tx, ty in self.goal_tried)

    def approach_candidates(self, ox, oy, rx, ry):
        """Return reachable-looking stand-off goals AROUND the object at
        (ox, oy), best first.

        The old code used exactly ONE goal: the point on the straight line
        between the robot and the object. If a box sat on that line the task
        died — even though the object's left/right/far side was wide open.
        Here we sweep a ring of directions and stand-off distances, drop any
        that don't fit the robot's footprint or that Nav2 already refused,
        and rank the rest by travel distance (plus a mild penalty for walking
        around to the far side). Nav2 still plans the actual path — this only
        picks WHERE to end up."""
        if self.map is None:
            return []
        base = math.atan2(ry - oy, rx - ox)      # object -> robot: our own side
        out = []
        for standoff in APPROACH_STANDOFFS:
            for off_deg in APPROACH_RING:
                a = base + math.radians(off_deg)
                gx = ox + standoff * math.cos(a)
                gy = oy + standoff * math.sin(a)
                if not self.is_goal_free(gx, gy):
                    continue
                if self.is_goal_tried(gx, gy):
                    continue
                travel = math.hypot(gx - rx, gy - ry)
                cost = travel + 0.30 * abs(math.radians(off_deg))
                out.append((cost, gx, gy))
        out.sort(key=lambda c: c[0])
        return [(gx, gy) for _, gx, gy in out]

    def nudge_to_free(self, x, y, rx, ry):
        """If (x,y) is in a wall/unknown, pull it back toward the robot to the
        nearest free point on that line. Returns None if nothing works."""
        if self.is_goal_free(x, y):
            return x, y
        for frac in (0.85, 0.7, 0.55, 0.4, 0.25):
            nx = rx + (x - rx) * frac; ny = ry + (y - ry) * frac
            if self.is_goal_free(nx, ny):
                return nx, ny
        return None

    # ── #18: approach the EDGE of the known map toward a far target ──
    def farthest_free_along(self, rx, ry, x, y, min_d=0.5):
        """Walk the robot->target line in map-resolution steps and return the
        FARTHEST free point that is at least min_d from the robot. When the
        target was projected into unmapped space (e.g. a chair seen 14 m away,
        beyond where SLAM has mapped), this gives Nav2 a reachable goal at the
        frontier of the known map; as the robot gets there SLAM extends the
        map and the next think-cycle pushes the goal further. Returns None
        only if there's no free point beyond min_d (then we should rescan)."""
        if self.map is None:
            return None
        dx, dy = x - rx, y - ry
        dist = math.hypot(dx, dy)
        if dist < min_d:
            return None
        step = max(self.map.info.resolution, 0.10)
        best = None
        n = int(dist / step)
        for i in range(1, n + 1):
            d = i * step
            px = rx + dx / dist * d
            py = ry + dy / dist * d
            if d >= min_d and self.is_goal_free(px, py):
                best = (px, py)     # keep the farthest; walls in between are
                                    # fine — Nav2 plans AROUND them (#5)
        return best

    # ── #17: pick a remembered instance of a class (qualifier-aware) ──
    def recall_instance(self, target):
        """Return the remembered map (x, y) of a previously seen instance of
        'target', honouring nearest/farthest. leftmost/rightmost are camera-
        relative so they don't apply to memory -> fall back to nearest."""
        # #23: "find a chair" ... "go to it" must return the chair we JUST
        # located — not whichever remembered chair happens to be nearest.
        # An explicit qualifier (nearest/farthest/...) still wins.
        if (self.last_located is not None and self.last_located[0] == target
                and self.target_qualifier is None):
            return self.last_located[1]
        known = self.seen_instances.get(target, [])
        if not known:
            return None
        pose = self.robot_pose_map()
        if pose is None:
            return known[-1]                     # best effort: most recent
        rx, ry, _ = pose
        if self.target_qualifier == "farthest":
            return max(known, key=lambda p: math.hypot(p[0]-rx, p[1]-ry))
        return min(known, key=lambda p: math.hypot(p[0]-rx, p[1]-ry))

    # ── INPUT LOOP (main thread) ──
    def run_input_loop(self):
        print(f"[{AGENT_VERSION}]")
        print("VLA agent ready. 'cancel'/'stop' stops a task, 'quit' exits.")
        print("Try: go to the nearest person | find a chair | move forward 2 m | "
              "turn right 30 | patrol the area | dock | undock | "
              "go to the person then to the chair\n")
        while not self.shutdown:
            try:
                cmd = input("Command> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd: continue
            low = cmd.lower()
            stripped = strip_politeness(low)

            if stripped.startswith(QUIT_WORDS):
                break

            # #1 + #25: cancel/stop preempts EVERYTHING, even with politeness
            # wrappers — and now even with a typo. ("cancle" used to reach the
            # LLM and come back as a reject.)
            hit = None
            if stripped.startswith(STOP_WORDS):
                hit = next(w for w in STOP_WORDS if stripped.startswith(w))
            else:
                hit = fuzzy_word(first_word(stripped), STOP_WORDS)
            if hit is not None:
                typed = first_word(stripped)
                if typed != hit:
                    print(f"(reading '{typed}' as '{hit}')")
                    self.mlog.log("FUZZY", f"'{typed}' -> '{hit}'")
                self.mlog.log("CMD", f"(cancel) {cmd}")
                self.cancel_task(); print("Task cancelled. Robot is idle."); continue

            # a pending yes/no question (e.g. patrol feed prompt)
            if self.pending is not None:
                self.resolve_pending(cmd); continue

            # #26: a bare "yes"/"no" with NO question open means nothing. It
            # used to go to the brain, which had nothing to attach it to and
            # invented a task from stale context ("yes" -> "checking if the
            # shelf is still there"). Answer honestly instead.
            bare = first_word(stripped)
            if (len(stripped.split()) == 1
                    and (bare in YES_WORDS or bare in NO_WORDS
                         or fuzzy_word(bare, YES_WORDS + NO_WORDS) is not None)):
                print("I'm not waiting on a yes/no right now — tell me what "
                      "you'd like me to do (e.g. 'go to the chair', 'patrol', 'dock').")
                self.mlog.log("CMD", f"(bare confirmation ignored) {cmd}")
                continue

            self.mlog.log("CMD", cmd)                                     # #14

            # #10: handle obvious dock/undock deterministically (no camera/LLM needed)
            di = self.dock_intent(stripped)
            if di is not None:
                self.cancel_navigation()
                self.queue = [{"action": di, "target": None,
                               "speech": "Undocking." if di == "undock" else "Returning to the dock."}]
                self.has_feed_step = False
                self.start_next_step()
                continue

            self.cancel_navigation()
            self.handle_command(cmd)
        self.shutdown = True

    def dock_intent(self, low):
        """Return 'dock'/'undock' for clear one-word docking commands, else None."""
        if low in ("undock", "un dock", "leave the dock", "leave dock",
                   "move off the dock", "get off the dock", "off the dock"):
            return "undock"
        if low in ("dock", "go dock", "dock now", "return to dock", "return to the dock",
                   "go to the dock", "go to dock", "return to base", "go home",
                   "go to base", "charge", "go charge", "go to the charging station"):
            return "dock"
        return None

    def handle_command(self, cmd):
        # Camera is only needed for vision tasks; the brain still runs without it
        # so move/dock/explore work before the camera is up.
        visible = []
        pair = self.current_pair()
        if pair is not None:
            detections = self.yolo_detect(pair[0])
            if detections:
                visible = sorted({d["name"] for d in detections})
        print(f"(I see: {visible if visible else 'nothing'})  thinking...")
        try:
            # #11: pass the previous action/target so "go to IT" resolves.
            # #27: ...but only while it's FRESH. Stale context is how a target
            # from many commands ago gets resurrected into a nonsense task.
            if self.ctx_last_target and time.monotonic() - self.ctx_time > CONTEXT_TTL:
                self.mlog.log("CTX", f"dropping stale context "
                                     f"({self.ctx_last_action} -> {self.ctx_last_target})")
                self.ctx_last_action = None
                self.ctx_last_target = None
            context = {"last_action": self.ctx_last_action,
                       "last_target": self.ctx_last_target}
            decision = decide(cmd, visible, context=context)
        except Exception as e:
            self.mlog.log("ERROR", f"brain failed: {e}")
            print(f"Brain error (is Ollama running?): {e}"); return
        self.mlog.log("BRAIN", f"{decision}")                             # #14
        steps = decision.get("steps")
        if not steps:
            steps = [decision] if decision.get("action") else []
        if not steps:
            print("I didn't catch a clear command."); return
        self.queue = list(steps)
        self.has_feed_step = any((s.get("action") or "").lower() == "feed" for s in steps)
        self.start_next_step()

    def start_next_step(self):
        if not self.queue:
            return
        step = self.queue.pop(0)
        action = (step.get("action") or "").lower()
        target = step.get("target")
        speech = step.get("speech", "")
        if speech:
            print(f"[{action}] {speech}")
        self.mlog.log("STEP", f"{action} target={target!r}")              # #14

        # #11: remember the last thing we acted on for pronoun resolution
        if target:
            self.ctx_last_action = action
            self.ctx_last_target = str(target).lower()
            self.ctx_time = time.monotonic()                              # #27

        # #10: if the robot is docked and the next step needs motion, undock first
        # then resume this step. (This is why Nav2 "stopped working" after docking.)
        if action in MOVE_ACTIONS and HAVE_CREATE and self.is_docked:
            self.queue.insert(0, step)
            self.notify("I'm docked — undocking first, then I'll continue.")
            self.start_dock_action("undock")
            return

        if action == "navigate":
            if not target:
                self.start_next_step(); return
            self.target = target.lower()
            self.target_qualifier = (step.get("qualifier") or "").lower() or None   # #3
            self.search_state = "ROTATE"
            self.turn_accum = 0.0; self.last_yaw = None; self.advance_start = None
            self.last_obj_xy = None
            self.target_announced = False
            self.nav_goal_xy = None
            self.sight_count = 0                                                    # #13
            self.seen_live = False                                                  # #17
            self.goal_fail_count = 0                                                # #18
            self.navigate_start = time.monotonic()                                  # #6
            self.mode = "navigate"
            extra = f" ({self.target_qualifier})" if self.target_qualifier else ""
            # #23 LIVE-FIRST: if the target class is visible RIGHT NOW, search
            # fresh (it locks on within a couple of think-cycles) and skip the
            # memory seed — no "I remember..." monologue for something in
            # plain sight, and no driving to a stale spot while the real
            # object sits in front of the camera. Memory's job (#17) is only
            # returning to things that AREN'T currently visible.
            live_now = False
            pair = self.current_pair()
            if pair is not None:
                dets = self.yolo_detect(pair[0]) or []
                live_now = any(self.target in d["name"].lower() for d in dets)
            seed = None
            if MEMORY_SEED and not live_now:
                seed = self.recall_instance(self.target)
            if seed is not None:
                self.last_obj_xy = seed
                self.target_announced = True            # suppress duplicate "Found it"
                pose = self.robot_pose_map()
                if pose is not None:
                    d0 = math.hypot(seed[0] - pose[0], seed[1] - pose[1])
                    print(f"I remember seeing a {self.target} about {d0:.1f} m away — "
                          f"heading to that spot and I'll confirm it on the way. "
                          f"(type 'cancel' to stop)")
                else:
                    print(f"I remember where a {self.target} was — heading there. "
                          f"(type 'cancel' to stop)")
                self.mlog.log("MEMORY", f"seeded {self.target} from remembered "
                                        f"instance at ({seed[0]:.2f}, {seed[1]:.2f})")
            else:
                print(f"Searching for the {self.target}{extra}. (type 'cancel' to stop)")

        elif action == "locate":                                                    # #7
            if not target:
                self.start_next_step(); return
            self.target = target.lower()
            self.target_qualifier = (step.get("qualifier") or "").lower() or None
            self.locate_start = time.monotonic()
            self.last_yaw = None; self.turn_accum = 0.0
            self.mode = "locate"
            print(f"Looking for a {self.target} (reporting only, not driving). (type 'cancel' to stop)")

        elif action == "move":                                                      # #9
            rot_deg = float(step.get("rotation_deg") or 0.0)
            dist_m  = float(step.get("distance_m") or 0.0)
            if abs(rot_deg) < 0.5 and abs(dist_m) < 0.01:
                print("That move had no distance or angle."); self.start_next_step(); return
            self.move_spec = {
                "phase":   "rotate" if abs(rot_deg) >= 0.5 else "translate",
                "rot_rad": abs(math.radians(rot_deg)),
                "rot_sign": 1 if rot_deg >= 0 else -1,        # + = left / CCW
                "dist_m":  dist_m,                             # + = forward, - = back
            }
            self.move_ref_yaw = None; self.move_ref_pos = None
            self.mode = "move"
            parts = []
            if abs(rot_deg) >= 0.5:
                parts.append(f"turn {abs(rot_deg):.0f} deg {'left' if rot_deg >= 0 else 'right'}")
            if abs(dist_m) >= 0.01:
                parts.append(f"move {'forward' if dist_m > 0 else 'back'} {abs(dist_m):.2f} m")
            print(f"Relative move: {', '.join(parts)}. (type 'cancel' to stop)")

        elif action == "find_another":
            if not target:
                self.start_next_step(); return
            self.target = target.lower()
            self.find_baseline = len(self.seen_instances.get(self.target, []))
            self.find_start = time.monotonic()
            self.search_state = "ROTATE"
            self.turn_accum = 0.0; self.last_yaw = None; self.advance_start = None
            self.mode = "find_more"
            print(f"Looking for another {self.target}. (type 'cancel' to stop)")

        elif action in ("explore", "patrol"):
            if action == "patrol" and not self.has_feed_step:
                self.pending = {"type": "feed_confirm"}
                self.notify("Do you want a live camera feed during the patrol? (yes / no)")
                return
            self.start_explore(label=action)

        elif action in ("dock", "undock"):                                          # #10
            self.start_dock_action(action)
            return                              # wait for the action to finish

        elif action == "count":
            t = (target or "").lower()
            saved_target = self.target
            self.target = t                      # locate_candidates works on self.target
            cands, _, _ = self.locate_candidates() if t else ([], None, None)
            self.target = saved_target
            n = len(cands)
            if t and cands:
                self.register_instances(t, [(c[0], c[1]) for c in cands])
                plural = t if n == 1 else t + "s"
                # #28: a count without distances made the follow-up question
                # ("at what distance are they?") necessary in the first place.
                print(f"I can see {n} {plural} right now — {self.describe_distances(cands)}.")
                self.mlog.log("COUNT", f"{n} {t}(s): "
                                       + ", ".join(f"{c[2]:.2f}m" for c in cands))
            else:
                # nothing ranged: fall back to a plain box count so the answer
                # is still honest when depth is missing (holes / out of range)
                pair = self.current_pair()
                det = (self.yolo_detect(pair[0]) or []) if pair is not None else []
                nb = sum(1 for d in det if t and t in d["name"].lower())
                if nb:
                    print(f"I can see {nb} '{target}' right now, but I can't get a "
                          f"depth reading on {'it' if nb == 1 else 'them'} from here.")
                else:
                    print(f"I can't see any '{target}' right now.")
            self.start_next_step()

        elif action == "forget":                                            # #24
            t = (target or "").lower().strip()
            if t:
                had = len(self.seen_instances.pop(t, []))
                if self.last_located is not None and self.last_located[0] == t:
                    self.last_located = None
                self.mlog.log("MEMORY", f"forgot {had} remembered {t} position(s)")
                print(f"Done — I've forgotten every {t} position I had remembered "
                      f"({had} of them)." if had
                      else f"I had no remembered {t} positions anyway.")
            else:
                n = sum(len(v) for v in self.seen_instances.values())
                self.seen_instances.clear()
                self.last_located = None
                self.mlog.log("MEMORY", f"cleared ALL remembered positions ({n})")
                print(f"Done — I've cleared my whole spatial memory ({n} position(s)).")
            self.start_next_step()

        elif action == "describe":
            # #28: "what do you see" now answers with distances too, so the
            # operator doesn't have to ask a second question for every object.
            pair = self.current_pair()
            det = (self.yolo_detect(pair[0]) or []) if pair is not None else []
            if not det:
                print("I see: nothing")
            else:
                parts = []
                for d in det:
                    nm = d["name"]
                    bd = self.robust_box_depth(d["box"], pair[1])   # #21a, same depth nav uses
                    parts.append(f"{nm} ({bd:.1f} m)" if bd is not None
                                 else f"{nm} (distance unclear)")
                print("I see: " + ", ".join(parts))
            self.start_next_step()

        elif action == "feed":
            self.start_feed()
            self.start_next_step()

        else:                                   # answer / reject / clarify -> speech already printed
            self.start_next_step()

    # ── start frontier explore/patrol ──
    def start_explore(self, label="explore"):
        self.blacklist = []
        self.visited_goals = []                 # #4: fresh coverage memory each patrol
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.explore_start_t = time.monotonic()
        self.warned_no_map = self.warned_no_frontier = False
        self.consecutive_stucks = 0
        self.mode = "explore"
        print(f"Starting to {label} and map the area. (type 'cancel' to stop)")

    # ── handle the answer to a pending yes/no question ──
    def resolve_pending(self, cmd):
        p = self.pending
        self.pending = None
        if p["type"] == "feed_confirm":
            if is_yes(cmd):                      # #25: tolerant of 'yse', 'yeh'
                self.start_feed()
            else:
                print("OK, patrolling without the live feed.")
            self.start_explore(label="patrol")

    # ── cancel + feed ──
    def cancel_navigation(self):
        self.task_id += 1
        self.mode = None
        self.search_state = None
        self.target = None
        self.target_qualifier = None
        self.queue = []
        self.pending = None
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.sight_count = 0                    # #13
        self.seen_live = False                  # #17
        self.goal_fail_count = 0                # #18
        self.best_dist = None                   # #19a
        self.last_progress_t = None             # #19a
        self.goal_tried = []                    # #19c
        self.jump_pending = None                # #21b
        self.goal_obj_xy = None                 # #30a
        self.navigate_start = None
        self.locate_start = None
        self.move_spec = None
        self.nudge_from = None                  # #29
        self.backup_nudges = 0                  # #29
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.last_pos = None; self.last_move_t = None
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
        self.goal_handle = None
        if self.dock_goal_handle is not None:
            try: self.dock_goal_handle.cancel_goal_async()
            except Exception: pass
            self.dock_goal_handle = None
        self.cmd = Twist()
        for _ in range(3):                      # publish a few zeros for a hard stop
            try: self.cmd_pub.publish(Twist())
            except Exception: pass

    def cancel_task(self):
        self.cancel_navigation()
        self.stop_feed()

    # ── #20: ANNOTATED LIVE FEED ──
    def _stamp_key(self, msg):
        s = msg.header.stamp
        return (s.sec, s.nanosec)

    def _class_color(self, name):
        """A stable BGR colour per class (md5, not hash(), so the colours are
        the same every run — screenshots stay consistent across slides)."""
        h = hashlib.md5(name.encode("utf-8")).digest()
        return (int(70 + h[0] % 186), int(70 + h[1] % 186), int(70 + h[2] % 186))

    def annotate_think(self):
        """Republish the camera image with YOLO boxes drawn on it, so the live
        window is screenshot-ready evidence. Only runs while the feed is open."""
        if self.feed_proc is None or not ANNOTATED_FEED:
            return
        pair = self.current_pair()
        if pair is None:
            return
        rgb_msg, depth_msg = pair
        try:
            img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception:
            return

        # reuse the think-loop's inference when it's for this same frame
        key = self._stamp_key(rgb_msg)
        if self.last_det is not None and self.last_det[0] == key:
            dets = self.last_det[1]
        else:
            dets = self.yolo_detect(rgb_msg)
        if dets is None:
            dets = []                            # YOLO down -> still show the video

        # the sim's OAK-D preview is only 250x250; upscale so the labels are
        # readable when this gets pasted into a slide
        h0, w0 = img.shape[:2]
        scale = max(1, int(math.ceil(ANNOT_MIN_WIDTH / float(max(w0, 1)))))
        img = (cv2.resize(img, (w0 * scale, h0 * scale), interpolation=cv2.INTER_LINEAR)
               if scale > 1 else img.copy())
        H, W = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = max(0.40, W / 1500.0)
        lw = max(2, int(round(W / 400.0)))

        for d in dets:
            try:
                x1, y1, x2, y2 = [float(b) for b in d["box"]]
                name = str(d.get("name", "?"))
                conf = float(d.get("confidence", 0.0))
            except Exception:
                continue
            # highlight the object we're actually acting on
            is_target = bool(self.target and self.target in name.lower())
            color = (0, 255, 0) if is_target else self._class_color(name)
            X1, Y1 = int(round(x1 * scale)), int(round(y1 * scale))
            X2, Y2 = int(round(x2 * scale)), int(round(y2 * scale))
            cv2.rectangle(img, (X1, Y1), (X2, Y2), color, lw + (1 if is_target else 0))

            label = f"{name} {conf * 100:.0f}%"
            bd = self.robust_box_depth(d["box"], depth_msg)       # #21a: same number nav uses
            if bd is not None:                   # YOLO + OAK-D depth, in one frame
                label += f"  {bd:.2f}m"
            if is_target:
                label += "  <= TARGET"
            (tw, tht), _ = cv2.getTextSize(label, font, fs, 1)
            ly = max(Y1, tht + 6)
            cv2.rectangle(img, (X1, ly - tht - 6), (X1 + tw + 6, ly), color, -1)
            cv2.putText(img, label, (X1 + 3, ly - 4), font, fs, (0, 0, 0), 1, cv2.LINE_AA)

        # caption bar ABOVE the image (never covers a detection)
        cap = (f"MODE: {self.mode or 'idle'}   TARGET: {self.target or '-'}   "
               f"OBJECTS: {len(dets)}   {datetime.datetime.now().strftime('%H:%M:%S')}")
        (tw, tht), _ = cv2.getTextSize(cap, font, fs, 1)
        bar = np.zeros((tht + 14, W, 3), dtype=np.uint8)
        cv2.putText(bar, cap, (6, tht + 5), font, fs, (255, 255, 255), 1, cv2.LINE_AA)
        img = np.vstack([bar, img])

        try:
            out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            out.header = rgb_msg.header
            self.annot_pub.publish(out)
        except Exception:
            pass

    def start_feed(self):
        self.stop_feed()
        topic = ANNOT_TOPIC if ANNOTATED_FEED else RGB_TOPIC
        try:
            self.feed_proc = subprocess.Popen(
                ["ros2", "run", "rqt_image_view", "rqt_image_view", topic],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if ANNOTATED_FEED:
                print(f"Live feed opened on {topic} — YOLO boxes, confidence and "
                      f"depth distance are drawn on it. (type 'cancel' to close it)")
            else:
                print("Live camera feed opened in a new window. (type 'cancel' to close it)")
        except FileNotFoundError:
            print("Live feed needs rqt_image_view:  sudo apt install ros-humble-rqt-image-view")

    def stop_feed(self):
        if self.feed_proc is not None:
            try: self.feed_proc.terminate()
            except Exception: pass
            self.feed_proc = None

    # ── hard stop — cancel the Nav2 goal AND publish zero velocity ──
    def stop_base(self):
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
            self.goal_handle = None
        self.cmd = Twist()
        for _ in range(3):
            try: self.cmd_pub.publish(Twist())
            except Exception: pass

    # ── #10 dock / undock as a managed action ──
    def start_dock_action(self, which):
        if not HAVE_CREATE or self.dock_client is None:
            self.notify("Docking isn't available here (irobot_create_msgs not found). "
                        "On the real Create 3 / TurtleBot 4 this will work.")
            self.start_next_step(); return
        client = self.dock_client if which == "dock" else self.undock_client
        if not client.wait_for_server(timeout_sec=3.0):
            self.notify(f"The '{which}' action server isn't up (is the Create 3 running?). "
                        f"Check: ros2 action list | grep -i dock")
            self.start_next_step(); return
        self.mode = "dock"
        self.notify("Docking…" if which == "dock" else "Undocking…")
        goal = Dock.Goal() if which == "dock" else Undock.Goal()
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._dock_accepted(f, which))

    def _dock_accepted(self, future, which):
        try:
            gh = future.result()
        except Exception as e:
            self.notify(f"Could not send the {which} request: {e}"); self._after_dock(); return
        if not gh.accepted:
            self.notify(f"The robot rejected the {which} request."); self._after_dock(); return
        self.dock_goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._dock_done(f, which))

    def _dock_done(self, future, which):
        self.dock_goal_handle = None
        # #16: check the REAL outcome — the old code assumed success, so a
        # failed dock left is_docked wrong and later auto-undock logic confused.
        ok = True
        try:
            ok = (future.result().status == GoalStatus.STATUS_SUCCEEDED)
        except Exception:
            ok = False
        if ok:
            self.is_docked = (which == "dock")
            # #22: a successful DOCK puts us exactly ON the dock — the best
            # possible fix of its position (always refresh). After an UNDOCK
            # we're ~0.5 m off it — good enough only if we know nothing yet.
            xy = self.robot_xy()
            if xy is not None and (which == "dock" or self.dock_xy is None):
                self.set_dock_position(*xy)
            self.notify("Docked successfully." if which == "dock" else "Undocked — ready to move.")
        else:
            self.notify(f"The {which} action did not complete — please check the robot. "
                        "(dock_status will keep me updated.)")
        self.mlog.log("DOCK", f"{which} {'ok' if ok else 'FAILED'}")
        self._after_dock()

    def _after_dock(self):
        self.mode = None
        self.start_next_step()                  # continue the queue (e.g. the nav we deferred)

    # ── fast hands ──
    def publish_cmd(self):
        if self.mode == "move":
            self.cmd_pub.publish(self.cmd)
        elif self.mode == "locate":
            self.cmd_pub.publish(self.cmd)
        elif self.mode in ("navigate", "find_more") and self.search_state in ("ROTATE", "ADVANCE"):
            self.cmd_pub.publish(self.cmd)

    # ── slow brain ──
    def think(self):
        self.check_stuck()
        if self.mode == "navigate":
            self.navigate_think()
        elif self.mode == "locate":             # #7
            self.locate_think()
        elif self.mode == "move":               # #9
            self.move_think()
        elif self.mode == "find_more":
            self.find_more_think()
        elif self.mode == "explore":
            self.explore_think()

    # ── stuck detection ──
    def check_stuck(self):
        driving = ((self.mode in ("navigate", "find_more")
                    and self.search_state in ("NAVIGATING", "ADVANCE"))
                   or (self.mode == "explore" and self.frontier_navigating))
        if not driving:
            self.last_pos = None; self.last_move_t = None; return
        rxy = self.robot_xy()
        if rxy is None: return
        now = time.monotonic()
        if self.last_pos is None:
            self.last_pos = rxy; self.last_move_t = now; return
        if math.hypot(rxy[0]-self.last_pos[0], rxy[1]-self.last_pos[1]) > STUCK_DIST:
            self.last_pos = rxy; self.last_move_t = now; return
        if now - self.last_move_t > STUCK_TIME:
            self.on_stuck()

    def on_stuck(self):
        self.last_pos = None; self.last_move_t = None
        self.mlog.log("STUCK", f"mode={self.mode} target={self.target!r}")   # #14
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
        self.goal_handle = None
        if self.mode == "explore":
            self.consecutive_stucks += 1
            self.blacklist.append(self.cur_goal)
            self.frontier_navigating = False
            if self.consecutive_stucks >= MAX_STUCKS:
                self.notify("I keep getting stuck while exploring. Please take MANUAL OVERRIDE "
                            "(drive me with teleop), then tell me to continue.")
                self.cancel_task()
            else:
                self.notify("I got stuck at a frontier — skipping it and trying another route.")
        else:
            self.notify(f"I'm stuck trying to reach the {self.target}. Please take MANUAL "
                        "OVERRIDE or give me a new command.")
            self.cancel_navigation()

    # ── NAVIGATE: drive to the target, pathing around obstacles ──
    def navigate_think(self):
        if not self.camera_ready() or not self.have_odom:
            return

        now = time.monotonic()
        pose = self.robot_pose_map()

        # #19a: two DIFFERENT clocks, because "I can't find it" and "I can't
        # reach it" are different failures.
        #   - not committed yet  -> plain SEARCH_TIMEOUT.
        #   - committed to a target -> no deadline at all while we keep
        #     closing the gap; only quit after NO_PROGRESS_LIMIT seconds of
        #     getting no nearer. The old flat 60 s killed Nav2 mid-drive on
        #     anything far away, which is what looked like "it can't avoid
        #     the obstacle".
        if self.last_obj_xy is None:
            if self.navigate_start and now - self.navigate_start > SEARCH_TIMEOUT:
                self.stop_base()
                if self.seen_live:
                    self.notify(f"I saw the {self.target} but lost track of it and "
                                f"couldn't find it again. Try asking me once more.")
                else:
                    self.notify(f"Sorry, I couldn't find any {self.target} in this area.")
                self.end_navigate(); return
        elif pose is not None:
            d_now = math.hypot(pose[0] - self.last_obj_xy[0], pose[1] - self.last_obj_xy[1])
            if self.best_dist is None or d_now < self.best_dist - PROGRESS_EPS:
                self.best_dist = d_now
                self.last_progress_t = now              # we're getting closer: keep going
            elif self.last_progress_t is None:
                self.last_progress_t = now
            elif now - self.last_progress_t > NO_PROGRESS_LIMIT:
                self.stop_base()
                self.notify(f"I know where the {self.target} is but I can't get any "
                            f"closer — the way looks blocked. I've remembered its spot, "
                            f"so try again from somewhere else, or clear the path.")
                self.mlog.log("WARN", f"no progress toward {self.target} for "
                                      f"{NO_PROGRESS_LIMIT:.0f}s; best={self.best_dist:.2f}m")
                self.end_navigate(); return

        found = self.locate_target()

        # #13: require SIGHT_CONFIRM sightings before committing to the target,
        # so one hallucinated frame doesn't send the robot chasing a ghost.
        # NEW: HOLD STILL between confirmation looks (the old code kept the
        # scan spinning, so look #2 was often past the object and confirmation
        # needed a whole extra revolution). A lost candidate resets the count.
        if found is not None:
            ox, oy, rx, ry, dist = found
            if self.last_obj_xy is None and self.sight_count + 1 < SIGHT_CONFIRM:
                self.sight_count += 1
                self.cmd = Twist()               # freeze; re-check next think-cycle
                return
            announced_before = self.target_announced
            first_live = not self.seen_live
            # #21b: a big unconfirmed jump must not touch our state at all —
            # keep driving to the CURRENT belief and wait for a second look.
            if not self.accept_sighting(ox, oy, dist):        # #30b
                pass
            else:
                self.seen_live = True
                self.goal_fail_count = 0         # #18: a fresh sighting resets failures
                # #19a: if the target itself has MOVED, the old "closest we ever
                # got" is meaningless — restart the progress clock so a walking
                # person (or a re-projected chair) can't trigger a false give-up.
                if (self.last_obj_xy is not None
                        and math.hypot(ox - self.last_obj_xy[0], oy - self.last_obj_xy[1]) > 2 * PROGRESS_EPS):
                    self.best_dist = None
                    self.last_progress_t = time.monotonic()
                    self.goal_tried = []         # a moved target deserves fresh angles
                self.last_obj_xy = (ox, oy)
                self.register_instances(self.target, [(ox, oy)])   # #17: keep memory fresh
                if not self.target_announced:
                    self.notify(f"Found the {self.target} — navigating to it.")
                    self.mlog.log("SIGHT", f"{self.target} at ({ox:.2f}, {oy:.2f}), "
                                           f"{dist:.2f} m away")
                    self.target_announced = True
                elif announced_before and first_live:
                    # we started from MEMORY and the camera has now re-confirmed it
                    self.notify(f"I can see the {self.target} now — it's where I remembered.")
                    self.mlog.log("SIGHT", f"memory-seeded {self.target} re-confirmed at "
                                           f"({ox:.2f}, {oy:.2f}), {dist:.2f} m away")
        elif self.last_obj_xy is None and self.sight_count > 0:
            self.sight_count = 0                 # candidate vanished mid-confirmation

        # arrive by pose (stop even if depth/YOLO drops at close range).
        # #17: be honest — if we only ever knew it from memory and never
        # re-saw it, say so instead of claiming a confirmed arrival.
        if self.last_obj_xy is not None and pose is not None:
            rx, ry, _ = pose
            if math.hypot(rx-self.last_obj_xy[0], ry-self.last_obj_xy[1]) <= STOP_DISTANCE + ARRIVE_TOL:
                self.stop_base()
                if self.seen_live:
                    self.notify(f"Arrived at the {self.target}.")
                else:
                    self.notify(f"I'm at the spot where I last saw the {self.target}, "
                                f"but I can't see it from here right now — it may have "
                                f"moved. Say 'go to the {self.target}' to search fresh.")
                    # stale memory: drop this instance so a fresh search isn't
                    # poisoned by it again
                    known = self.seen_instances.get(self.target, [])
                    self.seen_instances[self.target] = [
                        p for p in known
                        if math.hypot(p[0]-self.last_obj_xy[0], p[1]-self.last_obj_xy[1]) > 0.5]
                    # #23: don't let "go to it" resurrect the same dead point
                    if (self.last_located is not None
                            and self.last_located[0] == self.target
                            and math.hypot(self.last_located[1][0]-self.last_obj_xy[0],
                                           self.last_located[1][1]-self.last_obj_xy[1]) <= 0.5):
                        self.last_located = None
                self.end_navigate(); return

        # #5/#19c: if we know where it is (seen now OR remembered while
        # occluded), pick a stand-off goal and let Nav2 plan the full path
        # AROUND obstacles. We no longer insist on the single point directly
        # between us and the object — we take the cheapest reachable spot on
        # a ring around it.
        if self.last_obj_xy is not None and pose is not None:
            rx, ry, _ = pose
            ox, oy = self.last_obj_xy
            dx, dy = ox - rx, oy - ry
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                return

            # #30a: STICKY GOAL. If the goal we are already driving to is
            # still a sane approach to the (jittering) object position, KEEP
            # it and let Nav2 finish. Re-ranking the ring every cycle
            # preempted Nav2 about once a second — that is the dancing goal
            # marker and the "found it, navigating... but stuck" behaviour.
            if self.nav_goal_xy is not None and self.search_state == "NAVIGATING":
                g0x, g0y = self.nav_goal_xy
                d_obj = math.hypot(g0x - ox, g0y - oy)
                drift = (math.hypot(ox - self.goal_obj_xy[0], oy - self.goal_obj_xy[1])
                         if self.goal_obj_xy is not None else 0.0)
                if (GOAL_STICK_MIN <= d_obj <= GOAL_STICK_MAX
                        and drift <= OBJ_DRIFT_REISSUE
                        and not self.is_goal_tried(g0x, g0y)
                        and self.is_goal_free(g0x, g0y)):
                    return                       # goal still good — don't touch it

            safe = None
            cands = self.approach_candidates(ox, oy, rx, ry)
            if cands:
                safe = cands[0]
            elif self.goal_tried:
                # every ring spot has been refused already — forget the
                # refusals and let Nav2 have another go from where we are now
                self.mlog.log("WARN", f"all approach goals for {self.target} refused; "
                                      f"clearing the tried-list and retrying")
                self.goal_tried = []
                cands = self.approach_candidates(ox, oy, rx, ry)
                safe = cands[0] if cands else None
            if safe is None and self.map is None:
                # no map at all (e.g. Nav2/SLAM not up) — fall back to the
                # old straight-line stand-off rather than refusing to move
                standoff = max(0.0, dist - STOP_DISTANCE)
                safe = (rx + (dx / dist) * standoff, ry + (dy / dist) * standoff)
            if safe is None:
                # #18: the object is deep in unmapped territory. Approach the
                # EDGE of the known map so SLAM extends it; next cycles push
                # the goal further.
                safe = self.farthest_free_along(rx, ry, ox, oy)
            if safe is None:
                # truly nowhere to go from here toward it
                self.cmd = Twist()               # kill any stale scan spin (#18)
                self.goal_fail_count += 1
                if self.goal_fail_count >= GOAL_FAIL_LIMIT:
                    self.notify(f"I can't find a reachable path toward the "
                                f"{self.target} from here — searching again.")
                    self.mlog.log("WARN", f"dropping unreachable sighting of "
                                          f"{self.target} at ({ox:.2f}, {oy:.2f})")
                    self.last_obj_xy = None
                    self.nav_goal_xy = None
                    self.goal_fail_count = 0
                    self.target_announced = False
                    self.sight_count = 0
                    self.search_state = "ROTATE"
                    self.last_yaw = None; self.turn_accum = 0.0
                return
            gx, gy = safe
            # #19c: face the OBJECT from the goal, not just our travel
            # direction — matters now that the goal can be on its far side.
            fdx, fdy = ox - gx, oy - gy
            yaw = math.atan2(fdy, fdx) if math.hypot(fdx, fdy) > 1e-3 \
                else math.atan2(dy, dx)
            if (self.nav_goal_xy is None or
                    math.hypot(gx - self.nav_goal_xy[0], gy - self.nav_goal_xy[1]) > GOAL_REISSUE):
                self.cmd = Twist()
                self.search_state = "NAVIGATING"
                self.nav_goal_xy = (gx, gy)
                self.goal_obj_xy = (ox, oy)      # #30a: for drift tracking
                self.send_goal(gx, gy, yaw)
            return

        # never seen it (enough) yet — scan around
        if self.search_state == "NAVIGATING":
            return
        if self.search_state == "ROTATE":
            self.do_rotate_scan()
        elif self.search_state == "ADVANCE":
            self.do_advance()

    # ── LOCATE: look and REPORT only, never drive to it (#7) ──
    def locate_think(self):
        if not self.camera_ready():
            return
        if self.locate_start and time.monotonic() - self.locate_start > LOCATE_TIMEOUT:
            self.cmd = Twist()
            self.notify(f"I don't see any {self.target} in this area.")
            self.end_locate(); return
        found = self.locate_target()
        if found is not None:
            ox, oy, rx, ry, dist = found
            self.cmd = Twist()
            # #28: report EVERY instance in frame, not just the selected one.
            # "I can see 2 chairs" followed by a single distance was the gap.
            cands, _, _ = self.locate_candidates()
            if not cands:                        # vanished between the two looks
                cands = [(ox, oy, dist, 0.0)]
            self.register_instances(self.target, [(c[0], c[1]) for c in cands])
            self.last_located = (self.target, (ox, oy))            # #23: for "go to it"
            self.mlog.log("MEMORY", f"remembered {len(cands)} {self.target}(s) at "
                                    + ", ".join(f"({c[0]:.2f}, {c[1]:.2f}) d={c[2]:.2f}m"
                                                for c in cands))
            n = len(cands)
            plural = self.target if n == 1 else self.target + "s"
            article = f"a {self.target}" if n == 1 else f"{n} {plural}"
            self.notify(f"Yes — I can see {article} {self.describe_distances(cands)}. "
                        "I'll remember where they are. "
                        "(Locating only — not driving to them.)"
                        if n > 1 else
                        f"Yes — I can see {article} {self.describe_distances(cands)}. "
                        "I'll remember where it is. (Locating only — not driving to it.)")
            self.end_locate(); return
        # not visible yet -> turn in place to look around (no driving toward it)
        t = Twist(); t.angular.z = SEARCH_TURN_SPEED
        self.cmd = t

    # ── MOVE: precise relative turn then translate (#9) ──
    def move_think(self):
        if not self.have_odom or self.move_spec is None:
            return
        spec = self.move_spec

        if spec["phase"] == "rotate":
            if self.move_ref_yaw is None:
                self.move_ref_yaw = self.yaw; self.move_turned = 0.0
            dyaw = abs(math.atan2(math.sin(self.yaw - self.move_ref_yaw),
                                  math.cos(self.yaw - self.move_ref_yaw)))
            self.move_turned += dyaw; self.move_ref_yaw = self.yaw
            if spec["rot_rad"] - self.move_turned <= math.radians(2.0):
                self.cmd = Twist()
                spec["phase"] = "translate"; self.move_ref_pos = None
            else:
                t = Twist(); t.angular.z = ROTATE_SPEED * spec["rot_sign"]
                self.cmd = t
            return

        if spec["phase"] == "translate":
            if abs(spec["dist_m"]) < 1e-3:
                self.finish_move(); return
            if self.move_ref_pos is None:
                self.move_ref_pos = (self.x, self.y)
                # #29: lock the heading NOW. Progress is measured ALONG this
                # axis (signed), not as raw displacement — otherwise a forward
                # shuffle would look like backward progress being undone twice.
                self.move_axis = (math.cos(self.yaw), math.sin(self.yaw))
                self.move_moved = 0.0
                self.backup_best = 0.0
                self.backup_stall_t = None
                self.backup_nudges = 0
                self.nudge_from = None

            going_fwd = spec["dist_m"] > 0
            ax, ay = self.move_axis
            signed = ((self.x - self.move_ref_pos[0]) * ax +
                      (self.y - self.move_ref_pos[1]) * ay)
            # progress TOWARD the goal, always positive when going the right way
            self.move_moved = signed if going_fwd else -signed

            if going_fwd and self.front_min < OBSTACLE_STOP:
                self.cmd = Twist()
                self.notify("There's an obstacle ahead — stopping the move early.")
                self.finish_move(); return
            if self.move_moved >= abs(spec["dist_m"]):
                self.cmd = Twist(); self.finish_move(); return

            # ── #29: reverse ratchet ──
            if not going_fwd:
                # (a) currently shuffling forward to clear the firmware limit?
                if self.nudge_from is not None:
                    crept = math.hypot(self.x - self.nudge_from[0],
                                       self.y - self.nudge_from[1])
                    if crept >= BACKUP_NUDGE_DIST:
                        self.nudge_from = None
                        self.backup_stall_t = None
                        self.backup_best = self.move_moved   # restart the stall watch
                    elif self.front_min < OBSTACLE_STOP:
                        # boxed in: can't reverse (firmware) and can't creep
                        # forward (obstacle). Say so instead of grinding.
                        self.cmd = Twist()
                        self.notify(f"I can't reverse any further and there's something "
                                    f"in front of me, so I can't shuffle forward to reset "
                                    f"the limit. I managed {self.move_moved:.2f} m of the "
                                    f"{abs(spec['dist_m']):.2f} m.")
                        self.mlog.log("MOVE", "backup ratchet blocked front and rear")
                        self.finish_move(); return
                    else:
                        t = Twist(); t.linear.x = MOVE_SPEED
                        self.cmd = t
                        return

                # (b) reversing normally — watch for the firmware cutting us off
                now = time.monotonic()
                if self.move_moved > self.backup_best + BACKUP_STALL_EPS:
                    self.backup_best = self.move_moved
                    self.backup_stall_t = now
                elif self.backup_stall_t is None:
                    self.backup_stall_t = now
                elif now - self.backup_stall_t > BACKUP_STALL_TIME:
                    if self.backup_nudges >= MAX_BACKUP_NUDGES:
                        self.cmd = Twist()
                        self.notify(f"I've hit the reverse limit {self.backup_nudges} times "
                                    f"and only made {self.move_moved:.2f} m of the "
                                    f"{abs(spec['dist_m']):.2f} m. Something may be behind "
                                    f"me — please check, or turn me around and drive forward.")
                        self.mlog.log("MOVE", f"backup ratchet gave up after "
                                              f"{self.backup_nudges} nudges at "
                                              f"{self.move_moved:.2f} m")
                        self.finish_move(); return
                    self.backup_nudges += 1
                    self.nudge_from = (self.x, self.y)
                    print(f"(reverse limit reached at {self.move_moved:.2f} m — "
                          f"shuffling {BACKUP_NUDGE_DIST:.2f} m forward to clear it, "
                          f"then continuing back)")
                    self.mlog.log("MOVE", f"backup limit at {self.move_moved:.2f} m; "
                                          f"nudge #{self.backup_nudges} forward "
                                          f"{BACKUP_NUDGE_DIST} m")
                    t = Twist(); t.linear.x = MOVE_SPEED
                    self.cmd = t
                    return

            t = Twist(); t.linear.x = MOVE_SPEED * (1.0 if going_fwd else -1.0)
            self.cmd = t

    def finish_move(self):
        self.cmd = Twist()
        try: self.cmd_pub.publish(Twist())
        except Exception: pass
        # #29: if the ratchet was used, say so — it's real evidence of the
        # platform constraint being handled, not hidden.
        if self.backup_nudges:
            self.notify(f"Done with that move — I made {self.move_moved:.2f} m in "
                        f"reverse using {self.backup_nudges} forward shuffle"
                        f"{'s' if self.backup_nudges > 1 else ''} to clear the "
                        f"Create 3 backup limit.")
        else:
            self.notify("Done with that move.")
        self.backup_nudges = 0
        self.nudge_from = None
        self.task_id += 1
        self.mode = None
        self.move_spec = None
        self.start_next_step()

    # ── FIND ANOTHER: scan around until a NEW instance appears ──
    def find_more_think(self):
        if not self.camera_ready() or not self.have_odom:
            return
        if time.monotonic() - self.find_start > FIND_TIMEOUT:
            self.cmd = Twist()
            total = len(self.seen_instances.get(self.target, []))
            self.notify(f"I couldn't find another {self.target}. I still count {total}. "
                        "Waiting for further instructions.")
            self.end_find_more(); return

        pair = self.current_pair()
        if pair is not None:
            detections = self.yolo_detect(pair[0])
            if detections is not None:
                positions = self.locate_all(self.target, detections, pair[1])
                added = self.register_instances(self.target, positions)
                total = len(self.seen_instances.get(self.target, []))
                if added > 0 and total > self.find_baseline:
                    self.cmd = Twist()
                    plural = self.target if total == 1 else self.target + "s"
                    self.notify(f"I found another {self.target}. I now count {total} {plural}. "
                                "Waiting for further instructions.")
                    self.end_find_more(); return

        if self.search_state == "ROTATE":
            self.do_rotate_scan()
        elif self.search_state == "ADVANCE":
            self.do_advance()

    def do_rotate_scan(self):
        if self.last_yaw is None:
            self.last_yaw = self.yaw
        dyaw = abs(math.atan2(math.sin(self.yaw - self.last_yaw),
                              math.cos(self.yaw - self.last_yaw)))
        self.turn_accum += dyaw
        self.last_yaw = self.yaw
        t = Twist(); t.angular.z = SEARCH_TURN_SPEED
        self.cmd = t
        if self.turn_accum >= 2 * math.pi:
            self.turn_accum = 0.0; self.last_yaw = None
            if ENABLE_ADVANCE:
                self.advance_start = (self.x, self.y)
                self.search_state = "ADVANCE"
                self.notify(f"Can't see the {self.target} here — moving to look elsewhere.")

    def do_advance(self):
        if self.front_min < OBSTACLE_STOP:
            self.search_state = "ROTATE"; self.last_yaw = None
            self.turn_accum = math.pi; return
        if self.advance_start is not None:
            moved = math.hypot(self.x - self.advance_start[0], self.y - self.advance_start[1])
            if moved >= ADVANCE_DISTANCE:
                self.search_state = "ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
                return
        t = Twist(); t.linear.x = ADVANCE_SPEED
        self.cmd = t

    def end_navigate(self):
        self.task_id += 1
        self.mode = None
        self.search_state = None
        self.target = None
        self.target_qualifier = None
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.sight_count = 0                    # #13
        self.seen_live = False                  # #17
        self.goal_fail_count = 0                # #18
        self.best_dist = None                   # #19a
        self.last_progress_t = None             # #19a
        self.goal_tried = []                    # #19c
        self.jump_pending = None                # #21b
        self.goal_obj_xy = None                 # #30a
        self.navigate_start = None
        self.cmd = Twist()
        self.start_next_step()

    def end_locate(self):
        self.task_id += 1
        self.mode = None
        self.target = None
        self.target_qualifier = None
        self.locate_start = None
        self.cmd = Twist()
        self.start_next_step()

    def end_find_more(self):
        self.task_id += 1
        self.mode = None
        self.search_state = None
        self.target = None
        self.cmd = Twist()
        self.start_next_step()

    # ── EXPLORE / PATROL: frontier mapping with coverage memory (#4) ──
    def explore_think(self):
        if self.frontier_navigating: return
        now = time.monotonic()
        if self.map is None:
            if self.explore_start_t and now - self.explore_start_t > 6 and not self.warned_no_map:
                self.warned_no_map = True
                self.notify("I'm not receiving a /map. Is SLAM running? Launch with slam:=true.")
            return
        pose = self.robot_pose_map()
        if pose is None: return
        rx, ry, ryaw = pose
        clusters = [c for c in self.find_frontiers()
                    if not self.is_blacklisted(c[0], c[1]) and not self.is_visited(c[0], c[1])]
        if not clusters:
            if self.frontier_sent_goal:
                self.finish_explore()
            elif (self.explore_start_t and now - self.explore_start_t > 10
                  and not self.warned_no_frontier):
                self.warned_no_frontier = True
                self.notify("No unmapped area found. If you launched in localization mode the map "
                            "is already complete — relaunch with slam:=true to map fresh.")
            return

        # #4: prefer frontiers AHEAD (penalise turning around) so it stops
        # retreating down the path it just came from.
        def score(c):
            d = math.hypot(c[0] - rx, c[1] - ry)
            ang = math.atan2(c[1] - ry, c[0] - rx)
            turn = abs(math.atan2(math.sin(ang - ryaw), math.cos(ang - ryaw)))
            return d + TURN_WEIGHT * turn
        clusters.sort(key=score)
        gx, gy, _ = clusters[0]
        yaw = math.atan2(gy - ry, gx - rx)
        self.frontier_navigating = True
        self.frontier_sent_goal = True
        self.cur_goal = (gx, gy)
        self.send_goal(gx, gy, yaw)

    def is_visited(self, x, y):
        return any(math.hypot(x - vx, y - vy) < VISITED_RADIUS for vx, vy in self.visited_goals)

    def find_frontiers(self):
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x; oy = m.info.origin.position.y
        grid = np.array(m.data, dtype=np.int16).reshape((h, w))
        free = (grid == 0); unknown = (grid == -1)
        adj = np.zeros_like(unknown)
        adj[1:, :]  |= unknown[:-1, :]
        adj[:-1, :] |= unknown[1:, :]
        adj[:, 1:]  |= unknown[:, :-1]
        adj[:, :-1] |= unknown[:, 1:]
        frontier = free & adj
        ys, xs = np.where(frontier)
        if len(xs) == 0: return []
        bins = {}
        for cy, cx in zip(ys.tolist(), xs.tolist()):
            wx = ox + (cx + 0.5) * res; wy = oy + (cy + 0.5) * res
            key = (round(wx / BIN_SIZE), round(wy / BIN_SIZE))
            bins.setdefault(key, []).append((wx, wy))
        clusters = []
        for pts in bins.values():
            if len(pts) >= MIN_FRONTIER:
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                clusters.append((cx, cy, len(pts)))
        return clusters

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < BLACKLIST_RADIUS
                   for bx, by in self.blacklist)

    def finish_explore(self):
        self.notify("Area fully mapped — no frontiers left. Saving the map.")
        try:
            subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli",
                            "-f", SAVE_MAP_PATH], timeout=30, check=False)
            self.notify(f"Map saved to {SAVE_MAP_PATH}.yaml")
        except Exception as e:
            self.notify(f"Auto-save failed ({e}); save manually with map_saver_cli.")
        self.task_id += 1
        self.mode = None
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.start_next_step()

    # ── NAV2 plumbing ──
    def send_goal(self, x, y, yaw):
        if not self.nav_client.server_is_ready():
            return
        tid = self.task_id
        self.mlog.log("GOAL", f"({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}deg "
                              f"mode={self.mode}")                        # #14
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_goal_response(f, tid))

    def on_goal_response(self, future, tid):
        gh = future.result()
        if tid != self.task_id:
            try: gh.cancel_goal_async()
            except Exception: pass
            return
        if not gh.accepted:
            if self.mode == "explore":
                self.blacklist.append(self.cur_goal); self.frontier_navigating = False
            elif self.mode in ("navigate", "find_more"):
                # #19c: remember WHICH goal was refused so the next think-cycle
                # picks a different spot on the ring instead of re-sending the
                # same illegal goal until the task dies.
                if self.nav_goal_xy is not None:
                    self.goal_tried.append(self.nav_goal_xy)
                self.search_state = "ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
                self.nav_goal_xy = None
                self.goal_fail_count += 1                      # #18
                if self.mode == "navigate" and self.goal_fail_count >= GOAL_FAIL_LIMIT:
                    self.mlog.log("WARN", "nav goal rejected repeatedly — "
                                          "dropping sighting, rescanning")
                    self.last_obj_xy = None
                    self.target_announced = False
                    self.sight_count = 0
                    self.goal_fail_count = 0
                    self.goal_tried = []
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, tid))

    def on_result(self, future, tid):
        if tid != self.task_id:
            return
        status = future.result().status
        self.mlog.log("RESULT", f"nav goal status={status} "
                                f"({'ok' if status == GoalStatus.STATUS_SUCCEEDED else 'not-succeeded'})")
        if self.mode == "explore":
            if status == GoalStatus.STATUS_SUCCEEDED:      # was the magic number 4
                self.consecutive_stucks = 0
                self.visited_goals.append(self.cur_goal)   # #4: remember we covered it
            else:
                self.blacklist.append(self.cur_goal)
            self.frontier_navigating = False
        elif self.mode in ("navigate", "find_more"):
            self.search_state = "ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
            if status != GoalStatus.STATUS_SUCCEEDED:          # #18
                # #19c: Nav2 tried and gave up on THIS spot (no valid path, or
                # its recoveries failed). Rule that spot out and let the next
                # cycle approach the object from another side.
                if self.nav_goal_xy is not None:
                    self.goal_tried.append(self.nav_goal_xy)
                    self.mlog.log("WARN", f"nav goal ({self.nav_goal_xy[0]:.2f}, "
                                          f"{self.nav_goal_xy[1]:.2f}) failed (status={status}); "
                                          f"trying another approach angle")
                self.goal_fail_count += 1
                if self.mode == "navigate" and self.goal_fail_count >= GOAL_FAIL_LIMIT:
                    self.mlog.log("WARN", "every approach to the target failed — "
                                          "dropping sighting, rescanning")
                    self.last_obj_xy = None
                    self.target_announced = False
                    self.sight_count = 0
                    self.goal_fail_count = 0
                    self.goal_tried = []
            else:
                self.goal_fail_count = 0
            self.nav_goal_xy = None
        self.goal_handle = None


def main():
    rclpy.init()
    node = VLAAgent()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_input_loop()
    finally:
        node.cancel_task()
        node.mlog.log("RUN", "agent shutting down")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()