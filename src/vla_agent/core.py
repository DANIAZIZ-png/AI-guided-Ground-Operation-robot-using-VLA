"""Constants, the headless-safe print, and the small module-level helpers.

Extracted verbatim from src/vla_agent_v28.py (lines 307-1131) in Phase 2 step 1.
Nothing here was rewritten -- only moved -- so the comments are the originals
and still carry the reasoning for each value.

WHY `print` LIVES HERE AND MUST BE IMPORTED EXPLICITLY
    The module-level `def print(...)` below is agent fix #63, the headless-safe
    console. A module-level def shadows the builtin ONLY inside its own module,
    so any module that wants that behaviour has to import this one's `print`
    by name. Forgetting to is silent: output reverts to the builtin and the
    broken-pipe crash it fixes comes back.
"""

# The version banner. Printed at startup, written into every mission log and
# published in /vla/status, so it is what ties a recorded run to the code that
# produced it. It lives here because core.py's MissionLog, ros_io.py and
# state_machine.py all reference it.
AGENT_VERSION = ("vla_agent v28  (#63 headless-safe console output, "
                 "#62 direction-aware collision guard, "
                 "#61 nearest-cluster LiDAR ranging, "
                 "#60 manual override teleop, "
                 "#59 10 Hz collision guard, "
                 "#58 smoother approach, "
                 "#57 stand off far enough to keep the object in view, "
                 "#56 step-and-stare scan, "
                 "#55 LiDAR failure reasons reported, "
                 "#54 LiDAR object ranging, "
                 "#53 seeded depth intrinsics, "
                 "#52 depth intrinsics rescaled to frame size, "
                 "#51 compressedDepth transport, "
                 "#50 compressed RGB transport, "
                 "#49 annotated feed gated per topic, "
                 "#47 TF at colour-frame stamp, "
                 "#48 reject stale depth, #46 one subscription per camera stream, "
                 "#43 longer cached action-server wait, "
                 "#44 undock retry-loop breaker, #45 sync slop for 1 Hz depth, "
                 "#37 /robot1 namespace, #38 stereo depth topic, "
                 "#39 best-effort sensor QoS, #40 RGB->depth pixel mapping, "
                 "#41 encoding-based mm/m, #42 explicit namespaced TF, "
                 "#36 no patrol feed prompt, "
                 "#35 cancel actually stops the robot, "
                 "#34 feed publishes to any subscriber, "
                 "#22 dock no-detect zone, #23 live-first memory, "
                 "#24 forget, #25 typo-tolerant control words, #26 bare yes/no, "
                 "#27 context expiry, #28 all-instance distances, "
                 "#29 Create 3 backup ratchet, #30 sticky goals + range-scaled "
                 "gate + ring-safe blacklist, #31 incremental off-map hop goals, "
                 "#32 non-freezing live feed, #33 command/reply/status bridge)")

import base64
import datetime
import difflib
import hashlib
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time

import vla_paths              # VLA_ROOT-derived paths; see src/vla_paths.py

# ── #63 HEADLESS-SAFE CONSOLE OUTPUT ───────────────────────────────
# BUG THIS FIXES, observed through the operator console:
#     ROBOT [undock] Undocking.
#     ROBOT Something went wrong handling that command: [Errno 32] Broken pipe
# When the GUI launches the agent there is no terminal. The GUI reads the
# agent's stdout into its log pane, and the moment that pane's pipe is
# closed (GUI restarted, step stopped, buffer full) EVERY subsequent
# print() raises BrokenPipeError. #33 already handled input() -- it does
# not read a keyboard when there is no tty -- but it left all 55 print()
# calls unguarded, and notify() prints on every single robot message.
# So the exception was raised from inside the command handler, propagated
# up, and killed the command: undock announced itself and then died.
#
# Console output is COSMETIC. It must never be able to abort a robot
# action. The real output paths are /vla/reply and /vla/status, which are
# DDS topics and completely unaffected by a closed pipe.
#
# Rather than wrap 55 call sites -- and every future one -- print itself is
# replaced. Any code in this file that prints is now safe by construction.
_real_print = print
# keep a reference to the genuine builtin before shadowing the name

def print(*args, **kwargs):
    """print() that can never raise. Console output is never worth a crash."""
    try:
        _real_print(*args, **kwargs)
        # normal path: behaves exactly like the builtin
    except (BrokenPipeError, ValueError, OSError):
        # BrokenPipeError: the reader (GUI log pane) closed the pipe
        # ValueError    : "I/O operation on closed file" after shutdown
        # OSError       : parent of BrokenPipeError; catches EPIPE variants
        pass
        # swallow it. The message is lost; the robot keeps working.

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy,
                       QoSReliabilityPolicy, DurabilityPolicy)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup   # #32b
from rclpy.executors import MultiThreadedExecutor                  # #32b
from sensor_msgs.msg import Image, CameraInfo, LaserScan, CompressedImage
from std_msgs.msg import String                                    # #33
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
# #35: the ACTION-level cancel service. A default-constructed request has a
# zero goal-id and a zero timestamp, which the action spec defines as
# "cancel EVERY goal on this server" - stronger than cancelling one handle.
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage                                 # #42: manual TF subscription
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

# ── #37 ROBOT NAMESPACE ──────────────────────────────────────────
# The real TurtleBot 4 publishes EVERYTHING under a namespace (/robot1 by
# default). In simulation there was no namespace, so every topic name in
# v12 was written bare ("/scan", "/cmd_vel"). On the real robot those
# names do not exist: the agent would start cleanly, print no error, and
# silently receive nothing and drive nothing. This is the single most
# important hardware fix in this file.
#   Override without editing code:  export VLA_NS=/robot2
#   Run against an un-namespaced robot: export VLA_NS=""
NS = os.environ.get("VLA_NS", "/robot1").rstrip("/")
# read the namespace from an environment variable, defaulting to /robot1, and strip any trailing slash so "/robot1/" and "/robot1" behave identically

RGB_TOPIC   = NS + "/oakd/rgb/preview/image_raw"
RGB_COMPRESSED_TOPIC = RGB_TOPIC + "/compressed"
# #50: the JPEG version of the same colour stream, ~18x smaller on the wire.
# This is what the agent subscribes to by default on real hardware.
USE_COMPRESSED_RGB = os.environ.get("VLA_RAW_RGB", "0") != "1"
DEPTH_CFG_HEADER_BYTES = 12
# #51: size of compressed_depth_image_transport's ConfigHeader that prefixes
# the PNG payload: one int32 format enum plus two float32 quantisation values
# set VLA_RAW_RGB=1 to go back to the raw stream (useful in simulation, where
# bandwidth is free and the compressed topic may not exist)
# colour image, 250x250 on the real OAK-D (much smaller than the sim's 750x750)
DEPTH_TOPIC = NS + "/oakd/stereo/image_raw"
DEPTH_COMPRESSED_TOPIC = DEPTH_TOPIC + "/compressedDepth"
# #51: the driver's oakd_lite.yaml sets stereo.i_low_bandwidth: true, which
# makes it encode depth ON THE CAMERA and publish it here. The raw
# stereo/image_raw topic is then only a slow by-product of that pipeline.
# Measured over the same 20 s window on the real robot: 3 raw frames vs 48
# compressed frames - 16x more data available on the topic we were ignoring.
# This, not Wi-Fi bandwidth, is why depth ran at 0.15 Hz while raw colour ran
# at 13 Hz, and why a chair 1.5 m away was reported at 6.4 m and then 12.8 m:
# robust_box_depth was reading a frame captured many seconds earlier from a
# different pose, so it measured the wall behind the chair.
USE_COMPRESSED_DEPTH = os.environ.get("VLA_RAW_DEPTH", "0") != "1"
# set VLA_RAW_DEPTH=1 to go back to the raw topic (simulation publishes only that)
# #38: the real OAK-D publishes depth under stereo/, NOT rgb/preview/depth.
# rgb/preview/depth simply does not exist on hardware, so v12's depth
# subscription would never have fired even with the namespace corrected.
INFO_TOPIC  = NS + "/oakd/rgb/preview/camera_info"
# intrinsics (K) of the COLOUR image — this is what turns a pixel into a 3D ray
DEPTH_INFO_TOPIC = NS + "/oakd/stereo/camera_info"
# #53: stereo/camera_info is published by the RAW depth publisher, which the
# low-bandwidth pipeline runs at ~0.15 Hz. The agent consumes compressedDepth
# at ~2.4 Hz, so it can process HUNDREDS of depth frames before the intrinsics
# ever arrive - and until they do, rgb_px_to_depth_px falls back to crude
# proportional scaling. That fallback scales the vertical axis by h/250 (2.88)
# instead of the true 5.12 AND drops the optical-centre offset entirely, so it
# samples the wrong rows of the depth image. Objects were measured against
# whatever happened to be higher or lower in the scene, which is why a chair
# 1.5 m away read 6.4 m, then 7.7 m, then "distance unclear".
#   These defaults are the values read off this robot's own camera
# (/robot1/oakd/stereo/camera_info) for its native 1280x720 depth image, so
# the mapping is correct from the very first frame. A live camera_info always
# overrides them, and #52 rescales whichever set is in use to the actual frame
# size, so nothing here is hard-wired to one resolution.
DEPTH_K_DEFAULT = [1012.45264, 0.0, 649.956177,
                   0.0, 1012.45264, 347.630371,
                   0.0, 0.0, 1.0]
DEPTH_WH_DEFAULT = (1280, 720)

# #54: LiDAR ranging of detected objects.
USE_LIDAR_RANGE = os.environ.get("VLA_NO_LIDAR_RANGE", "0") != "1"
# set VLA_NO_LIDAR_RANGE=1 to force the old depth-only behaviour
# #64 (11 Sep, HARDWARE ONLY): VLA_RGB_ONLY=1 lets current_pair() return
# (rgb, None) when no depth frame has ever arrived, so the camera gate opens
# on colour alone and YOLO + #54 LiDAR ranging run. Reason: on the real
# OAK-D Lite the stereo pipeline costs the Pi +7 C and makes colour frames
# arrive ~4 s late (10 Sep), while colour-only frames arrive in ~50 ms
# (11 Sep); depth was only ever the FALLBACK ranging source behind the
# LiDAR. Default "0" = exactly the previous behaviour (simulation).
RGB_ONLY = os.environ.get("VLA_RGB_ONLY", "0") == "1"
LIDAR_BEARING_GUESS_M = 3.0
# starting distance used to estimate the bearing before the range is known;
# one refinement pass afterwards removes the error this introduces
LIDAR_MIN_HALF_SPAN = math.radians(1.0)
# never sample a span narrower than this, or a distant object gets too few beams
LIDAR_MAX_HALF_SPAN = math.radians(12.0)
# ── #61 NEAREST-CLUSTER RANGING (replaces the median) ──────────────
# THE BUG: scan_range_at() used to return the MEDIAN of every beam inside
# the detection box's angular span. That silently assumed the object fills
# most of the span. At LiDAR height a chair is not a solid 0.45 m surface --
# it is FOUR THIN LEGS. Most beams sail between them and hit the wall
# behind, so the median reported the WALL. On hardware that produced a
# chair "at 3.4 m" whether it truly stood at 2 m or at 1 m: a constant, not
# a distance, because the wall never moved.
# THE FIX: sort the beams by range and take the NEAREST coherent group.
# Beams landing on the chair legs sit close together in range; beams that
# miss jump to a much larger value. That jump is the boundary between the
# object and whatever is behind it.
LIDAR_CLUSTER_GAP   = 0.30   # m -- a range jump bigger than this starts a
                             # new surface. Wider than the depth of a chair
                             # or a person, narrower than the usual gap
                             # between an object and the wall behind it.
LIDAR_CLUSTER_MIN   = 2      # beams -- a genuine surface returns at least
                             # two adjacent beams; a single short return is
                             # dust, a reflection, or a passing hand.
# never wider than this, or a large nearby box swallows its neighbours
# #40: intrinsics of the DEPTH image. Depth is 1280x720 while colour is
# 250x250, so a YOLO pixel cannot index the depth array directly. Having
# both K matrices lets us compute the mapping at runtime instead of
# hard-coding a scale factor that would break on a different camera config.
SCAN_TOPIC  = NS + "/scan"
# RPLIDAR 2D laser scan, used for obstacle stopping
ODOM_TOPIC  = NS + "/odom"
# wheel odometry, used for relative moves and for confirming the robot has stopped
CMD_TOPIC   = NS + "/cmd_vel"
# velocity commands. SAFETY-CRITICAL: fix #35's zero-barrage publishes here,
# so a wrong name means "stop" silently does nothing while the robot drives on.
MAP_TOPIC   = NS + "/map"
# occupancy grid from SLAM Toolbox
TF_TOPIC        = NS + "/tf"
# #42: TF is namespaced too. tf2_ros.TransformListener subscribes to a bare
# "/tf", so on the real robot the transform buffer stayed permanently empty
# and every map-frame projection failed.
TF_STATIC_TOPIC = NS + "/tf_static"
# static transforms (camera mounting, wheel offsets) — published once, latched
NAV_ACTION      = NS + "/navigate_to_pose"
# Nav2's navigation action server, also namespaced on the real robot
NAV_CANCEL_SRV  = NS + "/navigate_to_pose/_action/cancel_goal"
# the cancel-all service belonging to that action server (#35)
DOCK_ACTION     = NS + "/dock"
# Create 3 docking action
UNDOCK_ACTION   = NS + "/undock"
# Create 3 undocking action
DOCK_STATUS_TOPIC = NS + "/dock_status"
# tells the agent whether it is currently sitting on the charger
ANNOT_TOPIC = "/vla/annotated"          # #20: RGB + YOLO boxes, for the live window
# #32d: the same annotated frame as JPEG. A 750x750 BGR image is ~1.7 MB;
# the JPEG is ~50 kB. Use THIS one over Wi-Fi and in the GUI.
ANNOT_COMPRESSED_TOPIC = ANNOT_TOPIC + "/compressed"
# #33: command bridge — voice node, GUI and any other client speak to the
# agent through these three topics and nothing else.
CMD_IN_TOPIC   = "/vla/command"         # std_msgs/String  (in)  a command
REPLY_TOPIC    = "/vla/reply"           # std_msgs/String  (out) what the agent says
STATUS_TOPIC   = "/vla/status"          # std_msgs/String  (out) JSON state, 2 Hz
MAP_FRAME   = "map"
ROBOT_FRAME = "base_link"

# ── #57 STOP FAR ENOUGH BACK THAT THE CAMERA CAN STILL SEE THE OBJECT ──
# The colour camera's field of view is 64.6 deg, worked out from the REAL
# measured intrinsics: 2*atan(cx/fx) = 2*atan(126.94/197.74) = 65.4 deg.
# A 0.9 m tall chair standing 0.6 m away fills 2*atan(0.45/0.6) = 73 deg of
# that view -- MORE than the camera's 65.4 deg -- so its top and bottom are
# cut off and YOLO-World can no longer recognise the shape. Detection died
# at exactly the moment the robot needed it to confirm arrival.
# at exactly the moment the robot needed it to confirm arrival. At 1.0 m the
# same chair fills 2*atan(0.45/1.0) = 48 deg, comfortably inside 65.4 deg, so
# the object stays recognisable all the way in.
# #64d (14 Sep, HARDWARE ONLY): VLA_STOP_DISTANCE / VLA_STANDOFFS override the
# 1.0 m stand-off. The operator wants the robot AT the object, not a metre
# short. Nav2 rejects goals closer than robot_radius (0.175) to an obstacle and
# inflates to 0.45 m, so 0.45 m from the object's laser points is the closest
# accepted goal (front bumper ~0.25-0.30 m from a chair). The camera loses a
# chair from view below ~1 m (#57); the final approach is pose-based, so that
# is acceptable. Defaults "1.0" / "1.00,1.30" = exactly the previous behaviour.
STOP_DISTANCE     = float(os.environ.get("VLA_STOP_DISTANCE", "1.0"))
# #64b (11 Sep, HARDWARE ONLY): VLA_ARRIVE_TOL overrides the 0.35 m slack.
# On the real robot Nav2's xy_goal_tolerance (0.25 m) lets it park up to
# 1.55 m from the belief when the 1.30 m stand-off is chosen, so the arrival
# test (STOP_DISTANCE + ARRIVE_TOL = 1.35 m) never passes and the robot sits
# at its goal re-sending it every second (two of three runs, 11 Sep). The
# hardware launcher sets 0.60. Default "0.35" = exactly the previous behaviour.
ARRIVE_TOL        = float(os.environ.get("VLA_ARRIVE_TOL", "0.35"))
# #64e (15 Sep, HARDWARE ONLY): VLA_ARRIVE_IF_SEEN_M — "if I can see it and it
# is this close, I have arrived." When the target is in the CURRENT camera
# frame and its measured (LiDAR) distance is at or under this value, the
# agent stops and announces arrival at once, instead of planning a stand-off
# goal. Reason: a person standing near furniture leaves no ring goal outside
# Nav2's 0.45 m inflation, every goal is refused, and the agent then drops
# the sighting and rescans -- driving away from a person one metre in front
# of it (20 refusals in a row, 15 Sep). Default "0" = disabled = previous
# behaviour (simulation). The hardware launcher sets 1.2.
ARRIVE_IF_SEEN_M  = float(os.environ.get("VLA_ARRIVE_IF_SEEN_M", "0"))
# ── #59 COLLISION GUARD (replaces the weak v24 version) ────────────
# v24 ran this check only inside the 1 Hz think-loop, and only once the
# BELIEVED gap to the target was already under 1.7 m. On hardware it fired
# at 0.38 m -- the robot had physically reached the chair. Two reasons it
# was too late, both fixed here:
#   (a) 1 Hz is not fast enough. At 0.26 m/s the robot travels 0.26 m
#       between checks, so a guard set at 0.55 m can be at 0.29 m before it
#       ever runs. The guard now has its OWN 10 Hz timer.
#   (b) It was gated on the believed distance to the target. That belief
#       goes stale exactly when it matters: YOLO loses the object at close
#       range, last_obj_xy freezes at the last (over-estimated) position,
#       and the robot drives past the real object toward a phantom one.
#       The guard is now UNCONDITIONAL while under power -- it does not
#       care what the robot believes, only what the LiDAR measures.
# The Create 3 is 0.34 m across, so 0.70 m leaves two body-widths of clear
# space -- enough that a 10 Hz guard can stop the robot before contact even
# at full speed.
# #64c (14 Sep, HARDWARE ONLY): VLA_MIN_FRONT_CLEAR overrides the guard
# distance. In a crowded room every relative move was stopped at 0.66-0.70 m by
# people standing nearby (three stops in one minute, 14 Sep); Nav2's inflation
# layer and the Create 3 bumpers remain. The hardware launcher sets 0.50.
# Default "0.70" = exactly the previous behaviour.
MIN_FRONT_CLEAR   = float(os.environ.get("VLA_MIN_FRONT_CLEAR", "0.70"))

# #65 (24 Sep, HARDWARE ONLY): which LASER-FRAME bearing points at the robot's
# FRONT. The RPLIDAR on this TurtleBot 4 is bolted on rotated: TF says
# base_link -> rplidar_link is yaw +90 deg, so the beam the scan calls 0 deg
# looks out of the robot's LEFT side. Both forward guards below sliced the scan
# around raw 0 deg, i.e. they watched the left flank and braked for whatever
# stood beside the robot -- measured 24 Sep: guards saw 0.49 m (a desk on the
# left) while the actual path ahead was 2.30 m clear. That is the "there's an
# obstacle ahead" with nothing ahead, and the reason MIN_FRONT_CLEAR was
# lowered to 0.35 on 14 Sep to work around it.
# Object RANGING was never affected: cam_bearing_in_laser() converts camera
# pixels through TF into the laser's own frame, so it was always consistent.
# Default "0" = exactly the old arithmetic, so the simulation is unchanged.
SCAN_FWD_RAD      = math.radians(float(os.environ.get("VLA_SCAN_FWD_DEG", "0")))

def _off_front(ang):
    """Angular distance from the robot's FRONT, for a raw laser-frame angle."""
    d = ang - SCAN_FWD_RAD
    return abs(math.atan2(math.sin(d), math.cos(d)))
# The v24 guard measured the minimum of a +/-15 deg cone. At 1 m that cone is
# only +/-0.27 m wide -- NARROWER THAN A CHAIR -- so the beams pass between
# the legs and read the wall behind. A wider arc actually intersects the legs.
GUARD_ARC_DEG     = 40.0
# Use the Nth-smallest beam rather than the single smallest, so one spurious
# short return (dust, a reflection, a passing hand) cannot brake the robot.
# N=3 still catches a chair leg, which spans several beams at 0.7 m.
GUARD_MIN_BEAMS   = 3
GUARD_PERIOD      = 0.1     # s -- the guard's own timer, 10x the think-loop
# ── #62 THE GUARD MUST BE DIRECTION-AWARE ──────────────────────────
# HARDWARE FAILURE this fixes: the robot came to rest ~0.68 m from a wall,
# inside MIN_FRONT_CLEAR. From then on EVERY command died instantly:
#     [move] Turning ninety degrees to the right.
#     [robot] Stopping -- an obstacle is 0.69 m ahead.
# It could not turn away and it could not reverse away. The guard had
# trapped the robot in the exact situation it existed to prevent.
#
# Why: v26 exempted rotation with `if self.search_state == "ROTATE"`, which
# is set ONLY by the search state-machine inside navigate/find_more. A
# `turn right 90` is a RELATIVE MOVE -- mode == "move", search_state is
# None -- so the exemption never applied.
#
# A front-facing guard is only meaningful against FORWARD motion. Rotation
# in place sweeps no new ground ahead; reversing moves AWAY from the thing
# being measured. #59's principle -- the guard must not depend on the belief
# it protects against -- is untouched: direction of travel is not a belief,
# it is the command the agent itself issued one tick ago.
#
# The catch: when Nav2 drives, the agent does NOT own self.cmd (Nav2
# publishes to /cmd_vel directly and self.cmd sits at a stale zero). Gating
# naively on self.cmd.linear.x > 0 would silently DISABLE the guard during
# real navigation -- the case that matters most. So the guard first asks who
# is holding the wheel, and only trusts self.cmd when the answer is "we are".
GUARD_MIN_FWD_SPEED = 0.02  # m/s -- below this, forward motion is noise, not
                            # travel. Guards against a float that is 1e-17
                            # instead of a clean 0.0.
# ── #56 STEP-AND-STARE SCAN ────────────────────────────────────────
# Old behaviour: spin continuously at 0.5 rad/s. The think-loop runs once a
# second, so the robot swept 0.5 rad = 28.6 deg between two looks -- nearly
# HALF the 65.4 deg field of view -- and every frame it did look at was
# motion-blurred by the spin. Objects fell into the gap between looks.
# New behaviour is what a human sentry does: turn a little, STOP, look
# properly, turn again. Slower turning shrinks the blur; the pause gives the
# laggy camera time to deliver a sharp frame and YOLO time to run on it.
SEARCH_TURN_SPEED = 0.35     # rad/s (was 0.5) -- about 20 deg per think-cycle
SCAN_STEP_RAD     = 0.61     # rad (~35 deg) of turn before each pause
SCAN_DWELL_S      = 3.5      # s spent stationary looking, after every step
# Step 35 deg inside a 65.4 deg view means consecutive looks OVERLAP by about
# 30 deg, so nothing can hide in a seam between two frames. The 2.5 s dwell is
# sized against THINK_PERIOD (1.0 s): it guarantees at least two, usually
# four, detection passes on a sharp stationary image at every step -- the
# earlier 1.6 s gave only one, so most frames were still blurred by the turn.
# 360/35 = ~10 steps, so a full scan takes about 55 s. Slower on purpose: a
# scan that misses the target is not faster, it just fails sooner.
ADVANCE_SPEED     = 0.20
ADVANCE_DISTANCE  = 1.5
OBSTACLE_STOP     = 0.6
ENABLE_ADVANCE    = True

# #58: was 0.5. Every trigger CANCELS the path Nav2 is driving and starts
# a new one -- the robot decelerates, replans, accelerates again. That
# stop-start IS the jerky motion, and each cancel also throws away the
# controller's smoothing history, which is how it ends up overshooting.
# Raising the threshold means small frame-to-frame jitter in the believed
# object position no longer interrupts a perfectly good path; only a
# genuine move of the object does.
GOAL_REISSUE        = 0.9    # re-send a nav goal only if the target moved > this (m)
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
# #56: raised from 45 s. A step-and-stare revolution takes ~50 s by design,
# so the old timeout would have killed the search BEFORE it had turned the
# full circle once -- the robot would have given up facing away from a target
# it was about to find. This must always exceed one revolution.
SEARCH_TIMEOUT      = 120.0  # never saw it -> give up (s)
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
# #57: raised alongside STOP_DISTANCE so the robot parks where the object
# is still inside the camera's field of view and can be confirmed.
APPROACH_STANDOFFS  = tuple(float(x) for x in
                            os.environ.get("VLA_STANDOFFS", "1.00,1.30").split(","))   # #64d
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

# ── #31: INCREMENTAL ("HOP") GOALS INTO UNMAPPED SPACE ──────────────
# Everything above assumes the object sits inside the SLAM'd area. When it
# does not, no legal full goal exists (is_goal_free() needs KNOWN-free cells
# and unmapped cells are -1), so the robot could never cross its own map
# frontier toward a target it could plainly see. Hop mode sends a SHORT goal
# along the bearing to the object, allowed to end in unknown-but-not-occupied
# space, and repeats. Each hop grows the map, so the next hop is planned on
# better information — this is deliberately a "crawl", not a leap of faith.
#   Why unknown is safe to enter for 1.5 m and not for 15 m:
#     * Nav2's global planner already accepts unknown cells (allow_unknown);
#       what it will NOT do is invent a goal for us.
#     * The LiDAR-fed LOCAL costmap sees real obstacles inside ~3 m in real
#       time, so a short hop is covered by live sensing end to end.
#     * Stereo depth error grows with range. At 10 m the projected object
#       position can be metres off; committing 1.5 m at a time and re-ranging
#       costs almost nothing when the estimate turns out to be wrong.
STEP_GOAL_DIST      = 1.5    # m — nominal length of one hop
STEP_GOAL_MIN       = 0.60   # m — shorter than this is not worth a Nav2 goal
STEP_ARRIVE_TOL     = 0.45   # m — this close to the hop goal = plan the next one
STEP_CLEARANCE      = 0.22   # m — free space required around a hop goal. Slightly
                             #   under ROBOT_CLEARANCE: hops are short and under
                             #   live LiDAR cover, so we can afford tighter gaps.
# Sidestep angles tried (deg, off the robot->object bearing) when the straight
# hop is blocked. Nav2 plans around obstacles; this only moves the END POINT
# off a wall so a legal goal exists at all.
STEP_FAN_DEG        = (0, 20, -20, 40, -40, 60, -60)
STEP_REPLAN_BEARING = 35.0   # deg — object drifted this far off the hop bearing
                             #   -> stop crawling that way and re-aim
STEP_TIMEOUT        = 25.0   # s — one 1.5 m hop should never take this long
STEP_FAIL_LIMIT     = 8      # consecutive failed hops before we drop the sighting
                             #   (more generous than GOAL_FAIL_LIMIT: hops are
                             #   exploratory by nature and cheap to retry)
STEP_MAX_RANGE      = 25.0   # m — refuse to hop toward a "sighting" further than
                             #   this. Beyond ~25 m an OAK-D depth reading is
                             #   noise, and chasing it would walk the robot out
                             #   of the building.

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

# #44: how many consecutive undock attempts may fail before the agent gives
# up on the queued task. Without this the auto-undock branch re-queues the
# pending step forever and the terminal fills with the same two messages.
UNDOCK_FAIL_LIMIT = 3

# #12: RGB-depth pairing tolerance.
# #45 HARDWARE: measured on the real robot, RGB arrives at ~15 Hz but depth
# only at ~1 Hz, because a raw 1280x720 16-bit depth frame is 1.84 MB and the
# Wi-Fi link cannot carry more. With a 0.1 s window a 15 Hz stream and a 1 Hz
# stream almost never land close enough together, so the synchronizer produced
# no pairs at all and the agent fell back to unsynchronized frames every time.
# 0.6 s comfortably covers one depth period while still rejecting frames that
# are a whole second stale.
SYNC_SLOP_S    = 0.6
SYNC_QUEUE     = 3
# #48: largest colour-to-depth capture gap that may still be projected to a
# map position. Beyond this the range reading describes where the object WAS.
MAX_PAIR_SKEW_S = 2.0
# #46: how many frames the synchronizer buffers per stream. Ten was pointless
# once the slop window covers more than one depth period, and the extra
# buffering only delayed pairs.
SYNC_WARN_S    = 6.0        # warn if raw frames flow but no synced pair after this long

# #15: don't spam "server down" more than once per this many seconds
YOLO_ERR_PERIOD = 10.0

# #14: mission logs (thesis evidence) live here, one file per run.
# VLA_LOG_DIR (config/paths.sh) wins; otherwise <repo>/logs.
LOG_DIR = vla_paths.LOG_DIR

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
SAVE_MAP_PATH    = os.path.join(vla_paths.MAP_DIR, "warehouse_map")

STUCK_DIST = 0.10      # moved less than this...
STUCK_TIME = 15.0      # ...for this many seconds while driving = stuck
MAX_STUCKS = 3         # give up exploring after this many stucks in a row

THINK_PERIOD = 1.0
PUB_PERIOD   = 0.1

# ── #35: MAKING "STOP" ACTUALLY STOP ────────────────────────────────
# While navigating, the wheels are driven by NAV2, not by this agent. Nav2's
# controller publishes to /cmd_vel at about 20 Hz. The old cancel fired
# goal_handle.cancel_goal_async() without checking the result and published
# THREE zero-velocity messages - which Nav2's very next tick overwrote. The
# operator saw "Task cancelled" while the robot kept driving at 0.26 m/s.
# Three changes fix it:
#   (a) cancel EVERY goal on the action server, not just our stored handle
#   (b) hold /cmd_vel at zero for a sustained barrage, faster than Nav2
#       publishes, so any in-flight command is overwritten rather than
#       merely competed with
#   (c) do not claim the robot has stopped until ODOMETRY says it has
STOP_BARRAGE_S       = 2.0    # s - how long to hold /cmd_vel at zero
STOP_BARRAGE_PERIOD  = 0.02   # s - 50 Hz, faster than Nav2's ~20 Hz
STOP_LIN_EPS         = 0.02   # m/s below this counts as stopped
STOP_ANG_EPS         = 0.05   # rad/s below this counts as stopped
STOP_CONFIRM_TICKS   = 3      # consecutive still readings before we believe it
STOP_CONFIRM_TIMEOUT = 6.0    # s - after this, report honestly that it did NOT stop

# #20/#32 annotated feed
ANNOTATED_FEED  = True     # False -> open the plain raw camera topic instead
# #32c: the feed no longer runs YOLO itself, so a frame costs only a draw and
# a JPEG encode (~3 ms). 0.2 s = 5 Hz, which actually looks live.
# #64f (21 Sep, HARDWARE ONLY): VLA_ANNOT_PERIOD overrides the 0.2 s (5 Hz) cap.
# The feed is PC-local (agent -> GUI) and a frame costs ~3 ms, so 30 Hz is free
# and makes the operator console as smooth as the raw stream, WITH the boxes.
# Default "0.2" = exactly the previous behaviour (simulation unchanged).
ANNOT_PERIOD    = float(os.environ.get("VLA_ANNOT_PERIOD", "0.2"))   # s between annotated frames
ANNOT_MIN_WIDTH = 750      # upscale narrow preview frames to at least this wide (px)
ANNOT_MAX_SCALE = 3        # #32d: never upscale more than this (bytes on the wire)
# #32c: reuse the think-loop's detections for at most this long. Older than
# this and the boxes would no longer describe what the camera is looking at,
# so we show clean video instead of lying with stale rectangles.
ANNOT_DET_TTL   = 1.5      # s
ANNOT_JPEG_Q    = 80       # #32d: JPEG quality for /vla/annotated/compressed

# #32a: a synchronized (rgb, depth) pair older than this is STALE and must not
# be reused. Without this the agent republished — and navigated on — a frozen
# frame whenever the RGB/depth synchronizer stopped matching, which on real
# hardware happens for seconds at a time. 1.5 s is ~1 think-cycle of grace.
PAIR_STALE_S    = 1.5
# #32e: no new camera frame at all for this long = say so once, don't sit mute.
CAM_STALL_WARN  = 5.0

# #33: how often the JSON status packet goes out for the GUI (s)
STATUS_PERIOD   = 0.5

QUIT_WORDS = ("quit", "exit")
STOP_WORDS = ("cancel", "stop", "halt", "abort")

# ── #60 MANUAL OVERRIDE (operator teleoperation) ───────────────────
# An autonomous ISR platform must be handable back to a human at any moment:
# to recover it from a spot the planner cannot solve, to inspect something
# the autonomy did not think worth approaching, or simply to drive it out of
# the way. These phrases hand the operator the wheel. They are matched
# DETERMINISTICALLY, before the LLM ever sees them -- the LLM was answering
# "manual override" with "not supported", because taking control is a system
# function, not something to reason about.
MANUAL_WORDS = ("manual override", "manual control", "take control",
                "manual", "teleop", "override", "i'll drive", "let me drive",
                "give me control", "manual mode")
# Standard ROS teleop_twist_keyboard layout, so anyone who has driven a ROS
# robot already knows it.
MANUAL_KEYS = {
    "u": ( 1.0,  1.0),   # forward + left
    "i": ( 1.0,  0.0),   # forward
    "o": ( 1.0, -1.0),   # forward + right
    "j": ( 0.0,  1.0),   # turn left on the spot
    "k": ( 0.0,  0.0),   # stop
    "l": ( 0.0, -1.0),   # turn right on the spot
    "m": (-1.0, -1.0),   # reverse + left  (reversed steering, as when driving backwards)
    ",": (-1.0,  0.0),   # reverse
    ".": (-1.0,  1.0),   # reverse + right
}
MANUAL_LIN_STEP  = 0.10     # m/s added/removed by '+'/'-'
MANUAL_ANG_STEP  = 0.20     # rad/s added/removed by '+'/'-'
MANUAL_LIN_MAX   = 0.26     # m/s -- the Create 3's own limit, do not exceed
MANUAL_ANG_MAX   = 1.00     # rad/s
# A key press commands motion for this long, then the robot stops on its own.
# This is a DEAD-MAN behaviour: if the operator walks away, or the terminal
# loses focus, or an SSH session drops, the robot halts instead of driving on.
MANUAL_HOLD_S    = 0.35
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


def ang_norm(a):
    """Wrap an angle to (-pi, pi]. Needed by #31: comparing two bearings
    without this makes 179 deg and -179 deg look 358 deg apart."""
    return math.atan2(math.sin(a), math.cos(a))


# ── #33: mirror the terminal onto /vla/reply ────────────────────────
class ReplyTee:
    """Everything the agent prints also goes out on /vla/reply.

    Why a stdout mirror instead of editing forty print() calls: the agent
    speaks from dozens of places (notify, step announcements, error paths,
    the brain's 'thinking...' line). Touching each one is forty chances to
    miss one and leave the GUI operator staring at a robot that said
    something only the terminal heard. Wrapping the stream captures all of
    them at once and changes no existing behaviour — the terminal still
    prints exactly what it printed before.

    Only complete lines are published, and the 'Command> ' prompt (which is
    written without a newline) is stripped so the GUI transcript stays clean.
    """

    def __init__(self, stream, publish):
        self.stream = stream
        self.publish = publish
        self.buf = ""
        self.lock = threading.Lock()

    def write(self, s):
        self.stream.write(s)
        try:
            with self.lock:
                self.buf += s
                lines = []
                while "\n" in self.buf:
                    line, self.buf = self.buf.split("\n", 1)
                    lines.append(line)
            for line in lines:
                line = line.replace("Command> ", "").strip()
                if line:
                    self.publish(line)
        except Exception:
            pass                      # a broken mirror must never break printing
        return len(s)

    def flush(self):
        self.stream.flush()

    def isatty(self):
        try:
            return self.stream.isatty()
        except Exception:
            return False

    def fileno(self):
        return self.stream.fileno()


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
        # #32b: the multi-threaded executor means think(), the feed timer and
        # the command worker can all log at the same instant. Interleaved
        # writes would corrupt the very evidence file this exists to produce.
        self.lock = threading.Lock()
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
            with self.lock:
                self.f.write(f"{ts} [{tag}] {msg}\n")
        except Exception:
            self.f = None                                # disk problem -> stop logging


class _DecodedFilter(message_filters.SimpleFilter):
    """#50: a message_filters source we drive by hand.

    ApproximateTimeSynchronizer only knows how to read from filters, but the
    compressed RGB stream has to be DECODED before the rest of the pipeline
    can use it. This filter is fed manually with the decoded Image message,
    so the synchronizer, the raw fallback and every downstream consumer keep
    working on ordinary sensor_msgs/Image exactly as before.
    """
    pass
    # SimpleFilter already provides registerCallback() and signalMessage();
    # no extra behaviour is needed, only a concrete class to instantiate
