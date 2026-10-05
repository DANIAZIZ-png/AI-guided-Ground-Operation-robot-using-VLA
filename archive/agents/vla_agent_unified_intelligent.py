#!/usr/bin/env python3
"""
vla_agent_improved.py — smarter TurtleBot 4 language agent

What changed compared with the earlier vla_agent.py:
  1) Object search no longer relies on blind "rotate then drive forward".
     When the target is not visible, the robot chooses map frontiers or
     uncovered free-space viewpoints and asks Nav2 to drive there.
  2) Patrol/explore uses a frontier + coverage planner instead of the nearest
     frontier only. It avoids repeatedly going to the same mapped area.
  3) Goal points are checked against the occupancy grid before being sent to
     Nav2, so the robot avoids goals inside walls, inflated obstacles, or unknown
     space.
  4) Navigation progress is monitored. Failed/stuck goals are blacklisted and the
     robot automatically tries another gap/viewpoint instead of stopping forever.
  5) Patrol continues as a coverage patrol after mapping is complete; explore
     saves the map and stops when no reachable frontier remains.

Run in ubuntu22-gpu with ROS 2 sourced, while Gazebo/Nav2/SLAM are running:
    python3 ~/vla_agent.py

Required model-side service:
    vla-box: source ~/yolo-env/bin/activate && python ~/yolo_server.py

This file still uses llm_brain.py for command parsing. The actual low-level path
execution remains delegated to Nav2; this agent provides semantic/task-level
intelligence and map-driven goal selection.
"""

import base64
import math
import os
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point

# Create 3 dock/undock actions. These exist on TurtleBot 4 / Create 3 Humble
# when ros-humble-irobot-create-msgs is installed. Keep the import optional so
# the rest of the agent still runs in Gazebo/test environments where the package
# is missing.
try:
    from irobot_create_msgs.action import Dock, Undock
except Exception:  # ImportError on systems without irobot_create_msgs
    Dock = None
    Undock = None

from llm_brain import decide

# ─────────────────────────────────────────────────────────────────────────────
# ROS topics / frames
# ─────────────────────────────────────────────────────────────────────────────
YOLO_URL    = "http://127.0.0.1:5001/detect"
RGB_TOPIC   = "/oakd/rgb/preview/image_raw"
DEPTH_TOPIC = "/oakd/rgb/preview/depth"
INFO_TOPIC  = "/oakd/rgb/preview/camera_info"
SCAN_TOPIC  = "/scan"
ODOM_TOPIC  = "/odom"
CMD_TOPIC   = "/cmd_vel"
MAP_TOPIC   = "/map"
MAP_FRAME   = "map"
ROBOT_FRAME = "base_link"

# ─────────────────────────────────────────────────────────────────────────────
# Behaviour tuning
# ─────────────────────────────────────────────────────────────────────────────
STOP_DISTANCE      = 0.70   # final standoff from a detected object, metres
ARRIVE_TOL         = 0.28   # extra tolerance around STOP_DISTANCE
MAX_OBJECT_HOP     = 2.5    # object-nav goals are sent in bounded hops
GOAL_REISSUE       = 0.45   # re-send object goal only if it moved this much

SEARCH_TURN_SPEED  = 0.45   # rad/s for 360-degree scanning at a viewpoint
OBSTACLE_STOP      = 0.55   # lidar stop threshold for manual rotate scan

# Occupancy-grid interpretation. SLAM map usually uses: free=0, obstacle=100,
# unknown=-1. These thresholds keep goal points conservative.
FREE_MAX           = 20
OCCUPIED_MIN       = 65
GOAL_CLEARANCE     = 0.30   # required obstacle clearance around a goal, metres
UNKNOWN_CLEARANCE  = 0.10   # do not place a goal directly on unknown boundary

# Frontier / coverage planning
FRONTIER_MIN_CELLS     = 8
FRONTIER_MIN_DISTANCE  = 0.80
FRONTIER_MAX_DISTANCE  = 6.0    # keep goals local; far frontier goals often abort in narrow maps
FRONTIER_BIN_RADIUS    = 0.40
BLACKLIST_RADIUS       = 0.90
RECENT_GOAL_RADIUS     = 0.75
COVERAGE_RADIUS        = 0.45   # area considered "visited" around robot trail
COVERAGE_STRIDE_METRES = 0.45   # sampling stride for known-map coverage goals
COVERAGE_MIN_DISTANCE  = 1.20
COVERAGE_MAX_DISTANCE  = 6.50

# Search limits
FIND_TIMEOUT        = 120.0  # target search / find-another timeout
NEW_INSTANCE_RADIUS = 0.80   # object-instance clustering radius, metres

# Stuck/progress handling
NAV_TIMEOUT          = 90.0
NAV_PROGRESS_TIMEOUT = 25.0
STUCK_DIST           = 0.10
STUCK_TIME           = 20.0
MAX_STUCKS           = 4

# Timers
THINK_PERIOD = 1.0
PUB_PERIOD   = 0.1

SAVE_MAP_PATH = os.path.expanduser("~/warehouse_map")

STOP_WORDS = ("cancel", "stop", "halt", "abort")
QUIT_WORDS = ("quit", "exit")
YES_WORDS  = ("yes", "y", "yeah", "yep", "sure", "ok", "okay", "affirmative")

# Words the planner may use for the charging dock. If the LLM mistakenly maps
# "go to the dock" as normal object navigation, the agent converts it to the
# Create 3 docking action instead of searching for a YOLO object called dock.
DOCK_TARGET_WORDS = {
    "dock", "charging dock", "charger", "charging station", "station",
    "base", "home base", "home", "charging base", "charge station",
}

GOAL_SUCCEEDED = 4


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class VLAAgent(Node):
    def __init__(self):
        super().__init__("vla_agent")
        self.bridge = CvBridge()

        # Sensor state
        self.rgb = None
        self.depth = None
        self.K = None
        self.cam_frame = None
        self.map = None
        self.map_grid = None
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.front_min = float("inf")

        # High-level task state
        self.mode = None                  # None | navigate | find_more | explore
        self.area_label = None             # explore | patrol | search
        self.target = None
        self.queue = []
        self.pending = None
        self.has_feed_step = False
        self.task_id = 0
        self.shutdown = False

        # Nav2 goal state
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None              # object | frontier | coverage | search_viewpoint
        self.goal_xy = None
        self.goal_sent_t = None
        self.goal_min_feedback_dist = None
        self.goal_last_progress_t = None
        self.cur_goal = None
        self.cmd = Twist()

        # Search / scan state
        self.search_state = None           # ROTATE | NAVIGATING | IDLE
        self.turn_accum = 0.0
        self.last_yaw = None
        self.search_start_t = 0.0
        self.scan_after_goal = False

        # Object tracking
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.seen_instances = {}           # target -> [(x, y), ...]
        self.find_baseline = 0

        # Coverage/exploration memory
        self.blacklist = []                # failed/rejected goal points
        self.goal_history = []             # all attempted semantic goals
        self.covered_cells = set()         # map cells visited by the robot footprint
        self.last_coverage_mark = None
        self.frontier_sent_goal = False
        self.explore_start_t = None
        self.warned_no_map = False
        self.warned_no_frontier = False

        # Stuck detection
        self.last_pos = None
        self.last_move_t = None
        self.consecutive_stucks = 0

        self.feed_proc = None

        # Dock/undock state
        self.dock_goal_handle = None
        self.dock_action_name = None

        # ROS wiring
        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, 1)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Relative action names respect a robot namespace if the node is launched
        # inside one. On a non-namespaced TurtleBot they resolve to /dock and /undock.
        self.dock_client = ActionClient(self, Dock, "dock") if Dock is not None else None
        self.undock_client = ActionClient(self, Undock, "undock") if Undock is not None else None

        self.create_timer(THINK_PERIOD, self.think)
        self.create_timer(PUB_PERIOD, self.publish_cmd)

    # ──────────────────────────────────────────────────────────────────────
    # Sensor callbacks
    # ──────────────────────────────────────────────────────────────────────
    def on_rgb(self, m):
        self.rgb = m

    def on_depth(self, m):
        self.depth = m

    def on_info(self, m):
        self.K = m.k
        self.cam_frame = m.header.frame_id

    def on_map(self, m):
        self.map = m
        self.map_grid = np.array(m.data, dtype=np.int16).reshape((m.info.height, m.info.width))

    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.have_odom = True
        self.mark_covered(self.x, self.y)

    def on_scan(self, m):
        if len(m.ranges) == 0:
            return
        cone = math.radians(18)
        best = float("inf")
        for i, r in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if -cone <= ang <= cone and math.isfinite(r) and r > 0.0:
                best = min(best, r)
        self.front_min = best

    def notify(self, msg):
        print(f"\n[robot] {msg}\nCommand> ", end="", flush=True)

    # ──────────────────────────────────────────────────────────────────────
    # Occupancy-grid helpers
    # ──────────────────────────────────────────────────────────────────────
    def world_to_cell(self, wx, wy):
        if self.map is None:
            return None
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)
        if 0 <= mx < self.map.info.width and 0 <= my < self.map.info.height:
            return mx, my
        return None

    def cell_to_world(self, mx, my):
        res = self.map.info.resolution
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        return ox + (mx + 0.5) * res, oy + (my + 0.5) * res

    def is_free_cell(self, mx, my, grid=None):
        if grid is None:
            grid = self.map_grid
        if grid is None:
            return False
        h, w = grid.shape
        if not (0 <= mx < w and 0 <= my < h):
            return False
        return 0 <= int(grid[my, mx]) <= FREE_MAX

    def cell_clearance_ok(self, mx, my, clearance_m=GOAL_CLEARANCE, unknown_clearance_m=UNKNOWN_CLEARANCE):
        if self.map_grid is None or self.map is None:
            return False
        res = self.map.info.resolution
        r = max(1, int(math.ceil(clearance_m / res)))
        ru = max(1, int(math.ceil(unknown_clearance_m / res)))
        h, w = self.map_grid.shape
        if mx - r < 0 or mx + r >= w or my - r < 0 or my + r >= h:
            return False
        if not self.is_free_cell(mx, my):
            return False
        patch = self.map_grid[my-r:my+r+1, mx-r:mx+r+1]
        if np.any(patch >= OCCUPIED_MIN):
            return False
        if ru > 0:
            upatch = self.map_grid[my-ru:my+ru+1, mx-ru:mx+ru+1]
            if np.any(upatch == -1):
                return False
        return True

    def nearest_safe_world(self, wx, wy, max_radius_m=0.9):
        """Return a nearby safe free-space goal, or None."""
        if self.map is None or self.map_grid is None:
            return wx, wy
        c = self.world_to_cell(wx, wy)
        if c is None:
            return None
        cx, cy = c
        res = self.map.info.resolution
        max_r = max(1, int(math.ceil(max_radius_m / res)))
        best = None
        best_d = float("inf")
        for r in range(max_r + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    mx, my = cx + dx, cy + dy
                    if not self.cell_clearance_ok(mx, my):
                        continue
                    sx, sy = self.cell_to_world(mx, my)
                    d = math.hypot(sx - wx, sy - wy)
                    if d < best_d:
                        best = (sx, sy)
                        best_d = d
            if best is not None:
                return best
        return None

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < BLACKLIST_RADIUS for bx, by in self.blacklist)

    def is_recent_goal(self, x, y):
        return any(math.hypot(x - gx, y - gy) < RECENT_GOAL_RADIUS for gx, gy in self.goal_history[-30:])

    def mark_covered(self, wx, wy):
        if self.map is None:
            return
        if self.last_coverage_mark is not None:
            if math.hypot(wx - self.last_coverage_mark[0], wy - self.last_coverage_mark[1]) < COVERAGE_RADIUS * 0.5:
                return
        c = self.world_to_cell(wx, wy)
        if c is None:
            return
        mx, my = c
        res = self.map.info.resolution
        r = max(1, int(math.ceil(COVERAGE_RADIUS / res)))
        for yy in range(my - r, my + r + 1):
            for xx in range(mx - r, mx + r + 1):
                if math.hypot(xx - mx, yy - my) * res <= COVERAGE_RADIUS:
                    self.covered_cells.add((xx, yy))
        self.last_coverage_mark = (wx, wy)

    def coverage_penalty(self, wx, wy):
        c = self.world_to_cell(wx, wy)
        if c is None:
            return 1.0
        mx, my = c
        res = self.map.info.resolution
        r = max(1, int(math.ceil(COVERAGE_RADIUS / res)))
        hits = 0
        total = 0
        for yy in range(my - r, my + r + 1):
            for xx in range(mx - r, mx + r + 1):
                if math.hypot(xx - mx, yy - my) * res <= COVERAGE_RADIUS:
                    total += 1
                    if (xx, yy) in self.covered_cells:
                        hits += 1
        return hits / max(total, 1)

    def unknown_gain(self, mx, my, radius_m=1.2):
        if self.map is None or self.map_grid is None:
            return 0
        res = self.map.info.resolution
        r = max(1, int(math.ceil(radius_m / res)))
        h, w = self.map_grid.shape
        x0, x1 = max(0, mx - r), min(w, mx + r + 1)
        y0, y1 = max(0, my - r), min(h, my + r + 1)
        return int(np.sum(self.map_grid[y0:y1, x0:x1] == -1))

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            if self.have_odom:
                return self.x, self.y
            return None

    # ──────────────────────────────────────────────────────────────────────
    # YOLO + projection
    # ──────────────────────────────────────────────────────────────────────
    def yolo_detect(self):
        if self.rgb is None:
            return None
        cv_rgb = self.bridge.imgmsg_to_cv2(self.rgb, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_rgb)
        if not ok:
            return None
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        try:
            resp = requests.post(YOLO_URL, json={"image": img_b64}, timeout=15)
            resp.raise_for_status()
            return resp.json().get("detections", [])
        except Exception as e:
            print(f"\n[robot] YOLO server error. Is yolo_server.py running? {e}")
            return None

    def project_pixel(self, u, v):
        if self.depth is None or self.K is None or self.cam_frame is None:
            return None
        depth_img = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
        h, w = depth_img.shape[:2]
        u = max(0, min(int(u), w - 1))
        v = max(0, min(int(v), h - 1))
        patch = depth_img[max(0, v-4):v+5, max(0, u-4):u+5].astype(float)
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            return None
        d = float(np.median(patch))
        if d > 100:
            d /= 1000.0
        fx, fy = self.K[0], self.K[4]
        cx, cy = self.K[2], self.K[5]
        pt = PointStamped()
        pt.header.frame_id = self.cam_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = (u - cx) * d / fx
        pt.point.y = (v - cy) * d / fy
        pt.point.z = d
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, self.cam_frame, rclpy.time.Time())
            obj = do_transform_point(pt, tf)
            return obj.point.x, obj.point.y
        except Exception:
            return None

    def locate_all(self, target, detections=None):
        target = (target or "").lower().strip()
        if detections is None:
            detections = self.yolo_detect() or []
        out = []
        for d in detections:
            name = str(d.get("name", "")).lower()
            if target and target in name:
                x1, y1, x2, y2 = d.get("box", [0, 0, 0, 0])
                proj = self.project_pixel((x1 + x2) / 2, (y1 + y2) / 2)
                if proj is not None:
                    out.append(proj)
        return out

    def locate_target(self):
        detections = self.yolo_detect()
        if detections is None:
            return None
        positions = self.locate_all(self.target, detections)
        if not positions:
            return None
        rxy = self.robot_xy()
        if rxy is None:
            return None
        rx, ry = rxy
        # If multiple objects match, use nearest projected instance.
        ox, oy = min(positions, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))
        return ox, oy, rx, ry, math.hypot(ox - rx, oy - ry)

    def register_instances(self, target, positions):
        target = (target or "").lower()
        known = self.seen_instances.setdefault(target, [])
        added = 0
        for x, y in positions:
            if all(math.hypot(x - kx, y - ky) > NEW_INSTANCE_RADIUS for kx, ky in known):
                known.append((x, y))
                added += 1
        return added

    def count_instances(self, target, frames=10, dt=0.15, min_fraction=0.2):
        target = (target or "").lower()
        positions = []
        used_frames = 0
        for _ in range(frames):
            det = self.yolo_detect()
            if det is not None:
                used_frames += 1
                positions.extend(self.locate_all(target, det))
            time.sleep(dt)

        clusters = []
        for x, y in positions:
            for c in clusters:
                if math.hypot(x - c[0], y - c[1]) <= NEW_INSTANCE_RADIUS:
                    n = c[2] + 1
                    c[0] = (c[0] * c[2] + x) / n
                    c[1] = (c[1] * c[2] + y) / n
                    c[2] = n
                    break
            else:
                clusters.append([x, y, 1])
        min_hits = max(1, round(min_fraction * max(used_frames, 1)))
        real = [c for c in clusters if c[2] >= min_hits]
        self.register_instances(target, [(c[0], c[1]) for c in real])
        return len(real)

    # ──────────────────────────────────────────────────────────────────────
    # Command loop / task setup
    # ──────────────────────────────────────────────────────────────────────
    def run_input_loop(self):
        print("VLA agent ready. 'cancel' stops a task, 'quit' exits.")
        print("Try: patrol the area and give live feed | map the area | find a person | "
              "go to the chair then to the door | count the chairs | find another chair | undock | dock\n")
        while not self.shutdown:
            try:
                cmd = input("Command> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            low = cmd.lower()
            if low.startswith(QUIT_WORDS):
                break
            if low.startswith(STOP_WORDS):
                self.cancel_task()
                print("Task cancelled. Robot is idle.")
                continue
            if self.pending is not None:
                self.resolve_pending(cmd)
                continue
            self.cancel_navigation(clear_queue=True)
            self.handle_command(cmd)
        self.shutdown = True

    def handle_command(self, cmd):
        # Dock/undock does not need YOLO, depth, or camera readiness. Handle
        # obvious single-step commands immediately so "undock" still works even
        # if the perception stack is not up yet or the LLM is offline. Compound
        # commands are still handled by llm_brain.py below.
        direct_steps = self.simple_local_intent(cmd)
        if direct_steps is not None:
            self.queue = direct_steps
            self.has_feed_step = any((s.get("action") or "").lower() == "feed" for s in self.queue)
            self.start_next_step()
            return

        direct_dock_action = self.simple_dock_intent(cmd)
        if direct_dock_action is not None:
            speech = "Starting undocking." if direct_dock_action == "undock" else "Returning to the dock."
            self.queue = [{"action": direct_dock_action, "target": None, "speech": speech}]
            self.start_next_step()
            return

        if self.rgb is None or self.depth is None or self.K is None:
            print("Camera/depth not ready yet — wait a second and try again.")
            return
        detections = self.yolo_detect()
        if detections is None:
            return
        visible = sorted({d.get("name", "") for d in detections})
        print("thinking...")
        try:
            decision = decide(cmd, visible)
        except Exception as e:
            print(f"Brain error. Is Ollama running and is llm_brain.py configured? {e}")
            return
        steps = decision.get("steps")
        if not steps:
            steps = [decision] if decision.get("action") else []
        if not steps:
            print("I didn't catch a clear command.")
            return
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

        # Safety/canonicalization: if the LLM says "navigate" to the dock/base,
        # use the Create 3 docking behavior instead of YOLO/Nav2 object search.
        target_norm = (target or "").lower().strip()
        if action == "navigate" and target_norm in DOCK_TARGET_WORDS:
            action = "dock"
            target = None

        if speech and action != "count":
            print(f"[{action}] {speech}")

        if action == "navigate":
            if not target:
                print("I need a target object to navigate to.")
                self.start_next_step()
                return
            self.start_target_search(target.lower(), mode="navigate")

        elif action == "find_another":
            if not target:
                print("I need to know what object to find another of.")
                self.start_next_step()
                return
            self.start_target_search(target.lower(), mode="find_more")

        elif action in ("explore", "patrol"):
            if action == "patrol" and not self.has_feed_step:
                self.pending = {"type": "feed_confirm"}
                self.notify("Do you want a live camera feed during patrol? (yes / no)")
                return
            self.start_area_task(action)

        elif action == "count":
            t = (target or "").lower()
            if not t:
                print("I'm not sure what you want me to count.")
                self.start_next_step()
                return
            print(f"Counting the {t}s — give me a moment...")
            n = self.count_instances(t)
            plural = t if n == 1 else t + "s"
            print(f"I can see {n} {plural} right now.")
            self.start_next_step()

        elif action == "describe":
            det = self.yolo_detect() or []
            names = [d.get("name", "") for d in det]
            print(f"I see: {names if names else 'nothing'}")
            self.start_next_step()

        elif action == "feed":
            self.start_feed()
            self.start_next_step()

        elif action in ("dock", "undock"):
            # Do not immediately continue the queue. Dock/undock is an action
            # that can take time; the result callback starts the next step.
            self.start_dock_action(action)

        elif action == "reject":
            print(speech or "I cannot perform that command.")
            self.start_next_step()

        elif action == "clarify":
            print(speech or "Please clarify the command.")
            self.start_next_step()

        else:
            print(f"Unknown planner action '{action}', skipping it.")
            self.start_next_step()

    def simple_dock_intent(self, cmd):
        """Return 'dock'/'undock' for obvious one-step docking commands.

        This is only a fallback. The normal route is still: command -> llm_brain
        -> JSON action. Keeping this fallback makes the safety-critical dock
        commands work even if the LLM server is temporarily unavailable.
        """
        low = " ".join((cmd or "").lower().replace("-", " ").split())
        if not low:
            return None
        # Avoid hijacking compound commands; the LLM should parse those.
        compound_words = (" then ", ",", " and ", " after ", " before ")
        if any(w in f" {low} " for w in compound_words):
            return None
        if low in {"undock", "un dock", "leave dock", "leave the dock", "move out of dock", "move out of the dock"}:
            return "undock"
        if "undock" in low or "un dock" in low:
            return "undock"
        if low in {
            "dock", "dock now", "dock the robot", "go dock", "go to dock", "go to the dock",
            "return to dock", "return to the dock", "return to base", "return to the base",
            "go home", "go to home", "go to base", "go to the base", "charge", "go charge",
            "go to charger", "go to the charger", "go to charging station",
            "go to the charging station", "return to charging station",
        }:
            return "dock"
        return None

    def simple_local_intent(self, cmd):
        """Small deterministic parser for common dock/patrol commands.

        This is a fallback for demo reliability. It does not replace llm_brain;
        it only catches commands where docking must work even if the LLM prompt
        has not been updated yet.
        """
        low = " ".join((cmd or "").lower().replace("-", " ").split())
        if not low:
            return None

        def has_dock(s):
            return ("return to base" in s or "return to the base" in s or
                    "go home" in s or "go charge" in s or
                    "go to charger" in s or "charging station" in s or
                    s.strip() in {"dock", "dock now", "go dock", "go to dock", "go to the dock"})

        def has_undock(s):
            return "undock" in s or "un dock" in s or "leave the dock" in s or "move out of the dock" in s

        def patrol_step():
            return {"action": "patrol", "target": None, "speech": "Starting patrol."}

        def explore_step():
            return {"action": "explore", "target": None, "speech": "Starting mapping."}

        if has_undock(low) and "patrol" in low:
            return [
                {"action": "undock", "target": None, "speech": "Starting undocking."},
                patrol_step(),
            ]
        if has_undock(low) and ("map" in low or "explore" in low):
            return [
                {"action": "undock", "target": None, "speech": "Starting undocking."},
                explore_step(),
            ]
        if "patrol" in low and has_dock(low):
            return [
                patrol_step(),
                {"action": "dock", "target": None, "speech": "Returning to the dock."},
            ]
        if ("map" in low or "explore" in low) and has_dock(low):
            return [
                explore_step(),
                {"action": "dock", "target": None, "speech": "Returning to the dock."},
            ]
        return None

    def start_dock_action(self, action):
        """Send Create 3 /dock or /undock action and continue queued steps on result."""
        self.cancel_navigation(clear_queue=False)
        self.cmd = Twist()
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

        if action == "dock":
            action_type = Dock
            client = self.dock_client
            server_name = "/dock"
            verb = "docking"
        else:
            action_type = Undock
            client = self.undock_client
            server_name = "/undock"
            verb = "undocking"

        if action_type is None or client is None:
            print(
                "Dock/undock support is missing because irobot_create_msgs is not importable.\n"
                "Install it in ubuntu22-gpu with:\n"
                "  sudo apt install ros-humble-irobot-create-msgs"
            )
            self.start_next_step()
            return

        print(f"Sending Create 3 {verb} request to {server_name}...")
        if not client.wait_for_server(timeout_sec=3.0):
            print(
                f"No {server_name} action server found. Check with:\n"
                f"  ros2 action list | grep {action}\n"
                "If your robot uses a namespace, run the agent in the same namespace or remap the action."
            )
            self.start_next_step()
            return

        goal_msg = action_type.Goal()
        self.dock_action_name = action
        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut, a=action: self.on_dock_goal_response(fut, a))

    def on_dock_goal_response(self, future, action):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.notify(f"Could not send {action} goal: {e}")
            self.dock_action_name = None
            self.start_next_step()
            return

        if not goal_handle.accepted:
            self.notify(f"Create 3 rejected the {action} request.")
            self.dock_goal_handle = None
            self.dock_action_name = None
            self.start_next_step()
            return

        self.dock_goal_handle = goal_handle
        self.notify(f"Create 3 accepted {action}. Waiting for completion...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, a=action: self.on_dock_result(fut, a))

    def on_dock_result(self, future, action):
        status = None
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as e:
            self.notify(f"{action.capitalize()} action ended with an error: {e}")
            self.dock_goal_handle = None
            self.dock_action_name = None
            self.start_next_step()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            done = "Docking completed." if action == "dock" else "Undocking completed."
            self.notify(done)
        else:
            if action == "dock":
                self.notify(
                    f"Docking did not succeed; action status={status}. The dock may be too far away, "
                    "not powered, or not visible to the robot."
                )
            else:
                self.notify(
                    f"Undocking did not succeed; action status={status}. The robot may already be undocked."
                )

        self.dock_goal_handle = None
        self.dock_action_name = None
        self.start_next_step()

    def start_target_search(self, target, mode="navigate"):
        self.target = target
        self.mode = mode
        self.area_label = "search"
        self.search_state = "ROTATE"
        self.search_start_t = time.monotonic()
        self.turn_accum = 0.0
        self.last_yaw = None
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.goal_kind = None
        self.consecutive_stucks = 0
        self.scan_after_goal = False
        if mode == "find_more":
            self.find_baseline = len(self.seen_instances.get(target, []))
            print(f"Looking for another {target}. I will scan, move to new viewpoints, and avoid repeated areas.")
        else:
            print(f"Searching for the {target}. I will use the map/frontiers instead of blind forward motion.")

    def start_area_task(self, label="explore"):
        self.mode = "explore"
        self.area_label = label
        self.search_state = None
        self.blacklist = []
        self.frontier_sent_goal = False
        self.explore_start_t = time.monotonic()
        self.warned_no_map = False
        self.warned_no_frontier = False
        self.consecutive_stucks = 0
        self.goal_kind = None
        self.cur_goal = None
        self.cmd = Twist()
        if label == "patrol":
            print("Starting intelligent patrol: frontiers first, then coverage waypoints. Type 'cancel' to stop.")
        else:
            print("Starting frontier mapping. I will move through reachable gaps and save the map when complete.")

    def resolve_pending(self, cmd):
        p = self.pending
        self.pending = None
        if p and p.get("type") == "feed_confirm":
            if cmd.lower().startswith(YES_WORDS):
                self.start_feed()
            else:
                print("OK, patrolling without the live feed.")
            self.start_area_task("patrol")

    # ──────────────────────────────────────────────────────────────────────
    # Feed / cancel / stop
    # ──────────────────────────────────────────────────────────────────────
    def cancel_navigation(self, clear_queue=False):
        self.task_id += 1
        self.mode = None
        self.area_label = None
        self.search_state = None
        self.target = None
        self.pending = None
        if clear_queue:
            self.queue = []
        self.goal_pending = False
        self.goal_kind = None
        self.goal_xy = None
        self.goal_sent_t = None
        self.goal_min_feedback_dist = None
        self.goal_last_progress_t = None
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.last_pos = None
        self.last_move_t = None
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        if self.dock_goal_handle is not None:
            try:
                self.dock_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.dock_goal_handle = None
            self.dock_action_name = None
        self.goal_handle = None
        if self.dock_goal_handle is not None:
            try:
                self.dock_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.dock_goal_handle = None
        self.dock_action_name = None
        self.cmd = Twist()
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def cancel_task(self):
        self.cancel_navigation(clear_queue=True)
        self.stop_feed()

    def start_feed(self):
        self.stop_feed()
        try:
            self.feed_proc = subprocess.Popen(
                ["ros2", "run", "rqt_image_view", "rqt_image_view", RGB_TOPIC],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print("Live camera feed opened in a new window. Type 'cancel' to close it.")
        except FileNotFoundError:
            print("Live feed needs rqt_image_view: sudo apt install ros-humble-rqt-image-view")

    def stop_feed(self):
        if self.feed_proc is not None:
            try:
                self.feed_proc.terminate()
            except Exception:
                pass
            self.feed_proc = None

    def stop_base(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_xy = None
        self.cmd = Twist()
        for _ in range(4):
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass
            time.sleep(0.02)

    # ──────────────────────────────────────────────────────────────────────
    # Timers / main thinking
    # ──────────────────────────────────────────────────────────────────────
    def publish_cmd(self):
        # Only manual rotate scan uses /cmd_vel here. Nav2 owns /cmd_vel during goals.
        if self.search_state == "ROTATE" and self.mode in ("navigate", "find_more", "explore"):
            self.cmd_pub.publish(self.cmd)

    def think(self):
        self.check_stuck()
        self.check_nav_timeout()
        if self.mode == "navigate":
            self.navigate_think()
        elif self.mode == "find_more":
            self.find_more_think()
        elif self.mode == "explore":
            self.area_think()

    # ──────────────────────────────────────────────────────────────────────
    # Target navigation / search
    # ──────────────────────────────────────────────────────────────────────
    def navigate_think(self):
        if self.rgb is None or self.depth is None or self.K is None or not self.have_odom:
            return
        found = self.locate_target()
        rxy = self.robot_xy()
        if rxy is None:
            return

        # Arrival by remembered object position. This is important because YOLO/depth
        # can fail at close range when the object fills the camera or drops out of
        # the depth image. The robot should still stop if odometry says it reached
        # the saved standoff zone.
        if self.last_obj_xy is not None:
            if math.hypot(rxy[0] - self.last_obj_xy[0], rxy[1] - self.last_obj_xy[1]) <= STOP_DISTANCE + ARRIVE_TOL:
                self.stop_base()
                self.notify(f"Arrived at the {self.target}.")
                self.end_target_task()
                return

        if found is not None:
            ox, oy, rx, ry, dist = found
            self.register_instances(self.target, [(ox, oy)])
            self.last_obj_xy = (ox, oy)
            if not self.target_announced:
                self.notify(f"Found the {self.target}. Switching from search to Nav2 approach.")
                self.target_announced = True

            if math.hypot(rxy[0] - ox, rxy[1] - oy) <= STOP_DISTANCE + ARRIVE_TOL:
                self.stop_base()
                self.notify(f"Arrived at the {self.target}.")
                self.end_target_task()
                return

            goal = self.compute_standoff_goal(ox, oy, rx, ry)
            if goal is None:
                self.blacklist.append((ox, oy))
                self.notify(f"I see the {self.target}, but the approach point is blocked. Trying another viewpoint.")
                self.start_rotate_scan()
                return
            gx, gy, yaw = goal
            if (self.nav_goal_xy is None or
                    math.hypot(gx - self.nav_goal_xy[0], gy - self.nav_goal_xy[1]) > GOAL_REISSUE or
                    self.goal_kind != "object"):
                self.nav_goal_xy = (gx, gy)
                self.send_goal(gx, gy, yaw, kind="object")
            return

        # If a Nav2 search/viewpoint goal is running, let it finish while the
        # camera continues to check for the target every second.
        if self.nav_active():
            return

        if time.monotonic() - self.search_start_t > FIND_TIMEOUT:
            self.stop_base()
            self.notify(f"I could not find the {self.target} within the search timeout. Try moving the robot or improving lighting.")
            self.end_target_task()
            return

        if self.search_state != "ROTATE":
            self.start_rotate_scan()
        if self.do_rotate_scan():
            self.send_next_search_viewpoint()

    def find_more_think(self):
        if self.rgb is None or self.depth is None or self.K is None or not self.have_odom:
            return
        detections = self.yolo_detect()
        if detections is not None:
            positions = self.locate_all(self.target, detections)
            added = self.register_instances(self.target, positions)
            total = len(self.seen_instances.get(self.target, []))
            if added > 0 and total > self.find_baseline:
                self.stop_base()
                plural = self.target if total == 1 else self.target + "s"
                self.notify(f"I found another {self.target}. I now count {total} {plural}.")
                self.end_target_task()
                return

        if self.nav_active():
            return

        if time.monotonic() - self.search_start_t > FIND_TIMEOUT:
            total = len(self.seen_instances.get(self.target, []))
            self.stop_base()
            self.notify(f"I could not find another {self.target}. I still count {total}.")
            self.end_target_task()
            return

        if self.search_state != "ROTATE":
            self.start_rotate_scan()
        if self.do_rotate_scan():
            self.send_next_search_viewpoint()

    def compute_standoff_goal(self, ox, oy, rx, ry):
        """Choose a safe point around the object and orient the robot toward it."""
        dx, dy = ox - rx, oy - ry
        dist = max(math.hypot(dx, dy), 1e-6)
        base_angle = math.atan2(dy, dx)
        # Candidate standoff points around the object. 0 means between robot and object;
        # side offsets help if the direct approach point is blocked.
        offsets = [0, math.radians(35), -math.radians(35), math.radians(70), -math.radians(70), math.pi]
        candidates = []
        for off in offsets:
            ang = base_angle + off
            sx = ox - math.cos(ang) * STOP_DISTANCE
            sy = oy - math.sin(ang) * STOP_DISTANCE
            safe = self.nearest_safe_world(sx, sy, max_radius_m=0.6)
            if safe is None:
                continue
            if self.is_blacklisted(safe[0], safe[1]):
                continue
            gd = math.hypot(safe[0] - rx, safe[1] - ry)
            if gd > MAX_OBJECT_HOP:
                # Bound each hop so localization and dynamic obstacles remain stable.
                ux = (safe[0] - rx) / max(gd, 1e-6)
                uy = (safe[1] - ry) / max(gd, 1e-6)
                sx2, sy2 = rx + ux * MAX_OBJECT_HOP, ry + uy * MAX_OBJECT_HOP
                safe2 = self.nearest_safe_world(sx2, sy2, max_radius_m=0.5)
                if safe2 is None:
                    continue
                safe = safe2
                gd = math.hypot(safe[0] - rx, safe[1] - ry)
            yaw = math.atan2(oy - safe[1], ox - safe[0])
            candidates.append((gd, safe[0], safe[1], yaw))
        if not candidates:
            # Fallback without map, useful during very early SLAM start.
            sx = rx + (dx / dist) * max(dist - STOP_DISTANCE, 0.1)
            sy = ry + (dy / dist) * max(dist - STOP_DISTANCE, 0.1)
            return sx, sy, base_angle
        candidates.sort(key=lambda c: c[0])
        _, gx, gy, yaw = candidates[0]
        return gx, gy, yaw

    def start_rotate_scan(self):
        self.search_state = "ROTATE"
        self.turn_accum = 0.0
        self.last_yaw = None
        self.cmd = Twist()

    def do_rotate_scan(self):
        """Rotate until a full 360-degree scan is complete. Returns True once."""
        if not self.have_odom:
            return False
        if self.front_min < OBSTACLE_STOP:
            # Still allow rotation in place, but slower if something is close.
            speed = max(0.25, SEARCH_TURN_SPEED * 0.6)
        else:
            speed = SEARCH_TURN_SPEED
        if self.last_yaw is None:
            self.last_yaw = self.yaw
        dyaw = abs(norm_angle(self.yaw - self.last_yaw))
        self.turn_accum += dyaw
        self.last_yaw = self.yaw
        t = Twist()
        t.angular.z = speed
        self.cmd = t
        if self.turn_accum >= 2.0 * math.pi:
            self.cmd = Twist()
            self.turn_accum = 0.0
            self.last_yaw = None
            self.search_state = "IDLE"
            return True
        return False

    def send_next_search_viewpoint(self):
        goal = self.choose_frontier_goal(prefer_near=False)
        if goal is None:
            goal = self.choose_coverage_goal()
        if goal is None:
            self.notify("I have no safe unexplored or uncovered viewpoint. I will rotate here and keep checking.")
            self.start_rotate_scan()
            return
        gx, gy, yaw, source = goal
        self.notify(f"No {self.target} here — moving to a new {source} viewpoint.")
        self.send_goal(gx, gy, yaw, kind="search_viewpoint")

    def end_target_task(self):
        self.task_id += 1
        self.mode = None
        self.area_label = None
        self.search_state = None
        self.target = None
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.goal_kind = None
        self.cmd = Twist()
        self.start_next_step()

    # ──────────────────────────────────────────────────────────────────────
    # Frontier and coverage planner
    # ──────────────────────────────────────────────────────────────────────
    def area_think(self):
        now = time.monotonic()
        if self.map is None or self.map_grid is None:
            if self.explore_start_t and now - self.explore_start_t > 6 and not self.warned_no_map:
                self.warned_no_map = True
                self.notify("I am not receiving /map. Launch with slam:=true for mapping/patrol exploration.")
            return
        if self.nav_active():
            return
        rxy = self.robot_xy()
        if rxy is None:
            return

        # After every reached waypoint, do a local 360-degree scan before selecting
        # the next waypoint. This improves SLAM coverage and prevents instant
        # rapid-fire goal selection in tight rack/corridor spaces.
        if self.search_state == "ROTATE":
            if self.do_rotate_scan():
                self.search_state = None
            else:
                return

        frontier_goal = self.choose_frontier_goal(prefer_near=True)
        if frontier_goal is not None:
            gx, gy, yaw, source = frontier_goal
            self.frontier_sent_goal = True
            self.send_goal(gx, gy, yaw, kind="frontier")
            return

        # No frontiers: explore should finish; patrol should continue coverage.
        if self.area_label == "explore":
            if self.frontier_sent_goal:
                self.finish_explore()
            elif now - self.explore_start_t > 10 and not self.warned_no_frontier:
                self.warned_no_frontier = True
                self.notify("No frontier found. In localization mode the map is already complete; use patrol for coverage.")
            return

        coverage_goal = self.choose_coverage_goal()
        if coverage_goal is not None:
            gx, gy, yaw, source = coverage_goal
            self.send_goal(gx, gy, yaw, kind="coverage")
            return

        self.notify("Patrol has no safe uncovered waypoint right now. Rotating in place and continuing to monitor.")
        self.start_rotate_scan()

    def find_frontiers(self):
        """Connected-component frontiers: free cells adjacent to unknown cells."""
        if self.map is None or self.map_grid is None:
            return []
        grid = self.map_grid
        h, w = grid.shape
        free = (grid >= 0) & (grid <= FREE_MAX)
        unknown = (grid == -1)
        adj_unknown = np.zeros_like(unknown, dtype=bool)
        adj_unknown[1:, :]  |= unknown[:-1, :]
        adj_unknown[:-1, :] |= unknown[1:, :]
        adj_unknown[:, 1:]  |= unknown[:, :-1]
        adj_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & adj_unknown

        visited = np.zeros_like(frontier, dtype=bool)
        clusters = []
        neighbours = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
        ys, xs = np.where(frontier)
        for sx, sy in zip(xs.tolist(), ys.tolist()):
            if visited[sy, sx] or not frontier[sy, sx]:
                continue
            q = deque([(sx, sy)])
            visited[sy, sx] = True
            pts = []
            while q:
                x, y = q.popleft()
                pts.append((x, y))
                for dx, dy in neighbours:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h and frontier[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((nx, ny))
            if len(pts) >= FRONTIER_MIN_CELLS:
                clusters.append(pts)
        return clusters

    def choose_frontier_goal(self, prefer_near=False):
        rxy = self.robot_xy()
        if rxy is None or self.map is None:
            return None
        rx, ry = rxy
        candidates = []
        for pts in self.find_frontiers():
            size = len(pts)
            # Use the member cell closest to the cluster centroid, then snap to safe free space.
            cx = sum(p[0] for p in pts) / size
            cy = sum(p[1] for p in pts) / size
            best_cell = min(pts, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            wx, wy = self.cell_to_world(best_cell[0], best_cell[1])
            safe = self.nearest_safe_world(wx, wy, max_radius_m=FRONTIER_BIN_RADIUS)
            if safe is None:
                continue
            gx, gy = safe
            dist = math.hypot(gx - rx, gy - ry)
            if dist < FRONTIER_MIN_DISTANCE or dist > FRONTIER_MAX_DISTANCE:
                continue
            if self.is_blacklisted(gx, gy):
                continue
            c = self.world_to_cell(gx, gy)
            if c is None:
                continue
            gain = self.unknown_gain(c[0], c[1])
            cov = self.coverage_penalty(gx, gy)
            repeat = 1.0 if self.is_recent_goal(gx, gy) else 0.0
            # Score: information gain + cluster size + novelty. Prefer moderate distance.
            distance_score = -0.20 * dist if prefer_near else -0.08 * abs(dist - 3.0)
            score = (math.log(size + 1) * 1.6) + (math.log(gain + 1) * 2.0) + distance_score - (cov * 2.5) - (repeat * 4.0)
            yaw = math.atan2(gy - ry, gx - rx)
            candidates.append((score, gx, gy, yaw, size, gain))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0], reverse=True)
        _, gx, gy, yaw, size, gain = candidates[0]
        return gx, gy, yaw, "frontier/gap"

    def choose_coverage_goal(self):
        if self.map is None or self.map_grid is None:
            return None
        rxy = self.robot_xy()
        if rxy is None:
            return None
        rx, ry = rxy
        res = self.map.info.resolution
        stride = max(2, int(math.ceil(COVERAGE_STRIDE_METRES / res)))
        h, w = self.map_grid.shape
        candidates = []
        # Sample known free cells. Avoid borders because Nav2 dislikes goals near unknown/obstacles.
        for my in range(stride, h - stride, stride):
            for mx in range(stride, w - stride, stride):
                if not self.cell_clearance_ok(mx, my):
                    continue
                wx, wy = self.cell_to_world(mx, my)
                dist = math.hypot(wx - rx, wy - ry)
                if dist < COVERAGE_MIN_DISTANCE or dist > COVERAGE_MAX_DISTANCE:
                    continue
                if self.is_blacklisted(wx, wy) or self.is_recent_goal(wx, wy):
                    continue
                cov = self.coverage_penalty(wx, wy)
                # Prefer uncovered cells at useful distance. Coverage patrol should not keep
                # returning to already-white/visited parts unless everything else is covered.
                score = (1.0 - cov) * 5.0 - 0.12 * abs(dist - 3.5)
                candidates.append((score, wx, wy, dist))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0], reverse=True)
        _, gx, gy, _ = candidates[0]
        yaw = math.atan2(gy - ry, gx - rx)
        return gx, gy, yaw, "coverage"

    def finish_explore(self):
        self.notify("Area fully mapped — no reachable frontiers left. Saving the map.")
        try:
            subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", SAVE_MAP_PATH],
                           timeout=30, check=False)
            self.notify(f"Map saved to {SAVE_MAP_PATH}.yaml")
        except Exception as e:
            self.notify(f"Auto-save failed ({e}); save manually with map_saver_cli.")
        self.task_id += 1
        self.mode = None
        self.area_label = None
        self.goal_kind = None
        self.start_next_step()

    # ──────────────────────────────────────────────────────────────────────
    # Nav2 plumbing and recovery
    # ──────────────────────────────────────────────────────────────────────
    def nav_active(self):
        return self.goal_pending or self.goal_handle is not None

    def send_goal(self, x, y, yaw, kind):
        safe = self.nearest_safe_world(x, y, max_radius_m=0.8)
        if safe is None:
            self.blacklist.append((x, y))
            self.notify(f"Rejected unsafe {kind} goal near ({x:.2f}, {y:.2f}); trying another.")
            return False
        x, y = safe
        if self.is_blacklisted(x, y):
            return False

        # Cancel previous goal if switching target/kind.
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.goal_handle = None

        if not self.nav_client.server_is_ready():
            self.notify("Waiting for Nav2 NavigateToPose action server...")
            return False

        tid = self.task_id
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pending = True
        self.goal_kind = kind
        self.goal_xy = (x, y)
        self.cur_goal = (x, y)
        self.goal_sent_t = time.monotonic()
        self.goal_min_feedback_dist = None
        self.goal_last_progress_t = self.goal_sent_t
        self.search_state = "NAVIGATING" if self.mode in ("navigate", "find_more") else None
        self.cmd = Twist()
        self.goal_history.append((x, y))
        print(f"[nav2] {kind} goal -> x={x:.2f}, y={y:.2f}")
        fut = self.nav_client.send_goal_async(goal, feedback_callback=self.on_goal_feedback)
        fut.add_done_callback(lambda f: self.on_goal_response(f, tid))
        return True

    def on_goal_feedback(self, msg):
        try:
            d = float(msg.feedback.distance_remaining)
        except Exception:
            return
        now = time.monotonic()
        if self.goal_min_feedback_dist is None or d < self.goal_min_feedback_dist - 0.08:
            self.goal_min_feedback_dist = d
            self.goal_last_progress_t = now

    def on_goal_response(self, future, tid):
        self.goal_pending = False
        try:
            gh = future.result()
        except Exception as e:
            self.notify(f"Nav2 goal request failed: {e}")
            self.handle_goal_failure()
            return
        if tid != self.task_id:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return
        if not gh.accepted:
            self.notify(f"Nav2 rejected {self.goal_kind} goal; blacklisting and choosing another.")
            self.handle_goal_failure()
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, tid))

    def on_result(self, future, tid):
        if tid != self.task_id:
            return
        status = None
        try:
            status = future.result().status
        except Exception:
            pass
        kind = self.goal_kind
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None

        if status == GOAL_SUCCEEDED:
            self.consecutive_stucks = 0
            if kind == "object":
                # navigate_think normally declares arrival using object distance. If Nav2
                # finished first, force one more perception cycle before ending.
                self.search_state = "IDLE"
                return
            if kind in ("search_viewpoint", "frontier", "coverage"):
                self.start_rotate_scan()
                return
        else:
            self.handle_goal_failure()

    def handle_goal_failure(self):
        failed_goal = self.cur_goal
        if failed_goal is not None:
            self.blacklist.append(failed_goal)

        self.consecutive_stucks += 1
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_xy = None
        self.nav_goal_xy = None
        self.cmd = Twist()

        if self.mode in ("navigate", "find_more"):
            # Do not kill the user command after a few failed Nav2 goals.
            # For object/person search, failed viewpoints are expected in cluttered
            # maps. Keep searching until FIND_TIMEOUT expires.
            if self.consecutive_stucks >= MAX_STUCKS:
                self.notify("Several routes failed, but I will keep searching with closer viewpoints until the search timeout.")
                self.consecutive_stucks = 0
            else:
                self.notify("That route failed; I am scanning and will try another gap/viewpoint.")
            self.start_rotate_scan()
            return

        if self.mode == "explore":
            # Patrol/explore should be persistent. The old version stopped after
            # MAX_STUCKS, which is exactly what caused you to repeatedly type
            # 'patrol'. This version blacklists bad goals and keeps trying.
            if self.consecutive_stucks >= MAX_STUCKS:
                self.notify("Several patrol/explore goals failed. I am not stopping; I will switch to closer goals and continue.")
                self.consecutive_stucks = 0
                self.blacklist = self.blacklist[-80:]
                self.goal_history = self.goal_history[-80:]
            else:
                self.notify("That patrol/explore goal failed; blacklisting it and trying another area.")
            self.search_state = None
            return

        self.notify("Navigation goal failed. Robot is idle.")

    def check_nav_timeout(self):
        if not self.nav_active() or self.goal_sent_t is None:
            return
        now = time.monotonic()
        if now - self.goal_sent_t > NAV_TIMEOUT:
            self.notify("Nav2 goal timeout — cancelling and trying another goal.")
            self.cancel_current_goal_for_recovery()
            return
        if self.goal_handle is not None and self.goal_last_progress_t is not None:
            if now - self.goal_last_progress_t > NAV_PROGRESS_TIMEOUT:
                self.notify("Nav2 has made no progress for a while — recovering with a new goal.")
                self.cancel_current_goal_for_recovery()

    def cancel_current_goal_for_recovery(self):
        # Invalidate the current Nav2 result callback so a cancelled goal does
        # not count as a second failure when Nav2 reports STATUS_CANCELED later.
        self.task_id += 1
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        if self.cur_goal is not None:
            self.blacklist.append(self.cur_goal)
        self.goal_handle = None
        self.goal_pending = False
        self.goal_kind = None
        self.goal_xy = None
        self.nav_goal_xy = None
        self.consecutive_stucks += 1
        self.cmd = Twist()

        if self.mode in ("navigate", "find_more"):
            if self.consecutive_stucks >= MAX_STUCKS:
                self.notify("Recovery triggered several times. I will keep searching from a new viewpoint until timeout.")
                self.consecutive_stucks = 0
            self.start_rotate_scan()
        elif self.mode == "explore":
            if self.consecutive_stucks >= MAX_STUCKS:
                self.notify("Recovery triggered several times during patrol. I will keep patrolling using closer goals.")
                self.consecutive_stucks = 0
                self.blacklist = self.blacklist[-80:]
                self.goal_history = self.goal_history[-80:]
            self.search_state = None

    def check_stuck(self):
        driving = self.nav_active()
        if not driving:
            self.last_pos = None
            self.last_move_t = None
            return
        rxy = self.robot_xy()
        if rxy is None:
            return
        now = time.monotonic()
        if self.last_pos is None:
            self.last_pos = rxy
            self.last_move_t = now
            return
        if math.hypot(rxy[0] - self.last_pos[0], rxy[1] - self.last_pos[1]) > STUCK_DIST:
            self.last_pos = rxy
            self.last_move_t = now
            return
        if now - self.last_move_t > STUCK_TIME:
            self.notify("Robot pose is not changing — treating this as stuck and choosing another route.")
            self.last_pos = None
            self.last_move_t = None
            self.cancel_current_goal_for_recovery()


def main():
    rclpy.init()
    node = VLAAgent()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_input_loop()
    finally:
        node.cancel_task()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
