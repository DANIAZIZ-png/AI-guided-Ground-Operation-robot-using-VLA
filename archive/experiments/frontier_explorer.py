#!/usr/bin/env python3
"""
Frontier-based autonomous exploration with stuck-recovery.

Patrols and builds the SLAM map of the whole area, then saves it and stops.
If it gets physically stuck, it pauses and asks you to teleop it free, then
resumes on its own once you've moved it.

Run with SLAM + Nav2 up (slam:=true nav2:=true), in ubuntu22-gpu:
    python3 frontier_explorer.py

NOTE: the area must be ENCLOSED (walls all around), or it never runs out of
frontiers. Your warehouse is enclosed.
"""
import math
import os
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import tf2_ros

# ─── Config (tune these) ─────────────────────────────────────────
MAP_TOPIC        = "/map"
MAP_FRAME        = "map"
ROBOT_FRAME      = "base_link"
PLAN_PERIOD      = 2.0      # how often to re-evaluate, seconds
BIN_SIZE         = 0.5      # metres: frontier cells grouped into 0.5 m bins
MIN_FRONTIER     = 4        # a bin needs this many frontier cells to count
BLACKLIST_RADIUS = 0.8      # metres: skip frontiers near a dead/failed one
REPEAT_DIST      = 0.4      # metres: if we re-pick a frontier this close to the
                            #         one we just visited, it won't clear -> drop it
SAVE_MAP_PATH    = os.path.expanduser("~/warehouse_map")

# stuck / operator-override settings
PROGRESS_DIST    = 0.15     # m: moving more than this counts as making progress
STUCK_TIME       = 15.0     # s: no progress for this long while driving = stuck
RESUME_DIST      = 0.5      # m: operator must move the robot this far to resume


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.map = None
        self.navigating = False
        self.sent_a_goal = False
        self.done = False
        self.state = "EXPLORING"         # EXPLORING | STUCK
        self.cur_goal = (0.0, 0.0)
        self.last_done_goal = None
        self.goal_handle = None
        self.blacklist = []

        # progress / stuck tracking
        self.last_move_pos = None
        self.last_move_time = self.now()
        self.stuck_pos = None

        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, 1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(PLAN_PERIOD, self.plan)
        self.get_logger().info("Frontier explorer started. Mapping the area...")

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_map(self, msg):
        self.map = msg

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def update_progress(self, rx, ry):
        if self.last_move_pos is None:
            self.last_move_pos = (rx, ry)
            self.last_move_time = self.now()
            return
        if math.hypot(rx - self.last_move_pos[0], ry - self.last_move_pos[1]) > PROGRESS_DIST:
            self.last_move_pos = (rx, ry)
            self.last_move_time = self.now()

    def find_frontiers(self):
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        grid = np.array(m.data, dtype=np.int16).reshape((h, w))

        free = (grid == 0)
        unknown = (grid == -1)
        adj = np.zeros_like(unknown)
        adj[1:, :]  |= unknown[:-1, :]
        adj[:-1, :] |= unknown[1:, :]
        adj[:, 1:]  |= unknown[:, :-1]
        adj[:, :-1] |= unknown[:, 1:]
        frontier = free & adj
        ys, xs = np.where(frontier)
        if len(xs) == 0:
            return []

        bins = {}
        for cy, cx in zip(ys.tolist(), xs.tolist()):
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
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

    def plan(self):
        if self.done:
            return
        rxy = self.robot_xy()
        if rxy is None:
            self.get_logger().info("Waiting for robot pose (TF)...")
            return
        rx, ry = rxy
        self.update_progress(rx, ry)

        # ── STUCK: wait for the operator to teleop the robot free ──
        if self.state == "STUCK":
            if self.stuck_pos and math.hypot(rx - self.stuck_pos[0],
                                             ry - self.stuck_pos[1]) > RESUME_DIST:
                self.get_logger().info(">>> Robot moved - RESUMING exploration. <<<")
                self.state = "EXPLORING"
                self.navigating = False
                self.last_move_time = self.now()
            else:
                self.get_logger().warn(
                    ">>> STUCK. Please TELEOP me free (W/A/S/D); I will resume once moved. <<<")
            return

        if self.map is None:
            self.get_logger().info("Waiting for /map...")
            return

        # ── while driving, watch for being physically stuck ──
        if self.navigating:
            if (self.now() - self.last_move_time) > STUCK_TIME:
                self.enter_stuck(rx, ry)
            return

        # ── EXPLORING and idle: choose the next frontier ──
        clusters = [c for c in self.find_frontiers()
                    if not self.is_blacklisted(c[0], c[1])]
        if not clusters:
            if self.sent_a_goal:
                self.finish()
            else:
                self.get_logger().info("No frontiers yet - map still forming...")
            return

        clusters.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
        gx, gy, _ = clusters[0]

        # if we're re-picking the frontier we just visited, it won't clear -> drop it
        if self.last_done_goal and math.hypot(gx - self.last_done_goal[0],
                                              gy - self.last_done_goal[1]) < REPEAT_DIST:
            self.get_logger().info(
                f"Frontier ({gx:.2f}, {gy:.2f}) won't clear; blacklisting and moving on.")
            self.blacklist.append((gx, gy))
            return

        yaw = math.atan2(gy - ry, gx - rx)
        self.send_goal(gx, gy, yaw)

    def enter_stuck(self, rx, ry):
        self.get_logger().warn(">>> STUCK - no progress. Cancelling goal. Please TELEOP me free. <<<")
        self.blacklist.append(self.cur_goal)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.navigating = False
        self.state = "STUCK"
        self.stuck_pos = (rx, ry)

    def send_goal(self, x, y, yaw):
        if not self.nav_client.server_is_ready():
            self.get_logger().info("Waiting for Nav2 action server...")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.navigating = True
        self.sent_a_goal = True
        self.cur_goal = (x, y)
        self.last_move_time = self.now()   # reset the stuck timer for the new goal
        self.get_logger().info(f"Exploring -> frontier at (x={x:.2f}, y={y:.2f}).")
        self.nav_client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Nav2 rejected the frontier; blacklisting it.")
            self.blacklist.append(self.cur_goal)
            self.navigating = False
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status != 4:                    # not SUCCEEDED -> unreachable, blacklist it
            self.blacklist.append(self.cur_goal)
        self.last_done_goal = self.cur_goal
        self.goal_handle = None
        if self.state != "STUCK":
            self.navigating = False

    def finish(self):
        self.done = True
        self.get_logger().info("*** AREA FULLY MAPPED - no frontiers left. ***")
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", SAVE_MAP_PATH],
                timeout=30, check=False,
            )
            self.get_logger().info(f"Map saved to {SAVE_MAP_PATH}.yaml / .pgm")
        except Exception as e:
            self.get_logger().warn(f"Auto-save failed ({e}); save manually with map_saver_cli.")


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()