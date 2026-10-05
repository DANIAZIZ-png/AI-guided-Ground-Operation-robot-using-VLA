import base64
import math
import sys

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point

# ─────────────────────────────────────────────────────────────────
#  Config  (tune these)
# ─────────────────────────────────────────────────────────────────
SERVER_URL    = "http://127.0.0.1:5001/detect"
RGB_TOPIC     = "/oakd/rgb/preview/image_raw"
DEPTH_TOPIC   = "/oakd/rgb/preview/depth"
INFO_TOPIC    = "/oakd/rgb/preview/camera_info"
SCAN_TOPIC    = "/scan"
ODOM_TOPIC    = "/odom"
CMD_TOPIC     = "/cmd_vel"
MAP_FRAME     = "map"
ROBOT_FRAME   = "base_link"

STOP_DISTANCE = 0.6     # final standoff in front of the object
ARRIVE_TOL    = 0.35    # if within STOP_DISTANCE + this, call it arrived
MAX_HOP       = 2.5     # never send a goal further than this (stays on mapped area)

SEARCH_TURN_SPEED = 0.5    # rad/s while scanning in place
ADVANCE_SPEED     = 0.20   # m/s while driving to a new area to map
ADVANCE_DISTANCE  = 1.5    # metres to advance before scanning again
OBSTACLE_STOP     = 0.6    # don't advance if something is this close ahead (metres)
ENABLE_ADVANCE    = True   # set False to ONLY rotate-scan (never drive forward to explore)

THINK_PERIOD = 1.0    # detect + decide (slow brain)
PUB_PERIOD   = 0.1    # publish cmd_vel at 10 Hz (Create 3 watchdog)


# ─── Natural-language: 'go to the chair' -> 'chair' ───────────────
COMMAND_WORDS = {
    "go", "to", "the", "a", "an", "find", "locate", "look", "for",
    "drive", "navigate", "move", "head", "toward", "towards", "get",
    "please", "robot", "and", "then", "over", "at", "into", "up",
}

def parse_target(sentence):
    words = sentence.lower().strip().strip(".!?").split()
    keep = [w for w in words if w not in COMMAND_WORDS]
    return " ".join(keep).strip()


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObjectExplorer(Node):
    def __init__(self, target):
        super().__init__("object_explorer")
        self.target = target
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.cam_frame = None

        # odom + lidar state
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.front_min = float("inf")

        # search bookkeeping
        self.state = "SEARCH_ROTATE"   # SEARCH_ROTATE | SEARCH_ADVANCE | NAVIGATING | DONE
        self.turn_accum = 0.0
        self.last_yaw = None
        self.advance_start = None
        self.cmd = Twist()             # what the fast publisher sends

        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(THINK_PERIOD, self.think)   # slow brain
        self.create_timer(PUB_PERIOD, self.publish_cmd)  # fast hands (10 Hz)
        self.get_logger().info(f"Explorer started. Searching for: '{self.target}'")

    # ── sensor callbacks ──
    def on_rgb(self, m): self.rgb = m
    def on_depth(self, m): self.depth = m
    def on_info(self, m): self.K = m.k; self.cam_frame = m.header.frame_id

    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.have_odom = True

    def on_scan(self, m):
        # minimum range in a +/-15 deg cone straight ahead (angle 0 = forward)
        n = len(m.ranges)
        if n == 0:
            return
        cone = math.radians(15)
        best = float("inf")
        for i, r in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if -cone <= ang <= cone and math.isfinite(r) and r > 0.0:
                best = min(best, r)
        self.front_min = best

    # ── fast publisher: keep the wheels fed at 10 Hz while searching ──
    def publish_cmd(self):
        if self.state == "NAVIGATING":
            return            # Nav2 owns /cmd_vel during a goal
        self.cmd_pub.publish(self.cmd)   # zero Twist when DONE = stay stopped

    # ── perception: detect target and return (ox,oy,rx,ry,dist) in map ──
    def locate_target(self):
        cv_rgb = self.bridge.imgmsg_to_cv2(self.rgb, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_rgb)
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        try:
            resp = requests.post(SERVER_URL, json={"image": img_b64}, timeout=15)
            detections = resp.json()["detections"]
        except Exception as e:
            self.get_logger().error(f"Detection server error: {e}")
            return None
        det = next((d for d in detections if self.target in d["name"]), None)
        if det is None:
            self.get_logger().info(
                f"Searching... '{self.target}' not in view. Seeing: {[d['name'] for d in detections]}"
            )
            return None

        x1, y1, x2, y2 = det["box"]
        u = int((x1 + x2) / 2); v = int((y1 + y2) / 2)
        depth_img = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
        h, w = depth_img.shape[:2]
        u = max(0, min(u, w - 1)); v = max(0, min(v, h - 1))
        patch = depth_img[max(0, v - 3):v + 4, max(0, u - 3):u + 4].astype(float)
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            return None
        d = float(np.median(patch))
        if d > 100:
            d = d / 1000.0

        fx, fy = self.K[0], self.K[4]; cx, cy = self.K[2], self.K[5]
        pt = PointStamped()
        pt.header.frame_id = self.cam_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = (u - cx) * d / fx
        pt.point.y = (v - cy) * d / fy
        pt.point.z = d
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, self.cam_frame, rclpy.time.Time())
            obj = do_transform_point(pt, tf)
            rtf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return None
        ox, oy = obj.point.x, obj.point.y
        rx, ry = rtf.transform.translation.x, rtf.transform.translation.y
        return ox, oy, rx, ry, math.hypot(ox - rx, oy - ry)

    # ── slow brain ──
    def think(self):
        if self.state in ("DONE", "NAVIGATING"):
            return
        if self.rgb is None or self.depth is None or self.K is None or not self.have_odom:
            self.get_logger().info("Waiting for sensors...")
            return

        # 1) Always try to detect — if we see it, stop searching and go.
        found = self.locate_target()
        if found is not None:
            ox, oy, rx, ry, dist = found
            self.get_logger().info(f"Found '{self.target}' at map (x={ox:.2f}, y={oy:.2f}), {dist:.2f} m away.")
            if dist <= STOP_DISTANCE + ARRIVE_TOL:
                self.get_logger().info(f"*** ARRIVED at the {self.target}! ***")
                self.cmd = Twist()
                self.state = "DONE"
                return
            dx, dy = ox - rx, oy - ry
            yaw = math.atan2(dy, dx)
            hop = min(dist - STOP_DISTANCE, MAX_HOP)
            gx, gy = rx + (dx / dist) * hop, ry + (dy / dist) * hop
            self.cmd = Twist()             # stop searching motion
            if self.send_goal(gx, gy, yaw):
                self.state = "NAVIGATING"
            return

        # 2) Not found -> keep searching.
        if self.state == "SEARCH_ROTATE":
            self.do_rotate_scan()
        elif self.state == "SEARCH_ADVANCE":
            self.do_advance()

    def do_rotate_scan(self):
        # accumulate how far we've turned; after a full circle, go explore forward
        if self.last_yaw is None:
            self.last_yaw = self.yaw
        dyaw = abs(math.atan2(math.sin(self.yaw - self.last_yaw),
                              math.cos(self.yaw - self.last_yaw)))
        self.turn_accum += dyaw
        self.last_yaw = self.yaw

        t = Twist(); t.angular.z = SEARCH_TURN_SPEED
        self.cmd = t

        if self.turn_accum >= 2 * math.pi:
            self.turn_accum = 0.0
            self.last_yaw = None
            if ENABLE_ADVANCE:
                self.advance_start = (self.x, self.y)
                self.state = "SEARCH_ADVANCE"
                self.get_logger().info("Full scan done, advancing to map a new area...")

    def do_advance(self):
        # blocked ahead? turn instead of ramming a wall
        if self.front_min < OBSTACLE_STOP:
            self.get_logger().info("Obstacle ahead, turning to a new heading.")
            self.state = "SEARCH_ROTATE"
            self.last_yaw = None
            self.turn_accum = math.pi   # turn ~half a circle before scanning again
            return
        if self.advance_start is not None:
            moved = math.hypot(self.x - self.advance_start[0], self.y - self.advance_start[1])
            if moved >= ADVANCE_DISTANCE:
                self.state = "SEARCH_ROTATE"
                self.last_yaw = None
                self.turn_accum = 0.0
                return
        t = Twist(); t.linear.x = ADVANCE_SPEED
        self.cmd = t

    # ── Nav2 ──
    def send_goal(self, x, y, yaw):
        if not self.nav_client.server_is_ready():
            self.get_logger().info("Waiting for Nav2 action server...")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f"Driving toward the {self.target}: goal (x={x:.2f}, y={y:.2f}).")
        self.nav_client.send_goal_async(goal).add_done_callback(self.on_goal_response)
        return True

    def on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Nav2 rejected the goal. Resuming search.")
            self.state = "SEARCH_ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
            return
        gh.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info("Hop complete. Re-checking the target...")
        else:
            self.get_logger().warn(f"Goal ended with status {status}. Resuming search.")
        # back to the brain: next think() re-detects (and re-hops if still visible)
        self.state = "SEARCH_ROTATE"; self.last_yaw = None; self.turn_accum = 0.0


def main():
    rclpy.init()
    raw = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "person"
    target = parse_target(raw)
    if not target:
        print(f"Couldn't find a target object in: '{raw}'. Try: go to the chair")
        rclpy.shutdown(); return
    print(f"Instruction: '{raw}'   ->   target object: '{target}'")
    node = ObjectExplorer(target)
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