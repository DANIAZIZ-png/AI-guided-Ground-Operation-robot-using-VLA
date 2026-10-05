import base64
import math
import sys

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point

# ─────────────────────────────────────────────────────────────────
#  Config
# ─────────────────────────────────────────────────────────────────
SERVER_URL    = "http://127.0.0.1:5001/detect"
RGB_TOPIC     = "/oakd/rgb/preview/image_raw"
DEPTH_TOPIC   = "/oakd/rgb/preview/depth"
INFO_TOPIC    = "/oakd/rgb/preview/camera_info"
MAP_FRAME     = "map"
ROBOT_FRAME   = "base_link"
STOP_DISTANCE = 0.6    # stop this many metres in front of the object
PERIOD        = 1.0


# ─────────────────────────────────────────────────────────────────
#  Step 4: pull the target object out of a full sentence
# ─────────────────────────────────────────────────────────────────
# These are command / filler words, NOT the object we want to drive to.
# Anything left after removing them is handed to YOLO-World as the target.
COMMAND_WORDS = {
    "go", "to", "the", "a", "an", "find", "locate", "look", "for",
    "drive", "navigate", "move", "head", "toward", "towards", "get",
    "please", "robot", "and", "then", "over", "at", "into", "up",
}


def parse_target(sentence):
    """'go to the person' -> 'person'.   'find the office chair' -> 'office chair'.

    Strips the command/filler words above; whatever remains is the object
    YOLO-World will search for. Because YOLO-World is open-vocabulary, the
    leftover word (or short phrase) just becomes its search prompt directly."""
    words = sentence.lower().strip().strip(".!?").split()
    keep = [w for w in words if w not in COMMAND_WORDS]
    return " ".join(keep).strip()


class ObjectNavigator(Node):
    def __init__(self, target):
        super().__init__("object_navigator")
        self.target = target
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None
        self.cam_frame = None
        self.goal_sent = False     # navigate only once

        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_timer(PERIOD, self.tick)
        self.get_logger().info(f"Object navigator started. Target: '{self.target}'")

    def on_rgb(self, msg):
        self.rgb = msg

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.K = msg.k
        self.cam_frame = msg.header.frame_id

    def tick(self):
        if self.goal_sent:
            return  # already navigating
        if self.rgb is None or self.depth is None or self.K is None:
            self.get_logger().info("Waiting for camera data...")
            return

        # 1. Detect via YOLO server
        cv_rgb = self.bridge.imgmsg_to_cv2(self.rgb, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_rgb)
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        try:
            resp = requests.post(SERVER_URL, json={"image": img_b64}, timeout=15)
            detections = resp.json()["detections"]
        except Exception as e:
            self.get_logger().error(f"Detection server error: {e}")
            return

        # 2. Find the target
        target_det = next((d for d in detections if self.target in d["name"]), None)
        if target_det is None:
            self.get_logger().info(f"'{self.target}' not in view. Seeing: {[d['name'] for d in detections]}")
            return

        # 3. Object center pixel + depth
        x1, y1, x2, y2 = target_det["box"]
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        depth_img = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
        h, w = depth_img.shape[:2]
        u = max(0, min(u, w - 1))
        v = max(0, min(v, h - 1))
        patch = depth_img[max(0, v - 3):v + 4, max(0, u - 3):u + 4].astype(float)
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            self.get_logger().info("No valid depth at the object.")
            return
        d = float(np.median(patch))
        if d > 100:
            d = d / 1000.0

        # 4. Pixel + depth -> 3D point in camera frame -> map frame
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
            robot_tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF not ready yet: {e}")
            return

        ox, oy = obj.point.x, obj.point.y
        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y

        # 5. Goal = stop short of the object, facing it
        dx, dy = ox - rx, oy - ry
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return
        gx = ox - (dx / dist) * STOP_DISTANCE
        gy = oy - (dy / dist) * STOP_DISTANCE
        yaw = math.atan2(dy, dx)

        self.send_goal(gx, gy, yaw)

    def send_goal(self, x, y, yaw):
        if not self.nav_client.server_is_ready():
            self.get_logger().info("Waiting for Nav2 action server...")
            return  # retry next tick

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_sent = True
        self.get_logger().info(
            f"Sending Nav2 goal: drive to (x={x:.2f}, y={y:.2f}) and face the {self.target}."
        )
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the goal.")
            self.goal_sent = False
            return
        self.get_logger().info("Nav2 accepted the goal. Driving to the target...")
        goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status == 4:   # 4 = SUCCEEDED
            self.get_logger().info(f"*** ARRIVED at the {self.target}! Goal achieved. ***")
        else:
            self.get_logger().info(f"Navigation ended with status {status}.")


def main():
    rclpy.init()

    # Accept either a quoted sentence OR loose words after the script name:
    #   python3 object_navigator.py go to the person
    #   python3 object_navigator.py "find the chair"
    raw = " ".join(sys.argv[1:]) if len(sys.argv) > 1 else "person"
    target = parse_target(raw)

    if not target:
        print(f"Couldn't find a target object in: '{raw}'.  "
              f"Try something like:  go to the chair")
        rclpy.shutdown()
        return

    print(f"Instruction: '{raw}'   ->   target object: '{target}'")
    node = ObjectNavigator(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():        # avoids the 'rcl_shutdown already called' traceback
            rclpy.shutdown()


if __name__ == "__main__":
    main()