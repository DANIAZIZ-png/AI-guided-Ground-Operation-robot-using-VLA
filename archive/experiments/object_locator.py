import base64
import math
import sys

import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point

# ─────────────────────────────────────────────────────────────────
#  Config
# ─────────────────────────────────────────────────────────────────
SERVER_URL   = "http://127.0.0.1:5001/detect"
RGB_TOPIC    = "/oakd/rgb/preview/image_raw"
DEPTH_TOPIC  = "/oakd/rgb/preview/depth"
INFO_TOPIC   = "/oakd/rgb/preview/camera_info"
MAP_FRAME    = "map"     # the fixed world frame Nav2 uses
PERIOD       = 1.0


class ObjectLocator(Node):
    def __init__(self, target):
        super().__init__("object_locator")
        self.target = target
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.K = None          # camera intrinsics [fx,0,cx, 0,fy,cy, 0,0,1]
        self.cam_frame = None  # the camera's coordinate frame name

        self.create_subscription(Image, RGB_TOPIC, self.on_rgb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth, 10)
        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)

        # For converting a point from the camera frame to the map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(PERIOD, self.locate)
        self.get_logger().info(f"Object locator started. Looking for: '{self.target}'")

    def on_rgb(self, msg):
        self.rgb = msg

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.K = msg.k
        self.cam_frame = msg.header.frame_id

    def locate(self):
        if self.rgb is None or self.depth is None or self.K is None:
            self.get_logger().info("Waiting for camera data...")
            return

        # 1. Detect via the YOLO server
        cv_rgb = self.bridge.imgmsg_to_cv2(self.rgb, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_rgb)
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        try:
            resp = requests.post(SERVER_URL, json={"image": img_b64}, timeout=15)
            detections = resp.json()["detections"]
        except Exception as e:
            self.get_logger().error(f"Detection server error: {e}")
            return

        # 2. Find the target object among the detections
        target_det = next((d for d in detections if self.target in d["name"]), None)
        if target_det is None:
            seen = [d["name"] for d in detections]
            self.get_logger().info(f"'{self.target}' not in view. Seeing: {seen}")
            return

        # 3. Center pixel of the object's box
        x1, y1, x2, y2 = target_det["box"]
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        # 4. Read depth around that pixel (median of a small patch = robust)
        depth_img = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
        h, w = depth_img.shape[:2]
        u = max(0, min(u, w - 1))
        v = max(0, min(v, h - 1))
        patch = depth_img[max(0, v - 3):v + 4, max(0, u - 3):u + 4].astype(float)
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            self.get_logger().info("No valid depth reading at the object.")
            return
        d = float(np.median(patch))
        if d > 100:        # depth came in millimetres, convert to metres
            d = d / 1000.0

        # 5. Turn (pixel + depth) into a 3D point in the camera's frame
        fx, fy = self.K[0], self.K[4]
        cx, cy = self.K[2], self.K[5]
        pt = PointStamped()
        pt.header.frame_id = self.cam_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = (u - cx) * d / fx
        pt.point.y = (v - cy) * d / fy
        pt.point.z = d

        # 6. Transform that point into the map frame
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, self.cam_frame, rclpy.time.Time())
            map_pt = do_transform_point(pt, tf)
        except Exception as e:
            self.get_logger().warn(f"Couldn't transform to '{MAP_FRAME}' yet: {e}")
            return

        self.get_logger().info(
            f"'{self.target}' is {d:.2f} m away  ->  map coordinate "
            f"(x={map_pt.point.x:.2f}, y={map_pt.point.y:.2f})"
        )


def main():
    rclpy.init()
    target = sys.argv[1] if len(sys.argv) > 1 else "box"
    node = ObjectLocator(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()