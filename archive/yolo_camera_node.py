import base64

import cv2
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# ─────────────────────────────────────────────────────────────────
#  Config
# ─────────────────────────────────────────────────────────────────
SERVER_URL   = "http://127.0.0.1:5001/detect"   # the YOLO server in vla-box
CAMERA_TOPIC = "/oakd/rgb/preview/image_raw"      # robot's camera
PERIOD       = 1.0   # how often to run detection (seconds)


class YoloCameraNode(Node):
    def __init__(self):
        super().__init__("yolo_camera_node")
        self.bridge = CvBridge()
        self.latest_image = None

        self.create_subscription(Image, CAMERA_TOPIC, self.on_image, 10)
        self.create_timer(PERIOD, self.detect)

        self.get_logger().info("YOLO camera node started. Waiting for camera...")

    def on_image(self, msg):
        self.latest_image = msg

    def detect(self):
        if self.latest_image is None:
            self.get_logger().info("No camera image yet...")
            return

        # ROS image -> OpenCV (BGR) -> JPEG -> base64
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_img)
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")

        # Ask the YOLO server what's in view
        try:
            resp = requests.post(SERVER_URL, json={"image": img_b64}, timeout=15)
            detections = resp.json()["detections"]
        except Exception as e:
            self.get_logger().error(f"Detection server error: {e}")
            return

        # Report what the robot sees
        if detections:
            seen = [f"{d['name']}({d['confidence']:.2f})" for d in detections]
            self.get_logger().info(f"Robot sees: {', '.join(seen)}")
        else:
            self.get_logger().info("Robot sees: (nothing recognized)")


def main():
    rclpy.init()
    node = YoloCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()