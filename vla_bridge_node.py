import base64
import sys

import cv2
import requests
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# ─────────────────────────────────────────────────────────────────
#  Config
# ─────────────────────────────────────────────────────────────────
SERVER_URL     = "http://127.0.0.1:5000/predict"   # the brain in vla-box
CAMERA_TOPIC   = "/oakd/rgb/preview/image_raw"      # robot's eyes
CMD_VEL_TOPIC  = "/cmd_vel"                          # robot's wheels
# Instruction comes from the command line if you give one, else this default.
# Example:  python3 vla_bridge_node.py "go to the door"
INSTRUCTION    = sys.argv[1] if len(sys.argv) > 1 else "move forward and patrol the area"

THINK_PERIOD   = 1.0    # how often OpenVLA decides (slow, ~1x per second)
PUBLISH_PERIOD = 0.1    # how often we RESEND the command (fast, 10x per second)


class VlaBridgeNode(Node):
    def __init__(self):
        super().__init__("vla_bridge_node")
        self.bridge = CvBridge()
        self.latest_image = None
        self.current_cmd = Twist()   # latest velocity from the brain (starts at 0 = stopped)

        # Two callback groups so the SLOW think() call cannot freeze the FAST publish loop.
        # slow_group: only one think at a time. fast_group: publish + camera run freely.
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.fast_group = ReentrantCallbackGroup()

        # Listen to the camera (fast group so it keeps updating during inference)
        self.create_subscription(
            Image, CAMERA_TOPIC, self.on_image, 10, callback_group=self.fast_group
        )

        # Publisher that drives the robot
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # SLOW loop: ask OpenVLA what to do
        self.create_timer(THINK_PERIOD, self.think, callback_group=self.slow_group)
        # FAST loop: keep re-sending the latest command so the robot doesn't time out
        self.create_timer(PUBLISH_PERIOD, self.publish_current, callback_group=self.fast_group)

        self.get_logger().info("VLA bridge started. Waiting for camera images...")

    def on_image(self, msg):
        # Remember the most recent camera frame
        self.latest_image = msg

    def think(self):
        """SLOW: ask OpenVLA what to do, and update current_cmd."""
        if self.latest_image is None:
            self.get_logger().info("No camera image yet...")
            return

        # ROS image -> OpenCV (BGR) -> JPEG -> base64 text
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_img)
        if not ok:
            self.get_logger().error("Could not encode camera image.")
            return
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")

        # Ask the brain (this is the slow part)
        try:
            resp = requests.post(
                SERVER_URL,
                json={"image": img_b64, "instruction": INSTRUCTION},
                timeout=60,
            )
            cmd = resp.json()
        except Exception as e:
            # If the brain is unreachable, stop the robot for safety
            self.get_logger().error(f"Brain unreachable ({e}). Stopping robot.")
            self.current_cmd = Twist()
            return

        # Save the new decision — the fast loop will keep sending it
        twist = Twist()
        twist.linear.x  = float(cmd["linear_x"])
        twist.angular.z = float(cmd["angular_z"])
        self.current_cmd = twist

        self.get_logger().info(
            f"New decision -> linear={twist.linear.x:+.3f} m/s   "
            f"angular={twist.angular.z:+.3f} rad/s"
        )

    def publish_current(self):
        """FAST: re-send the latest command so the robot keeps moving."""
        self.cmd_pub.publish(self.current_cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())   # all zeros = stop


def main():
    rclpy.init()
    node = VlaBridgeNode()

    # MultiThreadedExecutor lets the slow think() and the fast publish() run at the
    # same time, in separate threads, so neither blocks the other.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()