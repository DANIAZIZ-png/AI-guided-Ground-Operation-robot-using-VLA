#!/usr/bin/env python3
# v3 - JPEG compressed transport, depth-1 QoS. ~10x less Wi-Fi traffic.

import rclpy
# ROS 2 Python client library
from rclpy.node import Node
# Base class every ROS 2 node inherits from
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# Pieces used to build a custom QoS profile
from sensor_msgs.msg import CompressedImage
# Message type that carries a JPEG buffer instead of raw pixels
import numpy as np
# Wraps the JPEG bytes for OpenCV to decode
import cv2
# Decodes the JPEG and draws the window

TOPIC = "/robot1/oakd/rgb/preview/image_raw/compressed"
# The JPEG topic - about 10x smaller than image_raw over Wi-Fi
SCALE = 3
# Display magnification only - does not add real detail

LIVE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    # Matches the publisher - dropped frames are never retransmitted
    history=HistoryPolicy.KEEP_LAST,
    # Keep only the newest messages
    depth=1,
    # Exactly one frame queued - a new frame replaces the old one, no backlog
)


class Viewer(Node):
    def __init__(self):
        super().__init__("cam_viewer_compressed")
        # Register this node with ROS
        self.create_subscription(CompressedImage, TOPIC, self.cb, LIVE_QOS)
        # Subscribe to the JPEG stream with our depth-1 profile
        cv2.namedWindow("OAK-D compressed", cv2.WINDOW_NORMAL)
        # WINDOW_NORMAL makes the window drag-resizable
        self.n = 0
        # Frame counter
        self.get_logger().info(f"listening on {TOPIC}")
        # Confirms the subscription was created

    def cb(self, msg):
        # Runs each time a JPEG frame arrives
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        # Wrap the compressed bytes without copying them
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        # Decode JPEG into a normal BGR image
        if img is None:
            return
            # Corrupt or partial frame - skip it rather than crash
        self.n += 1
        # Advance the frame counter

        big = cv2.resize(img, None, fx=SCALE, fy=SCALE,
                         interpolation=cv2.INTER_NEAREST)
        # Enlarge purely for viewing - fastest interpolation, no blur

        kb = len(msg.data) / 1024.0
        # Size of this frame in kilobytes - shows the bandwidth saving
        cv2.putText(big, f"#{self.n}  {kb:5.1f} KB", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # Overlay counter and frame size on the image

        cv2.imshow("OAK-D compressed", big)
        # Draw it
        cv2.waitKey(1)
        # 1 ms for the GUI to redraw - mandatory or nothing appears


def main():
    rclpy.init()
    # Start the ROS 2 runtime
    node = Viewer()
    # Build the viewer
    try:
        rclpy.spin(node)
        # Loop forever handling incoming frames
    except KeyboardInterrupt:
        pass
        # Clean Ctrl+C exit
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
        # Release window and ROS resources


if __name__ == "__main__":
    main()