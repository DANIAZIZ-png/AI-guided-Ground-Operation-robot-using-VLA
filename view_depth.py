#!/usr/bin/env python3
# Depth viewer - colourises the depth map and prints the centre distance

import rclpy
# ROS 2 Python client library
from rclpy.node import Node
# Base class for all ROS 2 nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# Pieces used to build the depth-1 QoS profile
from sensor_msgs.msg import Image
# Depth arrives as a normal Image message, just with 16-bit pixels
import numpy as np
# Reshapes the buffer and does the median maths
import cv2
# Colourises and displays

TOPIC = "/robot1/oakd/stereo/image_raw"
# The depth stream we just enabled

LIVE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    # Match the publisher so the subscription actually connects
    history=HistoryPolicy.KEEP_LAST,
    # Keep only the newest frames
    depth=1,
    # Queue of one - no backlog, no lag
)


class DepthViewer(Node):
    def __init__(self):
        super().__init__("depth_viewer")
        # Register the node with ROS
        self.create_subscription(Image, TOPIC, self.cb, LIVE_QOS)
        # Subscribe to the depth topic
        cv2.namedWindow("depth", cv2.WINDOW_NORMAL)
        # Resizable window
        self.first = True
        # Flag so we only print the format details once

    def cb(self, msg):
        # Runs on every incoming depth frame
        if msg.encoding == "16UC1":
            d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            # 16-bit unsigned integers, one per pixel, units are millimetres
            scale = 0.001
            # Multiplier to convert millimetres into metres
        else:
            d = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
            # 32-bit floats, already in metres
            scale = 1.0
            # No conversion needed

        if self.first:
            self.get_logger().info(
                f"{msg.width}x{msg.height} encoding={msg.encoding}")
            # Print resolution and encoding once for the record
            self.first = False

        cx, cy = msg.width // 2, msg.height // 2
        # Coordinates of the exact centre pixel
        patch = d[cy - 2:cy + 3, cx - 2:cx + 3].astype(np.float32)
        # Grab a 5x5 block around the centre - same idea as robust_box_depth
        valid = patch[patch > 0]
        # Drop zero pixels, which mean "no depth measured here"
        centre_m = float(np.median(valid)) * scale if valid.size else 0.0
        # Median of the valid pixels ignores speckle noise and holes

        vis = cv2.convertScaleAbs(d, alpha=0.03)
        # Squash the wide depth range into 0-255 so it can be displayed
        vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        # Colour code it - near is blue, far is red
        cv2.circle(vis, (cx, cy), 5, (255, 255, 255), 2)
        # Mark the pixel we sampled
        cv2.putText(vis, f"centre {centre_m:.2f} m", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # Show the measured distance so you can check it with a tape measure

        cv2.imshow("depth", vis)
        # Draw the frame
        cv2.waitKey(1)
        # Let the GUI redraw


def main():
    rclpy.init()
    # Start ROS 2
    node = DepthViewer()
    # Build the viewer
    try:
        rclpy.spin(node)
        # Loop forever
    except KeyboardInterrupt:
        pass
        # Clean Ctrl+C
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
        # Tidy up


if __name__ == "__main__":
    main()