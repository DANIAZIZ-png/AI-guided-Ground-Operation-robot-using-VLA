#!/usr/bin/env python3
"""
Reset the TurtleBot 4's position in BOTH Gazebo and RViz/Nav2.

Run from INSIDE the GPU box (ubuntu22-gpu), while Gazebo + Nav2 are running:
    python3 reset_robot.py              # reset to origin (0, 0), facing forward
    python3 reset_robot.py 1.5 2.0      # reset to x=1.5, y=2.0
    python3 reset_robot.py 1.5 2.0 1.57 # also set heading (yaw, in radians)

Step 1 teleports the robot in Gazebo (ign 'set_pose' service).
Step 2 tells the ROS localization where the robot now is, by publishing to
        /initialpose -- exactly what RViz's "2D Pose Estimate" button does --
        so AMCL re-localizes and RViz snaps the robot to the new spot.

NOTE: Step 2 only takes effect in LOCALIZATION mode (AMCL: a saved map +
localization:=true -- your reliable demo). In live SLAM mode (slam_toolbox)
/initialpose is ignored and a teleport desyncs the live map, so restarting
SLAM is cleaner there.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

# ─────────────────────────────────────────────────────────────────
#  Settings — change WORLD_NAME if your world is not "warehouse"
#  (verify with:  ign service --list | grep set_pose )
# ─────────────────────────────────────────────────────────────────
WORLD_NAME = "warehouse"     # the Gazebo world name
ROBOT_NAME = "turtlebot4"    # the robot model name in Gazebo
SPAWN_Z    = 0.05            # small height above the floor so it doesn't clip
MAP_FRAME  = "map"


def yaw_to_quaternion(yaw):
    """Convert a yaw angle (radians) into the z/w parts of a quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def reset_in_gazebo(x, y, qz, qw):
    """Step 1 — teleport the robot body in the Gazebo world."""
    request = (
        f'name: "{ROBOT_NAME}", '
        f'position: {{x: {x}, y: {y}, z: {SPAWN_Z}}}, '
        f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
    )
    cmd = [
        "ign", "service",
        "-s", f"/world/{WORLD_NAME}/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "--req", request,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    output = (result.stdout or result.stderr).strip()
    if "true" in output.lower():
        print("  Gazebo : robot teleported.")
        return True
    print("  Gazebo : service response:", output)
    print("  (If this failed, check WORLD_NAME at the top, or try 'gz' instead of 'ign'.)")
    return False


def reset_in_rviz(x, y, qz, qw):
    """Step 2 — tell ROS localization (AMCL/Nav2) the new pose, so RViz follows."""
    rclpy.init()
    node = rclpy.create_node("reset_robot_initialpose")
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = MAP_FRAME
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = float(qz)
    msg.pose.pose.orientation.w = float(qw)
    # same default spread RViz's "2D Pose Estimate" uses, so AMCL trusts it
    msg.pose.covariance[0]  = 0.25     # x
    msg.pose.covariance[7]  = 0.25     # y
    msg.pose.covariance[35] = 0.0685   # yaw

    # let discovery settle, then publish a few times so AMCL definitely hears it
    time.sleep(0.5)
    for _ in range(10):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)

    print("  RViz   : published new pose to /initialpose.")
    node.destroy_node()
    rclpy.shutdown()


def main():
    x   = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y   = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    qz, qw = yaw_to_quaternion(yaw)

    print(f"Resetting '{ROBOT_NAME}' to x={x}, y={y}, yaw={yaw} rad ...")
    reset_in_gazebo(x, y, qz, qw)
    reset_in_rviz(x, y, qz, qw)
    print("Done — robot repositioned in Gazebo and RViz.")


if __name__ == "__main__":
    main()