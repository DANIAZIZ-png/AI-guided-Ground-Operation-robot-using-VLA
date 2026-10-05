import numpy as np

# ─────────────────────────────────────────────────────────────────
#  TurtleBot 4 Lite SAFE velocity limits (hard safety caps)
#  The robot can NEVER be told to move faster than these, no matter
#  what OpenVLA outputs. This protects the robot and anything near it.
# ─────────────────────────────────────────────────────────────────
MAX_LINEAR  = 0.30   # metres per second   (forward / backward)
MAX_ANGULAR = 1.00   # radians per second  (turning)

# ─────────────────────────────────────────────────────────────────
#  Tuning knobs — adjust these while testing in Gazebo
# ─────────────────────────────────────────────────────────────────
LINEAR_SCALE  = 30.0  # how strongly OpenVLA's X maps to forward speed
ANGULAR_SCALE = 30.0  # how strongly OpenVLA's yaw maps to turn rate
DEADZONE      = 0.001 # ignore only truly tiny values (treat as noise)


def vla_action_to_cmd_vel(action):
    """
    Translate OpenVLA's 7-value manipulator action into a wheeled-base
    velocity command.

    OpenVLA output layout:
        [move_x, move_y, move_z, roll, pitch, yaw, gripper]
              0       1       2     3      4     5        6

    We use only two of these for a wheeled robot:
        move_x (index 0) -> drive forward / backward
        yaw    (index 5) -> turn left / right
    The other values (Y, Z, roll, pitch, gripper) are ignored because
    a wheeled base cannot use them — it has no arm and cannot fly.

    Parameters
    ----------
    action : array-like of 7 floats

    Returns
    -------
    dict with keys 'linear_x' (m/s) and 'angular_z' (rad/s)
    """
    a = np.asarray(action, dtype=float).flatten()

    # Pull out the two values that map most naturally to ground motion
    move_x = a[0]
    yaw    = a[5]

    # Deadzone: kill tiny noisy values so the robot doesn't jitter
    if abs(move_x) < DEADZONE:
        move_x = 0.0
    if abs(yaw) < DEADZONE:
        yaw = 0.0

    # Scale the small action values up into a useful velocity range
    linear_x  = -move_x * LINEAR_SCALE
    angular_z = yaw    * ANGULAR_SCALE

    # Clamp to safe limits — this is the safety net
    linear_x  = float(np.clip(linear_x,  -MAX_LINEAR,  MAX_LINEAR))
    angular_z = float(np.clip(angular_z, -MAX_ANGULAR, MAX_ANGULAR))

    return {"linear_x": linear_x, "angular_z": angular_z}


# ─────────────────────────────────────────────────────────────────
#  Standalone test — run this file directly to check the mapping
#  WITHOUT needing OpenVLA or ROS 2 running. Pure logic check.
# ─────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    test_actions = {
        "strong forward": [ 0.08, 0.0, 0.0, 0.0, 0.0, 0.00, 1.0],
        "forward + turn": [ 0.05, 0.0, 0.0, 0.0, 0.0, 0.15, 1.0],
        "turn only":      [ 0.00, 0.0, 0.0, 0.0, 0.0, 0.30, 1.0],
        "tiny noise":     [ 0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 1.0],
        "backward":       [-0.06, 0.0, 0.0, 0.0, 0.0, 0.00, 1.0],
        "over the limit": [ 0.50, 0.0, 0.0, 0.0, 0.0, 0.90, 1.0],
    }

    print()
    print(f"{'Test case':<16}{'linear_x (m/s)':>16}{'angular_z (rad/s)':>20}")
    print("-" * 52)
    for name, act in test_actions.items():
        out = vla_action_to_cmd_vel(act)
        print(f"{name:<16}{out['linear_x']:>16.3f}{out['angular_z']:>20.3f}")
    print()
    print("Expected behaviour:")
    print("  - 'tiny noise'     -> both 0.000  (deadzone killed it)")
    print("  - 'over the limit' -> capped at MAX_LINEAR / MAX_ANGULAR")
    print()