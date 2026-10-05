#!/usr/bin/env python3
"""
Spawn detection-target objects into the RUNNING Gazebo (Ignition) world.

Run from INSIDE ubuntu22-gpu, AFTER Gazebo is up:
    python3 spawn_objects.py

Edit the OBJECTS list to choose what to place and where. Spawned objects are
NOT saved into the world file, so just re-run this script after each launch to
repopulate. Positions are world (map) coordinates -- pick open floor so the
robot can see and reach them; nudge the x/y if anything spawns inside a shelf.
"""
import math
import os
import subprocess

# ─────────────────────────────────────────────────────────────────
#  Settings
# ─────────────────────────────────────────────────────────────────
WORLD_NAME = "warehouse"                       # change if your world differs
OBJ_DIR    = os.environ.get("VLA_SIM_OBJ_DIR") or os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sim_objects")
# where the .sdf files live; VLA_SIM_OBJ_DIR overrides

# (sdf file, spawn name, x, y, z, yaw)   z = half the object's height (sits on floor)
OBJECTS = [
    ("red_box.sdf", "red_box1", 2.0,  1.0, 0.20, 0.0),
    ("cup.sdf",     "cup1",     2.5, -0.5, 0.06, 0.0),
    ("door.sdf",    "door1",    4.0,  0.0, 1.00, 1.5708),
]


def spawn(sdf_file, name, x, y, z, yaw):
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    path = os.path.join(OBJ_DIR, sdf_file)
    if not os.path.exists(path):
        print(f"  {name}: SKIPPED - file not found at {path}")
        return
    req = (
        f'sdf_filename: "{path}", '
        f'name: "{name}", '
        f'allow_renaming: true, '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}}}'
    )
    cmd = [
        "ign", "service",
        "-s", f"/world/{WORLD_NAME}/create",
        "--reqtype", "ignition.msgs.EntityFactory",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "--req", req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    output = (result.stdout or result.stderr).strip()
    print(f"  {name}: {'spawned' if 'true' in output.lower() else output}")


if __name__ == "__main__":
    print(f"Spawning {len(OBJECTS)} object(s) into world '{WORLD_NAME}' ...")
    for obj in OBJECTS:
        spawn(*obj)
    print("Done. If something failed, check WORLD_NAME, or try 'gz' instead of 'ign'.")