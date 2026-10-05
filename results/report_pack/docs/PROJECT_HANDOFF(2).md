# Project Handoff — AI-Guided Ground Operations Robot Using VLA Models

**Cadet:** Danyal Aziz · **Supervisor:** Sqn Ldr Raja Farukh
**Platform:** TurtleBot 4 Lite (Create 3 base) · ROS 2 Humble · Gazebo (Ignition)
**Last updated:** reflects completion of perception + navigation; final integration node delivered, awaiting test.

---

## 1. What the project is

Build an autonomous ground robot for ISR / patrol that takes a **natural-language instruction** (e.g. "go to the person") and **navigates to the named target** on its own — indoor navigation, obstacle avoidance, and language-commanded object-seeking.

**The core research problem — cross-embodiment transfer.** OpenVLA (the VLA model) was trained to move a 7-DoF robot *arm* on a *tabletop*. This project needed it to drive a *wheeled mobile base*. Those are different "bodies" with different action spaces.

**The thesis contribution.** Rather than fine-tune the VLA (impractical on a semester timeline), the project evaluates direct VLA control as a **baseline**, shows it fails, and then builds a **modular architecture** that works: an open-vocabulary perception model (YOLO-World) for seeing + Nav2 for navigating.

---

## 2. The research story (the before / after)

**Baseline — direct VLA control (the "before"):**
Feeding navigation images + instructions to OpenVLA and mapping its arm output to wheel velocities produces motion, but **not purposeful navigation**. The output direction is erratic (flips between forward/backward unpredictably), the instruction doesn't meaningfully change behaviour, and the robot wanders / trips the Create 3's backward safety limit. This is the documented finding: *a manipulation-trained VLA does not transfer to mobile navigation by direct mapping.* **(Recorded.)**

**Modular system — perception + Nav2 (the "after"):**
A real perception model (YOLO-World) detects the target by text prompt, the system computes the target's map coordinate from camera depth, and Nav2 plans a path and drives there avoiding obstacles. This produces **purposeful, language-commanded navigation.** **(In final integration.)**

---

## 3. Current status

| Component | Status |
|---|---|
| OpenVLA running persistently (baseline) | Done |
| TurtleBot 4 in Gazebo (camera, teleop, sensors) | Done |
| Direct VLA -> cmd_vel baseline + finding | Done & recorded |
| Nav2 autonomous navigation (click goal -> robot drives there) | Done & recorded |
| YOLO-World detection (static image) | Done |
| YOLO-World on the robot's LIVE camera | Done |
| Object localization (detection -> map coordinate) | Done |
| Object navigator (coordinate -> Nav2 goal -> drive -> confirm) | Delivered, awaiting test |
| Parse full instruction sentences (Step 4) | Next |
| Final documentation / presentation polish | Pending |

---

## 4. The working architecture (modular pipeline)

```
"go to the person"
        |
        v
 (target = "person")
        |
        v
 robot camera --> YOLO-World server (vla-box, GPU) --> where the object is in the image
        |
        v
 read depth at that pixel + camera lens info --> 3D point relative to robot
        |
        v
 transform to map frame (using robot's known position) --> object's MAP coordinate
        |
        v
 pick a goal just IN FRONT of the object, facing it --> send to Nav2
        |
        v
 Nav2 plans a path + drives there, avoiding obstacles --> ARRIVED
```

---

## 5. System layout

**Hardware:** Dell Precision 3660 · Ubuntu 24.04 host · NVIDIA RTX A4000 (16 GB) · TurtleBot 4 Lite.

**The home folder `~` is SHARED** across the host and both containers — a file created anywhere is visible everywhere. What matters is *where you run* a script, not where the file lives.

**Two Distrobox containers** (prompts look identical — use `echo $CONTAINER_ID` to tell which one you're in; empty = host):

- **`ubuntu22`** — ROS 2 Humble, TurtleBot 4 packages, Gazebo. **No GPU.** Runs all ROS nodes and the simulator. Uses `python3`, after `source /opt/ros/humble/setup.bash`.
- **`vla-box`** — created with `--nvidia`, **has the GPU.** Runs the model servers. Uses `python` *after activating a venv*.

**Two virtual environments (in vla-box, in shared `~`):**

- **`~/openvla-env`** — OpenVLA (baseline). **FROZEN — do not change.** Pinned: `transformers==4.40.1`, `accelerate==0.30.1` (critical), `bitsandbytes==0.43.2`, `timm==0.9.10`, `tokenizers==0.19.1`, PyTorch cu124, plus flask. Saved to `~/openvla_working_setup.txt`.
- **`~/yolo-env`** — YOLO-World (working system). Separate on purpose, to protect the fragile OpenVLA env. Has `ultralytics` + `flask`.

---

## 6. The files (all in shared `~`)

**Baseline (OpenVLA) — keep for the thesis "before":**
- `openvla_interactive.py` — interactive OpenVLA test loop.
- `openvla_action_translator.py` — converts OpenVLA's 7-DoF arm output to wheel velocities. Tuned: `LINEAR_SCALE=30, ANGULAR_SCALE=30, DEADZONE=0.001`; uses `abs(move_x)` to force forward motion (the model's sign is erratic).
- `vla_server.py` — OpenVLA Flask server, **port 5000**, loads model once + warmup.
- `vla_bridge_node.py` — ROS node (ubuntu22): camera -> server -> `/cmd_vel`. "Think slow 1 Hz, publish fast 10 Hz" to satisfy the Create 3 watchdog.

**Working system (YOLO-World + Nav2):**
- `yolo_test.py` — interactive YOLO-World tester (type/drag an image path).
- `yolo_server.py` — YOLO-World detection server, **port 5001**, `/detect` endpoint.
- `yolo_camera_node.py` — ROS node (ubuntu22): live camera -> YOLO server -> prints "Robot sees: ...".
- `object_locator.py` — ROS node (ubuntu22): detect target -> read depth -> compute & print the object's **map coordinate**.
- `object_navigator.py` — ROS node (ubuntu22): the finale. Detect target -> compute a goal just in front of it -> send to **Nav2** -> drive there -> report arrival.

**Utility:**
- `reset_robot.py` — teleport the robot back to a position in Gazebo: `python3 ~/reset_robot.py [x] [y] [yaw]`.

**Key sensor topics:**
- RGB: `/oakd/rgb/preview/image_raw`
- Depth (aligned with RGB): `/oakd/rgb/preview/depth`
- Camera info (lens details): `/oakd/rgb/preview/camera_info`
- Lidar: `/scan` · Movement: `/cmd_vel`

---

## 7. How to run the full working system

Free the GPU first if OpenVLA was running (`pkill -9 -f vla_server.py`, confirm with `nvidia-smi`). YOLO-World is small and shares the GPU with Gazebo fine.

**Terminal 1 — Nav2 stack (ubuntu22):**
```bash
distrobox enter ubuntu22
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite slam:=true nav2:=true rviz:=true
```
Wait until Gazebo + RViz are fully up (map building in RViz).

**Terminal 2 — YOLO detection server (vla-box):**
```bash
distrobox enter vla-box
source ~/yolo-env/bin/activate
python ~/yolo_server.py
```

**Terminal 3 — Object navigator (ubuntu22):**
```bash
distrobox enter ubuntu22
source /opt/ros/humble/setup.bash
python3 ~/object_navigator.py person
```
Replace `person` with any target word YOLO-World knows (`box`, `shelf`, `chair`, ...). Make sure the target is in the robot's camera view (nudge with teleop if needed). Expected: the node prints the goal, Nav2 accepts it, the robot drives over, and it prints **"ARRIVED at the person!"**

---

## 8. Recurring bugs & fixes (hard-won)

1. **Wrong box / venv.** Server + models -> **vla-box** + venv + `python`. ROS nodes -> **ubuntu22** + `source /opt/ros/humble/setup.bash` + `python3`. Never swap. `cv2`/`rclpy` errors usually mean a ROS node was run in vla-box.
2. **Which box am I in?** Prompts look identical — run `echo $CONTAINER_ID`.
3. **GPU contention.** OpenVLA (~15 GB) cannot coexist with Gazebo on a 16 GB card — Gazebo hangs / "not responding." **Free the GPU before launching Gazebo.** (YOLO-World is small and coexists fine.)
4. **GPU OOM / stale process.** `nvidia-smi` -> kill the big compute (`C`) python by PID, or `pkill -9 -f vla_server.py`. Never kill graphics (`G`) processes.
5. **Stale FastDDS shared memory** (TF "two unconnected trees", transport errors after restarts): stop nodes, then `rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`.
6. **Create 3 backward safety limit** ("Reached backup limit!"): the base refuses sustained reverse — handled with `abs(move_x)` (baseline only).
7. **Create 3 cmd_vel watchdog**: must publish ~10 Hz or the robot stalls between commands (baseline bridge uses fast-publish).
8. **Editing a file does nothing until you restart** the program that loaded it (especially a server).
9. **Gazebo launch arg**: `model:=lite` (not "lit"); valid options are `standard` / `lite`.
10. **`map` frame only exists when Nav2/SLAM is running** — the locator/navigator need the full Nav2 stack up, or TF-to-map fails.
11. **`tf2_geometry_msgs` import error**: `sudo apt install ros-humble-tf2-geometry-msgs`.

---

## 9. What's next

1. **Test `object_navigator.py`** — confirm the robot drives to a named target and reports arrival. Record this as the headline demo.
2. **Step 4 — full sentences.** Add a thin layer that extracts the target noun from a whole instruction ("go to the **person**" -> `person`). This is a small addition; the core pipeline is already done.
3. **(Optional) Search behaviour** — if the target isn't in view, rotate to look for it before navigating.
4. **(Optional) Visual arrival confirmation** — re-detect on arrival and check the object is large/centred in view, on top of Nav2's success report.
5. **Documentation & presentation** — the before/after demo pair is the centrepiece. Supervisor previously asked for more audiovisual elements, larger fonts, and speaker notes.

---

## 10. One standing recommendation

Because graduation rides on scope, keep the supervisor aligned on the framing: **OpenVLA is the evaluated baseline (and it fails by design — that's the finding); the working system is the modular YOLO-World + Nav2 pipeline.** Walk in with both the documented baseline and the working autonomous navigation — that's a complete, honest, defensible project.
