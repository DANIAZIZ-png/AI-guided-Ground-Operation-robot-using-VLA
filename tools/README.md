# tools/

Diagnostic probes and terminal launchers. These live in `~/vla_tools/` on the
host. Each was written to answer one question during debugging, which is why
there are so many.

## Launchers — start a stack component in its own xterm

The agent and the voice listener need a **real tty**: manual override is typed
into the agent's own window, and a pipe silently drops it back to headless mode.
That is why these open terminals instead of redirecting output.

| Script | Starts |
|---|---|
| `agent_xterm.sh` | the agent, with all the hardware env flags set |
| `voice_xterm.sh` | transcription — also sets `LD_LIBRARY_PATH` to the pip cuBLAS/cuDNN libraries, without which every transcription fails with `libcublas.so.12 not found` |
| `nav2_hwc_xterm.sh` / `nav2_hw_xterm.sh` | Nav2, composed / one process per server |
| `slam_xterm.sh` | SLAM Toolbox |
| `rviz_hw_xterm.sh` | RViz with the `/robot1` namespace and TF remaps |
| `restart_agent_hw.sh` | restart **only** the agent, leaving the rest of the stack up |
| `shutdown_stack.sh` | orderly shutdown |

## Probes — ground truth, not guesses

| Script | Answers |
|---|---|
| `say.py "<command>" [sec]` | sends one command on `/vla/command` exactly as the GUI would, and prints the agent's replies. A long-lived client with a clean shutdown, so it leaves no ghost. |
| `latency_probe.py` | how old camera frames actually are |
| `drift_probe.py` | whether frame age is growing over time |
| `range_probe.py` | maps a YOLO box to a laser distance |
| `scan_look.py` | clearance by bearing |
| `is_moving.py` | is the base actually moving |
| `daemoncheck.sh` | what environment the ROS daemon was started with |
| `hotspot_guard.sh` | watches for stray clients on the hotspot |
| `yolo_live_test.py`, `yolo_floor_probe.py` | detector checks against a live frame |
| `nav_goal.py X Y YAW` | sends one Nav2 goal |
| `undock_hw.py` | undock |
| `feed_http.py raw 8081` | serves the camera feed to a browser |

## Camera

| Script | Does |
|---|---|
| `oakd_pipeline.py RGB` | cycles the camera. **`RGB` is the mode in use** — do not switch to `RGBD` |
| `oakd_bandwidth.py` | sets fps / JPEG quality / width |
| `oakd_params.py` | reads current camera parameters |
| `oakd_rgbd.py` | the RGBD-only predecessor, no longer run |

The camera runs **colour-only**. Depth cost the Pi +7 °C, halved colour to 15 Hz
and, under load, delayed frames by seconds. Ranging is done with the LiDAR.

## Simulation helpers

`view_cam.py`, `view_depth.py`, `save_map.py`, `spawn_objects.py`,
`reset_robot.py` — run by hand, mostly against the Gazebo simulation.

## Two traps worth knowing before you run any of these

1. **`ping` does not work inside the ROS container.** It exits 2 with no output
   for every address, identically whether the robot is up or unplugged, because
   containers are not granted the raw socket ICMP needs. Test reachability with
   bash's `/dev/tcp` instead, which also tests a real TCP connection:
   ```bash
   timeout 5 bash -c "cat < /dev/null > /dev/tcp/10.42.0.169/22"   # exit 0 = reachable
   ```
2. **Short-lived ROS clients leave ghosts.** A probe killed by `timeout`, or one
   that exits without `rclpy.shutdown()`, stays in every node's view until the
   ROS daemon is restarted. Keep CLI probing on hardware to a minimum, and
   prefer `say.py`, which shuts down cleanly.
