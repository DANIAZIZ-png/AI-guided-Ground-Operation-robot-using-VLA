# vla_bringup

A ROS 2 (`ament_cmake`) package that ships **no code** — only the files needed
to start the stack through the normal ROS 2 mechanism:

```bash
ros2 launch vla_bringup robot.launch.py     # hardware: SLAM + Nav2, storm fixes applied
ros2 launch vla_bringup sim.launch.py       # simulation: Gazebo + SLAM + Nav2
```

## The files are installed, not moved

`CMakeLists.txt` installs the repository's `launch/`, `config/` and `maps/` into
this package's share directory. They stay where they are in the repository, so
`$VLA_LAUNCH_DIR`, `$VLA_CONFIG_DIR` and `$VLA_MAP_DIR` keep working and the
operator console's launch buttons — which refer to them by path — are unaffected.

`ament_cmake` rather than `ament_python` for the same reason: CMake installs
files from outside its own directory with plain paths, where setuptools
`data_files` does not handle `..` reliably.

## What `robot.launch.py` guarantees

It starts `slam_hw.launch.py` and `nav2_hw_composed.launch.py`, both of which
remap `/parameter_events` → `/pc/parameter_events` and `/rosout` →
`/pc/rosout`, and the composed Nav2 keeps `bond_timeout` at 30 s with a 20 s
delayed STARTUP.

Those remaps are not cosmetic. Without them Fast DDS floods the Raspberry Pi at
~7,400 packets/s (2 MB/s): the Pi throttles at 84 °C, camera frames stop, and
Nav2's lifecycle manager kills its own bring-up. The stock
`turtlebot4_navigation` launch files have neither remap and a 4 s bond timeout,
which is why this package exists rather than using them directly. Evidence:
`results/evidence/`, `docs/changelogs/CHANGELOG_2026-09-11.md` §2–3.

## Environment

`robot.launch.py` does **not** set the DDS environment — a launch file cannot
usefully export variables to its own process. Source it first:

```bash
source config/robot.env
ros2 launch vla_bringup robot.launch.py
```

Simulation and hardware need **opposite** settings and each fails *silently*
under the other's. `config/sim.env` is the simulation half.

## Building

```bash
mkdir -p ~/ros2_ws/src && ln -s "$PWD/vla_bringup" ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select vla_bringup
source install/setup.bash
```

`make bringup` does this for you.
