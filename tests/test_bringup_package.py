"""Static checks on the vla_bringup ROS 2 package.

NEW FILE, written during the reorg.

vla_bringup is what makes `ros2 launch vla_bringup robot.launch.py` work. It
installs the repository's own launch files rather than the stock
turtlebot4_navigation ones, which is the difference between a working bring-up
and the Fast DDS storm. Nothing about that is checked by importing code, so it
is checked here.

Pure file inspection: no ROS, no colcon, no GPU. Runs in CI.
"""
from __future__ import annotations

import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
PKG = REPO / "vla_bringup"
CMAKE = PKG / "CMakeLists.txt"

#: Files vla_bringup must install, because its own launch files reference them.
MUST_INSTALL = [
    "launch/nav2_hw_composed.launch.py",
    "launch/slam_hw.launch.py",
    "config/slam_vla.yaml",
    "config/fastdds_hw_wifi_only.xml",
    "maps/warehouse_map.yaml",
]


def test_the_package_exists_with_its_manifest_and_build_file():
    assert (PKG / "package.xml").is_file()
    assert CMAKE.is_file()
    assert (PKG / "launch" / "robot.launch.py").is_file()
    assert (PKG / "launch" / "sim.launch.py").is_file()


def test_the_manifest_declares_an_ament_build_type():
    text = (PKG / "package.xml").read_text()
    assert "<name>vla_bringup</name>" in text
    assert re.search(r"<build_type>ament_(cmake|python)</build_type>", text)


@pytest.mark.parametrize("rel", MUST_INSTALL)
def test_each_required_file_exists_in_the_repository(rel):
    assert (REPO / rel).is_file(), f"{rel} is missing from the repository"


@pytest.mark.parametrize("rel", MUST_INSTALL)
def test_cmakelists_installs_each_required_file(rel):
    assert pathlib.Path(rel).name in CMAKE.read_text(), (
        f"vla_bringup does not install {rel}; a launch run from the installed "
        "share directory would fail to find it"
    )


def test_robot_launch_uses_the_storm_fixed_launch_files():
    """The whole point of the package."""
    text = (PKG / "launch" / "robot.launch.py").read_text()
    assert "nav2_hw_composed.launch.py" in text
    assert "slam_hw.launch.py" in text


def test_robot_launch_never_names_the_stock_nav2_or_slam():
    text = (PKG / "launch" / "robot.launch.py").read_text()
    for bad in (r"turtlebot4_navigation.{0,40}nav2\.launch\.py",
                r"turtlebot4_navigation.{0,40}slam\.launch\.py"):
        hit = re.search(bad, text, re.S)
        assert hit is None, (
            f"robot.launch.py names {hit.group(0)!r}. The stock files have no "
            "/parameter_events or /rosout remap and a 4 s bond timeout."
        )


def test_robot_launch_defaults_bond_timeout_above_the_stock_four_seconds():
    text = (PKG / "launch" / "robot.launch.py").read_text()
    found = [float(m) for m in re.findall(
        r"'bond_timeout',\s*default_value='([\d.]+)'", text)]
    assert found, "robot.launch.py does not declare a bond_timeout default"
    assert min(found) >= 20.0, f"bond_timeout default is {min(found)}"


def test_robot_launch_delays_nav2_until_after_slam():
    """Nav2's lifecycle manager times out if it starts bonding before SLAM is
    publishing map->odom."""
    text = (PKG / "launch" / "robot.launch.py").read_text()
    assert "TimerAction" in text and "slam_delay" in text


def test_cmakelists_resolves_the_repo_root_through_a_symlink():
    """The package is reached through a symlink from a colcon workspace. A
    lexical '..' from there lands in the workspace's src/, not the repository,
    and the install then picks up nothing. This must be done in two steps,
    because REALPATH normalises the '..' before resolving the symlink.
    """
    text = CMAKE.read_text()
    assert text.count("REALPATH") >= 2, (
        "the repo root must be resolved with REALPATH twice: the package "
        "directory first, then '..' from its real location"
    )
    assert "CMAKE_CURRENT_LIST_DIR}/.." not in text.replace("_pkg_dir}/..", ""), (
        "a single REALPATH on '<pkg>/..' resolves to the colcon workspace src/"
    )


def test_cmakelists_fails_loudly_if_the_layout_is_wrong():
    assert "FATAL_ERROR" in CMAKE.read_text(), (
        "without a configure-time check, a wrong repo root installs nothing and "
        "the first symptom is a launch failing to find a file"
    )


def test_sim_launch_keeps_rviz_off_by_default():
    """RViz segfaults in the ubuntu22-gpu container ('egl: failed to create dri2
    screen', exit -11) and takes the whole launch down with it."""
    text = (PKG / "launch" / "sim.launch.py").read_text()
    assert re.search(r"'rviz',\s*default_value='false'", text)


def test_sim_launch_does_not_set_the_ros_domain():
    """Domain 42 was tried and rejected: gz_ros2_control runs inside the Gazebo
    process and does not inherit ROS_DOMAIN_ID, so controller_manager came up on
    0 while the spawner looked on 42 and the robot could not move."""
    text = (PKG / "launch" / "sim.launch.py").read_text()
    assert "ROS_DOMAIN_ID'] =" not in text
    assert "setdefault('ROS_DOMAIN_ID'" not in text
