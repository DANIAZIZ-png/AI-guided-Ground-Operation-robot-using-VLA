"""The shipped GUI configs must not be able to recreate the Fast DDS storm.

NEW FILE, written during the reorg.

WHY THIS EXISTS
    config/vla_gui.robot.json shipped with its SLAM and Nav2 buttons pointing at
    the stock `turtlebot4_navigation` launch files. Those have no
    /parameter_events or /rosout remap and a 4 s bond timeout. Pressing either
    button floods the robot at ~7,400 packets/s (2 MB/s): the Pi throttles at
    84 C, camera frames stop, and Nav2 aborts its own bring-up. That is the
    8 and 10 September failure, diagnosed on 11 September, with packet captures
    in results/evidence/.

    It survived because nothing checked it. The bring-up script had been fixed
    to use the repository's own launch files while the GUI buttons were left
    behind, and a config file is not covered by any test that imports code.

    These tests are cheap, need no ROS, no GPU and no robot, and would have
    caught it.

Pure JSON inspection. Runs in CI.
"""
from __future__ import annotations

import json
import os
import pathlib
import re

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
CONFIGS = sorted((REPO / "config").glob("vla_gui.*.json"))

#: Launch files that must never appear in a shipped hardware config. The stock
#: Nav2 is the storm; the stock SLAM carries the same missing remaps.
FORBIDDEN_ON_HARDWARE = [
    r"turtlebot4_navigation\s+nav2\.launch\.py",
    r"turtlebot4_navigation\s+slam\.launch\.py",
]

#: The repository's own replacements, which carry the remaps.
REQUIRED_NAV2 = "nav2_hw_composed.launch.py"
REQUIRED_SLAM = "slam_hw.launch.py"


def _load(path: pathlib.Path) -> dict:
    return json.loads(path.read_text())


def _steps(path: pathlib.Path):
    return _load(path).get("steps", [])


def test_at_least_one_gui_config_is_present():
    assert CONFIGS, "no config/vla_gui.*.json found"


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.name)
def test_config_is_valid_json_with_steps(path):
    data = _load(path)
    assert isinstance(data.get("steps"), list) and data["steps"]
    for step in data["steps"]:
        for key in ("name", "where", "cmd", "enabled"):
            assert key in step, f"step {step.get('name')!r} is missing {key!r}"


def test_the_hardware_config_never_names_the_stock_nav2_or_slam():
    """The regression this file exists for."""
    path = REPO / "config" / "vla_gui.robot.json"
    text = path.read_text()
    for pattern in FORBIDDEN_ON_HARDWARE:
        hit = re.search(pattern, text)
        assert hit is None, (
            f"{path.name} names {hit.group(0)!r}. The stock launch files have no "
            "/parameter_events or /rosout remap and a 4 s bond timeout, which "
            "floods the robot's Wi-Fi at ~7,400 packets/s and aborts the "
            f"bring-up. Use {REQUIRED_NAV2} / {REQUIRED_SLAM} instead."
        )


def test_the_hardware_nav2_button_uses_the_composed_launch_file():
    steps = _steps(REPO / "config" / "vla_gui.robot.json")
    nav2 = [s for s in steps if "nav2" in s["name"].lower()]
    assert nav2, "the hardware config has no Nav2 step"
    for step in nav2:
        assert REQUIRED_NAV2 in step["cmd"], (
            f"Nav2 step runs {step['cmd']!r}; it must use {REQUIRED_NAV2}, which "
            "carries the remaps, bond_timeout 30 s and the delayed STARTUP."
        )


def test_the_hardware_slam_button_uses_the_remapped_launch_file():
    steps = _steps(REPO / "config" / "vla_gui.robot.json")
    slam = [s for s in steps if "slam" in s["name"].lower()]
    assert slam, "the hardware config has no SLAM step"
    for step in slam:
        assert REQUIRED_SLAM in step["cmd"], (
            f"SLAM step runs {step['cmd']!r}; it must use {REQUIRED_SLAM}."
        )


def test_every_hardware_step_sources_robot_env():
    """Each step has to stand alone: the GUI hands its own environment to every
    child, and after a machine rebuild there is no shell rc file setting the
    discovery variables. Sourcing config/robot.env is also what selects
    fastdds_hw_wifi_only.xml (super client PLUS interface whitelist) over the
    older fastdds_super_client.xml.
    """
    for step in _steps(REPO / "config" / "vla_gui.robot.json"):
        if step["where"] != "here":
            continue
        assert "config/robot.env" in step["cmd"], (
            f"step {step['name']!r} does not source config/robot.env"
        )


def test_no_config_hard_codes_a_home_directory_path():
    """Phase 1 removed these; they must not come back through a config file."""
    for path in CONFIGS:
        text = path.read_text()
        assert "/home/danyalaziz" not in text, f"{path.name} hard-codes a host path"


def test_the_launch_files_the_hardware_config_names_actually_exist():
    """A config naming a file that is not in the repository is worse than a
    wrong one: it fails only when the button is pressed, on demo day."""
    expected = {
        REQUIRED_NAV2: REPO / "launch" / REQUIRED_NAV2,
        REQUIRED_SLAM: REPO / "launch" / REQUIRED_SLAM,
    }
    text = (REPO / "config" / "vla_gui.robot.json").read_text()
    for name, path in expected.items():
        if name in text:
            assert path.is_file(), f"{name} is referenced but {path} does not exist"


def test_the_repo_launch_files_carry_both_remaps():
    """The remaps are the actual fix. A launch file that lost them would pass
    every test above while still storming the robot."""
    for name in (REQUIRED_NAV2, REQUIRED_SLAM):
        text = (REPO / "launch" / name).read_text()
        assert "/pc/parameter_events" in text, f"{name} lost the /parameter_events remap"
        assert "/pc/rosout" in text, f"{name} lost the /rosout remap"


def test_the_composed_nav2_keeps_a_bond_timeout_well_above_four_seconds():
    """The stock 4 s timeout is what made the lifecycle manager shoot Nav2 dead
    mid-bring-up. 30 s is what makes 7/7 bonds activate reliably."""
    text = (REPO / "launch" / REQUIRED_NAV2).read_text()
    found = [float(m) for m in re.findall(r"bond_timeout['\"]?\s*,?\s*default_value=['\"]([\d.]+)", text)]
    found += [float(m) for m in re.findall(r"'bond_timeout':\s*([\d.]+)", text)]
    assert found, "could not find a bond_timeout default in " + REQUIRED_NAV2
    assert min(found) >= 20.0, f"bond_timeout default is {min(found)}, too close to the stock 4 s"
