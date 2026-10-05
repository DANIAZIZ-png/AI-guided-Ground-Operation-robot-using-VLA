"""Tests for the refactor parity gate itself.

NEW FILE, written during the Phase 2 reorg.

The gate is what licenses every "no behaviour changed" claim in the Phase 2
commits, so it needs its own tests. A gate that cannot fail would print
PARITY OK forever and prove nothing.

Pure Python: no ROS, no GPU, no robot. Runs in CI.
"""
from __future__ import annotations

import os
import sys

import pytest

TESTS = os.path.dirname(os.path.abspath(__file__))
if TESTS not in sys.path:
    sys.path.insert(0, TESTS)

from snapshot_agent_api import diff  # noqa: E402


def _base():
    return {
        "constants": {"STOP_DISTANCE": {"type": "float", "value": 0.6}},
        "functions": {"ang_norm": {"signature": "(a)"}},
        "classes": {
            "VLAAgent": {
                "methods": ["front_clearance", "collision_guard"],
                "bases": ["Node"],
                "mro": ["VLAAgent", "Node", "object"],
            }
        },
    }


# --------------------------------------------------------------------------
# it must pass when nothing changed
# --------------------------------------------------------------------------

def test_identical_snapshots_produce_no_problems():
    assert diff(_base(), _base()) == []


# --------------------------------------------------------------------------
# it must fail on every loss the split could cause
# --------------------------------------------------------------------------

def test_detects_a_lost_constant():
    now = _base()
    del now["constants"]["STOP_DISTANCE"]
    assert any("LOST constant: STOP_DISTANCE" in p for p in diff(_base(), now))


def test_detects_a_changed_constant_value():
    """The nastiest possible refactor bug: the name survives, the value does not.

    VLA_STOP_DISTANCE going from 0.60 to 0.45 would make Nav2 refuse every goal
    (it is inside the 0.45 m inflation radius) and nothing would say why.
    """
    now = _base()
    now["constants"]["STOP_DISTANCE"] = {"type": "float", "value": 0.45}
    problems = diff(_base(), now)
    assert any("CHANGED constant: STOP_DISTANCE" in p for p in problems)
    assert any("0.45" in p for p in problems)


def test_detects_a_lost_function():
    now = _base()
    del now["functions"]["ang_norm"]
    assert any("LOST function: ang_norm" in p for p in diff(_base(), now))


def test_detects_a_changed_function_signature():
    now = _base()
    now["functions"]["ang_norm"] = {"signature": "(a, b)"}
    assert any("CHANGED function: ang_norm" in p for p in diff(_base(), now))


def test_detects_a_lost_method():
    """A method dropped while moving a class into a mixin."""
    now = _base()
    now["classes"]["VLAAgent"]["methods"] = ["front_clearance"]
    problems = diff(_base(), now)
    assert any("LOST methods on VLAAgent" in p and "collision_guard" in p for p in problems)


def test_detects_a_lost_class():
    now = _base()
    del now["classes"]["VLAAgent"]
    assert any("LOST class: VLAAgent" in p for p in diff(_base(), now))


def test_detects_a_broken_inheritance_chain():
    """If VLAAgent stopped being an rclpy Node it would import fine and then
    fail the moment it tried to create a subscription."""
    now = _base()
    now["classes"]["VLAAgent"]["bases"] = ["GuardsMixin"]
    now["classes"]["VLAAgent"]["mro"] = ["VLAAgent", "GuardsMixin", "object"]
    assert any("no longer inherits from: Node" in p for p in diff(_base(), now))


# --------------------------------------------------------------------------
# it must NOT fail on the changes the split is supposed to make
# --------------------------------------------------------------------------

def test_mixin_added_to_bases_is_allowed_while_node_stays_in_the_mro():
    """This is exactly what extracting a mixin does, and it must pass."""
    now = _base()
    now["classes"]["VLAAgent"]["bases"] = ["GuardsMixin", "Node"]
    now["classes"]["VLAAgent"]["mro"] = ["VLAAgent", "GuardsMixin", "Node", "object"]
    assert diff(_base(), now) == []


def test_additions_are_allowed():
    now = _base()
    now["constants"]["NEW_THING"] = {"type": "int", "value": 1}
    now["functions"]["new_helper"] = {"signature": "()"}
    now["classes"]["GuardsMixin"] = {"methods": ["front_clearance"], "bases": ["object"],
                                     "mro": ["GuardsMixin", "object"]}
    now["classes"]["VLAAgent"]["methods"].append("brand_new_method")
    assert diff(_base(), now) == []


def test_a_baseline_without_an_mro_key_still_compares():
    """baseline_agent_api.json was captured before the tool recorded the MRO.
    It must keep working rather than needing a re-capture, since re-capturing
    after the split would defeat the entire point of a baseline.
    """
    base = _base()
    del base["classes"]["VLAAgent"]["mro"]
    now = _base()
    now["classes"]["VLAAgent"]["bases"] = ["GuardsMixin", "Node"]
    now["classes"]["VLAAgent"]["mro"] = ["VLAAgent", "GuardsMixin", "Node", "object"]
    assert diff(base, now) == []


def test_the_real_baseline_file_is_present_and_sane():
    import json
    path = os.path.join(TESTS, "baseline_agent_api.json")
    assert os.path.isfile(path), "the parity baseline is missing"
    with open(path) as fh:
        base = json.load(fh)
    assert len(base["constants"]) == 147
    assert len(base["functions"]) == 9
    assert set(base["classes"]) == {"VLAAgent", "ReplyTee", "MissionLog", "_DecodedFilter"}
    assert "Node" in base["classes"]["VLAAgent"]["bases"]


# --------------------------------------------------------------------------
# the baseline must be portable
# --------------------------------------------------------------------------
# Found by review on a second machine. Two constants (LOG_DIR, SAVE_MAP_PATH)
# are absolute paths derived from VLA_ROOT, and the baseline recorded them
# verbatim -- so on any clone at a different path the gate reported them as
# CHANGED and `make test` failed for a reason unrelated to the code. Verified:
# the pre-fix baseline does fail from a different path, naming exactly those two.

def test_the_baseline_contains_no_machine_specific_paths():
    import json
    path = os.path.join(TESTS, "baseline_agent_api.json")
    raw = open(path).read()
    assert "/home/" not in raw, (
        "the parity baseline records an absolute home path. It must store "
        "$VLA_ROOT / $HOME placeholders, or the gate fails on every clone at a "
        "different path."
    )
    b = json.load(open(path))
    assert b["constants"]["LOG_DIR"]["value"].startswith("$VLA_ROOT")
    assert b["constants"]["SAVE_MAP_PATH"]["value"].startswith("$VLA_ROOT")


def test_portable_replaces_the_repo_root_with_a_placeholder():
    from snapshot_agent_api import REPO, _portable
    assert _portable(os.path.join(REPO, "logs")) == "$VLA_ROOT/logs"


def test_portable_prefers_the_longer_prefix():
    """VLA_ROOT normally sits inside HOME. Replacing HOME first would leave
    "$HOME/repos/..." and the comparison would still be path-dependent."""
    from snapshot_agent_api import REPO, _portable
    got = _portable(os.path.join(REPO, "maps", "warehouse_map"))
    assert got == "$VLA_ROOT/maps/warehouse_map"
    assert "$HOME" not in got


def test_portable_leaves_an_unrelated_absolute_path_alone():
    """Paths outside the repo and home -- /opt/ros/humble, /dev/snd -- are the
    same on every machine and must not be rewritten."""
    from snapshot_agent_api import _portable
    assert _portable("/opt/ros/humble/setup.bash") == "/opt/ros/humble/setup.bash"


def test_portable_is_idempotent():
    """It runs on both sides of the comparison, so applying it to an already
    normalised value must not change it again."""
    from snapshot_agent_api import _portable
    once = _portable("$VLA_ROOT/logs")
    assert once == "$VLA_ROOT/logs"
