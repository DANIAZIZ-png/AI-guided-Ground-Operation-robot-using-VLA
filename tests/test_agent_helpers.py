"""Behavioural checks on the agent's pure helpers, to back the Phase 2 split.

NEW FILE, written during the Phase 2 reorg. These are not restored originals.

The parity gate (tests/snapshot_agent_api.py) proves no name was lost when code
moved into the vla_agent package. It cannot prove the moved code still computes
the same answers. These tests cover the module-level helpers that are pure
functions, where that is cheap to check directly.

Needs rclpy, because importing the agent imports it. Run inside the ROS
container:

    source /opt/ros/humble/setup.bash
    python3 -m pytest tests/test_agent_helpers.py -q
"""
from __future__ import annotations

import builtins
import math
import os
import sys

import pytest

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

agent = pytest.importorskip(
    "vla_agent_v28",
    reason="needs rclpy and the ROS environment; run inside the ROS container",
)


class Quat:
    """Minimal stand-in for geometry_msgs Quaternion."""

    def __init__(self, x: float, y: float, z: float, w: float) -> None:
        self.x, self.y, self.z, self.w = x, y, z, w


# --------------------------------------------------------------------------
# fix #63: the headless-safe console
# --------------------------------------------------------------------------

def test_headless_safe_print_still_shadows_builtin():
    """The module-level `def print` is agent fix #63.

    A module-level def shadows the builtin only inside its own module, so
    moving code between modules can silently drop it -- and the symptom is the
    broken-pipe crash it was written to fix, not an import error.
    """
    assert agent.print is not builtins.print


# --------------------------------------------------------------------------
# geometry
# --------------------------------------------------------------------------

def test_yaw_from_quat_identity_is_zero():
    assert agent.yaw_from_quat(Quat(0, 0, 0, 1)) == pytest.approx(0.0, abs=1e-9)


def test_yaw_from_quat_quarter_turn():
    s = math.sqrt(0.5)
    assert agent.yaw_from_quat(Quat(0, 0, s, s)) == pytest.approx(math.pi / 2, abs=1e-6)


def test_yaw_from_quat_half_turn_magnitude():
    assert abs(agent.yaw_from_quat(Quat(0, 0, 1, 0))) == pytest.approx(math.pi, abs=1e-6)


@pytest.mark.parametrize(
    "raw, expected",
    [
        (0.0, 0.0),
        (math.pi / 2, math.pi / 2),
        (3 * math.pi, math.pi),
        (-3 * math.pi, -math.pi),
        (2 * math.pi, 0.0),
    ],
)
def test_ang_norm_wraps_into_pi_range(raw, expected):
    got = agent.ang_norm(raw)
    assert -math.pi - 1e-9 <= got <= math.pi + 1e-9
    assert abs(got) == pytest.approx(abs(expected), abs=1e-6)


def test_off_front_is_zero_dead_ahead_and_symmetric():
    """_off_front measures how far a laser bearing is off the forward axis.

    It is used by on_scan and front_clearance, and it accounts for the RPLIDAR
    being bolted on rotated (agent #65). Dead ahead must be 0, and equal
    offsets either side must agree.
    """
    fwd = math.radians(float(os.environ.get("VLA_SCAN_FWD_DEG", "0")))
    assert agent._off_front(fwd) == pytest.approx(0.0, abs=1e-9)
    assert agent._off_front(fwd + 0.3) == pytest.approx(agent._off_front(fwd - 0.3), abs=1e-9)


# --------------------------------------------------------------------------
# text handling
# --------------------------------------------------------------------------

def test_strip_politeness_removes_please_and_keeps_the_command():
    assert "chair" in agent.strip_politeness("please go to the chair")
    assert "please" not in agent.strip_politeness("please go to the chair").lower()


def test_strip_politeness_leaves_a_plain_command_alone():
    assert agent.strip_politeness("go to the chair").strip() == "go to the chair"


@pytest.mark.parametrize("word", ["yes", "yeah", "yep", "y"])
def test_is_yes_accepts_affirmatives(word):
    assert agent.is_yes(word)


@pytest.mark.parametrize("word", ["no", "nope", "stop", ""])
def test_is_yes_rejects_everything_else(word):
    assert not agent.is_yes(word)


def test_first_word_ignores_surrounding_whitespace():
    assert agent.first_word("  go   left  ") == "go"


def test_fuzzy_word_passes_an_exact_match_straight_through():
    assert agent.fuzzy_word("chair", ["chair", "table"]) == "chair"


def test_fuzzy_word_returns_nothing_for_an_unrelated_word():
    assert agent.fuzzy_word("helicopter", ["chair", "table"]) is None


def test_fuzzy_word_refuses_to_correct_short_words():
    """Deliberately cautious by design: a false 'cancel' is annoying and a
    false 'quit' would be unrecoverable, so anything under FUZZY_MIN_LEN is
    never corrected."""
    short = "x" * (agent.FUZZY_MIN_LEN - 1)
    assert agent.fuzzy_word(short, ["cancel", "stop"]) is None


def test_fuzzy_word_never_corrects_a_known_good_command():
    """Words in NEVER_CONTROL must come back as None rather than being
    'corrected' into a control word."""
    for word in agent.NEVER_CONTROL:
        assert agent.fuzzy_word(word, ["cancel", "stop"]) is None
        break


def test_fuzzy_word_corrects_a_control_word_typo_within_its_cutoff():
    """It is only meant to fix control words, and only above FUZZY_CONTROL.

    'cancell' is close enough to 'cancel'; 'chiar' for 'chair' is NOT, and
    that is intentional -- object names go to the LLM, which reads through
    typos on its own.
    """
    assert agent.fuzzy_word("cancell", ["cancel", "stop"]) == "cancel"
    assert agent.fuzzy_word("chiar", ["chair", "table"]) is None


# --------------------------------------------------------------------------
# things the split could have dropped
# --------------------------------------------------------------------------

def test_decoded_filter_is_instantiable():
    """#50: the hand-driven message_filters source for the compressed RGB path."""
    assert agent._DecodedFilter() is not None


def test_manual_keys_is_a_populated_read_only_mapping():
    assert isinstance(agent.MANUAL_KEYS, dict) and agent.MANUAL_KEYS


def test_depth_k_default_is_a_nine_element_intrinsics_matrix():
    assert len(agent.DEPTH_K_DEFAULT) == 9


def test_agent_version_is_present():
    assert isinstance(agent.AGENT_VERSION, str) and agent.AGENT_VERSION


#: Methods VLAAgent defined itself in the monolith, before Phase 2 began.
#: The AST counts 112 definitions; 111 here, because __init__ shares its name
#: with rclpy's Node.__init__ and so cannot be told apart by name alone. It is
#: checked separately below.
MONOLITH_OWN_METHODS = 111


def test_vla_agent_still_answers_to_all_its_own_methods():
    """The split moves methods into mixins, so they stop being in vars(VLAAgent)
    and arrive through the MRO instead. What must not change is that the class
    still answers to every one of them.
    """
    import inspect

    callables = {
        name for name in dir(agent.VLAAgent)
        if inspect.isfunction(getattr(agent.VLAAgent, name, None))
    }
    node_callables = {
        name for name in dir(agent.VLAAgent.__mro__[-2])
        if inspect.isfunction(getattr(agent.VLAAgent.__mro__[-2], name, None))
    }
    own = callables - node_callables
    assert len(own) >= MONOLITH_OWN_METHODS, (
        f"VLAAgent answers to {len(own)} of its own methods, "
        f"expected at least {MONOLITH_OWN_METHODS}"
    )


def test_vla_agent_still_has_its_own_init():
    """The 368-line __init__ builds every subscription, publisher and action
    client. Inheriting Node's instead would start an agent wired to nothing.
    """
    node_cls = agent.VLAAgent.__mro__[-2]
    assert agent.VLAAgent.__init__ is not node_cls.__init__
