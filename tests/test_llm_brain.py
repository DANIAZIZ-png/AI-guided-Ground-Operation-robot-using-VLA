"""Unit tests for the reasoning layer: JSON parsing, schema validation, guards.

NEW FILE, written during the reorg.

No ROS, no GPU, no robot, and no Ollama -- the one test that exercises the HTTP
path mocks it. These are the tests that can run in CI.

WHY THIS IS THE RIGHT PLACE TO TEST
    The project's safety argument is "the LLM proposes, deterministic Python
    disposes". Everything in this file is the disposing half: the parser that
    must never crash on a bad reply, the schema validator that drops a
    hallucinated action before it reaches the agent, and the guards that
    override the model when it contradicts the sentence. If these are right,
    a wrong answer from the model is a wasted cycle rather than a moving robot.
"""
from __future__ import annotations

import json
import os
import sys
from unittest import mock

import pytest

SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
if SRC not in sys.path:
    sys.path.insert(0, SRC)

brain = pytest.importorskip("vla_brain.brain", reason="needs requests")


# ---------------------------------------------------------------------------
# _safe_json: the agent must never crash on a bad reply
# ---------------------------------------------------------------------------

def test_plain_json_parses():
    got = brain._safe_json('{"steps": [{"action": "navigate", "target": "chair"}]}')
    assert got["steps"][0]["target"] == "chair"


def test_json_wrapped_in_a_markdown_fence_parses():
    """Small local models fence their output constantly."""
    got = brain._safe_json('```json\n{"steps": [{"action": "dock"}]}\n```')
    assert got["steps"][0]["action"] == "dock"


def test_json_with_prose_around_it_parses():
    raw = 'Sure! Here is the plan:\n{"steps": [{"action": "stop"}]}\nHope that helps.'
    assert brain._safe_json(raw)["steps"][0]["action"] == "stop"


@pytest.mark.parametrize("raw", ["", "   ", "not json at all", "{broken", "null", "[]"])
def test_unparseable_input_never_raises(raw):
    """The one hard requirement on the parser: never raise. A crash here takes
    the agent down mid-command.

    Note it does NOT promise a dict -- "null" and "[]" are valid JSON of the
    wrong shape and come back as None and []. Making them safe is _validate's
    job, checked below.
    """
    brain._safe_json(raw)   # must not raise


@pytest.mark.parametrize("raw", ["", "   ", "not json at all", "{broken", "null", "[]"])
def test_parse_then_validate_always_yields_usable_steps(raw):
    """The real contract is the pair: whatever the model returns, the agent gets
    a list of valid steps."""
    out = brain._validate(brain._safe_json(raw))
    assert isinstance(out, dict) and out.get("steps")
    assert all(s["action"] in brain.VALID_ACTIONS for s in out["steps"])


# ---------------------------------------------------------------------------
# _validate: a hallucinated step must not reach the agent
# ---------------------------------------------------------------------------

def test_a_valid_step_survives_validation():
    out = brain._validate({"steps": [{"action": "navigate", "target": "Chair"}]})
    step = out["steps"][0]
    assert step["action"] == "navigate"
    assert step["target"] == "chair", "targets are lowercased and stripped"


def test_an_invented_action_is_dropped():
    """The model occasionally invents verbs. An unknown action must never be
    handed to the agent's dispatcher."""
    out = brain._validate({"steps": [{"action": "self_destruct", "target": "chair"}]})
    assert all(s["action"] in brain.VALID_ACTIONS for s in out["steps"])
    assert not any(s["action"] == "self_destruct" for s in out["steps"])


def test_a_mix_of_valid_and_invalid_steps_keeps_only_the_valid_ones():
    out = brain._validate({"steps": [
        {"action": "navigate", "target": "chair"},
        {"action": "teleport", "target": "mars"},
        {"action": "describe"},
    ]})
    assert [s["action"] for s in out["steps"]] == ["navigate", "describe"]


def test_stop_is_deliberately_not_a_brain_action():
    """cancel/stop never reaches the brain: the agent intercepts it at intake so
    it can preempt a command already queued behind a slow LLM call. A "stop"
    that had to wait for Ollama would be useless, so _validate dropping it is
    correct, not a gap.
    """
    assert "stop" not in brain.VALID_ACTIONS
    assert "cancel" not in brain.VALID_ACTIONS


def test_a_bare_single_step_dict_is_accepted():
    """An older reply shape, still produced sometimes."""
    out = brain._validate({"action": "dock"})
    assert out["steps"][0]["action"] == "dock"


@pytest.mark.parametrize("junk", [None, [], "steps", 42, {"steps": "nope"},
                                  {"steps": [1, 2, 3]}])
def test_structurally_wrong_input_yields_a_clarify_step(junk):
    out = brain._validate(junk)
    assert isinstance(out, dict) and out.get("steps")
    assert all(isinstance(s, dict) and s.get("action") for s in out["steps"])


def test_a_garbage_distance_becomes_zero_not_an_exception():
    """distance_m reaches arithmetic in the agent. A string there would raise
    somewhere far from the cause."""
    out = brain._validate({"steps": [
        {"action": "move", "distance_m": "about two metres"}]})
    assert isinstance(out["steps"][0].get("distance_m", 0.0), float)


def test_an_unknown_qualifier_is_dropped():
    out = brain._validate({"steps": [
        {"action": "navigate", "target": "person", "qualifier": "smelliest"}]})
    assert out["steps"][0].get("qualifier") in (None, *brain.VALID_QUALIFIERS)


def test_speech_is_always_a_string():
    out = brain._validate({"steps": [{"action": "describe", "speech": None}]})
    assert out["steps"][0]["action"] == "describe"
    assert isinstance(out["steps"][0]["speech"], str)


# ---------------------------------------------------------------------------
# _num
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("raw, expected", [
    (1.5, 1.5), ("1.5", 1.5), (2, 2.0), ("2", 2.0),
    (None, 0.0), ("", 0.0), ("lots", 0.0), ([], 0.0),
])
def test_num_coerces_or_returns_zero(raw, expected):
    assert brain._num(raw) == pytest.approx(expected)


# ---------------------------------------------------------------------------
# the guards: deterministic code overriding the model
# ---------------------------------------------------------------------------

def test_a_go_to_sentence_is_forced_to_navigate():
    """#17-era bug: "go to the chair" sometimes came back as locate, and the
    robot reported the chair instead of driving to it."""
    decision = {"steps": [{"action": "locate", "target": "chair"}]}
    out = brain._enforce_intent("go to the chair", decision)
    assert out["steps"][0]["action"] == "navigate"


def test_a_find_sentence_is_not_turned_into_a_drive():
    """The opposite error is worse: "find a chair" must never move the robot."""
    decision = {"steps": [{"action": "navigate", "target": "chair"}]}
    out = brain._enforce_intent("find a chair", decision)
    assert out["steps"][0]["action"] != "navigate"


def test_turn_left_is_forced_negative_and_right_positive():
    """The model got the sign wrong often enough to need a guard: a sign error
    turns the robot the wrong way, which during a demo looks like it ignored
    the command."""
    left = brain._enforce_turn_sign(
        "turn left 30", {"steps": [{"action": "rotate", "rotation_deg": 30.0}]})
    right = brain._enforce_turn_sign(
        "turn right 30", {"steps": [{"action": "rotate", "rotation_deg": -30.0}]})
    lv = left["steps"][0]["rotation_deg"]
    rv = right["steps"][0]["rotation_deg"]
    assert lv * rv < 0, "left and right must end up with opposite signs"
    assert abs(lv) == pytest.approx(30.0) and abs(rv) == pytest.approx(30.0)


def test_forget_is_never_routed_to_navigate():
    """#24: "forget the chair" used to be misrouted into a drive -- the robot
    set off towards the thing it had been told to forget."""
    out = brain._enforce_forget(
        "forget the chair", {"steps": [{"action": "navigate", "target": "chair"}]})
    assert out["steps"][0]["action"] != "navigate"


# ---------------------------------------------------------------------------
# decide(), with Ollama mocked
# ---------------------------------------------------------------------------

def _fake_ollama(payload: dict):
    """Build a requests-like response carrying an Ollama chat reply."""
    resp = mock.Mock()
    resp.status_code = 200
    resp.raise_for_status = mock.Mock()
    resp.json.return_value = {"message": {"content": json.dumps(payload)}}
    return resp


def test_decide_returns_a_validated_plan():
    with mock.patch.object(brain.requests, "post",
                           return_value=_fake_ollama(
                               {"steps": [{"action": "navigate", "target": "CHAIR"}]})):
        out = brain.decide("go to the chair", ["chair"])
    assert out["steps"][0]["action"] == "navigate"
    assert out["steps"][0]["target"] == "chair"


def test_decide_drops_an_invented_action_from_the_model():
    with mock.patch.object(brain.requests, "post",
                           return_value=_fake_ollama(
                               {"steps": [{"action": "launch_missiles"}]})):
        out = brain.decide("do something odd", [])
    assert all(s["action"] in brain.VALID_ACTIONS for s in out["steps"])


def test_decide_raises_when_ollama_is_unreachable_after_its_retry():
    """By design decide() does NOT invent a fallback plan when the model is
    unreachable -- it retries once and then raises. Guessing a plan with no
    model behind it is exactly what should not happen to a robot.

    The agent's call site catches it (state_machine.py: `except Exception` ->
    "Brain error (is Ollama running?)"), which is checked separately below.
    """
    with mock.patch.object(brain.requests, "post",
                           side_effect=brain.requests.ConnectionError("refused")):
        with pytest.raises(RuntimeError, match="unreachable"):
            brain.decide("go to the chair", ["chair"])


def test_the_agent_call_site_catches_a_brain_failure():
    """Pairs with the test above: decide() raising is only safe because the
    caller handles it. If this guard is ever removed, Ollama being down stops
    the agent dead instead of printing one line and carrying on.
    """
    sm = os.path.join(SRC, "vla_agent", "state_machine.py")
    text = open(sm).read()
    # anchor on the real call, not the word "decide(" in a docstring
    i = text.index("= decide(")
    window = text[i:i + 400]
    assert "except Exception" in window, (
        "the decide() call in state_machine.py is no longer wrapped in a broad "
        "except; an unreachable Ollama would now propagate"
    )


def test_decide_survives_a_non_json_reply():
    resp = mock.Mock()
    resp.status_code = 200
    resp.raise_for_status = mock.Mock()
    resp.json.return_value = {"message": {"content": "I'm afraid I can't do that"}}
    with mock.patch.object(brain.requests, "post", return_value=resp):
        out = brain.decide("go to the chair", ["chair"])
    assert isinstance(out, dict) and out.get("steps")


# ---------------------------------------------------------------------------
# configuration
# ---------------------------------------------------------------------------

def test_the_ollama_endpoint_and_model_are_the_recorded_ones():
    """env/MANIFEST.md pins qwen2.5:7b at digest 845dbda0ea48. A different
    model, or a different endpoint, would quietly change every answer."""
    assert "11434" in brain.OLLAMA_URL
    src = open(os.path.join(SRC, "vla_brain", "brain.py")).read()
    assert "qwen2.5" in src


def test_valid_actions_and_qualifiers_are_non_empty_sets_of_strings():
    for name in ("VALID_ACTIONS", "VALID_QUALIFIERS"):
        values = getattr(brain, name)
        assert values, f"{name} is empty"
        assert all(isinstance(v, str) for v in values)
