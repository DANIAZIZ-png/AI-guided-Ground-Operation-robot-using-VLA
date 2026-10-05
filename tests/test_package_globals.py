"""Every constant a vla_agent module references must resolve in that module.

NEW FILE, written during the Phase 2 reorg, after this exact bug bit.

WHY THIS EXISTS
    The parity gate inspects the public surface of `vla_agent_v28`. That is the
    right check for "did the split lose anything", but it has a blind spot: a
    constant can still be visible on the shim while being *unresolvable inside
    the module that uses it*.

    That is what happened. Step 1 moved MissionLog into core.py but left
    AGENT_VERSION in the shim, so `core.MissionLog.log()` referenced a name
    that was not in core's globals. The parity gate said OK -- AGENT_VERSION
    was still on the shim -- and py_compile said OK, because a NameError is a
    runtime error, not a syntax error. Nothing would have complained until the
    agent wrote its first mission log line, on the robot.

    pyflakes cannot catch it either: these modules use `from .core import *`,
    and a star import makes pyflakes give up on undefined-name analysis.

    So this test does it dynamically. It imports each module and checks that
    every ALL_CAPS name the module reads is resolvable there. ALL_CAPS because
    that is the project's convention for module-level constants, which is the
    class of name a module split actually drops.

Needs rclpy, because importing the package imports it.
"""
from __future__ import annotations

import ast
import builtins
import importlib
import os
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
SRC = REPO / "src"
PKG = SRC / "vla_agent"

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

pytest.importorskip(
    "rclpy", reason="needs the ROS environment; run inside the ROS container"
)

MODULES = sorted(p.stem for p in PKG.glob("*.py") if p.stem != "__init__")


def _referenced_constants(path: pathlib.Path) -> set[str]:
    """ALL_CAPS names the module READS (Load context), excluding ones it defines."""
    tree = ast.parse(path.read_text())
    read, assigned = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            if not (node.id.isupper() and len(node.id) > 2):
                continue
            if isinstance(node.ctx, ast.Load):
                read.add(node.id)
            else:
                assigned.add(node.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                assigned.add(alias.asname or alias.name.split(".")[0])
    return read - assigned


@pytest.mark.parametrize("modname", MODULES)
def test_every_constant_a_module_reads_resolves_in_that_module(modname):
    mod = importlib.import_module(f"vla_agent.{modname}")
    missing = sorted(
        name
        for name in _referenced_constants(PKG / f"{modname}.py")
        if not hasattr(mod, name) and not hasattr(builtins, name)
    )
    assert not missing, (
        f"vla_agent.{modname} reads {missing} but they do not resolve there. "
        "A star import does not carry underscore-prefixed names, and a constant "
        "left behind in another module raises NameError only at runtime."
    )


def test_agent_version_resolves_everywhere_it_is_used():
    """The specific regression that prompted this file.

    AGENT_VERSION is printed at startup, written into every mission log and
    published in /vla/status, so it is what ties a recorded run to the code
    that produced it.
    """
    for modname in ("core", "ros_io", "state_machine"):
        mod = importlib.import_module(f"vla_agent.{modname}")
        assert getattr(mod, "AGENT_VERSION", None), f"missing in vla_agent.{modname}"


def test_the_headless_safe_print_is_present_in_every_module_that_prints():
    """Fix #63. A module-level `def print` shadows the builtin only inside its
    own module, so a module that prints and did not import core's print has
    silently reverted to the builtin -- and the broken-pipe crash is back.
    """
    offenders = []
    for modname in MODULES:
        path = PKG / f"{modname}.py"
        tree = ast.parse(path.read_text())
        prints = any(
            isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "print"
            for n in ast.walk(tree)
        )
        if not prints:
            continue
        mod = importlib.import_module(f"vla_agent.{modname}")
        if getattr(mod, "print", builtins.print) is builtins.print:
            offenders.append(modname)
    assert not offenders, (
        f"these modules call print() but use the builtin, losing fix #63: {offenders}"
    )
