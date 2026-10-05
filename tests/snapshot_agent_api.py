#!/usr/bin/env python3
"""Dump the public surface of a module as JSON, for refactor parity checking.

Phase 2 splits src/vla_agent_v28.py (4,730 lines) into modules. That is a pure
"move code" refactor: no logic changes. The failure modes of such a refactor are
narrow and mechanical --

  * a module-level constant lost, or silently given a different value
  * a method dropped when a class was moved
  * an import left behind, so a name resolves to None or raises
  * a circular import between the new modules

-- and every one of them shows up as a difference in the module's public
surface. So: snapshot that surface from the monolith BEFORE the split, then
after each step re-snapshot and diff. It runs in about a second and needs no
Gazebo, no GPU and no robot.

It is deliberately not a behavioural test. It cannot prove the state machine
still drives correctly; tests/smoke_sim.sh is for that. What it does prove is
that nothing was lost in the move, which is the thing a human reviewer cannot
check by eye across 4,730 lines.

Usage:
    python3 tests/snapshot_agent_api.py vla_agent_v28 > tests/baseline.json
    python3 tests/snapshot_agent_api.py vla_agent_v28 --compare tests/baseline.json
"""
from __future__ import annotations

import argparse
import importlib
import inspect
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "src")

# Values whose repr is stable and meaningful to compare. Anything else (module
# objects, classes, compiled regexes, numpy arrays) is recorded by type only,
# because its repr embeds a memory address and would make the diff noise.
SCALARS = (bool, int, float, str, bytes, type(None))


def _portable(s: str) -> str:
    """Replace machine-specific prefixes with placeholders.

    WHY THIS EXISTS
        Some module-level constants are absolute paths derived from VLA_ROOT --
        LOG_DIR and SAVE_MAP_PATH. Recording them verbatim makes the baseline
        valid only on the machine that produced it: on any clone at a different
        path the gate reports them as CHANGED and `make test` fails for a reason
        that has nothing to do with the code. Found by review on a second
        machine, which is exactly where it would bite.

        Applied on BOTH sides -- when writing a snapshot and when comparing --
        so an old baseline and a new snapshot normalise to the same text.

        Longest prefix first: VLA_ROOT is usually inside HOME, and replacing
        HOME first would leave "$HOME/repos/..." instead of "$VLA_ROOT".
    """
    for value, placeholder in (
        (os.environ.get("VLA_ROOT") or REPO, "$VLA_ROOT"),
        (os.path.expanduser("~"), "$HOME"),
    ):
        if value and value != "/" and value in s:
            s = s.replace(value, placeholder)
    return s


def _value(v):
    if isinstance(v, str):
        return {"type": "str", "value": _portable(v)}
    if isinstance(v, SCALARS):
        return {"type": type(v).__name__, "value": v if not isinstance(v, bytes) else v.hex()}
    if isinstance(v, (list, tuple, set, frozenset)):
        try:
            inner = sorted(repr(x) for x in v)
        except TypeError:
            inner = [repr(x) for x in v]
        return {"type": type(v).__name__, "len": len(v), "items": inner}
    if isinstance(v, dict):
        return {"type": "dict", "len": len(v), "keys": sorted(map(repr, v.keys()))}
    return {"type": type(v).__name__}


def snapshot(modname: str) -> dict:
    if SRC not in sys.path:
        sys.path.insert(0, SRC)
    mod = importlib.import_module(modname)

    consts, funcs, classes = {}, {}, {}
    for name, obj in vars(mod).items():
        if name.startswith("__"):
            continue
        if inspect.ismodule(obj):
            continue
        if inspect.isclass(obj):
            # only classes actually defined in the project, not imported ones
            origin = getattr(obj, "__module__", "") or ""
            if not (origin == modname or origin.startswith("vla_")):
                continue
            classes[name] = {
                "methods": sorted(
                    m for m, _ in inspect.getmembers(obj, inspect.isfunction)
                    if not m.startswith("__")
                ),
                "bases": [b.__name__ for b in obj.__bases__],
                # The MRO matters more than the immediate bases: the Phase 2
                # split moves methods into mixins, which legitimately changes
                # __bases__ while every original base must still be reachable.
                "mro": [c.__name__ for c in obj.__mro__],
            }
        elif inspect.isfunction(obj):
            origin = getattr(obj, "__module__", "") or ""
            if not (origin == modname or origin.startswith("vla_")):
                continue
            try:
                sig = str(inspect.signature(obj))
            except (ValueError, TypeError):
                sig = "(?)"
            funcs[name] = {"signature": sig}
        elif not callable(obj):
            consts[name] = _value(obj)

    return {
        "module": modname,
        "constants": dict(sorted(consts.items())),
        "functions": dict(sorted(funcs.items())),
        "classes": dict(sorted(classes.items())),
    }


def diff(base: dict, now: dict) -> list[str]:
    """Human-readable differences. Additions are allowed; losses and changes are not.

    Classes are compared method by method rather than whole-object, for two
    reasons. First, the Phase 2 split deliberately moves methods into mixins,
    which changes __bases__ while keeping behaviour identical -- so what must
    hold is that every original base is still somewhere in the MRO, not that
    __bases__ is untouched. Second, dumping two 180-name method lists on any
    mismatch buries the one name that actually changed.
    """
    problems = []

    for kind in ("constants", "functions"):
        b, n = base.get(kind, {}), now.get(kind, {})
        for name in sorted(set(b) - set(n)):
            problems.append(f"LOST {kind[:-1]}: {name}")
        for name in sorted(set(b) & set(n)):
            if b[name] != n[name]:
                problems.append(
                    f"CHANGED {kind[:-1]}: {name}\n    was: {b[name]}\n    now: {n[name]}"
                )

    b, n = base.get("classes", {}), now.get("classes", {})
    for name in sorted(set(b) - set(n)):
        problems.append(f"LOST class: {name}")
    for name in sorted(set(b) & set(n)):
        lost = sorted(set(b[name].get("methods", [])) - set(n[name].get("methods", [])))
        if lost:
            problems.append(f"LOST methods on {name}: {', '.join(lost)}")
        # every base the class used to have must still be reachable
        mro = set(n[name].get("mro", n[name].get("bases", [])))
        gone = [x for x in b[name].get("bases", []) if x not in mro]
        if gone:
            problems.append(f"{name} no longer inherits from: {', '.join(gone)}")
    return problems


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("module")
    ap.add_argument("--compare", metavar="BASELINE.json")
    a = ap.parse_args()

    now = snapshot(a.module)

    if not a.compare:
        print(json.dumps(now, indent=2, sort_keys=True))
        return 0

    with open(a.compare) as fh:
        base = json.load(fh)
    problems = diff(base, now)

    nb = sum(len(base.get(k, {})) for k in ("constants", "functions", "classes"))
    nn = sum(len(now.get(k, {})) for k in ("constants", "functions", "classes"))
    if problems:
        print(f"PARITY FAIL  ({len(problems)} problem(s); baseline {nb} names, now {nn})")
        for p in problems:
            print("  " + p)
        return 1
    print(f"PARITY OK  ({nb} baseline names all present and unchanged; now {nn})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
