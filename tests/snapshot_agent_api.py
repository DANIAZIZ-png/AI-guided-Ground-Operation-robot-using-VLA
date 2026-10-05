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


def _value(v):
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
    """Human-readable differences. Additions are allowed; losses and changes are not."""
    problems = []
    for kind in ("constants", "functions", "classes"):
        b, n = base.get(kind, {}), now.get(kind, {})
        for name in sorted(set(b) - set(n)):
            problems.append(f"LOST {kind[:-1]}: {name}")
        for name in sorted(set(b) & set(n)):
            if b[name] != n[name]:
                problems.append(f"CHANGED {kind[:-1]}: {name}\n    was: {b[name]}\n    now: {n[name]}")
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
