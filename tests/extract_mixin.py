#!/usr/bin/env python3
"""Move named methods out of VLAAgent into a mixin module. Phase 2 tooling.

A pure mechanical move: the method source is copied byte-for-byte, including
its preceding comment block, and the methods keep their original 4-space
indentation, which is already correct inside a `class XMixin:` body. Method
resolution is preserved because the mixin is inserted into VLAAgent's bases
ahead of rclpy's Node.

Doing this with a tool rather than by hand is the point: 112 methods across
3,900 lines is exactly where hand-editing drops a decorator or a trailing
comment, and the parity gate would then tell you something broke without
telling you what.

    python3 tests/extract_mixin.py guards GuardsMixin \
        front_clearance agent_owns_cmd driving_forward collision_guard \
        --underscore _off_front

Then re-run the gate:

    python3 tests/snapshot_agent_api.py vla_agent_v28 \
        --compare tests/baseline_agent_api.json
"""
from __future__ import annotations

import argparse
import ast
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
SHIM = REPO / "src" / "vla_agent_v28.py"
PKG = REPO / "src" / "vla_agent"


def chunk_ranges(lines: list[str], names: list[str]) -> list[tuple[int, int, str]]:
    """1-indexed inclusive line ranges for each named method, comments attached."""
    tree = ast.parse("".join(lines))
    cls = next(
        n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "VLAAgent"
    )
    meths = [
        (n.name, n.lineno, n.end_lineno, n.decorator_list)
        for n in cls.body
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
    ]
    ends = {}
    prev_end = cls.lineno
    for name, lo, hi, _ in meths:
        ends[name] = prev_end
        prev_end = hi

    out = []
    for name in names:
        match = next((m for m in meths if m[0] == name), None)
        if match is None:
            sys.exit(f"ERROR: VLAAgent has no method {name!r}")
        _, lo, hi, decorators = match
        if decorators:
            lo = min(d.lineno for d in decorators)
        # walk back over the comment/blank block that documents this method,
        # but never past the previous method's last line
        i = lo
        floor = ends[name] + 1
        while i - 1 >= floor:
            s = lines[i - 2].strip()
            if s.startswith("#") or s == "":
                i -= 1
            else:
                break
        out.append((i, hi, name))
    return sorted(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("module", help="module name under src/vla_agent/, e.g. guards")
    ap.add_argument("mixin", help="class name, e.g. GuardsMixin")
    ap.add_argument("methods", nargs="+")
    ap.add_argument("--underscore", nargs="*", default=[],
                    help="underscore-prefixed names from core that the moved code needs; "
                         "`import *` skips these, so they must be named explicitly")
    ap.add_argument("--doc", default="", help="one-line description for the module docstring")
    a = ap.parse_args()

    lines = SHIM.read_text().splitlines(keepends=True)
    ranges = chunk_ranges(lines, a.methods)

    for (lo1, hi1, n1), (lo2, _, n2) in zip(ranges, ranges[1:]):
        if hi1 >= lo2:
            sys.exit(f"ERROR: ranges overlap: {n1} ends {hi1}, {n2} starts {lo2}")

    moved = "".join("".join(lines[lo - 1:hi]) for lo, hi, _ in ranges)

    und = ""
    if a.underscore:
        und = (
            "# `import *` deliberately skips underscore-prefixed names, so the ones the\n"
            "# moved code needs are imported explicitly. Omitting them imports cleanly\n"
            "# and then fails at runtime.\n"
            f"from .core import {', '.join(a.underscore)}   # noqa: F401\n"
        )

    (PKG / f"{a.module}.py").write_text(
        f'"""{a.doc or a.mixin}\n\n'
        "Methods moved verbatim out of VLAAgent in Phase 2 of the reorg. No logic\n"
        "changed; the comments are the originals. Mixed into VLAAgent in\n"
        "vla_agent/agent.py, ahead of rclpy's Node, so method resolution is unchanged.\n"
        '"""\n'
        "from .core import *            # noqa: F401,F403 - constants and ROS imports\n"
        "from .core import print        # noqa: A001 - headless-safe console, fix #63\n"
        + und
        + "\n\n"
        f"class {a.mixin}:\n"
        + moved
    )

    # remove the moved lines from the shim, back to front
    for lo, hi, _ in reversed(ranges):
        del lines[lo - 1:hi]

    text = "".join(lines)

    # insert the mixin into VLAAgent's bases and import it
    old_bases = "class VLAAgent(Node):"
    new_bases = f"class VLAAgent({a.mixin}, Node):"
    if old_bases in text:
        text = text.replace(old_bases, new_bases, 1)
    else:
        import re
        m = re.search(r"class VLAAgent\(([^)]*)\):", text)
        if not m:
            sys.exit("ERROR: could not find the VLAAgent class statement")
        bases = m.group(1)
        text = text.replace(m.group(0), f"class VLAAgent({a.mixin}, {bases}):", 1)

    anchor = "from vla_agent.core import _real_print"
    if anchor not in text:
        sys.exit("ERROR: could not find the core import anchor in the shim")
    line_end = text.index("\n", text.index(anchor)) + 1
    text = (
        text[:line_end]
        + f"from vla_agent.{a.module} import {a.mixin}   # noqa: E402\n"
        + text[line_end:]
    )

    SHIM.write_text(text)
    print(f"moved {len(ranges)} methods into src/vla_agent/{a.module}.py ({a.mixin})")
    for lo, hi, n in ranges:
        print(f"    {n:<22} {hi - lo + 1:>4} lines")
    print(f"  shim now {len(text.splitlines())} lines")
    return 0


if __name__ == "__main__":
    sys.exit(main())
