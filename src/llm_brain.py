"""Thin shim: the implementation now lives in vla_brain/brain.py.

Kept at this path because the launch scripts name it directly
(scripts/vla_demo.sh, scripts/vla_sim.sh, tools/*_xterm.sh). Phase 2 of
the reorg moved the code into a package without moving the entry point.
"""
from vla_brain.brain import *          # noqa: F401,F403

if __name__ == "__main__":
    from vla_brain.brain import main
    main()
