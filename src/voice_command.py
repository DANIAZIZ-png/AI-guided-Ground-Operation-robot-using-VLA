"""Thin shim: the implementation now lives in vla_voice/listener.py.

Kept at this path because the launch scripts name it directly
(scripts/vla_demo.sh, scripts/vla_sim.sh, tools/*_xterm.sh). Phase 2 of
the reorg moved the code into a package without moving the entry point.
"""
from vla_voice.listener import *          # noqa: F401,F403

if __name__ == "__main__":
    from vla_voice.listener import main
    main()
