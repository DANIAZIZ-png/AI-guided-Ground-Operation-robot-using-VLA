"""The VLA agent, as an importable package.

Phase 2 of the reorg split the 4,730-line src/vla_agent_v28.py into modules.
A pure "move code" refactor: no logic changed. Parity is enforced by
tests/snapshot_agent_api.py against tests/baseline_agent_api.json, captured from
the monolith before any of it moved.

    core.py               constants, the headless-safe print, module helpers
    ros_io.py             subscription callbacks, publishers, replies, status
    perception_client.py  camera pairing, the YOLO client, ranging, the feed
    navigation.py         occupancy grid, goal choice, Nav2 and Dock clients
    guards.py             front clearance, collision braking, stuck detection
    state_machine.py      command intake, dispatch, the per-task think steps
    agent.py              VLAAgent: __init__ plus the five mixins
    main.py               entry point

src/vla_agent_v28.py remains as a thin shim so every existing caller keeps
working unchanged: scripts/vla_demo.sh, tools/agent_xterm.sh,
scripts/run_agent_sim.sh and `python3 src/vla_agent_v28.py`.
"""
from .core import *                 # noqa: F401,F403
from .core import print             # noqa: A001 - fix #63
from .core import _real_print, _off_front, _DecodedFilter   # noqa: F401
from .agent import VLAAgent         # noqa: F401
from .main import main              # noqa: F401
