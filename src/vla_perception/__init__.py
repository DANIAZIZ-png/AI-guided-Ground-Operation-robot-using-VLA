"""The open-vocabulary detector, served over HTTP to the agent.

Phase 2 of the reorg made this an importable package. The code moved from
src/yolo_server.py unchanged; src/yolo_server.py is kept as a thin shim because the launch
scripts refer to it by path.
"""
from .server import *        # noqa: F401,F403
