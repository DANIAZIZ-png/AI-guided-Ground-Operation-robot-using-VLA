"""The operator console: conversation, annotated camera feed, push-to-talk.

Phase 2 of the reorg made this an importable package. The code moved from
src/vla_gui_v2.py unchanged; src/vla_gui_v2.py is kept as a thin shim because the launch
scripts refer to it by path.
"""
from .app import *        # noqa: F401,F403
