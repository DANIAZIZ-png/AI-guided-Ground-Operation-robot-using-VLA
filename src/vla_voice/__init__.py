"""Push-to-talk speech, transcribed locally with faster-whisper.

Phase 2 of the reorg made this an importable package. The code moved from
src/voice_command.py unchanged; src/voice_command.py is kept as a thin shim because the launch
scripts refer to it by path.
"""
from .listener import *        # noqa: F401,F403
