"""The reasoning layer: prompt Ollama, parse the plan, enforce the deterministic guardrails.

Phase 2 of the reorg made this an importable package. The code moved from
src/llm_brain.py unchanged; src/llm_brain.py is kept as a thin shim because the launch
scripts refer to it by path.
"""
from .brain import *        # noqa: F401,F403
