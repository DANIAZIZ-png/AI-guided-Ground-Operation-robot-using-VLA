"""Where everything lives. The Python counterpart of config/paths.sh.

Every path in the project derives from VLA_ROOT. If the environment already
provides VLA_ROOT or any of the derived variables (config/paths.sh exports them,
and so do config/sim.env and config/robot.env) they win; otherwise the root is
resolved from this file's location, which is one level below the repository
root. That resolution does not depend on the caller's working directory, which
is what the old hard-coded host paths were really standing in for.

Import is safe from a plain script: Python puts the script's own directory on
sys.path, so `import vla_paths` works when running `python3 src/<script>.py`
from anywhere.
"""
from __future__ import annotations

import os

__all__ = [
    "VLA_ROOT", "CONFIG_DIR", "SRC_DIR", "TOOLS_DIR", "LAUNCH_DIR",
    "MAP_DIR", "LOG_DIR", "MODEL_DIR", "model_path", "ensure_log_dir",
]


def _root() -> str:
    env = os.environ.get("VLA_ROOT")
    if env:
        return os.path.abspath(env)
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _sub(var: str, *parts: str) -> str:
    env = os.environ.get(var)
    if env:
        return os.path.abspath(env)
    return os.path.join(VLA_ROOT, *parts)


VLA_ROOT = _root()

CONFIG_DIR = _sub("VLA_CONFIG_DIR", "config")
SRC_DIR    = _sub("VLA_SRC_DIR", "src")
TOOLS_DIR  = _sub("VLA_TOOLS_DIR", "tools")
LAUNCH_DIR = _sub("VLA_LAUNCH_DIR", "launch")
MAP_DIR    = _sub("VLA_MAP_DIR", "maps")

# The two overridable output locations. Defaults are inside the repo and are
# git-ignored.
LOG_DIR   = _sub("VLA_LOG_DIR", "logs")
MODEL_DIR = _sub("VLA_MODEL_DIR", "models")


def ensure_log_dir() -> str:
    """Create LOG_DIR if needed and return it."""
    os.makedirs(LOG_DIR, exist_ok=True)
    return LOG_DIR


def model_path(filename: str) -> str:
    """Resolve a weights file.

    Returns the path inside MODEL_DIR when the file is actually there, and
    otherwise the bare filename. The fallback matters: ultralytics resolves a
    bare name against the working directory and will download the weights if it
    finds nothing, which is exactly what happened before this function existed.
    Keeping that path means a checkout with no models/ directory behaves as it
    always did.
    """
    candidate = os.path.join(MODEL_DIR, filename)
    return candidate if os.path.isfile(candidate) else filename
