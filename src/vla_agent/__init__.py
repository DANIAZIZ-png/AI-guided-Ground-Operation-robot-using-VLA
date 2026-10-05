"""The VLA agent, as an importable package.

Phase 2 of the reorg splits the 4,730-line src/vla_agent_v28.py into modules.
This is a pure "move code" refactor: no logic changes. Parity is enforced by
tests/snapshot_agent_api.py against tests/baseline_agent_api.json, which was
captured from the monolith before any of this moved.

src/vla_agent_v28.py remains as a thin shim, so every existing caller
(`python3 src/vla_agent_v28.py`, vla_demo.sh, agent_xterm.sh, run_agent_sim.sh)
keeps working unchanged.
"""
