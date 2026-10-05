# tests/

| File | Needs | What it does |
|---|---|---|
| `snapshot_agent_api.py` | ROS (rclpy) | dumps or compares a module's public surface — the **refactor parity gate** |
| `baseline_agent_api.json` | — | the surface of `vla_agent_v28` captured from the monolith **before** the Phase 2 split: 147 constants, 9 functions, 4 classes |

## The parity gate

Phase 2 splits `src/vla_agent_v28.py` (4,730 lines, 112 methods in one class)
into modules. That is a pure *move code* refactor, and its failure modes are
mechanical: a constant lost or changed, a method dropped when a class moved, an
import left behind, a circular import. All of them show up as a change in the
module's public surface.

```bash
# after every split step, inside the ROS container:
source /opt/ros/humble/setup.bash
python3 tests/snapshot_agent_api.py vla_agent_v28 --compare tests/baseline_agent_api.json
```

Additions pass. **Losses and changes fail**, and the output names each one.

It runs in about a second, needs no Gazebo, no GPU and no robot. It is *not* a
behavioural test — it cannot prove the state machine still drives correctly.
`smoke_sim.sh` is for that.
