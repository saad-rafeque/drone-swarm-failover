# Tests

```bash
python3 -m pytest                                  # 131 passed, 4 skipped without ROS 2 and PyTorch
PYTHONPATH=src .venv/bin/python -m pytest          # with PyTorch (RL environment): 134 passed, 2 skipped
python3 -m pytest --cov                            # with coverage of src/swarm_agent and src/swarm_tools
```

The tests need neither PX4 nor a browser. Tests that need PyTorch, Stable-Baselines3 or ROS 2 are
skipped when those are not installed. The Phase 2 coverage record (`election.py` 99 %, `formation.py`
100 %) is in `reports/logs/phase_2/pytest_coverage.txt`.

| File | What it checks |
|---|---|
| `test_election.py` | The leader election state machine against the rules in `docs/SPECIFICATION.md` and the documented design choices. |
| `test_formation.py` | V formation: slot geometry, parity-arm assignment, reassignment and the control law. |
| `test_safety.py` | Separation (repulsion) and the geofence. |
| `test_geometry.py` | Conversion from GPS to the shared East-North-Up frame, and the vector helpers. |
| `test_heartbeat.py` | The heartbeat binary codec. |
| `test_config.py` | Loading and validation of `config/swarm.yaml`. |
| `test_agent_core.py` | Mission-level behaviour of the agent in the point-mass simulator. |
| `test_agent_obstacles.py` | The agent among obstacles: a route around a wall, avoidance, and going home around it. |
| `test_obstacles.py` | Obstacle map queries and the leader's route planner. |
| `test_avoidance.py` | The avoiders (none, potential field, learned policy) and the obstacle simulator's agreement with the agent code. |
| `test_torch_sim.py` | The batched PyTorch simulator steps exactly like the numpy reference. |
| `test_long_route.py` | Long routes: positions far from the origin, planning in chunks, charging stops. |
| `test_gcs_backend.py` | The ground-control app: commands, faults, snapshots, takeover timing, and which files the Docs page may serve. |
| `test_tools_scripts.py` | Safety checks on helper scripts: the Kaggle bundle never packs a key, and validation and test courses stay apart. |
| `test_ros_constants.py` | Constants the ROS node relies on, checked against their sources (needs ROS 2). |

`conftest.py` holds the shared helpers: configuration loading and a small heartbeat bus for election
tests.
