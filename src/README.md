# Source code

Two Python packages. `swarm_agent` is the code that would run on every drone. `swarm_tools` is
everything around it for simulation and testing, and never runs onboard. Every tunable number is in
`config/swarm.yaml`. The design is described in [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md).

## swarm_agent: the onboard agent

| Module | Purpose |
|---|---|
| `agent_core.py` | The per-drone agent: mission phases, leader election, formation and safety in one class. |
| `election.py` | Leader election state machine with a term counter and planned handover. |
| `formation.py` | V formation: slot geometry, slot assignment and the follower velocity command. |
| `safety.py` | Separation (repulsion from nearby drones) and geofence. |
| `geometry.py` | WGS-84 latitude, longitude and altitude to one shared East-North-Up frame, and vector helpers. |
| `heartbeat.py` | The heartbeat message and its compact binary codec (45 bytes for 10 drones). |
| `obstacles.py` | Static obstacles (building footprints, trees): range sensing and clearance. |
| `planner.py` | The leader's route around obstacles: A* on an inflated grid with line-of-sight shortcuts. |
| `avoidance.py` | Local obstacle avoidance for the followers: a classical potential field or the learned policy. |
| `config.py` | Typed loader for `config/swarm.yaml`; unknown or missing keys are errors, not silent defaults. |
| `ros_node.py` | The ROS 2 node for one drone: wraps the agent and talks to that drone's own MAVROS. |

Every module except `ros_node.py` is plain Python with numpy, so the logic can be imported and tested
without ROS.

## swarm_tools: simulation and test tooling

| Module | Purpose |
|---|---|
| `sim_launch.py` | Start and stop N PX4 SIH instances and N MAVROS nodes (simulation only). |
| `link_emulator.py` | The simulated radio between drones: delay, jitter, loss, a cut transmitter, network splits. |
| `logger_node.py`, `logfmt.py` | Record every drone's state to `states.jsonl`, and its flat CSV form. |
| `mavros_link.py` | Test-harness handle for one drone's MAVROS namespace. |
| `resources.py` | CPU, memory and power-state sampling during simulation runs. |
| `puresim.py` | Point-mass swarm simulator that runs the real agent code for every drone (no PX4, no ROS). |
| `obstacle_sim.py` | Fast numpy simulator of formation flight through buildings and trees, for RL training and fair comparisons. |
| `torch_sim.py` | The same simulator batched in PyTorch: thousands of missions at once on a GPU. |
| `rl_vecenv.py` | Stable-Baselines3 vector environment over the obstacle simulator. |
| `osm.py` | Buildings, woods and trees from OpenStreetMap (Overpass API), cached in `data/osm/`. |
| `gcs/` | The ground-control web app: `server.py` (standard-library HTTP server on 127.0.0.1), `fastsim_backend.py` (runs the fast simulator) and `static/` (the Mission, 3D view, Results and Docs pages). |
