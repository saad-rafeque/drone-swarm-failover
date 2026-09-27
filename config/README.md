# Configuration

| File | Purpose |
|---|---|
| `swarm.yaml` | Every tunable of the system: swarm size, the origin of the shared frame, mission, formation, heartbeat, battery, safety, setpoint rates, autopilot stream rates, link emulator, logging and simulator settings. Loaded and validated by `src/swarm_agent/config.py`; unknown or missing keys are errors. |
| `mavros_pluginlists.yaml` | The lean MAVROS plugin list used for every drone: six plugins instead of about 25 (why: `reports/PHASE_1.md`). |
| `map_keys.example.yaml` | Template for the map service tokens of the ground-control app. |
| `map_keys.local.yaml` | Your own Mapbox and Cesium ion tokens. It is not in the repository: create it from the template. Git ignores it, and the Kaggle bundle and the documents never include it. |

A drone moves from simulation to real hardware by changing its connection URL; the agent code stays the
same. The mixed simulated and real profiles (`profiles/sim.yaml`, `profiles/mixed.yaml`) belong to
Phase 6, which has not started.
