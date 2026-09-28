# Configuration

| File | Purpose |
|---|---|
| `swarm.yaml` | Every tunable of the system: swarm size, the origin of the shared frame, mission, formation, heartbeat, battery, safety, setpoint rates, autopilot stream rates, link emulator, logging and simulator settings. Loaded and validated by `src/swarm_agent/config.py`; unknown or missing keys are errors. |
| `mavros_pluginlists.yaml` | The lean MAVROS plugin list used for every drone: six plugins instead of about 25 (why: `reports/PHASE_1.md`). |
| `profiles/` | Drone profiles: `sim.yaml` (every drone simulated), `standin.yaml` (drone 1 behind the telemetry-radio stand-in, for the Phase 6 test) and `mixed.yaml` (drone 1 real, for the flight test plan; checked as text only, never started). Check one with `scripts/check_profile.py`. |
| `map_keys.example.yaml` | Template for the map service tokens of the ground-control app. |
| `map_keys.local.yaml` | Your own Mapbox and Cesium ion tokens. It is not in the repository: create it from the template. Git ignores it, and the Kaggle bundle and the documents never include it. |

A drone moves from simulation to real hardware by changing its connection URL in a profile; the agent code
stays the same. `swarm.yaml` also holds the telemetry-radio stand-in settings (`radio_standin`, SiK
defaults with their source) and the formation shape (`formation.shape`: V, line, column or echelon).
