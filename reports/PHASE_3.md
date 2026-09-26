# Phase 3 — ROS 2 integration: formation flight, no faults
Status: PASSED — all flight criteria met in every run at the maximum stable N (10 PX4 drones).
Caveat: in 2 of the 6 recorded runs the test harness (not the drones) timed out; see below.

## Acceptance criteria
- [PASS] All drones reach the goal and land. In all 6 runs every drone landed in its formation
  slot around the goal: the leader 1.1–1.7 m from the goal point, the last drone of each arm
  about 49 m behind it (its slot). Evidence: `reports/logs/phase_3/<run>/metrics.json`
  (`final.*.ok`, `dist_to_target_m`).
- [PASS] Formation RMS error < 2 m during cruise: worst value per run 1.24–1.66 m
  (`formation_rms_cruise_m.max`), mean 0.54–0.62 m. Evidence: `metrics.json`, `formation_rms.csv`.
- [PASS] Minimum separation ≥ 5 m at all times: 7.80–8.65 m (`min_separation_m`).
- [PASS] Plot of trajectories saved to `reports/`: `reports/phase3_mission.png` (tracks with
  V snapshots, followers against their ideal slots, formation RMS over time).

Phase 3 tasks:
- [DONE] `swarm_agent` node wrapping the Phase 2 modules: `src/swarm_agent/ros_node.py`
  (one per drone, namespace `/uavN`, its own MAVROS).
- [DONE] `link_emulator` node with loss 0: `src/swarm_tools/link_emulator.py`.
- [DONE] Launch file: `launch/swarm.launch.py` (driven by `scripts/run_mission.py`).
- [DONE, with a note] Logger writing CSV: `src/swarm_tools/logger_node.py` writes JSON lines
  (`states.jsonl`, one sample per drone at 10 Hz); `swarm_tools.logfmt.jsonl_to_csv` turns them
  into CSV with fixed columns (`states.csv.gz` in every run folder).
- [DONE] Metrics script: `scripts/metrics.py` (formation RMS, minimum separation, time to goal).

## Key numbers
| Run | Harness | Drones | RMS max / mean / p95 (m) | Min separation (m) | Time to goal (s) | Leader to goal (m) | Leaders at once |
|---|---|---|---|---|---|---|---|
| run1 | completed | 10 | 1.32 / 0.56 / 1.08 | 8.11 | 221.4 | 1.2 | 1 |
| run2 | timeout (flight complete) | 10 | 1.39 / 0.62 / 1.13 | 8.65 | 220.4 | 1.3 | 1 |
| run3 | timeout (flight complete) | 10 | 1.66 / 0.56 / 1.02 | 7.80 | 220.3 | 1.7 | 1 |
| crosscheck1 | completed | 10 | 1.56 / 0.56 / 1.03 | 8.10 | 222.1 | 1.4 | 1 |
| crosscheck2 | completed | 10 | 1.24 / 0.54 / 1.00 | 8.57 | 220.6 | 1.4 | 1 |
| crosscheck3 | completed | 10 | 1.36 / 0.60 / 1.08 | 8.02 | 226.5 | 1.1 | 1 |

Mission: 1 km north of home, V formation, 10 m spacing, 30 m, 5 m/s, hover 10 s at the goal, land.
The cross-check runs were started from a clean `env -i` shell after killing every PX4 and MAVROS
process (`scripts/phase3_runs.sh`; console output in `reports/logs/phase_3/batch.out`).

## Sources verified (URLs / commands)
- PX4 Offboard mode needs a continuous setpoint stream of at least 2 Hz and exits Offboard after
  `COM_OF_LOSS_T` without it (https://docs.px4.io/main/en/flight_modes/offboard.html); the agent
  streams velocity setpoints at 20 Hz (`config/swarm.yaml` `setpoints.rate_hz`).
- MAVROS topics and services used by the node (state, extended_state, global_position/global,
  local_position/pose and velocity_local, battery, setpoint_velocity/cmd_vel, set_mode, arming,
  set_message_interval) were checked on a running system in Phase 1:
  `reports/logs/phase_1/lean_mavros_interfaces.txt`.
- MAVLink message IDs for the stream-rate requests were checked against pymavlink
  (`src/swarm_agent/ros_node.py`, `MAVLINK_MSG_IDS`; `tests/test_ros_constants.py`).

## Problems and fixes
- Formation RMS spiked at the start of cruise: the leader now ramps its speed (0.5 m/s²), so
  followers that see its velocity only at the heartbeat rate keep up.
- CPU saturation with 10 drones: MAVROS stream rates are set explicitly (position 20 Hz, others
  1–2 Hz) and the link emulator's delivery timer runs only when latency or jitter is set.
- The leader wandered off the line to the goal: leg guidance (fixed heading + cross-track correction).
- A steady follower error: position gain raised to 1.0.
- Drone 3 sometimes became leader at startup: the first election now waits until the whole fleet
  is heard (or a timeout).
- run2 and run3 ended with the harness's watcher thread stalled (`InvalidHandle` in the readiness
  check). The flights themselves completed (every drone landed, logs complete); the other four runs
  (run1 and the three cross-checks) completed normally. The readiness check has since been given its
  own executor and a stall is now reported as `WATCHER_STALLED`; that change has not been re-run on PX4.

## Known limitations / honest caveats
- PX4 SIH has a simple aircraft model and no wind; real aircraft will show larger errors.
- 10 drones is this laptop's limit for PX4 (Phase 1).
- No faults here by design; failover on PX4 is Phase 4 (not run yet).

## Next phase: what is needed from the user
Phase 4 (fault trials on PX4, 10 trials × 5 faults) takes about 5 hours of laptop time;
the scripts are ready (`scripts/faults.py`, `scripts/phase4_runs.sh`).
