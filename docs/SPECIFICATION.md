# Swarm Failover — Project specification

The specification this project was built against: goals, hard rules, phases and acceptance
criteria. Section and rule numbers quoted in the code and the reports (for example §6 or rule 3a)
refer to this document.

## 1. Goal
Build, on this laptop only, a simulated swarm of PX4 quadrotors that flies in V formation
from home to a goal waypoint. One drone is MASTER. If the master fails (killed, link lost,
or low battery), the lowest-ID alive drone becomes master within seconds and the swarm
continues to the goal.

The architecture must later run unchanged against real Pixhawk 6C drones: switching a drone
from simulated to real must only require changing its connection URL in config.

**In scope:** waypoint navigation, formation, leader election / failover, inter-drone
separation, geofence, fault injection, metrics.
**Out of scope:** payloads, targeting, or any logic that engages objects.

## 2. Environment (VERIFY in Phase 0 — do not assume)
- Laptop: Lenovo IdeaPad 5, Intel i3 11th gen, 8 GB RAM, no GPU
- OS: Ubuntu 24.04, ROS 2 (expected: Jazzy), Python 3.12
- Simulator: PX4 SIH running as SITL (headless, no Gazebo). Build target: `px4_sitl_sih`
- Drone link: MAVROS, one instance per drone (NOT uXRCE-DDS), because real drones will use
  MAVLink over low-bandwidth telemetry radios.
- User is on mobile most of the time and is notified via ntfy (see §4).

## 3. Hard rules (apply to every phase)
1. **Verify before use.** Every PX4 parameter, make target, MAVROS topic/service, ROS 2
   command, and package name must be confirmed from official docs (docs.px4.io,
   docs.ros.org, github.com/mavlink/mavros) OR from the installed system
   (`ros2 topic list`, `ros2 interface show`, `param show`, reading source). Record the
   source (URL or command) in the phase report. If you cannot verify something, say so —
   never guess.
2. **Evidence or it didn't happen.** Never report success without command output, a test
   result, or a metrics file. Put raw logs in `reports/logs/phase_N/`.
3. **Cross-check at phase end.** Before reporting a phase done:
   a. Kill all sim processes (`pkill -f px4; pkill -f mavros`), open a fresh shell,
      re-run the phase's acceptance tests from scratch.
   b. Re-read this file's acceptance criteria line by line and mark each PASS/FAIL with
      evidence.
   c. Confirm every number in the report matches the raw logs.
4. **Phase gates.** After each phase: write `reports/PHASE_N.md`, commit, tag `phase-N`,
   send the notification, then STOP and wait for the user to type `continue`.
   Do not start the next phase on your own.
5. **Stuck rule.** If the same error survives 3 fix attempts, stop, notify
   (priority high), and write what you tried and what you suspect in the report.
6. **Resource guard.** Build with `-j2` max. Check `free -m` before launching sims; if
   available memory < 800 MB, reduce drone count and report it. Kill orphan px4/mavros
   processes between runs.
7. **Safety.** Simulation only. Never open serial devices (`/dev/tty*`), never connect to
   a real flight controller, never flash firmware.
8. **Ask first** before: any `sudo`, apt/pip installs, deleting files outside this repo,
   or changing an architecture decision in this file (e.g., replacing MAVROS).
9. **Code standards.** Python 3.12, rclpy, type hints, all tunables in
   `config/swarm.yaml` (no hardcoded ports/IDs), pure-Python logic modules
   (election, formation, geometry) importable and testable WITHOUT ROS.

## 4. Notifications (user's phone via ntfy)
Send with: `scripts/notify.sh "<title>" "<message>" [priority]`
- Title must be plain ASCII (it is an HTTP header). Emojis only in the message.
- Phase done: `scripts/notify.sh "Phase N PASSED" "<1-line key result>. Waiting for continue."`
- Phase failed / stuck: same format, title `Phase N FAILED`, priority `high`.
- Long run (> 20 min) started: one message with the ETA.
- In Phase 0, send a test message and ask the user to confirm it arrived.

## 5. Mission defaults (`config/swarm.yaml`)
| Setting | Value |
|---|---|
| Drones (target) | 10 (start with 3, scale in Phase 1) |
| Formation | V, 10 m spacing between neighbors |
| Cruise altitude / speed | 30 m AGL / 5 m/s |
| Goal | single point ~1 km from home, loaded into every drone before takeoff |
| Heartbeat | 5 Hz, each drone broadcasts: id, role, term, position, velocity, battery, timestamp |
| Master dead after | 1.5 s without its heartbeat |
| Election rule | lowest alive ID wins; `term` counter resolves conflicts (see §6) |
| Low-battery handover | master hands over voluntarily at 30 %, then leaves formation and returns home |
| Min separation (hard) | 5 m between any two drones |
| Geofence | 1.5 km radius from home, 50 m max altitude |
| Setpoint stream | velocity setpoints at 20 Hz (PX4 drops offboard if the stream falls below 2 Hz — verify in docs) |

## 6. Architecture
- **One `swarm_agent` node per drone** (namespace `/uavN`) talking to its own MAVROS
  instance. There is NO central controller node — each agent decides its own role, so the
  same code can later run onboard each drone.
- Central tools allowed only for: launching, fault injection, link emulation, logging.
- **Heartbeat transport goes through a `link_emulator` node**: agents publish to
  `/uavN/hb_out`, the emulator forwards to every other `/uavM/hb_in` with configurable
  loss/latency per link. This is how "link lost" faults and radio-like conditions are simulated.
- **Election:** each agent keeps `term`. A drone that becomes master increments `term`.
  On receiving a master heartbeat with a higher `term` (or equal `term` and lower ID),
  a master steps down. After a partition heals there must be exactly one master.
- **Formation:** follower slot = rank among alive drones sorted by ID. Slot offset is
  rotated into the master's heading. Follower command = master velocity (feedforward)
  + P-control on position error, saturated; add a repulsion term when any neighbor
  is closer than 1.5× min separation.
- **Frames — critical:** every drone's MAVROS local frame has its own origin. NEVER
  compare drones' local positions directly. Convert each drone's global position
  (lat/lon/alt) to one shared ENU frame anchored at `config.origin`. Unit-test this.
- **No master heard and no election result yet:** hold position (hover).
- **At goal:** swarm hovers 10 s, then all drones land.

## 7. Phases

### Phase 0 — Environment audit and setup
Tasks:
- Record OS, `$ROS_DISTRO`, RAM, CPU cores, free disk (need ≥ 15 GB), Python version.
- Test ntfy (`scripts/notify.sh "Phase 0" "Test message"`) and ask user to confirm.
- Install (ask first): PX4-Autopilot toolchain for Ubuntu 24.04 per docs.px4.io,
  MAVROS for the installed ROS 2 distro, and the GeographicLib datasets MAVROS requires.
- Build `px4_sitl_sih` for quadrotor (`-j2`).
- Create repo skeleton: `config/`, `src/swarm_agent/`, `tests/`, `scripts/`, `reports/`.
Acceptance:
- One SIH drone + one MAVROS instance: script arms, takes off to 10 m, hovers 20 s, lands.
  Evidence: log with altitude over time.
- Peak RAM for 1 drone (PX4 + MAVROS) measured and recorded.

### Phase 1 — Multi-drone simulation scaling
Tasks:
- Launch script for N SIH instances + N MAVROS instances with unique ports/sysids
  (verify the multi-instance method in PX4 docs).
- Verify spawn positions. If instances spawn at the same point, give each a distinct home
  position using the documented method, and cite it.
- Scale 3 → 5 → 10. At each step measure RAM and CPU.
Acceptance:
- All N drones arm, take off to 10 m, hover 30 s, and land, with no crashes.
- Report: max stable N on this laptop with RAM/CPU numbers. If N < 10, state it plainly
  and propose options (e.g., lighter MAVLink bridge) — do not switch without approval.

### Phase 2 — Agent logic (pure Python, no ROS) + unit tests
Tasks:
- Modules: `geometry` (global → shared ENU), `election` (state machine + term),
  `formation` (V slots, rotation, rank reassignment), `safety` (separation, geofence).
- Pure-Python swarm simulator (point-mass, no PX4) to exercise election + formation fast.
Acceptance:
- `pytest` all green; coverage ≥ 90 % on election and formation modules.
- 1,000 randomized pure-Python runs (random kills, random link drops, partitions):
  exactly one master after convergence in 100 % of runs; median convergence time reported.

### Phase 3 — ROS 2 integration: formation flight, no faults
Tasks:
- `swarm_agent` node wrapping the Phase 2 modules; `link_emulator` node (loss = 0);
  launch file; logger node writing CSV (positions, roles, terms, timestamps).
- Metrics script: formation RMS error, min pairwise separation, time to goal.
Acceptance (3 runs at max stable N):
- All drones reach goal and land.
- Formation RMS error < 2 m during cruise.
- Min separation ≥ 5 m at all times.
- Plot of trajectories saved to `reports/`.

### Phase 4 — Failover under faults
Faults (fault injector node/script):
- F1 master killed (kill its PX4 + MAVROS processes)
- F2 master link lost (emulator drops 100 % of master heartbeats)
- F3 master low battery (use PX4's simulated battery drain if verifiable in docs;
  otherwise inject battery value at the agent level and say so in the report)
- F4 follower killed (no election expected; slots reassign)
- F5 partition then heal (two groups, then reconnect)
Acceptance (10 trials per fault, random fault time during cruise):
- F1/F2: new master heartbeat within 3.0 s median (4.0 s worst case) of failure.
- F3: planned handover within 1.0 s of trigger; old master leaves and returns home.
- F4: no master change; formation recovers.
- F5: single master within 3.0 s after heal.
- All: formation RMS back under 2 m within 15 s (F5: 20 s, set by the owner on 28 September 2026); min separation never < 5 m;
  goal reached in ≥ 9/10 trials per fault.
- Report table: per fault — median/worst handover time, recovery time, min separation,
  success rate.

### Phase 5 — Radio realism (telemetry-like links)
Tasks:
- Sweep link emulator: latency {50, 150, 300} ms × loss {0, 10, 30} %.
- Measure: false failovers (master change without a real fault), handover time,
  formation error.
Acceptance:
- Results table + plot. Identify the worst condition where Phase 4 criteria still hold.
- Zero false failovers at ≤ 10 % loss; if not, report and propose timeout/heartbeat changes.

### Phase 6 — Mixed-reality readiness (still software only)
Tasks:
- `config/profiles/`: `sim.yaml` and `mixed.yaml` (example: drone 1 real via serial URL
  placeholder, drones 2–N SIH). Do NOT open the serial port — validate config only.
- Hardware stand-in test: run one SIH drone through a UDP proxy that limits bandwidth and
  adds latency/loss like a telemetry radio; the swarm must still pass Phase 4 F1 and F2.
- Write `FLIGHT_TEST_PLAN.md`: 1 real + N virtual drones, geofence, RC override on every
  real drone, kill-switch procedure, go/no-go checklist, abort criteria.
- Write `README.md`: setup, how to run each phase, results summary, known limitations.
Acceptance:
- Stand-in test passes F1 and F2 criteria.
- Everything reproducible from a fresh clone with the README.
- FINAL STOP. Notify: "Software phase complete. Real-drone work needs user review."

## 8. Phase report template (`reports/PHASE_N.md`)
```
# Phase N — <name>
Status: PASSED / FAILED
## Acceptance criteria
- [PASS/FAIL] <criterion> — evidence: <file/command/number>
## Key numbers
## Sources verified (URLs / commands)
## Problems and fixes
## Known limitations / honest caveats
## Next phase: what is needed from the user (if anything)
```
