# Phase 2 — Agent logic (pure Python, no ROS) + unit tests
Status: PASSED

## Acceptance criteria
- [PASS] `pytest` all green: **91 passed** (`reports/logs/phase_2/pytest_coverage.txt`; also green
  in the clean-shell cross-check).
- [PASS] Coverage ≥ 90 % on the election and formation modules: `election.py` **99 %**
  (167 statements, lines 118–119 not hit), `formation.py` **100 %** (57 statements).
  All logic modules are ≥ 98 %; 99 % in total (`pytest --cov`, same file).
- [PASS] 1,000 randomized pure-Python runs (random kills, random link drops, partitions):
  exactly one master after convergence in **1000 / 1000 (100 %)** runs. **Median convergence
  0.145 s** after the last fault cleared (p95 2.12 s, max 2.72 s).
  Evidence: `reports/logs/phase_2/random_trials_summary.json` and `random_trials.jsonl` (one line
  per run: seed, N, latency, loss, applied faults, convergence, masters). The clean-shell
  cross-check (`crosscheck/`) reproduced the per-run file byte for byte.

## What was built (all importable and testable without ROS)

| Module | Role |
|---|---|
| `src/swarm_agent/geometry.py` | WGS-84 ↔ ECEF ↔ one shared ENU frame at `config.origin`; tuple vector helpers |
| `src/swarm_agent/heartbeat.py` | Heartbeat dataclass + 50-byte binary codec (radio-sized) |
| `src/swarm_agent/election.py` | Election state machine: term, timeouts, claims, step-down, planned handover |
| `src/swarm_agent/formation.py` | V slots, slot assignment, rotation into master heading, follower control law |
| `src/swarm_agent/safety.py` | Pairwise separation, repulsion, geofence |
| `src/swarm_agent/agent_core.py` | One drone's full logic: mission phases + election + formation + safety; the Phase 3 ROS node wraps it unchanged |
| `src/swarm_agent/config.py` | Typed, strict loader for `config/swarm.yaml` with cross-field validation |
| `src/swarm_tools/puresim.py` | Point-mass simulator running the real `AgentCore` per drone; lossy/delayed/blockable heartbeat network |
| `scripts/random_trials.py` | The 1,000-run acceptance experiment |
| `scripts/puresim_faults.py` | F1–F5 of Phase 4 in the pure simulator (design check before PX4) |

## Key numbers
Randomized runs (`random_trials_summary.json`):
- **Setup per run:** N uniform in 3..10 (121–132 runs each), link latency {0, 50, 150, 300} ms
  (+20 % jitter), random loss 0–30 %, and 1–5 random faults. Faults applied: kill_master 454,
  kill_random 470, master_link_lost 499, link_drops 494, partition 468, low_battery_master 397.
- **Wall time:** 100.4 s on 3 worker processes.
- **Median convergence by the run's last fault:** kill_master 1.73 s (a true failover),
  partition 0.29 s, link_drops 0.20 s, low_battery_master 0.18 s, kill_random 0.03 s,
  master_link_lost 0.03 s.
- **Split brain during faults:** 801 runs had more than one master at some point. That is
  expected: both sides of a partition elect a leader. Every run still ended with exactly one.

F1–F5 in the pure simulator, N = 10, 20 seeds each, ideal links (`puresim_faults_n10.txt/.jsonl`).
This checks the design before PX4, not Phase 4 evidence:

| Fault | Handover (median / worst) | Formation RMS < 2 m after (median / worst) | Min separation | Goal reached |
|---|---|---|---|---|
| F1 master killed | 1.64 / 1.75 s | 4.55 / 4.68 s | 9.35 m | 20/20 |
| F2 master link lost | 1.64 / 1.75 s | 4.55 / 4.68 s | 8.23 m | 20/20 |
| F3 master low battery | 0.17 / 0.20 s (planned) | 3.14 / 3.23 s | 9.35 m | 20/20; retiree lands ≤ 2.3 m from home |
| F4 follower killed | no master change | 1.12 / 1.15 s | 7.46 m | 20/20 |
| F5 partition, then heal | 0.18 / 0.25 s after heal | 9.93 / 13.75 s | 6.23 m | 20/20 |

## Design decisions that complete the spec (all tunables in `config/swarm.yaml`)
1. **Heartbeat fields beyond §5.** In addition to id, role, term, position, velocity, battery
   and timestamp, the heartbeat carries:
   - `phase`, the mission phase; followers copy the master's
   - `master_id`
   - `handover_to`
   - `flags`: MASTER_OK, READY, ELIGIBLE, ORPHAN, AIRBORNE
   - the master's formation `heading`
   - `members`, a bitmask of the followers the master hears

   Position and velocity are in the shared ENU frame; each drone converts its own lat/lon.
   Encoded size is 50 bytes, so 10 drones at 5 Hz is about 2.5 kB/s per receiver.
2. **Election** (`election.py` docstring). Lowest-ID alive eligible drone claims with
   term = highest seen + 1; the step-down rule is exactly §6. Additions:
   - (a) a drone never claims while any peer still reports hearing a live master (MASTER_OK).
     This prevents failovers caused by one drone's own bad link.
   - (b) it waits while a lower-ID eligible drone is alive.
   - (c) no preemption: a recovered lower ID does not take leadership back.
   - (d) nobody claims during the first 3 s after boot.
   - (e) planned low-battery handover: the master names the lowest-ID eligible peer and keeps
     leading until that drone's higher-term heartbeat arrives. If nobody takes over within
     1 s, it names the next candidate.
3. **Slots come from the master's member list**, so every follower computes the same
   assignment even when their own views differ. A drone the master cannot hear becomes an
   **orphan**: after missing from the list for 1 s it flies on a layer 8 m above cruise.
4. **Rank → V slot mapping.** Arm = ID parity (even left, odd right); row = rank by ID within
   the arm; arms rebalance only when their lengths differ by more than 2. This keeps §6 "slot
   = rank among alive drones sorted by ID" while avoiding a failure of the plain alternating
   mapping:
   - With alternating left/right ranks, losing one drone makes every higher rank swap sides.
     With 10 drones, F1 then requires lateral moves up to 64 m across the formation.
   - With parity arms, F4 only slides the same-arm drones behind the lost one by one slot.
     F1 keeps the new master's arm fixed and shifts the other arm by exactly 10 m. Both are
     unit-tested in `test_formation.py`.
5. **Transit layer.** A follower whose slot is more than 12 m away horizontally (for ≥ 0.5 s)
   flies to it on a layer 6 m below the master. It moves sideways at most 1 m/s until it is
   5 m (min separation) clear of the formation layer, and returns once within 4 m of the slot.
   This makes the large re-anchoring after a partition heals safe: a new master is often
   behind the rest.
6. **Altitude layers** relative to cruise (30 m): formation 0, transit −6, orphan +8,
   returning (low battery) +15 m. Validation requires layers ≥ 5 m apart and the top one below
   the 50 m fence minus a 3 m margin.
7. **Low battery (≤ 30 %).** The drone keeps the formation's horizontal velocity while climbing
   to the +15 m layer, flies home there at cruise speed, then lands.
8. **Ghost obstacles.** A drone keeps avoiding a lost peer's position, extrapolated from its
   last heartbeat, for 3 s.
9. **Mission.** IDLE → TAKEOFF (all expected drones READY, or a 30 s timeout) → CRUISE (every
   member within 1.5 m of 30 m) → HOLD (master within 3 m of the goal; hover 10 s) → LAND.
   Followers copy the phase. A new master restarts the current phase timer.
10. **Spawn layout = the V itself** (Phase 1), so takeoff needs no reshuffle.
11. **Vertical rates** follow PX4 defaults verified in source: descent 1.5 m/s
    (`MPC_Z_VEL_MAX_DN`), ascent ≤ 3 m/s (commanded 2), horizontal ≤ 12 m/s (commanded ≤ 10).

## Sources verified (URLs / commands)
- PX4 v1.18.0-rc1 defaults, read from `src/modules/mc_pos_control/*_params.yaml`:
  - `multicopter_position_control_limits_params.yaml`: `MPC_XY_VEL_MAX` 12.0, `MPC_Z_VEL_MAX_UP` 3.0, `MPC_Z_VEL_MAX_DN` 1.5
  - `multicopter_autonomous_params.yaml`: `MPC_ACC_HOR` 3.0
  - `multicopter_altitude_mode_params.yaml`: `MPC_ACC_UP_MAX` 4.0, `MPC_ACC_DOWN_MAX` 3.0
  - `multicopter_takeoff_land_params.yaml`: `MPC_LAND_SPEED` 0.7
- WGS-84 constants and ECEF/ENU formulas are standard. The implementation is checked against an
  independent curvature-radius formula (1 km offsets within 5 cm, and the 0.085 m poleward drift
  of a parallel matches d²·tan φ / 2R within 5 mm) and against ECEF distances (`test_geometry.py`).
- Phase 2 needs no new external interfaces (no ROS/MAVROS calls).

## Problems and fixes
1. **Alternating slot mapping caused large cross-formation moves** and separation dips below 5 m
   in the F1/F5 simulations. Fixed with parity arms (decision 4).
2. **Partition heal: the winning master (higher term) is usually behind the formation**, so the
   front group had to fall back through the rear. Fixed with the transit layer (decision 5).
3. **Bugs found by simulation, and fixed:**
   - *Retiring master counted as a member:* during a handover the retiring master was briefly
     listed as a member, shifting slots. Fix: members = master + follower peers only.
   - *Descent capped by a 3-D saturation:* one saturation on the 3-D correction also capped
     the descent rate. Fix: horizontal and vertical are limited separately.
   - *Ping-pong at 25 m:* the "climb before joining" rule fought the transit layer. Fix: it
     now applies only to drones that never reached cruise altitude.
4. **Four wrong test expectations** were corrected; the code was right each time:
   - the poleward curvature of a parallel
   - the heartbeat is 50 bytes, not 49
   - rebalancing moves two drones in that case
   - an orphan ranked behind another orphan

## Known limitations / honest caveats
- **Separation when drones cannot hear each other.** 143 of the 1,000 randomized runs had a
  minimum separation below 5 m (worst 0.02 m), and **every one of them contained a random
  partition or random link drops**. The 362 runs without those faults (kills, master link
  loss, low battery; loss up to 30 %, latency up to 300 ms, up to 5 faults) had **none**. Among
  single-fault runs: partition 5/33, link_drops 1/28, all other fault types 0.
  Randomly chosen groups are spatially interleaved; each group compacts its formation into
  space the other group occupies, and repulsion cannot act on drones that can't be heard.
  Phase 4 F5 will use **spatially separated groups** (the lower-ID front of the V vs the
  higher-ID back). That is what a distance-driven radio partition looks like, and in that case
  the pure-sim minimum was 6.23 m. Protecting against interleaved blind pairs would need
  onboard proximity sensing or sticky formation references. That is not in this phase; left
  for user review.
- The pure simulator is a point mass: first-order velocity tracking, τ 0.35 s, acceleration
  limits 4 m/s² horizontal and 3 m/s² vertical. PX4 dynamics differ; Phase 3/4 measure the
  real behaviour.
- Convergence in the randomized runs is measured after all links are healed and random loss is
  switched off (latency stays). Behaviour under sustained loss is Phase 5.
- The design decisions above complete gaps in docs/SPECIFICATION.md. Decisions 3–5 (orphan layer, parity
  arms, transit layer) change how slots and altitudes behave during faults; the user should
  review them.

## Next phase: what is needed from the user
Nothing to proceed. Please review the design decisions above, especially parity arms, the
transit and orphan layers, and the F5 partition choice.
