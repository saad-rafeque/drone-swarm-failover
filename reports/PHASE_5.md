# Phase 5 — Radio realism (telemetry-like links)
Status: NOT COMPLETED — the sweep passed in the fast simulator (the real agent code); the same sweep on PX4
(`scripts/phase5_runs.sh`) is scheduled in rounds on this laptop and is not finished yet.

## Acceptance criteria
- [DONE, fast simulator] Sweep the heartbeat link: delay {50, 150, 300} ms x loss {0, 10, 30} %. Nine
  conditions, each with 20 missions without a fault, 20 with the leader killed (F1) and 20 with the
  leader's radio cut (F2): 540 runs (`reports/logs/phase_5_fastsim/radio_sweep.jsonl`).
  PX4: pending, 18 missions (`scripts/phase5_runs.sh`).
- [DONE, fast simulator] Results table and plot: the table below (`reports/logs/phase_5_fastsim/summary.md`)
  and `reports/phase5_radio_sweep.png`.
- [PASS, fast simulator] Worst condition where the Phase 4 criteria still hold: **300 ms at 10 % loss** (new
  leader median / worst 2.65 / 3.09 s, formation back within 5.71 s) and **150 ms at 30 % loss**
  (2.59 / 3.27 s, 5.67 s). They fail only at 300 ms with 30 % loss: the median time to a new
  leader is 3.06 s against 3.0 s (worst 3.80 s, within the 4.0 s limit).
- [PASS, fast simulator] Zero false failovers at <= 10 % loss: 0 leader changes in 120 missions without a fault
  (and 0 in the 60 missions at 30 % loss). Closest pair 6.44 m or more in every run.

## Key numbers (fast simulator, 10 drones)
| Delay (ms) | Loss (%) | False leader changes (missions affected) | New leader median / worst (s) | Formation < 2 m median / worst (s) | Closest pair (m) | Goal | Phase 4 limits | No false change |
|---|---|---|---|---|---|---|---|---|
| 50 | 0 | 0 (0/20) | 1.66 / 1.74 | 4.47 / 4.59 | 8.23 | 60/60 | hold | yes |
| 50 | 10 | 0 (0/20) | 1.86 / 2.30 | 4.51 / 4.85 | 8.23 | 60/60 | hold | yes |
| 50 | 30 | 0 (0/20) | 2.26 / 3.25 | 4.72 / 5.55 | 6.82 | 60/60 | hold | yes |
| 150 | 0 | 0 (0/20) | 1.95 / 2.04 | 4.61 / 4.74 | 8.23 | 60/60 | hold | yes |
| 150 | 10 | 0 (0/20) | 2.16 / 2.60 | 4.70 / 5.05 | 8.23 | 60/60 | hold | yes |
| 150 | 30 | 0 (0/20) | 2.59 / 3.27 | 4.95 / 5.67 | 6.44 | 60/60 | hold | yes |
| 300 | 0 | 0 (0/20) | 2.40 / 2.49 | 4.92 / 5.04 | 8.24 | 60/60 | hold | yes |
| 300 | 10 | 0 (0/20) | 2.65 / 3.09 | 5.03 / 5.71 | 8.24 | 60/60 | hold | yes |
| 300 | 30 | 0 (0/20) | 3.06 / 3.80 | 5.56 / 9.47 | 7.68 | 60/60 | FAIL | yes |

![Radio sweep in the fast simulator: false leader changes, time to a new leader, formation recovery](phase5_radio_sweep.png)

## How it was measured
- `scripts/radio_sweep.py` runs the fast simulator (`src/swarm_tools/puresim.py`) with the real agent code
  for every drone. Its radio model delays every heartbeat on every link by the given delay and drops each one
  independently with the given loss; no jitter, like the PX4 sweep.
- **False leader change:** a new leader (a new drone or a new term) after the swarm has taken off, in a
  mission where nothing failed. The first election on the ground does not count.
- **New leader (F1, F2):** from the fault until every drone still flying follows the same new leader
  (`scripts/puresim_faults.py`, the same definition as the Phase 4 fast-simulator table).
- **Formation back:** from the fault until the formation error is below 2 m again.
- Limits: Phase 4's (new leader median 3.0 s and worst 4.0 s; formation back within 15 s; closest pair
  >= 5 m; goal in >= 9 of 10 runs) and Phase 5's (no false failover at <= 10 % loss).

## Sources verified
- The sweep grid and the criteria: `docs/SPECIFICATION.md`, Phase 5.
- The radio model: `Network` in `src/swarm_tools/puresim.py` (per-link delay, jitter and loss) and the PX4
  path's `src/swarm_tools/link_emulator.py`, which applies the same settings to the ROS 2 heartbeat topics
  (`scripts/run_mission.py --latency-ms --loss-pct`).

## Problems and fixes
- The first draft of the figure marked the 300 ms / 30 % cell "over limit" while showing the worst value
  (3.36 s, within the 4.0 s limit) instead of the median that failed; the panel now shows the median, with
  the worst value underneath.

## Known limitations / honest caveats
- **Fast simulator only so far.** Point-mass physics; PX4's own timing is not in these numbers. The PX4
  sweep (one mission without a fault and one F1 mission per condition) is scheduled in rounds; it is much
  smaller.
- **Independent packet loss.** Real radios lose packets in bursts (interference, range); bursts would stretch
  the time to a new leader more than the same average loss spread evenly.
- **No jitter** in this sweep; the scaling test (`reports/SCALING.md`) covers 10 % loss with 150 ms delay and
  30 ms jitter.

## Next phase: what is needed from the user
Nothing. The PX4 sweep is scheduled in rounds, as agreed with the owner on 28 September 2026: 10 drones, one
part at a time, only on the charger.
