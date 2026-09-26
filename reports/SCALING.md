# Scaling test — does the swarm logic work for 1 to 100 drones?
Status: PASSED in the fast simulator (not PX4). Date: 2026-09-26. Commit of the code under test: 2291af7.

## What was run
`scripts/scale_test.py` flies the default 1 km mission (`config/swarm.yaml`, V formation, 10 m spacing,
30 m, 5 m/s) in the pure-Python simulator (`src/swarm_tools/puresim.py`), which runs the real agent code
(`AgentCore`: election, formation, safety) for every drone on point-mass physics. For every N ≥ 2 the
leader (drone 1) is killed at t = 90 s, mid-cruise. Three seeds per N.

- Clean: `PYTHONPATH=src python3 scripts/scale_test.py --ns 1 2 3 5 10 20 50 100 --seeds 1 2 3`
  → `reports/logs/scaling/fastsim_scaling.jsonl` (24 runs)
- Harder radio: same with `--loss-pct 10 --latency-ms 150 --drift 0.15` (10 % of heartbeats lost per link,
  150 ms delay + up to 30 ms jitter, 0.15 m/s Gauss-Markov velocity drift) for N = 10, 50, 100
  → `reports/logs/scaling/fastsim_scaling_radio.jsonl` (9 runs)
- Machine: `reports/logs/scaling/machine.txt` (i3-1115G4, 8 GB, other apps open). Sim speed depends on load.

Definitions: *New leader agreed* = time from the kill until every alive drone follows the same new leader
with the same term. *Formation < 2 m after* = time from the kill until formation RMS error is back below 2 m.
*Closest pair* = minimum distance between any two airborne drones over the whole flight. *Landed* = mission
completed, every flying drone landed, exactly one leader at the end. Values are median (min–max) of 3 runs.

## Results — clean radio, no drift
| N | New leader agreed (s) | Formation < 2 m after (s) | Closest pair (m) | Cruise RMS max (m) | Landed | Sim speed (× real time) | Heartbeat (B) | Radio load (kbit/s) |
|---|---|---|---|---|---|---|---|---|
| 1 | – | – | – | – | 3/3 | 2459.6 (2321.3–2483.4) | 44 | 1.8 |
| 2 | 1.60 (1.50–1.60) | – | 10.00 | 0.39 | 3/3 | 1506.0 (1289.0–1654.6) | 44 | 3.5 |
| 3 | 1.70 (1.60–1.70) | 4.75 (4.65–4.75) | 9.49 | 0.73 | 3/3 | 818.5 (622.6–848.8) | 44 | 5.3 |
| 5 | 1.70 (1.60–1.70) | 4.60 (4.50–4.60) | 9.49 | 0.73 | 3/3 | 426.8 (416.8–439.6) | 44 | 8.8 |
| 10 | 1.70 (1.60–1.75) | 4.50 (4.40–4.55) | 9.49 | 0.73 | 3/3 | 162.1 (160.9–162.2) | 45 | 18.0 |
| 20 | 1.70 (1.70–1.75) | 4.60 (4.60–4.65) | 9.49 | 0.73 | 3/3 | 57.3 (56.8–57.6) | 46 | 36.8 |
| 50 | 1.75 (1.70–1.75) | 4.95 (4.90–4.95) | 9.49 | 0.73 | 3/3 | 10.9 (10.8–11.3) | 50 | 100.0 |
| 100 | 1.75 (1.70–1.75) | 8.75 (8.70–8.75) | 9.49 | 0.73 | 3/3 | 2.9 (2.8–2.9) | 56 | 224.0 |

## Results — 10 % loss, 150 ms delay, drift 0.15 m/s
| N | New leader agreed (s) | Formation < 2 m after (s) | Closest pair (m) | Cruise RMS max (m) | Max leaders at once | Leader sequence | Landed | Sim speed (× real time) | Heartbeat (B) | Radio load (kbit/s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 2.35 (2.15–2.40) | 4.80 (4.65–4.90) | 9.15 | 1.14 | 1 | 1→2 | 3/3 | 150.4 (147.8–153.5) | 45 | 18.0 |
| 50 | 2.55 (2.55–2.75) | 5.70 (5.40–6.30) | 8.68 | 1.11 | 1 | 1→2 | 3/3 | 10.3 (9.7–10.9) | 50 | 100.0 |
| 100 | 2.75 (2.50–2.95) | 9.20 (8.95–9.30) | 8.14 | 1.05 | 1 | 1→2 | 3/3 | 2.6 | 56 | 224.0 |

All 33 runs completed with every drone landed and exactly one leader at the end; in the harder-radio runs the
leader sequence was always 1 → 2 and there were never two leaders at once. Closest approach never went below
8.14 m (limit 5 m). The worst new-leader time was 2.95 s (N = 100, harder radio) against the Phase 4 target of
3.0 s median / 4.0 s worst.

## What changed to make 100 drones possible
The heartbeat's member list was a 64-bit mask, so the configuration refused more than 63 drones
(`ConfigError: drone IDs must be within 1..63`, checked before the change). The mask is now sent with only as
many bytes as the highest drone ID needs; IDs run 1..250, the PX4 `MAV_SYS_ID` range
(`src/modules/mavlink/mavlink_params.yaml` in PX4 v1.18.0-rc1: min 1, max 250). Heartbeat size: 44 B for up to
7 drones, 45 B for 10, 50 B for 50, 56 B for 100, 75 B for 250 (`tests/test_heartbeat.py`).

## Honest limits
- **Simplified physics.** This is the point-mass simulator, not PX4. PX4 SIH on this laptop stays at 10 drones
  (Phase 1: 77 % CPU at 10; PX4's default port plan in `px4-rc.mavlink` covers 10 instances).
- **Simulator cost grows with N².** Every drone checks every other drone: 162× real time at 10 drones,
  2.9× at 100 (clean runs).
- **Radio load grows with N.** Heartbeats go to every drone, 5 per second: 18 kbit/s at 10 drones, 224 kbit/s
  at 100 (payload only, before framing). ArduPilot's SiK radio documentation gives a default air rate of 64 kbit/s,
  a maximum of 250, and usable throughput of about 90 % of the air rate (half with error correction):
  https://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html
- **One V for 100 drones is impractical.** Each arm is about 490 m long and the last drones land 490 m from
  the target. Big swarms should fly as squads (for example ten groups of ten, each with its own leader).
- The formation takes longer to recover at 100 drones (8.75 s clean, 9.2 s harder radio) because every drone on
  one arm shifts one slot when the leader's successor moves to the tip; still inside the 15 s Phase 4 target.
