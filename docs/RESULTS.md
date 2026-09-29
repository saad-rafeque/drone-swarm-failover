# Results

Everything here is simulation. Each number points at the file it came from.

## PX4 autopilot in simulation (SIH + MAVROS, this laptop)

| What | Result | Evidence |
|---|---|---|
| One drone: arm, climb to 10 m, hover 20 s, land | passed twice; 135 MB RAM for PX4 + MAVROS | `reports/PHASE_0.md` |
| 10 drones: all arm, climb to 10 m, hover 30 s, land | passed (also from a clean shell); 1,074 MB RAM (107 MB per drone), 76.5 % CPU during hover | `reports/PHASE_1.md` |
| 10 drones fly a 1 km V mission and land (6 runs) | formation RMS max 1.24-1.66 m (limit 2), mean 0.54-0.62 m; closest pair 7.80-8.65 m (limit 5); leader lands 1.1-1.7 m from the goal | `reports/PHASE_3.md`, `reports/phase3_mission.png` |

## Agent logic (fast simulator, real agent code)

| What | Result | Evidence |
|---|---|---|
| Unit tests | 131 pass, 4 skipped without ROS / PyTorch (with PyTorch: 134 pass, 2 skipped) | `python3 -m pytest` |
| 1,000 random runs with crashes, radio loss, delays and splits | exactly one leader after convergence in 1,000 of 1,000; median convergence 0.145 s after the last fault (p95 2.12 s, max 2.72 s) | `reports/PHASE_2.md` |
| Known weak spot | 143 of those 1,000 runs had two drones closer than 5 m; every one contained a random radio split or random link drops (drones that cannot hear each other cannot be pushed apart) | `reports/PHASE_2.md` |

Fault types, 10 drones, 20 runs each (fast simulator). On PX4, Phase 4 passed on 28 September 2026 (`reports/PHASE_4.md`).
All 55 trials (10 per fault plus a clean-shell cross-check round) met every limit:
- new leader in 1.4–1.7 s;
- planned handover within 0.01 s;
- formation back within 6.6 s (13.4 s after a healed split);
- closest pair 6.03 m or more;
- goal reached 55 of 55.

The fast-simulator results:

| Fault | New leader agreed (median / worst) | Formation back under 2 m | Closest pair | Goal reached |
|---|---|---|---|---|
| F1 leader killed | 1.64 / 1.75 s | 4.55 / 4.68 s | 9.35 m | 20/20 |
| F2 leader's radio lost | 1.64 / 1.75 s | 4.55 / 4.68 s | 8.23 m | 20/20 |
| F3 leader low battery (planned handover) | 0.17 / 0.20 s | 3.14 / 3.23 s | 9.35 m | 20/20 |
| F4 follower killed | no leader change | 1.12 / 1.15 s | 7.46 m | 20/20 |
| F5 radio split, then healed | 0.18 / 0.25 s after the heal | 9.93 / 13.75 s | 6.23 m | 20/20 |

## Swarm size (fast simulator, leader killed at 90 s, 3 runs per size)

| Drones | New leader | Formation back | Closest pair | All landed | Simulator speed | Heartbeat |
|---|---|---|---|---|---|---|
| 2 | 1.60 s | - | 10.0 m | 3/3 | 1,506x real time | 44 B |
| 10 | 1.70 s | 4.50 s | 9.49 m | 3/3 | 162x | 45 B |
| 50 | 1.75 s | 4.95 s | 9.49 m | 3/3 | 11x | 50 B |
| 100 | 1.75 s | 8.75 s | 9.49 m | 3/3 | 2.9x | 56 B |

With 10 % message loss, 150 ms delay and wind-like drift, 100 drones: new leader in 2.75 s median
(worst 2.95 s), never two leaders at once, closest pair 8.14 m, all landed. Evidence:
`reports/SCALING.md`, `reports/logs/scaling/`.

## Radio sweep (Phase 5, fast simulator)
Heartbeat delay 50 / 150 / 300 ms x loss 0 / 10 / 30 % (no jitter), 10 drones; per condition 20 missions
without a fault, 20 with the leader killed (F1) and 20 with its radio cut (F2): 540 runs.

- **No false leader change** in any of the 180 missions without a fault, even at 30 % loss.
- **The Phase 4 limits hold in 8 of 9 conditions**, the worst being 300 ms with 10 % loss (new leader median
  2.65 s, worst 3.09 s) and 150 ms with 30 % loss (2.59 s / 3.27 s). At 300 ms with 30 % loss the median
  time to a new leader is 3.06 s against the 3.0 s limit (worst 3.80 s, within 4.0 s).
- Formation back under 2 m within 9.47 s in every run (limit 15 s); closest pair 6.44 m or more; every
  mission reached the goal.

- **On PX4** (18 missions with 10 drones, one without a fault and one with the leader killed per condition):
  - no false leader change;
  - a new leader in 1.59–2.48 s in all nine conditions, including 300 ms with 30 % loss (2.30 s);
  - formation back within 6.84 s, closest pair 6.89 m or more, every goal reached.

Evidence: `reports/PHASE_5.md`, `reports/logs/phase_5_fastsim/` (`summary.md`, `summary.json`,
`radio_sweep.jsonl`), `reports/logs/phase_5/` (PX4 runs, `phase5_px4_table.md`), `reports/phase5_radio_sweep.png`.

## Formation shapes (fast simulator)
Four shapes with the same agent code, 10 drones, clean radio, 10 runs each of: no fault, F1 (leader killed),
F4 (follower killed), F5 (radio split, then healed).

| Shape | Formation back after F1, median (s) | After F5, worst (s) | Fault runs closer than 5 m | Closest pair (m) | Goal |
|---|---|---|---|---|---|
| V | 4.47 | 12.20 | 0 of 30 | 6.17 | 40/40 |
| Line abreast | 4.71 | 12.45 | 8 of 30 | 0.57 | 40/40 |
| Column | 25.62 | 20.95 | 20 of 30 | 4.59 | 40/40 |
| Echelon (right) | 11.27 | 19.90 | 20 of 30 | 4.60 | 40/40 |

In normal flight every shape holds its slots equally well (formation error 0.02 m mean), and losing a
follower (F4) is handled the same way. After the leader is lost or a radio split heals, only the V keeps
every pair at 5 m or more: the rules that move drones to their new slots were designed for the V. In a line
abreast, drones slide sideways past each other at the same height (8 of 10 F5 runs under 5 m, worst
0.57 m); in a column or echelon, moves along the line come to about 4.6 m and take up to 21 s. **The V
stays the default**; the other shapes are available (`formation.shape`, and the Formation menu of the app)
but marked experimental. Evidence: `reports/logs/formations/` (`summary.md`, `runs.jsonl`),
`reports/formation_shapes.png`.

## Obstacle avoidance: learned policy vs classical
Fair comparison: 90 courses never used in training or model selection (30 per density), 10 drones,
every method on the same courses with the same wind-like drift. Success = no drone hits anything and
the whole V re-forms at the target. Classical = potential field + stopping-distance brake, tuned on
training courses. RL = the policy in use (`models/avoid_policy.npz`): PPO on a Kaggle T4 GPU, seed 2,
5.84 billion steps in 10.5 hours, best checkpoint on the validation courses.

| Method | Few obstacles | Medium | Dense | Crashes per mission (medium) | Drones left behind (medium) |
|---|---|---|---|---|---|
| No avoidance | 3/30 | 0/30 | 0/30 | 6.30 | 0.00 |
| Brake only | 8/30 | 3/30 | 0/30 | 2.63 | 0.80 |
| Classical | 15/30 | 7/30 | 3/30 | 0.57 | 1.20 |
| RL | 28/30 | 19/30 | 11/30 | 0.67 | 0.00 |
| **RL + brake** | **30/30** | **23/30** | **18/30** | 0.23 | 0.10 |

Paired on the same courses, RL + brake against classical: few obstacles 14 courses only RL + brake
completed vs 1 only classical (exact McNemar p = 0.001); medium 13 vs 1 (p = 0.002); dense 9 vs 1
(p = 0.021). RL + brake is now clearly better at every density, dense clutter included. The classical
controller let drones come as close as 0.88-2.04 m to each other; RL + brake 2.88 m (dense) to 4.47 m.
Evidence: the laptop re-test `reports/logs/rl/eval_kaggle_check/` (`summary.md`, `summary.json`,
`episodes.jsonl`, `comparison.png`), tuning `reports/logs/rl/apf_tuning.jsonl`, training
`reports/logs/rl/kaggle_seed2/`, `reports/rl_training_kaggle_seed2.png`.

### Training runs compared (the same 90 courses)
| Policy | Training | RL + brake: few / medium / dense | RL alone: few / medium / dense |
|---|---|---|---|
| Laptop (`models/avoid_policy_run1.npz`) | 6 million steps, 38 min on the laptop CPU | 26 / 18 / 5 | 13 / 7 / 1 |
| Kaggle seed 1 | 5.50 billion steps, 10.5 h on a T4 GPU | 27 / 18 / 14 | 26 / 15 / 4 |
| **Kaggle seed 2 (in use)** | 5.84 billion steps, 10.5 h on a T4 GPU | **30 / 23 / 18** | **28 / 19 / 11** |

- The two seeds are independent runs with the same settings; seed 2 is better on every count.
- The laptop re-test gave exactly Kaggle's RL numbers for seed 2. Kaggle's own classical row is not usable: its
  upload lacked the tuning file, so it ran untuned (`eval_kaggle_seed*/`, "APF {}").
- **More hours alone will not help.** The validation score of both runs rose for about 2.5 hours and then only
  went up and down (seed 1 best 51 of 90 at 2.5 h; seed 2 best 63 of 90 at 6.5 h, 62 already at 2.5 h). A next
  run needs a changed set-up, for example learning-rate decay or more dense courses.
- Rules for choosing (`docs/KAGGLE_GUIDE.md`, section 8): seed 2 has at least as many successes at every density,
  fewer crashes, and 0 hits on the real map. Seed 1 failed the last rule (1 hit).

What it means: the long GPU training made the policy much better, most of all in dense clutter (5 to 18 of 30
with the brake, 1 to 11 without). Dense clutter is still the hardest case: 12 of 30 dense missions still fail.

### On the real map, with the full agent code
F-9 Park to Faisal Mosque, Islamabad (3.2 km; 932 buildings and 26 woods from OpenStreetMap, low
flight height so all of them are obstacles), 10 drones, 5 runs with drift; in the second set the
leader is killed halfway. Drones that hit something over 5 runs / runs with no hit:

| Method | Leader alive | Leader killed halfway |
|---|---|---|
| No avoidance | 45 / 0 of 5 | 40 / 0 of 5 |
| Classical (tuned) | 10 / 1 of 5 | 8 / 0 of 5 |
| Classical (default settings) | 3 / 2 of 5, but many drones left far behind (formation error ~240 m) | 3 / 3 of 5, same caveat |
| RL | 13 / 0 of 5 | 12 / 0 of 5 |
| **RL + brake** | **0 / 5 of 5** | **0 / 5 of 5** |

Every mission completed; after the leader was killed a new leader was agreed in 1.50-1.65 s. The avoider runs
10 times a second, the rate the policy was trained at. Evidence: `reports/logs/rl/route_eval_kaggle_check/`.
Compared with the laptop policy (`reports/logs/rl/route_eval_10hz/`): RL alone hit much less (13 and 12 against
20 and 18), and RL + brake had 0 hits in both. **One caveat:** with the new policy two drones came to 4.77 m of each
other once on this route (laptop policy: 5.07 m). It is below the 5 m limit, so check it before
this policy flies near real drones.
The first run of 26 September (avoider every 0.05 s, laptop policy) is kept in `reports/logs/rl/route_eval/`.

## Long routes

| Route | Setup | Result |
|---|---|---|
| Islamabad F-9 Park -> Rawalpindi Saddar (33.5973, 73.0479), 12.01 km | normal height (30 m; the 266 OSM buildings and woods are all below 25 m), RL + brake, 2 charging stops | 10/10 drones landed at the target after 2,795 simulated seconds, no hits, both stops used; formation error while cruising max 1.42 m (mean 0.93 m); closest pair 8.53 m |
| same route | low height (16 m; all 253 buildings and 13 woods are obstacles), RL + brake, 3 stops | leader route 13.51 km around them; mission completed after 3,131 s; drones 7, 9 and 10 hit buildings (7 of 10 landed); closest pair 5.67 m |
| Islamabad -> Lahore, 272.55 km | normal height, 38 buildings of 25 m or more from OSM, RL + brake, 65 charging stops | **flown to the end on 28 September 2026**: all 10 drones landed in Lahore after 64,988 simulated seconds (18 h 03 min), all 65 stops used, no hits; formation error while cruising mean 1.40 m, with one episode up to 201 m (drone 9 held at a tall building for about 2 minutes, below); closest pair 5.49 m |

Both 12 km runs were repeated on 27 September 2026 with the current code (`scripts/run_route.py`)
and gave the same outcome as the first runs of 26 September (2,794.6 s and 10 landed; 3 hits by the
same three drones). At 16 m the three drones that crashed had first got stuck about 1.5 m from a
wall, crawling at 0.5-3 m/s and falling 100-1,250 m behind the formation, before they hit that
building; this is why the formation error while cruising was above 50 m for 446 of 2,704 s (mean
47 m, max 534 m) although the rest of the V held. Evidence: `reports/logs/long_route/rawalpindi_12km_auto.json`,
`rawalpindi_12km_low.json`.

### Islamabad -> Lahore, flown to the end (28 September 2026)
Set-up: 10 drones from F-9 Park, Islamabad (33.7036, 73.0231) to Lahore (31.5204, 74.3587), seed 1,
5 m/s, battery endurance 25 min, flight height "auto" (normal, 30 m on a route this long), avoidance
RL + brake. Map data: OpenStreetMap buildings of 25 m or more along the route, downloaded in 14 boxes
of 20 km with the light "tall buildings only" query: 38 buildings are in the way, 680 lower ones are
flown over. Leader route 272.58 km around them; 65 charging stops about every 4.1 km (half a battery
per leg, 3-minute battery swap).

| Simulated time | Stops used | Drones landed in Lahore | Obstacle hits | Formation error while cruising | Closest pair |
|---|---|---|---|---|---|
| 64,987.6 s (18 h 03 min) | 65 of 65 | 10 of 10 | 0 | mean 1.40 m; above 2 m for 295 s in total; max 201.3 m | 5.49 m |

Drone 1 led the whole way. The run took 24 minutes on the laptop (45x real time, with other jobs
running; the first run the same morning took 19 minutes at 57x and gave the same outcome and the same
events, as the fast simulator is repeatable).

**The 201 m episode, explained.** About 90 s after the swarm left charging stop 5 (t = 5,036 s), a tall
building beside the route pushed drone 9 more than 12 m from its slot, so it dropped to the 24 m crossing
layer - still below the top of the building - and was held 1.4-1.7 m from the wall, almost stopped, for
about 115 s while the V flew on. At the worst moment (t = 5,150 s) it was 599 m from its slot. It then got
free, caught up at 10 m/s and was back in its slot at t = 5,271 s, without a hit. This is the dead-end
weakness of the followers' avoider (`docs/KNOWN_ISSUES.md`), seen until now only when flying low. A second,
shorter episode on the last leg (t = 64,690-64,731 s, max 43 m) was not analysed.
Evidence: `reports/logs/long_route/lahore_full.json` (whole run: events, progress every 1,000 s, the
episodes above 10 m), `lahore_full_stop5_detail.json` (every follower every 5 s from 5,000 to 5,300 s,
recorded by replaying the mission with `scripts/route_window.py`).

The earlier run in the app on 26 September 2026 was stopped on purpose after 76.0 km (18 of 65 stops,
10/10 flying, 0 hits); its record is `reports/logs/long_route/lahore_stopped_2026-09-26.json`.

## Hardware readiness (Phase 6, software only)
- **Drone profiles** (`config/profiles/`): `sim.yaml`, `standin.yaml` (drone 1 behind the radio stand-in) and
  `mixed.yaml` (drone 1 real, through a serial URL placeholder). `scripts/check_profile.py` checks them; a real
  drone is checked as text only, and the simulation tools refuse to start a profile that has one
  (`tests/test_profiles.py`, 12 tests).
- **Telemetry-radio stand-in** (`src/swarm_tools/radio_proxy.py`): a relay between one drone's PX4 and MAVROS
  with SiK defaults - 64 kbit/s air rate halved by error correction to 32 kbit/s shared by both directions,
  transmit turns of up to 131 ms, 1 % loss, a 1 s buffer. Its tests (`tests/test_radio_proxy.py`, 6 tests)
  measure the rate (2,000 bytes delivered in 0.45-0.75 s), the shared channel, the turn wait, loss and the
  full buffer with local sockets.
- **The PX4 stand-in test** (F1 and F2 with drone 1 behind the stand-in): the first four runs failed because the
  simulated autopilot flooded the radio, 1.7–3.9 times its capacity.
  - The radio drone is now set up like a real telemetry port: PX4 Minimal mode at 1,200 B/s, the agent asking
    for position at 5 Hz, and 1 s allowed without autopilot data.
  - The 22 acceptance trials then passed, 29 September 2026 (`reports/PHASE_6.md`):
    - New leader: F1 median 1.51 s, worst 1.60 s; F2 median 1.46 s, worst 1.52 s (limits 3 / 4 s).
    - Formation back under 2 m within 6.74 s (limit 15 s); closest pair 7.65 m; goal reached 22 of 22.
    - Radio: 180,937 packets, none dropped for lack of room, 1.15 % lost, mean delay 68–84 ms.
- **Flight test plan**: `docs/FLIGHT_TEST_PLAN.md` (one real drone with simulated ones; PX4 safety
  parameters checked in the PX4 source; go/no-go checklist; abort criteria; kill-switch procedure).

## Ground-control app
Live 2-D map (Mapbox or Esri satellite, OSM streets), 3-D view (Cesium terrain, OpenStreetMap 3-D
buildings or Google photorealistic tiles, 3-D drone models with smooth motion, V-formation lines,
trails, chase / orbit / top / free cameras, fault buttons and event banners), real GPS home and
target, 1-100 drones, 8 fault types per drone, radio split, speed 1x-Max, obstacle maps and charging
stops on the map, results and documents inside the app. When a new leader takes over, the event log
says how long after the fault it happened (for example "Drone 2 became master (term 2, election),
1.5 s after the fault on drone 1"; checked by `tests/test_gcs_backend.py`).
