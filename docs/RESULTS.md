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
| Unit tests | 130 pass, 4 skipped without ROS / PyTorch (with PyTorch: 133 pass, 2 skipped) | `python3 -m pytest` |
| 1,000 random runs with crashes, radio loss, delays and splits | exactly one leader after convergence in 1,000 of 1,000; median convergence 0.145 s after the last fault (p95 2.12 s, max 2.72 s) | `reports/PHASE_2.md` |
| Known weak spot | 143 of those 1,000 runs had two drones closer than 5 m; every one contained a random radio split or random link drops (drones that cannot hear each other cannot be pushed apart) | `reports/PHASE_2.md` |

Fault types, 10 drones, 20 runs each (fast simulator). On PX4, Phase 4 was started and stopped (this
laptop is too weak for the 55-trial batch); its indicative results agree, except that F5's formation took
about 17 s to recover after the heal (`reports/PHASE_4.md`):

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

## Obstacle avoidance: learned policy vs classical
Fair comparison: 90 courses never used in training or model selection (30 per density), 10 drones,
every method on the same courses with the same wind-like drift. Success = no drone hits anything and
the whole V re-forms at the target. Classical = potential field + stopping-distance brake, tuned on
training courses; RL = PPO policy after 6 million steps (38 min on the laptop CPU).

| Method | Few obstacles | Medium | Dense | Crashes per mission (medium) | Drones left behind (medium) |
|---|---|---|---|---|---|
| No avoidance | 3/30 | 0/30 | 0/30 | 6.30 | 0.00 |
| Brake only | 8/30 | 3/30 | 0/30 | 2.63 | 0.80 |
| Classical | 15/30 | 7/30 | 3/30 | 0.57 | 1.20 |
| RL | 13/30 | 7/30 | 1/30 | 1.80 | 0.00 |
| **RL + brake** | **26/30** | **18/30** | **5/30** | 0.47 | 0.23 |

Paired on the same courses, RL + brake against classical: few obstacles 13 courses only RL + brake
completed vs 2 only classical (exact McNemar p = 0.007); medium 11 vs 0 (p = 0.001); dense 5 vs 3
(p = 0.73, no clear difference). The classical controller also let drones come as close as
0.88-2.04 m to each other; RL + brake stayed at 3.0 m or more.
Evidence: `reports/logs/rl/eval/` (`summary.md`, `summary.json`, `episodes.jsonl`, `comparison.png`),
tuning `reports/logs/rl/apf_tuning.jsonl`, training `reports/logs/rl/run1/`, `reports/rl_training.png`.

What it means: the policy alone is about as good as the classical controller but fails differently
(it keeps every drone with the formation, but crashes more); with the same brake on top it is
clearly better at low and medium density. Dense clutter is unsolved by every method. Training was
still improving when it stopped, which is why longer training (Kaggle) is the next step.

### On the real map, with the full agent code
F-9 Park to Faisal Mosque, Islamabad (3.2 km; 932 buildings and 26 woods from OpenStreetMap, low
flight height so all of them are obstacles), 10 drones, 5 runs with drift; in the second set the
leader is killed halfway. Drones that hit something over 5 runs / runs with no hit:

| Method | Leader alive | Leader killed halfway |
|---|---|---|
| No avoidance | 45 / 0 of 5 | 40 / 0 of 5 |
| Classical (tuned) | 10 / 1 of 5 | 8 / 0 of 5 |
| Classical (default settings) | 3 / 2 of 5, but many drones left far behind (formation error ~240 m) | 3 / 3 of 5, same caveat |
| RL | 20 / 0 of 5 | 18 / 0 of 5 |
| **RL + brake** | **0 / 5 of 5** | **0 / 5 of 5** |

Every mission completed; after the leader was killed a new leader was agreed in 1.55-1.65 s.
These numbers are from the re-run of 27 September 2026 with the current code, in which the agent
runs its avoider 10 times a second - the rate at which the policy was trained and the classical
controller was tuned. Evidence: `reports/logs/rl/route_eval_10hz/`. The first run (26 September,
avoider every 0.05 s) gave the same result for RL + brake (0 hits in all 10 runs) and for RL (20 and
18), but fewer hits for the classical controller (tuned: 4 / 3 of 5 and 7 / 1 of 5; default: 2 / 3 of 5
and 0 / 5 of 5): at twice the update rate the classical controller does better, still not as well
as RL + brake. Evidence of that run: `reports/logs/rl/route_eval/`.

## Long routes

| Route | Setup | Result |
|---|---|---|
| Islamabad F-9 Park -> Rawalpindi Saddar (33.5973, 73.0479), 12.01 km | normal height (30 m; the 266 OSM buildings and woods are all below 25 m), RL + brake, 2 charging stops | 10/10 drones landed at the target after 2,795 simulated seconds, no hits, both stops used; formation error while cruising max 1.42 m (mean 0.93 m); closest pair 8.53 m |
| same route | low height (16 m; all 253 buildings and 13 woods are obstacles), RL + brake, 3 stops | leader route 13.51 km around them; mission completed after 3,131 s; drones 7, 9 and 10 hit buildings (7 of 10 landed); closest pair 5.67 m |

Both 12 km runs were repeated on 27 September 2026 with the current code (`scripts/run_route.py`)
and gave the same outcome as the first runs of 26 September (2,794.6 s and 10 landed; 3 hits by the
same three drones). At 16 m the three drones that crashed had first got stuck about 1.5 m from a
wall, crawling at 0.5-3 m/s and falling 100-1,250 m behind the formation, before they hit that
building; this is why the formation error while cruising was above 50 m for 446 of 2,704 s (mean
47 m, max 534 m) although the rest of the V held. Evidence: `reports/logs/long_route/rawalpindi_12km_auto.json`,
`rawalpindi_12km_low.json`.
| Islamabad -> Lahore, 272.55 km | normal height, 38 buildings of 25 m or more from OSM, RL + brake, 65 charging stops | stopped on purpose after 76.0 km: 18 of 65 stops used, 10/10 drones flying, no hits (below) |

### Islamabad -> Lahore (stopped early on purpose)
Set-up: 10 drones from F-9 Park, Islamabad (33.7036, 73.0231) to Lahore (31.5204, 74.3587), seed 1,
5 m/s, battery endurance 25 min, flight height "auto" (normal, 30 m on a route this long), avoidance
RL + brake. Map data: OpenStreetMap buildings of 25 m or more along the route, downloaded in 14 boxes
of 20 km with the light "tall buildings only" query: 38 buildings are in the way, 680 lower ones are
flown over. Leader route 272.58 km around them; 65 charging stops about every 4.1 km (half a battery
per leg, 3-minute battery swap).

| When | Simulated time | Stops used | Flown | Left | Drones flying | Obstacle hits |
|---|---|---|---|---|---|---|
| progress check | 7,169 s | 7 of 65 | - | - | 10/10 | 0 |
| progress check | 14,156 s | 14 of 65 | 59.4 km | 213.1 km | 10/10 | 0 |
| **stopped** | **18,088 s (5 h 01 min)** | **18 of 65** | **76.0 km** | **196.6 km** | **10/10** | **0** |

Drone 1 was still the leader when it was stopped. The run went at about 16-38x real time (speed
"Max"); the remaining ~48,000 simulated seconds would have taken about 20-25 more minutes at the
38x it had reached.

Why it was stopped: it was a scale demonstration, not a new test - the charging-stop mechanism had
already completed end to end on the 12 km route above and in `tests/test_long_route.py` - and the
owner asked to stop it on 26 September 2026 so the laptop was free for the remaining work. What it
shows: the long-route machinery (strip download, chunked route planning, 65 charging stops, landing
and take-off at every stop, flying 76 km without a hit or a lost drone) works at city-to-city scale.
Not measured: formation error and closest pair over the whole run (the app only shows them live), and
the remaining 196.6 km. Evidence: `reports/logs/long_route/lahore_stopped_2026-09-26.json` (values read
from the running app; the app keeps no log files). To finish it: `docs/RUNBOOK.md`, section 4.

## Ground-control app
Live 2-D map (Mapbox or Esri satellite, OSM streets), 3-D view (Cesium terrain, OpenStreetMap 3-D
buildings or Google photorealistic tiles, 3-D drone models with smooth motion, V-formation lines,
trails, chase / orbit / top / free cameras, fault buttons and event banners), real GPS home and
target, 1-100 drones, 8 fault types per drone, radio split, speed 1x-Max, obstacle maps and charging
stops on the map, results and documents inside the app. When a new leader takes over, the event log
says how long after the fault it happened (for example "Drone 2 became master (term 2, election),
1.5 s after the fault on drone 1"; checked by `tests/test_gcs_backend.py`).
