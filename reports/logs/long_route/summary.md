# Long routes

Fast simulator with the real agent code, 10 drones, RL + brake, seed 1, 5 m/s, 25-minute batteries.

| Route | Flight height | Result | Record |
|---|---|---|---|
| F-9 Park -> Rawalpindi Saddar, 12.01 km | auto (30 m: all 266 OSM buildings and woods are lower) | 10/10 landed at the target after 2,795 s; 0 hits; both charging stops used; formation error while cruising max 1.42 m; closest pair 8.53 m | `rawalpindi_12km_auto.json` |
| same route | low (16 m: 253 buildings and 13 woods in the way; leader route 13.51 km) | 7/10 landed after 3,131 s; drones 7, 9 and 10 got stuck at a wall, fell behind and hit it; 3 charging stops; closest pair 5.67 m | `rawalpindi_12km_low.json` |
| F-9 Park -> Lahore, 272.55 km | auto (30 m: 38 buildings of 25 m or more in the way) | **flown to the end** (28 September 2026): 10/10 landed after 64,988 s; 0 hits; all 65 charging stops used; formation error while cruising mean 1.40 m, above 2 m for 295 s, max 201 m when drone 9 was held at a tall building after stop 5; closest pair 5.49 m | `lahore_full.json`, `lahore_full_stop5_detail.json` |
| same route, earlier run in the app | the same | stopped on purpose after 18,088 s (26 September 2026): 76.0 km flown, 18 of 65 stops used, 10/10 flying, 0 hits | `lahore_stopped_2026-09-26.json` |

The 12 km records were made on 27 September 2026 and the full Lahore record on 28 September 2026 with
`scripts/run_route.py` (same code as the app).
The stopped Lahore record of 26 September 2026 holds values read from the running app, which keeps no log files.
