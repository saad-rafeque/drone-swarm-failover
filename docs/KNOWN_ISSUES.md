# Known issues, what is not done, and next steps

## Not done (from the original plan in `docs/SPECIFICATION.md`)

| Item | State | What it takes |
|---|---|---|
| Phase 4: fault trials on PX4 (leader killed, radio lost, low battery, follower killed, radio split; 10 trials each) | started twice on 27 September 2026, stopped by the owner: this laptop saturates with 10 PX4 drones (indicative results: all takeovers within limits; F5 formation recovery ~17 s against 15 s) | `bash scripts/phase4_all.sh 1` on a machine with 8+ cores (~7 h), or `N=5` on this laptop; step by step in `reports/PHASE_4.md` |
| Phase 5: radio realism sweep (50/150/300 ms x 0/10/30 % loss) | not started | a sweep driver around `scripts/run_mission.py` with the link emulator settings; several hours |
| Phase 6: mixed-reality readiness (1 real + N simulated drones, telemetry-radio stand-in, flight test plan, go/no-go checklist) | not started | profiles in `config/profiles/`, a UDP proxy that limits bandwidth, `FLIGHT_TEST_PLAN.md` |
| Other formations (line, column, echelon, diamond, squads) and their comparison | not started | generalise `formation.py` beyond the V; compare formation error, failover and obstacle results |
| Longer RL training, several seeds | kit ready and checked on 27 September 2026, **not run** | one ~11-hour Kaggle GPU session per seed, step by step in `docs/KAGGLE.md` |
| Islamabad -> Lahore run to the end | stopped on purpose at 76 km (18 of 65 stops, 10/10 drones, 0 hits) | `PYTHONPATH=src python3 scripts/run_route.py --target 31.5204 74.3587 --out reports/logs/long_route/lahore.json` (no browser, about 30-60 min), or in the app |

## Known limitations
- **No real drone has flown this code.** The Pixhawk 6C cannot run it alone; each drone needs a
  companion computer (for example a Raspberry Pi 4/5) connected to the Pixhawk by serial cable,
  and a radio link between drones for the heartbeats (a bridge node replacing the link emulator).
- **Separation when drones cannot hear each other.** In 143 of 1,000 random fast-simulator runs,
  two drones came closer than 5 m; every such run had a random radio split or random link drops.
  The rules cannot push apart drones that do not hear each other. Needs onboard proximity sensing
  or sticky formation references before real flights.
- **Dense clutter is unsolved.** In the densest obstacle courses the best method (RL + brake)
  completes 5 of 30; followers still hit buildings in dense city blocks at 16 m.
- **Simplified physics.** PX4 SIH uses a simple aircraft model; the fast simulators use point
  masses with a first-order velocity response and a random drift instead of real wind.
- **Perfect sensing assumed.** Positions, velocities and range readings are exact in simulation.
  Real GPS drifts by metres (use RTK GPS or wider spacing); range readings would come from a real
  sensor (lidar or depth camera) or from a map that may be out of date.
- **2-D obstacles.** Obstacles are footprints at the flight height; flying over is not modelled,
  OSM heights are often missing (defaults: houses 9 m, woods 15 m).
- **Map data availability.** The public Overpass servers are often busy (HTTP 429/504); downloads
  retry and are cached, but a first long-route download can take tens of minutes.
- **Radio load.** Heartbeats go from every drone to every drone: 224 kbit/s for 100 drones, far
  more than one telemetry radio channel (SiK default 64 kbit/s). Large swarms need a mesh network,
  lower rates, or squads.
- **One V for large swarms.** With 100 drones each arm is about 490 m long; big swarms should fly
  as squads of about ten.
- **PX4 on this laptop is limited to 10 drones** (CPU, and PX4's default port plan).
- **Phase 3 harness:** two runs ended with the harness's watcher thread stalled (flights fine);
  the fix is in the code but has not been re-run on PX4.
- **Followers can get stuck against buildings when flying low.** On the 12 km route at 16 m, three
  followers got stuck about 1.5 m from a wall, crawled, fell 100-1,250 m behind the formation and
  finally hit the building (7 of 10 landed). At the normal 30 m the same route is clean (formation
  error max 1.42 m, no hits). A fix needs a way out of dead ends (for example re-planning a short
  path for a stuck follower) - the learned avoider only sees 25 m around it. Evidence:
  `reports/logs/long_route/`.
- **The classical avoider depends on its update rate.** On the real map it hit 4 and 7 buildings when
  run every 0.05 s but 10 and 8 at 10 Hz, its tuned rate; RL + brake had 0 in both (`docs/RESULTS.md`).
- **3-D view heights.** The simulators have flat ground; the 3-D view draws every drone at its
  simulated height above the real terrain under it (Cesium World Terrain), so on hills it shows
  height above ground, not a real climb. For the first seconds after the page opens the terrain is
  still coarse and the picture can be dark or blurred until the tiles arrive.
- **3-D view load.** Google photorealistic tiles and sun shadows are heavy for this laptop's
  integrated GPU; keep them off while a long simulation runs. The page draws at most 40 frames per
  second; above 20 drones only the leader gets a trail and a label.

## Road to real drones (in order)
1. Phase 4 and Phase 5 on PX4 (fault trials, bad radio).
2. Fix the separation weak spot for drones that cannot hear each other.
3. Measure the real drone: weight, battery capacity, motor and propeller data, PX4 flight logs;
   put the numbers into the simulators (a digital twin), re-run the tests.
4. Companion computer per drone (Ubuntu 24.04, ROS 2 Jazzy, MAVROS, this agent), wired to the
   Pixhawk 6C; a heartbeat radio bridge (WiFi mesh or telemetry radios).
5. Bench test with propellers off; then one real drone with nine simulated ones (Phase 6), RC
   override and a kill switch on every real drone, PX4 geofence and failsafes configured.
6. Three real drones in an open field with safety pilots; only then more.
7. Legal permission for flights (local civil aviation rules) before any outdoor test.

## Ideas for later
- RL where mistakes are cheap: area re-division when drones fail (search and rescue), charging and
  patrol schedules, battery-aware routing - planning on the ground rather than flight control.
- Train on real OpenStreetMap neighbourhoods instead of random courses; train with the brake in
  the loop; curriculum from sparse to dense clutter.
- Terrain heights (for example Mapbox Terrain-DEM or Copernicus) so the swarm keeps its height
  above hills such as the Margalla range.

## Found and fixed during the final check (27 September 2026)
- **Kaggle bundle packed the private map keys.** `scripts/make_kaggle_bundle.py` zipped the whole
  `config/` folder, including `config/map_keys.local.yaml`. The zip had not been uploaded. The script
  now skips `*.local.*` files and refuses to build if any packed file contains a key value;
  `tests/test_tools_scripts.py` checks both.
- **GPU training would have chosen its best policy on the test courses.** `build_pools.py` gave the
  trainer the same 90 courses the final comparison reports on. Now the trainer scores separate
  validation courses (`val.npz`, seeds 1,000,000-1,000,029); the test courses (1,000,100-1,000,129)
  are used only for the final report (checked by `tests/test_tools_scripts.py`).
- **3-D view:** drones were drawn before the terrain height was known, so their trails started
  hundreds of metres below the ground and the camera could end up under the terrain. Drones now
  appear once the ground height is known, trails are stored as height above ground, and the camera
  lifts itself above hills.
- **Learning-curve plot** could not read the GPU trainer's logs; `scripts/plot_rl_training.py` now
  reads both formats.
- **Stopping the app did not stop it.** `scripts/start_swarm.sh` saved the process ID of a helper shell
  instead of the app, so `scripts/stop_swarm.sh` said "stopped" while the app kept running. The start
  script now saves the app's own ID, and the stop script checks that the process really is the app
  (and falls back to looking for it) before and after stopping it.
- **In-app Docs page showed no documents.** `static/md.js` had a JavaScript syntax error ("use strict"
  inside a function with a default parameter). Fixed; the viewer also shows pictures and quote boxes now.
