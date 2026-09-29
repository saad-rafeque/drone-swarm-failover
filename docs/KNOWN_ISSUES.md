# Known issues, open work and roadmap

Recommendations for what to do next, with effort estimates, are in `docs/FUTURE_WORK.md`.

## Not done (from the original plan in `docs/SPECIFICATION.md`)

| Item | State | What it takes |
|---|---|---|
| Other formations (line, column, echelon, diamond, squads) and their comparison | **line abreast, column and echelon added and compared** on 28 September 2026: in normal flight as good as the V, but after the leader is lost or a radio split heals they come closer than 5 m (line down to 0.57 m), so they are marked experimental and the V stays the default. Diamond and squads: not done | shape-specific rules for moving to new slots (for example every slot change on the transit layer for single-line shapes), then the same comparison (`scripts/formation_compare.py`) |
| Longer RL training, several seeds | **done** on 28-29 September 2026: two 10.5-hour Kaggle GPU runs (seeds 1 and 2); seed 2's policy is now `models/avoid_policy.npz` (`docs/RESULTS.md`). Both runs stopped improving after about 2.5 hours, so more hours with the same settings will not help | a changed set-up (learning-rate decay, more dense courses, a third seed) for dense clutter, which no method solves yet; about 11 GPU hours per run |

## Known limitations
- **No real drone has flown this code.** The Pixhawk 6C cannot run it alone; each drone needs a
  companion computer (for example a Raspberry Pi 4/5) connected to the Pixhawk by serial cable,
  and a radio link between drones for the heartbeats (a bridge node replacing the link emulator).
- **Separation when drones cannot hear each other.** In 143 of 1,000 random fast-simulator runs,
  two drones came closer than 5 m; every such run had a random radio split or random link drops.
  The rules cannot push apart drones that do not hear each other. Needs onboard proximity sensing
  or sticky formation references before real flights.
- **A drone that keeps dropping out takes the lead back each time.** The election gives the lead to the
  lowest alive ID, so a leader whose own link drops in and out (seen in the first Phase 6 attempt, with an
  overloaded radio) keeps losing and retaking the lead. The overloaded link is fixed (`reports/PHASE_6.md`).
  A rule that makes a returning drone wait before it can lead again would change `docs/SPECIFICATION.md`
  section 6, and Phases 2–5 would have to be repeated; this is for the owner to decide.
- **A drone on a telemetry radio needs the radio settings.** PX4 on TELEM1 in Minimal mode at 1,200 B/s
  (`MAV_0_MODE` 7, `MAV_0_RATE` 1200), its agent started with `--radio-link`, and MAVROS time sync at 1 Hz
  (`config/swarm.yaml`, `radio_standin`; `docs/FLIGHT_TEST_PLAN.md`). With PX4's default Normal mode at
  1,200 B/s, position arrives at only about 1 Hz.
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
- **Followers can get stuck against buildings.** On the 12 km route at 16 m, three followers got stuck
  about 1.5 m from a wall, crawled, fell 100-1,250 m behind the formation and finally hit the building
  (7 of 10 landed). At the normal 30 m the same route is clean (formation error max 1.42 m, no hits).
  On the full Islamabad -> Lahore run at 30 m it happened once: after charging stop 5 a tall building
  pushed drone 9 onto the 24 m crossing layer, below the rooftop, where it was held at the wall for about
  2 minutes and fell 600 m behind before it got free and caught up, without a hit. A fix needs a way out
  of dead ends (for example re-planning a short path for a stuck follower, or crossing above a tall
  building instead of below) - the learned avoider only sees 25 m around it. Evidence:
  `reports/logs/long_route/` (`lahore_full_stop5_detail.json`).
- **Formation shapes other than the V are experimental.** Line abreast, column and echelon fly as well as
  the V in normal flight, but the rules that move drones to new slots after the leader is lost or a radio
  split heals were designed for the V: the line came down to 0.57 m between two drones (8 of 10 split
  runs under 5 m), the column and echelon to about 4.6 m (`reports/logs/formations/`).
- **The classical avoider depends on its update rate.** On the real map it hit 4 and 7 buildings when
  run every 0.05 s but 10 and 8 at 10 Hz, its tuned rate; RL + brake had 0 in both (`docs/RESULTS.md`).
- **F5 on PX4: the formation re-forms slowly after a radio split heals.** The group with the newer
  leader wins (the term rule), so the whole V re-forms around it; the transit layer and PX4's vertical
  speeds make that take about 17 s. On 28 September 2026 the owner set F5's limit to 20 s instead of
  changing the rules (`reports/PHASE_4.md`, "Why F5 is slow").
- **This laptop's power.** On battery the power-saver profile throttles the CPU and 10 PX4 drones
  saturate it; the battery also ran out twice on 27 September 2026. Run long PX4 jobs only on the
  charger (`scripts/phase4_runs.sh` waits for it).
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

Also before real flights: PX4 is pinned to a release candidate (v1.18.0-rc1, the first version with
the `px4_sitl_sih` target); move to a stable PX4 release for the real drones and run Phases 0-4 again.

## Ideas for later
- RL where mistakes are cheap: area re-division when drones fail (search and rescue), charging and
  patrol schedules, battery-aware routing - planning on the ground rather than flight control.
- Train on real OpenStreetMap neighbourhoods instead of random courses; train with the brake in
  the loop; curriculum from sparse to dense clutter.
- Terrain heights (for example Mapbox Terrain-DEM or Copernicus) so the swarm keeps its height
  above hills such as the Margalla range.

## Housekeeping
- **Backup: done on 27 September 2026.** The repository, with its full history and tags, is on GitHub
  as a private repository (`saad-rafeque/drone-swarm-failover`). The laptop pushes with a deploy key
  that works for this one repository only (`~/.ssh/drone_swarm_failover_deploy`, selected by the local
  Git setting `core.sshCommand`); if the laptop is lost, delete that key under the repository's
  Settings -> Deploy keys. Git-ignored files are not on GitHub and need their own copy if wanted:
  `config/map_keys.local.yaml` (keys), `.venv/`, `kaggle/swarm-rl-code.zip` (rebuilt by a script) and
  the `Swarm Control` launcher (rebuilt by `scripts/install_launcher.sh`).
- **Licence: Apache-2.0** (`LICENSE`, `NOTICE`), chosen on 29 September 2026. Third-party data keep
  their own terms: OpenStreetMap data (ODbL, attribution), Mapbox and Cesium ion / Google tiles (their
  terms of service, shown as credits in the app).
- **The online project brief is out of date** (published 26 September 2026; see `docs/RUNBOOK.md`,
  section 6).

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
