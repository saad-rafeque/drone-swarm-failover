# Runbook: installation and operation

All commands run from the repository root. The path of this repository contains a space
(`drone swarm`): quote it in shells, and note that PX4 itself cannot run from such a path, so its
working folders go to `/tmp/swarm_sim/` (`config/swarm.yaml` -> `sim.work_dir`).

## 1. Install from scratch (Ubuntu 24.04)
Steps that need `sudo` must be run by the owner of the machine.

1. ROS 2 Jazzy (desktop or base) from packages.ros.org, then `sudo apt install ros-jazzy-mavros`
   (installed here: ros-jazzy-mavros and ros-jazzy-mavros-msgs 2.15.1).
2. GeographicLib datasets for MAVROS (egm96-5 geoid and the magnetic model):
   `sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh`. When its SourceForge download
   fails (it did here), download the files by hand and copy them to `/usr/share/GeographicLib/`
   (record: `reports/logs/phase_0/sudo_installs.log`, `geographiclib_datasets.txt`).
3. PX4: `git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot --recursive`,
   `cd ~/PX4-Autopilot && git checkout v1.18.0-rc1 && git submodule update --init --recursive`,
   `bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools` (sudo), then from this repository
   `scripts/build_px4.sh` (builds `px4_sitl_sih` with 2 jobs; v1.17 has no `px4_sitl_sih`).
4. Python packages from Ubuntu (present here already): numpy, scipy, matplotlib, pyyaml, psutil, pytest
   (`sudo apt install python3-numpy python3-scipy python3-matplotlib python3-yaml python3-psutil python3-pytest python3-pytest-cov`).
5. Only for RL training on the laptop:
   `python3 -m venv --system-site-packages .venv`,
   `.venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu`,
   `.venv/bin/pip install gymnasium stable-baselines3` (about 0.9 GB).
6. Optional map keys: copy `config/map_keys.example.yaml` to `config/map_keys.local.yaml` and paste
   your Mapbox and Cesium ion tokens (never commit that file).
7. Desktop icon: `bash scripts/install_launcher.sh`.

Check the environment: `scripts/env_audit.sh`. Before any PX4 run: `free -m` should show more than
800 MB available; kill leftovers with `pkill -x px4; pkill -x mavros_node`.

## 2. Every day

| Task | Command |
|---|---|
| Open the ground-control app | double-click **Swarm Control** in the project folder or on the desktop (or **start_swarm_control.sh**: right-click → Run as a Program), or `python3 scripts/gcs.py` and open http://localhost:8080 |
| Stop it (when started by a launcher) | `scripts/stop_swarm.sh` |
| Install the launchers (once per machine, and again after moving the folder) | `bash scripts/install_launcher.sh`: desktop and menu icon, and the **Swarm Control** program in the folder (needs `gcc`; the program is not in git, so a fresh copy of the repository needs this once) |
| Run all tests | `python3 -m pytest` (131 pass, 4 skip without ROS/PyTorch); with PyTorch: `PYTHONPATH=src .venv/bin/python -m pytest` (134 pass, 2 skip) |
| Save changes to GitHub (private repository `saad-rafeque/drone-swarm-failover`) | `git status`, then `git add <files>`, `git commit -m "<message>"` and `git push` (the laptop's deploy key is used automatically; see `docs/KNOWN_ISSUES.md`, Housekeeping) |

In the app: choose home and target (type or pick on the map), number of drones (1-100), obstacles
(none or real buildings), avoidance, flight height, formation (V; line, column and echelon are marked experimental); Start; change the speed; use the fault buttons
(kill leader, kill 2/3/4, per-drone crash / motor / battery / GPS / radio faults, radio split).
Routes up to 400 km; longer than a battery leg they get charging stops automatically.

The **3D view** page follows the same simulation: the drones are 3-D models (leader blue with a ring,
followers white, leaving amber, no radio purple, down red), with V-formation lines, trails, and a
line down to the ground under each drone. Cameras: **Chase** (behind the formation), **Orbit**
(circles it), **Top** (straight down) — in these three, drag to turn and use the wheel to zoom —
and **Free** (move the camera yourself). Switches for trails, V lines, labels, obstacles, 3-D
buildings, sun shadows and Google photorealistic 3-D (heavy). Fault buttons: kill the leader, kill 2
at random, leader battery low, cut the leader's radio, split the radio net, restore radios. Big
events (a drone down, a new leader and how many seconds after the fault) appear as a banner, and the
last events are listed at the top right.

## 3. PX4 simulation (ROS 2 environment through `scripts/ros_env.sh`)

| Phase | Command | Output |
|---|---|---|
| 0: one drone, 10 m hover | `scripts/ros_env.sh python3 scripts/phase0_flight_test.py` | `reports/logs/phase_0/` |
| 1: N drones hover | `scripts/ros_env.sh python3 scripts/phase1_scale_test.py --n 10` | `reports/logs/phase_1/n10/` |
| 3: formation mission | `scripts/ros_env.sh python3 scripts/run_mission.py --n 10 --run-dir reports/logs/phase_3/run1` | run folder: `states.jsonl`, `metrics.json`, ... |
| 3: acceptance batch (3 runs + 3 clean-shell cross-checks) | `bash scripts/phase3_runs.sh` | `reports/logs/phase_3/` |
| **Re-run the PX4 tests of Phases 4, 5 and 6** (all 95 finished on 29 September 2026; the queue skips finished trials, so move or delete a trial folder to run it again) | `python3 scripts/px4_queue.py start`, `stop [--now]`, `battery on\|off`, `status` | the trial folders in `reports/logs/phase_4/`, `phase_5/`, `phase_6/` (and `*_crosscheck/`); the queue's log `reports/logs/px4_queue/runner.log` |
| 4: fault trials without the queue, one round at a time (F1-F5 once, ~30 min; 10 drones; waits for the charger) | `bash scripts/phase4_runs.sh <r> <r>` for round r = 1..10, then the clean-shell cross-check round (`scripts/phase4_all.sh` does all of it in one go, ~7 h); then `python3 scripts/phase4_metrics.py --out ... --md ... reports/logs/phase_4/F?_t*` and `python3 scripts/plot_phase4.py` | `reports/logs/phase_4/`; how-to: `reports/PHASE_4.md` |
| 5: radio sweep on PX4 (9 conditions, one mission without a fault and one F1 mission each, ~16 min per condition) | `bash scripts/phase5_runs.sh <first> <last>` (conditions 1-9) | `reports/logs/phase_5/d<delay>_l<loss>_<none\|F1>` |
| 6: radio stand-in test on PX4 (F1 and F2 with drone 1 behind the stand-in, ~16 min per round) | `bash scripts/phase6_runs.sh <r> <r>` for round r = 1..10 | `reports/logs/phase_6/` |
| Check a drone profile (never opens a serial port) | `PYTHONPATH=src python3 scripts/check_profile.py config/profiles/mixed.yaml` | console |
| One mission with a profile, for example drone 1 behind the radio stand-in | `scripts/ros_env.sh python3 scripts/run_mission.py --n 10 --run-dir <dir> --profile config/profiles/standin.yaml` | run folder, `proc_logs/radio_standin_1.json` (link statistics); drone 1's PX4 log shows `mode: Minimal, data rate: 1200 B/s` (its radio link set-up) |
| Metrics / plots of a run | `python3 scripts/metrics.py <run_dir>`; `python3 scripts/plot_mission.py <run_dir> out.png` | |
| Replay page from a run | `python3 scripts/make_replay.py <run_dir> scripts/replay_template.html reports/replay/swarm_replay.html` | open in the app: PX4 flights |
| Logs as CSV | `PYTHONPATH=src python3 -c "from swarm_tools.logfmt import jsonl_to_csv; jsonl_to_csv('<run>/states.jsonl', '<run>/states.csv')"` | |
| Phase 0 / Phase 1 figures | `python3 scripts/plot_phase0.py`; `python3 scripts/plot_phase1.py` | `reports/phase0_altitude.png`, `reports/phase1_resources.png` |
| Diagnostic: CPU and memory of idle PX4 + MAVROS for several N (Phase 1) | `scripts/ros_env.sh python3 scripts/mavros_idle_scaling.py --ns 1 3 5` | console |
| Diagnostic: where 2 SIH drones spawn without home settings (Phase 1) | `scripts/ros_env.sh python3 scripts/spawn_default_check.py` | console |

Every PX4 drone listens on UDP 14580+n and sends to 14540+n (PX4's own port plan, which covers 10
instances); MAVROS runs in namespace `/uav<id>`. Serial connection URLs are refused by the tools.

## 4. Fast-simulator experiments (no ROS)

| Experiment | Command |
|---|---|
| 1,000 random fault runs (Phase 2) | `PYTHONPATH=src python3 scripts/random_trials.py --runs 1000 --workers 3` |
| Fault types F1-F5 | `PYTHONPATH=src python3 scripts/puresim_faults.py --n 10 --seeds 20` |
| Swarm size 1-100 | `PYTHONPATH=src python3 scripts/scale_test.py --ns 1 2 3 5 10 20 50 100 --seeds 1 2 3 --jsonl reports/logs/scaling/fastsim_scaling.jsonl` |
| One app mission without the browser, saved as JSON (events, progress, formation error, closest pair) | `PYTHONPATH=src python3 scripts/run_route.py --target LAT LON [--altitude auto\|low\|normal] [--avoider rl+shield] --out reports/logs/long_route/NAME.json` |
| The 12 km route (F-9 Park -> Rawalpindi Saddar), ~2 min each | `... scripts/run_route.py --target 33.5973 73.0479 --altitude auto --out reports/logs/long_route/rawalpindi_12km_auto.json`, and `--altitude low` |
| Islamabad -> Lahore to the end (272.55 km, 65 stops), ~20-25 min | `PYTHONPATH=src python3 scripts/run_route.py --target 31.5204 74.3587 --out reports/logs/long_route/lahore_full.json` |
| Replay part of a route and record every follower (for example the stop-5 episode) | `PYTHONPATH=src python3 scripts/route_window.py --target 31.5204 74.3587 --from 5000 --to 5300 --out reports/logs/long_route/lahore_full_stop5_detail.json` |
| Phase 5 radio sweep in the fast simulator (540 runs, ~14 min with 2 jobs) | `PYTHONPATH=src python3 scripts/radio_sweep.py --seeds 20 --jobs 2` |
| Formation shapes compared (160 runs, ~3 min) | `PYTHONPATH=src python3 scripts/formation_compare.py --seeds 10 --jobs 3` |

## 5. Obstacle avoidance and RL

| Step | Command | Time on this laptop |
|---|---|---|
| Tune the classical avoider (training scenarios) | `PYTHONPATH=src python3 scripts/rl_tune_apf.py` | ~15 min |
| Train the policy (CPU, PPO) | `PYTHONPATH=src .venv/bin/python scripts/rl_train.py --steps 6000000 --run reports/logs/rl/run1` | ~40 min |
| Learning curve | `python3 scripts/plot_rl_training.py reports/logs/rl/run1 --out reports/rl_training.png` | seconds |
| Fair comparison, 90 unseen courses | `PYTHONPATH=src python3 scripts/rl_eval.py --policy models/avoid_policy.npz --episodes 30 --jobs 3` | ~9 min |
| Real map, full agent code | `PYTHONPATH=src python3 scripts/rl_eval_route.py --policy models/avoid_policy.npz --seeds 5 --jobs 3` | ~8 min |
| Use a policy in the app | copy its `best_policy.npz` to `models/avoid_policy.npz` | |
| Long training on a GPU | step by step in `docs/KAGGLE_GUIDE.md` | ~11 h per seed on Kaggle (two seeds can run at once) |
| Learning curve of a Kaggle run | `python3 scripts/plot_rl_training.py reports/logs/rl/kaggle_seed1 --out reports/rl_training_kaggle_seed1.png` | seconds |

## 6. Documents

| Document | Command |
|---|---|
| Project brief page | `PYTHONPATH=src python3 scripts/make_brief.py` -> `reports/brief/swarm_failover.html` |
| Handover PDF | `PYTHONPATH=src python3 scripts/make_handover_pdf.py` -> `docs/HANDOVER.pdf` (needs Google Chrome) |
| Online copy of the project brief | a copy of `reports/brief/swarm_failover.html` was published once (26 September 2026) as a private online page in the owner's account; it is not updated automatically - republish it after regenerating the brief |
| Kaggle code bundle | `python3 scripts/make_kaggle_bundle.py` -> `kaggle/swarm-rl-code.zip` (never contains `config/map_keys.local.yaml`) |
| Kaggle guide alone as a PDF | `PYTHONPATH=src python3 scripts/make_handover_pdf.py --only kaggle --out docs/KAGGLE_GUIDE.pdf` |
| 3-D drone model of the 3D view | `python3 scripts/make_drone_model.py` -> `src/swarm_tools/gcs/static/drone.glb` |

## 7. When something goes wrong

| Symptom | Cause and fix |
|---|---|
| PX4 exits at start with a path error | the repository path has a space; PX4 must run from `/tmp/swarm_sim/...` (done by `sim_launch.py`) |
| A PX4 build uses all cores and the laptop freezes | always build with `scripts/build_px4.sh` (2 jobs) |
| Drones do not arm / MAVROS not connected | leftovers from an earlier run: `pkill -x px4; pkill -x mavros_node`, check `free -m` |
| "Downloading buildings ... map server busy" | the public OpenStreetMap (Overpass) servers are overloaded; wait or use the normal flight height (light query); downloads are cached in `data/osm/` |
| 3D view says "Cesium key needed" | paste the Cesium ion token into `config/map_keys.local.yaml` and reload |
| 3D view dark or blurred for a few seconds after opening | the terrain tiles are still loading; wait, the camera glides in on its own |
| 3D view slow, laptop fan loud | switch off Photorealistic and Sun shadows; close the 3D tab during long simulations |
| Double-clicking `start_swarm_control.sh` opens a text editor | right-click → Run as a Program, or use the **Swarm Control** program next to it |
| The desktop or menu icon does nothing after the folder was moved | the icon stores the folder's path: run `bash scripts/install_launcher.sh` again (the **Swarm Control** program inside the folder finds its own place and keeps working) |
| Satellite map is Esri, not Mapbox | no Mapbox token in `config/map_keys.local.yaml`, or the server was still starting (reload) |
| RL options greyed out in the app | `models/avoid_policy.npz` is missing |
| The app is slow with obstacles | expected: about 15x real time with 10 drones and the RL avoider; open sky is ~150x |
