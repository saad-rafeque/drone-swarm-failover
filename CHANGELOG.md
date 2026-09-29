# Changelog

The history of the project by milestone, newest first. The tags `phase-0` to `phase-6` mark the phase
gates of the plan in `docs/SPECIFICATION.md`; `v1.0-handover` marks the first complete handover. Commit IDs refer to
this repository's history.

## 29 September 2026

### Quick start in one command; future work
- `./setup.sh`: installs the Python packages (`requirements.txt`) into `.venv` without `sudo`, runs the tests,
  creates an empty map-key file and starts the app. Tested on a fresh copy: about 3 minutes, all tests passed.
- A GitHub check that runs the same setup on a fresh copy after every push and pull request. GitHub did not start
  it ("recent account payments have failed"), so every push gave a "startup failure"; it is paused in
  `ci/github-quick-start.yml` until the account's billing is fixed or the repository is public. The same steps pass
  on a fresh copy with an empty home folder, so without PX4 or ROS 2 (checked on the laptop).
- `docs/FUTURE_WORK.md` (and `.pdf`): recommendations with effort estimates: one-click setup in three levels,
  a checklist before going public, swarm-logic fixes, the next RL run, the road to real drones.

### Software phase complete: Phase 6 passed; the Kaggle policy in use
- **Phase 6 passed on PX4.** With drone 1 behind the telemetry-radio stand-in, 22 of 22 trials met every F1 and
  F2 limit: a new leader in 1.39–1.60 s, the formation back within 6.74 s, closest pair 7.65 m, goal 22 of 22;
  the radio dropped no packet for lack of room (`reports/PHASE_6.md`). All 95 PX4 trials of Phases 4-6 are done.
- **Kaggle RL training.** Two 10.5-hour T4 GPU runs (seeds 1 and 2, 5.5 and 5.8 billion steps). Seed 2's best
  policy replaces `models/avoid_policy.npz` (the laptop policy is kept as `models/avoid_policy_run1.npz`). On the
  90 unseen courses RL + brake completes 30, 23 and 18 of 30 (laptop policy 26, 18, 5; tuned classical 15, 7, 3),
  and 0 drones hit anything on the Islamabad route in 10 of 10 runs. Re-tested on the laptop with the tuned
  classical controller (`reports/logs/rl/eval_kaggle_check/`, `route_eval_kaggle_check/`); learning curves
  `reports/rl_training_kaggle_seed1.png`, `reports/rl_training_kaggle_seed2.png`.
- **The app's PX4 tests page is removed**, with its buttons and `/api/tests`, now that every trial has run. The
  queue stays for re-runs from the command line (`python3 scripts/px4_queue.py start`).
- The app's results page and the project page show the new policy's results.

## 28 September 2026

### Phases 4 and 5 passed on PX4; Phase 6 radio link fixed (evening)
- **Phase 4 passed.** 55 PX4 trials: 10 per fault and a clean-shell cross-check round. New leader in
  1.4–1.7 s, planned handover within 0.01 s, formation back within 6.6 s (13.4 s after a split), closest pair
  6.03 m, goal 55 of 55 (`reports/PHASE_4.md`, `reports/phase4_faults.png`).
- **Phase 5 passed on PX4.** 18 missions: no false leader change in any radio condition; a new leader in
  1.6–2.5 s even at 300 ms with 30 % loss (`reports/PHASE_5.md`).
- **Phase 6: the first stand-in runs failed** (3 of 4 did not reach the goal). The simulated autopilot flooded
  the radio stand-in; the failed runs are kept in `reports/logs/phase_6_radio_saturated/`.
  - The radio drone now gets a real telemetry port's set-up: PX4 Minimal mode at 1,200 B/s, the agent's
    `--radio-link` streams and a 1 s data timeout, MAVROS time sync at 1 Hz. A config check refuses traffic
    over 80 % of the radio.
  - Two check flights then passed (`reports/PHASE_6.md`).
- **Stop now left the radio relay running.** The mission script now cleans up on SIGTERM, the queue and the
  launcher stop leftover relays, and the launcher refuses busy relay ports.

### PX4 tests
- A test queue (`scripts/px4_queue.py`) for the remaining PX4 trials of Phases 4, 5 and 6, run one at a time
  with Start and Stop buttons on the new "PX4 tests" page of the ground-control app. Nothing starts by itself;
  running on battery is the owner's choice. Finished trials are never re-run; a trial stopped halfway runs again
  from its start; tooling failures are kept as evidence.
- The first 23 Phase 4 trials (rounds 1-4 and part of round 5, on the charger) met every limit: a new leader in
  under 1.7 s, the planned handover in under 0.01 s, the formation back within 6.6 s (13.4 s after a radio
  split heals), no two drones closer than 6.0 m, every goal reached.
- The owner set the F5 formation-recovery limit to 20 s (15 s for the other faults).

### Phase 5 and Phase 6
- Phase 5 in the fast simulator (`scripts/radio_sweep.py`, 540 runs): no false leader change at any radio
  condition; the Phase 4 limits hold in 8 of 9 conditions (`reports/PHASE_5.md`).
- Phase 6 software parts: drone profiles (`config/profiles/`) with `scripts/check_profile.py`; the telemetry-
  radio stand-in (`src/swarm_tools/radio_proxy.py`, SiK defaults); `docs/FLIGHT_TEST_PLAN.md`
  (`reports/PHASE_6.md`).

### Formations and routes
- Formation shapes line abreast, column and echelon, selectable in the app; compared with the V
  (`scripts/formation_compare.py`): only the V stays 5 m apart after faults, so the others are experimental.
- The Islamabad -> Lahore run flown to the end: all 10 drones landed after 18.1 simulated hours, 65 charging
  stops, no hits; one follower was held at a tall building for about 2 minutes and caught up
  (`scripts/route_window.py` records the episode).

## 27 September 2026

### Publication
- Published as a private GitHub repository. Added a README to every folder and this changelog;
  rewrote the project README for GitHub; gave every document a formal title.
- Renamed `Start Swarm Control.sh` to `start_swarm_control.sh` and `docs/KAGGLE.md` to
  `docs/KAGGLE_GUIDE.md`, so all files follow one naming style. The project specification is now
  `docs/SPECIFICATION.md`, and local tool settings are no longer part of the repository.
- The in-app Docs page also opens the Markdown files the README links to (`CHANGELOG.md` and the
  folder READMEs), and never any other file type; checked by `tests/test_gcs_backend.py`.
- Documentation in English only. Recorded the open housekeeping items: backup, licence, online brief
  and PX4 version (`94a9ed5`).

### Phase 4: failover under faults on PX4 (not completed)
- Tooling: the agents publish the moment they claim or hand over the lead; fault timing and formation
  recovery are measured more fairly; a 55-trial batch ends with a clean-shell cross-check (`8252d4c`).
- Power guard: the first attempt, on battery power, saturated the laptop and ran the battery flat.
  Trials now wait for the charger and a profile other than power saver (`94a1148`, `a31f1f0`).
- Stopped by the owner because the laptop cannot run the batch reliably. The indicative results (all
  takeovers within their limits; re-forming after a healed network split takes about 17 s against a
  15 s target) and the complete procedure for a stronger computer are in `reports/PHASE_4.md`
  (`c6b7605`).

### Handover (tags `phase-3` and `v1.0-handover`)
- 3-D view with drone models, smooth motion and chase, orbit and top cameras; complete Kaggle guide;
  a Kaggle kit that never packs the map keys and picks its best policy on separate validation courses;
  evidence from the long routes (`e2fd6bf`).
- `scripts/stop_swarm.sh` now reliably stops the ground-control app (`2730262`).

## 26 September 2026

### Routes, maps and the ground-control app
- The avoider runs at 10 Hz in the fast simulator, as in training; cheaper policy input and inference
  (`e32ea62`).
- Phase 3 closed; ground-control app with Results and Docs pages; one-click launcher; GPU training kit
  (`68c7071`).
- City-to-city routes: map download along a strip, route planning in chunks, charging stops, choice of
  flight height (`74a3dd5`).
- Mapbox satellite imagery on the mission map and the first Cesium 3-D view (`b51a387`).

### Obstacle avoidance and reinforcement learning
- First trained policy (PPO, 6 million steps), a fair evaluation on 90 unseen courses and on a real
  city map, and a GPU trainer for long runs (`07fbce7`).
- Kaggle kit for longer training with several seeds; learning-curve plot (`71f3674`).
- Buildings and trees as obstacles, an A* route for the leader, classical and learned avoiders and the
  training pipeline (`f6640d9`); routes too long for an obstacle map are refused before any download
  (`63ac895`).

### Scaling and ground control
- One-page project brief generated from the scaling logs (`367862b`).
- Scaling test from 1 to 100 drones in the fast simulator, with clean and harder radio (`a551e04`);
  swarms of up to 250 drones through a variable-length member list in the heartbeat (`2291af7`); map
  markers drawn in the right place (`98e925c`).
- Ground-control web app: live map, real GPS targets and fault buttons on the fast simulator
  (`b91f720`).

### Phases 0 to 3
- Phase 3 integration: ROS 2 agent node, link emulator, logger, 10-drone PX4 formation flights and a
  replay page (`bbac5c3`); the phase passed with the report in `reports/PHASE_3.md` (tag `phase-3`).
- Phase 2 (tag `phase-2`): pure-Python agent logic, point-mass simulator, unit tests and 1,000
  randomized fault runs (`73b9304`).
- Phase 1 (tag `phase-1`): PX4 SIH and MAVROS scaled to 10 drones on the laptop (`bb5af55`).
- Phase 0 (tag `phase-0`): environment audit, PX4 SIH and MAVROS installed, first single-drone flight
  (`b2ffdb9`).

