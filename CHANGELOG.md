# Changelog

The history of the project by milestone, newest first. The tags `phase-0` to `phase-3` mark the phase
gates of the plan in `docs/SPECIFICATION.md`; `v1.0-handover` marks the first complete handover. Commit IDs refer to
this repository's history.

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

