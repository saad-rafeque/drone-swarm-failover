# Reports and evidence

Every result in this project comes from a logged run. This folder holds the phase reports, the figures
made from the logs, and the raw logs themselves.

## Phase reports

| Report | Status | Contents |
|---|---|---|
| [PHASE_0.md](PHASE_0.md) | Passed | Environment audit and setup; one PX4 SIH drone with MAVROS arms, climbs to 10 m, hovers 20 s and lands; peak memory. |
| [PHASE_1.md](PHASE_1.md) | Passed | Scaling to 10 PX4 SIH and MAVROS instances; memory and CPU at 3, 5 and 10 drones; the lean MAVROS plugin list. |
| [PHASE_2.md](PHASE_2.md) | Passed | Pure-Python agent logic, unit tests and coverage, and 1,000 randomized fault runs. |
| [PHASE_3.md](PHASE_3.md) | Passed | ROS 2 integration: 10-drone PX4 formation flights, three acceptance runs and three clean-shell cross-checks. |
| [PHASE_4.md](PHASE_4.md) | Passed | Fault trials F1 to F5 on PX4: 55 trials, every limit met; the earlier attempts; how each number is measured. |
| [PHASE_5.md](PHASE_5.md) | Passed | Radio realism: delay 50-300 ms x loss 0-30 %, in the fast simulator (540 runs) and on PX4 (18 missions). |
| [PHASE_6.md](PHASE_6.md) | Software done; PX4 stand-in test fixed, acceptance trials pending | Drone profiles, the telemetry-radio stand-in, why the first stand-in runs failed and the fix, the flight test plan, a fresh-clone check. |
| [SCALING.md](SCALING.md) | Passed (fast simulator) | 1 to 100 drones: failover time, formation recovery, separation, heartbeat size and radio load. |

## Figures

| File | Shows |
|---|---|
| `gcs_3d_view.jpg` | The 3-D view of the ground-control app: ten drones in V formation over F-9 Park, Islamabad. |
| `phase0_altitude.png` | The first PX4 flight: arm, climb to 10 m, hover 20 s, land. |
| `phase1_resources.png` | Memory and CPU of PX4 and MAVROS for 3, 5 and 10 drones, with the default and the lean plugin list. |
| `phase3_mission.png` | Ten PX4 drones on the 1 km V mission: tracks, formation snapshots and formation error. |
| `phase4_faults.png` | Phase 4: time to a new leader for every trial against the limits, and formation error around each fault. |
| `phase4_on_battery.png` | Takeover times and formation error in the first Phase 4 attempt (on battery power, indicative only). |
| `phase5_radio_sweep.png` | Phase 5 in the fast simulator: false leader changes, time to a new leader and formation recovery for every radio condition. |
| `formation_shapes.png` | The four formation shapes seen from above, and how they compare after faults. |
| `rl_training.png` | Reinforcement-learning training on the laptop: return per episode and scores on the validation courses. |
| `logs/rl/eval/comparison.png` | The fair comparison of obstacle avoiders on 90 unseen courses. |

## Generated pages

- `brief/swarm_failover.html`: the one-page project brief (`scripts/make_brief.py`).
- `replay/swarm_replay.html`: an interactive replay of a logged PX4 mission and a failover demonstration
  (`scripts/make_replay.py`); the ground-control app shows it on its PX4 flights page.

## Raw logs

| Folder | Contents |
|---|---|
| `logs/phase_0/` | Environment audit and its clean-shell repeat, installation logs, PX4 build, the first flight (`run1`) and its cross-check (`run2_crosscheck`). |
| `logs/phase_1/` | Hover tests with 3, 5 and 10 drones (`n3`, `n5`, `n10`), their clean-shell cross-checks, and the runs with the default plugin list (`*_default_mavros`). |
| `logs/phase_2/` | The 1,000 randomized fault runs and their cross-check, fault types F1 to F5 in the fast simulator, test coverage, separation breakdown. |
| `logs/phase_3/` | Acceptance runs `run1` to `run3`, clean-shell cross-checks `crosscheck1` to `crosscheck3`, and the development runs before them (`dev_*`). |
| `logs/phase_4/` | The Phase 4 trials (`F<k>_t<round>`, rounds 1 to 10), run on 28 September 2026, and their table (`phase4_table.md`). |
| `logs/phase_4_crosscheck/` | The Phase 4 cross-check round 11, from a clean shell, and its table. |
| `logs/phase_4_incomplete/`, `logs/phase_5_incomplete/` | Attempts that stopped for a tooling reason (the mission watcher stalled), with `why.txt`; each trial then ran again. |
| `logs/px4_queue/` | The PX4 test queue's log (`runner.log`): every start, result and stop. |
| `logs/phase_4_stopped/` | The second Phase 4 attempt of 27 September 2026, stopped by the owner: trial `F1_t1` and the table of what was measured. |
| `logs/phase_4_on_battery/` | The first Phase 4 attempt on battery power: 12 trials, not used for acceptance (see the folder's README). |
| `logs/phase_5_fastsim/` | Phase 5 in the fast simulator: every run (`radio_sweep.jsonl`) and the summary. |
| `logs/phase_5/` | Phase 5 on PX4: one mission without a fault and one with the leader killed per radio condition (`d<delay>_l<loss>_{none,F1}`), and `phase5_px4_table.md`. |
| `logs/phase_6/` | Phase 6 on PX4: the stand-in trials (when they run), the check flights after the radio fix (`dev_radio_*`), `fresh_clone_check.txt`. |
| `logs/phase_6_radio_saturated/` | The first four Phase 6 runs, with the overloaded radio link (not used for acceptance). |
| `logs/formations/` | The formation-shape comparison: every run and the summary. |
| `logs/scaling/` | The 1 to 100 drone scaling test, with clean and with harder radio, and the machine description. |
| `logs/rl/` | Tuning of the classical avoider, training run `run1`, the fair comparison (`eval/`) and the real-map evaluations (`route_eval/`, `route_eval_10hz/`). |
| `logs/long_route/` | The 12 km route records, the full Islamabad to Lahore run (`lahore_full.json`, with `lahore_full_stop5_detail.json` for the stop-5 episode) and the earlier stopped run. |

### Naming conventions

- **Run folder** (one PX4 mission): `states.jsonl` holds every drone's state as recorded by the logger
  node (`states.jsonl.gz` when compressed) and `states.csv.gz` the same as a flat table; `metrics.json`
  holds formation error, separation and goal results; also `formation_rms.csv`, `link_events.jsonl`
  (link emulator actions), `fault_events.jsonl` (Phase 4 faults), `resources.csv` (CPU and memory once a
  second), `run_summary.json`, and `proc_logs/` with `px4_<id>.log`, `mavros_<id>.log` and
  `ros2_launch.log`.
- **`F<k>_t<n>`**: fault type F1 to F5, trial (round) n. `F<k>_t<n>.out` is the console output of
  that trial.
- **`crosscheck`** in a name: repeated from a fresh shell after every simulator process was stopped
  (rule 3a of `docs/SPECIFICATION.md`).
- **`dev_`**: development runs made before the acceptance runs, kept for transparency; a trailing
  letter marks a later attempt (`dev_n10b` follows `dev_n10`).

Log files are kept as the tools wrote them, except that a few local tool paths and process names were
blanked and quoted commit IDs point to the current history; paths inside them refer to the machine they
ran on.
