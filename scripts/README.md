# Scripts

Entry points for every experiment, test batch, figure and document. Run them from the repository root.
Scripts that talk to PX4 or ROS 2 run through `scripts/ros_env.sh`; the others need `PYTHONPATH=src`
when they import the project packages. The exact command lines and run times are in
[docs/RUNBOOK.md](../docs/RUNBOOK.md).

## Ground-control app

| Script | Purpose |
|---|---|
| `gcs.py` | Start the ground-control web app on http://localhost:8080. |
| `start_swarm.sh`, `stop_swarm.sh` | Start the app in the background and open the browser; stop it again. |
| `install_launcher.sh`, `launcher.c` | Build the **Swarm Control** program in the project folder (no desktop or app-menu icon). |
| `run_route.py` | Fly one ground-control mission without the browser and save its events, progress, formation error and closest pair as JSON. |

## PX4 simulation (Phases 0 to 6)

| Script | Purpose |
|---|---|
| `build_px4.sh` | Build PX4 SITL (SIH) with at most two parallel jobs. |
| `env_audit.sh` | Phase 0 environment audit; prints raw command output as evidence. |
| `ros_env.sh` | Run a command with ROS 2 Jazzy sourced and `src/` on `PYTHONPATH`. |
| `phase0_flight_test.py` | Phase 0 acceptance: one SIH drone with MAVROS arms, takes off to 10 m, hovers and lands. |
| `phase1_scale_test.py` | Phase 1 acceptance: N SIH drones with N MAVROS instances arm, take off to 10 m, hover 30 s and land. |
| `spawn_default_check.py` | Phase 1 check: where two SIH instances spawn when no home position is set. |
| `mavros_idle_scaling.py` | Phase 1 diagnostic: CPU and memory of each idle PX4 and MAVROS instance against the number of drones. |
| `run_mission.py` | One full swarm mission: PX4, MAVROS, agents, link emulator and logger, optionally with a fault. |
| `phase3_runs.sh` | Phase 3 acceptance batch: three missions with 10 drones, then three more from a clean shell. |
| `faults.py` | Phase 4 fault injection (F1 to F5), used by `run_mission.py --fault`. |
| `phase4_runs.sh` | Phase 4 trials, interleaved by round; skips finished trials and waits for the charger. |
| `phase4_all.sh` | The full Phase 4 batch: rounds 1 to 10 of every fault, then a cross-check round from a clean shell. |
| `px4_queue.py` | The PX4 test queue: the remaining Phase 4, 5 and 6 trials one at a time, started and stopped by the owner from the command line (`start`, `stop [--now]`, `battery on\|off`, `status`). All 95 trials finished on 29 September 2026. |
| `phase5_runs.sh` | Phase 5 on PX4: the radio sweep (delay 50/150/300 ms x loss 0/10/30 %), a mission without a fault and one with the leader killed per condition; resumable. |
| `phase6_runs.sh` | Phase 6 on PX4: F1 and F2 with drone 1 behind the telemetry-radio stand-in (`config/profiles/standin.yaml`); resumable. |
| `batch_power.sh` | Shared by the PX4 batches: wait for the charger and a power profile other than power saver. |
| `check_profile.py` | Check a drone profile (`config/profiles/*.yaml`) and print every drone's link; real drones are checked as text only. |

## Metrics and figures

| Script | Purpose |
|---|---|
| `metrics.py` | Metrics of one mission run: `states.jsonl` to `metrics.json`. |
| `phase4_metrics.py` | Phase 4 metrics per trial and per fault, as JSON and Markdown tables. |
| `plot_mission.py` | One run: tracks with V snapshots, the formation in the leader's frame, and formation error over time. |
| `plot_phase0.py`, `plot_phase1.py`, `plot_phase4.py` | The Phase 0, 1 and 4 figures in `reports/`. |
| `plot_rl_training.py` | The learning curve of a training run (laptop or GPU trainer). |
| `make_replay.py`, `replay_template.html` | The interactive replay page of a logged PX4 mission with a failover demonstration. |

## Fast-simulator experiments

| Script | Purpose |
|---|---|
| `random_trials.py` | Phase 2 acceptance: randomized runs with random kills, link drops and network splits. |
| `puresim_faults.py` | The fault types F1 to F5 in the fast simulator. |
| `scale_test.py` | The swarm-size test from 1 to 100 drones. |
| `radio_sweep.py` | Phase 5 in the fast simulator: delay x loss, false leader changes, takeover time and formation recovery, with a figure. |
| `formation_compare.py` | The formation shapes (V, line, column, echelon) compared under the same faults, with a figure. |

## Obstacle avoidance and reinforcement learning

| Script | Purpose |
|---|---|
| `rl_tune_apf.py` | Tune the classical potential-field avoider on training courses. |
| `rl_train.py` | Train the avoidance policy with PPO (Stable-Baselines3) on the CPU. |
| `rl_train_gpu.py` | PPO on a GPU with the batched simulator; used on Kaggle. |
| `build_pools.py` | Build the training, validation and held-out test courses for GPU training. |
| `rl_eval.py` | Fair comparison of the avoiders on held-out courses never used in training or model selection. |
| `rl_eval_route.py` | Full-stack comparison on a real OpenStreetMap route, optionally losing the leader halfway. |
| `make_kaggle_bundle.py` | Pack the code Kaggle needs into `kaggle/swarm-rl-code.zip`; refuses to pack any file containing a map key. |

## Documents and assets

| Script | Purpose |
|---|---|
| `make_handover_pdf.py` | Build `docs/HANDOVER.pdf`, and with `--only kaggle` the Kaggle guide PDF. |
| `make_brief.py`, `brief_template.html` | Build the one-page project brief in `reports/brief/`. |
| `make_drone_model.py` | Generate the quadcopter model of the 3-D view (`src/swarm_tools/gcs/static/drone.glb`). |
| `notify.sh` | Optional phone notifications through ntfy.sh; the topic is read from outside the repository. |
