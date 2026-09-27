# Swarm Failover

A drone swarm that keeps flying its mission when the leader fails, built and tested entirely in
simulation on one laptop. Every drone runs the same agent code: it elects a leader (lowest healthy ID,
with a term counter), holds its slot in a V formation, keeps 5 m from other drones, stays inside a
geofence, and takes over within about two seconds when the leader crashes, loses its radio or runs
low on battery. On top of that: obstacle avoidance among real buildings (OpenStreetMap), a learned
(reinforcement learning) avoider compared fairly with a classical one, city-to-city routes with
charging stops, and a ground-control web app with a 2-D map, a 3-D view, fault buttons, results and
these documents.

![The 3D view of the ground-control app: ten drones in V formation over F-9 Park, Islamabad](reports/gcs_3d_view.jpg)

> **Urdu mein khulasa.** Yeh project drones ka aik group (swarm) hai jo leader ke girne, radio katne
> ya battery khatam hone par bhi mission jari rakhta hai: agla drone ~2 second mein leader ban jata
> hai. Sab kuch laptop par simulation mein bana aur test hua hai — koi asli drone abhi nahi udaya gaya.
> Shuru karne ke liye project folder mein **Swarm Control** par double-click karein (ya Desktop par
> **Swarm Control** icon) — poora program khud chal jata hai aur browser mein khul jata hai.
> Kaggle par RL training ka poora tareeqa: `docs/KAGGLE.md` (ya `docs/KAGGLE_GUIDE.pdf`).
> Sab kuch aik file mein: `docs/HANDOVER.pdf`.

**Scope and safety.** Simulation only: no code here opens a serial port, talks to a real flight
controller or flashes firmware. Payloads, targeting and anything that engages objects are out of
scope (see `docs/SPECIFICATION.md`).

## Status (27 September 2026)

| Part | State | Evidence |
|---|---|---|
| Phase 0 — environment, one PX4 drone | passed | `reports/PHASE_0.md` |
| Phase 1 — 10 PX4 drones on this laptop | passed | `reports/PHASE_1.md` |
| Phase 2 — agent logic, 1,000 random fault runs | passed | `reports/PHASE_2.md` |
| Phase 3 — 10-drone PX4 formation flights | passed | `reports/PHASE_3.md` |
| Swarm size 1–100 drones (fast simulator) | passed | `reports/SCALING.md` |
| Obstacle avoidance, RL vs classical | first results | `docs/RESULTS.md`, `reports/logs/rl/` |
| City-to-city routes, charging stops | working in the fast simulator; 12 km route completed end to end | `docs/RESULTS.md`, `reports/logs/long_route/` |
| Islamabad → Lahore run (272.55 km, 65 charging stops) | **stopped on purpose** after 76 km: 18 stops used, 10/10 drones flying, 0 hits | `docs/RESULTS.md`, `reports/logs/long_route/` |
| Ground-control app (2-D map, 3-D view, faults, results, docs) | working | `docs/RUNBOOK.md` |
| Phase 4 — fault trials on PX4 | **started, stopped by the owner** (laptop too weak; indicative results and a full how-to) | `reports/PHASE_4.md` |
| Phase 5 — radio realism sweep | **not run** | `docs/KNOWN_ISSUES.md` |
| Phase 6 — mixed reality, flight test plan | **not done** | `docs/KNOWN_ISSUES.md` |
| Longer RL training on Kaggle | kit ready and checked, **not run** (step-by-step guide) | `docs/KAGGLE.md` |
| Real drones | **never flown** | — |

Tests: `python3 -m pytest` → 130 passed, 4 skipped (the skipped ones need ROS 2 or PyTorch); with PyTorch
(`PYTHONPATH=src .venv/bin/python -m pytest`) 133 passed, 2 skipped.

## Quick start
One click, any of these (each starts the ground-control app if it is not running and opens
http://localhost:8080 in the browser):

- in this project folder: double-click **Swarm Control** (a small program built by
  `bash scripts/install_launcher.sh`; it is not in git, so a fresh copy needs that command once), or
  **Start Swarm Control.sh** (if the Files app opens it as text: right-click → Run as a Program);
- the **Swarm Control** icon on the desktop or in the application menu (also installed by
  `bash scripts/install_launcher.sh`).

Stop the app with `scripts/stop_swarm.sh`.

From a terminal:
```bash
python3 scripts/gcs.py            # then open http://localhost:8080
```
Pages: **Mission** (2-D map, mission setup, fault buttons), **3D view** (3-D drones over terrain and
3-D buildings, chase / orbit / top cameras, fault buttons),
**Results**, **PX4 flights** (replay of the PX4 runs), **Docs** (these documents and the handover PDF).
Satellite maps and the 3-D view need your own Mapbox and Cesium ion tokens in
`config/map_keys.local.yaml` (copy of `config/map_keys.example.yaml`; git-ignored).

Everything else (PX4 flights, tests, RL training, evaluations): `docs/RUNBOOK.md`.

## Repository map

| Path | What is there |
|---|---|
| `src/swarm_agent/` | The agent that would run on every drone: election, formation, safety, geometry, heartbeat codec, obstacles, route planner, avoiders, ROS 2 node. Pure Python + numpy (ROS only in `ros_node.py`). |
| `src/swarm_tools/` | Simulation and test tools: PX4/MAVROS launcher, link emulator, logger, fast simulator (`puresim.py`), obstacle simulator for RL (`obstacle_sim.py`, `torch_sim.py`), OpenStreetMap loader, ground-control app (`gcs/`). |
| `config/` | `swarm.yaml` (every tunable), MAVROS plugin list, map-key template. |
| `scripts/` | Entry points: flights, tests, evaluations, training, plots, launcher, PDF builder. |
| `tests/` | 135 tests (pytest; some need PyTorch or ROS 2 and are skipped without them). |
| `reports/` | Phase reports, plots and raw logs (`reports/logs/`). |
| `models/avoid_policy.npz` | The trained obstacle-avoidance policy (numpy weights). |
| `data/osm/` | Cached OpenStreetMap downloads ((c) OpenStreetMap contributors, ODbL). |
| `kaggle/` | Notebook for long RL training on Kaggle (the code zip is built by `scripts/make_kaggle_bundle.py`). |
| `Swarm Control`, `Start Swarm Control.sh` | One-click launchers for the app (see Quick start). |
| `docs/` | Architecture, runbook, results, decisions, known issues, Kaggle, handover PDF. |
| `docs/SPECIFICATION.md` | The original project brief: goals, hard rules, phases and acceptance criteria. |

## Documents
- `docs/HANDOVER.pdf` — everything below in one file, for handing the project over.
- `docs/ARCHITECTURE.md` — how it works.
- `docs/RUNBOOK.md` — how to install and run every part.
- `docs/RESULTS.md` — all results with numbers and where the evidence is.
- `docs/DECISIONS.md` — the important design decisions and why.
- `docs/KNOWN_ISSUES.md` — limitations, what is not done, next steps, road to real drones.
- `docs/KAGGLE.md` — long RL training on Kaggle, step by step: which files, how to start it, what
  gets trained, where to paste the results, how to change things (with a Roman Urdu version);
  also as `docs/KAGGLE_GUIDE.pdf`.

## Main versions
Ubuntu 24.04.5, ROS 2 Jazzy, PX4 v1.18.0-rc1 (`fca3df865a`, target `px4_sitl_sih`), MAVROS 2.15.1,
Python 3.12.3, numpy 1.26.4, scipy 1.11.4; RL in `.venv`: PyTorch 2.14.0 (CPU), Stable-Baselines3
2.9.0, Gymnasium 1.3.0. Laptop: Intel i3-1115G4 (2 cores, 4 threads), 8 GB RAM, no GPU.

## Credits
Map data (c) OpenStreetMap contributors (ODbL). Satellite imagery (c) Mapbox, (c) Maxar, or Esri
World Imagery. 3-D terrain, buildings and photorealistic tiles through Cesium ion. Every
result above comes from logged runs in `reports/`.
