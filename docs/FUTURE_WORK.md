# Future work and recommendations

**Status (29 September 2026): the software phase is complete.** Phases 0 to 6 of `docs/SPECIFICATION.md`
passed (tags `phase-0` to `phase-6`), the Kaggle-trained obstacle-avoidance policy is in use, and everything is on
`main`. This note lists what can come next, what I recommend, and roughly how much work each item is. Effort
figures are estimates, not measurements.

**In short:**

- **One-click setup is possible.** Level 1 is done: `./setup.sh` installs everything the app, the fast
  simulator and the tests need, in about 3 minutes, and a GitHub check tests it on every push. The full PX4 +
  ROS 2 system can be one command with a container (Docker), or one click in the browser with GitHub
  Codespaces, in about 2–3 days.
- **Before going public:** choose a licence, scan the whole history for secrets, add a responsible-use note.
- **Swarm logic:** stop a drone with a patchy link from grabbing the lead back, and keep drones apart when they
  cannot hear each other.
- **RL:** the next run should change the training set-up, not just run longer.
- **Real drones:** hardware, a stable PX4 release, bench tests, then the flight test plan, only with legal
  permission.

## 1. One-click setup for anyone who copies the repository

**Can it be done? Yes, in layers.** Today a new person has to follow 7 manual steps in `docs/RUNBOOK.md`,
section 1. They install ROS 2 Jazzy and MAVROS, fetch the GeographicLib data, clone and build PX4, install the
Python packages, set up the RL environment, add map keys and install the desktop icon. Some of that needs `sudo`,
the PX4 build takes about 15 GB of disk, and one download (GeographicLib from SourceForge) failed on this laptop
and had to be done by hand.

Not everyone needs all of it. Most visitors only want to see the swarm fly in the app, which needs none of the
heavy parts. So the setup comes in three levels; **level 1 is built** (29 September 2026), levels 2 and 3 are
planned:

| # | What the person gets | How they start it | What they need | Their waiting time | Work to build it |
|---|---|---|---|---|---|
| 1 | **Quick start (done):** the ground-control app (2-D map, 3-D view, faults, results), the fast simulator with the real agent code, the RL policy, all tests | `./setup.sh`, then the app opens in the browser | Linux or macOS with Python 3.12 (Windows through WSL); no `sudo` | about 3 minutes (measured on a fresh copy) | done |
| 2 | **Full install script:** everything, including 10 PX4 drones with MAVROS | `bash scripts/install_all.sh` | Ubuntu 24.04, 15 GB free disk, 8 GB RAM, the `sudo` password once | about 1–1.5 hours (downloads and the PX4 build) | about 1–2 days, tested on a clean Ubuntu 24.04 virtual machine |
| 3 | **Container:** the same as level 2, identical on every machine | `docker compose up`, or the "Open in Codespaces" button on GitHub (nothing installed at all) | Docker, or a GitHub account (Codespaces has a monthly free allowance; 10 PX4 drones need a 4-core machine) | about 10–20 minutes to download a ready image | about 2–3 days |

**What each level would do:**
- **Level 1, `setup.sh` (built):**
  1. Checks Python 3.12.
  2. Creates `.venv` and installs `requirements.txt` (numpy, scipy, matplotlib, PyYAML, psutil, pytest; the
     tested versions are written in the file).
  3. Runs the tests.
  4. Creates an empty `config/map_keys.local.yaml` for the user's own keys.
  5. Starts the app on port 8080 and opens the browser.
  PyTorch is not needed here: the policy runs on plain numpy. Tested on a fresh copy on 29 September 2026: about
  3 minutes, all tests passed, every page of the app loaded.
- **Level 2, `install_all.sh`:**
  1. Check Ubuntu 24.04, free disk (at least 15 GB) and memory.
  2. Install ROS 2 Jazzy, MAVROS and the GeographicLib data. Keep a second download source, because
     SourceForge failed once.
  3. Clone PX4 at `v1.18.0-rc1` and build `px4_sitl_sih` with 2 jobs (`scripts/build_px4.sh`).
  4. Do everything level 1 does, plus the optional RL training environment.
  5. Install the desktop icon.
  6. Finish with `scripts/env_audit.sh` and a one-drone take-off and landing, the Phase 0 test.
  The script must be safe to run twice and must say clearly which step failed.
- **Level 3, container:**
  - A `Dockerfile` starting from the official ROS 2 Jazzy image, with MAVROS, the GeographicLib data and PX4 SIH
    built inside.
  - A `docker-compose.yml` that starts the app on port 8080.
  - A `.devcontainer/devcontainer.json` for Codespaces and VS Code.
  - Build the image once and publish it on GitHub's container registry, so users download it instead of building
    it for an hour.

**The automatic check (GitHub Actions) is added** for level 1: `.github/workflows/quick-start.yml` runs
`./setup.sh` on a fresh copy after every push to `main` and every pull request, and checks that the app starts.
When the container exists, add its build to the same check. Without such a check, setup scripts break quietly
as packages change.

**What cannot be made automatic:**
- **Map keys** (Mapbox, Cesium ion) belong to a person's own free account and must never be shipped. The 2-D map
  works without them (OpenStreetMap streets). Satellite imagery needs a Mapbox key, and the 3-D view needs a
  Cesium ion key. The setup can ask for them and write `config/map_keys.local.yaml`.
- **The `sudo` password** for a native install (level 2). Containers avoid it.
- **Kaggle training** runs on the user's own Kaggle account (`docs/KAGGLE_GUIDE.md`).
- **Computer limits:** 10 PX4 drones used about 1.1 GB of RAM and 77 % of this laptop's CPU. Weaker machines
  should start with 3 drones.

**Recommendation:** level 1 is done. Build level 3 next,
because a container is the only way to make the full PX4 system truly one step on any computer. Keep level 2 for
people who want a native install, for example on a drone's companion computer.

## 2. Before making the repository public

- **Licence.** Without one, nobody may legally reuse the code, even if it is public.
  - **Apache-2.0** is permissive and includes a patent grant. **MIT** is the shortest permissive licence. With
    **GPL-3.0**, copies that are shared must stay open.
  - For a research project that others should build on, Apache-2.0 or MIT is the usual choice.
  - PX4, MAVROS and ROS 2 are not part of this repository, so their licences do not change this choice.
  - The OpenStreetMap files in `data/osm/` fall under the ODbL; the README already gives the required credit.
- **Secrets.** `config/map_keys.local.yaml` has never been committed. Still, scan the whole history before
  publishing, for example with `gitleaks`, because a key committed once stays in the history after it is removed.
- **Personal details.** 394 tracked files, mostly raw logs, contain the laptop's home folder path with the user
  name. Commits already use the GitHub no-reply e-mail address. Keeping the paths is harmless; removing them means
  rewriting the logs.
- **Size.** The history is about 206 MB, mostly raw trial logs. That is fine for GitHub, but cloning is slower.
  Option: move the raw logs to a GitHub Release download and keep the summaries in the repository.
- **Responsible-use note.** Add a short section to the README:
  - The project is for civil research (search and rescue, inspection, education).
  - It is simulation-tested only.
  - Real flights need the flight test plan and legal permission.
  - It must not be used for weapons or harm.
- **Nice to have:** a `CONTRIBUTING.md`, issue templates, and a badge showing that the automatic check passes.

## 3. Swarm logic: recommended improvements

1. **Stop leader flapping.** Today, a drone whose own link keeps dropping in and out takes the lead back every time
   it returns, because the lowest alive ID wins. This was seen in the first Phase 6 attempt with an overloaded
   radio (`docs/KNOWN_ISSUES.md`).
   - **Recommendation:** a hold-down rule. A drone that was counted as dead must be heard steadily, for example
     for 3 s (15 heartbeats), before it may claim the lead again. Until then it flies as a follower.
   - The lowest-ID rule and the term counter stay as they are.
   - **Cost:** this changes `docs/SPECIFICATION.md`, section 6, so the tests must be repeated. That means the
     1,000 random runs of Phase 2 (minutes) and the PX4 trials of Phases 4 and 5 (about 8 hours with the queue).
     About 1 day of work plus the re-runs.
2. **Keep drones apart when they cannot hear each other.** In 143 of 1,000 random fast-simulator runs, two drones
   came closer than 5 m. Every one of those runs had a radio split or random link drops.
   - **Cheap first fix:** height layers during link loss. Each drone moves to its own height, for example 2 m
     apart by ID, while it has not heard its neighbours, so drones that cannot talk still cannot meet.
   - **Also:** "sticky" slots. A follower that loses the leader keeps its last slot and heading instead of drifting
     towards others.
   - **For real drones:** an onboard proximity sensor as the last layer.
   - About 1–2 days, then repeat the random runs and Phase 4 F2 and F5.
3. **Separation with the new RL policy.** On the Islamabad route, two drones once came to 4.77 m (limit 5 m).
   - Extend the brake so it also stops a drone that is closing in on a neighbour, not only on an obstacle. Or add
     a separation penalty to training (section 4).
   - Then re-run `scripts/rl_eval_route.py`. About half a day.
4. **Safer slot changes for the other shapes.** The line, column and echelon come closer than 5 m after faults, so
   they are experimental. Moving to a new slot on the transit layer, as the V does, should fix it. Re-run
   `scripts/formation_compare.py`. About 1 day.

## 4. Reinforcement learning: the next run

The two Kaggle runs (10.5 hours each) stopped improving after about 2.5 hours (`docs/RESULTS.md`). More hours with
the same settings will not help. Change the set-up instead:
- **Learning-rate decay:** lower the learning rate step by step to zero over the run, so the policy settles
  instead of going up and down.
- **More dense courses:** dense clutter is still the weak case, with 18 of 30 missions passed. Use a curriculum
  from sparse to dense, or simply a larger share of dense courses.
- **The brake inside training,** so the policy learns to work with it, and a **separation penalty** for coming
  closer than 5 m to a neighbour.
- **Real map areas:** train on patches of real OpenStreetMap neighbourhoods as well as random courses.
- **Three seeds** for a result others can trust. Because of the plateau, 5 hours per seed is enough: three seeds
  cost about 15 GPU hours, which fits in Kaggle's 30 free hours per week.
- **One lesson from this run:** the uploaded zip must contain `reports/logs/rl/apf_tuning.jsonl`, or Kaggle's
  classical comparison runs untuned. Always re-test on the laptop (`docs/KAGGLE_GUIDE.md`, section 9).
- **Choose the new policy by the same rules** (`docs/KAGGLE_GUIDE.md`, section 8), plus one more: closest pair at
  least 5 m on the real route.

Where RL should **not** be used: choosing the leader (it must be predictable and provable) and motor or attitude
control (PX4 does this well).

## 5. Road to real drones

1. **Hardware per drone:**
   - a Pixhawk 6C with GPS;
   - a companion computer (for example a Raspberry Pi 5) running Ubuntu 24.04, ROS 2 Jazzy, MAVROS and this
     agent, wired to the Pixhawk by serial cable;
   - a telemetry radio;
   - a radio or Wi-Fi mesh for the heartbeats between drones;
   - an RC receiver with a kill switch.
2. **A stable PX4 release.** PX4 is pinned to a release candidate, `v1.18.0-rc1`. Move to the stable release and
   repeat Phases 0–4 in simulation.
3. **A digital twin.** Measure the real drone (weight, battery, motors, a PX4 flight log) and put the numbers into
   the simulators, then repeat the tests.
4. **Bench test with the propellers off,** then one real drone with nine simulated ones. The set-up is
   `config/profiles/mixed.yaml`: only the connection address changes. Every step follows
   `docs/FLIGHT_TEST_PLAN.md`: geofence, RC override, kill switch, go/no-go checklist.
5. **Three real drones** in an open field with safety pilots; only then more.
6. **Legal permission first.** Check the current drone rules of the civil aviation authority (in Pakistan, the
   PCAA) and any local permission needed before buying or flying.

## 6. Keeping the project healthy

- **Versions:**
  - ROS 2 Jazzy and Ubuntu 24.04 are long-term releases, supported until about 2029.
  - Pin the Python packages in `requirements.txt` (section 1) so results can be repeated.
  - Stay on PX4 `v1.18.0-rc1` until the stable release has been tested.
- **Evidence habit:** keep the rule that every claim comes with a log, and re-run the fresh-clone check before
  every release.
- **Backups:** GitHub holds the code, history and tags. The Kaggle notebooks (`rafiique/swarm-rl-train-seed1`,
  `-seed2`) hold their training output. The chosen policy and both result sets are already in the repository.

## 7. What to do first

| # | Item | Why | Effort (estimate) | Who |
|---|---|---|---|---|
| 1 | Choose a licence; scan the history for secrets | nothing can be shared safely without these | 1 hour | owner decides, then a short task |
| 2 | Level 1 one-click setup + automatic check | anyone can try the project in minutes | **done** (29 September 2026) | — |
| 3 | Hold-down rule against leader flapping | the one known weak point of the election | about 1 day + re-runs (about 8 h of PX4 trials) | owner approves the spec change |
| 4 | Height layers when drones cannot hear each other | the main separation risk before real flights | 1–2 days + re-runs | development |
| 5 | Container and Codespaces (level 3) | the full PX4 system in one step on any computer | 2–3 days | development |
| 6 | Next RL run with a changed set-up, 3 seeds | better in dense clutter; results others can trust | 1 day of set-up + about 15 GPU hours | owner approves the quota |
| 7 | Real drones: hardware, stable PX4, bench test, flight plan | the goal of the project | weeks; needs hardware and permission | owner |
