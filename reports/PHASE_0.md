# Phase 0 — Environment audit and setup
Status: PASSED

## Acceptance criteria
- [PASS] One SIH drone + one MAVROS instance: script arms, takes off to 10 m, hovers 20 s, lands.
  Evidence: `scripts/phase0_flight_test.py`, run twice, both `"result": "PASS"`:
  - run 1: `reports/logs/phase_0/run1/flight_summary.json`, `flight_altitude.csv` (518 rows, 10 Hz)
  - cross-check run (after `pkill`, from a clean `env -i` shell):
    `reports/logs/phase_0/run2_crosscheck/flight_summary.json`, `flight_altitude.csv`
  - plot of altitude over time: `reports/phase0_altitude.png`
  - PX4 console confirms `Landing detected` / `Disarmed by landing` (`run*/px4_1.log`)
- [PASS] Peak RAM for 1 drone (PX4 + MAVROS) measured and recorded: **135.0 MB**
  (PX4 10.1 MB + MAVROS 124.9 MB), kernel peak RSS (`VmHWM` from `/proc/<pid>/status`),
  matching the 10 Hz sampled peak exactly. Evidence: `run2_crosscheck/flight_summary.json`
  (`peak_rss_vmhwm_mb`, `peak_rss_sampled_mb`). Run 1 (sampled only): 135.2 MB.

Phase 0 tasks:
- [DONE] Environment recorded: `reports/logs/phase_0/env_audit.txt`, `env_audit_crosscheck.txt`.
- [SKIPPED — user's request] ntfy test. The user said not to bother with phone notifications.
  `scripts/notify.sh` exits 0 silently because `~/.config/swarm-ntfy/topic` does not exist.
- [DONE] Installed PX4 toolchain (`ubuntu.sh --no-nuttx --no-sim-tools`), `ros-jazzy-mavros`
  2.15.1, GeographicLib datasets: `sudo_installs.log`, `geographiclib_datasets.txt`.
- [DONE] Built `px4_sitl_sih` with 2 jobs: `px4_build.log` (`BUILD_EXIT=0`, `[784/784]`).
- [DONE] Repo skeleton: `config/`, `src/swarm_agent/`, `src/swarm_tools/`, `tests/`, `scripts/`, `reports/`.

## Key numbers
| Item | Value | Source |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS, kernel 7.0.0-34-generic, x86_64 | env_audit.txt |
| ROS 2 | Jazzy (`ROS_DISTRO=jazzy`) | env_audit.txt |
| Python | 3.12.3 | env_audit.txt |
| RAM | 7721 MB total; 2357–3287 MB available during this phase (Chrome open, ~3.1 GB) | env_audit*.txt, flight_summary.json |
| CPU | Intel i3-1115G4, 2 cores / 4 threads | env_audit.txt |
| Free disk | 131 GB before, 128 GB after the PX4 build (need ≥ 15 GB) | env_audit.txt, env_audit_crosscheck.txt |
| PX4 | v1.18.0-rc1, target `px4_sitl_sih`, model `sihsim_quadx` | env_audit_crosscheck.txt |
| MAVROS | 2.15.1 (apt) | env_audit_crosscheck.txt |
| PX4 build (resumed run) | 2 min 12 s wall; largest single process 383,512 kB (≈ 393 MB); `ninja -j2` | px4_build.log |

| Flight test | Run 1 | Cross-check |
|---|---|---|
| MAVROS connected after launch | 1.3 s | 1.3 s |
| OFFBOARD + armed | 2.0 s | 1.1 s |
| Climb 0 → 10 m (±0.3 m) | 10.3 s | 10.3 s |
| Hover duration | 20.0 s | 20.0 s |
| Hover altitude mean / min / max | 10.018 / 9.723 / 10.116 m | 9.943 / 9.706 / 10.125 m |
| AUTO.LAND → disarmed | 20.7 s | 19.6 s |
| Final landed_state | 1 (ON_GROUND) | 1 (ON_GROUND) |
| Peak RAM PX4 + MAVROS | 135.2 MB (sampled) | 135.0 MB (VmHWM) |

## Sources verified (URLs / commands)
- Build target `px4_sitl_sih`, run `make px4_sitl_sih sihsim_quadx`, multi-instance
  `sitl_multiple_run.sh`, `PX4_HOME_LAT/LON/ALT`: https://docs.px4.io/main/en/sim_sih/
- `boards/px4/sitl/sih.px4board` exists at tag v1.18.0-rc1 and `main`, but **not** at
  v1.17.0 (latest stable) or v1.16.2 (GitHub raw checks returned 404 at those two tags).
- Ubuntu setup `ubuntu.sh [--no-nuttx] [--no-sim-tools]`, 24.04 supported:
  https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu. The script was read before
  running: the `dialout` group change sits inside the NuttX block (lines 126–197), so
  `--no-nuttx` skips it. A pip dry run showed 16 new packages and **no upgrades** (numpy stays 1.26.4).
- MAVROS install + `install_geographiclib_datasets.sh` + `px4.launch`:
  https://github.com/mavlink/mavros/blob/ros2/mavros/README.md;
  `/opt/ros/jazzy/share/mavros/launch/{px4.launch,node.launch,px4_config.yaml,px4_pluginlists.yaml}`.
- Offboard: setpoints must stream ≥ 2 Hz before switching into OFFBOARD and while in it
  (`COM_OF_LOSS_T`, `COM_OBL_RC_ACT`): https://docs.px4.io/main/en/flight_modes/offboard
- Installed PX4 scripts (`build/px4_sitl_sih/etc/init.d-posix/`):
  - `px4-rc.mavlink`: offboard link of instance *n* listens on UDP 14580+*n* and sends to 14540+*n*
    (instances > 9 share 14549)
  - `rcS`: `MAV_SYS_ID = instance + 1`; `PX4_SIM_MODEL` selects the airframe by name
  - `px4-rc.sihsim`: `PX4_HOME_*` → `SIH_LOC_LAT0/LON0/H0`
  - `airframes/10040_sihsim_quadx`: SIH quad
  - `rcS`: `battery_simulator` starts when `SIM_BAT_DRAIN > 0` (for Phase 4 F3)
- PX4 Makefile lines 77–107: `-jN` is read from `ps T` (needs a terminal); `j=2` sets it explicitly.
- MAVROS interfaces seen on the running system: `ros2 topic list`, `ros2 service list`
  (`/uav1/mavros/state`, `local_position/pose`, `global_position/rel_alt`, `extended_state`,
  `setpoint_velocity/cmd_vel`, `cmd/arming`, `set_mode`), plus the message/service definitions
  (`mavros_interfaces.txt`, `/opt/ros/jazzy/share/mavros_msgs/msg/State.msg`).

## Problems and fixes
1. **The build target in docs/SPECIFICATION.md is missing from stable PX4.** `px4_sitl_sih` is absent from
   v1.17.0, so PX4 is pinned to **v1.18.0-rc1** (`config/swarm.yaml: sim.px4_version`).
2. **GeographicLib geoid and magnetic downloads failed.** The SourceForge mirror
   `yer.dl.sourceforge.net` failed its SSL handshake, and the MAVROS script hides errors and
   still exits 0. Fix: re-downloaded with the same tools (`geographiclib-get-geoids/-magnetic`,
   retry on a working mirror) into scratch space, then `sudo cp` into `/usr/share/GeographicLib`.
3. **The first build ran 6 parallel jobs**, breaking rule 6. PX4's Makefile only detects `-j2` from
   a terminal. It was stopped at 344/1160 and resumed with `make px4_sitl_sih j=2`
   (`scripts/build_px4.sh`); `ninja -j2` confirmed in `ps`.
4. **PX4's startup script breaks on paths with spaces.** The repo path contains one
   ("drone swarm") and the error was `no autostart file found`. PX4 working dirs now go to
   `sim.work_dir` (`/tmp/swarm_sim/<timestamp>/px4_<id>`): a fresh dir each run, never deleted
   by our tools. Console logs stay in the repo.
5. **MAVROS aborts on an empty parameter override** (`-p gcs_url:=`). The empty value is the
   default anyway, so the argument was removed.
6. **`ros2 interface show mavros_msgs/msg/State` printed ExtendedState's fields.** The
   authoritative `.msg` file and a live `ros2 topic echo` both show the correct State fields.
7. **`pkill -f px4` (rule 3a) also kills the shell that runs it**, because its own command line
   contains "px4". Replaced with exact-name matching (`pkill -x px4; pkill -x mavros_node`,
   and `kill_orphans()` in `src/swarm_tools/sim_launch.py`).

## Known limitations / honest caveats
- PX4 v1.18.0-rc1 is a release candidate. Real Pixhawk 6C drones should run the same version
  when Phase 6 mixed-reality tests happen.
- Two flight runs only; the evidence is for a single drone.
- After auto-disarm, `mavros/state.mode` reads `OFFBOARD` again, because the test still
  streams setpoints. The vehicle is disarmed on the ground; harmless, noted for Phase 3.
- The takeoff used OFFBOARD velocity setpoints (the pipeline Phases 3–6 need), not AUTO.TAKEOFF.
- PX4 writes `.ulg` flight logs into `/tmp/swarm_sim/...`; the OS cleans `/tmp`, our tools don't.
- CPU load was not measured in Phase 0 (not required); Phase 1 measures it.

## Next phase: what is needed from the user
Nothing. All sudo installs are done. For Phases 1 and 3–5, closing Chrome during multi-drone
runs frees about 3 GB of RAM.
