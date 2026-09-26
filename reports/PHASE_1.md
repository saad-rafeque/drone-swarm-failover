# Phase 1 — Multi-drone simulation scaling
Status: PASSED — max stable N on this laptop = **10** (the target), with a lean MAVROS plugin list

## Acceptance criteria
- [PASS] All N drones arm, take off to 10 m, hover 30 s, and land, with no crashes, for
  N = 3, 5 and 10. Each N ran twice: once, then again from a clean `env -i` shell after
  `pkill -x px4; pkill -x mavros_node`.
  Evidence: `reports/logs/phase_1/n{3,5,10}/summary.json` and `n{3,5,10}_crosscheck/summary.json`,
  all `"result": "PASS"`, with every criterion true:
  `all_armed_took_off_to_10m`, `hover_30s`, `all_landed_disarmed` (landed_state 1 for every
  drone), `no_process_crash` (no PX4/MAVROS process exited before shutdown),
  `distinct_spawn_positions`. Per-drone altitude traces: `n*/altitude.csv`.
- [PASS] Report of max stable N with RAM/CPU numbers: **N = 10**, 1,074 MB peak RAM for PX4 +
  MAVROS (107.4 MB per drone), 76.5 % mean system CPU during hover (cross-check run).
  N = 10 is also the most the PX4 SITL port plan supports unchanged
  (`px4-rc.mavlink`: instances > 9 share UDP port 14549).

Phase 1 tasks:
- [DONE] Launch script for N SIH + N MAVROS instances, each with unique ports and sysid:
  `src/swarm_tools/sim_launch.py`. PX4 instance *n* gets `-i n`; its offboard link is
  UDP 14580+*n* ↔ 14540+*n*; `MAV_SYS_ID` = *n*+1; MAVROS runs in namespace `/uav<id>` with
  `fcu_url udp://:14540+n@127.0.0.1:14580+n` and `tgt_system n+1`. Serial URLs are refused.
- [DONE] Verified spawn positions. Without `PX4_HOME_*`, two instances spawn at the same point
  (0.02 m apart, PX4 default home 47.3977427, 8.5455942): `spawn_default_check.txt`.
  Fix: each drone gets its own home via `PX4_HOME_LAT/LON/ALT`, the documented method
  (docs.px4.io/main/en/sim_sih/; mapped to `SIH_LOC_LAT0/LON0/H0` in `px4-rc.sihsim`).
  The layout is the V formation itself (`formation.initial_layout`, 10 m spacing, apex = lowest
  ID, facing the goal), so drones take off already in their slots. Measured against plan, from
  the GPS fix converted to the shared ENU frame: max error 0.19 m, min pair distance 9.75 m.
- [DONE] Scaled 3 → 5 → 10, measuring RAM (kernel VmHWM per process) and CPU (psutil, 1 Hz) at
  each step: `n*/resources.csv`. Plot: `reports/phase1_resources.png`.

## Key numbers
Idle laptop, no sim (Opera ≈ 31 % of a core, gnome-shell ≈ 12 %): **14.6 %** system CPU, 3,253 MB
available (`idle_baseline.txt`). System CPU is % of all 4 logical CPUs; per-process CPU is % of
one core. RAM is decimal MB.

| Run | MAVROS plugins | Result | RAM total (MB) | RAM / drone | System CPU hover mean / max | PX4 CPU (all) | MAVROS CPU (all) | Harness CPU | Min available RAM |
|---|---|---|---|---|---|---|---|---|---|
| n3_default_mavros | default | PASS | 513.7 | 171.2 | 44.1 / 46.7 % | 25.0 % | 84.7 % | 37.4 % | 2,867 MB |
| n5_default_mavros | default | PASS | 987.2 | 197.4 | **99.7 / 99.9 %** | 37.7 % | **281.4 %** | 42.8 % | 2,650 MB |
| n3 | lean | PASS | 288.1 | 96.0 | 25.0 / 42.0 % | 23.7 % | 23.0 % | 27.9 % | 3,609 MB |
| n5 | lean | PASS | 496.4 | 99.3 | 41.2 / 45.7 % | 38.9 % | 44.6 % | 52.1 % | 3,516 MB |
| n10 | lean | PASS | 1,070.9 | 107.1 | 76.1 / 77.2 % | 72.4 % | 116.0 % | 82.1 % | 3,203 MB |
| n3_crosscheck | lean | PASS | 287.6 | 95.9 | 24.9 / 26.3 % | 24.2 % | 22.8 % | 28.0 % | 3,766 MB |
| n5_crosscheck | lean | PASS | 496.3 | 99.3 | 41.0 / 46.0 % | 39.3 % | 45.0 % | 51.4 % | 3,522 MB |
| n10_crosscheck | lean | PASS | 1,074.2 | 107.4 | 76.5 / 77.4 % | 71.4 % | 121.8 % | 78.7 % | 3,477 MB |

| Run | Spawn error max | Spawn min pair | Hover min pair | Arm → all at 10 m | Land → all disarmed |
|---|---|---|---|---|---|
| n3 / n3_crosscheck | 0.10 / 0.11 m | 10.09 / 10.08 m | 9.18 / 9.13 m | 12.3 / 12.2 s | 22.4 / 20.3 s |
| n5 / n5_crosscheck | 0.19 / 0.19 m | 10.06 / 10.10 m | 9.00 / 8.94 m | 13.3 / 12.5 s | 23.5 / 21.2 s |
| n10 / n10_crosscheck | 0.17 / 0.16 m | 9.75 / 9.76 m | 8.32 / 9.05 m | 13.1 / 13.2 s | 23.6 / 25.0 s |

Idle MAVROS scaling (`idle_scaling_*.jsonl`; drones connected, on the ground, 15 s window):

| N | Default: MAVROS CPU per instance | Default: RSS per instance | Lean: MAVROS CPU per instance | Lean: RSS per instance |
|---|---|---|---|---|
| 1 | 17.8 % | 123.7 MB | 4.1 % | 81.9 MB |
| 3 | 36.6 % | 159.3 MB | 4.6 % | 85.2 MB |
| 5 | 65.7 % | 185.5 MB | 4.7 % | 88.3 MB |

## Sources verified (URLs / commands)
- Multi-instance SIH: `./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih` and
  manual `px4 -i <n> -d <build>/etc`: https://docs.px4.io/main/en/sim_sih/ and the script
  itself at v1.18.0-rc1 (one working dir per instance).
- Distinct home per instance, `PX4_HOME_LAT/LON/ALT`: https://docs.px4.io/main/en/sim_sih/;
  `build/px4_sitl_sih/etc/init.d-posix/px4-rc.sihsim`.
- Ports and sysid: `px4-rc.mavlink` (14580+n local, 14540+n remote, instances > 9 → 14549);
  `rcS` (`MAV_SYS_ID = px4_instance + 1`).
- MAVROS plugin names: `/opt/ros/jazzy/share/mavros/mavros_plugins.xml`. Allow/deny list
  parameters: the commented `plugin_allowlist` in `mavros/launch/px4_pluginlists.yaml`.
  With the lean list, every needed topic and service was checked on a running system:
  `lean_mavros_interfaces.txt`.
- Shared topics found with `ros2 topic list` / `ros2 topic info` (3 drones, default plugins):
  `/parameter_events` had 82 publishers and 84 subscribers; `/tf` and `/tf_static` had 3 and 3.
- Discovery range: installed `librcl.so` says "ROS_LOCALHOST_ONLY is deprecated … Use
  ROS_AUTOMATIC_DISCOVERY_RANGE", and `rmw/discovery_options.h` lists LOCALHOST / SUBNET / OFF.
  `scripts/ros_env.sh` sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
  docs.ros.org refused the automated fetch (bot-protection page), so the installed code was used.

## Problems and fixes
1. **Default MAVROS scaled O(N²)**, so 5 drones already saturated the CPU (99.7 %). Every MAVROS
   instance runs about 27 ROS nodes (one per plugin). Each publishes to and subscribes to the
   global `/parameter_events`, and the `param` plugin mirrors about 1,000 PX4 parameters into ROS
   parameters. Result: per-instance CPU climbed with N even while idle (17.8 → 36.6 → 65.7 %).
   Fix, still MAVROS (no architecture change):
   - `config/mavros_pluginlists.yaml` loads only the 6 plugins an agent needs
     (`sys_status, sys_time, command, local_position, global_position, setpoint_velocity`)
   - `/tf`, `/tf_static` and `/parameter_events` are remapped into each drone's namespace
   Per-instance CPU is now flat at 4.1–4.7 %, and N = 5 hover dropped to 41 % system CPU.
2. **PX4 default spawn is identical for all instances** (0.02 m apart). Fixed with per-drone
   `PX4_HOME_*` as documented (see above).
3. **rclpy prints "The following exception was never retrieved: cannot use Destroyable…"** at
   shutdown. These are fire-and-forget service calls still pending when the node is destroyed;
   cosmetic, after the test has finished.

## Known limitations / honest caveats
- **CPU is the constraint at N = 10**: 76.5 % of all 4 logical CPUs during hover. That includes
  the Python test harness (~80 % of one core) and about 15 % background (Opera, desktop).
  Phase 3 replaces the harness with 10 agent nodes, a link emulator and a logger. Their CPU is
  unmeasured yet and could push the laptop to saturation. Closing Opera/Chrome frees about 15 %.
- The 1-minute load average (12.8 mean at N = 10, up to 29 during startup) overstates pressure:
  MAVROS has 22 threads per instance, and load averages carry over between back-to-back runs.
  CPU % is the better measure.
- The lean plugin list drops `param`, `home_position`, `imu`, `rc_io` and others. PX4
  parameters for fault injection (e.g. `SIM_BAT_DRAIN`) will be set another way, to be verified
  in Phase 4.
- Spawn check uses SIH's simulated GPS (noise ≈ 0.1–0.2 m). Hover-time distances use the GPS
  fix horizontally and local z vertically; all homes share one ground altitude.
- Timing numbers are wall-clock; PX4 runs lockstep SIH. No sign of sim slowdown at N = 10
  (arm → all at 10 m took 13.1 s, vs 12.3 s at N = 3).

## Next phase: what is needed from the user
Nothing. If CPU becomes the limit in Phase 3 at N = 10, closing the browsers during runs is the
first option before reducing N.
