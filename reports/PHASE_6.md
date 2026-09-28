# Phase 6 — Mixed-reality readiness (still software only)
Status: NOT COMPLETED — the software parts are done and checked. The PX4 stand-in test (the swarm must still
pass Phase 4 F1 and F2 with drone 1 behind a telemetry-radio stand-in) is ready to run in the PX4 test queue.

> **In short.** The first stand-in runs on 28 September failed: 3 of 4 missions did not reach the goal. The
> test set-up was wrong. The simulated autopilot sent drone 1's telemetry at the rate of a fast onboard
> link, 1.7 to 3.9 times what the radio carries. About 70 % of the messages were dropped, the rest arrived
> a second late, and drone 1 kept losing and taking back the lead.
>
> The radio drone is now set up like a real Pixhawk telemetry port, and its agent asks only for what fits.
> Two check flights then passed: one with the leader killed, one with its radio cut. In both, the goal was
> reached, there was exactly one real leader change, and never two leaders at once. Radio delay averaged
> 75 ms instead of 1 s, and nothing was dropped for lack of room. The 22 acceptance trials are next.

## Acceptance criteria
- [DONE] `config/profiles/`: `sim.yaml` (every drone simulated) and `mixed.yaml` (drone 1 real through a serial
  URL placeholder, drones 2-N simulated), plus `standin.yaml` (drone 1 simulated behind the radio stand-in).
  Checked by `scripts/check_profile.py`; the real drone is validated as text only (URL format, baud rate,
  system ID) and the serial port is never opened. `SimLauncher` refuses any profile with a real drone before
  anything starts. Evidence: `tests/test_profiles.py` (12 tests, including invalid files and the refusal).
- [DONE] Hardware stand-in: `src/swarm_tools/radio_proxy.py`, a UDP relay between one drone's PX4 and its
  MAVROS that limits bandwidth and adds delay and loss like a telemetry radio (SiK defaults, sources below).
  Evidence: `tests/test_radio_proxy.py` (6 tests with local sockets: rate, shared channel, transmit-turn wait,
  loss, full buffer, the stand-in URL).
- [PENDING] The stand-in test passes the Phase 4 F1 and F2 criteria: 10 trials each at N = 10 with
  `config/profiles/standin.yaml`, then a clean-shell cross-check round (`reports/logs/phase_6/`,
  `phase_6_crosscheck/`), run by the PX4 test queue.
  - First attempt: 1 of 4 reached the goal, with the wrong link set-up (below).
  - After the fix: 2 of 2 development checks passed.
- [DONE] `FLIGHT_TEST_PLAN.md`: `docs/FLIGHT_TEST_PLAN.md` - one real drone and N simulated ones, the PX4
  geofence, RC override on the real drone (stick movement returns control), the kill-switch procedure, a
  go/no-go checklist, abort criteria, a test sequence that builds up one step per flight.
- [DONE] `README.md`: setup, how to run each phase, results summary, known limitations
  (`docs/RUNBOOK.md` for every command, `docs/KNOWN_ISSUES.md` for the limitations).
- [PASS] Reproducible from a fresh copy with the README: the files git publishes, copied into an empty folder
  (1,093 files; no virtual environment, no map keys, no PX4 build), pass `python3 -m pytest` (176 passed,
  4 skipped: PyTorch and ROS 2 tests), `scripts/check_profile.py` works, and the ground-control app starts,
  serves every page and flies a mission. Evidence: `reports/logs/phase_6/fresh_clone_check.txt`. The PX4 part
  needs the installation in `docs/RUNBOOK.md`, section 1, which was not repeated on a new machine.
- [SKIPPED on request] Final phone notification: the owner asked not to use notifications.

## Key numbers
| Item | Value | Source |
|---|---|---|
| Stand-in usable rate | 32,000 bit/s shared by both directions | SiK AIR_SPEED 64 kbit/s, ECC on halves it |
| Transmit-turn wait | up to 131 ms when the channel is idle | SiK MAX_WINDOW default 131 |
| Loss in the air / buffer | 1 % / 1 s | design choices (`config/swarm.yaml`, `radio_standin`) |
| Measured by the tests | 2,000 bytes in 0.45-0.75 s; a 0.2 s buffer keeps 6-10 of 30 packets | `tests/test_radio_proxy.py` |

## First attempt, the cause, and the fix (28 September 2026)
**What happened.** The queue ran four stand-in missions at 20:47–21:33 (`reports/logs/phase_6_radio_saturated/`).
- Only F1_t1 reached the goal. F1_t2, F2_t1 and F2_t2 ran out of time (10 minutes).
- The lead passed back and forth between drones 1 and 2, up to 12 times in one flight, and in F2_t2 three drones
  were leader at once.

**Cause.** The stand-in's statistics (`proc_logs/radio_standin_1.json`) show the link was overloaded.
- The traffic offered was 6,700–15,600 B/s in the two directions together (PX4 alone 6,300–14,100 B/s),
  against 4,000 B/s that the radio carries, shared by both directions.
- 68–74 % of the messages were dropped because the buffer was full, and the rest arrived after about 1 s.
- Three things combined:
  1. PX4's simulator starts the offboard link at 4,000,000 B/s in "onboard" mode (`px4-rc.mavlink`), which
     streams attitude at 100 Hz and much more. A real Pixhawk's TELEM1 port is capped at 1,200 B/s
     (`MAV_0_RATE` default).
  2. The agent asked for position at 20 Hz.
  3. MAVROS synchronised clocks 10 times a second.
- Drone 1's own position data then often arrived more than 0.5 s apart. With `fcu_timeout_s` = 0.5 s its
  agent declared itself dead and stopped its heartbeats, so drone 2 took over. The next position brought
  drone 1 back, and as the lowest ID it took the lead again.

**Fix.** The radio drone is set up like a real drone on a telemetry radio (`config/swarm.yaml`, `radio_standin`):
- **PX4 link:** Minimal mode at 1,200 B/s (`MAV_0_MODE` 7, `MAV_0_RATE` 1200 on a real TELEM1), with the stream
  rates below. `src/swarm_tools/sim_launch.py`, `set_radio_link`, restarts the link through the PX4 client, and
  PX4's own log confirms it: `mode: Minimal, data rate: 1200 B/s on udp port 14580`.
- **Agent** (`--radio-link`, from `radio_ids` in `launch/swarm.launch.py`): position and velocity at 5 Hz, the
  landed state and battery at 1 Hz, and 1 s allowed without autopilot data (5 missed positions) instead of 0.5 s.
- **MAVROS:** time sync at 1 Hz instead of 10 Hz.
- **A config check:** the load is worked out from MAVLink frame sizes, and the config is refused if the planned
  traffic exceeds 80 % of the radio. It is 65 % now (`tests/test_config.py`).

**Check flights after the fix** (development runs, `reports/logs/phase_6/dev_radio_*`, fault seed 1):

| Run | Fault | Result | Leader changes | Most leaders at once | New leader after the fault (s) | Formation error mean (m) | Closest pair (m) | Goal |
|---|---|---|---|---|---|---|---|---|
| before the fix, F2_t1 | F2 | TIMEOUT | 2 | 2 | — | 2.64 | 8.37 | no |
| before the fix, F1_t1 | F1 | COMPLETED | 12 | 2 | — | 3.13 | 7.52 | yes |
| dev_radio_F2b | F2 | COMPLETED | 2 (take-off, then the fault) | 1 | 1.50 | 0.67 | 8.02 | yes |
| dev_radio_F1 | F1 | COMPLETED | 2 (take-off, then the fault) | 1 | 1.63 | 0.68 | 8.31 | yes |

After the fix, the formation was back under 2 m 5.13 s and 5.90 s after the fault (limit 15 s). New leader and
recovery come from `scripts/phase4_metrics.py`; the other columns come from each run's `metrics.json`.

Radio in dev_radio_F1:
- No message was dropped for lack of room: 0 of 3,114 (before the fix, 68–74 %).
- 1.4 % were lost in the air, against a model value of 1 %.
- Delay averaged 71–80 ms, at most 220 ms.

Notes on the check flights:
- dev_radio_F2 stopped before take-off: Minimal mode sends no local position until asked, and the mission start
  waits for it. The launcher now sets the radio stream rates when it sets up the link.
- dev_radio_F2b ran through a leftover relay from the attempt stopped at 21:37, so its radio numbers are mixed
  and not quoted (see the next section).

## Sources verified (URLs / commands)
- SiK radio defaults and behaviour (AIR_SPEED 64, SERIAL_SPEED 57, ECC 1 "the data rate you can support is
  halved", time-division turns, MAX_WINDOW 131): ardupilot.org, "SiK Radio - Advanced Configuration"
  (common-3dr-radio-advanced-configuration-and-technical-information).
- MAVROS connection URLs (`serial:///path/to/serial/device[:baudrate][?ids=sysid,compid]`, `serial-hwfc://`,
  `udp://[bind_host][:port]@[remote_host][:port]`): github.com/mavlink/mavros, `docs/connection_urls.md`;
  the schemes also in the installed `/opt/ros/jazzy/include/mavconn/interface.hpp`.
- PX4 parameters in the flight test plan (`GF_ACTION`, `GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`, `COM_OBL_RC_ACT`,
  `COM_OF_LOSS_T`, `NAV_RCL_ACT`, `COM_RC_LOSS_T`, `NAV_DLL_ACT`, `COM_DL_LOSS_T`, `BAT_LOW_THR`, `BAT_CRIT_THR`,
  `BAT_EMERGEN_THR`, `COM_LOW_BAT_ACT`, `RC_MAP_KILL_SW`, `MAN_OVERRIDE_SPD`, `RC_MAP_FLTMODE`, `MPC_XY_VEL_MAX`,
  `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN`, `RTL_RETURN_ALT`, `MAV_SYS_ID`; the TELEM1 baud parameter comes from
  the MAVLink serial configuration): their definitions in the PX4 v1.18.0-rc1 source, `src/modules/*/`
  `*params.yaml` and `src/lib/battery/module.yaml`.
- The PX4 offboard port plan the relay sits in (PX4 listens on 14580+i and sends to 14540+i):
  `px4-rc.mavlink`, as recorded in `src/swarm_tools/sim_launch.py`.
- The radio drone's link, from the installed PX4 v1.18.0-rc1 source:
  - `src/modules/mavlink/module.yaml`: `MAV_0_RATE` default 1200 B/s, "if the configured streams exceed the
    maximum rate, the sending rate of each stream is automatically decreased"; `MAV_0_MODE` values,
    7 = Minimal.
  - `mavlink_main.cpp`: `update_rate_mult`, the stream lists of each mode, and the `-m` strings (there is no
    "normal": Normal is used without `-m`).
  - `mavlink status streams` on a running instance: in Normal mode at 1,200 B/s, position arrives at about
    1 Hz.
  - The PX4 client's `--instance`: `platforms/posix/src/px4/common/main.cpp`.
- MAVLink message IDs and frame sizes (SET_POSITION_TARGET_LOCAL_NED 65 B, TIMESYNC 30 B, SYSTEM_TIME 24 B):
  `src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml` in the same PX4 tree.
- MAVROS time sync rate: `/opt/ros/jazzy/share/mavros/launch/px4_config.yaml` (`/**/time`, `timesync_rate: 10.0`).

## Problems and fixes
- The first stand-in tests counted packets the operating system had dropped (400 sent at once) and timed
  the second direction when it was read, not when it arrived; the tests now pace their sends and listen on
  both sockets at once. The relay itself did not change.
- A precaution, not an observed fault: the launcher waits (up to 10 s) until the relay's port is open before it
  starts that drone's MAVROS, so MAVROS never sends into a port nobody listens on.
- The radio overload of the first attempt: see the section above.
- **"Stop now" left the radio relay running.**
  - Cause: the queue stops a mission with SIGTERM. `run_mission.py` had no handler for it, so Python exited
    without its clean-up, and the relay runs in its own session.
  - Effect: the next mission's relay could not open its ports, and that mission ran through the old relay.
  - Fix, in three parts:
    - `run_mission.py` now runs its normal clean-up on SIGTERM.
    - The queue and the launcher's orphan clean-up also stop relays, matched by their exact module argument
      (`tests/test_px4_queue.py`).
    - The launcher refuses to start a relay whose ports are already in use.

## Known limitations / honest caveats
- The stand-in models the link's rate, transmit turns, loss and buffer, not the radio's framing,
  retransmissions or signal strength; real loss comes in bursts.
- There is no path in this repository that starts a real drone, on purpose; the flight test plan lists that
  as a precondition to be reviewed by the owner.
- **A weak point of the election, shown by the first attempt:** a drone that keeps dropping out and coming back
  takes the lead again each time, as the lowest ID. The spec's rule (lowest alive ID wins) was kept; the fix
  removes the cause here (the overloaded link). A rule that stops a returning drone from taking the lead at
  once would change `docs/SPECIFICATION.md` section 6, and Phases 2–5 would have to be repeated. It is noted in
  `docs/KNOWN_ISSUES.md` for the owner to decide.
- The flight test plan has not been used; its numbers marked *proposal* are starting values.

## Next phase: what is needed from the user
Press Start on the PX4 tests page with the charger in: the 22 stand-in trials take about 2.5 hours. Real
flights need the owner's review of `docs/FLIGHT_TEST_PLAN.md`, hardware and legal permission.
