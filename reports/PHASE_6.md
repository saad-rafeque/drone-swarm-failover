# Phase 6 — Mixed-reality readiness (still software only)
Status: NOT COMPLETED — the software parts are done and checked; the PX4 test of the radio stand-in (the
swarm must still pass Phase 4 F1 and F2 with drone 1 behind it) runs when the owner presses Start on the PX4
tests page of the ground-control app (`scripts/px4_queue.py`).

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
  `phase_6_crosscheck/`), run by the PX4 test queue after Phases 4 and 5.
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

## Problems and fixes
- The first stand-in tests counted packets the operating system had dropped (400 sent at once) and timed
  the second direction when it was read, not when it arrived; the tests now pace their sends and listen on
  both sockets at once. The relay itself did not change.
- A precaution, not an observed fault: the launcher waits (up to 10 s) until the relay's port is open before it
  starts that drone's MAVROS, so MAVROS never sends into a port nobody listens on.

## Known limitations / honest caveats
- The stand-in models the link's rate, transmit turns, loss and buffer, not the radio's framing,
  retransmissions or signal strength; real loss comes in bursts.
- There is no path in this repository that starts a real drone, on purpose; the flight test plan lists that
  as a precondition to be reviewed by the owner.
- The flight test plan has not been used; its numbers marked *proposal* are starting values.

## Next phase: what is needed from the user
Nothing for the software. The PX4 stand-in test runs in the PX4 test queue. Real flights need the owner's
review of `docs/FLIGHT_TEST_PLAN.md`, hardware and legal permission.
