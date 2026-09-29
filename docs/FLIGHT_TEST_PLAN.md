# Flight test plan: one real drone with simulated swarm members

**Status: not flown.** Nothing in this repository has flown on a real drone. This plan describes how the
first outdoor test should be prepared and run: one real Pixhawk 6C drone and simulated drones (mixed
reality, profile `config/profiles/mixed.yaml`). The safety pilot and the owner must review and sign it
before any flight. Every number marked *proposal* is a starting value for them to confirm, not a tested
limit.

## 1. Purpose and scope

- **Goal:** check that the swarm software controls a real drone the way it controls simulated ones:
  take-off, V formation, a short leg to a goal, leader failover, separation, landing.
- **Set-up:** the real drone is drone 1 or 2; the other drones are PX4 SIH instances on the ground
  computer. All agents, MAVROS and the link emulator run on the ground computer; the real drone's MAVROS
  talks to it over a SiK telemetry radio pair (the link the Phase 6 radio stand-in imitates).
- **Out of scope:** payloads, anything that engages objects, flights beyond visual line of sight, more
  than one real drone, flights near people or buildings.

## 2. Before a field day is planned

All of these must be true; the evidence is in `reports/` and `docs/`.

| Precondition | Where it is recorded |
|---|---|
| Phase 4 (fault trials on PX4) passed | `reports/PHASE_4.md` |
| Phase 5 (radio realism) passed, including the radio settings that will be used | `reports/PHASE_5.md` |
| Phase 6 radio stand-in test passed (F1 and F2 with drone 1 behind the stand-in) | `reports/PHASE_6.md` |
| PX4 on the real drone is a stable release (not the v1.18.0-rc1 used in simulation), and the simulation tests were re-run with that same release | `docs/KNOWN_ISSUES.md`, road to real drones |
| A reviewed way to start the real drone exists. The simulation tools in this repository refuse any profile with a real drone, on purpose | `src/swarm_tools/sim_launch.py`, `src/swarm_tools/profiles.py` |
| The same mission, with the same configuration and profile, passed in simulation on the day before | a simulation run folder under `reports/logs/` |
| Written permission: the national civil aviation authority (in Pakistan, the Pakistan Civil Aviation Authority), any local authority, and the landowner | copies kept with the test records |

## 3. Team and roles

At least three people. Each role has one person; nobody holds two roles during a flight.

| Role | Duties |
|---|---|
| **Test director** | Runs the go/no-go poll, reads the test card, calls each step, decides to continue or stop. |
| **Safety pilot** | Holds the RC transmitter of the real drone for the whole flight, watches the drone (not a screen), takes control or uses the kill switch. Has the final word on safety. |
| **Ground station operator** | Starts and watches the swarm software and QGroundControl, announces leader changes, battery and link quality, triggers only the planned faults. |
| **Observer** | Watches the sky and the ground around the area; calls "abort" when a person, vehicle or aircraft comes near. |

Standard callouts, repeated back by the person addressed: **"Taking control"** (safety pilot), **"Abort"**
(anyone), **"Kill"** (safety pilot only).

## 4. Site, airspace and weather

- An open, flat field: no buildings, trees, power lines or roads inside the flight area, and nobody but the
  team within 50 m beyond the geofence (*proposal*).
- Outside controlled or restricted airspace; below the maximum height the permission allows.
- The whole flight in visual line of sight of the safety pilot.
- Weather: wind below 6 m/s (*proposal*, and within the drone's own limit), no rain, good visibility.

## 5. Configuration

### 5.1 Swarm software (ground computer)

| Setting | Value for the test | Where |
|---|---|---|
| Shared-frame origin | the take-off point in the field | `config/swarm.yaml`, `origin` |
| Geofence | radius 150 m, ceiling 30 m (*proposal*), smaller than the drone's own PX4 geofence | `config/swarm.yaml`, `safety` |
| Cruise height and speed | 20 m, 3 m/s (*proposal*) | `config/swarm.yaml`, `mission` |
| Goal | about 100 m from home, inside the geofence | `config/swarm.yaml`, `mission` |
| Real drone | its swarm ID, the ground radio's serial device and baud rate (57600), `mav_sys_id` equal to its ID; check with `scripts/check_profile.py` | `config/profiles/mixed.yaml` |
| Telemetry radio | the same settings as in the Phase 6 stand-in test (air rate, error correction, MAX_WINDOW) | the radio's own settings; `config/swarm.yaml`, `radio_standin` |
| The real drone's agent | started with `--radio-link`, so it asks PX4 for position at 5 Hz and allows 1 s without autopilot data, as in the Phase 6 stand-in test | `launch/swarm.launch.py` `radio_ids:=<id>`; `config/swarm.yaml`, `radio_standin` |
| MAVROS of the real drone | time sync at 1 Hz instead of 10 Hz (`/**/time` `timesync_rate`) | `src/swarm_tools/sim_launch.py`, `mavros_radio_params` |

### 5.2 The real drone (PX4 parameters)

Parameter names were checked in the PX4 v1.18.0-rc1 source; check them again in the stable release that
is flown. Export the full parameter file after setting them and keep it with the test records.

| Parameter | Setting | Why |
|---|---|---|
| `MAV_SYS_ID` | the drone's swarm ID | the swarm software addresses the drone by it |
| `SER_TEL1_BAUD` | 57600 | the radio's serial speed (SiK default `SERIAL_SPEED` 57) |
| `MAV_0_MODE` | 7 (Minimal) | the set of messages PX4 sends over the radio; the default 0 (Normal) sends many the swarm does not use, and at 1200 B/s leaves position at about 1 Hz |
| `MAV_0_RATE` | 1200 B/s (default) | PX4's cap on the radio link; it lowers every message rate to stay under it. With 20 Hz setpoints, the swarm's radio traffic plans for 65 % of the radio (`config/swarm.yaml`, `radio_standin`) |
| `GF_ACTION` | 3 (Return) | the autopilot's own geofence, independent of the swarm software (default 2, Hold) |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` | 200 m / 40 m (*proposal*) | larger than the software geofence; the default 0 means off |
| `COM_OBL_RC_ACT` | 0 (Position mode, default) | what the drone does when the offboard setpoint stream stops |
| `COM_OF_LOSS_T` | 1.0 s (default) | how long the stream may stop before that |
| `NAV_RCL_ACT` / `COM_RC_LOSS_T` | 2 (Return, default) / 0.5 s (default) | loss of the RC transmitter |
| `NAV_DLL_ACT` / `COM_DL_LOSS_T` | 3 (Land) / 5 s (*proposal*) | loss of the ground link; the default 0 does nothing |
| `COM_LOW_BAT_ACT` | 3 (Return at critical, land at emergency) | the default 0 only warns |
| `BAT_LOW_THR` / `BAT_CRIT_THR` / `BAT_EMERGEN_THR` | 0.30 / 0.20 / 0.10 (*proposal*; defaults 0.15 / 0.07 / 0.05) | a larger margin for test flights |
| `RC_MAP_KILL_SW` | a dedicated two-position switch | the kill switch |
| `MAN_OVERRIDE_SPD` | 1 (default; never negative) | moving the sticks gives control back to the pilot in Position mode |
| `RC_MAP_FLTMODE` | Position, Land and Return on the mode switch | quick manual modes for the safety pilot |
| `MPC_XY_VEL_MAX` / `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN` | 5 / 2 / 1.5 m/s (*proposal*; defaults 12 / 3 / 1.5) | slower flight leaves more time to react |
| `RTL_RETURN_ALT` | 25 m (*proposal*; default 60 m is above the test ceiling) | Return stays inside the permitted height |

## 6. Go / no-go checklist (every flight)

The test director reads each line; the named person answers "check". Any "no" means no flight.

1. Permissions and this plan signed and on site. *(test director)*
2. Area clear, observer in position, everyone knows the callouts. *(observer)*
3. Wind and visibility within limits. *(safety pilot)*
4. Flight battery, RC transmitter and ground computer charged; spares ready. *(safety pilot)*
5. Propellers, frame, motors and battery mount inspected. *(safety pilot)*
6. GPS 3-D fix, no pre-arm errors in QGroundControl. *(ground station operator)*
7. Parameters match section 5.2 (compare the exported file). *(ground station operator)*
8. Kill switch tested **with propellers off**: armed, switch on, motors stop and stay stopped. *(safety pilot)*
9. Stick override tested (propellers off, offboard mode): moving the sticks switches to Position mode. *(safety pilot)*
10. Radio link: signal strength and loss shown as good in QGroundControl. *(ground station operator)*
11. Software: the git commit or tag is written on the test card, the tests pass, `scripts/check_profile.py`
    says the profile is valid, logging is on (PX4 ULog and the swarm logs). *(ground station operator)*
12. Everyone says "go". *(test director)*

## 7. Test sequence

One step per flight. A step is repeated until it passes; the next step starts only after that.

| Step | What flies | Pass when |
|---|---|---|
| 1. Bench, propellers off | connect, arm and disarm from the software, enter and leave offboard mode, kill switch, stick override | every action works on the first try |
| 2. Manual flight | the safety pilot hovers, flies Position mode, triggers Return and Land | normal handling, no warnings |
| 3. Real drone alone | the swarm software takes off to 10 m, hovers 30 s, lands (like Phase 0) | height error under 1 m, clean landing |
| 4. Real drone as leader | drone 1 real with simulated followers; the short V leg to the goal and back | the V forms around it, no warnings, landing at home |
| 5. Real drone as follower | a profile with the real drone as drone 2; simulated leader | it holds its slot within 2 m during cruise |
| 6. Planned faults | only faults that keep the real drone's motors running: the simulated leader is killed (the real drone may take over); the real leader's battery handover (it hands over and returns home); the real leader's heartbeats are cut in the link emulator (the others elect a new leader; the real drone hears it, steps down and follows) | the behaviour matches the Phase 4 results |

The real drone's motors are never stopped as a "fault": killing its processes or cutting its RC link in
flight is not part of this plan.

## 8. Abort criteria

Any one of these ends the flight at once:

- anyone calls "Abort";
- a person, vehicle or aircraft comes near the area;
- the real drone leaves the planned area, flies higher than planned, or moves in a way nobody expected
  (oscillation, drifting away, a wrong direction);
- the software shows two leaders, or no leader, for more than 3 s;
- the real drone comes within 5 m of any other drone (real or simulated) or of any object;
- the radio link drops for more than 1 s, GPS degrades, or PX4 reports an estimator error;
- the battery reaches the planned landing level, or the wind rises above the limit.

**How to abort, in this order:**
1. The safety pilot calls "Taking control", moves the sticks (the drone switches to Position mode) and lands.
2. If the drone does not respond: switch to Land, or to Return if landing in place is not safe.
3. The kill switch only as a last resort (section 9).

## 9. Kill-switch procedure

- **When:** only when stopping the motors in the air is safer than any other action, for example a flyaway
  toward people, or a fire.
- **How:** the safety pilot calls "Kill", sets the kill switch and keeps it set. The motors stop and the
  drone falls.
- **After:** wait until the propellers have stopped; approach only when told by the safety pilot; watch the
  battery for smoke or swelling before touching it; disconnect it; save the logs; record what happened.

## 10. After each flight

- Download the PX4 ULog and the swarm logs; note battery voltage and flight time.
- Compare the flight with the same mission in simulation: formation error, leader changes, separation.
- Debrief: what worked, what did not, anything unexpected. Decide go or no-go for the next step.

## 11. Test card (one per flight)

Date and time; site; team and roles; weather; git commit or tag; profile; exported parameter file; step;
result (pass or fail); aborts and why; log file names; signatures of the test director and the safety pilot.
