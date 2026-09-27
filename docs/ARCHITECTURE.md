# Architecture

This document explains the system from the drone outwards. File names point at the code; every
tunable number lives in `config/swarm.yaml`.

## 1. One drone

```
            radio (heartbeats, 5 per second, to every other drone)
                 ^                                   |
                 |                                   v
   +-------------------------------------------------------------+
   | swarm agent  (src/swarm_agent, one per drone, no central    |
   |  controller)  election . mission . formation . avoidance    |
   |               . safety layer . heartbeat codec              |
   +-------------------------------------------------------------+
                 | velocity setpoints, 20 Hz      ^ position, velocity,
                 v (arm / OFFBOARD / LAND)        | battery, state
   +-------------------------------------------------------------+
   | MAVROS  (MAVLink bridge, ROS 2)                             |
   +-------------------------------------------------------------+
                 | MAVLink over UDP (simulation) or serial (real) |
   +-------------------------------------------------------------+
   | PX4 autopilot  (SIH simulator now, Pixhawk 6C later)        |
   +-------------------------------------------------------------+
```

- The agent logic is plain Python (`agent_core.py` and the modules it uses). The same class runs
  inside the ROS 2 node (`ros_node.py`, one per drone in namespace `/uavN`) and inside the fast
  simulator (`swarm_tools/puresim.py`).
- MAVROS runs with a lean plugin list (`config/mavros_pluginlists.yaml`: sys_status, sys_time,
  command, local_position, global_position, setpoint_velocity) and per-namespace remaps of `/tf`,
  `/tf_static` and `/parameter_events`; the default plugin set made CPU grow with the square of the
  drone count (Phase 1).
- Moving a drone to real hardware changes its MAVROS connection URL (UDP -> serial) and replaces
  the radio emulator by a real radio link. The agent code stays the same.

## 2. Positions: one shared frame
Every drone converts its own latitude/longitude/altitude to one shared East-North-Up frame anchored at
`config.origin` (WGS-84 -> ECEF -> ENU, `geometry.EnuFrame`). Drones never compare their MAVROS local
frames, which each start at their own power-on point. On long routes the ground falls below the
tangent plane (about 5.7 km at 270 km), so positions are converted back with
`EnuFrame.surface_point`, which puts them on the ground (without it they would be ~240 m off).

## 3. Heartbeat
Each drone broadcasts a heartbeat 5 times a second (`heartbeat.py`, codec version 2): id, role,
term, mission phase, leader id, handover target, flags (MASTER_OK, READY, ELIGIBLE, ORPHAN,
AIRBORNE), battery, position, velocity, formation heading, time stamp, and - from the leader - the
list of members it hears as a bit mask. A fixed 43-byte header plus only as many mask bytes as the
highest ID needs: 45 bytes for 10 drones, 56 for 100, up to ID 250 (PX4's `MAV_SYS_ID` limit). In
simulation the heartbeats go through `link_emulator.py`, which can add delay, jitter, loss, cut a
drone's transmitter or split the swarm into groups.

## 4. Choosing the leader (`election.py`)
- The lowest-numbered alive and eligible drone leads. Eligible: position valid, battery above the
  handover level (30 %), not retired.
- Every leader claim increases a `term` number. A leader that hears another leader with a higher
  term (or the same term and a lower ID) steps down, so after a radio split heals there is exactly
  one leader again.
- If nobody hears the leader for 1.5 s, the next eligible drone claims - but only if no other drone
  still hears a leader (MASTER_OK flag) and no lower eligible ID is alive. No pre-emption: a
  recovered lower ID does not take the job back.
- At startup drones listen first and the first election waits until the whole fleet is heard (or
  a timeout), so the intended leader is not beaten by a drone that booted earlier.
- A leader at 30 % battery names a successor (planned handover), leaves the formation and flies home.
- While no leader is known, drones hover.

## 5. Mission (`agent_core.py`)
The leader runs the mission and every follower copies its phase from the heartbeat:
IDLE -> TAKEOFF (whole fleet ready) -> CRUISE -> HOLD (10 s at the goal) -> LAND.
On long routes: at each charging stop LAND -> CHARGE (on the ground until every battery is ~95 %)
-> TAKEOFF (vertically where each drone stands) -> CRUISE again.
In open sky the leader flies a straight leg to the goal (fixed heading plus a cross-track
correction) with a speed ramp; with obstacles it follows the planned route (pure pursuit), turning
the formation heading at most 20 degrees per second.

## 6. Formation (`formation.py`)
- V formation, 10 m between neighbours, 45 degree arms. Even IDs fly the left arm, odd IDs the right,
  in ID order ("parity arms"), so losing a drone only slides the drones behind it on the same arm.
- Follower command = leader velocity (feed-forward) + P-control toward its slot, saturated at 3 m/s.
- The leader's heartbeat lists its members; a drone the leader cannot hear is an "orphan" and flies
  8 m higher behind its arm. Moves longer than 12 m happen 6 m below the formation ("transit
  layer") so crossing paths are vertically separated.

## 7. Safety layer (every command, every drone)
- Repulsion from any drone closer than 7.5 m (1.5 x the 5 m minimum), including drones that just
  went silent ("ghosts", extrapolated for 3 s).
- Geofence: no motion outward beyond radius - margin, capped altitude.
- Emergencies: GPS lost or battery at 10 % -> hand over, drop below the formation while keeping its
  speed (so the drones behind pass over it), land where it is. On long routes a low battery also
  lands in place (home may be far behind).

## 8. Obstacles and avoidance
- `obstacles.py`: 2-D obstacle map at flight height: building footprints (polygons) and trees
  (circles) with a grid index; 24-ray range "sensor", nearest obstacle point, clearance, occupancy.
- `planner.py`: the leader's route by A* on an inflated grid (8 m clearance) with line-of-sight
  shortcuts; long routes chunk by chunk; charging stops in open spots about every half battery.
- `avoidance.py`: followers add a horizontal correction from an avoider, then the safety layer:
  - none;
  - classical: potential field (repulsion from the nearest obstacle point, sliding, a tangential
    push) plus a stopping-distance brake, tuned on training scenarios;
  - learned: a small neural network (2 x 128, tanh) trained with PPO, run with numpy only;
  - learned + brake: the network's command passed through the same brake.
  All avoiders see the same input: desired velocity, own velocity, slot error, 24 range readings,
  the nearest obstacle point, the three nearest drones. The avoider runs at 10 Hz.
- Map data: `swarm_tools/osm.py` downloads buildings, woods and trees from OpenStreetMap (Overpass
  API) and caches them in `data/osm/`. Flight height decides what is an obstacle: low (16 m) - every
  building and wood; normal (30 m) - only structures of 25 m or more (OSM height or storeys).

## 9. Simulators

| Simulator | What it runs | Used for |
|---|---|---|
| PX4 SIH + MAVROS (`swarm_tools/sim_launch.py`) | the real autopilot firmware, simple physics | Phases 0-3: real flights of up to 10 drones |
| Fast simulator (`swarm_tools/puresim.py`) | the real agent code, point-mass physics, simulated radio | 1,000-run tests, scaling, the ground-control app |
| Obstacle simulator (`swarm_tools/obstacle_sim.py`) | numpy formation-through-obstacles | RL training (CPU) and the fair comparison |
| Batched simulator (`swarm_tools/torch_sim.py`) | the obstacle simulator on a GPU, 1024 missions at once | long RL training on Kaggle |

The obstacle simulator re-implements the follower law and repulsion in numpy; tests check it
against the agent code (`tests/test_avoidance.py`), and the batched version is checked step by
step against the numpy one (`tests/test_torch_sim.py`).

## 10. Ground-control app (`swarm_tools/gcs/`)
`fastsim_backend.py` runs the fast simulator in a background thread (1x to Max) and turns its state
into latitude/longitude; `server.py` (standard library HTTP server on 127.0.0.1) streams a snapshot
every 100 ms (Server-Sent Events) and accepts commands (start, pause, speed, faults, radio split).
Pages in `static/`: Mission (Leaflet map), 3D view (CesiumJS), Results, Docs. Slow preparation (map
downloads, long-route planning) runs in a background thread with progress on the page.

- **3D view** (`static/3d.html`, `static/view3d.js`): every drone is a small 3-D quadcopter
  (`static/drone.glb`, generated by `scripts/make_drone_model.py` - no outside model files) whose
  white body is tinted by role. Snapshots arrive every 100 ms; each drone glides from where it is
  drawn to its newest position over one snapshot interval, so the motion is smooth (about 0.1 s
  behind the simulation). The simulators have flat ground, so each drone is drawn at its simulated
  height above the terrain under it; a drone appears only once that terrain height is known. V lines
  join the leader to the followers behind it on each side. In the Chase, Orbit and Top modes the
  camera is placed every frame around the leader and the followers in its formation, and lifts
  itself when it would go below the terrain.
- **Takeover timing:** when a fault hits the leader (a fault button, a crash, an empty battery), the
  backend remembers when; the next "became master" event says how many seconds later the new
  leader took over (for example 1.5 s after a crash, 0.1 s after a planned handover).
- **Without the browser:** `scripts/run_route.py` drives the same backend step by step and writes a
  JSON record (events, progress, formation error and closest pair while cruising); the long-route
  evidence in `reports/logs/long_route/` comes from it.
- **One-click start:** `scripts/start_swarm.sh` starts `scripts/gcs.py` in the background if it is not
  already running and opens the browser. The **Swarm Control** program in the project folder
  (`scripts/launcher.c`), `start_swarm_control.sh` and the desktop icon all call it.

## 11. Where to change things

| To change | Edit |
|---|---|
| spacing, speeds, gains, timeouts, battery levels, geofence | `config/swarm.yaml` |
| election rules | `src/swarm_agent/election.py` (+ `tests/test_election.py`) |
| formation shape | `src/swarm_agent/formation.py` (V only today) |
| what counts as an obstacle, flight heights | `src/swarm_tools/osm.py`, `gcs/fastsim_backend.py` |
| the avoider used by the app | `models/avoid_policy.npz` (RL) or `PotentialField` parameters |
| the 3-D view (drone look, cameras, colours) | `src/swarm_tools/gcs/static/view3d.js`, `3d.html`; the model: `scripts/make_drone_model.py` |
| reward / training | `src/swarm_tools/obstacle_sim.py`, `scripts/rl_train.py`, `scripts/rl_train_gpu.py` |
