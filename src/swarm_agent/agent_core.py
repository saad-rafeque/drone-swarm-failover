"""Per-drone swarm agent (pure Python): mission phases + election + formation + safety.

The same class runs inside the ROS 2 node (Phase 3) and inside the point-mass simulator:
    core.on_heartbeat(hb, now)        for every heartbeat received from another drone
    cmd, hb = core.step(own, now)     at the setpoint rate; send cmd to the autopilot and
                                      broadcast hb when it is not None
There is no central controller: every drone decides its own role and command.

Mission (driven by the master, copied by followers from its heartbeat):
    IDLE -> TAKEOFF (all expected drones ready, or startup timeout)
         -> CRUISE  (all members near cruise altitude, or takeoff timeout)
         -> HOLD    (master within goal radius; hover hover_at_goal_s)
         -> LAND    (every drone lands in place)

Obstacles (optional World): the master follows a route planned around buildings and trees
(planner.py) instead of the straight leg; every drone adds its avoider's correction (avoidance.py:
classical potential field or learned policy) before the safety layer; a drone going home plans its
own route. Without a World the behaviour is exactly the open-sky one.

Long routes (World.stops): at each charging stop the master lands the swarm (LAND), waits on the
ground until every battery is nearly full (CHARGE), takes off vertically (TAKEOFF) and continues
the route (CRUISE). A drone whose battery gets low on such a route lands where it is instead of
flying home, which may be far behind.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

from .avoidance import RAY_ANGLES, RAY_RANGE_M, AvoidInput, NoAvoidance, brake
from .config import Config
from .election import Election
from .formation import Slot, follower_slot, follower_velocity, slot_position
from .geometry import ZERO, Vec3, add, clamp_xy_z, heading_of, norm_xy, scale, sub
from .heartbeat import Flag, Heartbeat, Phase, Role, mask_to_members, members_to_mask
from .obstacles import ObstacleMap
from .planner import PathFollower, plan_path
from .safety import geofence_limit, repulsion

ORPHAN_DELAY_S = 1.0        # must be missing from the master's member list this long
CLIMB_FIRST_MARGIN_M = 5.0  # a late joiner this far below cruise altitude climbs before joining
KP_ALT = 1.0                # altitude P-gain [1/s]
KP_HOLD = 0.5               # horizontal hold P-gain [1/s]
HOLD_MAX_MPS = 2.0
KP_CROSS_TRACK = 0.3        # master's cross-track correction toward its leg line [1/s]
CROSS_TRACK_MAX_MPS = 1.0
TRANSIT_ENTER_DELAY_S = 0.5  # slot error must exceed the threshold this long (filters glitches)
LAYER_TOL_M = 1.5
EMERGENCY_DROP_MARGIN_M = 2.0  # emergency descent keeps formation speed until min_sep + this below cruise           # "on the layer" tolerance
LAYER_CHANGE_MAX_CORRECTION_MPS = 1.0  # horizontal P-term limit until vertically clear of the formation layer
HEADING_RATE_DPS = 20.0     # the master turns the formation heading at most this fast along a route
ROUTE_CLEARANCE_M = 8.0     # planned routes keep this far from every obstacle
CHARGE_FULL_PCT = 95.0      # at a charging stop, take off again once every battery reaches this
CHARGE_TIMEOUT_S = 1800.0   # ...or after this long, whatever the batteries say
STOP_APPROACH_M = 25.0      # the master leaves the route follower and homes on the stop this far before it


class FlightMode(IntEnum):
    GROUND = 0    # stay on the ground, disarmed
    OFFBOARD = 1  # armed, OFFBOARD, follow Command.vel
    LAND = 2      # autopilot land mode at the current position


@dataclass(frozen=True, slots=True)
class OwnState:
    pos: Vec3           # shared ENU frame; z = height above the origin's ground level
    vel: Vec3
    battery_pct: float
    ready: bool         # valid position, able to arm
    landed: bool
    gps_ok: bool = True # False: position no longer trustworthy -> emergency landing


@dataclass(frozen=True, slots=True)
class Command:
    mode: FlightMode
    vel: Vec3
    reason: str
    target: Vec3 | None = None


def _clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


@dataclass
class World:
    """Static obstacles the drone knows about (a map in simulation; on a real drone the range readings
    would come from its own sensor), the avoider used around them, and the route planned before
    takeoff from home to the goal (ENU waypoints), loaded into every drone like the goal itself."""
    omap: ObstacleMap
    avoider: object = None            # correction(AvoidInput) -> (dvx, dvy); None = no avoidance
    route: list[tuple[float, float]] | None = None
    stops: list[tuple[float, float, float]] = field(default_factory=list)   # charging stops (s along route, x, y)


class AgentCore:
    def __init__(self, cfg: Config, my_id: int, home: Vec3, now: float, world: World | None = None) -> None:
        self.cfg = cfg
        self.world = world
        self._route = PathFollower(world.route) if world is not None and world.route else None
        self._home_route: PathFollower | None = None
        self._stops = list(world.stops) if world is not None and world.stops else []
        self.next_stop = 0              # index of the next charging stop
        self._stopping = False          # current LAND is at a charging stop, not at the goal
        self._final = False             # reached the goal: the current LAND ends the mission
        self._pos_xy = (home[0], home[1])
        self._takeoff_xy = (home[0], home[1])   # vertical takeoff from here (home, or a charging stop)
        self.my_id = my_id
        self.home = home
        hb = cfg.heartbeat
        self.election = Election(my_id, hb.master_timeout_s, hb.peer_timeout_s, hb.handover_timeout_s,
                                 cfg.mission.startup_listen_s, now, frozenset(cfg.drone_ids),
                                 cfg.mission.startup_timeout_s)
        m = cfg.mission
        self.goal: Vec3 = (m.goal_enu_m[0], m.goal_enu_m[1], m.cruise_alt_m)
        self.phase = Phase.IDLE
        self.phase_since = now
        self.boot_time = now
        self.heading = heading_of(self.goal[0] - home[0], self.goal[1] - home[1])
        self.half_angle = math.radians(cfg.formation.v_half_angle_deg)
        self.hb_period = 1.0 / hb.rate_hz
        self.last_hb_time = -math.inf
        self.orphan = False
        self.transit = False
        self._joined = False   # reached cruise altitude at least once this flight
        self._far_since: float | None = None
        self._not_member_since: float | None = None
        self._last_slot: Slot | None = None
        self._was_master = False
        self.retire_stage = ""
        self.emergency = False
        self._retire_vel: Vec3 = ZERO
        self._speed_cmd = 0.0   # master's ramped cruise speed
        self._speed_t = now
        self._leg_start: tuple[float, float] | None = None  # master's straight leg to the goal
        self.last_cmd: Command | None = None

    # ------------------------------------------------------------------ public API
    def on_heartbeat(self, hb: Heartbeat, now: float) -> None:
        self.election.on_heartbeat(hb, now)

    @property
    def mission_complete(self) -> bool:
        """This drone led the swarm to the goal and the final landing has begun."""
        return self._final and self.phase in (Phase.LAND, Phase.LANDED)

    @property
    def stops_total(self) -> int:
        return len(self._stops)

    @property
    def at_stop(self) -> bool:
        """The current landing / ground time is a charging stop, not the end of the mission."""
        return self._stopping or self.phase == Phase.CHARGE

    def step(self, own: OwnState, now: float) -> tuple[Command, Heartbeat | None]:
        e = self.election
        self._pos_xy = (own.pos[0], own.pos[1])
        low_battery = own.battery_pct <= self.cfg.battery.handover_pct
        emergency = not own.gps_ok or own.battery_pct <= self.cfg.battery.critical_pct \
            or (low_battery and bool(self._stops) and not own.landed)
        if (low_battery or emergency) and not e.retiring:
            self._retire_vel = (own.vel[0], own.vel[1], 0.0)
            e.start_retire(now)
        if emergency and not self.emergency:
            # GPS lost or battery critical: a master hands over first (start_retire); then the
            # drone drops out of the formation layer while keeping the formation's speed (so the
            # drones behind pass over it, not into it) and lands where it is instead of flying home.
            self.emergency = True
            self.retire_stage = "drop"
        e.set_eligible(own.ready and not low_battery and not emergency)
        e.update(now)

        if e.role == Role.MASTER:
            if not self._was_master:
                self.phase_since = now  # takeover: restart the current phase's timer
                self._speed_cmd = norm_xy(own.vel)  # continue from the current speed, then ramp
                self._speed_t = now
                self._leg_start = None  # new leg from wherever this drone is
                self._sync_route_state(own)
            self._master_phase(own, now)
        elif e.role == Role.FOLLOWER:
            info = e.master_info(now)
            if info is not None:
                if info.hb.phase != self.phase:
                    if info.hb.phase == Phase.TAKEOFF:
                        self._takeoff_xy = (own.pos[0], own.pos[1])
                    self.phase, self.phase_since = info.hb.phase, now
                self.heading = info.hb.heading
        self._was_master = e.role == Role.MASTER

        if own.pos[2] >= self.cfg.mission.cruise_alt_m - self.cfg.mission.takeoff_alt_tolerance_m:
            self._joined = True
        cmd = self._command(own, now)
        self.last_cmd = cmd
        return cmd, self._heartbeat(own, now)

    def _sync_route_state(self, own: OwnState) -> None:
        """A new leader works out where the swarm is along the route: progress, the next charging stop,
        whether the current landing is at a stop or at the goal."""
        if self._route is None:
            return
        s = self._route.relocate(own.pos[0], own.pos[1])
        at = [k for k, (_, x, y) in enumerate(self._stops) if math.hypot(x - own.pos[0], y - own.pos[1]) <= 60.0]
        if at and self.phase in (Phase.LAND, Phase.CHARGE):
            self.next_stop, self._stopping = at[0], self.phase == Phase.LAND
        elif at and self.phase == Phase.TAKEOFF:
            self.next_stop = at[0] + 1
        else:
            self.next_stop = next((k for k, st in enumerate(self._stops) if st[0] > s + 1.0), len(self._stops))
        goal_d = math.hypot(self.goal[0] - own.pos[0], self.goal[1] - own.pos[1])
        if self.phase in (Phase.LAND, Phase.LANDED) and not self._stopping and goal_d <= 3 * self.cfg.mission.goal_radius_m:
            self._final = True

    # ------------------------------------------------------------------ mission (master)
    def _set_phase(self, phase: Phase, now: float) -> None:
        if phase == Phase.TAKEOFF:
            self._takeoff_xy = self._pos_xy
        self.phase, self.phase_since = phase, now

    def _master_phase(self, own: OwnState, now: float) -> None:
        m = self.cfg.mission
        e = self.election
        peers = [p.hb for p in e.alive_peers(now) if p.hb.role != Role.RETIRED]
        to_goal = sub(self.goal, own.pos)
        if self.phase in (Phase.IDLE, Phase.TAKEOFF):
            self.heading = self._route.tangent(self._route.s) if self._route is not None \
                else heading_of(to_goal[0], to_goal[1])
        if self.phase == Phase.IDLE:
            ready = int(own.ready) + sum(1 for hb in peers if hb.has(Flag.READY))
            if ready >= self.cfg.swarm.num_drones or (own.ready and now - self.boot_time >= m.startup_timeout_s):
                self._set_phase(Phase.TAKEOFF, now)
        elif self.phase == Phase.TAKEOFF:
            floor = m.cruise_alt_m - m.takeoff_alt_tolerance_m
            if (own.pos[2] >= floor and all(hb.pos[2] >= floor for hb in peers)) or \
                    now - self.phase_since >= m.takeoff_timeout_s:
                self._set_phase(Phase.CRUISE, now)
        elif self.phase == Phase.CRUISE:
            if self._leg_start is None:
                # Straight leg from here to the goal; its bearing is the formation heading for the
                # whole leg (re-aiming at the goal from a wandering position rotates the V).
                self._leg_start = (own.pos[0], own.pos[1])
                if self._route is None:
                    self.heading = heading_of(to_goal[0], to_goal[1])
            if self.next_stop < len(self._stops):
                _, sx, sy = self._stops[self.next_stop]
                if math.hypot(sx - own.pos[0], sy - own.pos[1]) <= m.goal_radius_m:
                    self._stopping = True            # land the swarm at the charging stop
                    self._set_phase(Phase.LAND, now)
            elif norm_xy(to_goal) <= m.goal_radius_m:
                self._set_phase(Phase.HOLD, now)
        elif self.phase == Phase.HOLD:
            if now - self.phase_since >= m.hover_at_goal_s:
                self._final = True
                self._set_phase(Phase.LAND, now)
        elif self.phase == Phase.LAND and self._stopping:
            if own.landed and not any(hb.has(Flag.AIRBORNE) for hb in peers):
                self._set_phase(Phase.CHARGE, now)
        elif self.phase == Phase.CHARGE:
            full = own.battery_pct >= CHARGE_FULL_PCT and all(hb.battery_pct >= CHARGE_FULL_PCT for hb in peers)
            if full or now - self.phase_since >= CHARGE_TIMEOUT_S:
                self.next_stop += 1
                self._stopping = False
                self._set_phase(Phase.TAKEOFF, now)

    # ------------------------------------------------------------------ commands
    def _command(self, own: OwnState, now: float) -> Command:
        e = self.election
        if e.role == Role.RETIRED:
            return self._retire_command(own, now)
        if self.phase == Phase.IDLE:
            return Command(FlightMode.GROUND, ZERO, "idle")
        if self.phase == Phase.CHARGE:
            return Command(FlightMode.GROUND, ZERO, "charging")
        if self.phase in (Phase.LAND, Phase.LANDED):
            return Command(FlightMode.LAND, ZERO, "land")

        m, f = self.cfg.mission, self.cfg.formation
        target: Vec3 | None
        if self.phase == Phase.TAKEOFF:
            target = (self._takeoff_xy[0], self._takeoff_xy[1], m.cruise_alt_m)
            v, reason = self._goto(own.pos, target, HOLD_MAX_MPS), "takeoff"
        elif e.role == Role.MASTER:
            target = self.goal
            if self.phase == Phase.CRUISE:
                stop = self._stops[self.next_stop] if self.next_stop < len(self._stops) else None
                if stop is not None and self._route is not None and self._route.s >= stop[0] - STOP_APPROACH_M:
                    target = (stop[1], stop[2], m.cruise_alt_m)
                    v = self._goto(own.pos, target, m.cruise_speed_mps)
                else:
                    v = self._follow_route(own.pos, now) if self._route is not None else self._cruise_to_goal(own.pos, now)
                v, reason = self._avoid(own, v, None, now), "cruise"
            else:
                v, reason = self._goto(own.pos, self.goal, HOLD_MAX_MPS), "hold"
            if self.phase != Phase.CRUISE:
                self._speed_cmd, self._speed_t = 0.0, now
        elif not self._joined and own.pos[2] < m.cruise_alt_m - CLIMB_FIRST_MARGIN_M:
            target = (own.pos[0], own.pos[1], m.cruise_alt_m)
            v, reason = self._goto(own.pos, target, HOLD_MAX_MPS), "climb_to_join"
        else:
            info = e.master_info(now)
            if info is None:
                target, v, reason = None, ZERO, "hover_no_master"
            else:
                target, v, reason = self._formation(own, info.hb, now)
                v = self._avoid(own, v, target, now)
        return Command(FlightMode.OFFBOARD, self._safe(own, v, now), reason, target)

    def _vz(self, dz: float) -> float:
        m = self.cfg.mission
        return max(-m.descent_rate_mps, min(m.climb_rate_mps, KP_ALT * dz))

    def _goto(self, pos: Vec3, target: Vec3, max_xy: float) -> Vec3:
        m = self.cfg.mission
        d = sub(target, pos)
        return clamp_xy_z((KP_HOLD * d[0], KP_HOLD * d[1], KP_ALT * d[2]), max_xy, m.climb_rate_mps,
                          m.descent_rate_mps)

    def _cruise_to_goal(self, pos: Vec3, now: float) -> Vec3:
        """Follow the straight leg to the goal: along-track at cruise speed (speed-ups ramped at
        cruise_accel_mps2, so followers who see the master's velocity only at the heartbeat rate
        keep up) plus a saturated cross-track correction back onto the leg line."""
        m = self.cfg.mission
        d = sub(self.goal, pos)
        ux, uy = math.cos(self.heading), math.sin(self.heading)   # leg direction
        along = d[0] * ux + d[1] * uy                               # remaining distance along the leg
        cross = -d[0] * uy + d[1] * ux                              # + = the leg line is to my left
        dt = min(max(now - self._speed_t, 0.0), 0.5)
        speed = max(0.0, min(m.cruise_speed_mps, KP_HOLD * along, self._speed_cmd + m.cruise_accel_mps2 * dt))
        self._speed_cmd, self._speed_t = speed, now
        vc = max(-CROSS_TRACK_MAX_MPS, min(CROSS_TRACK_MAX_MPS, KP_CROSS_TRACK * cross))
        return (ux * speed - uy * vc, uy * speed + ux * vc, self._vz(d[2]))

    def _follow_route(self, pos: Vec3, now: float) -> Vec3:
        """Master on a planned route: pure pursuit at cruise speed (speed-ups ramped as on the straight
        leg), the formation heading turning toward the route direction at HEADING_RATE_DPS."""
        m = self.cfg.mission
        route = self._route
        vx, vy, _ = route.command(pos[0], pos[1], m.cruise_speed_mps, slow_radius_m=m.cruise_speed_mps / KP_HOLD)
        sp = math.hypot(vx, vy)
        dt = min(max(now - self._speed_t, 0.0), 0.5)
        speed = min(sp, self._speed_cmd + m.cruise_accel_mps2 * dt)
        self._speed_cmd, self._speed_t = speed, now
        k = speed / sp if sp > 1e-9 else 0.0
        turn = math.remainder(route.tangent(route.s) - self.heading, 2.0 * math.pi)
        step = math.radians(HEADING_RATE_DPS) * dt
        self.heading += max(-step, min(step, turn))
        return (vx * k, vy * k, self._vz(self.goal[2] - pos[2]))

    def _neighbours(self, now: float) -> list[tuple[Vec3, Vec3]]:
        """(position extrapolated to now, velocity) of every recently heard drone, ghosts included."""
        s, hb = self.cfg.safety, self.cfg.heartbeat
        horizon = hb.peer_timeout_s + s.ghost_timeout_s
        return [(add(p.hb.pos, scale(p.hb.vel, min(max(now - p.hb.stamp, 0.0), horizon))), p.hb.vel)
                for p in self.election.peers.values() if now - p.rx_time <= horizon]

    def _avoid(self, own: OwnState, v: Vec3, target: Vec3 | None, now: float, brake_only: bool = False) -> Vec3:
        """Add the avoider's horizontal correction (followers) or just the stopping-distance brake
        (the master on its route, drones going home). No World: unchanged."""
        w = self.world
        if w is None or w.avoider is None or isinstance(w.avoider, NoAvoidance):
            return v
        x, y = own.pos[0], own.pos[1]
        heading = self.heading
        inp = AvoidInput(heading, (v[0], v[1]), (own.vel[0], own.vel[1]),
                         (0.0, 0.0) if target is None else (target[0] - x, target[1] - y),
                         w.omap.raycast(x, y, heading + RAY_ANGLES, RAY_RANGE_M),
                         w.omap.nearest_point(x, y, RAY_RANGE_M),
                         [(p[0] - x, p[1] - y, pv[0] - own.vel[0], pv[1] - own.vel[1]) for p, pv in self._neighbours(now)])
        if brake_only or target is None:
            bx, by = brake((v[0], v[1]), inp)
        else:
            dvx, dvy = w.avoider.correction(inp)
            bx, by = v[0] + dvx, v[1] + dvy
        m = self.cfg.mission
        return clamp_xy_z((float(bx), float(by), v[2]), self.cfg.formation.max_speed_mps, m.climb_rate_mps,
                          m.descent_rate_mps)

    def _formation(self, own: OwnState, mhb: Heartbeat, now: float) -> tuple[Vec3, Vec3, str]:
        f, m = self.cfg.formation, self.cfg.mission
        e = self.election
        age = min(max(now - mhb.stamp, 0.0), e.master_timeout)
        master_pos = add(mhb.pos, scale(mhb.vel, age))  # compensate link latency
        members = mask_to_members(mhb.members)
        heard = [p.hb.drone_id for p in e.alive_peers(now) if p.hb.role != Role.RETIRED]
        slot_id, not_member = follower_slot(self.my_id, mhb.drone_id, members, heard)
        if not_member:
            if self._not_member_since is None:
                self._not_member_since = now
            if now - self._not_member_since >= ORPHAN_DELAY_S or self._last_slot is None:
                self.orphan = True
            else:
                slot_id = self._last_slot  # brief dropout: keep the old slot
        else:
            self._not_member_since = None
            self.orphan = False
        self._last_slot = slot_id
        alt_off = f.orphan_alt_offset_m if self.orphan else 0.0
        slot = slot_position(master_pos, mhb.heading, slot_id, f.spacing_m, self.half_angle, alt_off)
        # Long moves (e.g. after a partition heals, the new master may sit far behind) happen on
        # the transit layer below the formation, so crossing paths are vertically separated.
        err_xy = norm_xy(sub(slot, own.pos))
        if self.transit and err_xy < f.transit_exit_m:
            self.transit = False
        elif not self.transit and not self.orphan and err_xy > f.transit_threshold_m:
            if self._far_since is None:
                self._far_since = now
            if now - self._far_since >= TRANSIT_ENTER_DELAY_S:
                self.transit = True
        if err_xy <= f.transit_threshold_m:
            self._far_since = None
        target, max_corr, reason = slot, f.max_correction_mps, "orphan" if self.orphan else "formation"
        if self.transit:
            target = (slot[0], slot[1], master_pos[2] + f.transit_alt_offset_m)
            max_corr, reason = f.transit_max_correction_mps, "transit"
        formation_z = master_pos[2]
        if abs(target[2] - formation_z) > LAYER_TOL_M and \
                abs(own.pos[2] - formation_z) < self.cfg.safety.min_separation_m:
            # Leaving the formation layer: move sideways slowly until vertically clear of it.
            max_corr = min(max_corr, LAYER_CHANGE_MAX_CORRECTION_MPS)
        v = follower_velocity(own.pos, target, mhb.vel, f.pos_gain, max_corr, f.max_speed_mps,
                              m.climb_rate_mps, m.descent_rate_mps, gain_z=KP_ALT)
        return target, v, reason

    def _retire_command(self, own: OwnState, now: float) -> Command:
        m = self.cfg.mission
        if own.landed and self.retire_stage in ("", "land", "drop"):
            self.retire_stage = "land"
            return Command(FlightMode.GROUND, ZERO, "retired_on_ground")
        if self.retire_stage == "drop":
            floor = m.cruise_alt_m - self.cfg.safety.min_separation_m - EMERGENCY_DROP_MARGIN_M
            if own.pos[2] <= floor:
                self.retire_stage = "land"
            else:
                v = self._avoid(own, (self._retire_vel[0], self._retire_vel[1], -m.descent_rate_mps), None, now,
                                brake_only=True)
                return Command(FlightMode.OFFBOARD, self._safe(own, v, now), "emergency_drop",
                               (own.pos[0], own.pos[1], floor))
        return_alt = m.cruise_alt_m + self.cfg.battery.retire_alt_offset_m
        if self.retire_stage == "":
            self.retire_stage = "climb"
        if self.retire_stage == "climb":
            # Keep the formation's horizontal velocity while climbing out of the formation layer.
            target = (own.pos[0], own.pos[1], return_alt)
            v = self._avoid(own, (self._retire_vel[0], self._retire_vel[1], self._vz(return_alt - own.pos[2])),
                            None, now, brake_only=True)
            if own.pos[2] >= return_alt - 1.0:
                self.retire_stage = "transit"
            return Command(FlightMode.OFFBOARD, self._safe(own, v, now), "retire_climb", target)
        if self.retire_stage == "transit":
            target = (self.home[0], self.home[1], return_alt)
            d = sub(target, own.pos)
            dxy = norm_xy(d)
            if self.world is not None and not self.world.omap.empty:
                if self._home_route is None:
                    path = plan_path(self.world.omap, own.pos[:2], self.home[:2], ROUTE_CLEARANCE_M, res_m=4.0)
                    self._home_route = PathFollower(path or [own.pos[:2], self.home[:2]])
                vx, vy, _ = self._home_route.command(own.pos[0], own.pos[1], m.cruise_speed_mps,
                                                     slow_radius_m=m.cruise_speed_mps / KP_HOLD)
                v = self._avoid(own, (vx, vy, self._vz(d[2])), None, now, brake_only=True)
            else:
                speed = min(m.cruise_speed_mps, KP_HOLD * dxy)
                v = ((d[0] / dxy * speed, d[1] / dxy * speed) if dxy > 1e-6 else (0.0, 0.0)) + (self._vz(d[2]),)
            if dxy <= m.goal_radius_m:
                self.retire_stage = "land"
            return Command(FlightMode.OFFBOARD, self._safe(own, v, now), "retire_transit", target)
        if self.emergency:
            return Command(FlightMode.LAND, ZERO, "emergency_land", (own.pos[0], own.pos[1], 0.0))
        return Command(FlightMode.LAND, ZERO, "retire_land", (self.home[0], self.home[1], 0.0))

    # ------------------------------------------------------------------ safety
    def obstacles(self, now: float) -> list[Vec3]:
        """Neighbours' positions extrapolated to now, incl. recently lost ones ("ghosts")."""
        s, hb = self.cfg.safety, self.cfg.heartbeat
        horizon = hb.peer_timeout_s + s.ghost_timeout_s
        out = []
        for p in self.election.peers.values():
            if now - p.rx_time > horizon:
                continue
            age = min(max(now - p.hb.stamp, 0.0), horizon)
            out.append(add(p.hb.pos, scale(p.hb.vel, age)))
        return out

    def _safe(self, own: OwnState, v: Vec3, now: float) -> Vec3:
        s, f, m = self.cfg.safety, self.cfg.formation, self.cfg.mission
        v = add(v, repulsion(own.pos, self.obstacles(now), s.min_separation_m, s.repulsion_factor,
                             s.repulsion_gain))
        v = geofence_limit(own.pos, v, s.geofence_radius_m, s.geofence_max_alt_m, s.geofence_margin_m)
        return clamp_xy_z(v, f.max_speed_mps, m.climb_rate_mps, m.descent_rate_mps)

    # ------------------------------------------------------------------ heartbeat
    def _heartbeat(self, own: OwnState, now: float) -> Heartbeat | None:
        e = self.election
        due = now - self.last_hb_time >= self.hb_period - 1e-9
        if not (due or (e.changed and now - self.last_hb_time >= 0.02)):
            return None
        e.changed = False
        self.last_hb_time = now
        flags = e.flags(now)
        if own.ready:
            flags |= Flag.READY
        if self.orphan and e.role == Role.FOLLOWER:
            flags |= Flag.ORPHAN
        if not own.landed:
            flags |= Flag.AIRBORNE
        members = members_to_mask(e.members(now)) if e.role == Role.MASTER else 0
        return Heartbeat(
            drone_id=self.my_id, role=e.role, term=e.term, phase=self.phase, master_id=e.master_id,
            handover_to=e.handover_to, flags=flags, battery_pct=own.battery_pct, pos=own.pos,
            vel=own.vel, heading=self.heading, members=members, stamp=now,
        )
