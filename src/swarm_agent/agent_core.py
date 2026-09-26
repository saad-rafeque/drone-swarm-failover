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
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

from .config import Config
from .election import Election
from .formation import Slot, follower_slot, follower_velocity, slot_position
from .geometry import ZERO, Vec3, add, clamp_xy_z, heading_of, norm_xy, scale, sub
from .heartbeat import Flag, Heartbeat, Phase, Role, mask_to_members, members_to_mask
from .safety import geofence_limit, repulsion

ORPHAN_DELAY_S = 1.0        # must be missing from the master's member list this long
CLIMB_FIRST_MARGIN_M = 5.0  # a late joiner this far below cruise altitude climbs before joining
KP_ALT = 1.0                # altitude P-gain [1/s]
KP_HOLD = 0.5               # horizontal hold P-gain [1/s]
HOLD_MAX_MPS = 2.0
TRANSIT_ENTER_DELAY_S = 0.5  # slot error must exceed the threshold this long (filters glitches)
LAYER_TOL_M = 1.5           # "on the layer" tolerance
LAYER_CHANGE_MAX_CORRECTION_MPS = 1.0  # horizontal P-term limit until vertically clear of the formation layer


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


@dataclass(frozen=True, slots=True)
class Command:
    mode: FlightMode
    vel: Vec3
    reason: str
    target: Vec3 | None = None


def _clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


class AgentCore:
    def __init__(self, cfg: Config, my_id: int, home: Vec3, now: float) -> None:
        self.cfg = cfg
        self.my_id = my_id
        self.home = home
        hb = cfg.heartbeat
        self.election = Election(my_id, hb.master_timeout_s, hb.peer_timeout_s, hb.handover_timeout_s,
                                 cfg.mission.startup_listen_s, now)
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
        self._retire_vel: Vec3 = ZERO
        self.last_cmd: Command | None = None

    # ------------------------------------------------------------------ public API
    def on_heartbeat(self, hb: Heartbeat, now: float) -> None:
        self.election.on_heartbeat(hb, now)

    def step(self, own: OwnState, now: float) -> tuple[Command, Heartbeat | None]:
        e = self.election
        low_battery = own.battery_pct <= self.cfg.battery.handover_pct
        if low_battery and not e.retiring:
            self._retire_vel = (own.vel[0], own.vel[1], 0.0)
            e.start_retire(now)
        e.set_eligible(own.ready and not low_battery)
        e.update(now)

        if e.role == Role.MASTER:
            if not self._was_master:
                self.phase_since = now  # takeover: restart the current phase's timer
            self._master_phase(own, now)
        elif e.role == Role.FOLLOWER:
            info = e.master_info(now)
            if info is not None:
                if info.hb.phase != self.phase:
                    self.phase, self.phase_since = info.hb.phase, now
                self.heading = info.hb.heading
        self._was_master = e.role == Role.MASTER

        if own.pos[2] >= self.cfg.mission.cruise_alt_m - self.cfg.mission.takeoff_alt_tolerance_m:
            self._joined = True
        cmd = self._command(own, now)
        self.last_cmd = cmd
        return cmd, self._heartbeat(own, now)

    # ------------------------------------------------------------------ mission (master)
    def _set_phase(self, phase: Phase, now: float) -> None:
        self.phase, self.phase_since = phase, now

    def _master_phase(self, own: OwnState, now: float) -> None:
        m = self.cfg.mission
        e = self.election
        peers = [p.hb for p in e.alive_peers(now) if p.hb.role != Role.RETIRED]
        to_goal = sub(self.goal, own.pos)
        if self.phase in (Phase.IDLE, Phase.TAKEOFF):
            self.heading = heading_of(to_goal[0], to_goal[1])
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
            d = norm_xy(to_goal)
            if d > 2.0 * m.goal_radius_m:
                self.heading = heading_of(to_goal[0], to_goal[1])
            if d <= m.goal_radius_m:
                self._set_phase(Phase.HOLD, now)
        elif self.phase == Phase.HOLD:
            if now - self.phase_since >= m.hover_at_goal_s:
                self._set_phase(Phase.LAND, now)

    # ------------------------------------------------------------------ commands
    def _command(self, own: OwnState, now: float) -> Command:
        e = self.election
        if e.role == Role.RETIRED:
            return self._retire_command(own, now)
        if self.phase == Phase.IDLE:
            return Command(FlightMode.GROUND, ZERO, "idle")
        if self.phase in (Phase.LAND, Phase.LANDED):
            return Command(FlightMode.LAND, ZERO, "land")

        m, f = self.cfg.mission, self.cfg.formation
        target: Vec3 | None
        if self.phase == Phase.TAKEOFF:
            target = (self.home[0], self.home[1], m.cruise_alt_m)
            v, reason = self._goto(own.pos, target, HOLD_MAX_MPS), "takeoff"
        elif e.role == Role.MASTER:
            target = self.goal
            if self.phase == Phase.CRUISE:
                v, reason = self._cruise_to_goal(own.pos), "cruise"
            else:
                v, reason = self._goto(own.pos, self.goal, HOLD_MAX_MPS), "hold"
        elif not self._joined and own.pos[2] < m.cruise_alt_m - CLIMB_FIRST_MARGIN_M:
            target = (own.pos[0], own.pos[1], m.cruise_alt_m)
            v, reason = self._goto(own.pos, target, HOLD_MAX_MPS), "climb_to_join"
        else:
            info = e.master_info(now)
            if info is None:
                target, v, reason = None, ZERO, "hover_no_master"
            else:
                target, v, reason = self._formation(own, info.hb, now)
        return Command(FlightMode.OFFBOARD, self._safe(own, v, now), reason, target)

    def _vz(self, dz: float) -> float:
        m = self.cfg.mission
        return max(-m.descent_rate_mps, min(m.climb_rate_mps, KP_ALT * dz))

    def _goto(self, pos: Vec3, target: Vec3, max_xy: float) -> Vec3:
        m = self.cfg.mission
        d = sub(target, pos)
        return clamp_xy_z((KP_HOLD * d[0], KP_HOLD * d[1], KP_ALT * d[2]), max_xy, m.climb_rate_mps,
                          m.descent_rate_mps)

    def _cruise_to_goal(self, pos: Vec3) -> Vec3:
        m = self.cfg.mission
        d = sub(self.goal, pos)
        dxy = norm_xy(d)
        speed = min(m.cruise_speed_mps, KP_HOLD * dxy)
        vx, vy = (d[0] / dxy * speed, d[1] / dxy * speed) if dxy > 1e-6 else (0.0, 0.0)
        return (vx, vy, self._vz(d[2]))

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
        if own.landed and self.retire_stage in ("", "land"):
            self.retire_stage = "land"
            return Command(FlightMode.GROUND, ZERO, "retired_on_ground")
        return_alt = m.cruise_alt_m + self.cfg.battery.retire_alt_offset_m
        if self.retire_stage == "":
            self.retire_stage = "climb"
        if self.retire_stage == "climb":
            # Keep the formation's horizontal velocity while climbing out of the formation layer.
            target = (own.pos[0], own.pos[1], return_alt)
            v = (self._retire_vel[0], self._retire_vel[1], self._vz(return_alt - own.pos[2]))
            if own.pos[2] >= return_alt - 1.0:
                self.retire_stage = "transit"
            return Command(FlightMode.OFFBOARD, self._safe(own, v, now), "retire_climb", target)
        if self.retire_stage == "transit":
            target = (self.home[0], self.home[1], return_alt)
            d = sub(target, own.pos)
            dxy = norm_xy(d)
            speed = min(m.cruise_speed_mps, KP_HOLD * dxy)
            v = ((d[0] / dxy * speed, d[1] / dxy * speed) if dxy > 1e-6 else (0.0, 0.0)) + (self._vz(d[2]),)
            if dxy <= m.goal_radius_m:
                self.retire_stage = "land"
            return Command(FlightMode.OFFBOARD, self._safe(own, v, now), "retire_transit", target)
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
