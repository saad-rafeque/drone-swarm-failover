"""Heartbeat message and its compact binary codec (50 bytes; radio-friendly).

docs/SPECIFICATION.md §5 fields: id, role, term, position, velocity, battery, timestamp. Extra fields the
design needs (documented in reports/PHASE_2.md):
  phase        mission phase the sender is in (followers copy the master's)
  master_id    whom the sender follows (itself if master, 0 if none)
  handover_to  successor named by a retiring master (0 = none)
  flags        MASTER_OK (sender hears a live master), READY, ELIGIBLE, ORPHAN, AIRBORNE
  heading      the master's formation heading [rad, ENU]
  members      bitmask of drone IDs the master currently hears (bit i = ID i)
Position/velocity are in the shared ENU frame (each drone converts its own lat/lon/alt).
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum

from .geometry import Vec3

MAGIC = 0xA7
VERSION = 1
_FMT = struct.Struct("<BBBBBBIBBHiiihhhhQd")
SIZE = _FMT.size


class Role(IntEnum):
    FOLLOWER = 0
    MASTER = 1
    RETIRED = 2   # left the formation (low battery), returning home


class Phase(IntEnum):
    IDLE = 0      # on the ground, waiting for takeoff
    TAKEOFF = 1
    CRUISE = 2
    HOLD = 3      # hovering at the goal
    LAND = 4
    LANDED = 5


class Flag:
    MASTER_OK = 1   # sender currently hears a live master (or is master)
    READY = 2       # sender has a valid position and can arm
    ELIGIBLE = 4    # sender may become master (battery above handover level, not retired)
    ORPHAN = 8      # sender's master does not list it as a member
    AIRBORNE = 16


@dataclass(frozen=True, slots=True)
class Heartbeat:
    drone_id: int
    role: Role
    term: int
    phase: Phase
    master_id: int
    handover_to: int
    flags: int
    battery_pct: float
    pos: Vec3
    vel: Vec3
    heading: float
    members: int
    stamp: float

    def has(self, flag: int) -> bool:
        return bool(self.flags & flag)


def members_to_mask(ids) -> int:
    mask = 0
    for i in ids:
        if not 1 <= i <= 63:
            raise ValueError(f"drone ID {i} outside 1..63")
        mask |= 1 << i
    return mask


def mask_to_members(mask: int) -> frozenset[int]:
    return frozenset(i for i in range(1, 64) if mask >> i & 1)


def _i16(x: float) -> int:
    return max(-32768, min(32767, round(x)))


def _i32(x: float) -> int:
    return max(-2147483648, min(2147483647, round(x)))


def encode(hb: Heartbeat) -> bytes:
    """Quantisation: position 1 mm, velocity 1 cm/s, heading 1e-4 rad, battery 0.01 %."""
    return _FMT.pack(
        MAGIC, VERSION, hb.drone_id, int(hb.role), int(hb.phase), hb.flags,
        hb.term & 0xFFFFFFFF, hb.master_id, hb.handover_to,
        max(0, min(10000, round(hb.battery_pct * 100))),
        _i32(hb.pos[0] * 1000), _i32(hb.pos[1] * 1000), _i32(hb.pos[2] * 1000),
        _i16(hb.vel[0] * 100), _i16(hb.vel[1] * 100), _i16(hb.vel[2] * 100),
        _i16(math.remainder(hb.heading, 2 * math.pi) * 1e4),
        hb.members & 0xFFFFFFFFFFFFFFFF, hb.stamp,
    )


def decode(data: bytes) -> Heartbeat:
    if len(data) != SIZE:
        raise ValueError(f"heartbeat must be {SIZE} bytes, got {len(data)}")
    (magic, version, did, role, phase, flags, term, master_id, handover_to, batt,
     px, py, pz, vx, vy, vz, heading, members, stamp) = _FMT.unpack(data)
    if magic != MAGIC or version != VERSION:
        raise ValueError(f"bad heartbeat magic/version {magic:#x}/{version}")
    return Heartbeat(
        drone_id=did, role=Role(role), term=term, phase=Phase(phase), master_id=master_id,
        handover_to=handover_to, flags=flags, battery_pct=batt / 100.0,
        pos=(px / 1000.0, py / 1000.0, pz / 1000.0), vel=(vx / 100.0, vy / 100.0, vz / 100.0),
        heading=heading / 1e4, members=members, stamp=stamp,
    )
