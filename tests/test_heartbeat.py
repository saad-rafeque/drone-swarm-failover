"""Heartbeat binary codec."""
from __future__ import annotations

import math

import pytest

from swarm_agent.heartbeat import (HEADER_SIZE, MAX_ID, Flag, Heartbeat, Phase, Role, decode, encode,
                                   encoded_size, mask_to_members, members_to_mask)


def sample(**kw) -> Heartbeat:
    base = dict(drone_id=7, role=Role.MASTER, term=42, phase=Phase.CRUISE, master_id=7, handover_to=3,
                flags=Flag.MASTER_OK | Flag.READY | Flag.AIRBORNE, battery_pct=63.27,
                pos=(-123.4567, 987.6543, 30.0012), vel=(1.234, -4.987, 0.05), heading=1.2345,
                members=members_to_mask([1, 3, 7, 10]), stamp=1234.5678)
    base.update(kw)
    return Heartbeat(**base)


def test_size_is_small_and_grows_only_with_the_highest_id():
    assert HEADER_SIZE == 43
    assert len(encode(sample())) == encoded_size(10) == 45
    big = sample(members=members_to_mask(range(1, 101)))
    assert len(encode(big)) == encoded_size(100) == 56
    assert decode(encode(big)).members == big.members
    assert encoded_size(MAX_ID) == 75


def test_round_trip_within_quantisation():
    hb = sample()
    out = decode(encode(hb))
    for field in ("drone_id", "role", "term", "phase", "master_id", "handover_to", "flags", "members", "stamp"):
        assert getattr(out, field) == getattr(hb, field)
    assert all(abs(a - b) <= 0.0005 for a, b in zip(out.pos, hb.pos))
    assert all(abs(a - b) <= 0.005 for a, b in zip(out.vel, hb.vel))
    assert abs(out.heading - hb.heading) <= 5e-5
    assert abs(out.battery_pct - hb.battery_pct) <= 0.005
    assert out.has(Flag.READY) and not out.has(Flag.ORPHAN)


def test_heading_wraps_and_extremes_clamp():
    out = decode(encode(sample(heading=3 * math.pi, vel=(500.0, -500.0, 0.0), battery_pct=150.0, term=2**32 + 5)))
    assert abs(abs(out.heading) - math.pi) < 1e-3
    assert out.vel[0] == pytest.approx(327.67) and out.vel[1] == pytest.approx(-327.68)
    assert out.battery_pct == 100.0
    assert out.term == 5  # uint32 wraps; terms never get near that in practice


def test_rejects_bad_input():
    data = bytearray(encode(sample()))
    with pytest.raises(ValueError):
        decode(bytes(data[:-1]))
    with pytest.raises(ValueError):
        decode(bytes(data) + b"\x00")
    data[0] = 0x00
    with pytest.raises(ValueError):
        decode(bytes(data))


def test_members_mask():
    assert mask_to_members(members_to_mask([1, 2, 63])) == frozenset({1, 2, 63})
    assert mask_to_members(0) == frozenset()
    with pytest.raises(ValueError):
        members_to_mask([0])
    assert mask_to_members(members_to_mask([64, 250])) == frozenset({64, 250})
    with pytest.raises(ValueError):
        members_to_mask([MAX_ID + 1])
    with pytest.raises(ValueError):
        encode(sample(members=1 << (MAX_ID + 1)))
