"""Election state machine: docs/SPECIFICATION.md §5/§6 rules and the documented design choices."""
from __future__ import annotations

import pytest

from conftest import Bus, hb_from, make_bus
from swarm_agent.election import Election, PeerInfo
from swarm_agent.heartbeat import Flag, Role


def settle(bus: Bus, seconds: float = 4.0) -> None:
    bus.run(seconds)


def test_startup_lowest_id_becomes_master_with_term_1():
    bus = make_bus([1, 2, 3])
    settle(bus)
    assert bus.masters() == [1]
    assert all(e.master_id == 1 and e.term == 1 for e in bus.elections.values())


def test_nobody_claims_during_startup_listen():
    bus = make_bus([1, 2, 3])
    bus.run(2.9)
    assert bus.masters() == []


def test_master_killed_lowest_alive_takes_over_with_higher_term():
    bus = make_bus([1, 2, 3, 4])
    settle(bus)
    bus.dead.add(1)
    took = bus.run_until(lambda: bus.masters() == [2] and all(
        bus.elections[i].master_id == 2 for i in (3, 4)), timeout=4.0)
    assert 1.5 <= took <= 2.2
    assert bus.elections[2].term == 2


def test_followers_hover_without_master_until_election():
    """Between master timeout and the new master's first heartbeat, followers have no master."""
    bus = make_bus([1, 2, 3])
    settle(bus)
    bus.dead.add(1)
    bus.run_until(lambda: bus.elections[3].master_id == 0, timeout=2.0)
    assert bus.elections[3].events[-1].kind == "master_lost"
    assert bus.elections[3].events[-1].detail == {"master": 1}
    bus.run_until(lambda: bus.elections[3].master_id == 2, timeout=1.0)


def test_waits_for_lower_id_candidate():
    e = Election(3, 1.5, 1.5, 1.0, 0.0, 0.0)
    lower = Election(2, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.on_heartbeat(hb_from(lower, 0.1), 0.1)
    e.update(0.2)
    assert e.role == Role.FOLLOWER  # drone 2 is alive and eligible: let it claim
    e.update(2.0)                    # drone 2 silent for > peer timeout: now drone 3 may claim
    assert e.role == Role.MASTER


def test_no_claim_while_a_peer_still_hears_the_master():
    """Asymmetric link: 3 no longer hears master 1, but 2 does and says so -> no false failover."""
    bus = make_bus([1, 2, 3])
    settle(bus)
    bus.blocked.add((1, 3))
    bus.run(5.0)
    assert bus.masters() == [1]
    assert bus.elections[3].role == Role.FOLLOWER
    bus.blocked.clear()
    bus.run(1.0)
    assert bus.elections[3].master_id == 1


def test_master_link_lost_new_master_and_old_steps_down():
    """F2: master's outgoing heartbeats all dropped; it still hears the others."""
    bus = make_bus([1, 2, 3])
    settle(bus)
    bus.blocked |= {(1, 2), (1, 3)}
    bus.run_until(lambda: bus.masters() == [2], timeout=4.0)
    assert bus.elections[1].role == Role.FOLLOWER and bus.elections[1].master_id == 2


def test_partition_then_heal_gives_exactly_one_master():
    bus = make_bus([1, 2, 3, 4, 5])
    settle(bus)
    bus.partition({1, 2}, {3, 4, 5})
    bus.run(5.0)
    assert sorted(bus.masters()) == [1, 3]
    bus.blocked.clear()
    took = bus.run_until(lambda: len(bus.masters()) == 1 and all(
        e.master_id == bus.masters()[0] for e in bus.elections.values()), timeout=3.0)
    assert bus.masters() == [3]  # higher term wins
    assert took < 1.0


def test_no_preemption_by_recovered_lower_id():
    bus = make_bus([1, 2, 3])
    settle(bus)
    bus.dead.add(1)
    bus.run_until(lambda: bus.masters() == [2], timeout=4.0)
    bus.dead.discard(1)
    e1 = bus.elections[1]
    e1.role, e1.master_id = Role.FOLLOWER, 0   # restarted drone: follower, knows nothing
    bus.run(3.0)
    assert bus.masters() == [2]
    assert e1.master_id == 2


@pytest.mark.parametrize("mine,theirs,their_id,steps_down", [
    (1, 2, 5, True),    # higher term wins
    (2, 2, 1, True),    # equal term, lower ID wins
    (2, 2, 5, False),   # equal term, higher ID loses
    (3, 2, 1, False),   # lower term loses even with a lower ID
])
def test_master_step_down_rule(mine, theirs, their_id, steps_down):
    e = Election(3, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.role, e.term, e.master_id, e.max_term_seen = Role.MASTER, mine, 3, mine
    other = Election(their_id, 1.5, 1.5, 1.0, 0.0, 0.0)
    other.role, other.term, other.master_id = Role.MASTER, theirs, their_id
    e.on_heartbeat(hb_from(other, 1.0), 1.0)
    assert (e.role == Role.FOLLOWER) is steps_down
    if steps_down:
        assert e.master_id == their_id and e.term == theirs


def test_follower_switches_to_better_master_and_ignores_worse():
    e = Election(5, 1.5, 1.5, 1.0, 0.0, 0.0)
    m_a = Election(2, 1.5, 1.5, 1.0, 0.0, 0.0)
    m_a.role, m_a.term, m_a.master_id = Role.MASTER, 3, 2
    m_b = Election(4, 1.5, 1.5, 1.0, 0.0, 0.0)
    m_b.role, m_b.term, m_b.master_id = Role.MASTER, 2, 4
    e.on_heartbeat(hb_from(m_b, 1.0), 1.0)
    assert e.master_id == 4
    e.on_heartbeat(hb_from(m_a, 1.1), 1.1)
    assert e.master_id == 2 and e.term == 3
    e.on_heartbeat(hb_from(m_b, 1.2), 1.2)
    assert e.master_id == 2
    e.update(1.3)
    assert e.master_id == 2


def test_master_stepping_down_clears_follower_master():
    e = Election(5, 1.5, 1.5, 1.0, 0.0, 0.0)
    m = Election(2, 1.5, 1.5, 1.0, 0.0, 0.0)
    m.role, m.term, m.master_id = Role.MASTER, 1, 2
    e.on_heartbeat(hb_from(m, 1.0), 1.0)
    assert e.master_id == 2
    m.role = Role.FOLLOWER
    e.on_heartbeat(hb_from(m, 1.1), 1.1)
    assert e.master_id == 0


def test_planned_handover_on_low_battery():
    bus = make_bus([1, 2, 3])
    settle(bus)
    t0 = bus.t
    bus.elections[1].start_retire(bus.t)
    took = bus.run_until(lambda: bus.masters() == [2] and bus.elections[1].role == Role.RETIRED
                         and bus.elections[3].master_id == 2, timeout=2.0)
    assert took < 1.0
    assert bus.elections[2].term == 2
    kinds = [ev.kind for ev in bus.elections[1].events]
    assert "handover_named" in kinds and "step_down" in kinds
    assert [ev.detail["reason"] for ev in bus.elections[2].events if ev.kind == "claim"] == ["handover"]
    assert bus.t - t0 < 1.0


def test_handover_skips_unresponsive_successor():
    bus = make_bus([1, 2, 3])
    settle(bus)
    bus.blocked |= {(1, 2)}  # 2 never hears the handover request
    bus.elections[1].start_retire(bus.t)
    bus.run_until(lambda: bus.elections[1].role == Role.RETIRED, timeout=3.0)
    assert bus.elections[1].events[-2].kind in ("handover_named", "step_down")
    named = [ev.detail["successor"] for ev in bus.elections[1].events if ev.kind == "handover_named"]
    assert named == [2, 3]


def test_handover_with_nobody_left_retires_immediately():
    e = Election(1, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.role, e.term, e.master_id = Role.MASTER, 1, 1
    e.start_retire(5.0)
    assert e.role == Role.RETIRED
    assert e.events[-1].kind == "handover_failed"
    e.start_retire(6.0)  # idempotent
    assert e.role == Role.RETIRED


def test_follower_with_low_battery_retires_and_never_claims():
    e = Election(1, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.start_retire(1.0)
    assert e.role == Role.RETIRED and not e.eligible
    e.update(10.0)
    assert e.role == Role.RETIRED
    e.set_eligible(True)
    assert not e.eligible
    assert not (e.flags(10.0) & Flag.ELIGIBLE)


def test_retired_drone_ignores_master_heartbeats():
    e = Election(4, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.start_retire(0.5)
    m = Election(2, 1.5, 1.5, 1.0, 0.0, 0.0)
    m.role, m.term, m.master_id = Role.MASTER, 1, 2
    e.on_heartbeat(hb_from(m, 1.0), 1.0)
    assert e.role == Role.RETIRED and e.master_id == 0


def test_ineligible_drone_does_not_claim_and_does_not_accept_handover():
    e = Election(2, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.set_eligible(False)
    e.update(5.0)
    assert e.role == Role.FOLLOWER
    m = Election(1, 1.5, 1.5, 1.0, 0.0, 0.0)
    m.role, m.term, m.master_id, m.handover_to = Role.MASTER, 1, 1, 2
    e.on_heartbeat(hb_from(m, 5.1), 5.1)
    assert e.role == Role.FOLLOWER and e.master_id == 1


def test_members_only_followers_and_self():
    e = Election(1, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.role, e.term, e.master_id = Role.MASTER, 2, 1
    for did, role in ((2, Role.FOLLOWER), (3, Role.RETIRED), (4, Role.MASTER), (5, Role.FOLLOWER)):
        other = Election(did, 1.5, 1.5, 1.0, 0.0, 0.0)
        other.role, other.term = role, 1
        e.peers[did] = PeerInfo(hb_from(other, 1.0), 1.0)
    assert e.members(1.5) == frozenset({1, 2, 5})
    assert e.members(3.0) == frozenset({1})  # everyone timed out


def test_stale_master_peer_does_not_block_claim():
    """peer_timeout longer than master_timeout: a dead master still 'alive' as a peer is ignored."""
    e = Election(2, 1.5, 3.0, 1.0, 0.0, 0.0)
    m = Election(1, 1.5, 3.0, 1.0, 0.0, 0.0)
    m.role, m.term, m.master_id = Role.MASTER, 1, 1
    e.on_heartbeat(hb_from(m, 1.0), 1.0)
    e.update(1.1)
    assert e.master_id == 1
    e.update(2.6)  # master heartbeat 1.6 s old: dead as master, still within peer timeout
    assert e.role == Role.MASTER and e.term == 2


def test_own_heartbeat_ignored_and_flags():
    e = Election(1, 1.5, 1.5, 1.0, 0.0, 0.0)
    e.on_heartbeat(hb_from(e, 0.1), 0.1)
    assert e.peers == {}
    assert e.flags(0.2) == Flag.ELIGIBLE
    e.update(3.1)
    assert e.role == Role.MASTER
    assert e.flags(3.2) & Flag.MASTER_OK
    assert e.master_info(3.2) is None  # masters follow nobody


def test_latency_does_not_break_convergence():
    bus = make_bus([1, 2, 3, 4], latency=0.3)
    settle(bus, 5.0)
    assert bus.masters() == [1]
    bus.dead.add(1)
    took = bus.run_until(lambda: bus.masters() == [2] and all(
        bus.elections[i].master_id == 2 for i in (3, 4)), timeout=5.0)
    assert took < 3.0


def test_first_election_waits_for_whole_fleet_so_lowest_id_wins():
    """Agents boot seconds apart (PX4 runs showed this): drone 1 last must still become master."""
    bus = make_bus([1, 2, 3], boot={1: 4.0, 2: 0.0, 3: 0.0})
    for e in bus.elections.values():
        e.expected_ids, e.startup_timeout = frozenset({1, 2, 3}) - {e.my_id}, 30.0
    bus.dead.add(1)
    bus.run(4.0)                       # 2 and 3 are up, 1 is not: nobody claims yet
    assert bus.masters() == []
    bus.dead.discard(1)
    bus.run(4.0)
    assert bus.masters() == [1]


def test_first_election_proceeds_after_startup_timeout_without_missing_drone():
    bus = make_bus([1, 2, 3])
    for e in bus.elections.values():
        e.expected_ids, e.startup_timeout = frozenset({1, 2, 3}) - {e.my_id}, 10.0
    bus.dead.add(1)                    # drone 1 never starts
    bus.run(9.0)
    assert bus.masters() == []
    bus.run(2.0)
    assert bus.masters() == [2]
