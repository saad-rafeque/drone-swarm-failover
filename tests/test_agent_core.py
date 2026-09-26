"""Mission-level behaviour of AgentCore, exercised in the point-mass simulator (no PX4/ROS)."""
from __future__ import annotations

import math

import pytest

from swarm_agent.agent_core import AgentCore, FlightMode, OwnState
from swarm_agent.geometry import ZERO, norm_xy, sub
from swarm_agent.heartbeat import Phase, Role
from swarm_tools.puresim import PureSim

GOAL_TOL_M = 60.0  # every drone lands in its own slot around the goal


def at_goal(sim: PureSim, i: int) -> bool:
    gx, gy = sim.cfg.mission.goal_enu_m
    d = sim.drones[i]
    return d.landed and math.hypot(d.pos[0] - gx, d.pos[1] - gy) < GOAL_TOL_M


class Recorder:
    def __init__(self) -> None:
        self.phases: list[Phase] = []
        self.rms: list[float] = []
        self.masters_seen: set[int] = set()

    def __call__(self, s: PureSim, st) -> None:
        for m in st.masters:
            self.masters_seen.add(m)
            ph = s.agents[m].phase
            if not self.phases or self.phases[-1] != ph:
                self.phases.append(ph)
        if st.formation_rms is not None and s.agents[st.masters[0]].phase == Phase.CRUISE \
                and s.t - s.agents[st.masters[0]].phase_since > 10.0:
            self.rms.append(st.formation_rms)


def test_nominal_mission_three_drones(cfg):
    sim = PureSim(cfg.with_num_drones(3), seed=11)
    rec = Recorder()
    sim.run_until(300.0, rec)
    assert rec.phases == [Phase.TAKEOFF, Phase.CRUISE, Phase.HOLD, Phase.LAND]
    assert rec.masters_seen == {1}
    assert all(at_goal(sim, i) for i in sim.alive_ids())
    assert sim.min_sep_seen >= cfg.safety.min_separation_m
    assert max(rec.rms) < 2.0


def fault_sim(cfg, n: int, seed: int = 3) -> PureSim:
    sim = PureSim(cfg.with_num_drones(n), seed=seed)
    sim.run_until(60.0)  # well into cruise
    assert sim.masters() == [1] and sim.agents[1].phase == Phase.CRUISE
    return sim


def time_to(sim: PureSim, cond, timeout: float) -> float:
    t0 = sim.t
    while not cond():
        assert sim.t - t0 < timeout, "condition not met in time"
        sim.step()
    return sim.t - t0


def finish(sim: PureSim) -> None:
    sim.run_until(330.0)
    flyers = [i for i in sim.alive_ids() if sim.role(i) != Role.RETIRED]
    assert all(at_goal(sim, i) for i in flyers)
    assert sim.min_sep_seen >= sim.cfg.safety.min_separation_m


def test_f1_master_killed(cfg):
    sim = fault_sim(cfg, 5)
    sim.kill(1)
    took = time_to(sim, lambda: sim.masters() == [2] and sim.converged(), 4.0)
    assert took < 3.0
    finish(sim)


def test_f2_master_link_lost_old_master_becomes_orphan(cfg):
    sim = fault_sim(cfg, 5)
    sim.net.block_tx(1, sim.alive_ids())
    time_to(sim, lambda: sim.masters() == [2], 4.0)
    time_to(sim, lambda: sim.agents[1].orphan and sim.agents[1].election.master_id == 2, 4.0)
    sim.run_until(sim.t + 15.0)
    layer = sim.drones[2].pos[2] + cfg.formation.orphan_alt_offset_m
    assert abs(sim.drones[1].pos[2] - layer) < 1.0      # flies on the orphan layer
    sim.net.heal()
    time_to(sim, lambda: not sim.agents[1].orphan, 4.0)  # rejoins once heard again
    assert sim.masters() == [2]                          # no preemption
    finish(sim)


def test_f3_low_battery_handover_and_return_home(cfg):
    sim = fault_sim(cfg, 5)
    sim.set_battery(1, cfg.battery.handover_pct - 1.0)
    took = time_to(sim, lambda: sim.masters() == [2] and sim.converged(), 2.0)
    assert took < 1.0
    time_to(sim, lambda: sim.agents[1].retire_stage == "transit", 15.0)
    assert sim.drones[1].pos[2] >= cfg.mission.cruise_alt_m + cfg.battery.retire_alt_offset_m - 1.5
    finish(sim)
    home = sim.agents[1].home
    assert sim.drones[1].landed and norm_xy(sub(sim.drones[1].pos, home)) < cfg.mission.goal_radius_m + 1.0


def test_f4_follower_killed_no_master_change(cfg):
    sim = fault_sim(cfg, 5)
    sim.kill(4)
    sim.run_until(sim.t + 20.0)
    assert sim.masters() == [1] and sim.agents[1].election.term == 1
    assert sim.formation_rms() < 2.0
    finish(sim)


def test_f5_partition_then_heal_single_master(cfg):
    sim = fault_sim(cfg, 6)
    sim.net.partition([{1, 2, 3}, {4, 5, 6}])
    sim.run_until(sim.t + 10.0)
    assert sorted(sim.masters()) == [1, 4]
    sim.net.heal()
    took = time_to(sim, lambda: sim.converged(), 3.0)
    assert took < 1.0 and sim.masters() == [4]
    finish(sim)


def test_low_battery_follower_retires_home(cfg):
    sim = fault_sim(cfg, 4)
    sim.set_battery(3, 10.0)
    sim.run_until(sim.t + 1.0)
    assert sim.role(3) == Role.RETIRED and sim.masters() == [1]
    finish(sim)
    assert norm_xy(sub(sim.drones[3].pos, sim.agents[3].home)) < cfg.mission.goal_radius_m + 1.0


def test_master_waits_for_missing_drone_until_startup_timeout(cfg):
    sim = PureSim(cfg.with_num_drones(3), seed=5)
    sim.kill(3)                       # never boots
    sim.run_until(cfg.mission.startup_timeout_s - 1.0)
    assert sim.agents[1].phase == Phase.IDLE and sim.masters() == []
    # first election and takeoff both wait for the startup timeout after drone 1's own boot (<= 5 s)
    sim.run_until(cfg.mission.startup_timeout_s + sim.boot[1] + 1.0)
    assert sim.masters() == [1] and sim.agents[1].phase == Phase.TAKEOFF


def test_heartbeat_rate_and_immediate_send_on_role_change(cfg):
    c = cfg.with_num_drones(1)
    core = AgentCore(c, 1, ZERO, 0.0)
    own = OwnState(ZERO, ZERO, 90.0, ready=True, landed=True)
    sent = [t / 100 for t in range(0, 1001) if core.step(own, t / 100)[1] is not None]
    assert 49 <= len([t for t in sent if t < 10.0]) <= 52   # 5 Hz, plus one when it becomes master
    became_master = next(ev.t for ev in core.election.events if ev.kind == "claim")
    assert any(abs(t - became_master) < 1e-9 for t in sent)


def test_commands_by_phase_and_geofence(cfg):
    c = cfg.with_num_drones(1)
    core = AgentCore(c, 1, ZERO, 0.0)
    ground = OwnState(ZERO, ZERO, 90.0, ready=True, landed=True)
    assert core.step(ground, 0.1)[0].mode == FlightMode.GROUND      # IDLE
    core.step(ground, 3.2)                                          # claims, all 1 drones ready
    assert core.phase == Phase.TAKEOFF
    cmd = core.step(ground, 3.3)[0]
    assert cmd.mode == FlightMode.OFFBOARD and cmd.vel[2] > 0 and cmd.reason == "takeoff"
    core.phase = Phase.CRUISE
    outside = OwnState((c.safety.geofence_radius_m + 20.0, 0.0, 30.0), ZERO, 90.0, True, False)
    cmd = core.step(outside, 4.0)[0]
    assert cmd.vel[0] < 0.0                                           # pushed back inside
    core.phase = Phase.LAND
    assert core.step(outside, 4.1)[0].mode == FlightMode.LAND


def test_late_joiner_climbs_before_joining(cfg):
    sim = fault_sim(cfg, 3)
    d = sim.drones[3]
    d.pos, d.vel, d.landed = (d.pos[0], d.pos[1], 0.0), ZERO, True
    sim.agents[3]._joined = False
    sim.step()
    assert sim.agents[3].last_cmd.reason == "climb_to_join"
    time_to(sim, lambda: sim.agents[3].last_cmd.reason in ("formation", "transit"), 30.0)


def test_brief_member_dropout_keeps_slot(cfg):
    """A follower missing from the member list for < 1 s keeps its slot instead of going orphan."""
    sim = fault_sim(cfg, 3)
    sim.net.block_tx(3, [1])
    # master drops 3 from its members ~1.5 s after its last heartbeat arrived; 3 learns that from
    # the master's next heartbeat (<= 0.2 s) and only goes orphan after ORPHAN_DELAY_S = 1 s more
    sim.run_until(sim.t + 2.5)
    assert not sim.agents[3].orphan and sim.agents[3]._not_member_since is not None
    sim.run_until(sim.t + 1.0)
    assert sim.agents[3].orphan
    sim.net.heal()
    sim.run_until(sim.t + 0.5)
    assert not sim.agents[3].orphan


@pytest.mark.parametrize("latency,loss", [(0.15, 0.1), (0.3, 0.3)])
def test_mission_completes_on_lossy_slow_links(cfg, latency, loss):
    sim = PureSim(cfg.with_num_drones(5), seed=21, latency_s=latency, jitter_s=latency * 0.2, loss=loss)
    sim.run_until(330.0)
    assert all(at_goal(sim, i) for i in sim.alive_ids())
    assert sim.min_sep_seen >= cfg.safety.min_separation_m
