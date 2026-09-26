"""Leader election state machine for one drone (pure Python, no ROS).

Rules (docs/SPECIFICATION.md §5/§6) and the design choices that complete them (reports/PHASE_2.md):
 1. Each drone keeps a `term`. A drone that becomes master takes term = highest term seen + 1.
 2. A master steps down when it hears another master with a higher term, or an equal term and a
    lower ID. Among masters that can hear each other, exactly one survives (after a partition
    heals there is one master).
 3. The master is dead after master_timeout_s without its heartbeat. Followers then hover
    (no master heard, no election result yet).
 4. Election: the lowest-ID alive *eligible* drone claims. A drone does NOT claim while
    (a) a lower-ID eligible drone is alive (wait for it to claim), or
    (b) any alive peer still reports hearing a live master (MASTER_OK) — the fault may be this
        drone's own link rather than the master, so claiming would cause a false failover.
 5. No preemption: a recovered lower-ID drone does not take leadership back from a live master.
 6. Planned handover (low battery): the master names the lowest-ID eligible peer in
    `handover_to` and keeps leading; that drone claims at once (rule 1 gives it a higher term),
    the old master hears it, steps down (rule 2) and retires. If nobody takes over within
    handover_timeout_s the next candidate is named.
 7. Startup: nobody claims before startup_listen_s, so drones first learn who else is alive.
Peer and master liveness use the LOCAL receive time; timestamps in heartbeats are only used for
extrapolating positions.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .heartbeat import Flag, Heartbeat, Role


@dataclass(slots=True)
class PeerInfo:
    hb: Heartbeat
    rx_time: float


@dataclass(slots=True)
class ElectionEvent:
    t: float
    kind: str        # claim | step_down | follow | master_lost | retire | handover_named
    detail: dict = field(default_factory=dict)


def _better(term_a: int, id_a: int, term_b: int, id_b: int) -> bool:
    """True if master (term_a, id_a) outranks master (term_b, id_b) (rule 2)."""
    return term_a > term_b or (term_a == term_b and id_a < id_b)


class Election:
    def __init__(self, my_id: int, master_timeout_s: float, peer_timeout_s: float,
                 handover_timeout_s: float, startup_listen_s: float, now: float) -> None:
        self.my_id = my_id
        self.master_timeout = master_timeout_s
        self.peer_timeout = peer_timeout_s
        self.handover_timeout = handover_timeout_s
        self.startup_listen = startup_listen_s
        self.boot_time = now
        self.role = Role.FOLLOWER
        self.term = 0              # my term if master, else the term of the master I follow
        self.max_term_seen = 0
        self.master_id = 0         # 0 = no live master known
        self.peers: dict[int, PeerInfo] = {}
        self.eligible = True       # set by the agent: ready and battery above handover level
        self.retiring = False
        self.handover_to = 0
        self._handover_started = 0.0
        self._handover_tried: set[int] = set()
        self.changed = False       # role/master changed since last heartbeat -> send one now
        self.events: list[ElectionEvent] = []

    # ------------------------------------------------------------------ inputs
    def set_eligible(self, value: bool) -> None:
        self.eligible = value and not self.retiring

    def on_heartbeat(self, hb: Heartbeat, now: float) -> None:
        if hb.drone_id == self.my_id:
            return
        self.peers[hb.drone_id] = PeerInfo(hb, now)
        if hb.term > self.max_term_seen:
            self.max_term_seen = hb.term
        if hb.role != Role.MASTER:
            if hb.drone_id == self.master_id and self.role != Role.MASTER:
                self.master_id = 0   # my master stepped down; update() picks the next one
            return
        if self.role == Role.MASTER:
            if _better(hb.term, hb.drone_id, self.term, self.my_id):
                self._step_down(now, hb)
        elif self.role == Role.FOLLOWER:
            if hb.handover_to == self.my_id and self.eligible:
                self._claim(now, "handover", from_master=hb.drone_id)
                return
            self._consider_master(hb, now)

    def start_retire(self, now: float) -> None:
        """Battery low: stop being a candidate; a master hands over first (rule 6)."""
        if self.retiring:
            return
        self.retiring = True
        self.eligible = False
        self.events.append(ElectionEvent(now, "retire", {"role": self.role.name}))
        if self.role == Role.MASTER:
            self._name_successor(now)
        else:
            self.role = Role.RETIRED
            self.master_id = 0
            self.changed = True

    # ------------------------------------------------------------------ periodic
    def update(self, now: float) -> None:
        if self.role == Role.MASTER:
            if self.retiring and self.handover_to and now - self._handover_started > self.handover_timeout:
                self._name_successor(now)
            return
        if self.role == Role.RETIRED:
            return
        best = self._best_live_master(now)
        if best is not None:
            if best.drone_id != self.master_id:
                self.events.append(ElectionEvent(now, "follow", {"master": best.drone_id, "term": best.term}))
                self.changed = True
            self.master_id = best.drone_id
            self.term = best.term
            return
        if self.master_id != 0:
            self.events.append(ElectionEvent(now, "master_lost", {"master": self.master_id}))
            self.master_id = 0
            self.changed = True
        if self._may_claim(now):
            self._claim(now, "election")

    # ------------------------------------------------------------------ queries
    def alive_peers(self, now: float) -> list[PeerInfo]:
        return [p for p in self.peers.values() if now - p.rx_time < self.peer_timeout]

    def members(self, now: float) -> frozenset[int]:
        """Drones this (master) drone currently hears that fly in formation as followers, plus
        itself. Other masters (e.g. a retiring one mid-handover) and retired drones are excluded."""
        return frozenset([self.my_id] + [p.hb.drone_id for p in self.alive_peers(now) if p.hb.role == Role.FOLLOWER])

    def master_info(self, now: float) -> PeerInfo | None:
        """Latest heartbeat of the master this drone follows, if that master is live."""
        if self.role != Role.FOLLOWER or self.master_id == 0:
            return None
        p = self.peers.get(self.master_id)
        if p is None or now - p.rx_time >= self.master_timeout or p.hb.role != Role.MASTER:
            return None
        return p

    def hears_live_master(self, now: float) -> bool:
        return self.role == Role.MASTER or self.master_info(now) is not None

    def flags(self, now: float) -> int:
        f = 0
        if self.hears_live_master(now):
            f |= Flag.MASTER_OK
        if self.eligible and not self.retiring:
            f |= Flag.ELIGIBLE
        return f

    # ------------------------------------------------------------------ internals
    def _best_live_master(self, now: float) -> Heartbeat | None:
        best: Heartbeat | None = None
        for p in self.peers.values():
            hb = p.hb
            if hb.role != Role.MASTER or now - p.rx_time >= self.master_timeout:
                continue
            if best is None or _better(hb.term, hb.drone_id, best.term, best.drone_id):
                best = hb
        return best

    def _consider_master(self, hb: Heartbeat, now: float) -> None:
        current = self.master_info(now)
        if current is None or hb.drone_id == self.master_id or _better(
                hb.term, hb.drone_id, current.hb.term, current.hb.drone_id):
            if hb.drone_id != self.master_id:
                self.events.append(ElectionEvent(now, "follow", {"master": hb.drone_id, "term": hb.term}))
                self.changed = True
            self.master_id = hb.drone_id
            self.term = hb.term

    def _may_claim(self, now: float) -> bool:
        if not self.eligible or self.retiring or now - self.boot_time < self.startup_listen:
            return False
        for p in self.alive_peers(now):
            hb = p.hb
            if hb.role != Role.FOLLOWER:
                # RETIRED drones never lead; a MASTER here is stale (a live one would be followed).
                continue
            if hb.has(Flag.MASTER_OK):
                return False
            if hb.drone_id < self.my_id and hb.has(Flag.ELIGIBLE):
                return False
        return True

    def _claim(self, now: float, reason: str, **detail) -> None:
        self.role = Role.MASTER
        self.term = self.max_term_seen + 1
        self.max_term_seen = self.term
        self.master_id = self.my_id
        self.handover_to = 0
        self.changed = True
        self.events.append(ElectionEvent(now, "claim", {"term": self.term, "reason": reason, **detail}))

    def _step_down(self, now: float, winner: Heartbeat) -> None:
        self.events.append(ElectionEvent(now, "step_down", {"to": winner.drone_id, "term": winner.term,
                                                            "my_term": self.term}))
        self.handover_to = 0
        self.term = winner.term
        if self.retiring:
            self.role = Role.RETIRED
            self.master_id = 0
        else:
            self.role = Role.FOLLOWER
            self.master_id = winner.drone_id
        self.changed = True

    def _name_successor(self, now: float) -> None:
        candidates = sorted(
            p.hb.drone_id for p in self.alive_peers(now)
            if p.hb.role != Role.RETIRED and p.hb.has(Flag.ELIGIBLE) and p.hb.drone_id not in self._handover_tried
        )
        if not candidates:
            # Nobody to hand over to: retire now; the rest elect normally after the timeout.
            self.events.append(ElectionEvent(now, "handover_failed", {"tried": sorted(self._handover_tried)}))
            self.role = Role.RETIRED
            self.master_id = 0
            self.handover_to = 0
            self.changed = True
            return
        self.handover_to = candidates[0]
        self._handover_tried.add(self.handover_to)
        self._handover_started = now
        self.changed = True
        self.events.append(ElectionEvent(now, "handover_named", {"successor": self.handover_to}))
