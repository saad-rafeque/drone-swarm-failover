"""Telemetry-radio stand-in (Phase 6): a UDP relay between one simulated drone's PX4 and its MAVROS that
behaves like a pair of SiK telemetry radios, so the swarm can be tested with one drone on a radio-like link
before a real drone is flown (central test tool, never onboard).

    PX4 instance i  --(sends to 14540+i)-->  relay  --(to mavros_port_base+i)-->  MAVROS
    PX4 instance i  <--(to 14580+i)--------  relay  <--(sent to relay_port_base+i)--  MAVROS

Link model, per packet (settings in config/swarm.yaml, radio_standin):
  * lost in the air with probability loss_pct;
  * both directions share one air channel at the usable rate (AIR_SPEED, halved with ECC), as SiK's
    time-division turns do; a packet waits for everything queued before it, in either direction;
  * when the channel is idle, a packet first waits for its side's transmit turn: uniform 0..MAX_WINDOW;
  * a packet that would wait longer than max_queue_s is dropped (the radio's buffer is full).
Order within each direction is kept. This models the link's rate, turns, loss and buffer, not the radio's
framing or retransmissions.

Usage (started by SimLauncher for drones listed in a profile with link "standin"):
  python3 -m swarm_tools.radio_proxy --instance 0 --stats <run_dir>/proc_logs/radio_standin_1.json
"""
from __future__ import annotations

import argparse
import heapq
import json
import random
import selectors
import signal
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from swarm_agent.config import RadioStandinCfg, default_config_path, load_config

PX4_OFFBOARD_LOCAL_BASE = 14580    # PX4 listens here (+instance); same plan as swarm_tools.sim_launch
PX4_OFFBOARD_REMOTE_BASE = 14540   # PX4 sends here (+instance)
HOST = "127.0.0.1"


@dataclass
class DirStats:
    packets_in: int = 0
    bytes_in: int = 0
    delivered: int = 0
    bytes_delivered: int = 0
    lost_in_air: int = 0
    dropped_queue_full: int = 0
    delay_sum_s: float = 0.0
    delay_max_s: float = 0.0

    def as_dict(self) -> dict:
        d = dict(self.__dict__)
        d["delay_mean_ms"] = round(1000.0 * self.delay_sum_s / self.delivered, 1) if self.delivered else None
        d["delay_max_ms"] = round(1000.0 * self.delay_max_s, 1)
        del d["delay_sum_s"], d["delay_max_s"]
        return d


@dataclass
class RadioRelay:
    cfg: RadioStandinCfg
    px4_bind: tuple[str, int]      # where PX4 sends to
    px4_addr: tuple[str, int]      # where PX4 listens
    mavros_bind: tuple[str, int]   # where MAVROS sends to
    mavros_addr: tuple[str, int]   # where MAVROS listens
    stats_path: Path | None = None
    up: DirStats = field(default_factory=DirStats)     # MAVROS -> PX4 (commands, setpoints)
    down: DirStats = field(default_factory=DirStats)   # PX4 -> MAVROS (telemetry)

    def __post_init__(self) -> None:
        self.rng = random.Random(self.cfg.seed)
        self.rate_bytes_s = self.cfg.usable_rate_bps / 8.0
        self.s_px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s_px4.bind(self.px4_bind)
        self.s_mav = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s_mav.bind(self.mavros_bind)
        self.channel_free_at = 0.0
        self.queue: list[tuple[float, int, int, bytes]] = []   # (deliver_at, seq, direction, data)
        self.seq = 0
        self.arrival: dict[int, float] = {}          # seq -> when the relay received it
        self.t0 = time.monotonic()
        self.stop_event = threading.Event()

    @property
    def px4_port(self) -> int:
        return self.s_px4.getsockname()[1]

    @property
    def mavros_port(self) -> int:
        return self.s_mav.getsockname()[1]

    def _admit(self, data: bytes, direction: int, now: float) -> None:
        st = self.up if direction == 0 else self.down
        st.packets_in += 1
        st.bytes_in += len(data)
        if self.rng.random() * 100.0 < self.cfg.loss_pct:
            st.lost_in_air += 1
            return
        if now >= self.channel_free_at:            # idle channel: wait for this side's transmit turn
            start = now + self.rng.uniform(0.0, self.cfg.max_window_ms / 1000.0)
        else:                                      # busy: queue behind everything already accepted
            start = self.channel_free_at
        deliver = start + len(data) / self.rate_bytes_s
        if deliver - now > self.cfg.max_queue_s:
            st.dropped_queue_full += 1
            return
        self.channel_free_at = deliver
        self.seq += 1
        heapq.heappush(self.queue, (deliver, self.seq, direction, data))
        self.arrival[self.seq] = now

    def _deliver_due(self, now: float) -> None:
        while self.queue and self.queue[0][0] <= now:
            _, seq, direction, data = heapq.heappop(self.queue)
            if direction == 0:
                self.s_px4.sendto(data, self.px4_addr)
                st = self.up
            else:
                self.s_mav.sendto(data, self.mavros_addr)
                st = self.down
            delay = now - self.arrival.pop(seq)
            st.delivered += 1
            st.bytes_delivered += len(data)
            st.delay_sum_s += delay
            st.delay_max_s = max(st.delay_max_s, delay)

    def stats(self) -> dict:
        return {"model": {"usable_rate_bps": self.cfg.usable_rate_bps, "max_window_ms": self.cfg.max_window_ms,
                          "loss_pct": self.cfg.loss_pct, "max_queue_s": self.cfg.max_queue_s},
                "uptime_s": round(time.monotonic() - self.t0, 1),
                "mavros_to_px4": self.up.as_dict(), "px4_to_mavros": self.down.as_dict()}

    def write_stats(self) -> None:
        if self.stats_path is not None:
            self.stats_path.parent.mkdir(parents=True, exist_ok=True)
            self.stats_path.write_text(json.dumps(self.stats(), indent=1) + "\n")

    def run(self) -> None:
        sel = selectors.DefaultSelector()
        sel.register(self.s_px4, selectors.EVENT_READ, 1)     # from PX4: goes down to MAVROS
        sel.register(self.s_mav, selectors.EVENT_READ, 0)     # from MAVROS: goes up to PX4
        next_stats = time.monotonic() + 5.0
        try:
            while not self.stop_event.is_set():
                now = time.monotonic()
                timeout = min(0.05, max(0.0, self.queue[0][0] - now)) if self.queue else 0.05
                for key, _ in sel.select(timeout):
                    data, _src = key.fileobj.recvfrom(65535)
                    self._admit(data, key.data, time.monotonic())
                self._deliver_due(time.monotonic())
                if time.monotonic() >= next_stats:
                    self.write_stats()
                    next_stats += 5.0
        finally:
            self.write_stats()
            sel.close()
            self.s_px4.close()
            self.s_mav.close()


def for_instance(cfg: RadioStandinCfg, instance: int, stats_path: Path | None = None) -> RadioRelay:
    """The relay for PX4 instance `instance`, with the port plan of sim_launch and config/swarm.yaml."""
    return RadioRelay(cfg,
                      px4_bind=(HOST, PX4_OFFBOARD_REMOTE_BASE + instance),
                      px4_addr=(HOST, PX4_OFFBOARD_LOCAL_BASE + instance),
                      mavros_bind=(HOST, cfg.relay_port_base + instance),
                      mavros_addr=(HOST, cfg.mavros_port_base + instance),
                      stats_path=stats_path)


def standin_fcu_url(cfg: RadioStandinCfg, instance: int) -> str:
    """MAVROS fcu_url of a stand-in drone: bind its own port, send to the relay."""
    return f"udp://:{cfg.mavros_port_base + instance}@{HOST}:{cfg.relay_port_base + instance}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--instance", type=int, required=True, help="PX4 instance number (drone ID - 1)")
    ap.add_argument("--config", default=str(default_config_path()))
    ap.add_argument("--stats", help="write link statistics (JSON) here every 5 s and at exit")
    args = ap.parse_args()
    relay = for_instance(load_config(args.config).radio_standin, args.instance,
                         Path(args.stats) if args.stats else None)
    signal.signal(signal.SIGTERM, lambda *_: relay.stop_event.set())
    signal.signal(signal.SIGINT, lambda *_: relay.stop_event.set())
    print(f"radio stand-in for PX4 instance {args.instance}: PX4 side {relay.px4_bind[1]}, "
          f"MAVROS side {relay.mavros_bind[1]}, {relay.cfg.usable_rate_bps:.0f} bit/s", flush=True)
    relay.run()


if __name__ == "__main__":
    main()
