"""Telemetry-radio stand-in (swarm_tools.radio_proxy): shared rate, transmit-turn wait, loss, full buffer and
packet order, with local UDP sockets in place of PX4 and MAVROS (no simulator)."""
from __future__ import annotations

import dataclasses
import selectors
import socket
import threading
import time

from swarm_tools.radio_proxy import RadioRelay, standin_fcu_url
from swarm_tools.sim_launch import assert_simulated_url


def endpoint() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(0.02)
    return s


class Link:
    """A relay between two local sockets: `px4` stands in for PX4, `mav` for MAVROS."""

    def __init__(self, cfg, **changes):
        self.px4, self.mav = endpoint(), endpoint()
        self.relay = RadioRelay(dataclasses.replace(cfg.radio_standin, **changes),
                                px4_bind=("127.0.0.1", 0), px4_addr=self.px4.getsockname(),
                                mavros_bind=("127.0.0.1", 0), mavros_addr=self.mav.getsockname())
        self.thread = threading.Thread(target=self.relay.run, daemon=True)
        self.thread.start()

    def down(self, data: bytes) -> None:                 # PX4 -> MAVROS
        self.px4.sendto(data, ("127.0.0.1", self.relay.px4_port))

    def up(self, data: bytes) -> None:                   # MAVROS -> PX4
        self.mav.sendto(data, ("127.0.0.1", self.relay.mavros_port))

    @staticmethod
    def collect(sock: socket.socket, seconds: float) -> list[tuple[float, bytes]]:
        return Link.collect_both([sock], seconds)[0]

    @staticmethod
    def collect_both(socks: list[socket.socket], seconds: float) -> list[list[tuple[float, bytes]]]:
        """Arrival times as they happen, on every socket at once."""
        sel, got = selectors.DefaultSelector(), [[] for _ in socks]
        for k, s in enumerate(socks):
            sel.register(s, selectors.EVENT_READ, k)
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            for key, _ in sel.select(left):
                data, _ = key.fileobj.recvfrom(65535)
                got[key.data].append((time.monotonic(), data))
        sel.close()
        return got

    def close(self) -> None:
        self.relay.stop_event.set()
        self.thread.join(timeout=2.0)
        self.px4.close()
        self.mav.close()


def packet(i: int, size: int = 100) -> bytes:
    return i.to_bytes(4, "big") + bytes(size - 4)


def test_rate_is_the_air_rate_halved_by_error_correction_and_order_is_kept(cfg):
    link = Link(cfg, air_rate_bps=64000.0, ecc=1, max_window_ms=0.0, loss_pct=0.0, max_queue_s=10.0)
    try:
        t0 = time.monotonic()
        for i in range(20):                               # 2,000 bytes at 32 kbit/s (4,000 B/s): 0.5 s
            link.down(packet(i))
        got = Link.collect(link.mav, 1.2)
    finally:
        link.close()
    assert [int.from_bytes(d[:4], "big") for _, d in got] == list(range(20))
    assert 0.45 <= got[-1][0] - t0 <= 0.75


def test_both_directions_share_one_channel(cfg):
    link = Link(cfg, air_rate_bps=64000.0, ecc=1, max_window_ms=0.0, loss_pct=0.0, max_queue_s=10.0)
    try:
        t0 = time.monotonic()
        for i in range(10):
            link.down(packet(i))
            link.up(packet(100 + i))
        got_mav, got_px4 = Link.collect_both([link.mav, link.px4], 0.9)
    finally:
        link.close()
    assert len(got_mav) == 10 and len(got_px4) == 10
    last = max(got_mav[-1][0], got_px4[-1][0])
    assert 0.45 <= last - t0 <= 0.8                       # 2,000 bytes in total, not 1,000 per direction


def test_idle_packets_wait_for_their_transmit_turn(cfg):
    link = Link(cfg, air_rate_bps=6.4e6, ecc=1, max_window_ms=100.0, loss_pct=0.0, max_queue_s=10.0)
    delays = []
    try:
        for i in range(8):
            t = time.monotonic()
            link.up(packet(i))
            got = Link.collect(link.px4, 0.2)
            assert len(got) == 1
            delays.append(got[0][0] - t)
    finally:
        link.close()
    assert max(delays) <= 0.13 and sum(delays) / len(delays) > 0.02


def test_loss_and_statistics(cfg):
    link = Link(cfg, air_rate_bps=6.4e6, ecc=0, max_window_ms=0.0, loss_pct=25.0, max_queue_s=10.0)
    got: list = []
    reader = threading.Thread(target=lambda: got.extend(Link.collect(link.mav, 1.6)))
    try:
        reader.start()
        for i in range(400):                              # paced, so the operating system drops nothing
            link.down(packet(i, 40))
            time.sleep(0.002)
        reader.join()
        stats = link.relay.stats()["px4_to_mavros"]
    finally:
        link.close()
    assert 260 <= len(got) <= 340
    assert stats["packets_in"] == 400 and stats["lost_in_air"] + stats["delivered"] == 400


def test_full_buffer_drops_packets_instead_of_delaying_them(cfg):
    link = Link(cfg, air_rate_bps=64000.0, ecc=1, max_window_ms=0.0, loss_pct=0.0, max_queue_s=0.2)
    try:
        t0 = time.monotonic()
        for i in range(30):                               # 3,000 bytes; 0.2 s holds only about 800
            link.down(packet(i))
        got = Link.collect(link.mav, 0.8)
        stats = link.relay.stats()["px4_to_mavros"]
    finally:
        link.close()
    assert 6 <= len(got) <= 10 and stats["dropped_queue_full"] == 30 - len(got)
    assert got[-1][0] - t0 <= 0.35


def test_standin_url_is_udp(cfg):
    url = standin_fcu_url(cfg.radio_standin, 0)
    assert url == "udp://:15540@127.0.0.1:15580"
    assert_simulated_url(url)
