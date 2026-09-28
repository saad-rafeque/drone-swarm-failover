"""Start/stop N PX4 SIH instances + N MAVROS nodes on this laptop (simulation only).

Verified sources (see reports/PHASE_0.md / PHASE_1.md):
  * PX4 multi-instance start: `px4 -i <n> -d <build>/etc` run from a per-instance working
    directory (Tools/simulation/sitl_multiple_run.sh; docs.px4.io/main/en/sim_sih/).
  * Model: PX4_SIM_MODEL=sihsim_quadx selects airframe 10040_sihsim_quadx (rcS).
  * Home: PX4_HOME_LAT/LON/ALT -> SIH_LOC_LAT0/LON0/H0 (px4-rc.sihsim).
  * Offboard MAVLink link of instance n: PX4 listens on UDP 14580+n and sends to 14540+n
    (px4-rc.mavlink); MAV_SYS_ID = n+1 (rcS).
  * MAVROS: mavros_node with px4_config.yaml as in mavros px4.launch, but our own lean plugin
    list (config/mavros_pluginlists.yaml) instead of px4_pluginlists.yaml.
  * Radio drone (profile link "standin", Phase 6): its PX4 offboard link is restarted like a real
    TELEM1 port: `mavlink stop -u <port>` and `mavlink start ... -r <B/s> [-m <mode>]` through the PX4
    client (`px4-mavlink --instance <n>`, platforms/posix/src/px4/common/main.cpp); PX4 then lowers every
    stream so the link stays under that rate (Mavlink::update_rate_mult, src/modules/mavlink/mavlink_main.cpp).
    Defaults of a real TELEM1: MAV_0_RATE 1200 B/s (src/modules/mavlink/module.yaml). MAVROS time sync on
    that drone runs at radio_standin.timesync_rate_hz (px4_config.yaml /**/time timesync_rate, default 10 Hz).
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import psutil

from swarm_agent.config import Config
from swarm_agent.geometry import GeoPoint

if TYPE_CHECKING:
    from swarm_tools.profiles import Profile

REPO_ROOT = Path(__file__).resolve().parents[2]
MAVROS_SHARE = Path("/opt/ros/jazzy/share/mavros/launch")
MAVROS_NODE = Path("/opt/ros/jazzy/lib/mavros/mavros_node")

# px4-rc.mavlink (PX4 v1.18.0-rc1)
PX4_OFFBOARD_LOCAL_BASE = 14580   # PX4 listens here (+instance)
PX4_OFFBOARD_REMOTE_BASE = 14540  # PX4 sends here (+instance); MAVROS binds it
MAX_INSTANCES = 10                # instances > 9 share remote port 14549 in px4-rc.mavlink


def px4_build_dir(cfg: Config) -> Path:
    return Path(os.path.expanduser(cfg.sim.px4_dir)) / "build" / cfg.sim.build_target


def instance_of(cfg: Config, drone_id: int) -> int:
    return drone_id - cfg.swarm.first_id


def sysid_of(cfg: Config, drone_id: int) -> int:
    return instance_of(cfg, drone_id) + 1  # rcS: MAV_SYS_ID = px4_instance + 1


def sim_fcu_url(cfg: Config, drone_id: int) -> str:
    i = instance_of(cfg, drone_id)
    if not 0 <= i < MAX_INSTANCES:
        raise ValueError(f"instance {i} outside 0..{MAX_INSTANCES - 1} (port plan in px4-rc.mavlink)")
    return f"udp://:{PX4_OFFBOARD_REMOTE_BASE + i}@127.0.0.1:{PX4_OFFBOARD_LOCAL_BASE + i}"


def radio_link_commands(cfg: Config, drone_id: int) -> list[list[str]]:
    """PX4 client commands that turn this drone's offboard link into a telemetry-radio link
    (radio_standin.px4_link_mode / px4_link_rate_bytes_s; same ports and flags as px4-rc.mavlink), then set
    the stream rates the agent asks for over a radio (radio_standin.autopilot_streams_hz), as a real drone's
    link would be set up at boot: Minimal mode has no LOCAL_POSITION_NED until then, and the mission
    start waits for every drone's position."""
    i = instance_of(cfg, drone_id)
    r = cfg.radio_standin
    client = [str(px4_build_dir(cfg) / "bin" / "px4-mavlink"), "--instance", str(i)]
    local, remote = str(PX4_OFFBOARD_LOCAL_BASE + i), str(PX4_OFFBOARD_REMOTE_BASE + i)
    start = client + ["start", "-x", "-u", local, "-r", str(r.px4_link_rate_bytes_s), "-f"]
    if r.px4_link_mode != "normal":        # PX4 has no "-m normal": Normal is what it uses without -m
        start += ["-m", r.px4_link_mode]
    streams = [client + ["stream", "-u", local, "-s", name, "-r", f"{hz:g}"]
               for name, hz in r.autopilot_streams_hz.items()]
    return [client + ["stop", "-u", local], start + ["-o", remote], *streams]


def assert_simulated_url(url: str) -> None:
    """Hard rule 7: never open serial devices from the simulation tools."""
    if not url.startswith("udp://"):
        raise RuntimeError(f"refusing non-UDP fcu_url {url!r}: simulation tools never open serial ports")


def namespace_of(drone_id: int) -> str:
    return f"/uav{drone_id}"


def require_ros_env() -> None:
    if "/opt/ros/jazzy" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        raise RuntimeError("ROS 2 environment not sourced; run through scripts/ros_env.sh")


def is_radio_relay(cmdline: list[str]) -> bool:
    """A telemetry-radio stand-in process: `python -m swarm_tools.radio_proxy ...` (exact module argument)."""
    return any(a == "-m" and b == "swarm_tools.radio_proxy" for a, b in zip(cmdline, cmdline[1:]))


def kill_orphans(timeout_s: float = 5.0) -> list[int]:
    """Kill leftover px4 / mavros_node processes (rule 6), matched by exact process name, and leftover
    telemetry-radio relays, matched by their exact module argument."""
    victims = [p for p in psutil.process_iter(["name", "cmdline"])
               if p.info["name"] in ("px4", "mavros_node") or is_radio_relay(p.info["cmdline"] or [])]
    for p in victims:
        try:
            p.send_signal(signal.SIGTERM)
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(victims, timeout=timeout_s)
    for p in alive:
        try:
            p.kill()
        except psutil.NoSuchProcess:
            pass
    return [p.pid for p in victims]


def _vmhwm_mb(pid: int) -> float:
    """Peak resident set size of one process from /proc/<pid>/status (VmHWM, kB) in MB."""
    try:
        with open(f"/proc/{pid}/status", encoding="ascii") as fh:
            for line in fh:
                if line.startswith("VmHWM:"):
                    return int(line.split()[1]) * 1024 / 1e6
    except FileNotFoundError:
        pass
    return 0.0


@dataclass
class DroneProcs:
    drone_id: int
    px4: subprocess.Popen
    mavros: subprocess.Popen | None = None
    relay: subprocess.Popen | None = None      # telemetry-radio stand-in (profile link "standin")


@dataclass
class SimLauncher:
    """PX4 working dirs go under sim.work_dir (PX4's rcS breaks on paths with spaces, and the
    repo path has one); a fresh timestamped dir per run, so nothing is ever deleted there.
    Process logs go to log_dir inside the repo."""
    cfg: Config
    homes: dict[int, GeoPoint | None]   # None: leave PX4's default home (diagnostics only)
    log_dir: Path = field(default_factory=lambda: REPO_ROOT / ".sim")
    pluginlists: Path = field(default_factory=lambda: REPO_ROOT / "config" / "mavros_pluginlists.yaml")
    procs: dict[int, DroneProcs] = field(default_factory=dict)
    profile: "Profile | None" = None   # config/profiles/*.yaml; None: every drone plain simulated
    work_dir: Path = field(init=False)

    def __post_init__(self) -> None:
        if self.profile is not None and self.profile.real:
            raise RuntimeError(f"profile {self.profile.name!r} has real drones {self.profile.real}: the simulation "
                               "tools never connect to a real flight controller (docs/SPECIFICATION.md, rule 7)")
        base = Path(os.path.expanduser(self.cfg.sim.work_dir))
        if " " in str(base):
            raise ValueError(f"sim.work_dir must not contain spaces (PX4 rcS limitation): {base}")
        self.work_dir = base / time.strftime("%Y%m%d-%H%M%S")
        suffix = 1
        while self.work_dir.exists():
            self.work_dir = base / f"{time.strftime('%Y%m%d-%H%M%S')}-{suffix}"
            suffix += 1

    def start_px4(self, drone_id: int) -> None:
        build = px4_build_dir(self.cfg)
        workdir = self.work_dir / f"px4_{drone_id}"
        workdir.mkdir(parents=True)
        home = self.homes[drone_id]
        env = dict(os.environ)
        env["PX4_SIM_MODEL"] = self.cfg.sim.model
        for key in ("PX4_HOME_LAT", "PX4_HOME_LON", "PX4_HOME_ALT"):
            env.pop(key, None)
        if home is not None:
            env.update(PX4_HOME_LAT=f"{home.lat_deg:.9f}", PX4_HOME_LON=f"{home.lon_deg:.9f}",
                       PX4_HOME_ALT=f"{home.alt_m:.3f}")
        log = open(self.log_dir / f"px4_{drone_id}.log", "w", encoding="utf-8")
        proc = subprocess.Popen(
            [str(build / "bin" / "px4"), "-i", str(instance_of(self.cfg, drone_id)), "-d", str(build / "etc")],
            cwd=workdir, env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.procs[drone_id] = DroneProcs(drone_id, proc)

    def _wait_in_log(self, path: Path, text: str, timeout_s: float, what: str) -> None:
        deadline = time.time() + timeout_s
        while True:
            try:
                if text in path.read_text(encoding="utf-8", errors="replace"):
                    return
            except FileNotFoundError:
                pass
            if time.time() > deadline:
                raise RuntimeError(f"{what}: {text!r} did not appear in {path} within {timeout_s:.0f} s")
            time.sleep(0.2)

    def set_radio_link(self, drone_id: int) -> None:
        """Restart this drone's PX4 offboard MAVLink link with the telemetry-radio settings. PX4 prints the
        client's output in its own log, so the new link's own start line is the proof."""
        px4_log = self.log_dir / f"px4_{drone_id}.log"
        self._wait_in_log(px4_log, "Startup script returned successfully", 60.0, f"PX4 of drone {drone_id}")
        for cmd in radio_link_commands(self.cfg, drone_id):
            subprocess.run(cmd, check=True, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                           stderr=subprocess.STDOUT, timeout=15)
        port = PX4_OFFBOARD_LOCAL_BASE + instance_of(self.cfg, drone_id)
        rate = self.cfg.radio_standin.px4_link_rate_bytes_s
        self._wait_in_log(px4_log, f"data rate: {rate} B/s on udp port {port}", 10.0, f"radio link of drone {drone_id}")

    def mavros_radio_params(self, drone_id: int) -> Path:
        """MAVROS parameters for a drone on a telemetry radio (written next to its logs as evidence)."""
        path = self.log_dir / f"mavros_radio_{drone_id}.yaml"
        path.write_text("/**/time:\n  ros__parameters:\n"
                        f"    timesync_rate: {self.cfg.radio_standin.timesync_rate_hz}\n", encoding="utf-8")
        return path

    def start_relay(self, drone_id: int) -> None:
        """The telemetry-radio stand-in between this drone's PX4 and its MAVROS (swarm_tools.radio_proxy)."""
        i = instance_of(self.cfg, drone_id)
        port = self.cfg.radio_standin.relay_port_base + i
        busy = {c.laddr.port for c in psutil.net_connections(kind="udp") if c.laddr}
        taken = sorted({port, PX4_OFFBOARD_REMOTE_BASE + i} & busy)
        if taken:   # a relay left over from an interrupted run would carry this drone's traffic instead
            raise RuntimeError(f"radio stand-in for drone {drone_id}: UDP port(s) {taken} already in use")
        log = open(self.log_dir / f"radio_standin_{drone_id}.log", "w", encoding="utf-8")
        proc = subprocess.Popen(
            [sys.executable, "-m", "swarm_tools.radio_proxy", "--instance", str(instance_of(self.cfg, drone_id)),
             "--stats", str(self.log_dir / f"radio_standin_{drone_id}.json")],
            stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True,
            env={**os.environ, "PYTHONPATH": str(REPO_ROOT / "src")},
        )
        self.procs[drone_id].relay = proc
        deadline = time.time() + 10.0
        while not any(c.laddr and c.laddr.port == port for c in psutil.net_connections(kind="udp")):
            if proc.poll() is not None or time.time() > deadline:
                raise RuntimeError(f"radio stand-in for drone {drone_id} did not start (see its log in {self.log_dir})")
            time.sleep(0.1)

    def fcu_url(self, drone_id: int) -> str:
        url = self.profile.of(drone_id).fcu_url if self.profile is not None else sim_fcu_url(self.cfg, drone_id)
        assert_simulated_url(url)
        return url

    def start_mavros(self, drone_id: int) -> None:
        require_ros_env()
        url = self.fcu_url(drone_id)
        radio = self.profile is not None and self.profile.of(drone_id).link == "standin"
        if radio:
            self.set_radio_link(drone_id)
            self.start_relay(drone_id)
        args = [
            str(MAVROS_NODE), "--ros-args",
            "-r", f"__ns:={namespace_of(drone_id)}",
            # Keep TF and parameter-event traffic inside this drone's namespace, so N instances
            # don't all process each other's events (O(N^2) CPU, see config/mavros_pluginlists.yaml).
            "-r", "/tf:=tf", "-r", "/tf_static:=tf_static", "-r", "/parameter_events:=parameter_events",
            "--params-file", str(self.pluginlists),
            "--params-file", str(MAVROS_SHARE / "px4_config.yaml"),
            *(["--params-file", str(self.mavros_radio_params(drone_id))] if radio else []),
            "-p", f"fcu_url:={url}",
            "-p", f"tgt_system:={sysid_of(self.cfg, drone_id)}",
            "-p", "tgt_component:=1",
            "-p", "fcu_protocol:=v2.0",
        ]
        log = open(self.log_dir / f"mavros_{drone_id}.log", "w", encoding="utf-8")
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
        self.procs[drone_id].mavros = proc

    def start(self, drone_ids: list[int], stagger_s: float = 1.0) -> None:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        for did in drone_ids:
            self.start_px4(did)
            time.sleep(stagger_s)
        for did in drone_ids:
            self.start_mavros(did)

    def pids(self, drone_id: int) -> list[int]:
        dp = self.procs[drone_id]
        return [p.pid for p in (dp.px4, dp.mavros) if p is not None and p.poll() is None]

    def kill_drone(self, drone_id: int) -> None:
        """Fault F1/F4: hard-kill this drone's PX4 + MAVROS."""
        dp = self.procs[drone_id]
        for p in (dp.px4, dp.mavros):
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGKILL)

    def rss_mb(self) -> dict[str, float]:
        """Current resident memory per process group [MB] (px4 total, mavros total)."""
        out = {"px4": 0.0, "mavros": 0.0}
        for dp in self.procs.values():
            for key, p in (("px4", dp.px4), ("mavros", dp.mavros)):
                if p is None or p.poll() is not None:
                    continue
                try:
                    proc = psutil.Process(p.pid)
                    rss = proc.memory_info().rss
                    for child in proc.children(recursive=True):
                        rss += child.memory_info().rss
                    out[key] += rss / 1e6
                except psutil.NoSuchProcess:
                    pass
        return out

    def peak_rss_mb(self) -> dict[str, float]:
        """Kernel-tracked peak resident memory (VmHWM) per group [MB]; call while processes live."""
        out = {"px4": 0.0, "mavros": 0.0}
        for dp in self.procs.values():
            for key, p in (("px4", dp.px4), ("mavros", dp.mavros)):
                if p is not None and p.poll() is None:
                    out[key] += _vmhwm_mb(p.pid)
        return out

    def stop(self) -> None:
        for dp in self.procs.values():
            for p in (dp.mavros, dp.px4, dp.relay):
                if p is not None and p.poll() is None:
                    try:
                        os.killpg(p.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
        deadline = time.time() + 5.0
        for dp in self.procs.values():
            for p in (dp.mavros, dp.px4, dp.relay):
                if p is None:
                    continue
                try:
                    p.wait(timeout=max(0.1, deadline - time.time()))
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
        kill_orphans()
