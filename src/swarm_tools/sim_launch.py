"""Start/stop N PX4 SIH instances + N MAVROS nodes on this laptop (simulation only).

Verified sources (see reports/PHASE_0.md / PHASE_1.md):
  * PX4 multi-instance start: `px4 -i <n> -d <build>/etc` run from a per-instance working
    directory (Tools/simulation/sitl_multiple_run.sh; docs.px4.io/main/en/sim_sih/).
  * Model: PX4_SIM_MODEL=sihsim_quadx selects airframe 10040_sihsim_quadx (rcS).
  * Home: PX4_HOME_LAT/LON/ALT -> SIH_LOC_LAT0/LON0/H0 (px4-rc.sihsim).
  * Offboard MAVLink link of instance n: PX4 listens on UDP 14580+n and sends to 14540+n
    (px4-rc.mavlink); MAV_SYS_ID = n+1 (rcS).
  * MAVROS: mavros_node with px4_pluginlists.yaml + px4_config.yaml (mavros px4.launch).
"""
from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import psutil

from swarm_agent.config import Config
from swarm_agent.geometry import GeoPoint

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


def assert_simulated_url(url: str) -> None:
    """Hard rule 7: never open serial devices from the simulation tools."""
    if not url.startswith("udp://"):
        raise RuntimeError(f"refusing non-UDP fcu_url {url!r}: simulation tools never open serial ports")


def namespace_of(drone_id: int) -> str:
    return f"/uav{drone_id}"


def require_ros_env() -> None:
    if "/opt/ros/jazzy" not in os.environ.get("AMENT_PREFIX_PATH", ""):
        raise RuntimeError("ROS 2 environment not sourced; run through scripts/ros_env.sh")


def kill_orphans(timeout_s: float = 5.0) -> list[int]:
    """Kill leftover px4 / mavros_node processes (rule 6). Matches exact process names only."""
    victims = [p for p in psutil.process_iter(["name"]) if p.info["name"] in ("px4", "mavros_node")]
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


@dataclass
class SimLauncher:
    """PX4 working dirs go under sim.work_dir (PX4's rcS breaks on paths with spaces, and the
    repo path has one); a fresh timestamped dir per run, so nothing is ever deleted there.
    Process logs go to log_dir inside the repo."""
    cfg: Config
    homes: dict[int, GeoPoint]
    log_dir: Path = field(default_factory=lambda: REPO_ROOT / ".sim")
    procs: dict[int, DroneProcs] = field(default_factory=dict)
    work_dir: Path = field(init=False)

    def __post_init__(self) -> None:
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
        env.update(
            PX4_SIM_MODEL=self.cfg.sim.model,
            PX4_HOME_LAT=f"{home.lat_deg:.9f}",
            PX4_HOME_LON=f"{home.lon_deg:.9f}",
            PX4_HOME_ALT=f"{home.alt_m:.3f}",
        )
        log = open(self.log_dir / f"px4_{drone_id}.log", "w", encoding="utf-8")
        proc = subprocess.Popen(
            [str(build / "bin" / "px4"), "-i", str(instance_of(self.cfg, drone_id)), "-d", str(build / "etc")],
            cwd=workdir, env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.procs[drone_id] = DroneProcs(drone_id, proc)

    def start_mavros(self, drone_id: int) -> None:
        require_ros_env()
        url = sim_fcu_url(self.cfg, drone_id)
        assert_simulated_url(url)
        args = [
            str(MAVROS_NODE), "--ros-args",
            "-r", f"__ns:={namespace_of(drone_id)}",
            "--params-file", str(MAVROS_SHARE / "px4_pluginlists.yaml"),
            "--params-file", str(MAVROS_SHARE / "px4_config.yaml"),
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
            for p in (dp.mavros, dp.px4):
                if p is not None and p.poll() is None:
                    try:
                        os.killpg(p.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
        deadline = time.time() + 5.0
        for dp in self.procs.values():
            for p in (dp.mavros, dp.px4):
                if p is None:
                    continue
                try:
                    p.wait(timeout=max(0.1, deadline - time.time()))
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
        kill_orphans()
