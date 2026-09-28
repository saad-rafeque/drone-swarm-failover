#!/usr/bin/env python3
"""PX4 test queue: runs the remaining PX4 trials of Phases 4, 5 and 6 one at a time, only when the owner
starts it, and stops when the owner stops it. Nothing starts by itself (not at login, not when the charger is
plugged in). A finished trial is never lost: Start always continues with the first unfinished trial, also
after a shutdown or restart.

Order: Phase 4 (rounds 1-10 of F1-F5, then the clean-shell cross-check round 11), Phase 5 (nine radio
conditions: one mission without a fault and one with the leader killed) and Phase 6 (rounds 1-10 of F1 and
F2 with drone 1 behind the telemetry-radio stand-in, then a clean-shell cross-check round 11). 10 drones.

A trial is finished when its folder holds run_summary.json with the result COMPLETED or TIMEOUT (the flight
itself ended; a TIMEOUT counts as a mission that did not reach its goal and is never re-run, so bad flights
are not re-rolled). A trial stopped by a tooling failure (WATCHER_STALLED, LAUNCH_EXITED, ERROR) is moved
unchanged to <phase>_incomplete/ as evidence and run again from its start, at most three failed attempts.
A trial cut short by "stop now", an unplugged charger (when running on battery is off) or a shutdown holds
no result: its partial files are
removed and it runs again from its start (this never counts as a failed attempt).

Control and state (reports/logs/px4_queue/):
  control.json   {"mode": "run" | "stop_after" | "stop_now", "allow_battery": true | false}; written by the
                 Start / Stop buttons and the battery switch on the "PX4 tests" page of the ground-control app,
                 or by the commands below
  status.json    what the runner is doing: state, current trial, progress, recent trials; rewritten every 5 s
  runner.log     one line per event
With allow_battery off, trials run only on the charger in a power profile other than power saver, and a trial
stops (to run again later) if the charger is unplugged. With it on (the owner's choice), trials also run on
battery; every trial records the power state in its run_summary.json, because a throttled CPU on battery can
slow the drones' messages (reports/PHASE_4.md, the first attempt). At least 800 MB of memory must be free.

Usage: python3 scripts/px4_queue.py start                   start the runner in the background (the Start button)
       python3 scripts/px4_queue.py stop [--now]            stop after the running trial, or at once
       python3 scripts/px4_queue.py battery on|off | status
       python3 scripts/px4_queue.py run                     the runner itself, in the foreground
"""
from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from swarm_tools.resources import power_state  # noqa: E402

QDIR = REPO / "reports" / "logs" / "px4_queue"
CONTROL, STATUS, LOG, LOCK = QDIR / "control.json", QDIR / "status.json", QDIR / "runner.log", QDIR / "runner.lock"
FINISHED = {"COMPLETED", "TIMEOUT"}       # the flight ended; never re-run
MAX_ATTEMPTS = 3
MIN_AVAILABLE_MB = 800
N_DRONES = 10
PHASE5 = [(50, 0), (50, 10), (50, 30), (150, 0), (150, 10), (150, 30), (300, 0), (300, 10), (300, 30)]


@dataclass(frozen=True)
class Trial:
    phase: str          # "phase_4", "phase_5", "phase_6"
    name: str           # e.g. "F1_t3", "d150_l10_F1"
    folder: str         # relative to the repository
    args: tuple         # run_mission.py arguments after --n and --run-dir
    clean_shell: bool = False

    @property
    def path(self) -> Path:
        return REPO / self.folder


def queue() -> list[Trial]:
    out = []
    for r in range(1, 11):
        for f in ("F1", "F2", "F3", "F4", "F5"):
            out.append(Trial("phase_4", f"{f}_t{r}", f"reports/logs/phase_4/{f}_t{r}", ("--fault", f, "--fault-seed", str(r))))
    for f in ("F1", "F2", "F3", "F4", "F5"):
        out.append(Trial("phase_4", f"{f}_t11", f"reports/logs/phase_4_crosscheck/{f}_t11",
                         ("--fault", f, "--fault-seed", "11"), clean_shell=True))
    for k, (delay, loss) in enumerate(PHASE5, start=1):
        base = ("--latency-ms", str(delay), "--loss-pct", str(loss))
        out.append(Trial("phase_5", f"d{delay}_l{loss}_none", f"reports/logs/phase_5/d{delay}_l{loss}_none", base))
        out.append(Trial("phase_5", f"d{delay}_l{loss}_F1", f"reports/logs/phase_5/d{delay}_l{loss}_F1",
                         base + ("--fault", "F1", "--fault-seed", str(k))))
    prof = ("--profile", "config/profiles/standin.yaml")
    for r in range(1, 11):
        for f in ("F1", "F2"):
            out.append(Trial("phase_6", f"{f}_t{r}", f"reports/logs/phase_6/{f}_t{r}", ("--fault", f, "--fault-seed", str(r)) + prof))
    for f in ("F1", "F2"):
        out.append(Trial("phase_6", f"{f}_t11", f"reports/logs/phase_6_crosscheck/{f}_t11",
                         ("--fault", f, "--fault-seed", "11") + prof, clean_shell=True))
    return out


def result_of(t: Trial) -> str | None:
    try:
        return json.loads((t.path / "run_summary.json").read_text()).get("result")
    except (OSError, ValueError):
        return None


def incomplete_dir(t: Trial) -> Path:
    return REPO / "reports" / "logs" / f"{t.phase}_incomplete"


TOOLING_FAILURES = ("WATCHER_STALLED", "LAUNCH_EXITED", "ERROR", "exit code")


def attempts_of(t: Trial) -> int:
    """Failed attempts kept as evidence (tooling failures only; interruptions are not attempts)."""
    d = incomplete_dir(t)
    if not d.exists():
        return 0
    return sum(1 for p in d.glob(f"{t.name}_*")
               if (p / "why.txt").exists() and (p / "why.txt").read_text().startswith(TOOLING_FAILURES))


def is_done(t: Trial) -> bool:
    return result_of(t) in FINISHED


def is_given_up(t: Trial) -> bool:
    return not is_done(t) and attempts_of(t) >= MAX_ATTEMPTS


def discard_partial(t: Trial, why: str) -> None:
    """An interrupted trial holds no result: remove its partial files before it runs again."""
    removed = False
    if t.path.exists():
        shutil.rmtree(t.path)
        removed = True
    if Path(str(t.path) + ".out").exists():
        Path(str(t.path) + ".out").unlink()
        removed = True
    if removed:
        log(f"{t.phase}/{t.name}: partial files of an interrupted attempt removed ({why})")


def set_aside(t: Trial, why: str) -> None:
    """Keep a failed attempt as evidence before the trial runs again."""
    if not t.path.exists() and not Path(str(t.path) + ".out").exists():
        return
    states = t.path / "states.jsonl"
    if states.exists():
        subprocess.run(["gzip", "-f", str(states)], check=False)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    dest = incomplete_dir(t) / f"{t.name}_{stamp}"
    dest.mkdir(parents=True, exist_ok=True)
    if t.path.exists():
        shutil.move(str(t.path), str(dest / "run"))
    if Path(str(t.path) + ".out").exists():
        shutil.move(str(t.path) + ".out", str(dest / "console.out"))
    (dest / "why.txt").write_text(why + "\n")
    log(f"{t.phase}/{t.name}: attempt kept in {dest.relative_to(REPO)} ({why})")


def read_json(path: Path, default: dict) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return dict(default)


def write_json(path: Path, obj: dict) -> None:
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(obj, indent=1) + "\n")
    os.replace(tmp, path)


def log(msg: str) -> None:
    QDIR.mkdir(parents=True, exist_ok=True)
    line = f"{dt.datetime.now().isoformat(timespec='seconds')} {msg}"
    with open(LOG, "a", encoding="utf-8") as fh:
        fh.write(line + "\n")
    print(line, flush=True)


def control() -> dict:
    c = read_json(CONTROL, {})
    m = {"pause_after": "stop_after", "pause_now": "stop_now"}.get(c.get("mode"), c.get("mode", "stopped"))
    return {"mode": m, "allow_battery": bool(c.get("allow_battery", True))}


def mode() -> str:
    return control()["mode"]


def set_control(**changes) -> None:
    QDIR.mkdir(parents=True, exist_ok=True)
    c = control()
    c.update(changes)
    c["set_at"] = dt.datetime.now().isoformat(timespec="seconds")
    write_json(CONTROL, c)


def set_mode(m: str) -> None:
    set_control(mode=m)


def power_ok() -> tuple[bool, str]:
    """May a trial run now? Always yes when the owner allowed running on battery."""
    if control()["allow_battery"]:
        return True, ""
    p = power_state()
    if p.get("ac_online") is False:
        return False, "waiting for the charger (running on battery is switched off)"
    if p.get("profile") == "power-saver":
        return False, "waiting for a power profile other than power saver"
    return True, ""


def runner_running() -> bool:
    """True while a runner holds the lock."""
    QDIR.mkdir(parents=True, exist_ok=True)
    with open(LOCK, "a") as fh:
        try:
            fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(fh, fcntl.LOCK_UN)
    return False


def start_runner() -> str:
    """The Start button: set the mode to run and start a runner in the background if none is running."""
    set_mode("run")
    if runner_running():
        return "The tests are already running."
    with open(QDIR / "runner.console", "a", encoding="utf-8") as out:
        subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "run"], cwd=REPO, stdout=out,
                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    return "Started: the first unfinished trial runs now."


def memory_ok() -> bool:
    import psutil
    return psutil.virtual_memory().available / 1e6 >= MIN_AVAILABLE_MB


def progress(trials: list[Trial]) -> dict:
    out = {}
    for ph in ("phase_4", "phase_5", "phase_6"):
        ts = [t for t in trials if t.phase == ph]
        out[ph] = {"done": sum(is_done(t) for t in ts), "given_up": sum(is_given_up(t) for t in ts), "total": len(ts)}
    return out


class Runner:
    def __init__(self) -> None:
        self.trials = queue()
        self.state, self.detail, self.current, self.started = "starting", "", None, None
        self.recent: list[dict] = read_json(STATUS, {}).get("recent", [])[-15:]
        self.durations: list[float] = [r["minutes"] * 60 for r in self.recent if r.get("result") in FINISHED]

    def write_status(self) -> None:
        prog = progress(self.trials)
        left = sum(p["total"] - p["done"] - p["given_up"] for p in prog.values())
        per = sum(self.durations) / len(self.durations) if self.durations else 8.5 * 60
        write_json(STATUS, {
            "state": self.state, "detail": self.detail, "mode": mode(), "pid": os.getpid(),
            "updated": dt.datetime.now().isoformat(timespec="seconds"),
            "current": None if self.current is None else {
                "phase": self.current.phase, "name": self.current.name, "folder": self.current.folder,
                "started": self.started, "elapsed_s": round(time.time() - self._t0) if self.state == "running" else None},
            "progress": prog, "trials_left": left, "minutes_per_trial": round(per / 60, 1),
            "eta_hours": round(left * per / 3600, 1), "recent": self.recent[-15:],
            "trials": [{"phase": t.phase, "name": t.name, "state": self.trial_state(t), "result": result_of(t)}
                       for t in self.trials]})

    def trial_state(self, t: Trial) -> str:
        if self.current is not None and t == self.current and self.state == "running":
            return "running"
        if is_done(t):
            return "done" if result_of(t) == "COMPLETED" else "timeout"
        if is_given_up(t):
            return "given_up"
        return "retry" if attempts_of(t) else "pending"

    def wait(self, state: str, detail: str, seconds: float = 20.0) -> None:
        self.state, self.detail, self.current = state, detail, None
        self.write_status()
        time.sleep(seconds)

    def run_trial(self, t: Trial) -> str:
        """Run one trial; returns its result, or "interrupted: <why>"."""
        earlier = result_of(t)
        if earlier is not None and earlier.startswith(TOOLING_FAILURES):
            set_aside(t, earlier)
        else:                               # cut short by a shutdown, Stop now or an unplugged charger
            discard_partial(t, "no result: stopped before the end")
        t.path.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["scripts/ros_env.sh", "python3", "scripts/run_mission.py", "--n", str(N_DRONES),
               "--run-dir", t.folder, *t.args] + (["--allow-battery"] if control()["allow_battery"] else [])
        if t.clean_shell:        # rule 3a: the cross-check round runs from a fresh, minimal environment
            inner = " ".join(f"'{c}'" for c in cmd)
            env = {"HOME": os.environ.get("HOME", ""), "USER": os.environ.get("USER", ""), "PATH": "/usr/local/bin:/usr/bin:/bin"}
            cmd = ["bash", "--noprofile", "--norc", "-c", f"cd '{REPO}' && {inner}"]
        else:
            env = dict(os.environ)
        subprocess.run(["pkill", "-x", "px4"], check=False)
        subprocess.run(["pkill", "-x", "mavros_node"], check=False)
        self.current, self.state, self.detail = t, "running", ""
        self._t0, self.started = time.time(), dt.datetime.now().isoformat(timespec="seconds")
        log(f"{t.phase}/{t.name}: start" + (" (clean shell)" if t.clean_shell else ""))
        with open(str(t.path) + ".out", "w", encoding="utf-8") as out:
            proc = subprocess.Popen(cmd, cwd=REPO, env=env, stdout=out, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
            why = None
            while proc.poll() is None:
                if mode() == "stop_now":
                    why = "stopped by the owner"
                else:
                    ok, detail = power_ok()
                    if not ok:
                        why = "charger unplugged or power saver on"
                if why:
                    self.stop(proc)
                    break
                self.write_status()
                time.sleep(5.0)
        result = f"interrupted: {why}" if why else (result_of(t) or f"exit code {proc.returncode}, no summary")
        states = t.path / "states.jsonl"
        if states.exists() and not why:
            subprocess.run(["gzip", "-f", str(states)], check=False)
        dur = time.time() - self._t0
        if result in FINISHED:
            self.durations.append(dur)
        self.recent.append({"phase": t.phase, "name": t.name, "result": result, "minutes": round(dur / 60, 1),
                            "finished": dt.datetime.now().isoformat(timespec="seconds")})
        log(f"{t.phase}/{t.name}: {result} after {dur / 60:.1f} min")
        if why:
            discard_partial(t, result)
        elif result not in FINISHED:
            set_aside(t, result)
        self.current, self.state, self.detail = None, "between trials", ""
        self.write_status()                 # the page shows the finished trial at once
        return result

    @staticmethod
    def stop(proc: subprocess.Popen) -> None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
        subprocess.run(["pkill", "-x", "px4"], check=False)
        subprocess.run(["pkill", "-x", "mavros_node"], check=False)

    def loop(self) -> None:
        log(f"runner started (pid {os.getpid()}); mode {mode()}, running on battery "
            f"{'allowed' if control()['allow_battery'] else 'not allowed'}")
        while True:
            if mode() != "run":
                self.state, self.detail, self.current = "stopped", "press Start to continue", None
                self.write_status()
                log("runner stopped, as asked")
                return
            todo = [t for t in self.trials if not is_done(t) and not is_given_up(t)]
            if not todo:
                self.state, self.detail, self.current = "finished", "every trial has run", None
                self.write_status()
                log("queue finished")
                return
            ok, detail = power_ok()
            if not ok:
                self.wait("waiting", detail, 30.0)
                continue
            if not memory_ok():
                self.wait("waiting", f"waiting for {MIN_AVAILABLE_MB} MB of free memory", 30.0)
                continue
            self.run_trial(todo[0])
            self.current = None


def main() -> int:
    ap = argparse.ArgumentParser(description="PX4 test queue, started and stopped by the owner")
    ap.add_argument("command", choices=["start", "stop", "battery", "status", "run"])
    ap.add_argument("value", nargs="?", choices=["on", "off"], help="with battery: allow running on battery or not")
    ap.add_argument("--now", action="store_true", help="with stop: stop the running trial at once (it runs again later)")
    args = ap.parse_args()
    QDIR.mkdir(parents=True, exist_ok=True)
    if args.command == "start":
        print(start_runner())
        return 0
    if args.command == "stop":
        set_mode("stop_now" if args.now else "stop_after")
        print("stopping" + (" now" if args.now else " after the running trial"))
        return 0
    if args.command == "battery":
        if args.value is None:
            ap.error("battery needs on or off")
        set_control(allow_battery=args.value == "on")
        print(f"running on battery {'allowed' if args.value == 'on' else 'not allowed'}")
        return 0
    if args.command == "status":
        st = read_json(STATUS, {})
        print(json.dumps({k: st.get(k) for k in ("state", "detail", "mode", "current", "progress", "trials_left",
                                                   "eta_hours", "updated")}, indent=1))
        return 0
    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("another runner is already active")
        return 1
    if mode() != "run":           # the runner only runs when started (the Start button sets the mode first)
        set_mode("run")
    Runner().loop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
