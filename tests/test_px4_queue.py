"""PX4 test queue (scripts/px4_queue.py) and its app endpoints: which trials count as finished, what is kept
as evidence, what runs again, that "stop now" stops a running trial, that the runner exits when stopped, and
that running on battery is the owner's choice. No PX4: the trial command is replaced by a sleeping process."""
from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import px4_queue as q  # noqa: E402

from swarm_tools.gcs.server import queue_control, queue_status  # noqa: E402


@pytest.fixture
def repo(tmp_path, monkeypatch):
    monkeypatch.setattr(q, "REPO", tmp_path)
    monkeypatch.setattr(q, "QDIR", tmp_path / "reports/logs/px4_queue")
    for name in ("CONTROL", "STATUS", "LOG", "LOCK"):
        monkeypatch.setattr(q, name, tmp_path / "reports/logs/px4_queue" / getattr(q, name).name)
    return tmp_path


def finish(t: q.Trial, result: str) -> None:
    t.path.mkdir(parents=True, exist_ok=True)
    (t.path / "run_summary.json").write_text(json.dumps({"result": result}))


def test_queue_order_and_contents():
    ts = q.queue()
    assert len(ts) == 95 and [t.phase for t in ts[:55]] == ["phase_4"] * 55
    assert ts[0].name == "F1_t1" and ts[50].name == "F1_t11" and ts[50].clean_shell
    assert {t.phase for t in ts[55:73]} == {"phase_5"} and ts[55].name == "d50_l0_none"
    assert ts[-1].name == "F2_t11" and ts[-1].clean_shell and "config/profiles/standin.yaml" in ts[-1].args


def test_finished_flights_are_never_rerun_and_tooling_failures_are_kept(repo):
    a, b, c = q.queue()[:3]
    finish(a, "COMPLETED")
    finish(b, "TIMEOUT")                                   # a flight outcome: counts, never re-rolled
    assert q.is_done(a) and q.is_done(b) and not q.is_done(c)
    for k in range(3):
        finish(c, "WATCHER_STALLED")
        q.set_aside(c, "WATCHER_STALLED")
        time.sleep(1.05)                                   # attempt folders are time-stamped to the second
        assert q.attempts_of(c) == k + 1 and not c.path.exists()
    assert q.is_given_up(c)
    kept = sorted((repo / "reports/logs/phase_4_incomplete").iterdir())
    assert len(kept) == 3 and (kept[0] / "run" / "run_summary.json").exists()


def test_interruptions_are_discarded_and_do_not_count(repo):
    t = q.queue()[0]
    t.path.mkdir(parents=True)
    (t.path / "states.jsonl").write_text("{}\n")
    q.discard_partial(t, "paused by the owner")
    assert not t.path.exists() and q.attempts_of(t) == 0 and not q.is_given_up(t)


def test_stop_now_stops_the_running_trial(repo, monkeypatch):
    real_popen = subprocess.Popen
    monkeypatch.setattr(q.subprocess, "Popen", lambda cmd, **kw: real_popen(["sleep", "60"], **kw))
    monkeypatch.setattr(q, "power_ok", lambda: (True, ""))
    monkeypatch.setattr(q.subprocess, "run", lambda *a, **k: None)          # no pkill / gzip in the test
    q.set_mode("run")
    runner = q.Runner()
    t = runner.trials[0]
    threading.Timer(1.0, lambda: q.set_mode("stop_now")).start()
    t0 = time.time()
    result = runner.run_trial(t)
    assert result == "interrupted: stopped by the owner" and time.time() - t0 < 30
    assert not t.path.exists() and q.attempts_of(t) == 0
    status = json.loads(q.STATUS.read_text())
    assert status["recent"][-1]["name"] == "F1_t1" and len(status["trials"]) == 95


def test_runner_exits_when_stopped_and_battery_is_the_owners_choice(repo, monkeypatch):
    q.set_control(mode="stop_after", allow_battery=False)
    runner = q.Runner()
    runner.loop()                                           # returns at once: nothing runs unless started
    assert json.loads(q.STATUS.read_text())["state"] == "stopped"
    monkeypatch.setattr(q, "power_state", lambda: {"ac_online": False, "profile": "balanced"})
    assert q.power_ok()[0] is False                         # battery not allowed: wait for the charger
    q.set_control(allow_battery=True)
    assert q.power_ok() == (True, "")                       # the owner allowed battery: run anyway


def test_app_endpoints(tmp_path):
    assert not queue_control("explode", tmp_path)["ok"]
    assert queue_control("stop_after", tmp_path)["ok"]
    assert json.loads((tmp_path / "control.json").read_text())["mode"] == "stop_after"
    assert queue_control("battery", tmp_path, value=False)["ok"]
    st = queue_status(tmp_path)
    assert st["mode"] == "stop_after" and st["allow_battery"] is False and st["runner_alive"] is False
    started = []
    assert queue_control("start", tmp_path, start=lambda: started.append(1) or "Started")["msg"] == "Started"
    assert started == [1]
    (tmp_path / "status.json").write_text(json.dumps({"state": "running", "pid": 1, "updated": "2000-01-01T00:00:00"}))
    assert queue_status(tmp_path)["runner_alive"] is False                  # stale status: not alive


def test_the_app_can_load_the_queue_script():
    """The Start button loads scripts/px4_queue.py inside the app (this is what failed on 28 September 2026)."""
    import sys as _sys
    from swarm_tools.gcs import server
    saved = _sys.modules.pop("px4_queue", None)
    try:
        mod = server._queue_module()
        assert len(mod.queue()) == 95 and callable(mod.start_runner)
    finally:
        _sys.modules.pop("px4_queue", None)
        if saved is not None:
            _sys.modules["px4_queue"] = saved


def test_leftover_radio_relays_are_stopped_and_nothing_else():
    """Stop now once left a telemetry-radio relay running (its own session), and the next trial's relay could
    not bind its ports. The queue stops relays by their exact module argument, and nothing that only looks
    similar; the launcher's clean-up uses the same rule."""
    from swarm_tools.sim_launch import is_radio_relay
    sleep = "import time; time.sleep(60)"
    relay = subprocess.Popen([sys.executable, "-c", sleep, "-m", "swarm_tools.radio_proxy"])
    other = subprocess.Popen([sys.executable, "-c", sleep, "-m", "swarm_tools.radio_proxy_x"])
    try:
        time.sleep(0.3)
        stopped = q.stop_radio_relays()
        assert relay.pid in stopped and other.pid not in stopped
        assert relay.wait(timeout=5) != 0 and other.poll() is None
    finally:
        for p in (relay, other):
            p.kill()
            p.wait()
    assert is_radio_relay(["python3", "-m", "swarm_tools.radio_proxy", "--instance", "0"])
    assert not is_radio_relay(["python3", "scripts/radio_proxy.py"])
