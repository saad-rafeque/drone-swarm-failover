// PX4 tests page: the status of the PX4 test queue (scripts/px4_queue.py) and its pause / resume buttons.
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const PHASES = {
    phase_4: "Phase 4: faults F1-F5, 10 rounds and a clean-shell cross-check",
    phase_5: "Phase 5: radio sweep, 9 conditions (no fault and leader killed)",
    phase_6: "Phase 6: F1 and F2 with drone 1 behind the radio stand-in",
  };
  const STATES = {   // icon + label, never colour alone
    running: ["▶", "Running"], paused: ["❚❚", "Paused"], waiting: ["⏳", "Waiting"],
    finished: ["✓", "Finished"], starting: ["○", "Starting"], "not started": ["○", "Not started"],
    "between trials": ["▶", "Running"],
  };
  const MARK = { done: "✓", timeout: "!", running: "▶", retry: "↻", given_up: "✕", pending: "" };
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const mins = (s) => (s == null ? "" : `${Math.floor(s / 60)} min ${String(s % 60).padStart(2, "0")} s`);

  function render(st) {
    const alive = st.runner_alive;
    let key = st.state || "not started";
    if (!alive && key !== "finished") key = "not started";
    const [icon, label] = STATES[key] || ["○", key];
    $("q-icon").textContent = icon;
    $("q-icon").dataset.state = key;
    let detail = st.detail || "";
    if (key === "running" && st.current) {
      detail = `${PHASES[st.current.phase].split(":")[0]}, trial ${st.current.name}: ${mins(st.current.elapsed_s)} so far`;
    } else if (!alive && st.state !== "finished") {
      detail = st.updated ? `The runner is not running (last seen ${st.updated.replace("T", " ")}). Start it with: bash scripts/px4_queue_service.sh install`
        : "The runner has not been started. Start it with: bash scripts/px4_queue_service.sh install";
    }
    if (alive && st.mode !== "run" && key === "running") detail += " — pausing after this trial";
    $("q-state").textContent = alive && st.mode === "pause_now" && key === "running" ? "Pausing…" : label;
    $("q-detail").textContent = detail;
    $("b-pause-after").disabled = st.mode !== "run";
    $("b-pause-now").disabled = st.mode === "pause_now" || key !== "running";
    $("b-resume").disabled = st.mode === "run";

    const prog = st.progress || {};
    $("q-progress").innerHTML = Object.entries(PHASES).map(([ph, name]) => {
      const p = prog[ph] || { done: 0, total: 0, given_up: 0 };
      const pct = p.total ? Math.round((100 * p.done) / p.total) : 0;
      return `<div class="qbar"><div class="qbar-label"><span>${esc(name)}</span><span class="mono">${p.done} of ${p.total}${p.given_up ? `, ${p.given_up} skipped` : ""}</span></div>
        <div class="qbar-track" role="progressbar" aria-valuemin="0" aria-valuemax="${p.total}" aria-valuenow="${p.done}" aria-label="${esc(name)}"><div class="qbar-fill" style="width:${pct}%"></div></div></div>`;
    }).join("");
    $("q-eta").textContent = st.trials_left == null ? "" : st.trials_left === 0 ? "Every trial has run." :
      `${st.trials_left} trials left, about ${st.minutes_per_trial} minutes each: roughly ${st.eta_hours} hours of testing.`;

    const trials = st.trials || [];
    $("q-grid").innerHTML = Object.entries(PHASES).map(([ph, name]) => {
      const cells = trials.filter((t) => t.phase === ph).map((t) =>
        `<span class="qsq ${t.state}" title="${esc(`${t.name}: ${t.state}${t.result ? ` (${t.result})` : ""}`)}">${MARK[t.state] || ""}</span>`).join("");
      return `<div class="qgroup"><div class="hint">${esc(name.split(":")[0])}</div><div class="qcells">${cells}</div></div>`;
    }).join("");

    const recent = (st.recent || []).slice().reverse();
    $("q-recent").innerHTML = recent.length ? `<table><thead><tr><th>Trial</th><th>Result</th><th>Minutes</th><th>Finished</th></tr></thead><tbody>${
      recent.map((r) => `<tr><td>${esc(PHASES[r.phase].split(":")[0])}, ${esc(r.name)}</td><td>${esc(r.result)}</td><td class="mono">${r.minutes}</td><td class="mono">${esc(r.finished.replace("T", " "))}</td></tr>`).join("")
    }</tbody></table>` : `<p class="hint">No trial has finished yet.</p>`;
  }

  async function refresh() {
    try {
      const r = await fetch("/api/tests", { cache: "no-store" });
      render(await r.json());
    } catch (e) {
      $("q-detail").textContent = "The app cannot read the queue status right now.";
    }
  }
  async function act(action) {
    const r = await fetch("/api/tests", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ action }) });
    const res = await r.json();
    $("q-msg").textContent = res.msg || "";
    refresh();
  }
  $("b-pause-after").addEventListener("click", () => act("pause_after"));
  $("b-pause-now").addEventListener("click", () => act("pause_now"));
  $("b-resume").addEventListener("click", () => act("resume"));
  refresh();
  setInterval(refresh, 3000);
})();
