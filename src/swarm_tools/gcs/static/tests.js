// PX4 tests page: the status of the PX4 test queue (scripts/px4_queue.py), its Start / Stop buttons and battery switch.
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const PHASES = {
    phase_4: "Phase 4: faults F1-F5, 10 rounds and a clean-shell cross-check",
    phase_5: "Phase 5: radio sweep, 9 conditions (no fault and leader killed)",
    phase_6: "Phase 6: F1 and F2 with drone 1 behind the radio stand-in",
  };
  const STATES = {   // icon + label, never colour alone
    running: ["▶", "Running"], stopped: ["■", "Stopped"], waiting: ["⏳", "Waiting"],
    finished: ["✓", "Finished"], starting: ["○", "Starting"], "not started": ["○", "Not started"],
    "between trials": ["▶", "Running"],
  };
  const MARK = { done: "✓", timeout: "!", running: "▶", retry: "↻", given_up: "✕", pending: "" };
  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  const mins = (s) => (s == null ? "" : `${Math.floor(s / 60)} min ${String(s % 60).padStart(2, "0")} s`);

  function render(st) {
    const alive = st.runner_alive;
    let key = st.state || "not started";
    if (!alive && key !== "finished") key = st.state === "stopped" || st.updated ? "stopped" : "not started";
    const [icon, label] = STATES[key] || ["○", key];
    $("q-icon").textContent = icon;
    $("q-icon").dataset.state = key;
    let detail = st.detail || "";
    if (key === "running" && st.current) {
      detail = `${PHASES[st.current.phase].split(":")[0]}, trial ${st.current.name}: ${mins(st.current.elapsed_s)} so far`;
    } else if (!alive && st.state !== "finished") {
      detail = st.state === "stopped" ? "Press Start to continue with the next unfinished trial."
        : st.updated ? `Not running (last seen ${st.updated.replace("T", " ")}). Press Start to continue.`
        : "Not started yet. Press Start when you want the tests to run.";
    }
    if (alive && st.mode === "stop_after") detail += " — stopping after this trial";
    $("q-state").textContent = alive && st.mode === "stop_now" ? "Stopping…" : label;
    $("q-detail").textContent = detail;
    $("b-start").disabled = alive && st.mode === "run";
    $("b-stop-after").disabled = !alive || st.mode !== "run";
    $("b-stop-now").disabled = !alive || st.mode === "stop_now";
    if (document.activeElement !== $("c-battery")) $("c-battery").checked = !!st.allow_battery;
    const pw = st.power || {};
    const onBattery = pw.ac_online === false;
    $("q-power").textContent = pw.ac_online == null ? "" : (onBattery
      ? `On battery (${pw.battery_pct} %${pw.profile ? `, ${pw.profile} mode` : ""}).` + (st.allow_battery
        ? " Trials run anyway; on battery the CPU can slow down and results can be a little worse (each trial records the power state)."
        : " Trials wait for the charger.")
      : `On the charger${pw.profile ? ` (${pw.profile} mode)` : ""}.`);

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
  async function act(action, value) {
    const r = await fetch("/api/tests", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ action, value }) });
    const res = await r.json();
    $("q-msg").textContent = res.msg || "";
    refresh();
  }
  $("b-start").addEventListener("click", () => act("start"));
  $("b-stop-after").addEventListener("click", () => act("stop_after"));
  $("b-stop-now").addEventListener("click", () => act("stop_now"));
  $("c-battery").addEventListener("change", (e) => act("battery", e.target.checked));
  refresh();
  setInterval(refresh, 3000);
})();
