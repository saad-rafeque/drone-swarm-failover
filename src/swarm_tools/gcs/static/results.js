// Results page: reads the committed result files and shows them.
(async () => {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const get = async (url, kind = "text") => { const r = await fetch(url); if (!r.ok) throw new Error(url); return kind === "json" ? r.json() : r.text(); };
  const METHODS = [["none", "No avoidance"], ["none+shield", "Brake only"], ["apf", "Classical"], ["rl", "RL"], ["rl+shield", "RL + brake"]];
  const LEVELS = [["low", "Few obstacles"], ["medium", "Medium"], ["high", "Dense"]];
  try {
    const j = await get("/reports/logs/rl/eval_kaggle_check/summary.json", "json");
    const m = j.meta;
    $("rl-meta").textContent = `${m.episodes} unseen routes per density, ${m.drones} drones, every method on the same routes. ` +
      "Success = no crash and the whole formation re-forms at the target.";
    const rows = METHODS.map(([k, name]) => `<tr><td>${name}</td>` + LEVELS.map(([lv]) => {
      const s = j.summary[`${lv}/${k}`]; return `<td class="num">${s.success}/${s.episodes}</td>`;
    }).join("") + `<td class="num">${j.summary["medium/" + k].crashes_per_episode.toFixed(2)}</td>` +
      `<td class="num">${j.summary["medium/" + k].stuck_per_episode.toFixed(2)}</td></tr>`).join("");
    $("rl-table").innerHTML = `<div class="tbl"><table><thead><tr><th>Method</th>${LEVELS.map(([, n]) => `<th class="num">${n}</th>`).join("")}` +
      `<th class="num">Crashes / mission (medium)</th><th class="num">Left behind (medium)</th></tr></thead><tbody>${rows}</tbody></table></div>`;
    $("rl-paired").textContent = "Paired on the same routes (RL + brake vs classical, successes only one of them had): " +
      Object.entries(j.paired).map(([lv, p]) => `${lv}: RL ${p.rl_only}, classical ${p.apf_only}`).join("; ") +
      ". The RL-only column in the file compares RL without the brake.";
  } catch (e) { $("rl-meta").textContent = "RL evaluation results are not available yet (reports/logs/rl/eval_kaggle_check)."; }
  try { $("longroute").innerHTML = renderMarkdown(await get("/reports/logs/long_route/summary.md"), "/reports/logs/long_route/"); }
  catch (e) { $("longroute").textContent = "Not available yet."; }
  // real-map comparison: the policy in use (Kaggle seed 2) first, else the laptop policy's 10 Hz re-run
  try { $("route").innerHTML = renderMarkdown(await get("/reports/logs/rl/route_eval_kaggle_check/summary.md")); }
  catch (e) {
    try { $("route").innerHTML = renderMarkdown(await get("/reports/logs/rl/route_eval_10hz/summary.md")); }
    catch (e2) { $("route").textContent = "Not available yet."; }
  }
  try {
    const md = await get("/reports/SCALING.md");
    const start = md.indexOf("## Results");
    const end = md.indexOf("## What changed");
    $("scaling").innerHTML = renderMarkdown(md.slice(start, end > 0 ? end : undefined), "/reports/");
  } catch (e) { $("scaling").textContent = "Not available yet."; }
})();
