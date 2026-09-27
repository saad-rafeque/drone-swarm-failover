// Docs page: the handover PDF and the project's Markdown documents, rendered in place.
(() => {
  "use strict";
  const DOCS = [
    ["Handover (PDF)", "/docs/HANDOVER.pdf"],
    ["README: start here", "/docs/README.md"],
    ["How it works", "/docs/ARCHITECTURE.md"],
    ["How to run everything", "/docs/RUNBOOK.md"],
    ["Results", "/docs/RESULTS.md"],
    ["Decisions and why", "/docs/DECISIONS.md"],
    ["Known issues and next steps", "/docs/KNOWN_ISSUES.md"],
    ["Project specification", "/docs/SPECIFICATION.md"],
    ["RL training on Kaggle", "/docs/KAGGLE_GUIDE.md"],
    ["Kaggle guide (PDF)", "/docs/KAGGLE_GUIDE.pdf"],
    ["Phase 0 report", "/reports/PHASE_0.md"],
    ["Phase 1 report", "/reports/PHASE_1.md"],
    ["Phase 2 report", "/reports/PHASE_2.md"],
    ["Phase 3 report", "/reports/PHASE_3.md"],
    ["Phase 4 report (not completed)", "/reports/PHASE_4.md"],
    ["Scaling test", "/reports/SCALING.md"],
  ];
  const list = document.getElementById("doclist"), view = document.getElementById("docview");
  async function show(url, title) {
    if (url.endsWith(".pdf")) { window.open(url, "_blank"); return; }
    try {
      const r = await fetch(url);
      if (!r.ok) throw new Error();
      const base = url.slice(0, url.lastIndexOf("/") + 1);
      view.innerHTML = renderMarkdown(await r.text(), base);
    } catch (e) { view.innerHTML = `<p class="hint">${title} is not written yet.</p>`; }
    list.querySelectorAll("a").forEach((a) => a.setAttribute("aria-current", String(a.dataset.url === url)));
    history.replaceState(null, "", "#" + encodeURIComponent(url));
  }
  list.innerHTML = DOCS.map(([t, u]) => `<li><a href="#" data-url="${u}">${t}</a></li>`).join("");
  list.querySelectorAll("a").forEach((a) => a.addEventListener("click", (ev) => { ev.preventDefault(); show(a.dataset.url, a.textContent); }));
  const start = decodeURIComponent(location.hash.slice(1)) || "/docs/README.md";
  show(start, "This document");
})();
