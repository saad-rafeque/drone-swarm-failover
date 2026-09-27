#!/usr/bin/env python3
"""Build docs/HANDOVER.pdf: every document of the project, the key figures and the phase reports in
one printable file for handing the project over.

Usage: PYTHONPATH=src python3 scripts/make_handover_pdf.py [--out docs/HANDOVER.pdf] [--keep-html]
       PYTHONPATH=src python3 scripts/make_handover_pdf.py --only kaggle --out docs/KAGGLE_GUIDE.pdf
       (--only <section id> prints one section as its own document; ids are in SECTIONS below)
Needs the `markdown` Python package (Ubuntu: python3-markdown) and Google Chrome (headless printing).
"""
from __future__ import annotations

import argparse
import datetime as dt
import html
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import markdown

ROOT = Path(__file__).resolve().parents[1]
SECTIONS = [
    ("start", "Start here", "README.md"),
    ("how", "How it works", "docs/ARCHITECTURE.md"),
    ("results", "Results", "docs/RESULTS.md"),
    ("figures", "Key figures", None),
    ("run", "How to install and run everything", "docs/RUNBOOK.md"),
    ("decisions", "Decisions and why", "docs/DECISIONS.md"),
    ("issues", "Known issues, what is not done, next steps", "docs/KNOWN_ISSUES.md"),
    ("kaggle", "Long RL training on Kaggle", "docs/KAGGLE.md"),
    ("p0", "Appendix A - Phase 0 report", "reports/PHASE_0.md"),
    ("p1", "Appendix B - Phase 1 report", "reports/PHASE_1.md"),
    ("p2", "Appendix C - Phase 2 report", "reports/PHASE_2.md"),
    ("p3", "Appendix D - Phase 3 report", "reports/PHASE_3.md"),
    ("p4", "Appendix E - Phase 4 report (not completed)", "reports/PHASE_4.md"),
    ("scaling", "Appendix F - Scaling test", "reports/SCALING.md"),
    ("spec", "Appendix G - Project specification", "docs/SPECIFICATION.md"),
]
FIGURES = [
    ("reports/gcs_3d_view.jpg", "The 3D view of the ground-control app: ten drones in V formation over F-9 Park, Islamabad "
                                "(Chase camera, 27 September 2026). Leader 1 has the blue ring; V lines, height lines and "
                                "short trails; red = OpenStreetMap buildings that are obstacles at 16 m, green = woods, "
                                "yellow = the leader's planned route to Faisal Mosque."),
    ("reports/phase3_mission.png", "Ten PX4 drones fly a 1 km V mission: tracks with V snapshots, every follower against its "
                                   "ideal slot, formation error over time (Phase 3, cross-check run 2)."),
    ("reports/logs/rl/eval/comparison.png", "Obstacle avoidance on 90 unseen courses: success rate, crashes and drones left "
                                            "behind for five methods at three obstacle densities."),
    ("reports/rl_training.png", "RL training on the laptop: return per episode and held-out scores; still improving at 6 "
                                "million steps."),
    ("reports/phase1_resources.png", "PX4 + MAVROS resources for 3, 5 and 10 drones on this laptop (Phase 1)."),
    ("reports/phase0_altitude.png", "The first PX4 flight: arm, climb to 10 m, hover 20 s, land (Phase 0)."),
]
CSS = """
@page { size: A4; margin: 18mm 16mm 20mm;
  @bottom-left { content: "Swarm Failover - handover"; font: 8.5pt 'DejaVu Sans', sans-serif; color: #6b7b87; }
  @bottom-right { content: "page " counter(page) " of " counter(pages); font: 8.5pt 'DejaVu Sans', sans-serif; color: #6b7b87; } }
@page :first { @bottom-left { content: none; } @bottom-right { content: none; } }
* { box-sizing: border-box; }
body { font: 10pt/1.5 'Noto Sans', 'DejaVu Sans', 'Liberation Sans', Arial, sans-serif; color: #14212b; margin: 0; }
h1, h2, h3, h4 { font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; color: #10202c; line-height: 1.25; break-after: avoid; }
h1 { font-size: 20pt; margin: 0 0 10pt; }
h2 { font-size: 14pt; margin: 16pt 0 6pt; border-bottom: 1px solid #d2dbe1; padding-bottom: 3pt; }
h3 { font-size: 11.5pt; margin: 12pt 0 4pt; }
p, li { orphans: 3; widows: 3; }
a { color: #2767b3; text-decoration: none; }
code { font: 8.6pt 'DejaVu Sans Mono', monospace; background: #eef2f4; padding: 0 2pt; border-radius: 2pt; overflow-wrap: anywhere; }
pre { background: #eef2f4; border: 1px solid #d2dbe1; border-radius: 4pt; padding: 6pt 8pt; white-space: pre-wrap;
  word-break: break-word; font-size: 8.6pt; break-inside: avoid; }
pre code { background: none; padding: 0; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0 10pt; font-size: 8.4pt; }
th, td { border: 1px solid #d2dbe1; padding: 3pt 5pt; vertical-align: top; text-align: left; overflow-wrap: anywhere; }
th { background: #e2ecf8; }
tr { break-inside: avoid; }
blockquote { margin: 8pt 0; padding: 6pt 10pt; background: #f4f7f9; border-left: 3pt solid #2767b3; }
section.doc { break-before: page; }
.cover { height: 250mm; display: flex; flex-direction: column; justify-content: space-between; }
.cover h1 { font-size: 34pt; margin-top: 40mm; }
.cover .sub { font-size: 14pt; color: #475763; max-width: 150mm; }
.cover .meta { font-size: 10pt; color: #475763; }
.toc ol { font-size: 11pt; line-height: 1.9; }
.kicker { font-size: 8.5pt; letter-spacing: 0.08em; text-transform: uppercase; color: #6b7b87; margin-bottom: 2pt; }
figure { margin: 0 0 14pt; break-inside: avoid; }
figure img { width: 100%; border: 1px solid #d2dbe1; }
figcaption { font-size: 8.8pt; color: #475763; margin-top: 3pt; }
"""


def md_to_html(text: str, base: Path) -> str:
    body = markdown.markdown(text, extensions=["tables", "fenced_code", "sane_lists"])
    body = re.sub(r"<h1>(.*?)</h1>", r"<h2>\1</h2>", body, count=1)   # the section title is the page's h1

    def link(m: re.Match) -> str:
        href = m.group(1)
        if re.match(r"^(https?:|#|mailto:)", href):
            return m.group(0)
        return f'href="{(base / href).resolve().as_uri()}"'
    def image(m: re.Match) -> str:
        src = m.group(1)
        return m.group(0) if re.match(r"^(https?:|data:)", src) else f'src="{(base / src).resolve().as_uri()}"'
    body = re.sub(r'src="([^"]+)"', image, body)
    return re.sub(r'href="([^"]+)"', link, body)


def build_html(only: str | None = None) -> str:
    today = dt.date.today().strftime("%d %B %Y")
    sections = [x for x in SECTIONS if only is None or x[0] == only]
    if not sections:
        raise SystemExit(f"unknown section {only!r}; choose one of {', '.join(x[0] for x in SECTIONS)}")
    name = "handover" if only is None else sections[0][1]
    css = CSS.replace('"Swarm Failover - handover"', f'"Swarm Failover - {name}"')
    parts = [f"<!doctype html><html lang='en'><head><meta charset='utf-8'><title>Swarm Failover - {html.escape(name)}</title>"
             f"<style>{css}</style></head><body>"]
    if only is not None:
        sid, title, path = sections[0]
        parts.append(f"""<div class="cover">
  <div><p class="kicker">Swarm Failover</p><h1>{html.escape(title)}</h1>
  <p class="sub">Printed from <code>{html.escape(path or "reports/")}</code>. The complete handover, with every other document, is
  <code>docs/HANDOVER.pdf</code>.</p></div>
  <div class="meta">Generated {html.escape(today)}. Regenerate with
  <code>scripts/make_handover_pdf.py --only {html.escape(sid)}</code>.</div>
</div>""")
    else:
        parts.append(f"""<div class="cover">
  <div><p class="kicker">Handover document</p><h1>Swarm Failover</h1>
  <p class="sub">A drone swarm that keeps flying its mission when the leader fails - simulation, tests, results,
  how to run everything and what is left, in one file.</p></div>
  <div class="meta">Generated {html.escape(today)} from the repository at <code>{html.escape(str(ROOT))}</code>.<br>
  Everything described here was built and tested in simulation; no real drone has flown this code.<br>
  Source documents: README.md, docs/*.md, reports/*.md. Regenerate with <code>scripts/make_handover_pdf.py</code>.</div>
</div>""")
    if only is None:
        parts.append('<section class="doc toc"><h1>Contents</h1><ol>' +
                     "".join(f'<li><a href="#{sid}">{html.escape(title)}</a></li>' for sid, title, _ in SECTIONS) +
                     "</ol></section>")
    for sid, title, path in sections:
        parts.append(f'<section class="doc" id="{sid}"><h1>{html.escape(title)}</h1>')
        if path is None:
            for img, cap in FIGURES:
                f = ROOT / img
                if f.exists():
                    parts.append(f'<figure><img src="{f.as_uri()}" alt=""><figcaption>{html.escape(cap)}</figcaption></figure>')
        else:
            f = ROOT / path
            text = f.read_text(encoding="utf-8") if f.exists() else f"*{path} is missing.*"
            parts.append(f'<p class="kicker">{html.escape(path)}</p>')
            parts.append(md_to_html(text, f.parent))
        parts.append("</section>")
    parts.append("</body></html>")
    return "\n".join(parts)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "HANDOVER.pdf"))
    ap.add_argument("--keep-html", action="store_true")
    ap.add_argument("--only", help="print only this section (for example: kaggle)")
    args = ap.parse_args()
    chrome = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")
    if chrome is None:
        raise SystemExit("Google Chrome (or Chromium) is needed to print the PDF")
    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        page = Path(tmp) / "handover.html"
        page.write_text(build_html(args.only), encoding="utf-8")
        if args.keep_html:
            shutil.copy(page, out.with_suffix(".html"))
        subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", "--no-sandbox",
                        f"--user-data-dir={tmp}/profile", "--allow-file-access-from-files",
                        f"--print-to-pdf={out}", page.as_uri()], check=True, capture_output=True, timeout=180)
    print(f"wrote {out} ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
