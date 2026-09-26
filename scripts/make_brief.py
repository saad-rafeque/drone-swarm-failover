"""Build the Swarm Failover project brief (an HTML page body) from the scaling logs.

Numbers in the charts and tables are read from reports/logs/scaling/, not typed in.
Usage: PYTHONPATH=src python3 scripts/make_brief.py reports/brief/swarm_failover.html
The output is a page body for an online page viewer (no <html>/<head> wrapper).
"""
import json
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "reports/brief/swarm_failover.html"
sys.path.insert(0, str(REPO / "src"))
from swarm_agent.heartbeat import encoded_size  # noqa: E402


def load(name):
    p = REPO / "reports/logs/scaling" / name
    g = defaultdict(list)
    if p.exists():
        for line in p.read_text().splitlines():
            r = json.loads(line)
            g[r["n"]].append(r)
    return dict(sorted(g.items()))


clean = load("fastsim_scaling.jsonl")
radio = load("fastsim_scaling_radio.jsonl")


def med(rs, key):
    v = [r[key] for r in rs if r.get(key) is not None]
    return st.median(v) if v else None


def fmt(x, d=1, unit=""):
    return "&ndash;" if x is None else f"{x:.{d}f}{unit}"


# ---------------------------------------------------------------- failover chart (categorical N)
ns = [n for n in clean if n >= 2]
W, H, L, R, T, B = 720, 260, 64, 690, 18, 206
ymax = 16.0
X = {n: L + (R - L) * k / (len(ns) - 1) for k, n in enumerate(ns)}
Y = lambda v: B - (v / ymax) * (B - T)  # noqa: E731
parts = []
for v in (0, 3, 6, 9, 12, 15):
    parts.append(f'<line class="grid" x1="{L}" x2="{R}" y1="{Y(v):.1f}" y2="{Y(v):.1f}"/>'
                 f'<text class="tick" x="{L - 10}" y="{Y(v) + 4:.1f}" text-anchor="end">{v}</text>')
for v, lab in ((3, "goal: new leader within 3 s"), (15, "goal: formation back within 15 s")):
    parts.append(f'<line class="ref" x1="{L}" x2="{R}" y1="{Y(v):.1f}" y2="{Y(v):.1f}"/>'
                 f'<text class="reflab" x="{R}" y="{Y(v) - 6:.1f}" text-anchor="end">{lab}</text>')
for n in ns:
    parts.append(f'<text class="tick" x="{X[n]:.1f}" y="{B + 20}" text-anchor="middle">{n}</text>')
parts.append(f'<text class="axlab" x="{(L + R) / 2:.0f}" y="{H - 4}" text-anchor="middle">drones in the swarm</text>')
parts.append(f'<text class="axlab" x="14" y="{(T + B) / 2:.0f}" text-anchor="middle" '
             f'transform="rotate(-90 14 {(T + B) / 2:.0f})">seconds</text>')
fo = [(n, med(clean[n], "failover_s")) for n in ns]
rr = [(n, med(clean[n], "rms_recovery_s")) for n in ns if med(clean[n], "rms_recovery_s") is not None]
parts.append('<polyline class="s-form" points="' + " ".join(f"{X[n]:.1f},{Y(v):.1f}" for n, v in rr) + '"/>')
parts.append('<polyline class="s-lead" points="' + " ".join(f"{X[n]:.1f},{Y(v):.1f}" for n, v in fo) + '"/>')
for n, v in rr:
    parts.append(f'<rect class="m-form" x="{X[n] - 4.5:.1f}" y="{Y(v) - 4.5:.1f}" width="9" height="9"/>')
for n, v in fo:
    parts.append(f'<circle class="m-lead" cx="{X[n]:.1f}" cy="{Y(v):.1f}" r="5"/>')
n_last = ns[-1]
parts.append(f'<text class="vlab lead" x="{X[n_last] - 10:.1f}" y="{Y(fo[-1][1]) + 18:.1f}" text-anchor="end">{fo[-1][1]:.2f} s</text>')
parts.append(f'<text class="vlab form" x="{X[n_last] - 10:.1f}" y="{Y(rr[-1][1]) - 10:.1f}" text-anchor="end">{rr[-1][1]:.1f} s</text>')
chart_failover = (f'<svg viewBox="0 0 {W} {H}" role="img" aria-labelledby="cf-t"><title id="cf-t">Time to a new leader '
                  f'and to a restored formation, for 2 to {n_last} drones</title>' + "".join(parts) + "</svg>")

# ---------------------------------------------------------------- radio load chart (linear N)
W2, H2, L2, R2, T2, B2 = 720, 250, 64, 690, 18, 196
ymax2, nmax = 260.0, 100
X2 = lambda n: L2 + (R2 - L2) * n / nmax  # noqa: E731
Y2 = lambda v: B2 - (v / ymax2) * (B2 - T2)  # noqa: E731
rate = 5  # heartbeats per second (config/swarm.yaml)
load = lambda n: n * rate * encoded_size(n) * 8 / 1000  # noqa: E731
p2 = []
for v in (0, 50, 100, 150, 200, 250):
    p2.append(f'<line class="grid" x1="{L2}" x2="{R2}" y1="{Y2(v):.1f}" y2="{Y2(v):.1f}"/>'
              f'<text class="tick" x="{L2 - 10}" y="{Y2(v) + 4:.1f}" text-anchor="end">{v}</text>')
for n in (0, 20, 40, 60, 80, 100):
    p2.append(f'<text class="tick" x="{X2(n):.1f}" y="{B2 + 20}" text-anchor="middle">{n}</text>')
p2.append(f'<line class="ref" x1="{L2}" x2="{R2}" y1="{Y2(64):.1f}" y2="{Y2(64):.1f}"/>'
          f'<text class="reflab" x="{R2}" y="{Y2(64) - 6:.1f}" text-anchor="end">SiK telemetry radio, default air rate: 64 kbit/s</text>')
p2.append(f'<line class="ref" x1="{L2}" x2="{R2}" y1="{Y2(250):.1f}" y2="{Y2(250):.1f}"/>'
          f'<text class="reflab" x="{L2 + 8}" y="{Y2(250) - 6:.1f}">SiK radio, highest air rate: 250 kbit/s</text>')
p2.append('<polyline class="s-lead" points="' + " ".join(f"{X2(n):.1f},{Y2(load(n)):.1f}" for n in range(1, nmax + 1)) + '"/>')
v10, v100 = load(10), load(100)
p2.append(f'<line class="lead-line" x1="{X2(10):.1f}" y1="{Y2(v10) - 7:.1f}" x2="{X2(10):.1f}" y2="{Y2(v10) - 58:.1f}"/>'
          f'<text class="vlab lead" x="{X2(10) - 4:.1f}" y="{Y2(v10) - 64:.1f}">10 drones: {v10:.0f} kbit/s</text>')
p2.append(f'<text class="vlab lead" x="{X2(100) - 12:.1f}" y="{Y2(v100) - 2:.1f}" text-anchor="end">100 drones: {v100:.0f} kbit/s</text>')
for n, v in ((10, v10), (100, v100)):
    p2.append(f'<circle class="m-lead" cx="{X2(n):.1f}" cy="{Y2(v):.1f}" r="5"/>')
p2.append(f'<text class="axlab" x="{(L2 + R2) / 2:.0f}" y="{H2 - 4}" text-anchor="middle">drones in the swarm</text>')
p2.append(f'<text class="axlab" x="14" y="{(T2 + B2) / 2:.0f}" text-anchor="middle" '
          f'transform="rotate(-90 14 {(T2 + B2) / 2:.0f})">kbit/s</text>')
chart_radio = (f'<svg viewBox="0 0 {W2} {H2}" role="img" aria-labelledby="cr-t"><title id="cr-t">Radio data needed for '
               f'heartbeats as the swarm grows, against telemetry radio air rates</title>' + "".join(p2) + "</svg>")
cross = next(n for n in range(1, 300) if load(n) > 64)

# ---------------------------------------------------------------- tables
def row(n, rs, extra=False):
    ok = all(r["mission_done"] and r["all_landed"] for r in rs)
    one = all(len(r["final_masters"]) == 1 for r in rs)
    cells = [f"<td>{n}</td>",
             f'<td class="num">{fmt(med(rs, "failover_s"), 2)}</td>',
             f'<td class="num">{fmt(med(rs, "rms_recovery_s"), 1)}</td>',
             f'<td class="num">{fmt(min((r["min_sep_m"] for r in rs if r["min_sep_m"] is not None), default=None), 1)}</td>']
    if extra:
        cells.append(f'<td class="num">{max(r.get("max_simultaneous_masters", 1) for r in rs)}</td>')
    cells.append(f'<td><span class="chip {"ok" if ok and one else "bad"}">{len(rs)}/{len(rs)} landed</span></td>'
                 if ok and one else f'<td><span class="chip bad">{sum(r["mission_done"] and r["all_landed"] for r in rs)}/{len(rs)}</span></td>')
    if not extra:
        cells.append(f'<td class="num">{med(rs, "speedup"):.0f}&times;</td>')
        cells.append(f'<td class="num">{rs[0]["hb_bytes"]}</td>')
    return "<tr>" + "".join(cells) + "</tr>"


table_clean = "".join(row(n, rs) for n, rs in clean.items())
table_radio = "".join(row(n, rs, extra=True) for n, rs in radio.items())
n100 = clean.get(100, [])
facts = {
    "fo_all": st.median([r["failover_s"] for rs in clean.values() for r in rs if r["failover_s"] is not None]),
    "fo100": med(n100, "failover_s"), "rr100": med(n100, "rms_recovery_s"), "sp100": med(n100, "speedup"),
    "sp10": med(clean.get(10, []), "speedup"), "far100": med(n100, "max_dist_from_goal_m"),
    "runs": sum(len(rs) for rs in clean.values()), "runs_radio": sum(len(rs) for rs in radio.values()),
    "cross": cross,
}
print(json.dumps(facts))

page = (Path(__file__).parent / "brief_template.html").read_text()
for k, v in {"CHART_FAILOVER": chart_failover, "CHART_RADIO": chart_radio, "TABLE_CLEAN": table_clean,
             "TABLE_RADIO": table_radio, "FO_ALL": f"{facts['fo_all']:.1f}", "FO100": f"{facts['fo100']:.2f}",
             "RR100": f"{facts['rr100']:.1f}", "SP100": f"{facts['sp100']:.0f}", "SP10": f"{facts['sp10']:.0f}",
             "FAR100": f"{facts['far100']:.0f}", "RUNS": str(facts["runs"]), "RUNS_RADIO": str(facts["runs_radio"]),
             "CROSS": str(cross)}.items():
    page = page.replace("{{" + k + "}}", v)
assert "{{" not in page, page[page.index("{{"):page.index("{{") + 40]
OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(page)
