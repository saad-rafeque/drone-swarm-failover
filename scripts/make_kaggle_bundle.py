#!/usr/bin/env python3
"""Pack the code Kaggle needs (src, scripts, config, cached map data) into kaggle/swarm-rl-code.zip.

Upload that zip to Kaggle as a private dataset named swarm-rl-code, then run kaggle/train_swarm_rl.ipynb.
"""
from __future__ import annotations

import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INCLUDE = ["src", "scripts", "config", "data/osm", "pyproject.toml"]


def main() -> None:
    out = ROOT / "kaggle" / "swarm-rl-code.zip"
    out.parent.mkdir(exist_ok=True)
    n = 0
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for item in INCLUDE:
            p = ROOT / item
            files = [p] if p.is_file() else sorted(q for q in p.rglob("*") if q.is_file())
            for f in files:
                if "__pycache__" in f.parts or f.suffix == ".pyc":
                    continue
                z.write(f, f.relative_to(ROOT))
                n += 1
    print(f"wrote {out} ({n} files, {out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
