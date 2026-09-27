#!/usr/bin/env python3
"""Pack the code Kaggle needs (src, scripts, config, cached map data) into kaggle/swarm-rl-code.zip.

Upload that zip to Kaggle as a private dataset named swarm-rl-code, then run kaggle/train_swarm_rl.ipynb
(full steps: docs/KAGGLE_GUIDE.md).

Secrets never go into the zip: files with ".local." in the name (config/map_keys.local.yaml holds your
Mapbox and Cesium tokens) are skipped, and the build stops if any packed file still contains one of the
values from that file.
"""
from __future__ import annotations

import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INCLUDE = ["src", "scripts", "config", "data/osm", "pyproject.toml"]
KEYS_FILE = ROOT / "config" / "map_keys.local.yaml"


def secret_values() -> list[bytes]:
    """The token values in config/map_keys.local.yaml (read only to make sure none of them is packed)."""
    if not KEYS_FILE.exists():
        return []
    vals = []
    for line in KEYS_FILE.read_text(encoding="utf-8").splitlines():
        if ":" in line and not line.lstrip().startswith("#"):
            v = line.split(":", 1)[1].strip().strip("'\"")
            if len(v) >= 16:
                vals.append(v.encode())
    return vals


def packed_files() -> list[Path]:
    out = []
    for item in INCLUDE:
        p = ROOT / item
        for f in [p] if p.is_file() else sorted(q for q in p.rglob("*") if q.is_file()):
            if "__pycache__" in f.parts or f.suffix == ".pyc" or ".local." in f.name:
                continue
            out.append(f)
    return out


def main() -> None:
    out = ROOT / "kaggle" / "swarm-rl-code.zip"
    out.parent.mkdir(exist_ok=True)
    files, secrets = packed_files(), secret_values()
    for f in files:
        data = f.read_bytes()
        if any(s in data for s in secrets):
            raise SystemExit(f"refusing to pack {f.relative_to(ROOT)}: it contains a value from {KEYS_FILE.name}")
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for f in files:
            z.write(f, f.relative_to(ROOT))
    print(f"wrote {out} ({len(files)} files, {out.stat().st_size / 1e6:.1f} MB; no *.local.* files, no map keys)")


if __name__ == "__main__":
    main()
