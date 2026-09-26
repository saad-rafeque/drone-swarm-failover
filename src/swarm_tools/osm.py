"""Buildings and trees from OpenStreetMap (Overpass API) as an ObstacleMap in the shared ENU frame.

Low-altitude profile: the swarm flies below rooftops and tree canopies, so every building footprint,
wood/forest area and single mapped tree is an obstacle to go around. OSM rarely records heights, so
flying over is not attempted. Downloads are cached as JSON (data (c) OpenStreetMap contributors, ODbL).
"""
from __future__ import annotations

import hashlib
import json
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from swarm_agent.geometry import EnuFrame, GeoPoint
from swarm_agent.obstacles import ObstacleMap

CACHE_DIR = Path(__file__).resolve().parents[2] / "data" / "osm"
OVERPASS_URLS = ("https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter",
                 "https://overpass.private.coffee/api/interpreter")
TREE_RADIUS_M = 3.0


def overpass_query(south: float, west: float, north: float, east: float) -> str:
    b = f"{south:.6f},{west:.6f},{north:.6f},{east:.6f}"
    return (f'[out:json][timeout:90];(way["building"]({b});way["natural"="wood"]({b});'
            f'way["landuse"="forest"]({b});node["natural"="tree"]({b}););out geom;')


def cache_path(south: float, west: float, north: float, east: float, cache_dir: str | Path = CACHE_DIR) -> Path:
    q = overpass_query(south, west, north, east)
    return Path(cache_dir) / f"{hashlib.sha1(q.encode()).hexdigest()[:16]}.json"


def fetch(south: float, west: float, north: float, east: float, cache_dir: str | Path = CACHE_DIR) -> dict:
    q = overpass_query(south, west, north, east)
    cache = cache_path(south, west, north, east, cache_dir)
    if cache.exists():
        return json.loads(cache.read_text())
    body = urllib.parse.urlencode({"data": q}).encode()
    errors = []
    for attempt in range(2):
        for url in OVERPASS_URLS:
            req = urllib.request.Request(url, data=body,
                                         headers={"User-Agent": "swarm-failover-sim/0.1 (research simulation)"})
            try:
                with urllib.request.urlopen(req, timeout=120) as resp:
                    data = json.loads(resp.read().decode())
                break
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
                errors.append(f"{url}: {exc}")
        else:
            time.sleep(10)
            continue
        break
    else:
        raise RuntimeError("Overpass download failed: " + "; ".join(errors))
    data["source"] = url
    data["query"] = q
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps(data))
    return data


def to_obstacles(data: dict, frame: EnuFrame) -> tuple[ObstacleMap, dict, list[str]]:
    """(obstacle map, counts, kind of every polygon: "building" or "wood")."""
    polys, circles, kinds = [], [], []
    counts = {"buildings": 0, "woods": 0, "trees": 0}
    for el in data.get("elements", []):
        tags = el.get("tags", {})
        if el["type"] == "node" and tags.get("natural") == "tree":
            e, n, _ = frame.to_enu(GeoPoint(el["lat"], el["lon"], 0.0))
            circles.append((e, n, TREE_RADIUS_M))
            counts["trees"] += 1
        elif el["type"] == "way" and len(el.get("geometry", [])) >= 4:
            pts = [frame.to_enu(GeoPoint(p["lat"], p["lon"], 0.0))[:2] for p in el["geometry"]]
            if pts[0] != pts[-1]:
                continue            # not a closed area
            polys.append(pts)
            kinds.append("building" if "building" in tags else "wood")
            counts["buildings" if "building" in tags else "woods"] += 1
    return ObstacleMap(polys, circles), counts, kinds
