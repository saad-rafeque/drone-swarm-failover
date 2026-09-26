"""Buildings and trees from OpenStreetMap (Overpass API) as an ObstacleMap in the shared ENU frame.

Low-altitude profile: the swarm flies below rooftops and tree canopies, so every building footprint,
wood/forest area and single mapped tree is an obstacle to go around. OSM rarely records heights, so
flying over is not attempted. Downloads are cached as JSON (data (c) OpenStreetMap contributors, ODbL).
"""
from __future__ import annotations

import hashlib
import json
import math
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
LEVEL_M = 3.2              # storey height when OSM gives building:levels
DEFAULT_BUILDING_M = 9.0   # about three storeys when OSM gives no height at all
WOOD_M = 15.0              # canopy height of woods and forests without a height tag
TREE_M = 12.0


def height_m(tags: dict, default: float) -> float:
    """Height from OSM tags: height (metres), else building:levels x 3.2 m, else the default."""
    for key, scale in (("height", 1.0), ("building:levels", LEVEL_M)):
        v = str(tags.get(key, "")).strip().lower().replace("m", "").replace(",", ".").strip()
        try:
            if v:
                return float(v.split(";")[0].split()[0]) * scale
        except ValueError:
            pass
    return default


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


def corridor_boxes(frame: EnuFrame, goal_en: tuple[float, float], tile_m: float = 3000.0,
                   margin_deg: float = 0.004) -> list[tuple[float, float, float, float]]:
    """Lat/lon boxes (south, west, north, east) covering the straight route home -> goal in tile_m pieces,
    each widened by margin_deg (about 400 m): the strip a long route needs, not the whole region."""
    d = math.hypot(*goal_en)
    n = max(1, math.ceil(d / tile_m))
    pts = [frame.surface_point(goal_en[0] * k / n, goal_en[1] * k / n) for k in range(n + 1)]
    return [(min(a.lat_deg, b.lat_deg) - margin_deg, min(a.lon_deg, b.lon_deg) - margin_deg,
             max(a.lat_deg, b.lat_deg) + margin_deg, max(a.lon_deg, b.lon_deg) + margin_deg)
            for a, b in zip(pts, pts[1:])]


def fetch_boxes(boxes, progress=None, pause_s: float = 1.0, cache_dir: str | Path = CACHE_DIR) -> dict:
    """Download (or read from cache) every box and merge the elements, without duplicates. Pauses between
    real downloads to be gentle with the public Overpass servers."""
    elements: dict[tuple[str, int], dict] = {}
    for k, bb in enumerate(boxes):
        cached = cache_path(*bb, cache_dir).exists()
        data = fetch(*bb, cache_dir=cache_dir)
        for el in data.get("elements", []):
            elements[(el["type"], el["id"])] = el
        if progress:
            progress(k + 1, len(boxes))
        if not cached and k + 1 < len(boxes):
            time.sleep(pause_s)
    return {"elements": list(elements.values())}


def to_obstacles(data: dict, frame: EnuFrame, min_height_m: float = 0.0) -> tuple[ObstacleMap, dict, list[str]]:
    """(obstacle map, counts, kind of every polygon: "building" or "wood").

    min_height_m > 0 keeps only what reaches the flight height: at 30 m most houses and trees are below
    the drones, so only taller structures remain obstacles (counted in counts["below"] otherwise)."""
    polys, circles, kinds = [], [], []
    counts = {"buildings": 0, "woods": 0, "trees": 0, "below": 0}
    for el in data.get("elements", []):
        tags = el.get("tags", {})
        default = TREE_M if el["type"] == "node" else (DEFAULT_BUILDING_M if "building" in tags else WOOD_M)
        if min_height_m > 0.0 and height_m(tags, default) < min_height_m:
            counts["below"] += 1
            continue
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
