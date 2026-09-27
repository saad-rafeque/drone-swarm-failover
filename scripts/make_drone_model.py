#!/usr/bin/env python3
"""Generate the quadcopter model used by the 3-D view (src/swarm_tools/gcs/static/drone.glb).

A simple X-frame quadcopter built from boxes and cylinders: white body, dark arms and motors,
propeller discs, landing skids, red lights on the front arms and green ones at the back. About
0.55 m motor to motor; the nose points along glTF +Z (Cesium turns that into +X). Written as
binary glTF 2.0 with numpy only.

Usage: python3 scripts/make_drone_model.py [--out src/swarm_tools/gcs/static/drone.glb]
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def box(cx, cy, cz, sx, sy, sz, yaw=0.0):
    """Axis-aligned box (z up) rotated by yaw about z; returns (positions, normals, indices)."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    faces = [((1, 0, 0), [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)]),
             ((-1, 0, 0), [(-hx, hy, -hz), (-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz)]),
             ((0, 1, 0), [(hx, hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz)]),
             ((0, -1, 0), [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)]),
             ((0, 0, 1), [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]),
             ((0, 0, -1), [(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)])]
    c, s = math.cos(yaw), math.sin(yaw)
    rot = lambda v: (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])  # noqa: E731
    pos, nrm, idx = [], [], []
    for n, quad in faces:
        base = len(pos)
        for v in quad:
            r = rot(v)
            pos.append((r[0] + cx, r[1] + cy, r[2] + cz))
            nrm.append(rot(n))
        idx += [base, base + 1, base + 2, base, base + 2, base + 3]
    return pos, nrm, idx


def cylinder(cx, cy, cz, r, h, seg=24):
    """Vertical cylinder with caps."""
    pos, nrm, idx = [], [], []
    for k in range(seg):
        a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
        base = len(pos)
        for a in (a0, a1):
            pos.append((cx + r * math.cos(a), cy + r * math.sin(a), cz - h / 2))
            nrm.append((math.cos(a), math.sin(a), 0.0))
        for a in (a0, a1):
            pos.append((cx + r * math.cos(a), cy + r * math.sin(a), cz + h / 2))
            nrm.append((math.cos(a), math.sin(a), 0.0))
        idx += [base, base + 1, base + 3, base, base + 3, base + 2]
    for z, nz in ((cz + h / 2, 1.0), (cz - h / 2, -1.0)):
        centre = len(pos)
        pos.append((cx, cy, z))
        nrm.append((0.0, 0.0, nz))
        ring = len(pos)
        for k in range(seg):
            a = 2 * math.pi * k / seg
            pos.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
            nrm.append((0.0, 0.0, nz))
        for k in range(seg):
            a, b = ring + k, ring + (k + 1) % seg
            idx += [centre, a, b] if nz > 0 else [centre, b, a]
    return pos, nrm, idx


def merge(parts):
    pos, nrm, idx = [], [], []
    for p, n, i in parts:
        base = len(pos)
        pos += p
        nrm += n
        idx += [base + k for k in i]
    return pos, nrm, idx


def build() -> dict[str, tuple]:
    """Parts by material. Chunkier than a real frame so the drone still reads when drawn only ~60 pixels wide."""
    arm = 0.275                                    # motor distance from the centre (x-frame, 0.55 m diagonal)
    diag = [(math.cos(a), math.sin(a)) for a in (math.radians(45), math.radians(135), math.radians(225), math.radians(315))]
    body = [box(0.01, 0, 0.0, 0.24, 0.17, 0.09)]                                      # white shell (tinted per role)
    body += [box(0.0, 0, 0.055, 0.15, 0.11, 0.03)]                                    # top plate
    body += [box(0.125, 0, 0.0, 0.03, 0.12, 0.06)]                                    # nose block
    frame = [box(dx * arm / 2, dy * arm / 2, 0, arm, 0.045, 0.03, math.atan2(dy, dx)) for dx, dy in diag]  # arms
    frame += [cylinder(dx * arm, dy * arm, 0.015, 0.036, 0.055, 16) for dx, dy in diag]  # motors
    frame += [cylinder(0, 0, 0.1, 0.022, 0.06, 12)]                                  # GPS mast
    frame += [box(0, y, -0.085, 0.24, 0.018, 0.018) for y in (-0.07, 0.07)]           # skids
    frame += [box(x, y, -0.055, 0.018, 0.018, 0.06) for x in (-0.07, 0.07) for y in (-0.07, 0.07)]
    props = [cylinder(dx * arm, dy * arm, 0.047, 0.13, 0.006, 32) for dx, dy in diag]    # propeller discs
    nose = [cylinder(dx * arm, dy * arm, -0.02, 0.022, 0.02, 12) for dx, dy in diag if dx > 0]  # red lights front
    tail = [cylinder(dx * arm, dy * arm, -0.02, 0.022, 0.02, 12) for dx, dy in diag if dx < 0]  # green lights back
    return {"body": merge(body), "frame": merge(frame), "props": merge(props), "nose": merge(nose), "tail": merge(tail)}


def write_glb(parts: dict, materials: dict, out: Path) -> None:
    buf = bytearray()
    views, accessors, meshes, nodes = [], [], [], []

    def add_view(data: bytes, target: int) -> int:
        while len(buf) % 4:
            buf.append(0)
        views.append({"buffer": 0, "byteOffset": len(buf), "byteLength": len(data), "target": target})
        buf.extend(data)
        return len(views) - 1

    for name, (pos, nrm, idx) in parts.items():
        # model axes: x forward, y left, z up. glTF: +Y up, +Z forward, +X left -> (x, y, z) -> (y, z, x)
        p = np.array([(y, z, x) for x, y, z in pos], dtype=np.float32)
        n = np.array([(y, z, x) for x, y, z in nrm], dtype=np.float32)
        i = np.array(idx, dtype=np.uint16)
        vp = add_view(p.tobytes(), 34962)
        accessors.append({"bufferView": vp, "componentType": 5126, "count": len(p), "type": "VEC3",
                          "min": p.min(axis=0).tolist(), "max": p.max(axis=0).tolist()})
        vn = add_view(n.tobytes(), 34962)
        accessors.append({"bufferView": vn, "componentType": 5126, "count": len(n), "type": "VEC3"})
        vi = add_view(i.tobytes(), 34963)
        accessors.append({"bufferView": vi, "componentType": 5123, "count": len(i), "type": "SCALAR"})
        a = len(accessors)
        meshes.append({"name": name, "primitives": [{"attributes": {"POSITION": a - 3, "NORMAL": a - 2}, "indices": a - 1,
                                                     "material": list(materials).index(name)}]})
        nodes.append({"mesh": len(meshes) - 1, "name": name})
    gltf = {
        "asset": {"version": "2.0", "generator": "swarm-failover scripts/make_drone_model.py"},
        "scene": 0, "scenes": [{"nodes": list(range(len(nodes)))}], "nodes": nodes, "meshes": meshes,
        "materials": [dict(m, name=k) for k, m in materials.items()],
        "buffers": [{"byteLength": len(buf)}], "bufferViews": views, "accessors": accessors,
    }
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    while len(buf) % 4:
        buf.append(0)
    total = 12 + 8 + len(js) + 8 + len(buf)
    out.write_bytes(struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(js), 0x4E4F534A) + js
                    + struct.pack("<II", len(buf), 0x004E4942) + bytes(buf))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "src/swarm_tools/gcs/static/drone.glb"))
    args = ap.parse_args()
    materials = {   # the 3-D view multiplies the whole model by a role colour, so the body is white and the frame dark
        "body": {"pbrMetallicRoughness": {"baseColorFactor": [0.97, 0.98, 1.0, 1.0], "metallicFactor": 0.05, "roughnessFactor": 0.45}},
        "frame": {"pbrMetallicRoughness": {"baseColorFactor": [0.13, 0.15, 0.17, 1.0], "metallicFactor": 0.3, "roughnessFactor": 0.55}},
        "props": {"pbrMetallicRoughness": {"baseColorFactor": [0.85, 0.88, 0.9, 0.35], "metallicFactor": 0.0, "roughnessFactor": 0.6},
                  "alphaMode": "BLEND", "doubleSided": True},
        "nose": {"pbrMetallicRoughness": {"baseColorFactor": [1.0, 0.15, 0.1, 1.0]}, "emissiveFactor": [1.0, 0.1, 0.05]},
        "tail": {"pbrMetallicRoughness": {"baseColorFactor": [0.1, 1.0, 0.3, 1.0]}, "emissiveFactor": [0.1, 1.0, 0.3]},
    }
    out = Path(args.out)
    write_glb(build(), materials, out)
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
