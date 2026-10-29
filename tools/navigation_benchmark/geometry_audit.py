#!/usr/bin/env python3
"""Inventory vertical mesh intersections; never infer body or swept clearance.

For timed 3D poses and measured transforms, use geometry_swept.py, which checks
oriented body boxes against mesh surfaces and conservatively screens intervals.
Legacy logs lack the full poses/transforms that analysis requires.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/global_relocalization'))
from actual_collision_support import CollisionSupportScene

WORLD = ROOT / 'src/rm_27_stimulation/world/RMUC2026_world/RMUC2026_world.world'
OUTPUT = ROOT / 'tools/navigation_benchmark/results/20261028/geometry_evidence.json'
QUERIES = [(4.808, 6.532), (5.031, 6.815), (4.5, 6.5), (5.0, 6.5), (5.0, 7.0), (4.5, 7.0)]


def intersections(scene, x, y):
    ids = scene.buckets.get((math.floor(x / .5), math.floor(y / .5)), [])
    result = set()
    for i in ids:
        a, b, c = scene.triangles[i]
        den = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(den) < 1e-12:
            continue
        u = ((b[1]-c[1])*(x-c[0]) + (c[0]-b[0])*(y-c[1])) / den
        v = ((c[1]-a[1])*(x-c[0]) + (a[0]-c[0])*(y-c[1])) / den
        if u >= -1e-7 and v >= -1e-7 and u + v <= 1 + 1e-7:
            z = u*a[2] + v*b[2] + (1-u-v)*c[2]
            if -.1 <= z <= .7:
                result.add((round(float(z), 5), scene.names[i].split('#')[0]))
    return [{'z_m': z, 'collision': name} for z, name in sorted(result)]


def main():
    scene = CollisionSupportScene.from_world(WORLD)
    sources = {str(Path(p).relative_to(ROOT)): digest for p, digest in scene.source_sha256.items()}
    sources[str(Path(__file__).resolve().relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result = {
        'method': 'Vertical lines intersect transformed collision STL triangles and boxes from SDF world; z is world frame. All surface orientations are included.',
        'inputs_sha256': sources,
        'queries': [{'xy_m': [x, y], 'intersections': intersections(scene, x, y)} for x, y in QUERIES],
        'observed_log': {'path': 'tools/navigation_benchmark/results/20261028/safe_baseline/repeat_01/launch.log', 'footprint_query_map_xy_m': [5.031, 6.815], 'ceiling_odom_z_m': .132, 'reference_odom_z_m': .060},
        'limitations': ['Point intersections do not establish swept vehicle clearance or collision.', 'Log heights are odom-frame and geometry heights world-frame; direct comparison requires measured TF.', 'Vertical triangles are omitted, and physics-engine collision margins and convex hull substitutions are not modeled.', 'The static scene does not prove the origin of an individual lidar return.']
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result['queries'], indent=2))
    print('Wrote', OUTPUT)


if __name__ == '__main__':
    main()
