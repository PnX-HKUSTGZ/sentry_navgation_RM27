#!/usr/bin/env python3
"""Reproduce coordinate fit and nearby mesh surfaces for two logged ramp stops."""
import glob
import hashlib
import json
from pathlib import Path

import numpy as np
from geometry_audit import ROOT, WORLD, CollisionSupportScene, intersections
from actual_collision_support import _clip_z, _xy_polygon_distance

BASE = ROOT / 'tools/navigation_benchmark/results/20261028/safe_baseline'
OUTPUT = ROOT / 'tools/navigation_benchmark/results/20261028/ramp_geometry_evidence.json'
# Coordinates and occupied lower boundaries transcribed from the quoted log lines.
QUERIES = {
    'repeat_01': [(5.050, 6.803, .079), (5.031, 6.815, .132), (5.053, 6.804, .149)],
    'repeat_02': [(-5.064, -3.776, .144), (-5.058, -3.761, .267)],
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    scene = CollisionSupportScene.from_world(WORLD)
    result = {'method': 'Planar rigid fit to low-speed truth/odom samples and median height offset; intersect static collision triangles with a 5 cm height band above each logged occupied lower boundary, then measure XY distance to clipped polygons.',
              'sources_sha256': dict(scene.source_sha256), 'repeats': {},
              'limitations': ['Truth and odometry samples are not perfectly synchronized; fit residuals describe these recorded samples, not a calibrated systematic error bound.',
                  'The 75 mm neighborhood is a reported search radius, not a proven localization, map or lidar error bound. Nearest-mesh matches do not prove individual returns or causal origin.',
                  'A logged ceiling is the conservative bottom of an occupied voxel run, not necessarily a real ceiling. Other point returns or dynamics may contribute.',
                  'A vertical ray reports nearby surfaces only; no sampled point or nearest surface demonstrates whole-body collision, swept clearance, or that a stop was necessary.']}
    result['sources_sha256'][str(Path(__file__).resolve())] = sha(__file__)
    for rep, queries in QUERIES.items():
        samples = []
        for path in sorted(glob.glob(str(BASE / rep / 'run_??.json'))):
            result['sources_sha256'][path] = sha(path)
            for s in json.loads(Path(path).read_text())['samples']:
                keys = ['x', 'y', 'z', 'odom_x', 'odom_y', 'odom_z', 'odom_stamp', 't', 'vx', 'vy']
                if all(isinstance(s.get(k), (int, float)) and np.isfinite(s[k]) for k in keys) and abs(s['t']-s['odom_stamp']) <= .03 and np.hypot(s['vx'],s['vy']) < .05:
                    samples.append(s)
        a = np.array([[s['odom_x'], s['odom_y']] for s in samples])
        b = np.array([[s['x'], s['y']] for s in samples])
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a-ac).T @ (b-bc))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            raise ValueError('fit produced reflection')
        translation = bc - rotation @ ac
        residual = np.linalg.norm(a @ rotation.T + translation - b, axis=1)
        dz = np.array([s['z']-s['odom_z'] for s in samples])
        z_offset = float(np.median(dz))
        entry = {'sample_count': len(samples), 'fit_yaw_rad': float(np.arctan2(rotation[1,0],rotation[0,0])),
                 'fit_xy_translation_m': translation.tolist(), 'median_z_translation_m': z_offset,
                 'xy_residual_p95_m': float(np.quantile(residual,.95)), 'xy_residual_max_m': float(residual.max()),
                 'z_residual_max_m': float(np.max(np.abs(dz-z_offset))), 'queries': []}
        log = BASE / rep / 'launch.log'
        result['sources_sha256'][str(log)] = sha(log)
        for x, y, ceiling in queries:
            z = ceiling + z_offset
            xy = np.array([x,y])
            near = np.all(scene.maximum[:,:2] >= xy-.075, axis=1) & np.all(scene.minimum[:,:2] <= xy+.075, axis=1) & (scene.maximum[:,2] >= z) & (scene.minimum[:,2] <= z+.05)
            matches = {}
            for i in np.flatnonzero(near):
                poly = _clip_z(scene.triangles[i],z,True)
                if len(poly):
                    poly = _clip_z(poly,z+.05,False)
                if len(poly):
                    dist = _xy_polygon_distance(xy,poly)
                    if dist <= .075:
                        name = scene.names[i].split('#')[0]
                        matches[name] = min(matches.get(name,float('inf')),dist)
            entry['queries'].append({'query_map_xy_m': [x,y], 'logged_odom_ceiling_lower_m': ceiling,
                'estimated_world_lower_m': z, 'world_band_upper_m': z+.05,
                'nearest_surface_xy_distance_m': dict(sorted(matches.items(),key=lambda item:item[1])),
                'vertical_intersections_at_query': intersections(scene,x,y)})
        result['repeats'][rep] = entry
    OUTPUT.write_text(json.dumps(result,indent=2)+'\n')
    print(OUTPUT)


if __name__ == '__main__':
    main()
