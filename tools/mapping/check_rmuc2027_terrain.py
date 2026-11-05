#!/usr/bin/env python3
"""Check downsampling coverage and terrain against actual world collisions."""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
import yaml

from generate_rmuc2027_pcd import ROOT, BRINGUP, digest, read_pcd

sys.path.insert(0, str(ROOT/'tools/global_relocalization'))
from actual_collision_support import CollisionSupportScene


def statistics(values):
    return dict(samples=len(values), median_m=float(np.median(values)),
        p95_m=float(np.quantile(values, .95)), max_m=float(np.max(values)),
        within_5cm=float(np.mean(values <= .05)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    points, reference = read_pcd(args.pcd), read_pcd(args.reference)
    meta = json.loads(args.pcd.with_name(args.pcd.stem+'_manifest.json').read_text())
    world = ROOT/meta['reference_world']['world_path']
    scene = CollisionSupportScene.from_world(world)
    tree, reference_tree = cKDTree(points), cKDTree(reference)
    coverage, _ = tree.query(reference, workers=2)
    map_path = BRINGUP/'map/simulation/RMUC2027.yaml'
    config = yaml.safe_load(map_path.read_text())
    pixels = np.asarray(Image.open(map_path.parent/config['image']))
    free = (255-pixels.astype(float))/255 <= config['free_thresh']
    clear = distance_transform_edt(free)*config['resolution']
    ox, oy, yaw = config['origin']
    if yaw != 0:
        raise ValueError('Expected axis-aligned navigation map')
    height, width = pixels.shape
    x, y = np.meshgrid(np.arange(ox+.125, ox+width*config['resolution'], .25),
        np.arange(oy+.125, oy+height*config['resolution'], .25))
    xy = np.column_stack((x.ravel(), y.ravel()))
    col = np.floor((xy[:,0]-ox)/config['resolution']).astype(int)
    row = height-1-np.floor((xy[:,1]-oy)/config['resolution']).astype(int)
    xy = xy[clear[row,col] >= .15]
    support = np.array([scene.height_at(p) for p in xy])
    finite = np.isfinite(support)
    queries = np.column_stack((xy[finite], support[finite]))
    distances, indices = tree.query(queries, workers=2)
    baseline, _ = reference_tree.query(queries, workers=2)
    z_errors = np.abs(points[indices,2]-queries[:,2])
    elevated = queries[:,2] > .10
    slopes = []
    for p in queries:
        hx = [scene.height_at(p[:2]+[d,0]) for d in [-.025,.025]]
        hy = [scene.height_at(p[:2]+[0,d]) for d in [-.025,.025]]
        slopes.append(np.degrees(np.arctan(np.hypot(hx[1]-hx[0],hy[1]-hy[0])/.05)))
    slopes = np.array(slopes)
    ramps = (slopes > 2) & (slopes < 28)
    checks = dict(no_coordinate_motion=meta['prior_pcd_transform'] == [0.]*6,
        full_dense_cloud_coverage=bool(coverage.max() <= np.sqrt(3)*meta['voxel_size_m']+1e-6),
        support_coverage_preserved=bool(np.mean(distances <= .05) >= np.mean(baseline <= .05)-.001),
        support_p95_below_5cm=bool(np.quantile(distances,.95) < .05),
        terrain_z_p95_below_3cm=bool(np.quantile(z_errors,.95) < .03),
        ramps_and_platforms_sampled=bool(ramps.any() and elevated.any()))
    report = dict(passed=all(checks.values()), checks=checks,
        pcd_sha256=digest(args.pcd), reference_sha256=digest(args.reference),
        source_world=str(world.relative_to(ROOT)), source_world_sha256=digest(world),
        map_yaml_sha256=digest(map_path), map_image_sha256=digest(map_path.parent/config['image']),
        terrain_grid_spacing_m=.25, obstacle_clearance_m=.15,
        map_free_probe_points=len(xy), collision_supported_probe_points=len(queries),
        unsupported_probe_points=int((~finite).sum()),
        dense_to_candidate_distance=statistics(coverage),
        collision_support_to_dense_pcd=statistics(baseline),
        collision_support_to_candidate_pcd=statistics(distances),
        nearest_terrain_z_error=statistics(z_errors),
        elevated_surface=dict(points=int(elevated.sum()), distance=statistics(distances[elevated])),
        ramp_surface=dict(points=int(ramps.sum()), distance=statistics(distances[ramps])),
        support_height_range_m=[float(queries[:,2].min()),float(queries[:,2].max())],
        scope='geometry and downsampling checks; unknown map cells are excluded, no navigation configuration changes')
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if not report['passed']:
        raise SystemExit('Terrain checks failed')


if __name__ == '__main__':
    main()
