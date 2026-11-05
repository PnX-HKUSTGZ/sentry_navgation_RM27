#!/usr/bin/env python3
"""Generate a dense, visible-surface prior from the actual RM27 Gazebo world.

Uses the offline trimesh/embreex dependencies in rmuc2027_requirements.txt;
does not require the official STEP or change the simulation geometry.
"""

import argparse
import json
from pathlib import Path
import time

import numpy as np
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector

from check_rmuc2027_map_alignment import world_triangles, summary
from generate_rmuc2027_pcd import (
    BRINGUP, ROOT, digest, read_pcd, sample_surface, viewpoints,
    visible_surface, write_pcd,
)


def check_alignment(points, mesh, rays):
    rng = np.random.default_rng(27027)
    probes = points[rng.choice(len(points), min(100000, len(points)), replace=False)]
    residuals = np.full(len(probes), np.inf)
    for direction in np.vstack((np.eye(3), -np.eye(3))):
        hits, ids, _ = rays.intersects_location(probes-direction*.03,
            np.tile(direction, (len(probes), 1)), multiple_hits=False)
        residuals[ids] = np.minimum(residuals[ids], np.linalg.norm(hits-probes[ids], axis=1))
    ray_summary = summary(residuals)
    # First-hit rays can stop on an adjacent occluding face in thin structures.
    # Resolve those cases with exact point-to-triangle proximity, rather than
    # changing the distance threshold or dropping the unmatched samples.
    ambiguous = residuals > .001
    if np.any(ambiguous):
        _, exact, _ = trimesh.proximity.closest_point(mesh, probes[ambiguous])
        residuals[ambiguous] = np.minimum(residuals[ambiguous], exact)
    result = summary(residuals)
    result['within_1mm'] = float(np.mean(residuals <= .001))
    result['max_m'] = float(residuals.max())
    result['exact_proximity_fallback_points'] = int(ambiguous.sum())
    result['raw_six_direction_ray_probe'] = ray_summary
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BRINGUP/'pcd/simulation/RMUC2027.pcd')
    parser.add_argument('--report-dir', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=.0075)
    parser.add_argument('--voxel', type=float, default=.037)
    parser.add_argument('--min-bytes', type=int, default=5_800_000)
    parser.add_argument('--max-bytes', type=int, default=6_200_000)
    parser.add_argument('--input-pcd', type=Path, help='Downsample a verified world-surface export without resampling geometry')
    parser.add_argument('--representative', choices=['first', 'nearest_centroid'], default='nearest_centroid',
        help='Keep an original surface point, preferring the point nearest each voxel centroid')
    parser.add_argument('--check-only', action='store_true', help='Validate an existing export against current world assets')
    args = parser.parse_args()
    if not (0 < args.spacing <= args.voxel and 1024 < args.min_bytes < args.max_bytes < 10_000_000):
        parser.error('Require 0 < spacing <= voxel and 1024 < min-bytes < max-bytes < 10 MB')
    started = time.monotonic()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    print('Reading placed Gazebo visual meshes', flush=True)
    triangles, provenance = world_triangles()
    mesh = trimesh.Trimesh(vertices=triangles.reshape(-1, 3),
        faces=np.arange(len(triangles)*3).reshape(-1, 3), process=False)
    rays = RayMeshIntersector(mesh)
    if args.check_only:
        points = read_pcd(args.output)
        metadata = json.loads(args.output.with_name(args.output.stem+'_manifest.json').read_text())
        sources_match = all((ROOT/p).is_file() and digest(ROOT/p) == expected
                           for p, expected in metadata['source_asset_sha256'].items())
        alignment = check_alignment(points, mesh, rays)
        checks = dict(source_assets_match=sources_match,
            manifest_hash_matches=digest(args.output) == metadata['map_sha256'],
            manifest_point_count_matches=len(points) == metadata['pcd_points'],
            finite_xyz=bool(np.isfinite(points).all()),
            file_size_in_budget=args.min_bytes <= args.output.stat().st_size < args.max_bytes,
            frame_is_map=metadata['frame_id'] == 'map' and metadata['units'] == 'metres',
            world_transform_is_identity=bool(np.allclose(metadata['T_map_world'], np.eye(4))),
            surface_within_1mm=alignment['within_1mm'] > .999)
        report = dict(passed=all(checks.values()), checks=checks,
            pcd_sha256=digest(args.output), reference_world=provenance,
            source_assets_match=sources_match, source_surface_distance_bound=alignment,
            pcd_points=len(points), pcd_bytes=args.output.stat().st_size,
            voxel_size_m=metadata['voxel_size_m'], validation_wall_sec=time.monotonic()-started)
        (args.report_dir/'alignment.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        if not report['passed']:
            raise SystemExit('World surface alignment check failed')
        return
    cutoff = float(mesh.bounds[0, 2]) + .003
    parent = None
    if args.input_pcd:
        parent = json.loads(args.input_pcd.with_name(args.input_pcd.stem+'_manifest.json').read_text())
        if (digest(args.input_pcd) != parent['map_sha256'] or parent['frame_id'] != 'map'
                or not np.allclose(parent['T_map_world'], np.eye(4))
                or not all(digest(ROOT/p) == h for p, h in parent['source_asset_sha256'].items())):
            raise RuntimeError('Input PCD provenance does not match the current world')
        visible = read_pcd(args.input_pcd)
        viewpoint_count = parent['visibility_viewpoints']
        print(f'Downsampling {len(visible)} verified visible surface points', flush=True)
    else:
        samples = sample_surface(triangles, args.spacing, cutoff)
        print(f'Sampled {len(samples)} points from {len(triangles)} triangles', flush=True)
        _, ids = np.unique(np.floor(samples/(args.voxel*.5)).astype(np.int32), axis=0, return_index=True)
        candidates = samples[ids]
        del samples
        cameras = viewpoints(mesh, rays)
        if not len(cameras):
            raise RuntimeError('No supported sensor viewpoints in simulation world')
        viewpoint_count = len(cameras)
        print(f'Visibility: {len(candidates)} candidates, {viewpoint_count} viewpoints', flush=True)
        visible = visible_surface(candidates, cameras, rays)
        del candidates
    leaf = args.voxel
    # Keep original face samples rather than centroids across wall corners.
    while True:
        _, ids, inverse = np.unique(np.floor(visible/leaf).astype(np.int32), axis=0,
            return_index=True, return_inverse=True)
        points = visible[ids]
        size = points.nbytes + 1024
        if size >= args.max_bytes:
            leaf *= 1.04
        elif size < args.min_bytes:
            if len(points) == len(visible):
                raise RuntimeError('Insufficient unique visible samples; reduce --spacing')
            leaf /= 1.04
        else:
            break
    if args.representative == 'nearest_centroid':
        counts = np.bincount(inverse)
        centroids = np.column_stack([np.bincount(inverse, weights=visible[:,axis])/counts
            for axis in range(3)])
        error = np.sum((visible-centroids[inverse])**2, axis=1)
        order = np.lexsort((error, inverse))
        sorted_voxels = inverse[order]
        ids = order[np.r_[True, sorted_voxels[1:] != sorted_voxels[:-1]]]
        points = visible[ids]
    write_pcd(args.output, points)
    reread = read_pcd(args.output)
    assert np.array_equal(points, reread) and np.isfinite(points).all()
    assert args.min_bytes <= args.output.stat().st_size < args.max_bytes
    alignment = check_alignment(reread, mesh, rays)
    passed = bool(alignment['within_1mm'] > .999)
    assets = ROOT/'src/pnx-stimulation/rm_27_stimulation/meshes'
    source_paths = [ROOT/provenance['world_path']]
    for name in ('RM27_battlefield_visual', 'RM27_battlefield_collision'):
        source_paths.extend(sorted(p for p in (assets/name).rglob('*') if p.is_file()))
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in source_paths}
    metadata = dict(schema_version=1, frame_id='map', units='metres',
        source_type='gazebo_RM27_world_visible_surface_prior',
        source_world_key='RMUC2027', reference_world=provenance,
        source_asset_sha256=hashes, T_map_world=np.eye(4).tolist(),
        attribution='RM27 Gazebo field assets; source attribution retained in mesh bundle',
        prior_pcd_transform=[0.]*6, transform_prior_map_with_lidar_offset=False,
        viewpoint_semantics='identity; coordinates already in Gazebo world/map',
        triangles=len(triangles), random_seed=2027, sample_spacing_m=args.spacing,
        voxel_size_m=leaf, representative=args.representative+' original visible surface point per voxel',
        visibility_viewpoints=viewpoint_count, visible_samples=len(visible),
        visibility_tolerance_m=.008, underground_cutoff_m=cutoff,
        pcd_points=len(points), pcd_bytes=args.output.stat().st_size,
        min_bytes=args.min_bytes, max_bytes=args.max_bytes,
        pcd_bounds_m=[points.min(0).tolist(), points.max(0).tolist()],
        map_sha256=digest(args.output), pcd_sha256=digest(args.output))
    if parent is not None:
        metadata['sample_spacing_m'] = parent['sample_spacing_m']
        metadata['downsampled_from'] = dict(pcd_sha256=parent['map_sha256'],
            pcd_points=parent['pcd_points'], voxel_size_m=parent['voxel_size_m'],
            method='original surface representative per voxel; no coordinate transform')
    args.output.with_name(args.output.stem+'_manifest.json').write_text(json.dumps(metadata, indent=2)+'\n')
    report = dict(passed=passed, pcd_sha256=metadata['map_sha256'],
        reference_world=provenance, source_surface_distance_bound=alignment,
        pcd_points=len(points), pcd_bytes=metadata['pcd_bytes'],
        voxel_size_m=leaf, generation_wall_sec=time.monotonic()-started,
        runtime_validation='pending; geometric consistency does not establish localization success')
    (args.report_dir/'alignment.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)
    if not passed:
        raise SystemExit('World surface alignment check failed')


if __name__ == '__main__':
    main()
