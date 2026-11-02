#!/usr/bin/env python3
"""Offline sensor-only SE(3) graph refinement, with a fixed first sensor gauge.

Only capture odom/cloud fields enter optimization. GT evaluation is a separate
program. The first prototype interpolates keyframe corrections; it does not
claim all-frame leave-one-out refinement or removal of intra-keyframe spikes.
"""
import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class Rules:
    keyframe_translation_m: float = .30
    keyframe_rotation_deg: float = 20.
    keyframe_interval_sec: float = 2.
    candidate_distance_m: float = 1.5
    candidate_rotation_deg: float = 40.
    loop_separation_sec: float = 5.
    match_leaf_m: float = .08
    submap_half_frames: int = 2
    normal_radius_m: float = .35
    normal_min_neighbors: int = 5
    normal_max_curvature: float = .03
    normal_min_middle_ratio: float = .05
    correspondence_m: float = .30
    min_coverage: float = .65
    min_plane_matches: int = 50
    min_heldout_matches: int = 15
    huber_distance_m: float = .03
    max_iterations: int = 20
    max_relative_update_m: float = .30
    max_relative_update_deg: float = 10.
    cycle_translation_m: float = .03
    cycle_rotation_deg: float = 1.
    heldout_max_degradation_m: float = .002
    information_relative_floor: float = 1e-5
    minimum_loop_rank: int = 5
    measurement_sigma_m: float = .03
    maximum_effective_matches: int = 100
    loop_cap_per_node: int = 4
    odometry_translation_sigma_sqrt_sec: float = .01
    odometry_rotation_sigma_sqrt_sec: float = .01
    graph_max_nfev: int = 600
    max_cumulative_position_update_m: float = .5
    max_cumulative_rotation_update_deg: float = 10.


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def invert(t):
    x = np.eye(4)
    x[:3, :3] = t[:3, :3].T
    x[:3, 3] = -x[:3, :3] @ t[:3, 3]
    return x


def transform(points, t):
    return points @ t[:3, :3].T + t[:3, 3]


def relative_error(a, b, measured):
    error = invert(measured) @ invert(a) @ b
    return np.r_[Rotation.from_matrix(error[:3, :3]).as_rotvec(), error[:3, 3]]


def voxel(points, leaf):
    keys = np.floor(points / leaf).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inv)
    return np.column_stack([np.bincount(inv, weights=points[:, k]) / count for k in range(3)])


def information_root(h, relative_floor):
    """Retain weak directions as zero information; never fill missing axes."""
    values, vectors = np.linalg.eigh((h + h.T) / 2)
    keep = values > max(1e-12, values[-1] * relative_floor)
    return np.sqrt(np.maximum(values, 0) * keep)[:, None] * vectors.T, int(keep.sum()), values


def normals(points, rules):
    tree = cKDTree(points)
    result = np.zeros_like(points)
    good = np.zeros(len(points), bool)
    for i, neighborhood in enumerate(tree.query_ball_point(points, rules.normal_radius_m)):
        if len(neighborhood) < rules.normal_min_neighbors:
            continue
        values, vectors = np.linalg.eigh(np.cov(points[neighborhood].T, bias=True))
        if (values.sum() > 1e-12 and values[0] / values.sum() <= rules.normal_max_curvature
                and values[1] / max(values[2], 1e-12) >= rules.normal_min_middle_ratio):
            good[i], result[i] = True, vectors[:, 0]
    return tree, result, good


def pair_fit(source, target, target_geometry, initial, rules):
    tree, normal, good = target_geometry
    t = initial.copy()
    train = np.arange(len(source)) % 5 != 0
    hold = ~train

    def correspond(x):
        p = transform(source[x], t)
        distances, ids = tree.query(p)
        valid = (distances <= rules.correspondence_m) & good[ids]
        return p[valid], target[ids[valid]], normal[ids[valid]], distances, valid

    def heldout(pose):
        p = transform(source[hold], pose)
        distance, ids = tree.query(p)
        use = (distance <= rules.correspondence_m) & good[ids]
        residual = np.einsum('ij,ij->i', p[use] - target[ids[use]], normal[ids[use]])
        return {'count': int(use.sum()), 'plane_rmse_m': float(np.sqrt(np.mean(residual ** 2))) if len(residual) else 1e9,
                'capped_distance_rmse_m': float(np.sqrt(np.mean(np.minimum(distance, rules.correspondence_m) ** 2)))}

    before = heldout(initial)
    for _ in range(rules.max_iterations):
        p, q, n, _, _ = correspond(train)
        if len(p) < rules.min_plane_matches:
            return None, {'reason': 'insufficient planar matches'}
        residual = np.einsum('ij,ij->i', p - q, n)
        jacobian = np.c_[np.cross(p, n), n]
        weight = np.minimum(1., rules.huber_distance_m / np.maximum(abs(residual), 1e-12))
        h = jacobian.T @ (jacobian * weight[:, None])
        gradient = jacobian.T @ (weight * residual)
        values, vectors = np.linalg.eigh(h)
        keep = values > max(1e-12, values[-1] * rules.information_relative_floor)
        delta = -vectors[:, keep] @ ((vectors[:, keep].T @ gradient) / values[keep])
        if np.linalg.norm(delta[:3]) > .1 or np.linalg.norm(delta[3:]) > .1:
            return None, {'reason': 'unbounded ICP step'}
        update = np.eye(4)
        update[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
        update[:3, 3] = delta[3:]
        t = update @ t
        if np.linalg.norm(delta) < 1e-5:
            break
    p, q, n, distances, valid = correspond(train)
    jacobian = np.c_[np.cross(p, n), n]
    h = jacobian.T @ jacobian / max(1, len(p)) * min(rules.maximum_effective_matches, len(p)) / rules.measurement_sigma_m ** 2
    # ICP linearizes left increments in target coordinates. Relative graph error
    # lives in the source coordinates (Z^-1 * predicted); transport its tangent.
    adjoint = np.zeros((6, 6)); adjoint[:3, :3] = t[:3, :3]; adjoint[3:, 3:] = t[:3, :3]
    v = t[:3, 3]; skew = np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])
    adjoint[3:, :3] = skew @ t[:3, :3]
    root, rank, eigenvalues = information_root(adjoint.T @ h @ adjoint, rules.information_relative_floor)
    distance_all, _ = tree.query(transform(source, t))
    after = heldout(t)
    relative = t @ invert(initial)
    metrics = {'reason': 'fit', 'coverage': float(np.mean(distance_all <= rules.correspondence_m)),
               'plane_matches': len(p), 'rank': rank, 'information_eigenvalues': eigenvalues.tolist(),
               'heldout_before': before, 'heldout_after': after,
               'origin_update_m': float(np.linalg.norm(t[:3, 3] - initial[:3, 3])),
               'rotation_update_deg': float(Rotation.from_matrix(relative[:3, :3]).magnitude() * 180 / np.pi)}
    if (metrics['coverage'] < rules.min_coverage or after['count'] < rules.min_heldout_matches
            or metrics['origin_update_m'] > rules.max_relative_update_m
            or metrics['rotation_update_deg'] > rules.max_relative_update_deg
            or after['plane_rmse_m'] > before['plane_rmse_m'] + rules.heldout_max_degradation_m
            or after['capped_distance_rmse_m'] > before['capped_distance_rmse_m'] + rules.heldout_max_degradation_m):
        metrics['reason'] = 'coverage, heldout or update gate'
        return None, metrics
    return (t, root), metrics


def validate_pair(forward, reverse, rules, is_loop):
    if forward is None or reverse is None:
        return False, 'independent directional fit failed'
    a, am = forward; b, bm = reverse
    cycle = b[0] @ a[0]
    if np.linalg.norm(cycle[:3, 3]) > rules.cycle_translation_m or Rotation.from_matrix(cycle[:3, :3]).magnitude() > np.deg2rad(rules.cycle_rotation_deg):
        return False, 'independent reverse cycle failed'
    if is_loop and min(am['rank'], bm['rank']) < rules.minimum_loop_rank:
        return False, 'loop not sufficiently observable'
    return True, 'validated geometry'


def select_keyframes(poses, stamps, rules):
    indices = [0]
    for i in range(1, len(poses)):
        last = indices[-1]
        if (np.linalg.norm(poses[i, :3, 3] - poses[last, :3, 3]) >= rules.keyframe_translation_m
                or Rotation.from_matrix(poses[last, :3, :3].T @ poses[i, :3, :3]).magnitude() >= np.deg2rad(rules.keyframe_rotation_deg)
                or stamps[i] - stamps[last] >= rules.keyframe_interval_sec):
            indices.append(i)
    if indices[-1] != len(poses) - 1:
        indices.append(len(poses) - 1)
    return np.array(indices)


def optimize_graph(initial, edges, rules):
    n = len(initial)
    ii = np.array([e['i'] for e in edges]); jj = np.array([e['j'] for e in edges])
    z = np.array([e['measurement'] for e in edges]); roots = np.array([e['root'] for e in edges])
    rz = np.transpose(z[:, :3, :3], (0, 2, 1)); tz = z[:, :3, 3]

    def unpack(x):
        delta = np.zeros((n, 6)); delta[1:] = x.reshape(-1, 6)
        result = initial.copy()
        result[:, :3, :3] = initial[:, :3, :3] @ Rotation.from_rotvec(delta[:, :3]).as_matrix()
        result[:, :3, 3] += delta[:, 3:]
        return result

    def residual(x):
        poses = unpack(x)
        ri = np.transpose(poses[ii, :3, :3], (0, 2, 1))
        relative_r = ri @ poses[jj, :3, :3]
        relative_t = np.einsum('nij,nj->ni', ri, poses[jj, :3, 3] - poses[ii, :3, 3])
        rotation = Rotation.from_matrix(rz @ relative_r).as_rotvec()
        translation = np.einsum('nij,nj->ni', rz, relative_t - tz)
        return np.einsum('nij,nj->ni', roots, np.c_[rotation, translation]).ravel()

    sparsity = lil_matrix((len(edges) * 6, (n - 1) * 6), dtype=int)
    for k, e in enumerate(edges):
        for node in [e['i'], e['j']]:
            if node:
                sparsity[k * 6:(k + 1) * 6, (node - 1) * 6:node * 6] = 1
    x0 = np.zeros((n - 1) * 6)
    before = residual(x0)
    fit = least_squares(residual, x0, jac_sparsity=sparsity.tocsr(), loss='huber', f_scale=2.,
                        max_nfev=rules.graph_max_nfev, ftol=1e-6, xtol=1e-6, gtol=1e-6)
    final = unpack(fit.x)
    return final, {'success': bool(fit.success), 'message': fit.message, 'evaluations': fit.nfev,
                   'initial_normalized_rmse': float(np.sqrt(np.mean(before ** 2))),
                   'final_normalized_rmse': float(np.sqrt(np.mean(residual(fit.x) ** 2))), 'cost': float(fit.cost)}


def interpolate_corrections(raw, indices, optimized, stamps):
    correction = optimized @ np.linalg.inv(raw[indices])
    rotations = Slerp(stamps[indices], Rotation.from_matrix(correction[:, :3, :3]))(stamps).as_matrix()
    result = np.tile(np.eye(4), (len(raw), 1, 1)); result[:, :3, :3] = rotations
    for k in range(3):
        result[:, k, 3] = np.interp(stamps, stamps[indices], correction[:, k, 3])
    return result



def verify_source(source, raw_prior):
    capture = json.loads((source / 'capture.json').read_text())
    manifest = json.loads((raw_prior / 'manifest.json').read_text())
    marker = json.loads((raw_prior / 'raw_prior_export_complete.json').read_text())
    if (manifest.get('capture_complete') is not True or capture.get('failure') is not None
            or capture.get('completed_waypoints') != capture.get('total_waypoints')
            or manifest.get('ground_truth_used_for_scan_fusion') is not False
            or manifest.get('ideal_3d_map_used') is not False
            or manifest.get('optimized_graph_used_for_3d_fusion') is not False
            or manifest.get('all_source_registered_scans_used') is not True
            or manifest.get('dropped_source_scan_indices') != []):
        raise ValueError('raw prior completeness/sensor provenance mismatch')
    if Path(manifest['source_capture_directory']).resolve() != source:
        raise ValueError('raw prior refers to a different capture')
    if (sha256(raw_prior / 'map.pcd') != manifest['map_sha256']
            or marker.get('map_sha256') != manifest['map_sha256']
            or marker.get('manifest_sha256') != sha256(raw_prior / 'manifest.json')
            or marker.get('completed') is not True):
        raise ValueError('raw prior map/completion hash mismatch')
    paths = [raw_prior / 'manifest.json', raw_prior / 'map.pcd', raw_prior / 'raw_prior_export_complete.json', Path(__file__).resolve()]
    for name in ['capture.json', 'mapping_params.yaml', 'manifest.json', 'mapping_binary_provenance.json']:
        expected = manifest['source_sha256'].get(name)
        if expected is None or sha256(source / name) != expected:
            raise ValueError('raw prior source hash mismatch: ' + name)
        paths.append(source / name)
    for name in ['capture.json', 'mapping_params.yaml', 'mapping_binary_provenance.json']:
        if sha256(raw_prior / name) != sha256(source / name):
            raise ValueError('raw prior copied input differs: ' + name)
        paths.append(raw_prior / name)
    clouds = capture['clouds']
    if ([x['index'] for x in clouds] != list(range(len(clouds)))
            or manifest['fused_scans'] != len(clouds)):
        raise ValueError('raw prior frame inventory mismatch')
    for i in range(len(clouds)):
        path = source / 'registered_scans' / f'{i:06d}.npy'
        if not path.is_file() or sha256(path) != sha256(raw_prior / 'registered_scans' / path.name):
            raise ValueError('raw prior scan differs from source: ' + str(i))
        paths.append(path)
    return {str(path.resolve()): sha256(path) for path in paths}


def check_optimized_poses(raw, poses, fit, rules):
    if not fit['success']:
        raise ValueError('graph solver did not converge: ' + fit['message'])
    if not np.isfinite(poses).all():
        raise ValueError('nonfinite optimized poses')
    rotation = poses[:, :3, :3]
    if (not np.allclose(np.transpose(rotation, (0, 2, 1)) @ rotation, np.eye(3), atol=1e-6)
            or not np.allclose(np.linalg.det(rotation), 1., atol=1e-6)
            or not np.allclose(poses[:, 3, :], [0, 0, 0, 1], atol=1e-9)):
        raise ValueError('optimized poses are not SE3')
    position = np.linalg.norm(poses[:, :3, 3] - raw[:, :3, 3], axis=1)
    angle = Rotation.from_matrix(np.transpose(raw[:, :3, :3], (0, 2, 1)) @ rotation).magnitude()
    if position.max() > rules.max_cumulative_position_update_m or angle.max() > np.deg2rad(rules.max_cumulative_rotation_update_deg):
        raise ValueError('cumulative correction exceeds frozen physical guard')
    if not np.allclose(poses[0], raw[0], atol=1e-10):
        raise ValueError('first sensor gauge changed')
    return {'maximum_position_update_m': float(position.max()), 'maximum_rotation_update_deg': float(angle.max() * 180 / np.pi)}


def refine(source, raw_prior, output, rules):
    source, raw_prior, output = Path(source).resolve(), Path(raw_prior).resolve(), Path(output).resolve()
    if output.exists() or output == source or output.is_relative_to(source):
        raise FileExistsError('refuse existing or source output: ' + str(output))
    source_hashes = verify_source(source, raw_prior)
    output.mkdir(parents=True)
    (output / 'source_inputs.json').write_text(json.dumps(source_hashes, indent=2) + '\n')
    (output / 'optimizer_source.py').write_bytes(Path(__file__).read_bytes())
    # Freeze the complete rules before estimating any edge or reading any GT.
    (output / 'rules.json').write_text(json.dumps({'rules': asdict(rules), 'tool_sha256': sha256(__file__),
        'selection_uses_gt_or_queries': False, 'all_frame_leave_one_out_implemented': False}, indent=2) + '\n')
    start = time.monotonic(); capture = json.loads((source / 'capture.json').read_text())
    if capture['failure'] is not None or capture['completed_waypoints'] != capture['total_waypoints']:
        raise ValueError('incomplete survey')
    stamps = np.array([x['stamp'] for x in capture['odom']]); raw = np.array([x['matrix'] for x in capture['odom']])
    clouds = capture['clouds']
    if len(clouds) != len(raw) or any(abs(x['stamp'] - t) > 1e-6 for x, t in zip(clouds, stamps)):
        raise ValueError('this prototype requires complete one-to-one registered cloud/pose timestamps')
    indices = select_keyframes(raw, stamps, rules); initial = raw[indices]; key_times = stamps[indices]
    own_cache, submap_cache = {}, {}

    def cloud(i):
        return np.load(source / 'registered_scans' / f'{i:06d}.npy').astype(float)

    def own(node):
        if node not in own_cache:
            i = indices[node]; own_cache[node] = voxel(transform(cloud(i), invert(raw[i])), rules.match_leaf_m)
        return own_cache[node]

    def submap(node, exclude_index):
        i = indices[node]; frames = [j for j in range(max(0, i - rules.submap_half_frames), min(len(raw), i + rules.submap_half_frames + 1)) if j != exclude_index]
        cache_key = (node, exclude_index if abs(exclude_index - i) <= rules.submap_half_frames else -1)
        # Excluded input frames must not leak into target geometry.
        if cache_key not in submap_cache:
            points = voxel(transform(np.concatenate([cloud(j) for j in frames]), invert(raw[i])), rules.match_leaf_m)
            submap_cache[cache_key] = (points, normals(points, rules))
        return submap_cache[cache_key]

    edges = []; diagnostics = []
    for j in range(1, len(indices)):
        i = j - 1; dt = key_times[j] - key_times[i]
        root = np.diag([1 / (rules.odometry_rotation_sigma_sqrt_sec * np.sqrt(dt))] * 3 + [1 / (rules.odometry_translation_sigma_sqrt_sec * np.sqrt(dt))] * 3)
        edges.append({'i': i, 'j': j, 'kind': 'raw_odometry', 'measurement': invert(initial[i]) @ initial[j], 'root': root})
    candidates = [(i, i + 1, False) for i in range(len(indices) - 1)]
    loops = []
    for a, b in cKDTree(initial[:, :3, 3]).query_pairs(rules.candidate_distance_m):
        if (key_times[b] - key_times[a] >= rules.loop_separation_sec
                and Rotation.from_matrix(initial[a, :3, :3].T @ initial[b, :3, :3]).magnitude() <= np.deg2rad(rules.candidate_rotation_deg)):
            loops.append((a, b, True))
    loops.sort(key=lambda x: (np.linalg.norm(initial[x[0], :3, 3] - initial[x[1], :3, 3]), -(key_times[x[1]] - key_times[x[0]]), x[0], x[1]))
    candidates.extend(loops); loop_degree = np.zeros(len(indices), int)
    for k, (i, j, is_loop) in enumerate(candidates):
        if is_loop and (loop_degree[i] >= rules.loop_cap_per_node or loop_degree[j] >= rules.loop_cap_per_node):
            diagnostics.append({'i': i, 'j': j, 'loop': True, 'accepted': False, 'reason': 'predeclared loop degree cap'}); continue
        target_j, geometry_j = submap(j, indices[i]); target_i, geometry_i = submap(i, indices[j])
        guess = invert(initial[j]) @ initial[i]
        forward, fm = pair_fit(own(i), target_j, geometry_j, guess, rules)
        reverse, rm = pair_fit(own(j), target_i, geometry_i, invert(guess), rules)
        valid, reason = validate_pair(None if forward is None else (forward, fm), None if reverse is None else (reverse, rm), rules, is_loop)
        diagnostics.append({'i': i, 'j': j, 'source_frame_i': int(indices[i]), 'source_frame_j': int(indices[j]),
                            'loop': is_loop, 'accepted': valid, 'reason': reason, 'forward': fm, 'reverse': rm})
        if valid:
            # Measurement maps node i into j; use graph edge from j to i.
            edges.append({'i': j, 'j': i, 'kind': 'loop_geometry' if is_loop else 'adjacent_geometry', 'measurement': forward[0], 'root': forward[1]})
            if is_loop: loop_degree[i] += 1; loop_degree[j] += 1
        if k % 50 == 0:
            print(json.dumps({'processed_pairs': k + 1, 'total_pairs': len(candidates), 'geometry_edges': len(edges) - len(indices) + 1, 'wall_sec': time.monotonic() - start}), flush=True)
    (output / 'edge_diagnostics.json').write_text(json.dumps(diagnostics, indent=2) + '\n')
    (output / 'graph_edges.json').write_text(json.dumps([{**e, 'measurement': e['measurement'].tolist(), 'root': e['root'].tolist()} for e in edges], indent=2) + '\n')
    optimized, fit = optimize_graph(initial, edges, rules)
    corrections = interpolate_corrections(raw, indices, optimized, stamps)
    poses = corrections @ raw
    (output / 'graph_fit.json').write_text(json.dumps(fit, indent=2) + '\n')
    try:
        correction_audit = check_optimized_poses(raw, poses, fit, rules)
    except ValueError as error:
        (output / 'failed.json').write_text(json.dumps({'reason': str(error), 'graph_fit': fit}, indent=2) + '\n')
        raise
    (output / 'optimized_poses.json').write_text(json.dumps([{'index': i, 'stamp': float(t), 'matrix': pose.tolist(), 'correction': co.tolist()} for i, (t, pose, co) in enumerate(zip(stamps, poses, corrections))]) + '\n')
    np.savez(output / 'optimized_keyframes.npz', indices=indices, initial=initial, optimized=optimized)
    prior_manifest = json.loads((raw_prior / 'manifest.json').read_text()); world_camera = np.array(prior_manifest['T_world_camera_init'])
    points = np.empty((0, 3), np.float32); leaf = float(prior_manifest['voxel_size'])
    for begin in range(0, len(raw), 100):
        chunk = np.concatenate([transform(cloud(i), corrections[i]) for i in range(begin, min(begin + 100, len(raw)))]).astype(np.float32)
        points = np.concatenate([points, chunk]); _, keep = np.unique(np.floor(points / leaf).astype(np.int32), axis=0, return_index=True); points = points[np.sort(keep)]
    points = transform(points, world_camera).astype('<f4'); path = output / 'map.pcd'
    header = f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
    with path.open('wb') as f: f.write(header.encode()); f.write(points.tobytes())
    for input_path, expected in source_hashes.items():
        if sha256(input_path) != expected:
            raise ValueError('source input changed during refinement: ' + input_path)
    manifest = {'schema_version': 1, 'frame_id': prior_manifest['frame_id'], 'T_map_world': prior_manifest['T_map_world'],
        'T_world_camera_init': world_camera.tolist(), 'T_base_lidar': prior_manifest['T_base_lidar'], 'voxel_size': leaf,
        'pose_source': 'sensor-only full SE3 keyframe graph with interpolated corrections', 'capture_complete': True,
        'ground_truth_used_for_scan_fusion': False, 'ground_truth_used_for_optimization': False, 'ideal_3d_map_used': False,
        'optimized_graph_used_for_3d_fusion': True, 'slam_toolbox_used': False, 'optimized_2d_graph_used': False,
        'all_frame_leave_one_out_implemented': False, 'all_source_frames_preserved': True, 'fused_scans': len(raw), 'dropped_cloud_indices': [],
        'derived_from': str(raw_prior), 'source_capture': str(source), 'source_hashes': source_hashes,
        'source_map_sha256': sha256(raw_prior / 'map.pcd'), 'map_sha256': sha256(path), 'points': len(points),
        'bounds': [points.min(axis=0).tolist(), points.max(axis=0).tolist()], 'rules_sha256': sha256(output / 'rules.json'),
        'keyframes': len(indices), 'raw_odometry_edges': len(indices) - 1, 'adjacent_geometry_edges': sum(e['kind'] == 'adjacent_geometry' for e in edges),
        'loop_geometry_edges': sum(e['kind'] == 'loop_geometry' for e in edges), 'graph_fit': fit, 'correction_audit': correction_audit, 'prototype_only': True, 'source_inputs_verified_unchanged': True, 'wall_sec': time.monotonic() - start}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (output / 'complete.json').write_text(json.dumps({'map_sha256': manifest['map_sha256'], 'source_frames': len(raw), 'complete': True}) + '\n')
    print(json.dumps(manifest, indent=2), flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path); parser.add_argument('raw_prior', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); refine(args.source, args.raw_prior, args.output, Rules())


if __name__ == '__main__':
    main()
