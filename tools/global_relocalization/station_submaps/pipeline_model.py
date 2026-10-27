"""Parameterized frozen v3 plane model; no dataset-specific IO or station count.

Numerical fitter/evaluator are lifted from the reviewed v2 functions, with
explicit station/rule arguments. Targets use the fixed v3 GoodNormalTree.
"""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from refine_sensor_prior import Rules, invert, transform, information_root
from supported_tree import GoodNormalTree

class NoSupportedNormals:
    """Keep unsupported targets as explicit zero-overlap failures, not aborts."""
    def query(self, points):
        return np.full(len(points), np.inf), np.zeros(len(points), dtype=np.int64)


def supported_geometry(points, normal, good):
    tree = GoodNormalTree(points, good) if np.any(good) else NoSupportedNormals()
    return tree, normal, good


def freeze_domain(source, target, seed):
    """Potential overlap only: fixed seed and target TRAIN normals, no fit."""
    dt, it = target['geom'][0].query(transform(source['train'], seed))
    dh, ih = target['geom'][0].query(transform(source['held'], seed))
    ti, hi = np.flatnonzero(dt <= .3), np.flatnonzero(dh <= .3)
    meta = {
        'train_domain_points': len(ti),
        'train_domain_fraction': len(ti) / max(1, len(source['train'])),
        'heldout_domain_points': len(hi),
        'heldout_domain_fraction': len(hi) / max(1, len(source['held'])),
        'seed_train_unique_target_cells': len(np.unique(it[ti])),
        'seed_heldout_unique_target_cells': len(np.unique(ih[hi])),
        'train_support': compact_stats(transform(source['train'][ti], seed), target['geom'][1][it[ti]]),
        'heldout_support': compact_stats(transform(source['held'][hi], seed), target['geom'][1][ih[hi]]),
        'heldout_frames': [],
        'not_a_visibility_proof': True,
    }
    for ordinal in source['ordinals']:
        chosen = source['hf'][hi] == ordinal
        meta['heldout_frames'].append({'ordinal': int(ordinal), 'domain_points': int(chosen.sum()),
                                      'unique_seed_target_cells': len(np.unique(ih[hi][chosen]))})
    arrays = {'train_indices': ti.astype(np.int32), 'held_indices': hi.astype(np.int32),
              'held_frame': source['hf'][hi], 'held_raw_point': source['rawids'][hi],
              'seed_train_target': it[ti].astype(np.int32),
              'seed_held_target': ih[hi].astype(np.int32)}
    return arrays, meta

def compact_stats(p, n):
    if not len(p):
        return {'points': 0}
    centered = p - p.mean(0)
    eig = np.linalg.eigvalsh(centered.T @ centered / len(p))
    nv = np.linalg.eigvalsh(n.T @ n / max(1, len(n))) if len(n) else np.zeros(3)
    return {'points': len(p), 'bbox_extent_m': np.ptp(p, axis=0).tolist(), 'spatial_eigenvalues_m2': eig.tolist(), 'normal_second_moment_eigenvalues': nv.tolist()}

def evaluate(stations, src, tgt, domain, z):
    s, t = (stations[src], stations[tgt])
    q = transform(s['held'][domain['hi']], z)
    d, ids = t['geom'][0].query(q)
    valid = (d <= 0.3) & t['geom'][2][ids]
    plane = np.full(len(q), 0.3)
    distance = np.full(len(q), 0.3)
    plane[valid] = np.minimum(abs(np.einsum('ij,ij->i', q[valid] - t['train'][ids[valid]], t['geom'][1][ids[valid]])), 0.3)
    distance[valid] = np.minimum(d[valid], 0.3)
    frames = []
    for f in s['ordinals']:
        mask = s['hf'][domain['hi']] == f
        count = int(mask.sum())
        frames.append({'ordinal': f, 'fixed_denominator': count, 'successful': int(valid[mask].sum()), 'unique_valid_target_cells': len(np.unique(ids[mask & valid])), 'coverage': float(valid[mask].mean()) if count else 0.0, 'plane_mse': float(np.mean(plane[mask] ** 2)) if count else 0.09, 'capped_nn_mse': float(np.mean(distance[mask] ** 2)) if count else 0.09})
    return {'fixed_domain_points': len(q), 'frame_equal_coverage': float(np.mean([f['coverage'] for f in frames])), 'post_support_lost_points': int((~valid).sum()), 'frame_equal_plane_rmse_m': float(np.sqrt(np.mean([f['plane_mse'] for f in frames]))), 'frame_equal_capped_nn_rmse_m': float(np.sqrt(np.mean([f['capped_nn_mse'] for f in frames]))), 'pooled_plane_rmse_m': float(np.sqrt(np.mean(plane ** 2))) if len(q) else 0.3, 'pooled_capped_nn_rmse_m': float(np.sqrt(np.mean(distance ** 2))) if len(q) else 0.3, 'every_frame_min15': all((f['fixed_denominator'] >= 15 and f['unique_valid_target_cells'] >= 15 for f in frames)), 'frames': frames}

def fit(stations, rules, src, tgt, D, initial_z):
    s, t = (stations[src], stations[tgt])
    source = s['train'][D['ti']]
    tree, normal, good = t['geom']
    z = initial_z.copy()
    converged = False
    reject = []
    iterations = 0
    for iteration in range(rules.max_iterations):
        p = transform(source, z)
        d, ids = tree.query(p)
        valid = (d <= 0.3) & good[ids]
        pp, qq, n = (p[valid], t['train'][ids[valid]], normal[ids[valid]])
        if len(pp) < rules.min_plane_matches:
            reject.append('insufficient_train_plane_matches')
            break
        residual = np.einsum('ij,ij->i', pp - qq, n)
        J = np.c_[np.cross(pp, n), n]
        weight = np.minimum(1.0, 0.03 / np.maximum(abs(residual), 1e-12))
        h = J.T @ (J * weight[:, None])
        b = J.T @ (weight * residual)
        v, u = np.linalg.eigh(h)
        keep = v > max(1e-12, v[-1] * rules.information_relative_floor)
        delta = -u[:, keep] @ (u[:, keep].T @ b / v[keep])
        iterations = iteration + 1
        if np.linalg.norm(delta[:3]) > 0.1 or np.linalg.norm(delta[3:]) > 0.1:
            reject.append('unbounded_train_step')
            break
        update = np.eye(4)
        update[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix()
        update[:3, 3] = delta[3:]
        z = update @ z
        if np.linalg.norm(delta) < 1e-05:
            converged = True
            break
    p = transform(source, z)
    d, ids = tree.query(p)
    valid = (d <= 0.3) & good[ids]
    pp, n = (p[valid], normal[ids[valid]])
    root = np.zeros((6, 6))
    rank = 0
    eigen = np.zeros(6)
    if len(pp):
        J = np.c_[np.cross(pp, n), n]
        h = J.T @ J / max(1, len(pp)) * min(100, len(pp)) / 0.03 ** 2
        adj = np.zeros((6, 6))
        adj[:3, :3] = z[:3, :3]
        adj[3:, 3:] = z[:3, :3]
        v = z[:3, 3]
        skew = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
        adj[3:, :3] = skew @ z[:3, :3]
        root, rank, eigen = information_root(adj.T @ h @ adj, rules.information_relative_floor)
    before = evaluate(stations, src, tgt, D, initial_z)
    after = evaluate(stations, src, tgt, D, z)
    update = z @ invert(initial_z)
    origin_update = float(np.linalg.norm(z[:3, 3] - initial_z[:3, 3]))
    rotation_update = float(Rotation.from_matrix(update[:3, :3]).magnitude() * 180 / np.pi)
    coverage = float(valid.mean()) if len(valid) else 0.0
    if not converged:
        reject.append('train_not_converged')
    if D['meta']['seed_train_unique_target_cells'] < 50 or len(np.unique(ids[valid])) < 50:
        reject.append('train_unique_cells<50')
    if coverage < 0.65:
        reject.append('train_domain_coverage<.65')
    if rank < 5:
        reject.append('information_rank<5')
    if origin_update > 0.3 or rotation_update > 10:
        reject.append('relative_update_bound')
    if not before['every_frame_min15'] or not after['every_frame_min15']:
        reject.append('scan_holdout_unique_cells<15_in_some_frame')
    if after['frame_equal_coverage'] < 0.65:
        reject.append('scan_holdout_domain_coverage<.65')
    if after['frame_equal_plane_rmse_m'] > before['frame_equal_plane_rmse_m'] + 0.002:
        reject.append('scan_holdout_fixed_plane_loss_degraded')
    if after['frame_equal_capped_nn_rmse_m'] > before['frame_equal_capped_nn_rmse_m'] + 0.002:
        reject.append('scan_holdout_fixed_NN_loss_degraded')
    full_seed = stations[tgt]['full']
    ftree = cKDTree(full_seed)
    orig_before = ftree.query(transform(s['full'], initial_z))[0]
    orig_after = ftree.query(transform(s['full'], z))[0]
    metrics = {'accepted': not reject, 'rejected_gates': sorted(set(reject)), 'converged': converged, 'iterations': iterations, 'train_domain_coverage': coverage, 'train_normal_matches': int(valid.sum()), 'train_unique_target_cells': len(np.unique(ids[valid])), 'information_rank': rank, 'information_eigenvalues': eigen.tolist(), 'origin_update_m': origin_update, 'rotation_update_deg': rotation_update, 'scan_heldout_before': before, 'scan_heldout_after': after, 'original_full_cloud_coverage_at_seed': float(np.mean(orig_before <= 0.3)), 'original_full_cloud_coverage_after': float(np.mean(orig_after <= 0.3))}
    return (z, root, metrics)
