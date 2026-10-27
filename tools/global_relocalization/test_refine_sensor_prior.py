import json
from pathlib import Path
from dataclasses import replace

import pytest
import numpy as np
from scipy.spatial.transform import Rotation

from refine_sensor_prior import (Rules, information_root, interpolate_corrections,
    invert, optimize_graph, pair_fit, normals, relative_error, refine, transform, validate_pair, sha256, verify_source, check_optimized_poses)


def pose(translation, rotation=(0, 0, 0)):
    t = np.eye(4); t[:3, :3] = Rotation.from_rotvec(rotation).as_matrix(); t[:3, 3] = translation
    return t


def test_relative_pose_direction_and_fixed_gauge():
    a = pose([2, -3, 1], [.1, -.2, .3]); b = pose([-1, 5, .2], [-.1, .3, -.4])
    z = invert(a) @ b
    assert np.linalg.norm(relative_error(a, b, z)) < 1e-12
    assert np.linalg.norm(relative_error(a, b, invert(z))) > 1
    initial = np.array([a, pose(b[:3, 3] + [.1, 0, 0], [-.1, .3, -.4])])
    edges = [{'i': 0, 'j': 1, 'measurement': z, 'root': np.eye(6) * 100}]
    optimized, metrics = optimize_graph(initial, edges, Rules())
    assert np.array_equal(optimized[0], a)
    assert np.linalg.norm(optimized[1] - b) < 1e-6
    gauge = pose([20, 5, -3], [.4, .1, -.5])
    changed, _ = optimize_graph(gauge @ initial, edges, Rules())
    assert np.linalg.norm(changed - gauge @ optimized) < 1e-5


def test_identity_correction_and_all_frame_interpolation():
    raw = np.array([pose([x, .2*x, 0], [0, 0, .2*x]) for x in [0, .3, .6]])
    corrections = interpolate_corrections(raw, np.array([0, 2]), raw[[0, 2]], np.array([0., 1., 2.]))
    points = np.array([[2., 1., .4]])
    assert np.allclose(corrections, np.eye(4), atol=1e-12)
    # Registered points are already in the common frame and must not receive raw pose twice.
    assert np.allclose(transform(points, corrections[1]), points)
    assert not np.allclose(transform(points, raw[1]), points)


def test_plane_information_does_not_invent_horizontal_or_yaw_certainty():
    x, y = np.meshgrid(np.linspace(-1, 1, 10), np.linspace(-1, 1, 10))
    p = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]; n = np.tile([0, 0, 1.], (len(p), 1))
    jacobian = np.c_[np.cross(p, n), n]
    root, rank, _ = information_root(jacobian.T @ jacobian, 1e-5)
    assert rank == 3
    for axis in [2, 3, 4]:
        assert np.linalg.norm(root[:, axis]) < 1e-12
    assert np.linalg.norm(root[:, 5]) > 1


def test_bad_independent_loop_is_rejected():
    rules = Rules(); a = (np.eye(4), np.eye(6)); b = (pose([.2, 0, 0]), np.eye(6))
    assert validate_pair((a, {'rank': 6}), (b, {'rank': 6}), rules, True) == (False, 'independent reverse cycle failed')
    assert not validate_pair((a, {'rank': 3}), (a, {'rank': 3}), rules, True)[0]
    assert validate_pair((a, {'rank': 3}), (a, {'rank': 3}), rules, False)[0]


def test_room_scan_recovers_injected_full_se3():
    rng = np.random.default_rng(12); count = 1500
    p = rng.uniform(-1, 1, (count, 3)); groups = np.arange(count) % 3
    for i in range(3): p[groups == i, i] = 1.
    truth = pose([.025, -.018, .022], [.012, -.015, .01]); source = transform(p, invert(truth))
    rules = replace(Rules(), min_plane_matches=40)
    fit, metrics = pair_fit(source, p, normals(p, rules), np.eye(4), rules)
    assert fit is not None, metrics
    assert np.linalg.norm(relative_error(np.eye(4), fit[0], truth)) < 1e-3
    assert metrics['rank'] == 6


def test_export_preserves_frames_gauge_and_does_not_read_gt(tmp_path):
    source, prior, output = [tmp_path / x for x in ['source', 'prior', 'derived']]
    source.mkdir(); prior.mkdir(); (source / 'registered_scans').mkdir()
    points = np.array([[.6, .1, .3], [.8, -.2, .2], [1.2, .4, .5]], dtype=np.float32)
    raw = np.array([pose([0, 0, 0]), pose([.4, .1, .05], [.1, .2, .1])])
    for i in range(2): np.save(source / 'registered_scans' / f'{i:06d}.npy', points)
    capture = {'failure': None, 'completed_waypoints': 2, 'total_waypoints': 2,
        'odom': [{'stamp': float(i), 'matrix': t.tolist()} for i, t in enumerate(raw)],
        'clouds': [{'index': i, 'stamp': float(i)} for i in range(2)], 'gt': 'INVALID AND MUST NOT BE ACCESSED'}
    (source / 'capture.json').write_text(json.dumps(capture)); (source / 'mapping_params.yaml').write_text('{}')
    world = pose([9, -2, .4], [.1, .2, -.3])
    (source / 'manifest.json').write_text(json.dumps({'capture_complete': True}))
    (source / 'mapping_binary_provenance.json').write_text('{}')
    (prior / 'registered_scans').symlink_to(source / 'registered_scans', target_is_directory=True)
    for name in ['capture.json', 'mapping_params.yaml', 'mapping_binary_provenance.json']:
        (prior / name).write_bytes((source / name).read_bytes())
    manifest = {'frame_id': 'map', 'T_map_world': np.eye(4).tolist(), 'T_world_camera_init': world.tolist(), 'T_base_lidar': np.eye(4).tolist(), 'voxel_size': .03}
    (prior / 'map.pcd').write_text('source map remains unchanged')
    manifest.update({'capture_complete': True, 'ground_truth_used_for_scan_fusion': False, 'ideal_3d_map_used': False,
        'optimized_graph_used_for_3d_fusion': False, 'all_source_registered_scans_used': True, 'dropped_source_scan_indices': [],
        'source_capture_directory': str(source), 'map_sha256': sha256(prior / 'map.pcd'), 'fused_scans': 2,
        'source_sha256': {name: sha256(source / name) for name in ['capture.json', 'mapping_params.yaml', 'manifest.json', 'mapping_binary_provenance.json']}})
    (prior / 'manifest.json').write_text(json.dumps(manifest))
    (prior / 'raw_prior_export_complete.json').write_text(json.dumps({'completed': True, 'map_sha256': manifest['map_sha256'], 'manifest_sha256': sha256(prior / 'manifest.json')}))
    frozen = verify_source(source.resolve(), prior.resolve())
    assert str((source / 'registered_scans/000000.npy').resolve()) in frozen
    result = refine(source, prior, output, Rules())
    assert result['fused_scans'] == 2 and result['dropped_cloud_indices'] == []
    assert not result['ground_truth_used_for_optimization'] and not result['all_frame_leave_one_out_implemented']
    with (output / 'map.pcd').open('rb') as f:
        while not f.readline().startswith(b'DATA binary'): pass
        actual = np.frombuffer(f.read(), dtype='<f4').reshape(-1, 3)
    assert np.allclose(actual, transform(points, world), atol=1e-6)
    assert (prior / 'map.pcd').read_text() == 'source map remains unchanged'
    assert np.allclose(np.array(json.loads((output / 'optimized_poses.json').read_text())[0]['matrix']), raw[0])


def test_solver_and_cumulative_correction_failure_are_explicit():
    raw=np.array([np.eye(4),np.eye(4)])
    with pytest.raises(ValueError, match='did not converge'):
        check_optimized_poses(raw,raw,{'success':False,'message':'budget exceeded'},Rules())
    wrong=raw.copy();wrong[1,0,3]=1.
    with pytest.raises(ValueError, match='cumulative correction'):
        check_optimized_poses(raw,wrong,{'success':True},Rules())
    wrong=raw.copy();wrong[1,0,0]=np.nan
    with pytest.raises(ValueError, match='nonfinite'):
        check_optimized_poses(raw,wrong,{'success':True},Rules())
