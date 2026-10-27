import hashlib
import json

import numpy as np
import pytest

from export_raw_prior import export_raw_prior, sha256
from full_slam_mapping import se2
from sensor_mapping import base_sensor


def source_capture(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'registered_scans').mkdir()
    binary = tmp_path / 'archive/pointlio_mapping'
    binary.parent.mkdir()
    binary.write_bytes(b'archived mapper fixture; never executed')
    provenance = {'processes': [{'roles': ['point_lio'], 'executable': {
        'path': str(binary), 'sha256': sha256(binary)}}]}
    (source / 'mapping_binary_provenance.json').write_text(json.dumps(provenance))
    (source / 'manifest.json').write_text(json.dumps({'capture_complete': True, 'voxel_size': .001,
        'T_base_lidar': base_sensor().tolist(), 'slam_toolbox_used': True, 'pose_source': 'full graph'}))
    (source / 'mapping_params.yaml').write_text('point_lio:\n  ros__parameters: {}\n')
    rows, clouds, truth = [], [], []
    for index in range(4):
        np.save(source / 'registered_scans' / f'{index:06d}.npy', [[index*.1+1, 0., 0.]])
        clouds.append({'index': index, 'stamp': float(index), 'frame': 'camera_init'})
        rows.append({'stamp': float(index), 'matrix': (se2([index*.1, 0, 0]) @ base_sensor()).tolist()})
        truth.append({'stamp': float(index), 'matrix': se2([10+index*.1, 20, 0]).tolist()})
    capture = {'failure': None, 'completed_waypoints': 2, 'total_waypoints': 2,
        'control_source': 'simulator', 'clouds': clouds, 'odom': rows, 'gt': truth, 'commands': []}
    (source / 'capture.json').write_text(json.dumps(capture))
    return source, capture, binary


def test_all_scans_fused_with_one_gt_anchor_without_changing_source(tmp_path):
    source, capture, _ = source_capture(tmp_path)
    hashes = {str(path): sha256(path) for path in source.rglob('*') if path.is_file()}
    output = tmp_path / 'raw'
    manifest = export_raw_prior(source, output)
    assert manifest['fused_scans'] == 4
    assert manifest['all_source_registered_scans_used'] is True
    assert manifest['optimized_graph_used_for_3d_fusion'] is False
    assert manifest['ground_truth_used_for_control'] is True
    assert manifest['source_completed_waypoints'] == manifest['source_total_waypoints'] == 2
    assert manifest['accuracy']['strict_0_10m_2deg']['passed'] == 4
    assert (output / 'registered_scans').is_symlink()
    assert (output / 'raw_prior_export_complete.json').is_file()
    assert (output / 'source_manifest.json').is_file()
    assert hashes == {str(path): sha256(path) for path in source.rglob('*') if path.is_file()}
    payload = (output / 'map.pcd').read_bytes().split(b'DATA binary\n')[1]
    points = np.frombuffer(payload, dtype='<f4').reshape(-1, 3)
    np.testing.assert_allclose(points[:, 0], [11, 11.1, 11.2, 11.3], atol=1e-6)
    for row in capture['gt'][1:]:
        row['matrix'][0][3] += 100
    (source / 'capture.json').write_text(json.dumps(capture))
    second = tmp_path / 'raw_different_later_truth'
    later = export_raw_prior(source, second)
    assert (second / 'map.pcd').read_bytes() == (output / 'map.pcd').read_bytes()
    assert later['accuracy']['strict_0_10m_2deg']['failed'] == 3
    with pytest.raises(FileExistsError):
        export_raw_prior(source, output)


def test_incomplete_capture_and_changed_mapper_are_rejected(tmp_path):
    source, capture, binary = source_capture(tmp_path)
    capture['completed_waypoints'] = 1
    (source / 'capture.json').write_text(json.dumps(capture))
    with pytest.raises(ValueError, match='completed full survey'):
        export_raw_prior(source, tmp_path / 'incomplete')
    capture['completed_waypoints'] = 2
    (source / 'capture.json').write_text(json.dumps(capture))
    binary.write_bytes(b'different mapper')
    with pytest.raises(ValueError, match='archived exact binary'):
        export_raw_prior(source, tmp_path / 'wrong_binary')
    with pytest.raises(ValueError, match='outside the source'):
        export_raw_prior(source, source / 'nested_output')
