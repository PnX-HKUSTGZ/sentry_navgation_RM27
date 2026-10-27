#!/usr/bin/env python3
"""Export every registered Point-LIO scan from a completed full-SLAM capture.

Writes a fresh derived directory. The estimator is never run and no graph poses
or ideal map are read. Ground truth supplies one initial SE(3) gauge only and
independent trajectory scores, using the existing sensor-only exporter.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np

from sensor_mapping import base_sensor, export


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def recorded_mapper(source, override=None):
    provenance = json.loads((source / 'mapping_binary_provenance.json').read_text())
    candidates = []
    for process in provenance.get('processes', []):
        executable = process.get('executable', process.get('exe'))
        path = executable.get('path') if isinstance(executable, dict) else executable
        digest = executable.get('sha256') if isinstance(executable, dict) else process.get('sha256')
        if path and (Path(path).name == 'pointlio_mapping' or 'point_lio' in process.get('roles', [])):
            candidates.append((Path(path), digest))
    if len(candidates) != 1 or not candidates[0][1]:
        raise ValueError('source must record exactly one Point-LIO executable hash')
    recorded_path, expected = candidates[0]
    actual_path = Path(override).resolve() if override else recorded_path
    if not actual_path.is_file() or sha256(actual_path) != expected:
        raise ValueError('recorded mapper no longer matches; pass --mapper with its archived exact binary')
    return actual_path, expected


def export_raw_prior(source, output, voxel=None, mapper=None, link_mode='symlink'):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output == source or output.is_relative_to(source):
        raise ValueError('derived output must be outside the source capture directory')
    if output.exists() or output.is_symlink():
        raise FileExistsError('raw prior output already exists: ' + str(output))
    required = ['capture.json', 'mapping_params.yaml', 'manifest.json', 'mapping_binary_provenance.json']
    source_hashes = {name: sha256(source / name) for name in required}
    capture = json.loads((source / 'capture.json').read_text())
    source_manifest = json.loads((source / 'manifest.json').read_text())
    if (capture.get('failure') is not None or not capture.get('total_waypoints') or
            capture.get('completed_waypoints') != capture['total_waypoints'] or
            source_manifest.get('capture_complete') is not True):
        raise ValueError('raw prior requires a completed full survey; partial captures remain rejected')
    clouds = capture['clouds']
    if not clouds or [row['index'] for row in clouds] != list(range(len(clouds))):
        raise ValueError('all registered scans must have contiguous original indices')
    if any(row.get('frame') != 'camera_init' for row in clouds):
        raise ValueError('raw exporter requires registered scans in camera_init')
    for row in clouds:
        if not (source / 'registered_scans' / f"{row['index']:06d}.npy").is_file():
            raise FileNotFoundError('source registered scan missing: ' + str(row['index']))
    if 'T_base_lidar' not in source_manifest or not np.allclose(source_manifest['T_base_lidar'], base_sensor(), atol=1e-10):
        raise ValueError('source lidar mount differs from the sensor exporter calibration')
    voxel = float(source_manifest['voxel_size'] if voxel is None else voxel)
    if not np.isfinite(voxel) or voxel <= 0 or link_mode not in ('symlink', 'hardlink'):
        raise ValueError('voxel must be positive and scan link mode must be symlink or hardlink')
    executable, mapper_hash = recorded_mapper(source, mapper)
    output.mkdir(parents=True, exist_ok=False)
    for name in required:
        destination = 'source_manifest.json' if name == 'manifest.json' else name
        shutil.copyfile(source / name, output / destination)
    for name in ['route.json', 'station_scans.json', 'mapping_summary.json']:
        if (source / name).is_file():
            shutil.copyfile(source / name, output / ('source_' + name))
    if link_mode == 'symlink':
        (output / 'registered_scans').symlink_to(source / 'registered_scans', target_is_directory=True)
    else:
        (output / 'registered_scans').mkdir()
        for row in clouds:
            name = f"{row['index']:06d}.npy"
            os.link(source / 'registered_scans' / name, output / 'registered_scans' / name)
    method = 'raw Point-LIO registered scans; no 2D graph correction'
    args = SimpleNamespace(voxel=voxel, method=method, mapper=executable,
        control_source=capture.get('control_source', 'estimated'),
        route_map='source route occupancy used only by the independent survey controller',
        source_revision=source_manifest.get('mapping_source_revision', 'see recorded source binary provenance'),
        speed=source_manifest.get('survey_speed_mps'), rotation_speed=source_manifest.get('survey_rotation_rad_sec'))
    node = SimpleNamespace(out=output, clouds=clouds,
        odom=[(row['stamp'], np.array(row['matrix'])) for row in capture['odom']],
        gt=[(row['stamp'], np.array(row['matrix'])) for row in capture['gt']])
    export(node, output, output / 'mapping_params.yaml', args)
    # Verify frozen metadata remained unchanged during the read-only export.
    if any(sha256(source / name) != digest for name, digest in source_hashes.items()):
        raise RuntimeError('source metadata changed during export; derived prior must not be accepted')
    errors = json.loads((output / 'mapping_errors.json').read_text())
    accuracy = {'strict_0_10m_2deg': {'translation_limit_m': .10, 'orientation_limit_deg': 2.,
                    'count': len(errors), 'passed': sum(row['strict_correct'] for row in errors)},
                'sensor_map_0_15m_3deg': {'translation_limit_m': .15, 'orientation_limit_deg': 3.,
                    'count': len(errors), 'passed': sum(row['sensor_map_correct'] for row in errors)}}
    for value in accuracy.values():
        value['failed'] = value['count'] - value['passed']
    manifest_path = output / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest.update(schema_version=3, source_capture_directory=str(source), source_sha256=source_hashes,
        source_method=source_manifest.get('pose_source'), source_completed_waypoints=capture['completed_waypoints'],
        source_total_waypoints=capture['total_waypoints'], source_failure=capture['failure'],
        source_slam_toolbox_ran=source_manifest.get('slam_toolbox_used', False),
        optimized_graph_used_for_3d_fusion=False, two_dimensional_graph_used=False,
        all_source_registered_scans_used=True, dropped_source_scan_indices=[],
        registered_scan_link_mode=link_mode, source_registered_scans_read_only=True,
        ground_truth_used_for_control=capture.get('control_source') in ('simulator', 'ground_truth'),
        ground_truth_used_for_scan_fusion=False, ideal_3d_map_used=False,
        mapper_sha256=mapper_hash, accuracy=accuracy,
        exporter_sha256=sha256(Path(__file__)), source_exporter_sha256=sha256(Path(__file__).with_name('sensor_mapping.py')))
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    summary_path = output / 'mapping_summary.json'
    summary = json.loads(summary_path.read_text())
    summary.update(method=method, source_capture_directory=str(source), accuracy=accuracy,
                   optimized_graph_used_for_3d_fusion=False, all_source_registered_scans_used=True)
    summary_path.write_text(json.dumps(summary, indent=2)+'\n')
    (output / 'raw_prior_export_complete.json').write_text(json.dumps({
        'completed': True, 'map_sha256': manifest['map_sha256'],
        'manifest_sha256': sha256(manifest_path), 'source_capture_directory': str(source)}, indent=2)+'\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--voxel', type=float)
    parser.add_argument('--mapper', type=Path, help='archived source binary if its recorded path has since changed')
    parser.add_argument('--scan-link-mode', choices=('symlink', 'hardlink'), default='symlink')
    args = parser.parse_args()
    manifest = export_raw_prior(args.source, args.output, args.voxel, args.mapper, args.scan_link_mode)
    print(json.dumps({'output': str(args.output.resolve()), 'fused_scans': manifest['fused_scans'],
                      'points': manifest['points'], 'map_sha256': manifest['map_sha256'],
                      'accuracy': manifest['accuracy']}))


if __name__ == '__main__':
    main()
