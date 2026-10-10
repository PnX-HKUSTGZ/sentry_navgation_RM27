#!/usr/bin/env python3
"""Headless Patchwork++/HDBSCAN preprocessing; never writes original scans."""
import argparse
import hashlib
import json
from pathlib import Path

import hdbscan
import numpy as np
import pypatchworkpp
import yaml


def preprocess(sequence, config, start, end):
    options = config.get('preprocessing', {})
    height = float(options.get('sensor_height', config['extrinsic']['sensor_height']))
    min_cluster = int(options.get('min_cluster_size', 15))
    min_samples = int(options.get('min_samples', 5))
    if not np.isfinite(height) or height <= 0 or min_cluster < 2 or min_samples < 1:
        raise ValueError('Invalid measured sensor_height / HDBSCAN parameters')
    params = pypatchworkpp.Parameters()
    params.verbose = False
    params.sensor_height = height
    estimator = pypatchworkpp.patchworkpp(params)
    for directory in ('patchwork', 'hdbscan'):
        (sequence / directory).mkdir(exist_ok=False)
    records = []
    for frame in range(start, end + 1):
        scan = np.fromfile(sequence / 'velodyne' / f'{frame:06d}.bin', dtype='<f4').reshape(-1, 4)
        if not len(scan) or not np.isfinite(scan).all():
            raise ValueError(f'Invalid points in frame {frame}')
        estimator.estimateGround(scan)
        ground_indices = np.asarray(estimator.getGroundIndices(), dtype=np.int64)
        if len(ground_indices) and (ground_indices.min() < 0 or ground_indices.max() >= len(scan)):
            raise ValueError('Patchwork++ returned out-of-bounds indices')
        ground = np.zeros(len(scan), dtype='<u4')
        ground[ground_indices] = 1
        nonground = ground == 0
        xyz = scan[nonground, :3].astype(np.float64)
        cluster_labels = np.full(len(xyz), -1, dtype=np.int64)
        # Sparse or all-ground frames are valid; HDBSCAN cannot fit an empty/tiny input.
        if len(xyz) >= max(min_cluster, min_samples + 1):
            cluster_labels = hdbscan.HDBSCAN(min_cluster_size=min_cluster, min_samples=min_samples,
                                             core_dist_n_jobs=1).fit_predict(xyz)
        instances = np.zeros(len(scan), dtype='<u4')
        if len(cluster_labels) and cluster_labels.max() >= 65535:
            raise ValueError('Too many clusters for SemanticKITTI uint16 instance encoding')
        instances[nonground] = ((cluster_labels + 1).astype('<u4') << 16)
        ground_file = sequence / 'patchwork' / f'{frame:06d}.label'
        instance_file = sequence / 'hdbscan' / f'{frame:06d}.label'
        ground.tofile(ground_file)
        instances.tofile(instance_file)
        record = {'frame': frame, 'points': len(scan), 'ground_points': int((ground != 0).sum()),
                  'nonground_noise': int((cluster_labels < 0).sum()),
                  'cluster_count': int(cluster_labels.max() + 1) if len(cluster_labels) else 0,
                  'ground_sha256': hashlib.sha256(ground_file.read_bytes()).hexdigest(),
                  'instances_sha256': hashlib.sha256(instance_file.read_bytes()).hexdigest()}
        records.append(record)
        print(f"Preprocessed {frame:06d}: {record['ground_points']} ground, {record['cluster_count']} clusters", flush=True)
    report = {'method': 'Patchwork++ + HDBSCAN', 'sensor_height': height,
              'min_cluster_size': min_cluster, 'min_samples': min_samples,
              'ground_encoding': 'uint32 little endian, ground=1',
              'instance_encoding': 'uint32 little endian, positive instance ID in upper 16 bits; noise/ground=0',
              'frames': records}
    (sequence.parent.parent / 'preprocessing.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sequence', type=Path)
    p.add_argument('config', type=Path)
    p.add_argument('start', type=int)
    p.add_argument('end', type=int)
    a = p.parse_args()
    preprocess(a.sequence, yaml.safe_load(a.config.read_text()), a.start, a.end)
