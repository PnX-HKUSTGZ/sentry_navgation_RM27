#!/usr/bin/env python3
"""Preserve upstream's two-argument sequence interface in a fresh isolated run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pointlio_export'))
from atomic_directory import commit_directory
from preprocess import preprocess


def run(args):
    sequence = args.sequence.expanduser().resolve(strict=True)
    parameter = args.parameter.expanduser().resolve(strict=True)
    candidate = Path(os.environ.get('ERASOR2_OUTPUT_DIR', str(sequence.parent / (sequence.name + '_erasor2_output')))).expanduser().absolute()
    output = candidate.parent.resolve() / candidate.name
    if os.path.lexists(output):
        raise ValueError('Output exists; set ERASOR2_OUTPUT_DIR to a new run directory')
    if sequence == output or sequence in output.parents:
        raise ValueError('Output must be outside the original sequence directory')
    config = yaml.safe_load(parameter.read_text())
    if not isinstance(config, dict):
        raise ValueError('Config must be a mapping')
    dl = config.setdefault('dataloader', {})
    if dl.get('dataset_name', 'SemanticKITTI') != 'SemanticKITTI':
        raise ValueError('Independent custom-sequence wrapper requires SemanticKITTI direct pose convention')
    if dl.get('instance_seg_method', 'hdbscan') != 'hdbscan':
        raise ValueError('Independent preprocessing supports HDBSCAN')
    if dl.get('run_traj_clustering', False):
        raise ValueError('Independent custom-sequence wrapper requires run_traj_clustering=false')
    extrinsic = config.get('extrinsic', {})
    rotation = np.array(extrinsic.get('rotation', np.eye(3).ravel()), dtype=float).reshape(3, 3)
    translation = np.array(extrinsic.get('translation', [0, 0, 0]), dtype=float)
    if not np.allclose(rotation, np.eye(3)) or not np.allclose(translation, 0):
        raise ValueError('ERASOR2 extrinsic must be identity: exported poses are already direct T_world_scan')
    scans = sorted((sequence / 'velodyne').glob('*.bin'))
    if not scans or [s.name for s in scans] != [f'{i:06d}.bin' for i in range(len(scans))]:
        raise ValueError('Scans must be contiguous 000000.bin onward')
    pose_file = sequence / 'poses_suma_optim.txt'
    poses = np.loadtxt(pose_file, ndmin=2)
    if poses.shape != (len(scans), 12) or not np.isfinite(poses).all():
        raise ValueError('One finite 3x4 row-major direct pose is required per scan')
    for pose in poses:
        r = pose.reshape(3, 4)[:, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(r), 1, atol=1e-4):
            raise ValueError('Input pose rotation must be proper orthonormal')
    interval = int(dl.get('accum_interval', 1))
    start = int(config.get('start_frame', 0))
    requested_end = int(config.get('end_frame', -1))
    if interval < 1 or start < 0:
        raise ValueError('Invalid accum_interval/start_frame')
    safe_end = len(scans) - interval
    end = safe_end if requested_end < 0 else min(requested_end, safe_end)
    end = start + ((end - start) // interval) * interval
    if end < start:
        raise ValueError('Insufficient sequence length for requested frame range/interval')
    # Validate all scans before expensive preprocessing/building. Source sequence stays read-only.
    scan_hashes = []
    for scan in scans:
        if scan.stat().st_size == 0 or scan.stat().st_size % 16:
            raise ValueError(f'Invalid float32 xyzi binary size: {scan}')
        points = np.fromfile(scan, dtype='<f4')
        if not np.isfinite(points).all():
            raise ValueError(f'Nonfinite input scan: {scan}')
        scan_hashes.append(hashlib.sha256(scan.read_bytes()).hexdigest())
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.' + output.name + '.', dir=output.parent))
    build = Path(os.environ.get('ERASOR2_BUILD_DIR', str(ROOT / '.build/erasor2'))).expanduser().resolve()
    try:
        work = stage / 'input' / 'mapping'
        work.mkdir(parents=True)
        # The original scan folder is linked for read-only C++ access. All generated labels
        # and configs are created under stage. Upstream writes no files into velodyne/.
        (work / 'velodyne').symlink_to(sequence / 'velodyne', target_is_directory=True)
        shutil.copy2(pose_file, work / 'poses_suma_optim.txt')
        for name in ('times.txt', 'manifest.json'):
            if (sequence / name).is_file():
                shutil.copy2(sequence / name, work / name)
        results = stage / 'results'
        results.mkdir()
        # Upstream mapgen may read trailing frames through end+interval-1.
        # Dummy GT labels are only plumbing for custom scans; never an accuracy reference.
        (work / 'labels').mkdir()
        for frame in range(start, end + interval):
            with (work / 'labels' / f'{frame:06d}.label').open('wb') as stream:
                stream.truncate(scans[frame].stat().st_size // 4)
        dl.update(dataset_name='SemanticKITTI', abs_data_dir=str(work.parent), sequence='mapping',
                  abs_save_dir=str(results), instance_seg_method='hdbscan', accum_interval=interval)
        config.update(start_frame=start, end_frame=end, stop_for_each_frame=False)
        config['rerun'] = {'enabled': False, 'spawn': False}
        effective = stage / 'requested_config.yaml'
        effective.write_text(yaml.safe_dump(config, sort_keys=False))
        preprocess(work, config, start, end)
        if not (build / 'mapgen').is_file() or not (build / 'run_erasor2').is_file():
            build_env=os.environ.copy()
            build_env['ERASOR2_BUILD_DIR']=str(build)
            subprocess.run([str(ROOT / 'scripts/build_erasor2.sh')], env=build_env, check=True)
        env = os.environ.copy()
        env.update(ERASOR2_OUTPUT_DIR=str(results), ERASOR2_BUILD_DIR=str(build),
                   ERASOR2_PYTHON=sys.executable, ERASOR2_JOBS=env.get('ERASOR2_JOBS', '2'))
        with (stage / 'run.log').open('w') as log:
            subprocess.run(['bash', str(ROOT / 'erasor2/vendor/scripts/run_sequence.sh'), str(work), str(effective)],
                           env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        static_maps = sorted(results.glob('*_estimated.pcd'))
        if not static_maps:
            raise ValueError('ERASOR2 produced no estimated static PCD')
        mos_records=[]
        radius=float(extrinsic.get('robot_body_size', 0.25))
        radius_squared=float(np.float32(float(np.float32(radius))**2))
        for frame in range(start, end + 1, interval):
            points=np.fromfile(scans[frame],dtype='<f4').reshape(-1,4)
            kept=int((points[:,0].astype(float)**2 + points[:,1].astype(float)**2 >= radius_squared).sum())
            label=results / 'mos' / f'{frame:06d}.label'
            actual_bytes=label.stat().st_size
            if actual_bytes != kept*4:
                raise ValueError(f'Upstream MOS label count differs from filtered scan: frame {frame}')
            mos_records.append({'frame':frame,'raw_points':len(points),'filtered_scan_points':kept,
                                'dropped_body_radius_points':len(points)-kept,
                                'mos_label_count':actual_bytes//4,'mos_bytes':actual_bytes})
        # Store usable configs after the staging directory is atomically renamed.
        for cfg in (effective, results / 'effective_config.yaml'):
            final_config = yaml.safe_load(cfg.read_text())
            final_config['dataloader']['abs_data_dir'] = str(output / 'input')
            final_config['dataloader']['abs_save_dir'] = str(output / 'results')
            cfg.write_text(yaml.safe_dump(final_config, sort_keys=False))
        (results / '.sequence_run_metadata').unlink(missing_ok=True)
        manifest = {'schema': 1, 'upstream': json.loads((ROOT/'erasor2/UPSTREAM.json').read_text()),
                    'source_sequence': str(sequence), 'source_parameter': str(parameter),
                    'parameter_sha256': hashlib.sha256(parameter.read_bytes()).hexdigest(),
                    'pose_sha256': hashlib.sha256(pose_file.read_bytes()).hexdigest(),
                    'source_scan_sha256': scan_hashes, 'start_frame': start, 'end_frame': end,
                    'accum_interval': interval, 'static_maps': [str(p.relative_to(stage)) for p in static_maps],
                    'python': sys.executable, 'labels': 'Synthetic GT labels for upstream plumbing; ground/instances from Patchwork++/HDBSCAN',
                    'evaluation_warning': 'GT/dynamic metrics using dummy labels are not accuracy measurements',
                    'source_policy': 'Original sequence never written; run/input/mapping/velodyne links original scans',
                    'map_frame': 'Same fixed output_world_frame as source manifest, direct T_world_scan',
                    'navigation_policy': 'No map replacement, 2D occupancy conversion or navigation integration',
                    'pcd_intensity': 'ERASOR2 internal ground/instance encoding, not original sensor reflectance',
                    'mos_indexing': 'uint32 labels index scans AFTER x*x+y*y < robot_body_size*robot_body_size rejection, not original full scans',
                    'mos_frame_counts':mos_records}
        (stage / 'run_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        commit_directory(stage, output)
    except BaseException:
        # Keep diagnostics outside the requested output; a failed run never looks complete.
        error_log = output.parent / (output.name + '.failed.log')
        if (stage / 'run.log').exists():
            with error_log.open('a') as log:
                log.write((stage/'run.log').read_text())
            print(f'Failure log: {error_log}', file=sys.stderr)
        shutil.rmtree(stage, ignore_errors=True)
        raise
    print(f'Static map run complete: {output}')
    for p in static_maps:
        print(output / p.relative_to(stage))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sequence', type=Path)
    parser.add_argument('parameter', type=Path)
    args = parser.parse_args()
    try:
        run(args)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Sequence failed: {error}\n')
