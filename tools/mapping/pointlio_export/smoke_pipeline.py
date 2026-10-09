#!/usr/bin/env python3
"""Real rosbag2 export + actual Patchwork++/HDBSCAN + C++ ERASOR2 acceptance smoke."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
import yaml

from test_export import make_bag
from export_bag import quaternion_rotation

ROOT = Path(__file__).resolve().parents[1]


def fingerprint(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*') if p.is_file()}


def command(args, env=None, success=True):
    result = subprocess.run([str(x) for x in args], env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if (result.returncode == 0) != success:
        raise AssertionError(f'Unexpected command exit {result.returncode}: {args}\n{result.stdout}')
    return result.stdout


def pcd_header(path):
    lines=[]
    with path.open('rb') as stream:
        while True:
            raw=stream.readline()
            if not raw: raise AssertionError('Missing PCD DATA header')
            line=raw.decode('ascii').strip(); lines.append(line)
            if line.startswith('DATA '): break
    values={line.split(' ',1)[0]:line.split(' ',1)[1] for line in lines if ' ' in line and not line.startswith('#')}
    assert int(values['POINTS']) > 0
    return values


def smoke(directory):
    directory.mkdir(parents=True,exist_ok=False)
    bag=directory/'bag'
    make_bag(bag)
    bag_before=fingerprint(bag)
    command([sys.executable,ROOT/'pointlio_export/test_export.py'])
    export=ROOT/'scripts/export_pointlio.sh'
    command([export,bag,directory/'sequence'])
    sequence=directory/'sequence'
    expected=np.load(directory/'expected_local.npz')
    for i in range(8):
        actual=np.fromfile(sequence/'velodyne'/f'{i:06d}.bin',dtype='<f4').reshape(-1,4)
        np.testing.assert_allclose(actual,expected[f'frame_{i}'],atol=1e-6,rtol=1e-5)
    manifest=json.loads((sequence/'manifest.json').read_text())
    assert manifest['frames'][0]['dropped_nonfinite']==1
    assert manifest['scan_frame']=='body'
    assert all(x['pose_match']['method']=='exact' for x in manifest['frames'])
    # Real lidar origin with a nontrivial static rotation/translation and a fixed map alignment.
    body_lidar=np.eye(4)
    body_lidar[:3,:3]=quaternion_rotation([0,0,np.sin(.12),np.cos(.12)])
    body_lidar[:3,3]=[.2,-.1,.08]
    alignment=np.eye(4)
    alignment[:3,:3]=quaternion_rotation([0,0,np.sin(.3),np.cos(.3)])
    alignment[:3,3]=[10,4,-2]
    extrinsics=directory/'calibration.yaml'; extrinsics.write_text(yaml.safe_dump({'T_body_lidar':body_lidar.tolist()}))
    align_file=directory/'alignment.yaml'; align_file.write_text(yaml.safe_dump({'T_output_world':alignment.tolist()}))
    command([export,bag,directory/'lidar_sequence','--scan-frame','lidar','--extrinsics',extrinsics,
             '--alignment',align_file,'--output-world-frame','map'])
    lidar_poses=np.loadtxt(directory/'lidar_sequence/poses_suma_optim.txt').reshape(-1,3,4)
    body_poses=np.loadtxt(sequence/'poses_suma_optim.txt').reshape(-1,3,4)
    for i in range(8):
        local=np.fromfile(directory/'lidar_sequence/velodyne'/f'{i:06d}.bin',dtype='<f4').reshape(-1,4)
        expected_local=expected[f'frame_{i}'].astype(float)
        np.testing.assert_allclose(local[:,:3],(expected_local[:,:3]-body_lidar[:3,3])@body_lidar[:3,:3],atol=2e-6,rtol=1e-5)
        body=np.eye(4);body[:3]=body_poses[i]
        np.testing.assert_allclose(lidar_poses[i],(alignment@body@body_lidar)[:3],atol=1e-9)
    assert fingerprint(bag)==bag_before
    sequence_before=fingerprint(sequence)
    env=os.environ.copy();env['ERASOR2_OUTPUT_DIR']=str(directory/'run')
    print(command([ROOT/'scripts/run_sequence.sh',sequence,ROOT/'configs/mid360_start.yaml'],env=env))
    assert fingerprint(sequence)==sequence_before
    run=directory/'run'; results=run/'results'
    maps=list(results.glob('*_streaming_estimated.pcd')); assert len(maps)==1
    header=pcd_header(maps[0])
    for name in ('original','voxel'):
        pattern='*_original.pcd' if name=='original' else '*_w_interval_1_voxel_0_1.pcd'
        for path in results.glob(pattern): pcd_header(path)
    preprocessing=json.loads((run/'preprocessing.json').read_text())
    assert len(preprocessing['frames'])==8
    assert all(x['ground_points']>0 and x['cluster_count']>0 for x in preprocessing['frames'])
    mos=[]
    for i in range(8):
        points=np.fromfile(sequence/'velodyne'/f'{i:06d}.bin',dtype='<f4').reshape(-1,4)
        kept=(points[:,0].astype(float)**2+points[:,1].astype(float)**2 >= .25**2).sum()
        labels=results/'mos'/f'{i:06d}.label'
        assert labels.stat().st_size==kept*4
        assert set(np.unique(np.fromfile(labels,dtype='<u4'))).issubset({0,251})
        mos.append({'frame':i,'filtered_points':int(kept),'bytes':labels.stat().st_size})
    # Original output must not be overwritten, including a dangling symlink.
    command([export,bag,sequence],success=False)
    symlink=directory/'dangling';symlink.symlink_to(directory/'does-not-exist')
    command([export,bag,symlink],success=False)
    assert symlink.is_symlink() and not (directory/'does-not-exist').exists()
    command([ROOT/'scripts/run_sequence.sh',sequence,ROOT/'configs/mid360_start.yaml'],env=env,success=False)
    # Fail after several output frames: no partial directory and no source mutation.
    bad=directory/'bad_bag';make_bag(bad,bad_pose_frame=4);bad_before=fingerprint(bad)
    command([export,bad,directory/'failed_sequence'],success=False)
    assert not (directory/'failed_sequence').exists()
    assert not list(directory.glob('.failed_sequence.*'))
    assert fingerprint(bad)==bad_before
    summary={'rosbag2_frames':8,'points_per_frame':3300,'local_geometry_comparisons':8,
             'lidar_extrinsic_alignment_comparisons':8,'source_bag_unchanged':True,'source_sequence_unchanged':True,
             'strict_failure_atomic':True,'existing_output_and_symlink_rejected':True,
             'real_preprocessing':'pypatchworkpp + hdbscan','real_cpp':'mapgen + run_erasor2 streaming',
             'static_pcd':str(maps[0]),'static_points':int(header['POINTS']),'static_pcd_bytes':maps[0].stat().st_size,
             'mos_filtered_scan_indexing':mos,
             'scope':'smoke only; no dynamic-removal quality, navigation or real MID360 validation'}
    (directory/'acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,help='New fixture directory; defaults to a temporary directory')
    a=p.parse_args()
    output=a.output or (Path(tempfile.mkdtemp(prefix='mapping-acceptance-'))/'fixture')
    smoke(output.expanduser().absolute())
