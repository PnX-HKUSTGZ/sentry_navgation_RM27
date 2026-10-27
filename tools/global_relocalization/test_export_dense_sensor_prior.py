import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from export_dense_sensor_prior import (Trajectory, VoxelAccumulator, export_dense,
    fixed_extrinsic, registered_reprojection_check, sha256)
from refine_sensor_prior import transform, voxel


def make_pose(x=0, yaw=0):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();t[0,3]=x;return t


def test_timestamp_brackets_no_extrapolation_and_rotation_wrap():
    rows=[{'stamp':1.,'matrix':make_pose(0,np.deg2rad(179)).tolist()},
          {'stamp':3.,'matrix':make_pose(2,np.deg2rad(-179)).tolist()}]
    t=Trajectory(rows,2.1);mid,meta=t.at(2000000000)
    assert meta['mode']=='interpolated' and np.allclose(mid[:3,3],[1,0,0])
    assert Rotation.from_matrix(mid[:3,:3]).magnitude()==pytest.approx(np.pi)
    assert t.at(999999999)[0] is not None  # explicit 1ns serialization tolerance
    assert t.at(999999998)[1]['reason']=='before_LIO_support'
    assert t.at(3000000002)[1]['reason']=='after_LIO_support'
    assert Trajectory(rows,.15).at(2000000000)[1]['reason']=='LIO_bracket_gap'


def test_voxel_union_keeps_first_points_across_chunks_and_negative_cells():
    a=VoxelAccumulator(.03);p=np.array([[-.02,0,0],[.01,.01,.01],[.02,.02,.02]])
    a.add(p);a.add(p+[0.001,0,0]);assert len(a.points)==2
    assert any(np.allclose(x,p[0])for x in a.points)
    assert any(np.allclose(x,p[1])for x in a.points)
    with pytest.raises(ValueError,match='21-bit'):a.add(np.array([[1e8,0,0]]))


def params():
    return {'point_lio':{'ros__parameters':{'mapping':{'extrinsic_est_en':False,'extrinsic_R':np.eye(3).ravel().tolist(),'extrinsic_T':[0,0,0]},
        'preprocess':{'instantaneous_scan':True,'blind':.5},'point_filter_num':2,'filter_size_surf':.2}},
        'ign_sim_pointcloud_tool':{'ros__parameters':{'instantaneous_scan':True,'ang_bottom':7.22,'ang_res_y':2.01419,'n_scan':32}}}


def test_registered_centroid_identity_check_detects_wrong_pose():
    raw=np.c_[np.linspace(1,2,60),np.linspace(-.1,.3,60),np.full(60,.2)].astype(np.float32)
    t=make_pose(.3,.2);p=params();reg=transform(voxel(raw[::2],.2),t)
    good=registered_reprojection_check(raw,t,reg,p,fixed_extrinsic(p))
    wrong=registered_reprojection_check(raw,np.eye(4),reg,p,np.eye(4))
    assert good['registered_to_reproduced_p95_m']<1e-10
    assert wrong['registered_to_reproduced_p95_m']>.1
    p['point_lio']['ros__parameters']['mapping']['extrinsic_est_en']=True
    with pytest.raises(ValueError,match='estimated extrinsics'):fixed_extrinsic(p)


def test_full_stream_preserves_all_synchronized_frames_and_records_filtering(tmp_path,monkeypatch):
    # The source-provenance verifier has separate hash/identity tests. This
    # fixture isolates the dense stream, range accounting, no-extrapolation and
    # single initial gauge, with an intentionally unusable GT field.
    source,prior,output=[tmp_path/x for x in ['source','prior','dense']];source.mkdir();prior.mkdir();(source/'raw_bag').mkdir();(source/'registered_scans').mkdir()
    import yaml
    (source/'mapping_params.yaml').write_text(yaml.safe_dump(params()));(source/'mapping_binary_provenance.json').write_text('{}')
    poses=[make_pose(),make_pose(.2)];capture={'gt':'MUST NOT READ','clouds':[{'stamp':1.},{'stamp':2.}],'odom':[{'stamp':float(i+1),'matrix':p.tolist()}for i,p in enumerate(poses)]}
    (source/'capture.json').write_text(json.dumps(capture));raw=np.array([[1,0,.2],[1.2,.3,.2],[.1,0,0],[31,0,0],[np.nan,0,0]],np.float32)
    for i,p in enumerate(poses):np.save(source/'registered_scans'/f'{i:06d}.npy',transform(voxel(raw[:2:2],.2),p))
    world=make_pose(5,.4);(prior/'manifest.json').write_text(json.dumps({'T_world_camera_init':world.tolist(),'frame_id':'map','T_map_world':np.eye(4).tolist(),
        'T_base_lidar':np.eye(4).tolist(),'voxel_size':.03,'scan_model':'instantaneous','mapping_converter_timing':'instantaneous','mapper_sha256':'frozen-test'}))
    monkeypatch.setattr('export_dense_sensor_prior.verify_source',lambda *_:{str((source/'capture.json').resolve()):sha256(source/'capture.json')})
    # Raw frame outside support must be documented, never assigned an endpoint.
    frames=[(t,'left_mid360',raw)for t in [900000000,1000000000,2000000000,2100000000]]
    # Calibration check is separately tested above; these tiny streams cannot
    # reproduce the configured 8-point production preprocessing.
    monkeypatch.setattr('export_dense_sensor_prior.registered_reprojection_check',lambda *_:{'registered_to_reproduced_p95_m':0})
    report=export_dense(source,prior,output,frames=frames,expected_raw_count=4)
    assert report['fused_scans']==2 and report['unsynchronized_raw_frames']==2
    assert report['fused_raw_points']==4
    assert report['filtered_point_counts']=={'input_points':20,'nonfinite_points':4,'below_min_range_points':4,'above_max_range_points':4}
    inventory=json.loads((output/'raw_frame_inventory.json').read_text())
    assert inventory[0]['association']['reason']=='before_LIO_support'
    assert inventory[-1]['association']['reason']=='after_LIO_support'
    assert not report['ground_truth_used_for_scan_fusion']
    assert report['map_sha256']==sha256(output/'map.pcd')
    with pytest.raises(FileExistsError):export_dense(source,prior,output,frames=frames,expected_raw_count=4)
    bad_count=tmp_path/'count_failure'
    with pytest.raises(ValueError,match='raw frame count differs'):
        export_dense(source,prior,bad_count,frames=frames,expected_raw_count=5)
    assert not(bad_count/'dense_prior_export_complete.json').exists() and not(bad_count/'map.pcd').exists()
    original_at=Trajectory.at
    monkeypatch.setattr(Trajectory,'at',lambda self,stamp:(None,{'reason':'LIO_bracket_gap','gap_sec':.4}) if stamp==1000000000 else original_at(self,stamp))
    gap_output=tmp_path/'gap_failure'
    with pytest.raises(ValueError,match='internal LIO support gap'):
        export_dense(source,prior,gap_output,frames=frames,expected_raw_count=4)
    assert not(gap_output/'dense_prior_export_complete.json').exists()
    monkeypatch.setattr(Trajectory,'at',original_at)
    capture['clouds'][1]['stamp']=2.1;(source/'capture.json').write_text(json.dumps(capture))
    with pytest.raises(ValueError,match='timestamp inventory differs'):
        export_dense(source,prior,tmp_path/'stamp_failure',frames=frames,expected_raw_count=4)



def test_trajectory_rejects_nonrigid_nonfinite_and_bad_limits():
    for broken in [np.diag([2.,1,1,1]),np.diag([-1.,1,1,1]),np.full((4,4),np.nan)]:
        with pytest.raises(ValueError):Trajectory([{'stamp':0,'matrix':np.eye(4).tolist()},{'stamp':1,'matrix':broken.tolist()}])
    with pytest.raises(ValueError):Trajectory([{'stamp':0,'matrix':np.eye(4).tolist()},{'stamp':1,'matrix':np.eye(4).tolist()}],0)
    with pytest.raises(ValueError,match='invalid range'):export_dense('/unused','/unused','/unused',min_range=30,max_range=.5)
