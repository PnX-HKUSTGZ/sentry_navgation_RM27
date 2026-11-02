#!/usr/bin/env python3
"""Fuse full frozen raw LiDAR with the unchanged recorded LIO trajectory.

No estimator, graph correction, GT trajectory or ideal map is read. The original
raw prior supplies its single initial gauge. Header timestamps require exact or
bounded interpolation; outside support is recorded and never extrapolated.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml
from refine_sensor_prior import invert, transform, verify_source, voxel


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


class Trajectory:
    def __init__(self, rows, maximum_gap=.15):
        self.stamps=np.array([round(x['stamp']*1e9)for x in rows],dtype=np.int64)
        self.poses=np.array([x['matrix']for x in rows],dtype=float);self.maximum_gap=maximum_gap
        if (not np.isfinite(maximum_gap) or maximum_gap<=0 or len(rows)<2 or self.poses.shape!=(len(rows),4,4)
                or np.any(np.diff(self.stamps)<=0)or not np.isfinite(self.poses).all()):raise ValueError('invalid LIO trajectory')
        r=self.poses[:,:3,:3]
        if (not np.allclose(np.transpose(r,(0,2,1))@r,np.eye(3),atol=1e-6)
                or not np.allclose(np.linalg.det(r),1.,atol=1e-6)
                or not np.allclose(self.poses[:,3,:],[0,0,0,1],atol=1e-9)):raise ValueError('LIO poses are not SE3')
    def at(self,stamp):
        i=int(np.searchsorted(self.stamps,stamp))
        if i<len(self.stamps)and abs(int(self.stamps[i])-stamp)<=1:return self.poses[i],{'mode':'exact','index':i}
        if i and abs(int(self.stamps[i-1])-stamp)<=1:return self.poses[i-1],{'mode':'exact','index':i-1}
        if i==0:return None,{'reason':'before_LIO_support'}
        if i==len(self.stamps):return None,{'reason':'after_LIO_support'}
        gap=(self.stamps[i]-self.stamps[i-1])*1e-9
        if gap>self.maximum_gap:return None,{'reason':'LIO_bracket_gap','gap_sec':gap}
        u=(stamp-self.stamps[i-1])/(self.stamps[i]-self.stamps[i-1]);a,b=self.poses[i-1],self.poses[i]
        ra,rb=Rotation.from_matrix(a[:3,:3]),Rotation.from_matrix(b[:3,:3]);t=np.eye(4);t[:3,:3]=(ra*Rotation.from_rotvec(u*(ra.inv()*rb).as_rotvec())).as_matrix();t[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
        return t,{'mode':'interpolated','left_index':i-1,'right_index':i,'gap_sec':gap}


class VoxelAccumulator:
    def __init__(self,leaf):
        self.leaf=leaf;self.keys=np.empty(0,np.int64);self.points=np.empty((0,3),np.float32)
    def add(self,points):
        if not len(points):return
        grid=np.floor(np.asarray(points)/self.leaf).astype(np.int64);bias=1<<20
        if (grid< -bias).any()or(grid>=bias).any():raise ValueError('voxel index outside explicit 21-bit bound')
        grid+=bias;keys=(grid[:,0]<<42)|(grid[:,1]<<21)|grid[:,2];keys,first=np.unique(keys,return_index=True);points=np.asarray(points,dtype=np.float32)[first]
        if len(self.keys):
            ix=np.searchsorted(self.keys,keys);new=(ix==len(self.keys));new|=self.keys[np.minimum(ix,len(self.keys)-1)]!=keys;keys,points=keys[new],points[new]
        if len(keys):
            keys=np.r_[self.keys,keys];points=np.concatenate([self.points,points]);order=np.argsort(keys);self.keys,self.points=keys[order],points[order]


def raw_frames(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=metadata['storage_identifier']),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/livox/lidar']))
    while reader.has_next():
        topic,payload,_=reader.read_next();msg=deserialize_message(payload,PointCloud2)
        data=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=False)
        xyz=np.column_stack([data[n].reshape(-1)for n in ['x','y','z']])
        yield msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,msg.header.frame_id,xyz


def fixed_extrinsic(params):
    p=params['point_lio']['ros__parameters'];m=p['mapping']
    if m['extrinsic_est_en']:raise ValueError('estimated extrinsics were not captured per frame')
    e=np.eye(4);e[:3,:3]=np.array(m['extrinsic_R']).reshape(3,3);e[:3,3]=m['extrinsic_T']
    if not np.allclose(e,np.eye(4),atol=1e-12):raise ValueError('this exporter currently verifies only the captured identity IMU/LiDAR calibration')
    if not p['preprocess']['instantaneous_scan']or not params['ign_sim_pointcloud_tool']['ros__parameters']['instantaneous_scan']:raise ValueError('full-scan projection requires verified instantaneous source model')
    return e


def registered_reprojection_check(raw,pose,registered,params,extrinsic):
    conv=params['ign_sim_pointcloud_tool']['ros__parameters'];p=params['point_lio']['ros__parameters'];q=raw[np.isfinite(raw).all(axis=1)]
    angle=np.rad2deg(np.arctan2(q[:,2],np.linalg.norm(q[:,:2],axis=1)));ring=((angle+conv['ang_bottom'])/conv['ang_res_y']).astype(int);q=q[(ring>=0)&(ring<conv['n_scan'])]
    q=q[::p['point_filter_num']];d=np.linalg.norm(q,axis=1);q=q[d>p['preprocess']['blind']]
    expected=transform(voxel(q,p['filter_size_surf']),pose@extrinsic);distance,_=cKDTree(expected).query(registered)
    return {'registered_points':len(registered),'reproduced_centroids':len(expected),'registered_to_reproduced_median_m':float(np.median(distance)),
        'registered_to_reproduced_p95_m':float(np.quantile(distance,.95)),'registered_to_reproduced_max_m':float(distance.max())}


def export_dense(source,raw_prior,output,frames=None,min_range=.5,max_range=30.,maximum_gap=.15,expected_raw_count=None):
    if not np.isfinite([min_range,max_range,maximum_gap]).all()or min_range<0 or max_range<=min_range or maximum_gap<=0:raise ValueError('invalid range or interpolation limits')
    source,raw_prior,output=[Path(x).resolve()for x in [source,raw_prior,output]]
    if output.exists()or output==source or output.is_relative_to(source):raise FileExistsError(str(output))
    source_hashes=verify_source(source,raw_prior);source_hashes[str(Path(__file__).resolve())]=sha256(__file__)
    bag=source/'raw_bag'
    for path in sorted(bag.iterdir()):
        if path.is_file():source_hashes[str(path.resolve())]=sha256(path)
    if frames is None:
        metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        counts=[x['message_count']for x in metadata['topics_with_message_count']if x['topic_metadata']['name']=='/livox/lidar']
        if len(counts)!=1 or counts[0]<=0:raise ValueError('raw LiDAR metadata count unavailable')
        expected_raw_count=counts[0]
    elif expected_raw_count is None or expected_raw_count<=0:raise ValueError('injected stream requires independent expected_raw_count')
    capture=json.loads((source/'capture.json').read_text());prior=json.loads((raw_prior/'manifest.json').read_text());params=yaml.safe_load((source/'mapping_params.yaml').read_text());extrinsic=fixed_extrinsic(params)
    trajectory=Trajectory(capture['odom'],maximum_gap)
    if len(capture['clouds'])!=len(trajectory.stamps)or any(round(x['stamp']*1e9)!=int(t)for x,t in zip(capture['clouds'],trajectory.stamps)):raise ValueError('source cloud/LIO timestamp inventory differs')
    if not np.isfinite(prior['voxel_size'])or prior['voxel_size']<=0:raise ValueError('invalid source voxel size')
    check_indices=set(np.rint(np.linspace(0,len(trajectory.stamps)-1,min(12,len(trajectory.stamps)))).astype(int).tolist())
    world=np.array(prior['T_world_camera_init']);output.mkdir(parents=True)
    (output/'source_inputs.json').write_text(json.dumps(source_hashes,indent=2)+'\n');(output/'exporter_source.py').write_bytes(Path(__file__).read_bytes())
    rules={'min_range_m':min_range,'max_range_m':max_range,'maximum_LIO_bracket_gap_sec':maximum_gap,'extrapolation':False,'voxel_size_m':prior['voxel_size'],
        'point_selection':'all finite in-range raw LiDAR returns, no point_filter_num/feature/registration/GT-based selection','pose_source':'unchanged source capture LIO','T_imu_lidar':extrinsic.tolist(),'required_registered_check_indices':sorted(check_indices),'expected_raw_lidar_frames':int(expected_raw_count)}
    (output/'rules.json').write_text(json.dumps(rules,indent=2)+'\n');start=time.monotonic();accumulator=VoxelAccumulator(prior['voxel_size']);pending=[];records=[];checks=[];last=None
    covered=set();checked=set()
    for index,(stamp,frame,points)in enumerate(raw_frames(bag)if frames is None else frames):
        if last is not None and stamp<=last:raise ValueError('raw LiDAR header timestamp order invalid')
        last=stamp
        if frame!='left_mid360':raise ValueError('unexpected raw sensor frame: '+frame)
        pose,association=trajectory.at(stamp)
        if association.get('reason')=='LIO_bracket_gap':raise ValueError('internal LIO support gap: '+json.dumps(association))
        if association.get('mode')=='exact':covered.add(association['index'])
        finite=np.isfinite(points).all(axis=1);q=points[finite];distance=np.linalg.norm(q,axis=1);near=distance<min_range;far=distance>max_range;selected=q[~(near|far)]
        row={'raw_index':index,'stamp_ns':int(stamp),'frame':frame,'input_points':len(points),'nonfinite_points':int((~finite).sum()),'below_min_range_points':int(near.sum()),'above_max_range_points':int(far.sum()),'finite_in_range_points':len(selected),'association':association,'fused':pose is not None}
        records.append(row)
        if pose is None:continue
        if association['mode']=='exact'and association['index']in check_indices:
            reg=np.load(source/'registered_scans'/f"{association['index']:06d}.npy");check=registered_reprojection_check(points,pose,reg,params,extrinsic);checks.append({'stamp_ns':int(stamp),'source_index':association['index'],**check});checked.add(association['index'])
            if check['registered_to_reproduced_p95_m']>params['point_lio']['ros__parameters']['filter_size_surf']*.25:raise ValueError('registered reprojection/calibration check failed: '+json.dumps(check))
        pending.append(transform(selected,pose@extrinsic))
        if len(pending)>=20:accumulator.add(np.concatenate(pending));pending=[]
        if index%200==0:print(json.dumps({'raw_frames':index+1,'map_points':len(accumulator.points),'wall_sec':time.monotonic()-start}),flush=True)
    if len(records)!=expected_raw_count:raise ValueError(f'raw frame count differs: actual={len(records)}, expected={expected_raw_count}')
    if covered!=set(range(len(trajectory.stamps))):raise ValueError('raw inventory does not cover every captured LIO/cloud timestamp')
    if checked!=check_indices:raise ValueError('required registered reprojection checks not all executed')
    if pending:accumulator.add(np.concatenate(pending))
    (output/'raw_frame_inventory.json').write_text(json.dumps(records,indent=2)+'\n');(output/'registered_reprojection_checks.json').write_text(json.dumps(checks,indent=2)+'\n')
    points=transform(accumulator.points,world).astype('<f4');path=output/'map.pcd'
    header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
    with path.open('wb')as f:f.write(header.encode());f.write(points.tobytes())
    for input_path,expected in source_hashes.items():
        if sha256(input_path)!=expected:raise ValueError('source changed during dense export: '+input_path)
    used=[x for x in records if x['fused']];unmatched=[x for x in records if not x['fused']]
    manifest={'schema_version':1,'frame_id':prior['frame_id'],'T_map_world':prior['T_map_world'],'T_world_camera_init':world.tolist(),'T_base_lidar':prior['T_base_lidar'],
        'pose_source':'unchanged raw LIO trajectory, dense full raw LiDAR projection','capture_complete':True,'ground_truth_used_for_scan_fusion':False,'ground_truth_used_for_optimization':False,
        'ideal_3d_map_used':False,'optimized_graph_used_for_3d_fusion':False,'slam_toolbox_used':False,'two_dimensional_graph_used':False,
        'source_capture':str(source),'derived_from':str(raw_prior),'source_hashes':source_hashes,'source_inputs_verified_unchanged':True,
        'source_lio_pose_count':len(trajectory.stamps),'source_lio_stamps_all_exactly_covered':True,'required_registered_checks_all_executed':True,'metadata_raw_frame_count_verified':True,'raw_lidar_frame_count':len(records),'fused_scans':len(used),'synchronized_raw_frames_all_preserved':True,
        'unsynchronized_raw_frames':len(unmatched),'unsynchronized_raw_frame_inventory':'raw_frame_inventory.json','dropped_cloud_indices':[x['raw_index']for x in unmatched],
        'fused_raw_points':sum(x['finite_in_range_points']for x in used),'filtered_point_counts':{k:sum(x[k]for x in records)for k in ['input_points','nonfinite_points','below_min_range_points','above_max_range_points']},
        'scan_model':prior['scan_model'],'mapping_converter_timing':prior['mapping_converter_timing'],'pointlio_instantaneous_scan':True,'timing_model_consistent':True,
        'voxel_size':prior['voxel_size'],'map_sha256':sha256(path),'points':len(points),'bounds':[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
        'mapper_sha256':prior['mapper_sha256'],'mapping_params_sha256':sha256(source/'mapping_params.yaml'),'rules_sha256':sha256(output/'rules.json'),
        'trajectory_accuracy':'unchanged raw LIO trajectory; no GT sample used here; evaluate independently','wall_sec':time.monotonic()-start}
    shutil.copyfile(source/'mapping_params.yaml',output/'mapping_params.yaml');shutil.copyfile(source/'mapping_binary_provenance.json',output/'mapping_binary_provenance.json')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(output/'dense_prior_export_complete.json').write_text(json.dumps({'completed':True,'map_sha256':manifest['map_sha256'],'manifest_sha256':sha256(output/'manifest.json')},indent=2)+'\n');print(json.dumps(manifest,indent=2),flush=True);return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('raw_prior',type=Path);p.add_argument('output',type=Path);a=p.parse_args();export_dense(a.source,a.raw_prior,a.output)


if __name__=='__main__':main()
