#!/usr/bin/env python3
"""Extract exact sensor messages and static extrinsics; never reads GT topics."""
import argparse
import json
import hashlib
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage


def transform(t):
    q=t.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    a=np.eye(4);a[:3,:3]=r;a[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return a


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag');p.add_argument('output');p.add_argument('--start',type=float,required=True,help='Original action ROS start stamp, not record start')
    p.add_argument('--end',type=float,default=float('inf'))
    p.add_argument('--sensor-frame',default='left_mid360');p.add_argument('--base-frame',default='base_footprint');p.add_argument('--imu-frame',default=None)
    args=p.parse_args();reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=args.bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    scans=[];imu=[];edges={};imu_frame=args.imu_frame
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/tf_static':
            for t in deserialize_message(data,TFMessage).transforms:
                edges[(t.header.frame_id,t.child_frame_id)]=transform(t.transform)
        elif topic=='/livox/imu':
            m=deserialize_message(data,Imu);imu_frame=imu_frame or m.header.frame_id
            if m.header.frame_id != imu_frame: raise RuntimeError('IMU frame changed')
            t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if args.start<=t<=args.end+0.05:imu.append([t,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
        elif topic=='/livox/lidar':
            m=deserialize_message(data,PointCloud2);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if args.start<=t<=args.end:scans.append((t,m))
    def lookup(target,source):
        queue=[(target,np.eye(4))];seen=set()
        while queue:
            frame,a=queue.pop(0)
            if frame==source:return a
            if frame in seen:continue
            seen.add(frame)
            for (parent,child),t in edges.items():
                if parent==frame:queue.append((child,a@t))
                if child==frame:queue.append((parent,a@np.linalg.inv(t)))
        raise RuntimeError(f'No exact static transform {target} <- {source}')
    si=lookup(args.sensor_frame,imu_frame);bs=lookup(args.base_frame,args.sensor_frame)
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    hashes={str(f.name):hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(args.bag).glob('*.mcap')}
    provenance=dict(source_bag=str(Path(args.bag).resolve()),source_mcap_sha256=hashes,action_start=args.start,action_end=args.end,sensor_frame=args.sensor_frame,imu_frame=imu_frame,base_frame=args.base_frame,numerical_only=True)
    for n,(stamp,m) in enumerate(scans):
        if m.header.frame_id != args.sensor_frame: raise RuntimeError('scan frame mismatch')
        points=point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=False).astype('<f4')
        stem=out/f'{n:06d}';header=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
        stem.with_suffix('.pcd').write_bytes(header.encode()+points.tobytes())
        # Exact IMU history and one upper bracket for this scan, as the live worker.
        upper=next((k for k,s in enumerate(imu) if s[0]>=stamp),len(imu)-1)
        samples=imu[:upper+1]
        j=dict(provenance=provenance,schema=1,stamp=stamp,instantaneous=True,sensor_from_imu=si.ravel().tolist(),base_from_sensor=bs.ravel().tolist(),params=dict(stationary_duration=2.,stationary_min_samples=40,leaf_size=.12,max_correspondence=.5,max_iterations=15,max_fitness=.12,min_inlier_ratio=.65,range_filter=False,min_range=.5,max_range=30.,deterministic=False,submap_scans=1,imu_rotation_prior=False,gyro_noise_density=.005,rotation_sigma_floor=.005,numerical_recovery=False),imu=samples)
        stem.with_suffix('.json').write_text(json.dumps(j))
    print(json.dumps(dict(scans=len(scans),imu=len(imu),sensor_frame=args.sensor_frame,imu_frame=imu_frame,action_start=args.start,output=str(out))))

if __name__=='__main__':main()
