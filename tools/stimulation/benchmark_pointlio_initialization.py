#!/usr/bin/env python3
"""Deterministic synthetic sensor replay through the actual Point-LIO ROS node.

No truth topic is published. Truth is used only for offline scoring. Input packets
are identical for legacy/joint runs. Uses a private ROS_DOMAIN_ID supplied by caller.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

EPOCH = 100.0
SCENARIOS = ['static_level', 'static_tilted', 'static_bias_noise',
             'dynamic_sweep', 'dynamic_snapshot', 'dynamic_extrinsics', 'degenerate_plane']

def stamp(t):
    ns = round(t * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

def secs(t):
    return t.sec + t.nanosec * 1e-9

def truth(t, moving, tilted):
    t = np.asarray(t)
    yaw = np.zeros_like(t)
    pitch = np.full_like(t, .18 if tilted else 0.)
    roll = np.full_like(t, -.5235987756 if tilted else 0.)
    p = np.zeros(t.shape + (3,)); v = p.copy(); a = p.copy()
    if moving:
        yaw = .35*t + .12*np.sin(1.3*t)
        pitch += .15*np.sin(.9*t); roll += .12*np.sin(1.1*t)
        p = np.stack((.25*t+.4*np.sin(1.1*t), .2*(1-np.cos(.8*t)), .08*np.sin(.6*t)), axis=-1)
        v = np.stack((.25+.44*np.cos(1.1*t), .16*np.sin(.8*t), .048*np.cos(.6*t)), axis=-1)
        a = np.stack((-.484*np.sin(1.1*t), .128*np.cos(.8*t), -.0288*np.sin(.6*t)), axis=-1)
    rotation = Rotation.from_euler('ZYX', np.stack((yaw,pitch,roll), axis=-1))
    return rotation,p,v,a

def dataset(name, repeat, duration):
    if name not in SCENARIOS:
        raise ValueError(f'Unknown scenario: {name}')
    moving = name.startswith('dynamic')
    tilted = name != 'static_level'
    snapshot = name.endswith('snapshot')
    noisy = name not in ('static_level','static_tilted','degenerate_plane')
    bg = np.array([.012,-.008,.01]) if noisy else np.zeros(3)
    ba = np.array([.06,-.04,.08]) if noisy else np.zeros(3)
    # Calibrated transform from LiDAR to IMU, matching the C++ scenario.
    extrinsic_R = Rotation.from_euler('Y', .18 if name == 'dynamic_extrinsics' else 0.)
    extrinsic_T = np.array([-.011, -.02329, .04412]) if name == 'dynamic_extrinsics' else np.zeros(3)
    u,v=np.meshgrid(np.arange(-20,21)*.1,np.arange(-20,21)*.1)
    u=u.ravel(); v=v.ravel()
    world=np.stack((np.stack((np.full_like(u,4),u,v),axis=1),
                    np.stack((u,np.full_like(u,4),v),axis=1),
                    np.stack((u,v,np.full_like(u,-2)),axis=1)),axis=1).reshape(-1,3)
    if name=='degenerate_plane': world=world[2::3]
    rng=np.random.default_rng(repeat)
    imu_times=np.arange(round(duration*200)+2)*.005
    R,p,vel,acc=truth(imu_times,moving,tilted)
    omega=(truth(imu_times-1e-5,moving,tilted)[0].inv()*
           truth(imu_times+1e-5,moving,tilted)[0]).as_rotvec()/2e-5
    omega += bg + rng.normal(0,.002 if noisy else 0,omega.shape)
    acc=R.inv().apply(acc+np.array([0,0,9.81]))+ba
    acc += rng.normal(0,.015 if noisy else 0,acc.shape)
    packets=[]; digest=hashlib.sha256(omega.tobytes()+acc.tobytes())
    dtype=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),
                    ('time','<f4'),('ring','<u2')])
    for frame in range(round(duration*10)):
        begin=frame*.1; end=(frame+1)*.1
        t=np.full(len(world),end) if snapshot else begin+.1*(np.arange(len(world))+1)/len(world)
        R,p,_,_=truth(t,moving,tilted)
        points=extrinsic_R.inv().apply(R.inv().apply(world-p)-extrinsic_T)
        points += rng.normal(0,.005 if noisy else 0,world.shape)
        data=np.zeros(len(world),dtype=dtype)
        for axis,col in enumerate('xyz'):data[col]=points[:,axis]
        data['intensity']=1
        data['time']=1e-9 if snapshot else t-begin
        data['ring']=np.arange(len(world))%16
        binary=data.tobytes();digest.update(binary)
        packets.append((end if snapshot else begin,end,binary,len(data)))
    return dict(name=name,repeat=repeat,moving=moving,tilted=tilted,packets=packets,
                imu_times=imu_times,omega=omega,acc=acc,sha256=digest.hexdigest(),
                extrinsic_R=extrinsic_R.as_matrix().ravel().tolist(),extrinsic_T=extrinsic_T.tolist())

def common_interval_metrics(rows, moving, tilted, begin=3.0, end=6.0):
    segment=[r for r in rows if begin-1e-5 <= r['t'] <= end+1e-5]
    if not segment:
        return {}
    first,last=segment[0],segment[-1]
    R0,p0,_,_=truth(first['t'],moving,tilted)
    _,p1,_,_=truth(last['t'],moving,tilted)
    estimated=Rotation.from_quat(first['q']).inv().apply(np.array(last['p'])-first['p'])
    actual=R0.inv().apply(p1-p0)
    speed=[]
    for row in segment:
        R,_,v,_=truth(row['t'],moving,tilted)
        speed.append(float(np.linalg.norm(np.array(row['body_v'])-R.inv().apply(v))))
    return {'common_interval_begin_s':first['t'],'common_interval_end_s':last['t'],
            'common_interval_rpe_m':float(np.linalg.norm(estimated-actual)),
            'common_interval_velocity_rmse_mps':float(np.sqrt(np.mean(np.square(speed))))}

def run_case(node, exe, params, data, method, directory):
    directory.mkdir(parents=True,exist_ok=True)
    ident=f"/{data['name']}_{data['repeat']}_{method}"
    cfg=copy.deepcopy(params)
    cfg['use_sim_time']=True
    cfg['initialization']['method']=method
    cfg['common']['lid_topic']=ident+'/lidar';cfg['common']['imu_topic']=ident+'/imu'
    cfg['common']['reliable_lidar']=True
    cfg['common']['imu_receive_depth']=100
    cfg['preprocess']['scan_line']=16
    cfg['mapping']['extrinsic_T']=data['extrinsic_T']
    cfg['mapping']['extrinsic_R']=data['extrinsic_R']
    cfg['mapping']['gravity']=[0.,0.,-9.81];cfg['mapping']['acc_norm']=9.81
    cfg['publish']['scan_publish_en']=False
    yaml_path=directory/'params.yaml';yaml_path.write_text(yaml.safe_dump({'/**':{'ros__parameters':cfg}}))
    rows=[]; diagnostics=[]
    def odom_callback(msg):
        q=msg.pose.pose.orientation;p=msg.pose.pose.position;v=msg.twist.twist.linear
        rows.append(dict(t=secs(msg.header.stamp)-EPOCH,p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w],
                         body_v=[v.x,v.y,v.z],wall=time.monotonic()))
    def diagnostic_callback(msg):
        for s in msg.status:
            diagnostics.append(dict(t=secs(msg.header.stamp)-EPOCH,name=s.name,message=s.message,
                                    values={v.key:v.value for v in s.values}))
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
    imu_pub=node.create_publisher(Imu,ident+'/imu',qos)
    cloud_pub=node.create_publisher(PointCloud2,ident+'/lidar',qos)
    clock_pub=node.create_publisher(Clock,'/clock',10)
    odom_sub=node.create_subscription(Odometry,ident+'/odom',odom_callback,qos)
    diag_sub=node.create_subscription(DiagnosticArray,ident+'/diagnostics',diagnostic_callback,qos)
    with (directory/'node.log').open('w') as log:
        proc=subprocess.Popen([str(exe),'--ros-args','--params-file',str(yaml_path),
                               '-r','aft_mapped_to_init:='+ident+'/odom',
                               '-r','point_lio/diagnostics:='+ident+'/diagnostics'],stdout=log,stderr=subprocess.STDOUT)
        try:
            limit=time.monotonic()+15
            while (imu_pub.get_subscription_count()==0 or cloud_pub.get_subscription_count()==0 or
                   odom_sub.get_publisher_count()==0 or diag_sub.get_publisher_count()==0 or
                   clock_pub.get_subscription_count()==0):
                if proc.poll() is not None: raise RuntimeError(f'node exited: {directory}/node.log')
                if time.monotonic()>limit:raise RuntimeError('DDS discovery timeout')
                clock_pub.publish(Clock(clock=stamp(EPOCH)))
                rclpy.spin_once(node,timeout_sec=.02)
            # IMU ingress uses SensorDataQoS (best effort). Graph discovery can
            # precede full reader readiness; let it settle before the first data.
            # Pacing below also avoids a burst of 20 IMUs into that reader.
            settled_until=time.monotonic()+1.
            while time.monotonic()<settled_until:
                clock_pub.publish(Clock(clock=stamp(EPOCH)))
                rclpy.spin_once(node,timeout_sec=.01)
            first_wall=time.monotonic();imu_cursor=0;missing_packets=0
            for begin,end,binary,count in data['packets']:
                clock_pub.publish(Clock(clock=stamp(EPOCH+end+.005)))
                while imu_cursor<len(data['imu_times']) and data['imu_times'][imu_cursor]<=end+.005+1e-8:
                    msg=Imu();msg.header.stamp=stamp(EPOCH+data['imu_times'][imu_cursor]);msg.header.frame_id='imu_link'
                    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=map(float,data['omega'][imu_cursor])
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=map(float,data['acc'][imu_cursor])
                    imu_pub.publish(msg);imu_cursor+=1
                    time.sleep(.001)
                rclpy.spin_once(node,timeout_sec=.005)
                cloud=PointCloud2();cloud.header.stamp=stamp(EPOCH+begin);cloud.header.frame_id='lidar'
                cloud.height=1;cloud.width=count;cloud.is_dense=True;cloud.point_step=22;cloud.row_step=count*22
                cloud.fields=[PointField(name=n,offset=o,datatype=t,count=1) for n,o,t in
                              [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('time',16,7),('ring',20,4)]]
                cloud.data=binary;cloud_pub.publish(cloud)
                deadline=time.monotonic()+(2 if method=='joint' or rows else .08)
                acknowledged=False
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.005)
                    if ((rows and rows[-1]['t']>=end-1e-5) or
                        (diagnostics and diagnostics[-1]['t']>=end-1e-5)):
                        acknowledged=True;break
                    if proc.poll() is not None:raise RuntimeError('node exited during replay')
                if not acknowledged and (method=='joint' or rows):missing_packets+=1
            for _ in range(10):rclpy.spin_once(node,timeout_sec=.01)
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
    result=dict(scenario=data['name'],repeat=data['repeat'],method=method,input_sha256=data['sha256'],
                success=bool(rows),output_count=len(rows),unacknowledged_packets=missing_packets,
                process_returncode=proc.returncode)
    if rows:
        first=rows[0];r0,p0,v0,_=truth(first['t'],data['moving'],data['tilted'])
        e0=Rotation.from_quat(first['q']);ep0=np.array(first['p'])
        result['first_output_sensor_s']=first['t'];result['first_output_wall_s']=first['wall']-first_wall
        tilt=[];speed=[];rpe=[];times=[]
        for row in rows:
            r,p,v,_=truth(row['t'],data['moving'],data['tilted']);er=Rotation.from_quat(row['q'])
            tilt.append(float(np.degrees(np.arccos(np.clip(np.dot(er.inv().apply([0,0,1]),r.inv().apply([0,0,1])),-1,1)))))
            speed.append(float(np.linalg.norm(np.array(row['body_v'])-r.inv().apply(v))))
            # Relative displacement in the initial body frame: no trajectory fitting or scale adjustment.
            rpe.append(float(np.linalg.norm(e0.inv().apply(np.array(row['p'])-ep0)-r0.inv().apply(p-p0))))
            times.append(row['t'])
        result['first_tilt_deg']=tilt[0];result['first_velocity_error_mps']=speed[0]
        idx=int(np.argmin(np.abs(np.array(times)-(first['t']+3))))
        result['rpe_at_3s_m']=rpe[idx];result['rpe_horizon_s']=times[idx]-first['t']
        result['velocity_rmse_mps']=float(np.sqrt(np.mean(np.square(speed))))
        result['tilt_rmse_deg']=float(np.sqrt(np.mean(np.square(tilt))))
        result['final_rpe_m']=rpe[-1]
        result['nonmonotonic_stamps']=sum(b['t']<=a['t'] for a,b in zip(rows,rows[1:]))
        result['imu_update_rejections']=sum(int(d['values'].get('rejected_imu_updates',0)) for d in diagnostics if d['name']=='point_lio/estimator')
        result['dropped_lidar_frames']=max([int(d['values'].get('dropped_lidar_frames',0)) for d in diagnostics]+[0])
    result.update(common_interval_metrics(rows,data['moving'],data['tilted']))
    (directory/'odom.json').write_text(json.dumps(rows,indent=2))
    (directory/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2))
    (directory/'result.json').write_text(json.dumps(result,indent=2))
    for x in [odom_sub,diag_sub]:node.destroy_subscription(x)
    for x in [imu_pub,cloud_pub,clock_pub]:node.destroy_publisher(x)
    return result

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scenarios',nargs='+',choices=SCENARIOS,default=SCENARIOS[:5])
    parser.add_argument('--repetitions',type=int,default=3)
    parser.add_argument('--methods',nargs='+',choices=['legacy','joint'],default=['legacy','joint'])
    parser.add_argument('--duration',type=float,default=6)
    parser.add_argument('--initialization-overrides',type=Path,help='YAML mapping of initialization ROS parameters')
    args=parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID'):raise RuntimeError('Set an isolated ROS_DOMAIN_ID before replay')
    params=yaml.safe_load((args.workspace/'src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml').read_text())['point_lio']['ros__parameters']
    if args.initialization_overrides:
        params['initialization'].update(yaml.safe_load(args.initialization_overrides.read_text()))
    exe=args.workspace/'build/point_lio/pointlio_mapping'
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('initialization_benchmark');results=[]
    try:
        for scenario in args.scenarios:
            for repeat in range(1,args.repetitions+1):
                data=dataset(scenario,repeat,args.duration)
                for method in args.methods:
                    result=run_case(node,exe,params,data,method,args.output/f'{scenario}_{repeat}_{method}')
                    results.append(result);print(json.dumps(result),flush=True)
                    (args.output/'summary.json').write_text(json.dumps(results,indent=2))
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
