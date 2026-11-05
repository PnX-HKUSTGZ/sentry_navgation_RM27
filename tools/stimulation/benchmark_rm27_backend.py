#!/usr/bin/env python3
"""Reproducible RM27 backend resource/accuracy and full-stack A/B checks.

Requires a private ROS_DOMAIN_ID. CPU=100% means one logical CPU. Full-stack
RSS/PSS sums exclude this observer; Gazebo is reported separately from navigation.
"""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
import psutil
import yaml
import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data
from rm27_localization_interfaces.msg import FrontendFrame
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('startup', Path(__file__).with_name('benchmark_pointlio_initialization.py'))
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


def stop(proc, group=False):
    if proc.poll() is None:
        if group:
            os.killpg(proc.pid, signal.SIGINT)
        else:
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            if group:
                os.killpg(proc.pid, signal.SIGKILL)
            else:
                proc.kill()
            proc.wait()


class Monitor:
    def __init__(self, roots):
        self.roots = roots
        self.samples = []
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        previous = {}
        while not self.done.wait(.2):
            now = time.monotonic()
            processes = {}
            for pid in self.roots():
                try:
                    parent = psutil.Process(pid)
                    for p in [parent] + parent.children(recursive=True):
                        processes[p.pid] = p
                except psutil.Error:
                    pass
            row = {'wall': now, 'processes': []}
            for pid, p in processes.items():
                try:
                    name = p.name()
                    cmd = ' '.join(p.cmdline())
                    if 'rm27_backend_node' in cmd:
                        category = 'backend'
                    elif 'pointlio_mapping' in cmd:
                        category = 'pointlio'
                    elif ('gz sim' in cmd or 'gz-sim' in name or 'ruby' == name):
                        category = 'simulator'
                    else:
                        category = 'other_navigation'
                    cpu = sum(p.cpu_times()[:2])
                    before = previous.get(pid)
                    percent = 100*(cpu-before[1])/(now-before[0]) if before else 0.
                    previous[pid] = (now, cpu)
                    mem = p.memory_full_info()
                    row['processes'].append(dict(pid=pid, category=category, name=name,
                        cpu_percent=percent, rss=mem.rss, pss=getattr(mem, 'pss', mem.rss)))
                except (psutil.Error, OSError):
                    pass
            self.samples.append(row)

    def finish(self, directory, since=0):
        self.done.set()
        self.thread.join()
        (directory/'resources.json').write_text(json.dumps(self.samples, indent=2))
        result = {}
        groups = dict(backend=['backend'], pointlio=['pointlio'], simulator=['simulator'],
                      navigation=['backend', 'pointlio', 'other_navigation'],
                      total=['backend', 'pointlio', 'other_navigation', 'simulator'])
        for name, categories in groups.items():
            rows = [[p for p in r['processes'] if p['category'] in categories]
                    for r in self.samples if r['wall'] >= since]
            rows = [r for r in rows if r]
            if not rows:
                continue
            cpu = [sum(p['cpu_percent'] for p in r) for r in rows]
            rss = [sum(p['rss'] for p in r)/2**20 for r in rows]
            pss = [sum(p['pss'] for p in r)/2**20 for r in rows]
            result[name] = dict(samples=len(rows), cpu_mean=float(np.mean(cpu)),
                cpu_p95=float(np.percentile(cpu,95)), rss_mean_mib=float(np.mean(rss)),
                rss_max_mib=max(rss), pss_mean_mib=float(np.mean(pss)), pss_max_mib=max(pss))
        return result


class Observer:
    def __init__(self, node):
        self.node = node
        self.odom = []
        self.frames = {}
        self.diagnostics = []
        self.errors = []
        self.subs = [
            node.create_subscription(Odometry, '/rm27_backend/odometry', self.on_odom, 100),
            node.create_subscription(FrontendFrame, '/point_lio/backend_frame', self.on_frame, qos_profile_sensor_data),
            node.create_subscription(DiagnosticArray, '/rm27_backend/diagnostics', self.on_diag, 100),
        ]

    def on_odom(self, m):
        self.odom.append(m)

    def on_frame(self, m):
        self.frames[(m.odometry.header.stamp.sec, m.odometry.header.stamp.nanosec)] = m.odometry

    def on_diag(self, m):
        for s in m.status:
            self.diagnostics.append({'wall': time.monotonic(), 'message': s.message,
                                     'values': {v.key: v.value for v in s.values}})

    def finish(self, directory, truth_data=None):
        errors = []
        rows = []
        for m in self.odom:
            key = (m.header.stamp.sec, m.header.stamp.nanosec)
            f = self.frames.get(key)
            def pose(msg):
                p = msg.pose.pose.position; q = msg.pose.pose.orientation
                return np.array([p.x,p.y,p.z]), Rotation.from_quat([q.x,q.y,q.z,q.w])
            p, r = pose(m)
            row = dict(t=startup.secs(m.header.stamp)-startup.EPOCH,p=p.tolist(),q=r.as_quat().tolist())
            if f is not None:
                fp, fr = pose(f)
                errors.append((float(np.linalg.norm(p-fp)), float(np.degrees((fr.inv()*r).magnitude()))))
            rows.append(row)
        (directory/'backend_poses.json').write_text(json.dumps(rows,indent=2))
        (directory/'backend_diagnostics.json').write_text(json.dumps(self.diagnostics,indent=2))
        summary = {'outputs': len(rows), 'paired_frontend_outputs': len(errors)}
        if errors:
            summary.update(position_difference_max_m=max(e[0] for e in errors),
                           rotation_difference_max_deg=max(e[1] for e in errors))
        if self.diagnostics:
            summary['last_diagnostics'] = self.diagnostics[-1]['values']
        if truth_data and rows:
            aligned = [r for r in rows if 3-1e-5<=r['t']<=6+1e-5]
            if aligned:
                first,last = aligned[0],aligned[-1]
                r0,p0,_,_ = startup.truth(first['t'],truth_data['moving'],truth_data['tilted'])
                _,p1,_,_ = startup.truth(last['t'],truth_data['moving'],truth_data['tilted'])
                estimate = Rotation.from_quat(first['q']).inv().apply(np.array(last['p'])-first['p'])
                summary['common_interval_rpe_m'] = float(np.linalg.norm(estimate-r0.inv().apply(p1-p0)))
        for s in self.subs:
            self.node.destroy_subscription(s)
        return summary


def backend_process(directory, overrides=None):
    cfg = yaml.safe_load((ROOT/'src/pnx-localization/backend/config/backend.yaml').read_text())
    cfg['rm27_backend']['ros__parameters'].update(overrides or {})
    path = directory/'backend_params.yaml';path.write_text(yaml.safe_dump(cfg))
    log = (directory/'backend.log').open('w')
    proc = subprocess.Popen([str(ROOT/'install/rm27_backend/lib/rm27_backend/rm27_backend_node'),
        '--ros-args','--params-file',str(path)],stdout=log,stderr=subprocess.STDOUT)
    log.close()
    return proc


def spin(node, duration):
    deadline=time.monotonic()+duration
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.01)


def pointlio(args, node):
    results=[]
    params=yaml.safe_load((ROOT/'src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml').read_text())['point_lio']['ros__parameters']
    for scenario in args.scenarios:
        for repeat in range(1,args.repetitions+1):
            data=startup.dataset(scenario,repeat,6)
            for enabled in [False,True]:
                d=args.output/f'{scenario}_{repeat}_{"backend" if enabled else "baseline"}';d.mkdir(parents=True,exist_ok=True)
                backend=backend_process(d) if enabled else None
                observer=Observer(node) if enabled else None
                spin(node,1)
                # Monitor only child processes; no synthetic-data generator CPU.
                monitor=Monitor(lambda:[p.pid for p in psutil.Process().children()])
                try:
                    front=startup.run_case(node,ROOT/'build/point_lio/pointlio_mapping',params,data,'joint',d/'front')
                    if front['process_returncode'] != 0:
                        raise RuntimeError(f'Point-LIO did not exit cleanly: {d}, code={front["process_returncode"]}')
                    spin(node,1)
                finally:
                    resources=monitor.finish(d)
                    if backend:stop(backend)
                result=dict(scenario=scenario,repeat=repeat,enabled=enabled,frontend=front,resources=resources)
                if observer:result['backend']=observer.finish(d,data)
                results.append(result);print(json.dumps(result),flush=True)
                (args.output/'summary.json').write_text(json.dumps(results,indent=2))


def synthetic(args,node):
    directory=args.output;proc=backend_process(directory,{'storage.directory':str(directory/'archive') if args.archive else '',
        'graph.max_states':args.max_states,'graph.lag_seconds':args.lag_seconds,'storage.cache_mb':args.cache_mb})
    observer=Observer(node);publisher=node.create_publisher(FrontendFrame,'/point_lio/backend_frame',qos_profile_sensor_data)
    monitor=Monitor(lambda:[proc.pid]);rng=np.random.default_rng(42)
    local=rng.uniform([-8,-8,-1],[8,8,3],size=(args.points,3)).astype('f4')
    expected={};sequence=0
    try:
        spin(node,2);start=time.monotonic()
        for i in range(round(args.duration*10)):
            sequence+=1;t=i*.1
            m=FrontendFrame();m.instance_id='benchmark';m.epoch=0;m.sequence=sequence
            m.odometry.header.stamp=startup.stamp(100+t);m.odometry.header.frame_id='camera_init';m.odometry.child_frame_id='body'
            # Smooth analytic motion + known drift. Backend should preserve it.
            R=Rotation.from_euler('ZYX',[.04*t,.1*np.sin(.3*t),.1*np.sin(.5*t)])
            p=np.array([.3*t+.01*t,.5*np.sin(.2*t),.1*np.sin(.4*t)])
            q=R.as_quat();pose=m.odometry.pose.pose;pose.position.x,pose.position.y,pose.position.z=map(float,p)
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,q)
            for k in range(6):m.odometry.pose.covariance[k*6+k]=.01
            m.lidar_updates=1;m.effective_points=200
            points=np.column_stack((R.apply(local)+p,np.ones(args.points))).astype('<f4')
            c=PointCloud2();c.header=m.odometry.header;c.height=1;c.width=args.points;c.point_step=16;c.row_step=args.points*16
            c.fields=[PointField(name=n,offset=j*4,datatype=7,count=1) for j,n in enumerate(['x','y','z','intensity'])]
            c.data=points.tobytes();c.is_dense=True;m.cloud=c
            publisher.publish(m)
            deadline=start+(i+1)*.1/args.speed
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.002)
            rclpy.spin_once(node,timeout_sec=.001)
        spin(node,1.2)
        result=observer.finish(directory);result['resources']=monitor.finish(directory,since=start)
        result.update(duration_sensor_s=args.duration,input_points=args.points,speed=args.speed,archive=args.archive)
        (directory/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    finally:
        monitor.done.set();monitor.thread.join();stop(proc);node.destroy_publisher(publisher)


def fullstack(args,node):
    results=[]
    for repeat in range(1,args.repetitions+1):
        # Alternate order to reduce warm-cache and temperature bias.
        for enabled in ([False,True] if repeat%2 else [True,False]):
            d=args.output/f'run_{repeat}_{"backend" if enabled else "baseline"}';d.mkdir(parents=True,exist_ok=True)
            config=yaml.safe_load((ROOT/'src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml').read_text())
            config['rm27_backend']['ros__parameters']['enabled']=enabled
            config['point_lio']['ros__parameters']['publish']['backend_frame']=enabled
            params=d/'params.yaml';params.write_text(yaml.safe_dump(config))
            log=(d/'launch.log').open('w');environment=dict(os.environ,GZ_PARTITION=f'rm27_backend_{os.getpid()}_{repeat}_{enabled}')
            proc=subprocess.Popen(['ros2','launch','rm_27_stimulation','sim_with_nav.launch.py',
                'world:=RMUC2027','gui:=false','use_rviz:=false','gimbal_panel:=false',
                'use_ground_truth_odom:=false','navigation_mode:=minco','params_file:='+str(params)],
                cwd=ROOT,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            monitor=Monitor(lambda:[proc.pid]);observer=Observer(node) if enabled else None
            truth_samples=[]
            def truth_callback(m):
                p=m.pose.pose.position
                truth_samples.append([startup.secs(m.header.stamp),p.x,p.y,p.z])
            truth_sub=node.create_subscription(Odometry,'/ground_truth/odometry',truth_callback,qos_profile_sensor_data)
            start=time.monotonic()
            goal_status=None
            action=None
            goal_handle=None
            try:
                spin(node,15)
                if args.navigate:
                    action=ActionClient(node,NavigateToPose,'navigate_to_pose')
                    if action.wait_for_server(timeout_sec=3):
                        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
                        goal.pose.pose.position.x=5.5;goal.pose.pose.position.y=4.9;goal.pose.pose.orientation.w=1.
                        future=action.send_goal_async(goal)
                        deadline=time.monotonic()+3
                        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
                        if future.done():goal_handle=future.result()
                        if goal_handle and goal_handle.accepted:
                            result_future=goal_handle.get_result_async()
                            goal_status='accepted'
                        else:goal_status='rejected'
                    else:goal_status='server_unavailable'
                spin(node,max(0,args.duration-(time.monotonic()-start)))
                if goal_status=='accepted' and result_future.done():goal_status=int(result_future.result().status)
                resources=monitor.finish(d,since=start+15)
                result=dict(repeat=repeat,enabled=enabled,resources=resources,duration_wall_s=args.duration,
                            workload='headless RMUC2027 navigation goal (5.5,4.9)' if args.navigate else 'headless RMUC2027 stationary full navigation',
                            goal_status=goal_status)
                if len(truth_samples)>1:
                    points=np.array(truth_samples)[:,1:]
                    result['truth_travel_m']=float(np.sum(np.linalg.norm(np.diff(points,axis=0),axis=1)))
                    result['truth_displacement_m']=float(np.linalg.norm(points[-1]-points[0]))
                    result['truth_sensor_span_s']=truth_samples[-1][0]-truth_samples[0][0]
                (d/'truth.json').write_text(json.dumps(truth_samples))
                if observer:result['backend']=observer.finish(d)
                results.append(result)
                (args.output/'summary.json').write_text(json.dumps(results,indent=2));print(json.dumps(result),flush=True)
            finally:
                if goal_handle and goal_handle.accepted:
                    goal_handle.cancel_goal_async();spin(node,.2)
                if action:action.destroy()
                node.destroy_subscription(truth_sub)
                monitor.done.set();monitor.thread.join();stop(proc,group=True);log.close();spin(node,1)


def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['pointlio','synthetic','fullstack']);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repetitions',type=int,default=3);p.add_argument('--duration',type=float,default=40)
    p.add_argument('--points',type=int,default=30000);p.add_argument('--speed',type=float,default=1)
    p.add_argument('--cache-mb',type=int,default=32);p.add_argument('--lag-seconds',type=float,default=30.)
    p.add_argument('--max-states',type=int,default=120)
    p.add_argument('--archive',action='store_true');p.add_argument('--scenarios',nargs='+',default=startup.SCENARIOS[:6])
    p.add_argument('--navigate',action='store_true')
    args=p.parse_args();args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=True)
    if not os.getenv('ROS_DOMAIN_ID'):raise RuntimeError('Use a private ROS_DOMAIN_ID')
    rclpy.init();node=rclpy.create_node('rm27_backend_benchmark')
    try:globals()[args.mode](args,node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
