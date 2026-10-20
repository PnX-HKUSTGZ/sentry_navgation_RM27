#!/usr/bin/env python3
"""One isolated real-simulation competition trial, with observation-only GT scoring."""
import argparse
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import psutil
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from global_relocalization_interfaces.action import Relocalize
from odometry_probe import matrix, interpolate
from reset_pose import reset_pose


class Observe(Node):
    def __init__(self):
        super().__init__('competition_trial_observer')
        self.gt, self.imu, self.states, self.commands = [], [], [], []
        self.create_subscription(Odometry, '/ground_truth/odometry', lambda m: self.gt.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,matrix(m.pose.pose))), qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', lambda m: self.imu.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'omega':math.sqrt(m.angular_velocity.x**2+m.angular_velocity.y**2+m.angular_velocity.z**2)}), qos_profile_sensor_data)
        self.create_subscription(String, '/global_relocalization/status', self.status, 100)
        self.create_subscription(Twist, '/cmd_vel', lambda m:self.commands.append({'wall':time.monotonic(),'omega':m.angular.z,'linear':[m.linear.x,m.linear.y,m.linear.z]}), 100)
        self.action=ActionClient(self,Relocalize,'/global_relocalization/run')
        self.trigger=self.create_client(Trigger,'/global_relocalization/start_match')

    def status(self,m):
        self.states.append({'received_wall':time.monotonic(),**json.loads(m.data)})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('node','params','map','manifest','output','trial'):p.add_argument('--'+name,required=True)
    p.add_argument('--trials',default=str(Path(__file__).parent/'config/trials_left_physical_120.json'))
    p.add_argument('--early',action='store_true')
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    if hashlib.sha256(Path(a.map).read_bytes()).hexdigest()!=json.loads(Path(a.manifest).read_text())['map_sha256']:raise ValueError('frozen map hash mismatch')
    trial=next(t for t in json.loads(Path(a.trials).read_text())['trials'] if t['trial_id']==a.trial)
    reset_pose(trial['requested_pose'],log_path=out/'reset_attempts.json')
    command=[a.node,'--ros-args','--params-file',a.params,'-p','map_path:='+str(Path(a.map).resolve()),'-p','manifest_path:='+str(Path(a.manifest).resolve()),'-p','visualization_enabled:=false']
    log=(out/'node.log').open('w');process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close()
    rclpy.init();node=Observe();cost=psutil.Process(process.pid);cpu0=cost.cpu_times();peak=0;started=time.monotonic()
    def spin():
        nonlocal peak
        rclpy.spin_once(node,timeout_sec=.02)
        try:peak=max(peak,cost.memory_info().rss)
        except psutil.NoSuchProcess:pass
    def wait(future,timeout=150):
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('node exited; inspect node.log')
            spin()
        if not future.done():raise TimeoutError('future deadline exceeded')
        return future.result()
    paths=[Path(a.node),Path(a.node).parents[1]/'libglobal_relocalization_core.so',Path(a.params),Path(a.map),Path(a.manifest),Path(a.trials),Path(__file__)]
    provenance={str(path.resolve()):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    result={'sha256':provenance,**trial,'early_trigger_test':a.early,'command':command,'environment':{k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','GZ_PARTITION')},'gt_used_by_estimator':False}
    try:
        if not node.action.wait_for_server(timeout_sec=15) or not node.trigger.wait_for_service(timeout_sec=5):raise RuntimeError('server unavailable')
        mappings=Path('/proc/'+str(process.pid)+'/maps').read_text()
        (out/'node_maps.txt').write_text(mappings)
        expected_library=(Path(a.node).parents[1]/'libglobal_relocalization_core.so').resolve()
        if str(expected_library) not in mappings:raise RuntimeError('node loaded unexpected core library')
        audit_command=['/usr/bin/python3',str(Path(__file__).parent/'audit_runtime_parameters.py'),'--params',a.params,'--output',str(out/'runtime_parameters.json'),'--timeout','10']
        audit=subprocess.run(audit_command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=15)
        (out/'runtime_parameters.log').write_text(audit.stdout)
        if audit.returncode:raise RuntimeError('runtime parameter audit failed')
        until=time.monotonic()+2
        while time.monotonic()<until:spin()
        action_started=time.monotonic();cpu0=cost.cpu_times();peak=cost.memory_info().rss
        handle=wait(node.action.send_goal_async(Relocalize.Goal()),10)
        if not handle.accepted:raise RuntimeError('goal rejected')
        future=handle.get_result_async();triggered=False;trigger_wall=None;trigger_response=None;armed_wall=None;deadline=time.monotonic()+140
        while not future.done() and time.monotonic()<deadline:
            spin()
            if armed_wall is None and any(s.get('stage')=='ARMED' for s in node.states):armed_wall=time.monotonic()
            if not triggered and (a.early or (armed_wall is not None and time.monotonic()-armed_wall>=.3)):
                triggered=True;trigger_wall=time.monotonic();response=wait(node.trigger.call_async(Trigger.Request()),5);trigger_response={'success':response.success,'message':response.message}
        response=wait(future,2);pose=response.result.pose;stamp=pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9
        result.update(action={'status':response.status,'success':response.result.success,'reason':response.result.reason,'pose_stamp_sec':stamp,'fitness':response.result.fitness,'inlier_ratio':response.result.inlier_ratio},trigger=trigger_response,trigger_wall=trigger_wall,action_finished_wall=time.monotonic())
        if response.result.success:
            # Obtain a future GT bracket; this is observation only, outside latency measurement.
            deadline=time.monotonic()+1
            while node.gt and node.gt[-1][0]<stamp and time.monotonic()<deadline:spin()
            truth=interpolate(node.gt,stamp,.25)
            if truth is None:result['score']={'valid':False,'reason':'missing GT bracket'}
            else:
                estimate=matrix(pose.pose);map_world=np.array(json.loads(Path(a.manifest).read_text())['T_map_world']);truth=map_world@truth
                distance=float(np.linalg.norm(estimate[:3,3]-truth[:3,3]));angle=float(np.degrees(Rotation.from_matrix(truth[:3,:3].T@estimate[:3,:3]).magnitude()))
                result['score']={'valid':True,'translation_3d_m':distance,'orientation_so3_deg':angle,'strict_correct':distance<=.1 and angle<=2}
        terminal_wall=time.monotonic()
        until=terminal_wall+1
        while time.monotonic()<until:spin()
        result['commands_after_terminal_grace']=[c for c in node.commands if c['wall']>terminal_wall+.2]
        result['terminal_status']=node.states[-1] if node.states else None
        spin_stamps=[s.get('status_ros_ns',0)*1e-9 for s in node.states if s.get('stage')=='SPINNING']
        trigger_ros=(result['terminal_status'] or {}).get('competition',{}).get('trigger_ros_sec')
        if spin_stamps:
            begin=min(spin_stamps);end=trigger_ros if trigger_ros and trigger_ros>0 else float('inf')
            rows=[(stamp,pose) for stamp,pose in node.gt if begin<=stamp<=end]
            accumulated=0.;full_turn=None;rates=[]
            for (ta,pa),(tb,pb) in zip(rows,rows[1:]):
                angle=float(Rotation.from_matrix(pa[:3,:3].T@pb[:3,:3]).magnitude())
                if tb>ta:rates.append(angle/(tb-ta))
                if full_turn is None and accumulated<2*math.pi<=accumulated+angle:
                    full_turn=ta+(2*math.pi-accumulated)/angle*(tb-ta)-rows[0][0]
                accumulated+=angle
            omegas=[r['omega'] for r in node.imu if begin<=r['stamp']<=end]
            result['physical_rotation']={'gt_accumulated_so3_rad':accumulated,'measured_first_full_turn_sec':full_turn,'gt_pose_rate_median_rad_sec':float(np.median(rates)) if rates else None,'imu_omega_median_rad_sec':float(np.median(omegas)) if omegas else None,'gt_max_translation_m':max((float(np.linalg.norm(row[1][:3,3]-rows[0][1][:3,3])) for row in rows),default=None)}
        result['wall_sec']=time.monotonic()-started
        cpu1=cost.cpu_times();resource_wall=time.monotonic()-action_started
        result['algorithm_resource']={'window':'goal submission through terminal plus one second passive observation','wall_sec':resource_wall,'cpu_core_seconds':cpu1.user+cpu1.system-cpu0.user-cpu0.system,'cpu_percent':100*(cpu1.user+cpu1.system-cpu0.user-cpu0.system)/resource_wall,'peak_rss_bytes':peak}
    except Exception as e:result['exception']=repr(e)
    finally:
        result['status_events']=node.states;result['commands']=node.commands;result['imu']=node.imu
        result['gt']=[{'stamp':t,'matrix':m.tolist()} for t,m in node.gt]
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()
    print(json.dumps({k:v for k,v in result.items() if k not in ('gt','imu','commands','status_events')},indent=2))
    return 0 if result.get('action',{}).get('success') or (a.early and result.get('trigger',{}).get('success') is False) else 1

if __name__=='__main__':raise SystemExit(main())
