#!/usr/bin/env python3
"""Run one empty Relocalize goal and persist every outcome and process cost."""
import argparse
import json
import math
import os
from pathlib import Path
import platform
import time
import importlib.util

_repo_root=next(p for p in Path(__file__).resolve().parents if (p/'tools/global_relocalization/odometry_probe.py').exists())
_motion_spec=importlib.util.spec_from_file_location('relocalization_motion_evaluation',_repo_root/'tools/global_relocalization/odometry_probe.py')
_motion=importlib.util.module_from_spec(_motion_spec)
_motion_spec.loader.exec_module(_motion)

import psutil
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Imu
from global_relocalization_interfaces.action import Relocalize


class ProcessCost:
    def __init__(self, predicate):
        self.predicate = predicate
        self.baseline = {}
        self.last = {}
        self.peak = 0
        self.start = time.monotonic()

    def sample(self):
        rss = 0
        for p in psutil.process_iter(['pid','cmdline','memory_info','cpu_times']):
            try:
                if not self.predicate(p.info['cmdline'] or []):
                    continue
                environment=p.environ()
                if environment.get('ROS_DOMAIN_ID') != os.environ.get('ROS_DOMAIN_ID') or environment.get('GZ_PARTITION') != os.environ.get('GZ_PARTITION'):
                    continue
                pid, cpu = p.pid, p.info['cpu_times']
                self.baseline.setdefault(pid, (cpu.user,cpu.system))
                self.last[pid] = (cpu.user,cpu.system)
                rss += p.info['memory_info'].rss
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self.peak = max(self.peak, rss)

    def result(self):
        self.sample()
        if not self.baseline:
            return {'pids': [], 'measurement_valid': False, 'cpu_user_delta_sec': None, 'cpu_sys_delta_sec': None,
                'wall_sec': time.monotonic()-self.start, 'cpu_percent': None, 'core_seconds': None, 'peak_rss_bytes': None}
        user = sum(v[0]-self.baseline[k][0] for k,v in self.last.items())
        system = sum(v[1]-self.baseline[k][1] for k,v in self.last.items())
        wall = time.monotonic()-self.start
        return {'pids': sorted(self.baseline), 'measurement_valid': True, 'cpu_user_delta_sec':user, 'cpu_sys_delta_sec':system,
            'wall_sec':wall,'cpu_percent':(user+system)/wall*100, 'core_seconds':user+system,'peak_rss_bytes':self.peak}


class Trial(Node):
    def __init__(self, args):
        super().__init__('global_relocalization_trial_client')
        self.args = args
        self.client = ActionClient(self, Relocalize, '/global_relocalization/run')
        self.clock_ns = 0
        self.odom_count = 0
        self.raw_counts = {'lidar': 0, 'imu': 0}
        self.raw_received = {'lidar': None, 'imu': None}
        self.events = []
        self.graph_fault = ''
        self.motion_rows={'lio':[],'gt':[]}
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry', self.odom, 10)
        if args.input_mode == 'raw_imu':
            self.create_subscription(PointCloud2, args.raw_cloud_topic, lambda m: self.raw_input('lidar'), qos_profile_sensor_data)
            self.create_subscription(Imu, args.raw_imu_topic, lambda m: self.raw_input('imu'), qos_profile_sensor_data)
        # Evaluation side channel only: these samples never affect readiness,
        # goal contents, motion commands, or localization hypotheses.
        self.create_subscription(Odometry, '/ground_truth/odometry', self.gt, qos_profile_sensor_data)
        self.create_subscription(String, '/global_relocalization/status', self.status, 10)
        self.algorithm = ProcessCost(lambda c: bool(c) and Path(c[0]).name == 'global_relocalization_node')
        self.gazebo = ProcessCost(lambda c: bool(c) and (c[0].startswith('gz sim ') or 'gz-sim-server' in c[0] or (Path(c[0]).name=='gz' and len(c)>1 and c[1]=='sim')))
        self.lio = ProcessCost(lambda c: args.input_mode == 'lio' and bool(c) and any(Path(a).name in ['pointlio_mapping','loam_interface_node','sensor_scan_generation_node','instantaneous_cloud_converter.py'] for a in c))
        self.evaluator = ProcessCost(lambda c: bool(c) and any(Path(a).name=='run_trial.py' for a in c))
        self.recorder = ProcessCost(lambda c: bool(c) and (('bag' in c and 'record' in c) or
            any(Path(a).name == 'passive_receipt_probe.py' for a in c)))
        self.create_timer(.2, self.sample)

    def sample(self):
        self.algorithm.sample()
        self.gazebo.sample()
        self.lio.sample()
        self.evaluator.sample()
        self.recorder.sample()
        topics=['/clock','/ground_truth/odometry','/cmd_vel']
        if self.args.input_mode == 'raw_imu':
            topics += [self.args.raw_cloud_topic, self.args.raw_imu_topic]
        for topic in topics:
            if self.count_publishers(topic)>1:
                self.graph_fault=f'multiple publishers on {topic}; isolated simulation violated'

    def clock(self,msg):
        stamp=msg.clock.sec*1000000000+msg.clock.nanosec
        if self.clock_ns and stamp<self.clock_ns:
            self.graph_fault='simulation clock moved backward during trial'
        self.clock_ns = stamp

    def odom(self,msg):
        self.odom_count += 1
        self.add_motion('lio',msg)

    def raw_input(self, key):
        self.raw_counts[key] += 1
        self.raw_received[key] = time.monotonic()

    def inputs_ready(self):
        if self.args.input_mode == 'lio':
            return self.odom_count >= 5
        now = time.monotonic()
        return self.raw_counts['lidar'] >= 2 and self.raw_counts['imu'] >= 10 and all(
            stamp is not None and now - stamp <= 1.5 for stamp in self.raw_received.values())

    def gt(self,msg):
        self.add_motion('gt',msg)

    def add_motion(self,key,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not self.motion_rows[key] or stamp>self.motion_rows[key][-1][0]:
            self.motion_rows[key].append((stamp,_motion.matrix(msg.pose.pose)))

    def evaluate_motion(self,start_ns,end_ns):
        import numpy as np
        from scipy.spatial.transform import Rotation
        rows={key:[r for r in values if start_ns*1e-9<=r[0]<=end_ns*1e-9]
            for key,values in self.motion_rows.items()}
        result={'ground_truth_used_for_estimation_or_control':False,
            'samples':{key:len(values) for key,values in rows.items()},
            'lio_relative_motion':{k:v for k,v in _motion.score(rows,.25).items() if k!='errors'}}
        gt=rows['gt']
        if gt:
            positions=np.asarray([pose[:3,3] for stamp,pose in gt])
            angles=Rotation.from_matrix([pose[:3,:3] for stamp,pose in gt]).as_euler('xyz',degrees=True)
            yaw_rad=np.unwrap(np.radians(angles[:,2]))
            result['gt']={'start_ros':gt[0][0],'end_ros':gt[-1][0],
                'start_position':positions[0].tolist(),'end_position':positions[-1].tolist(),
                'max_translation_from_start_m':float(np.linalg.norm(positions-positions[0],axis=1).max()),
                'max_abs_roll_deg':float(np.abs(angles[:,0]).max()),
                'max_abs_pitch_deg':float(np.abs(angles[:,1]).max()),
                'height_range_m':float(np.ptp(positions[:,2])),
                'accumulated_yaw_deg':float(np.degrees(yaw_rad[-1]-yaw_rad[0]))}
        else:
            result['gt']=None
        return result

    def status(self,msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            data = {'stage':'unparsed','text':msg.data}
        self.events.append({'wall':time.monotonic(),'ros_ns':self.clock_ns,**data})

    def wait(self,predicate,deadline):
        while rclpy.ok() and not predicate() and not self.graph_fault and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.1)
        return predicate()

    def run(self):
        preparation_start = time.monotonic()
        deadline = preparation_start+self.args.ready_timeout
        ready = self.wait(lambda:self.inputs_ready() and self.clock_ns and self.client.server_is_ready(),deadline)
        readiness={'clock_received':bool(self.clock_ns),'action_server_ready':self.client.server_is_ready(),
            'inputs_ready':self.inputs_ready(),'unmet':[]}
        if not readiness['clock_received']:
            readiness['unmet'].append('clock')
        if not readiness['action_server_ready']:
            readiness['unmet'].append('action_server')
        if self.args.input_mode == 'raw_imu':
            readiness['raw_sources']={key:{'count':self.raw_counts[key],
                'wall_age_sec':time.monotonic()-stamp if stamp is not None else None}
                for key,stamp in self.raw_received.items()}
            for key,minimum in [('lidar',2),('imu',10)]:
                age=readiness['raw_sources'][key]['wall_age_sec']
                if self.raw_counts[key]<minimum or age is None or age>1.5:
                    readiness['unmet'].append('raw_'+key)
        elif not readiness['inputs_ready']:
            readiness['unmet'].append('lio_odometry')
        started, ros_start = time.monotonic(), self.clock_ns
        raw_counts_at_start = dict(self.raw_counts)
        self.algorithm = ProcessCost(self.algorithm.predicate)
        self.gazebo = ProcessCost(self.gazebo.predicate)
        self.lio = ProcessCost(self.lio.predicate)
        self.evaluator = ProcessCost(self.evaluator.predicate)
        self.recorder = ProcessCost(self.recorder.predicate)
        self.sample()
        success, reason, timeout, pose, metrics = False, 'pipeline readiness timeout', not ready, None, {}
        timeout_source = 'readiness:'+','.join(readiness['unmet']) if not ready else None
        goal_started_wall = None
        goal_started_ros_ns = None
        if not ready:
            reason='pipeline readiness timeout: '+','.join(readiness['unmet'])
        if ready:
            # This timestamp begins once for the original goal and is never
            # restarted for internal localization recovery attempts.
            goal_started_wall = time.monotonic()
            goal_started_ros_ns = self.clock_ns
            goal_future = self.client.send_goal_async(Relocalize.Goal())
            deadline = goal_started_wall+self.args.timeout
            if self.wait(goal_future.done,deadline):
                handle = goal_future.result()
                if handle.accepted:
                    future = handle.get_result_async()
                    if self.wait(future.done,deadline):
                        result = future.result().result
                        success,reason = result.success,result.reason
                        timeout = (not success and 'timeout' in reason.lower())
                        timeout_source = 'server' if timeout else None
                        metrics = {'fitness':result.fitness,'inlier_ratio':result.inlier_ratio,'elapsed_sec':result.elapsed_sec}
                        metrics={k:(v if math.isfinite(v) else None) for k,v in metrics.items()}
                        p = result.pose
                        if success:
                            pose = {'frame_id':p.header.frame_id,'stamp_ns':p.header.stamp.sec*1000000000+p.header.stamp.nanosec,
                                'position':{'x':p.pose.position.x,'y':p.pose.position.y,'z':p.pose.position.z},
                                'orientation':{'x':p.pose.orientation.x,'y':p.pose.orientation.y,'z':p.pose.orientation.z,'w':p.pose.orientation.w}}
                    else:
                        timeout,reason=True,'action wall timeout'
                        timeout_source = 'client_action_wall'
                        handle.cancel_goal_async()
                        self.wait(lambda:False,time.monotonic()+1)
                else:
                    reason='goal rejected'
            else:
                timeout,reason=True,'goal response wall timeout'
                timeout_source = 'client_goal_response_wall'
        if self.graph_fault:
            success,reason,timeout,pose=False,self.graph_fault,False,None
            timeout_source = None
        finished = time.monotonic()
        ros_finished=self.clock_ns
        algorithm_cost,gazebo_cost,lio_cost=self.algorithm.result(),self.gazebo.result(),self.lio.result()
        evaluator_cost=self.evaluator.result()
        recorder_cost=self.recorder.result()
        if not ready:
            for cost in [algorithm_cost,gazebo_cost,lio_cost,evaluator_cost,recorder_cost]:
                cost.update({'measurement_valid':False,'reason':'no action window: readiness failed',
                    'cpu_user_delta_sec':None,'cpu_sys_delta_sec':None,'wall_sec':None,
                    'cpu_percent':None,'core_seconds':None,'peak_rss_bytes':None})
        if self.args.input_mode == 'raw_imu':
            lio_cost['reason'] = 'not_applicable: Point-LIO disabled'
        # Status and action results use different DDS topics. Drain the terminal
        # status after freezing the action/resource window so its stage metrics survive.
        if ready and not self.graph_fault:
            self.wait(lambda:any(e['wall']>=started and str(e.get('stage','')).upper() in
                ['SUCCEEDED','FAILED','CANCELED'] for e in self.events),time.monotonic()+.5)
        timing = {'total_ros':(ros_finished-ros_start)*1e-9 if ros_start and ros_finished else None,'total_wall':finished-started,
                  'goal_to_client_finish_wall':finished-goal_started_wall if goal_started_wall is not None else None,
                  'pipeline_ready_wall':started-preparation_start}
        stage_names={'settling':'static','settle':'static','static':'static','spinning':'sweep','sweeping':'sweep','spin':'sweep','sweep':'sweep',
                     'retrieving':'retrieval','retrieval':'retrieval','refining':'gicp','gicp':'gicp','confirming':'confirmation','confirmation':'confirmation','stopping':'stop_settle','searching':'search_total'}
        for name in ['static','sweep','retrieval','gicp','confirmation','stop_settle','search_total']:
            timing[name]={'ros':None,'wall':None}
        events=[e for e in self.events if e['wall']>=started]
        for i,event in enumerate(events):
            name=stage_names.get(str(event.get('stage','')).lower())
            end=events[i+1] if i+1<len(events) else {'wall':finished,'ros_ns':ros_finished}
            if name:
                timing[name]['wall'] = (timing[name]['wall'] or 0.) + max(0,min(finished,end['wall'])-min(finished,event['wall']))
                timing[name]['ros'] = (timing[name]['ros'] or 0.) + max(0,(min(ros_finished,end['ros_ns'])-min(ros_finished,event['ros_ns']))*1e-9)
        for event in events:
            for field,unit in [('stage_wall_sec','wall'),('stage_ros_sec','ros')]:
                for stage,value in event.get(field,{}).items():
                    name=stage_names.get(stage.lower())
                    if name and isinstance(value,(int,float)) and math.isfinite(value) and value>=0:
                        timing[name][unit]=value
            for stage in ['retrieval','gicp','confirmation']:
                value=event.get('metrics',{}).get(stage+'_sec')
                if isinstance(value,(int,float)) and math.isfinite(value) and value>0:
                    timing[stage]['wall']=value
        row={'run_id':self.args.run_id,'repetition':self.args.repetition,
            'trial_id':self.args.trial_id,'trial_index':self.args.trial_index,
            'source_trial_index':self.args.source_trial_index if self.args.source_trial_index is not None else self.args.trial_index,
            'requested_pose':json.loads(self.args.requested_pose),'initial_stamp_ns':ros_start,'final_stamp_ns':ros_finished,
            'action_goal_start_ros_ns':goal_started_ros_ns,
            'action_goal_started_wall_monotonic_sec':goal_started_wall,
            'input_mode':self.args.input_mode,
            'action_window_exists':bool(ready),
            'readiness_observation':readiness,
            'action':{'success':success,'reason':reason,'timeout':timeout,'timeout_source':timeout_source,'pose_stamp_ns':pose['stamp_ns'] if pose else None,'pose':pose,**metrics},
            'timing_sec':timing,'algorithm_process':algorithm_cost,'gazebo_process_load':gazebo_cost,'lio_pipeline':lio_cost,
            'trial_evaluator_process':evaluator_cost,
            'diagnostic_recorder_process':recorder_cost,
            'frontend_process':{'measurement_valid':False,'pids':[], 'cpu_percent':None,'peak_rss_bytes':None,
                'cpu_user_delta_sec':None,'cpu_sys_delta_sec':None,'wall_sec':None,'core_seconds':None,
                'included_in_algorithm':self.args.input_mode == 'raw_imu',
                'reason':'raw frontend runs within algorithm PID' if self.args.input_mode == 'raw_imu' else 'legacy LIO measured in lio_pipeline'},
            'raw_input_observation':{'counts_since_client_start':self.raw_counts,
                'counts_during_action':{key:self.raw_counts[key]-raw_counts_at_start[key] for key in self.raw_counts},
                'readiness_used_ground_truth':False},
            'rtf':timing['total_ros']/max(timing['total_wall'],1e-9) if timing['total_ros'] is not None else None,'status_events':events,
            'motion_evaluation':self.evaluate_motion(ros_start,ros_finished),
            'environment':{'hostname':platform.node(),'platform':platform.platform(),'cpu_count':psutil.cpu_count(),
                'physical_cpu_count':psutil.cpu_count(logical=False),'total_memory_bytes':psutil.virtual_memory().total,
                'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'gz_partition':os.environ.get('GZ_PARTITION'),
                'gazebo_gui':False}}
        Path(self.args.output).write_text(json.dumps(row,indent=2)+'\n')
        print(json.dumps({'trial_id':self.args.trial_id,'success':success,'reason':reason,'timing_sec':timing}),flush=True)
        return 0 if success else 2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--trial-id',default='pilot_000')
    p.add_argument('--trial-index',type=int,default=0)
    p.add_argument('--source-trial-index',type=int)
    p.add_argument('--run-id')
    p.add_argument('--repetition',type=int)
    p.add_argument('--requested-pose',default='{}')
    p.add_argument('--ready-timeout',type=float,default=90)
    p.add_argument('--timeout',type=float,default=180)
    p.add_argument('--input-mode', choices=['lio','raw_imu'], default='lio')
    p.add_argument('--raw-cloud-topic', default='/livox/lidar')
    p.add_argument('--raw-imu-topic', default='/livox/imu')
    args=p.parse_args()
    if (args.run_id is None) != (args.repetition is None):
        p.error('--run-id and --repetition must be supplied together')
    if args.run_id is not None and (not args.run_id.strip() or args.repetition <= 0):
        p.error('--run-id must be non-empty and --repetition must be positive')
    if args.source_trial_index is not None and args.source_trial_index < 0:
        p.error('--source-trial-index must be non-negative')
    rclpy.init(args=[])
    node=Trial(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
