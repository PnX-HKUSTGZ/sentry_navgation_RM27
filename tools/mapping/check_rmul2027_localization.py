#!/usr/bin/env python3
"""Verify the live Point-LIO/GICP chain against independent Gazebo ground truth."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rcl_interfaces.msg import Log
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('/tmp/rmul2027_localization_check.json'))
    parser.add_argument('--gimbal',action='store_true')
    args=parser.parse_args()
    if os.environ.get('GZ_PARTITION')!='rmul2027-map-validation':
        raise RuntimeError('Use the isolated rmul2027-map-validation simulation partition')
    rclpy.init()
    node=rclpy.create_node('check_rmul2027_localization',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer(); listener=TransformListener(buffer,node)
    state=dict(localized=False,acceptances=0,gimbal={})
    def log(message):
        if message.name!='relocalization_manager':return
        text=re.sub(r'\x1b\[[0-9;]*m','',message.msg)
        if 'GICP converge successfully' in text:state['acceptances']+=1
        if ' -> LOCALIZED ' in text or 'Localization state: LOCALIZED' in text:state['localized']=True
        if ' -> LOCALIZING ' in text:state['localized']=False
    node.create_subscription(Log,'/rosout',log,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/ground_truth/odometry',lambda m:state.update(truth=m),qos_profile_sensor_data)
    node.create_subscription(String,'/sim/gimbal/state',lambda m:state.update(gimbal=json.loads(m.data)),1)
    def scan_timing(message):
        offsets=read_points_numpy(message,field_names=('time',),skip_nans=True)
        if len(offsets):
            state['offset_min_s']=float(offsets.min())
            state['offset_max_s']=float(offsets.max())
    node.create_subscription(PointCloud2,'/velodyne_points',scan_timing,qos_profile_sensor_data)
    initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    command=node.create_publisher(Twist,'/cmd_vel',1)
    clients={key:node.create_client(SetBool,'/sim/gimbal/set_'+key) for key in ('upright','spin')}
    metrics=[]

    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)

    def wait(predicate,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise RuntimeError('Timed out waiting for localization / joint feedback')

    def call(key,value):
        assert clients[key].wait_for_service(timeout_sec=3)
        future=clients[key].call_async(SetBool.Request(data=value))
        wait(future.done,5)
        assert future.result().success,future.result().message

    def reset_with_offset(dx,dy,yaw):
        pose=state['truth'].pose.pose
        message=PoseWithCovarianceStamped()
        message.header.frame_id='map';message.header.stamp=node.get_clock().now().to_msg()
        message.pose.pose.position.x=pose.position.x+dx
        message.pose.pose.position.y=pose.position.y+dy
        message.pose.pose.position.z=pose.position.z+.02
        q=pose.orientation
        rotation=Rotation.from_euler('z',yaw)*Rotation.from_quat([q.x,q.y,q.z,q.w])
        target=message.pose.pose.orientation
        target.x,target.y,target.z,target.w=rotation.as_quat().tolist()
        state['localized']=False;state['acceptances']=0
        initial.publish(message)
        wait(lambda:state['localized'] and state['acceptances']>=3)
        spin(.5)

    def measure(label):
        errors=[]
        for _ in range(12):
            spin(.05)
            pose=state['truth'].pose.pose
            transform=buffer.lookup_transform('map','base_footprint',Time()).transform
            p,q=transform.translation,transform.rotation
            error=np.array([p.x-pose.position.x,p.y-pose.position.y,p.z-pose.position.z])
            truth=pose.orientation
            angle=(Rotation.from_quat([q.x,q.y,q.z,q.w]).inv()*
                   Rotation.from_quat([truth.x,truth.y,truth.z,truth.w])).magnitude()
            errors.append([float(np.linalg.norm(error[:2])),abs(float(error[2])),float(np.degrees(angle))])
        median=np.median(errors,axis=0)
        result=dict(stage=label,xy_error_m=float(median[0]),z_error_m=float(median[1]),
                    orientation_error_deg=float(median[2]),accepted_registrations=state['acceptances'])
        metrics.append(result);print(json.dumps(result),flush=True)
        assert median[0]<.05 and median[1]<.03 and median[2]<2.0,result

    try:
        wait(lambda:'truth' in state and buffer.can_transform('map','base_footprint',Time()))
        wait(lambda:state['localized'])
        spin(1)
        assert 0<state['offset_min_s']<=state['offset_max_s']<1e-8,state
        measure('initial')
        reset_with_offset(.15,-.10,math.radians(5))
        measure('perturbed_initial_pose')
        for vx,vy,label in [(.20,0,'after_forward_motion'),(0,.20,'after_lateral_motion')]:
            state['localized']=False;state['acceptances']=0
            deadline=time.monotonic()+3.0
            while time.monotonic()<deadline:
                msg=Twist();msg.linear.x=float(vx);msg.linear.y=float(vy);command.publish(msg);spin(.05)
            command.publish(Twist())
            wait(lambda:state['localized'] and state['acceptances']>=3)
            spin(.5);measure(label)
        if args.gimbal:
            call('upright',True);wait(lambda:state['gimbal'].get('upright'))
            call('spin',True);spin(3)
            reset_with_offset(.10,-.10,math.radians(-3))
            measure('rotating_gimbal')
        args.output.write_text(json.dumps(dict(passed=True,measurements=metrics,
            point_time_offset_s=[state['offset_min_s'],state['offset_max_s']]),indent=2)+'\n')
    finally:
        command.publish(Twist())
        if args.gimbal and clients['upright'].service_is_ready():
            call('upright',False)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
