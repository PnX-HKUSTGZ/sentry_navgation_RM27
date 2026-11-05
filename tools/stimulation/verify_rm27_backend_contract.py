#!/usr/bin/env python3
"""Exercise atomic-input rejection and reset boundaries through the ROS node."""
import argparse
import copy
import json
import os
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from rm27_localization_interfaces.msg import FrontendFrame
from rclpy.qos import qos_profile_sensor_data
from benchmark_rm27_backend import backend_process, spin, stop, startup


def frame(instance,epoch,sequence,t,x):
    m=FrontendFrame();m.instance_id=instance;m.epoch=epoch;m.sequence=sequence
    o=m.odometry;o.header.stamp=startup.stamp(t);o.header.frame_id='camera_init';o.child_frame_id='body'
    o.pose.pose.position.x=float(x);o.pose.pose.orientation.w=1.
    for i in range(6):o.pose.covariance[i*6+i]=.01
    m.effective_points=100;m.lidar_updates=1
    points=np.array([[x+i*.2,j*.2,1.,1.] for i in range(10) for j in range(10)],dtype='<f4')
    c=PointCloud2();c.header=copy.deepcopy(o.header);c.width=len(points);c.height=1;c.point_step=16;c.row_step=16*len(points)
    c.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z','intensity'])]
    c.data=points.tobytes();m.cloud=c;return m


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID'):raise RuntimeError('Private ROS_DOMAIN_ID required')
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('backend_contract_test');rows=[];diagnostics=[];tf=[]
    node.create_subscription(Odometry,'/rm27_backend/odometry',lambda m:rows.append(m),100)
    node.create_subscription(DiagnosticArray,'/rm27_backend/diagnostics',lambda m:diagnostics.extend(m.status),100)
    node.create_subscription(TFMessage,'/tf',lambda m:tf.append(m),qos_profile_sensor_data)
    publisher=node.create_publisher(FrontendFrame,'/point_lio/backend_frame',qos_profile_sensor_data)
    proc=backend_process(args.output,{'keyframe.min_interval':0.,'keyframe.max_interval':.1})
    try:
        spin(node,2)
        packets=[frame('A',0,1,100,0),frame('A',0,1,100,0),frame('A',0,2,100.1,.1)]
        wrong_stamp=frame('A',0,3,100.2,.2);wrong_stamp.cloud.header.stamp.nanosec+=1;packets.append(wrong_stamp)
        packets.extend([frame('A',0,3,100.2,.2),frame('A',0,2,100.1,.1),frame('A',1,1,1,0),
            frame('A',0,4,100.3,.3),frame('A',1,2,1.1,.1),frame('B',0,1,2,0),
            frame('A',1,999,200,1),frame('B',0,2,3.5,.1),frame('B',0,3,3.6,100)])
        malformed=frame('B',0,4,3.7,100.1);malformed.cloud.data=b'';packets.append(malformed)
        packets.append(frame('B',0,4,3.7,100.1))
        for packet in packets:publisher.publish(packet);spin(node,.15)
        spin(node,1.2)
        values={v.key:v.value for v in diagnostics[-1].values}
        report=dict(sent=len(packets),outputs=len(rows),frames=sorted({m.header.frame_id for m in rows}),
                    tf_messages=len(tf),diagnostics=values)
        (args.output/'result.json').write_text(json.dumps(report,indent=2))
        assert len(rows)==9,report
        assert len(report['frames'])==5,report
        assert int(values['rejected'])==6,report
        assert int(values['graph_failures'])==0,report
        assert int(values['active_states'])<=2,report
        assert not tf,report
        print(json.dumps(report),flush=True)
    finally:
        stop(proc);node.destroy_node();rclpy.shutdown()
    assert proc.returncode==0,proc.returncode


if __name__=='__main__':main()
