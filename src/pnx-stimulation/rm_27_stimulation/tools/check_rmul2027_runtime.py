#!/usr/bin/env python3
"""Check an isolated RMUL2027 world: ground contacts, lidar and chassis motion."""

import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy


def support(triangles, x, y):
    a,b,c = triangles[:,0,:],triangles[:,1,:],triangles[:,2,:]
    ab,ac = b-a,c-a
    determinant = ab[:,0]*ac[:,1]-ab[:,1]*ac[:,0]
    valid = np.abs(determinant)>1e-12
    denominator = np.where(valid,determinant,1)
    dx,dy = x-a[:,0],y-a[:,1]
    u = (dx*ac[:,1]-dy*ac[:,0])/denominator
    v = (ab[:,0]*dy-ab[:,1]*dx)/denominator
    valid &= (u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)
    indices = np.flatnonzero(valid)
    if not len(indices):
        raise RuntimeError(f'No CAD support at {(x,y)}')
    height = a[:,2]+u*ab[:,2]+v*ac[:,2]
    i = indices[np.argmax(height[indices])]
    normal = np.cross(ab[i],ac[i]); normal /= np.linalg.norm(normal)
    return float(height[i]),abs(float(normal[2]))


def service(name, request_type, request):
    result = subprocess.run(['gz','service','-s','/world/RMUL2027/'+name,
                             '--reqtype',request_type,'--reptype','gz.msgs.Boolean',
                             '--timeout','5000','--req',request],capture_output=True,text=True,timeout=8)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError(result.stdout+result.stderr)


def probe(name,x,y,radius=.025):
    root = ET.Element('sdf',version='1.9')
    model = ET.SubElement(root,'model',name=name)
    ET.SubElement(model,'pose').text = f'{x} {y} 1.0 0 0 0'
    link = ET.SubElement(model,'link',name='body')
    inertial = ET.SubElement(link,'inertial')
    ET.SubElement(inertial,'mass').text = '.1'
    inertia = ET.SubElement(inertial,'inertia')
    for axis in ('ixx','iyy','izz'):
        ET.SubElement(inertia,axis).text = str(.4*.1*radius*radius)
    collision = ET.SubElement(link,'collision',name='sphere')
    sphere = ET.SubElement(ET.SubElement(collision,'geometry'),'sphere')
    ET.SubElement(sphere,'radius').text = str(radius)
    plugin = ET.SubElement(model,'plugin',filename='librm27_ground_truth_odometry_system.so',
                           name='rm_27_stimulation::GroundTruthOdometrySystem')
    ET.SubElement(plugin,'odom_topic').text = '/rmul2027/probes/odometry'
    ET.SubElement(plugin,'robot_base_frame').text = name
    ET.SubElement(plugin,'odom_publish_frequency').text = '20'
    service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(ET.tostring(root,encoding='unicode')))


def mesh_probe_positions(assets):
    candidates = []
    for collision in ET.parse(assets/'RMUL2027_collision/model.sdf').findall('.//collision'):
        shape = collision.find('geometry/mesh')
        if shape is None:
            continue
        pose = np.fromstring(collision.findtext('pose'),sep=' ')
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        if abs(rotation[2,2]) < .999:
            continue
        scale = np.fromstring(shape.findtext('scale'),sep=' ')
        triangle = np.array([[0,0,0],[scale[0],0,0],[0,scale[1],0]])@rotation.T+pose[:3]
        point = triangle.mean(axis=0)
        distances = [np.linalg.norm(np.cross(b-a,point-a))/np.linalg.norm(b-a)
                     for a,b in zip(triangle,np.roll(triangle,-1,axis=0))]
        if min(distances) > .008 and point[2]+scale[2]/2 > .02:
            candidates.append((scale[0]*scale[1],point[:2]))
    return [xy for _,xy in sorted(candidates,key=lambda item:-item[0])[:4]]


def main():
    if os.environ.get('GZ_PARTITION') != 'rmul2027-validation':
        raise RuntimeError('Run in the isolated rmul2027-validation Gazebo partition')
    assets = Path(__file__).resolve().parents[1]/'meshes/RMUL2027'
    groups = json.loads((assets/'geometry.json').read_text())['materials']
    triangles = np.concatenate([np.asarray(g['vertices'])[np.asarray(g['triangles'])] for g in groups])
    rclpy.init()
    node = rclpy.create_node('check_rmul2027_runtime')
    state,counts = {},dict(clouds=0,finite_points=0)

    def cloud(msg):
        points = read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
        counts['clouds'] += 1
        counts['finite_points'] = max(counts['finite_points'],int(np.isfinite(points).all(axis=1).sum()))

    node.create_subscription(Odometry,'/ground_truth/odometry',lambda m: state.update(robot=m),10)
    node.create_subscription(Odometry,'/rmul2027/probes/odometry',
                             lambda m: state.update({m.child_frame_id:m}),10)
    node.create_subscription(PointCloud2,'/livox/lidar',cloud,qos_profile_sensor_data)
    pub = node.create_publisher(Twist,'/cmd_vel',1)

    def wait(seconds, drive=False):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if drive:
                command = Twist(); command.linear.x = .15; pub.publish(command)
            rclpy.spin_once(node,timeout_sec=.05)

    created, radii = [], {}
    try:
        wait(3)
        assert 'robot' in state,'No robot ground truth'
        positions = [((-4,0),.025),((0,.7),.025),((2.5,0),.025),((6.15,0),.025)]
        positions.extend((xy,.005) for xy in mesh_probe_positions(assets))
        for i,((x,y),radius) in enumerate(positions):
            name = f'rmul2027_probe_{i}'
            probe(name,x,y,radius); created.append(name); radii[name]=radius
        wait(5)
        checks = []
        for name in created:
            assert name in state, 'Missing probe bridge: '+name
            p = state[name].pose.pose.position
            height, normal_z = support(triangles,p.x,p.y)
            expected = height+radii[name]/normal_z
            error = abs(p.z-expected)
            checks.append(dict(name=name,xy=[p.x,p.y],height=p.z,expected=expected,error=error))
            assert error < .0045, checks[-1]
        assert counts['finite_points']>100,counts
        start = state['robot'].pose.pose.position
        start = np.array([start.x,start.y,start.z])
        wait(2.5,drive=True)
        pub.publish(Twist()); wait(1)
        end = state['robot'].pose.pose.position
        travelled = math.hypot(end.x-start[0],end.y-start[1])
        assert travelled>.15,(travelled,start,end)
        assert .075<end.z<.15,end
        print(json.dumps(dict(probes=checks,lidar=counts,robot_travelled_m=travelled,
                              robot_height_m=end.z),indent=2))
    finally:
        pub.publish(Twist())
        for name in created:
            service('remove','gz.msgs.Entity',f'name: "{name}" type: MODEL')
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
