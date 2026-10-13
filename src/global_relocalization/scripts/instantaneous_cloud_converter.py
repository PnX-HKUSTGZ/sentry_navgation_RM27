#!/usr/bin/env python3
"""Adapt instantaneous Gazebo GPU scans to Point-LIO Velodyne fields.
The upstream converter assigns fictitious 100 ms point times. A constant 1 ns
positive offset explicitly disables Point-LIO's zero-time auto spin synthesis;
all points remain simultaneous (1 ns approximation).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class Converter(Node):
    def __init__(self):
        super().__init__('instantaneous_cloud_converter')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else None
        self.pub = self.create_publisher(PointCloud2, '/velodyne_points', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/livox/lidar', self.convert, qos_profile_sensor_data)
        self.dtype = np.dtype({'names': ['x','y','z','intensity','ring','time'],
            'formats': ['<f4','<f4','<f4','<f4','<u2','<f4'], 'offsets': [0,4,8,12,16,20], 'itemsize': 24})

    def convert(self, msg):
        raw = point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=False)
        xyz = np.stack([raw[name].reshape(-1) for name in ('x','y','z')], axis=1)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            return
        out = np.zeros(len(xyz), self.dtype)
        for index,name in enumerate(('x','y','z')):
            out[name] = xyz[:,index]
        elevation = np.arctan2(xyz[:,2], np.hypot(xyz[:,0],xyz[:,1]))
        out['ring'] = np.clip(np.rint((np.degrees(elevation)+7.22)/((55.22+7.22)/31)),0,31).astype(np.uint16)
        out['time'] = 1e-9
        result = PointCloud2(header=msg.header, height=1, width=len(out), is_bigendian=False, is_dense=True,
            point_step=24, row_step=24*len(out), data=out.tobytes())
        result.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n,o,t in
            [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',20,7)]]
        self.pub.publish(result)


def main():
    rclpy.init()
    node = Converter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
