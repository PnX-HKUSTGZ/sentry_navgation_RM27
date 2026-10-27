#!/usr/bin/env python3
"""Round-trip the built simulator converter with finite and invalid returns."""
import argparse
import math
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def check(node, executable, instantaneous):
    received = []
    subscription = node.create_subscription(PointCloud2, '/timing_test/output', received.append,
                                            qos_profile_sensor_data)
    publisher = node.create_publisher(PointCloud2, '/timing_test/input', 10)
    header = Header(frame_id='test_lidar')
    header.stamp.sec, header.stamp.nanosec = 42, 12345
    points = [[2.,0.,0.],[0.,2.,0.],[math.inf,0.,0.],[math.nan,0.,0.]]
    message = point_cloud2.create_cloud_xyz32(header, points)
    process = subprocess.Popen([executable, '--ros-args', '-r', '__node:=timing_test_converter',
        '-p', 'pcd_topic:=/timing_test/input', '-r', 'velodyne_points:=/timing_test/output',
        '-p', 'instantaneous_scan:=' + str(instantaneous).lower()])
    try:
        deadline = time.monotonic()+15
        while not received and time.monotonic()<deadline:
            if process.poll() is not None:
                raise RuntimeError('converter exited')
            publisher.publish(message)
            rclpy.spin_once(node,timeout_sec=.1)
        if not received:
            raise RuntimeError('converter produced no frame')
        result=received[-1]
        assert result.header == header
        values=list(point_cloud2.read_points(result, field_names=('x','y','z','time')))
        assert len(values)==2, values
        assert values[0][3] == 0.
        if instantaneous:
            assert all(row[3] == 0. for row in values)
        else:
            assert values[1][3] > 0.
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_publisher(publisher)
        node.destroy_subscription(subscription)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--converter',required=True)
    args=parser.parse_args()
    rclpy.init()
    node=rclpy.create_node('converter_timing_test')
    try:
        check(node,args.converter,True)
        check(node,args.converter,False)
        print('converter instantaneous / legacy time, header, invalid-return filtering passed')
    finally:
        node.destroy_node()
        rclpy.shutdown()
