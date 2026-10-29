#!/usr/bin/env python3
"""Independent-process best-effort 10 Hz large PointCloud2 transport probe."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=['publisher', 'subscriber'])
    parser.add_argument('output')
    parser.add_argument('--count', type=int, default=50)
    args = parser.parse_args()
    rclpy.init()
    node = Node('dds_cloud_probe_' + args.role)
    records = []
    topic = '/isolated_dds_cloud_probe'
    if args.role == 'subscriber':
        def receive(msg):
            records.append({'tag': msg.header.frame_id, 'recv_ns': time.monotonic_ns(),
                            'bytes': len(msg.data)})
        node.create_subscription(PointCloud2, topic, receive, qos_profile_sensor_data)
        deadline = time.monotonic() + 27
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        pub = node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
        deadline = time.monotonic() + 8
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('subscriber discovery timed out')
        time.sleep(1)
        msg = PointCloud2()
        msg.height = 1
        msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)]
        msg.point_step = 16
        for mib in (1, 2, 3):
            msg.width = mib * 1024 * 1024 // msg.point_step
            msg.row_step = msg.width * msg.point_step
            msg.data = bytes(msg.row_step)
            start = time.monotonic()
            for seq in range(args.count):
                target = start + seq * 0.1
                time.sleep(max(0, target - time.monotonic()))
                msg.header.frame_id = f'{mib}:{seq}'
                before = time.monotonic_ns()
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                after = time.monotonic_ns()
                records.append({'tag': msg.header.frame_id, 'send_ns': before,
                                'publish_end_ns': after, 'bytes': msg.row_step})
        time.sleep(1)
    Path(args.output).write_text(json.dumps(records, indent=2) + '\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
