#!/usr/bin/env python3
"""Passive receipt timing only: no GT, command publisher, raw scan storage or control."""
import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from receipt_metrics import ReceiptMetrics
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('global_relocalization_passive_receipt_probe')
    metrics = ReceiptMetrics()
    lidar_qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    running = True
    def finish(*unused):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, finish)
    signal.signal(signal.SIGTERM, finish)
    with Path(args.output).open('w') as output:
        def lidar(message):
            stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            metrics.receive(stamp, time.monotonic())
        def status(message):
            try:
                status = json.loads(message.data)
            except ValueError:
                return
            output.write(json.dumps({'wall_monotonic': time.monotonic(), **metrics.status(status.get('run_id')),
                'observer_lidar_qos': {'reliability': 'reliable', 'history': 'keep_last', 'depth': 64},
                'source_callback_age_sec': time.monotonic()-metrics.last_wall if metrics.last_wall is not None else None,
                'run_id': status.get('run_id'), 'stage': status.get('stage'),
                'status_ros_ns': status.get('status_ros_ns', status.get('stamp_ns', status.get('ros_ns'))),
                'status_steady_sec': status.get('status_steady_sec'),
                'status_elapsed_sec': status.get('elapsed_sec'), 'source_last_receipt_wall': metrics.last_wall,
                'raw_frontend': status.get('raw_frontend', {}), 'ground_truth_used': False})+'\n')
        node.create_subscription(PointCloud2, '/livox/lidar', lidar, lidar_qos)
        node.create_subscription(String, '/global_relocalization/status', status, 10)
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
        output.flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
