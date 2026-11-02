#!/usr/bin/env python3
"""Observe the isolated simulation graph and advancing sensor timestamps."""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=6)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node("global_relocalization_simulation_audit")
    topics = {
        "/clock": Clock,
        "/ground_truth/odometry": Odometry,
        "/livox/lidar": PointCloud2,
        "/livox/imu": Imu,
    }
    stamps = {topic: [] for topic in topics}
    last_received = {}
    frames = {topic: set() for topic in topics}

    def observe(topic, message):
        stamp = message.clock if topic == "/clock" else message.header.stamp
        stamps[topic].append(stamp.sec * 1000000000 + stamp.nanosec)
        last_received[topic] = time.monotonic()
        if topic != "/clock":
            frames[topic].add(message.header.frame_id)

    for topic, message_type in topics.items():
        node.create_subscription(
            message_type,
            topic,
            lambda message, name=topic: observe(name, message),
            # The bridge publishes reliable data. A reliable audit observes
            # every large cloud, rather than silently losing UDP fragments.
            QoSProfile(depth=30, reliability=(ReliabilityPolicy.BEST_EFFORT if topic == "/clock"
                                            else ReliabilityPolicy.RELIABLE)),
        )
    start = time.monotonic()
    deadline = start + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    report = {
        "ground_truth_used_for_estimation_or_control": False,
        "scope": "this audit observer only; the separate survey controller is described by its mapping manifest",
        "topics": {},
        "observation_wall_sec": time.monotonic() - start,
        "environment": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "gz_partition": os.environ.get("GZ_PARTITION"),
        },
    }
    for topic, values in stamps.items():
        publishers = node.get_publishers_info_by_topic(topic)
        report["topics"][topic] = {
            "publisher_count": len(publishers),
            "publishers": [
                {
                    "node_name": item.node_name,
                    "node_namespace": item.node_namespace,
                    "topic_type": item.topic_type,
                }
                for item in publishers
            ],
            "samples": len(values),
            "header_rate_hz": ((len(values) - 1) * 1e9 / (values[-1] - values[0])
                               if len(values) >= 2 and values[-1] > values[0] else 0),
            "max_header_gap_sec": (max(b - a for a, b in zip(values, values[1:])) * 1e-9
                                   if len(values) >= 2 else None),
            "first_stamp_ns": values[0] if values else None,
            "last_stamp_ns": values[-1] if values else None,
            "advancing": len(values) >= 2 and values[-1] > values[0],
            "nondecreasing": all(b >= a for a, b in zip(values, values[1:])),
            "frames": sorted(frames[topic]),
            "last_callback_wall_age_sec": (
                time.monotonic() - last_received[topic]
                if topic in last_received
                else None
            ),
        }
    minimum_rates = {"/clock": 200, "/ground_truth/odometry": 40, "/livox/lidar": 8, "/livox/imu": 80}
    report["minimum_header_rate_hz"] = minimum_rates
    report["passed"] = all(
        item["publisher_count"] == 1
        and item["samples"] >= 2
        and item["advancing"]
        and item["nondecreasing"]
        and item["last_callback_wall_age_sec"] is not None
        and item["last_callback_wall_age_sec"] < 1.5
        and item["header_rate_hz"] >= minimum_rates[topic]
        for topic, item in report["topics"].items()
    )
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
