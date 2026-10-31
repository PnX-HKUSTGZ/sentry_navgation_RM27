#!/usr/bin/env python3
"""Record scan/IMU/odometry timing without sending motion commands.

Run with the simulation's ROS_DOMAIN_ID. --reliable-lidar requires both the
converter and Point-LIO to use the reliable transport option. The JSONL keeps
individual acquisition and receive times; the JSON summarizes timing and
warnings. A missing sample is not a measurement of zero vehicle speed.
"""

import argparse
import collections
import json
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--reliable-lidar", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node(
        "odometry_pipeline_trace",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    samples = collections.defaultdict(list)
    warnings = []
    subs = []
    trace_path = args.output.with_suffix(".jsonl")
    with trace_path.open("w") as stream:

        def receive(topic, msg):
            row = {
                "topic": topic,
                "wall": time.monotonic() - started,
                "sim": node.get_clock().now().nanoseconds * 1e-9,
            }
            if hasattr(msg, "header"):
                row["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                row["age"] = row["sim"] - row["stamp"]
            if isinstance(msg, PointCloud2):
                row["points"] = msg.width * msg.height
            if isinstance(msg, DiagnosticArray):
                row["values"] = {
                    value.key: value.value for status in msg.status for value in status.values
                }
            if isinstance(msg, Log):
                if msg.level < Log.WARN:
                    return
                row.update(name=msg.name, message=msg.msg)
                warnings.append(row)
            else:
                samples[topic].append(row)
            stream.write(json.dumps(row) + "\n")

        for topic, typ in [
            ("/livox/lidar", PointCloud2),
            ("/velodyne_points", PointCloud2),
            ("/livox/imu", Imu),
            ("/aft_mapped_to_init", Odometry),
            ("/lidar_odometry", Odometry),
            ("/odometry", Odometry),
            ("/point_lio/diagnostics", DiagnosticArray),
            ("/rosout", Log),
        ]:
            qos = qos_profile_sensor_data
            if topic == "/livox/lidar" or (
                topic == "/velodyne_points" and args.reliable_lidar
            ):
                qos = QoSProfile(depth=2)
            subs.append(node.create_subscription(
                typ, topic, lambda msg, topic=topic: receive(topic, msg), qos
            ))
        flushed = started
        try:
            while rclpy.ok() and not stopping and time.monotonic() - started < args.duration:
                rclpy.spin_once(node, timeout_sec=0.02)
                if time.monotonic() - flushed >= 1.0:
                    stream.flush()
                    flushed = time.monotonic()
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def percentiles(values):
        return np.percentile(values, [50, 95, 100]).tolist() if len(values) else []

    report = {
        "duration_wall_s": time.monotonic() - started,
        "reliable_lidar_observer": args.reliable_lidar,
        "percentile_order": ["p50", "p95", "max"],
        "trace": str(trace_path),
        "topics": {},
        "warnings": warnings,
    }
    for topic, rows in samples.items():
        report["topics"][topic] = {
            "count": len(rows),
            "receive_interval_wall_s": percentiles(np.diff([r["wall"] for r in rows])),
            "measurement_age_on_receive_sim_s": percentiles([r["age"] for r in rows]),
        }
    diagnostics = samples.get("/point_lio/diagnostics", [])
    if diagnostics:
        report["point_lio_last_diagnostics"] = diagnostics[-1]["values"]
        report["point_lio_processing_ms"] = percentiles([
            float(r["values"]["processing_ms"]) for r in diagnostics
        ])
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["topics"], indent=2))


if __name__ == "__main__":
    main()
