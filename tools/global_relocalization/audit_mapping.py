#!/usr/bin/env python3
"""Audit the live repository Point-LIO mapper without relying on the ROS daemon."""
import argparse
import hashlib
import json
import os
import time
from pathlib import Path

import psutil
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--expected-mapper", required=True, type=Path)
    parser.add_argument("--include-slam", action="store_true")
    a = parser.parse_args()
    rclpy.init(args=[])
    node = Node("repository_mapping_audit")
    report = {"passed": False}
    try:
        client = AsyncParameterClient(node, "/point_lio")
        if not client.wait_for_services(timeout_sec=15):
            raise RuntimeError("Point-LIO parameter services unavailable")
        def wait(future):
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or future.exception():
                raise RuntimeError("Point-LIO parameter response timed out")
            return future.result()
        names = wait(client.list_parameters()).result.names
        values = wait(client.get_parameters(names)).values
        actual = {k: parameter_value_to_python(v) for k, v in zip(names, values)}
        def subscriptions_ready(name, required):
            # Parameter responses may arrive before the separate ROS graph
            # discovery cache (especially with fast local SHM). Audit the
            # actual subscriptions once the graph has discovered them.
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    subscriptions = node.get_subscriber_names_and_types_by_node(name, "/")
                    if required.issubset({topic for topic, _ in subscriptions}):
                        return subscriptions
                except rclpy.node.NodeNameNonExistentError:
                    pass
            raise RuntimeError(f"subscription discovery timed out: {name}")
        subscriptions = subscriptions_ready("point_lio", {"/velodyne_points", "/livox/imu"})
        report.update(actual=actual, subscriptions=subscriptions)
        if actual.get("prior_pcd.enable") is not False:
            raise RuntimeError("mapper loaded a prior map")
        if actual.get("mapping.imu_en") is not True:
            raise RuntimeError("mapper IMU disabled")
        if any("ground_truth" in topic for topic, _ in subscriptions):
            raise RuntimeError("mapper subscribed to GT")
        if a.include_slam:
            slam_client = AsyncParameterClient(node, "/slam_toolbox")
            if not slam_client.wait_for_services(timeout_sec=10):
                raise RuntimeError("SLAM Toolbox parameter services unavailable")
            slam_names = ["use_scan_matching", "do_loop_closing", "transform_publish_period",
                          "scan_topic", "mode", "base_frame", "map_frame", "odom_frame"]
            slam_values = wait(slam_client.get_parameters(slam_names)).values
            report["slam_toolbox"] = {
                "actual": {k: parameter_value_to_python(v) for k, v in zip(slam_names, slam_values)},
                "subscriptions": subscriptions_ready("slam_toolbox", {"/obstacle_scan"})}
        processes = []
        for process in psutil.process_iter(["pid", "exe"]):
            try:
                if (process.info["exe"] and Path(process.info["exe"]).resolve() == a.expected_mapper.resolve()
                        and process.environ().get("ROS_DOMAIN_ID") == os.environ.get("ROS_DOMAIN_ID")):
                    processes.append(process)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        if len(processes) != 1:
            raise RuntimeError(f"expected one mapping process, found {len(processes)}")
        process = processes[0]
        actual_path = Path(process.exe()).resolve()
        if actual_path != a.expected_mapper.resolve():
            raise RuntimeError(f"unexpected mapper executable: {actual_path}")
        report.update(pid=process.pid, executable=str(actual_path), command=process.cmdline(),
                      executable_sha256=hashlib.sha256(actual_path.read_bytes()).hexdigest(), passed=True)
    except Exception as error:
        report["error"] = repr(error)
    finally:
        a.output.write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k: v for k, v in report.items() if k not in ("actual", "subscriptions", "command")}))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
