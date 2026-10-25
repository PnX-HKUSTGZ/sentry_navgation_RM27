#!/usr/bin/env python3
"""Read declared ROS parameters and verify every frozen profile value."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import parameter_value_to_python
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=90)
    args = parser.parse_args()
    expected = yaml.safe_load(Path(args.params).read_text())["global_relocalization"][
        "ros__parameters"
    ]
    rclpy.init(args=[])
    node = Node("global_relocalization_runtime_parameter_audit")
    client = AsyncParameterClient(node, "/global_relocalization")
    deadline = time.monotonic() + args.timeout
    report = {
        "expected": expected,
        "declared_names": [],
        "actual": {},
        "mismatches": [],
        "ground_truth_used": False,
        "passed": False,
    }
    try:
        if not client.wait_for_services(timeout_sec=args.timeout):
            raise RuntimeError("parameter services unavailable")

        def result(future):
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=max(0, deadline - time.monotonic())
            )
            if not future.done() or future.exception():
                raise RuntimeError("parameter response timeout or failure")
            return future.result()

        report["declared_names"] = sorted(result(client.list_parameters()).result.names)
        subscriptions = None
        graph_error = None
        while time.monotonic() < deadline:
            try:
                subscriptions = node.get_subscriber_names_and_types_by_node(
                    "global_relocalization", "/"
                )
                break
            except Exception as error:
                graph_error = error
                if "nonexistent" not in str(error):
                    raise
                rclpy.spin_once(node, timeout_sec=0.05)
        if subscriptions is None:
            raise RuntimeError(f"global_relocalization node not visible: {graph_error}")
        report["actual_subscriptions"] = [
            {"topic": topic, "types": types} for topic, types in subscriptions
        ]
        if expected.get("input_mode") == "raw_imu":
            forbidden = [
                item["topic"]
                for item in report["actual_subscriptions"]
                if item["topic"]
                in ["/ground_truth/odometry", expected.get("odom_topic", "/odometry")]
            ]
            if forbidden:
                report["mismatches"].append(
                    {"forbidden_raw_estimator_subscriptions": forbidden}
                )
        missing = sorted(set(expected) - set(report["declared_names"]))
        if missing:
            report["mismatches"].append({"undeclared": missing})
        names = [name for name in expected if name not in missing]
        values = result(client.get_parameters(names)).values
        for name, value in zip(names, values):
            actual = parameter_value_to_python(value)
            report["actual"][name] = actual
            wanted = expected[name]
            equal = type(actual) is type(wanted) and actual == wanted
            if not equal:
                report["mismatches"].append(
                    {"name": name, "expected": wanted, "actual": actual}
                )
        report["passed"] = not report["mismatches"]
    except Exception as error:
        report["error"] = str(error)
    finally:
        Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    print(
        json.dumps(
            {
                "parameter_audit_passed": report["passed"],
                "checked": len(report["actual"]),
                "mismatches": report["mismatches"],
                "error": report.get("error"),
            }
        ),
        flush=True,
    )
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
