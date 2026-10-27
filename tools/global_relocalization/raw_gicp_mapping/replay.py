#!/usr/bin/env python3
"""Sensor-only GICP replay with independent, post-estimation GT scoring."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
FROZEN_CORE = HERE.parent / ".build/ros/install/global_relocalization/lib/libglobal_relocalization_core.so"
sys.path.insert(0, str(HERE.parent))
from evaluation import interpolate, matrix, score_pose
from sensor_mapping import base_sensor, stats, stop


class Observer(Node):
    def __init__(self):
        super().__init__("raw_mapping_replay_observer", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom, self.gt, self.status = [], [], []
        self.cloud_count = 0
        self.create_subscription(Odometry, "/aft_mapped_to_init", self.odometry, 200)
        self.create_subscription(Odometry, "/ground_truth/odometry", self.truth, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/cloud_registered", self.cloud, 20)
        self.create_subscription(String, "/raw_mapping/status", self.diagnostic, 200)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odometry(self, msg):
        self.odom.append((self.stamp(msg), matrix(msg.pose.pose)))

    def truth(self, msg):
        self.gt.append((self.stamp(msg), matrix(msg.pose.pose)))

    def cloud(self, msg):
        self.cloud_count += 1

    def diagnostic(self, msg):
        self.status.append(json.loads(msg.data))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--node", required=True, type=Path)
    parser.add_argument("--profile", type=Path, default=HERE / "profile.yaml")
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--rate", type=float, default=.5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    observer = Observer()
    processes, failure = [], None
    node_command = [str(args.node.resolve()), "--ros-args", "--params-file", str(args.profile.resolve())]
    play_command = ["ros2", "bag", "play", str(args.bag.resolve()), "--rate", str(args.rate),
                    "--delay", "3", "--read-ahead-queue-size", "20000", "--playback-duration", str(args.duration),
                    "--topics", "/clock", "/livox/lidar", "/livox/imu", "/ground_truth/odometry"]
    try:
        for name, command in [("mapper", node_command), ("playback", play_command)]:
            with (args.output / f"{name}.log").open("w") as log:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + args.duration / args.rate + 45
        while processes[-1].poll() is None:
            rclpy.spin_once(observer, timeout_sec=.01)
            if processes[0].poll() is not None:
                raise RuntimeError(f"raw mapping process exited with code {processes[0].returncode}")
            if time.monotonic() > deadline:
                raise TimeoutError("replay deadline exceeded")
        if processes[-1].returncode:
            raise RuntimeError(f"playback exited with code {processes[-1].returncode}")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=.01)
    except Exception as error:
        failure = repr(error)
    finally:
        for process in reversed(processes):
            stop(process)
    capture = {"failure": failure, "ground_truth_used_by_estimator": False,
               "recorded_odometry_used": False, "source_bag": str(args.bag.resolve()),
               "node_command": node_command, "play_command": play_command,
               "environment": {name: os.environ.get(name) for name in ["ROS_DOMAIN_ID", "OMP_NUM_THREADS", "FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE"]},
               "odom": [{"stamp": t, "matrix": p.tolist()} for t, p in observer.odom],
               "gt": [{"stamp": t, "matrix": p.tolist()} for t, p in observer.gt],
               "status": observer.status, "cloud_count": observer.cloud_count,
               "sha256": {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [args.node, args.profile, FROZEN_CORE, args.bag / "metadata.yaml"]}}
    scores = []
    if observer.odom:
        stamp, first = observer.odom[0]
        truth = interpolate(observer.gt, stamp, .25)
        if truth is not None:
            # One gauge alignment after the complete replay; no trajectory fit.
            world_reference = truth @ base_sensor() @ np.linalg.inv(first)
            for stamp, pose in observer.odom:
                truth = interpolate(observer.gt, stamp, .25)
                if truth is not None:
                    scores.append({"stamp": stamp, **score_pose(world_reference @ pose @ np.linalg.inv(base_sensor()), truth)})
    capture["evaluation"] = {"single_initial_gauge_alignment_only": True, "samples": len(scores),
                             "translation_3d_m": stats([s["translation_3d_m"] for s in scores]),
                             "orientation_so3_deg": stats([s["orientation_so3_deg"] for s in scores]),
                             "scores": scores}
    (args.output / "capture.json").write_text(json.dumps(capture, indent=2) + "\n")
    (args.output / "status.jsonl").write_text("".join(json.dumps(s) + "\n" for s in observer.status))
    print(json.dumps({"failure": failure, "poses": len(observer.odom), "clouds": observer.cloud_count,
                      "translation_3d_m": capture["evaluation"]["translation_3d_m"],
                      "orientation_so3_deg": capture["evaluation"]["orientation_so3_deg"]}, indent=2))
    observer.destroy_node()
    rclpy.shutdown()
    return int(failure is not None or not scores)


if __name__ == "__main__":
    raise SystemExit(main())
