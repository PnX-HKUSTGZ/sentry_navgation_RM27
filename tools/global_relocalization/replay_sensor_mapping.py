#!/usr/bin/env python3
"""Rebuild a Point-LIO map from captured sensors; recorded odometry is excluded."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml

from sensor_mapping import Survey, export, stop, mapping_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--pointlio-overrides", type=Path)
    parser.add_argument("--mapper", required=True)
    parser.add_argument("--duration", type=float, default=-1)
    parser.add_argument("--rate", type=float, default=1)
    parser.add_argument("--read-ahead-queue-size", type=int, default=20000,
                        help="buffer sensor/clock messages ahead to absorb MCAP read stalls")
    parser.add_argument("--voxel", type=float, default=.03)
    a = parser.parse_args()
    if a.read_ahead_queue_size <= 0 or a.rate <= 0:
        parser.error("read-ahead queue size and playback rate must be positive")
    a.method = "repository_pointlio_sensor_replay"
    a.route_map = "not read by replay; see original capture route"
    a.source_revision = "e85e79558cf746f6699888a54285fe48b3b0ac71"
    a.speed, a.rotation_speed = None, None
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    params = out / "mapping_params.yaml"
    profile = mapping_profile(a.params, a.pointlio_overrides)
    profile["point_lio"]["ros__parameters"]["pcd_save"]["pcd_save_en"] = False
    if profile["point_lio"]["ros__parameters"]["prior_pcd"]["enable"]:
        raise ValueError("sensor-only replay cannot load a prior map")
    params.write_text(yaml.safe_dump(profile))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise InterruptedError("sensor replay interrupted")
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = Survey(out)
    processes, failure = [], None
    try:
        commands = [
            ("converter", ["ros2", "run", "ign_sim_pointcloud_tool", "ign_sim_pointcloud_tool_node",
                           "--ros-args", "-r", "__node:=ign_sim_pointcloud_tool", "--params-file", str(params)]),
            ("mapper", [a.mapper, "--ros-args", "-r", "__node:=point_lio", "--params-file", str(params)]),
            ("playback", ["ros2", "bag", "play", str(a.bag.resolve()), "--rate", str(a.rate),
                          "--read-ahead-queue-size", str(a.read_ahead_queue_size),
                          "--delay", "3", "--playback-duration", str(a.duration), "--topics",
                          "/clock", "/livox/lidar", "/livox/imu", "/ground_truth/odometry"])]
        for name, command in commands:
            with (out / f"{name}.log").open("w") as log:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        start = time.monotonic()
        while processes[-1].poll() is None:
            if any(p.poll() is not None for p in processes[:-1]):
                raise RuntimeError("mapping process exited during replay")
            if time.monotonic() - start > (a.duration if a.duration > 0 else 1200) / a.rate + 60:
                raise TimeoutError("sensor replay timeout")
            rclpy.spin_once(node, timeout_sec=.02)
        node.spin_for(2)
        if processes[-1].returncode:
            raise RuntimeError("rosbag playback failed")
    except Exception as error:
        failure = repr(error)
    finally:
        for process in reversed(processes):
            stop(process)
        capture = {"failure": failure, "completed_waypoints": 0, "total_waypoints": 0,
                   "replay_source": str(a.bag.resolve()), "replay_duration_sec": a.duration,
                   "replay_rate": a.rate, "recorded_odometry_used": False,
                   "replay_read_ahead_queue_size": a.read_ahead_queue_size,
                   "odom": [{"stamp": t, "matrix": p.tolist()} for t, p in node.odom],
                   "gt": [{"stamp": t, "matrix": p.tolist()} for t, p in node.gt],
                   "clouds": node.clouds, "commands": []}
        (out / "capture.json").write_text(json.dumps(capture) + "\n")
        if node.clouds and node.odom:
            export(node, out, params, a)
            manifest_path = out / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest.update(replay_source=str(a.bag.resolve()), recorded_odometry_used=False,
                            replay_duration_sec=a.duration, replay_rate=a.rate,
                            replay_read_ahead_queue_size=a.read_ahead_queue_size,
                            source_bag_metadata_sha256=hashlib.sha256((a.bag / "metadata.yaml").read_bytes()).hexdigest())
            manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
            if failure is None:
                (out / "replay_complete.json").write_text(json.dumps({"completed": True, "clouds": len(node.clouds)}) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)


if __name__ == "__main__":
    main()
