#!/usr/bin/env python3
"""Inspect original sensor delivery around the largest Point-LIO pose jump."""
import argparse
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True)
    a = parser.parse_args()
    out = a.map_dir
    capture = json.loads((out / "capture.json").read_text())
    poses = np.array([r["matrix"] for r in capture["odom"]])
    stamps = np.array([r["stamp"] for r in capture["odom"]])
    steps = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    index = int(np.argmax(steps))
    start, end = stamps[index] - 2, stamps[index + 1] + 2
    types = {"/livox/lidar": PointCloud2, "/livox/imu": Imu, "/ground_truth/odometry": Odometry}
    rows, imu, truth = {name: [] for name in types}, [], []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out / "raw_bag"), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    reader.seek(int((start - 1) * 1e9))
    while reader.has_next():
        topic, payload, recorded = reader.read_next()
        if recorded > (end + 1) * 1e9:
            break
        msg = deserialize_message(payload, types[topic])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not start <= t <= end:
            continue
        rows[topic].append(t)
        if topic == "/livox/imu":
            acc, gyro = msg.linear_acceleration, msg.angular_velocity
            imu.append([t, acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z])
        elif topic == "/ground_truth/odometry":
            p = msg.pose.pose.position
            truth.append([t, p.x, p.y, p.z])
    streams = {}
    for name, times in rows.items():
        difference = np.diff(times)
        streams[name] = {"count": len(times), "max_header_gap_sec": float(max(difference)) if len(difference) else None,
                         "nonincreasing_intervals": int(sum(difference <= 0)),
                         "first_stamp": times[0] if times else None, "last_stamp": times[-1] if times else None}
    im, gt = np.array(imu), np.array(truth)
    result = {"largest_raw_pointlio_step_m": float(steps[index]),
              "largest_step_dt_sec": float(stamps[index + 1] - stamps[index]),
              "before": capture["odom"][index], "after": capture["odom"][index + 1],
              "sensor_window": [float(start), float(end)], "sensor_streams": streams,
              "truth_xyz_extent_m": np.ptp(gt[:, 1:], axis=0).tolist() if len(gt) else None,
              "imu_acceleration_extent_mps2": np.ptp(im[:, 1:4], axis=0).tolist() if len(im) else None,
              "imu_max_angular_speed_rad_sec": float(max(np.linalg.norm(im[:, 4:], axis=1))) if len(im) else None,
              "interpretation": "The jump is already present in raw Point-LIO before SLAM Toolbox's planar correction. Sensor arrival and GT motion are independent evidence; this audit does not establish a unique numerical or sensor-model cause.",
              "maps_modified": False, "gt_fused": False}
    (out / "mapping_failure_analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k:v for k,v in result.items() if k not in ["before", "after"]}))


if __name__ == "__main__":
    main()
