#!/usr/bin/env python3
"""Reindex captured raw messages by sensor stamps without changing payload bytes.

WSL wall-clock corrections can make recorder receive timestamps go backwards.
This produces a separate replay bag and per-topic order-independent payload
hashes, while retaining the original bag. Recorded estimated poses are excluded.
"""
import argparse
import hashlib
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    a = parser.parse_args()
    if a.output.exists():
        raise ValueError("output already exists")
    metadata = __import__("yaml").safe_load((a.input / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(a.input), storage_id=metadata["storage_identifier"]),
                rosbag2_py.ConverterOptions("", ""))
    topics = {t.name: t for t in reader.get_all_topics_and_types() if t.name in
              ["/clock", "/livox/imu", "/livox/lidar", "/velodyne_points", "/ground_truth/odometry"]}
    types = {name: get_message(t.type) for name, t in topics.items()}
    rows, hashes = [], {name: [] for name in topics}
    order_errors, latest = {name: 0 for name in topics}, {}
    while reader.has_next():
        topic, payload, received = reader.read_next()
        if topic not in topics:
            continue
        msg = deserialize_message(payload, types[topic])
        stamp = msg.clock if topic == "/clock" else msg.header.stamp
        stamp = stamp.sec * 1000000000 + stamp.nanosec
        if stamp < latest.get(topic, stamp):
            order_errors[topic] += 1
        latest[topic] = stamp
        rows.append((stamp, len(rows), topic, payload))
        hashes[topic].append(hashlib.sha256(payload).digest())
    rows.sort(key=lambda row: (row[0], row[1]))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(a.output), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for topic in topics.values():
        writer.create_topic(topic)
    output_hashes = {name: [] for name in topics}
    for stamp, _, topic, payload in rows:
        writer.write(topic, payload, stamp)
        output_hashes[topic].append(hashlib.sha256(payload).digest())
    del writer
    report = {"source": str(a.input.resolve()), "output": str(a.output.resolve()),
              "method": "sort by existing sensor header stamps; serialized payloads unchanged",
              "recorded_estimates_replayed": False, "input_order_regressions": order_errors,
              "message_count": len(rows), "topics": {}}
    for topic, values in hashes.items():
        source_hash = hashlib.sha256(b"".join(sorted(values))).hexdigest()
        output_hash = hashlib.sha256(b"".join(sorted(output_hashes[topic]))).hexdigest()
        if source_hash != output_hash:
            raise RuntimeError("payload integrity mismatch")
        report["topics"][topic] = {"messages": len(values), "payload_multiset_sha256": source_hash,
                                   "output_payload_multiset_sha256": output_hash}
    a.output.with_suffix(".audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"message_count": len(rows), "regressions": order_errors}), flush=True)


if __name__ == "__main__":
    main()
