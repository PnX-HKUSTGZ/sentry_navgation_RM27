"""Regression: wall-clock indexed playback must not corrupt sensor chronology."""
import json
from pathlib import Path
import subprocess
import sys

import rosbag2_py
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def test_normalization_preserves_sensor_payloads_and_excludes_recorded_estimates(tmp_path):
    original, repaired = tmp_path / "original", tmp_path / "repaired"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(original), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    for topic, type_name in [("/livox/imu", "sensor_msgs/msg/Imu"),
                              ("/aft_mapped_to_init", "nav_msgs/msg/Odometry")]:
        writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=type_name, serialization_format="cdr"))
    expected = []
    for index, stamp in enumerate([3, 1, 2]):
        message = Imu()
        message.header.stamp.sec = stamp
        message.angular_velocity.z = stamp * .1
        payload = serialize_message(message)
        expected.append(payload)
        writer.write("/livox/imu", payload, 10000000000 + index * 1000000000)
    writer.write("/aft_mapped_to_init", serialize_message(Odometry()), 14000000000)
    del writer
    subprocess.run([sys.executable, str(Path(__file__).with_name("normalize_sensor_bag.py")),
                    "--input", str(original), "--output", str(repaired)], check=True, capture_output=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(repaired), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    messages, stamps = [], []
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        assert topic == "/livox/imu"
        assert stamp == deserialize_message(payload, Imu).header.stamp.sec * 1000000000
        messages.append(payload)
        stamps.append(stamp)
    assert stamps == [1000000000, 2000000000, 3000000000]
    assert sorted(messages) == sorted(expected)
    report = json.loads(repaired.with_suffix(".audit.json").read_text())
    assert report["message_count"] == 3
    assert report["input_order_regressions"]["/livox/imu"] == 1
    assert report["recorded_estimates_replayed"] is False
