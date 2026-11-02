"""The global seed initializes map->odom without changing simulator odometry."""

import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener


@pytest.mark.parametrize("wait_for_seed", [False, True])
def test_simulator_preserves_odometry_and_uses_only_one_timestamped_seed(
    tmp_path, wait_for_seed
):
    executable = (
        Path(get_package_prefix("rm_27_stimulation"))
        / "lib/rm_27_stimulation/rm27_ground_truth_localizer"
    )
    domain = int(os.environ.get("RM27_SIM_HANDOFF_TEST_DOMAIN", "193"))
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node("simulation_seed_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    acks, odom = [], []
    node.create_subscription(
        PoseWithCovarianceStamped,
        "/rm27_ground_truth_localizer/initialpose_accepted",
        acks.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(Odometry, "/odometry", odom.append, 10)
    truth = node.create_publisher(Odometry, "/ground_truth/odometry", 10)
    seed = node.create_publisher(
        PoseWithCovarianceStamped, "/relocalization/initialpose", 10
    )
    log = (tmp_path / "localizer.log").open("w")
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            f"wait_for_initial_pose:={str(wait_for_seed).lower()}",
            "-p",
            "odom_buffer_duration:=10.0",
        ],
        env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
        stdout=log,
        stderr=subprocess.STDOUT,
    )

    def spin_until(predicate, seconds=6):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), (tmp_path / "localizer.log").read_text()

    def drain():
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def publish_truth(stamp, x):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = "world", "base_footprint"
        msg.header.stamp.sec = stamp
        msg.pose.pose.position.x = x
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.7
        truth.publish(msg)
        spin_until(lambda: odom and odom[-1].header.stamp.sec == stamp)

    try:
        spin_until(
            lambda: truth.get_subscription_count() > 0
            and node.count_publishers("/odometry") > 0
        )
        for stamp, x in [(10, 100.0), (11, 101.0), (12, 102.0)]:
            publish_truth(stamp, x)
        assert odom[-1].pose.pose.position.x == pytest.approx(2.0)
        assert odom[-1].twist.twist.linear.x == pytest.approx(0.7)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp.sec = 11
        pose.pose.pose.position.x = 10.0
        pose.pose.pose.orientation.w = 1.0
        if wait_for_seed:
            drain()
            assert not buffer.can_transform("map", "odom", rclpy.time.Time())
            spin_until(lambda: seed.get_subscription_count() > 0)
            pose.header.frame_id = "wrong_map"
            seed.publish(pose)
            drain()
            assert not acks
            assert not buffer.can_transform("map", "odom", rclpy.time.Time())
            pose.header.frame_id = "map"
            seed.publish(pose)
            spin_until(lambda: bool(acks))
            assert acks[-1] == pose
        spin_until(lambda: buffer.can_transform("map", "odom", rclpy.time.Time()))
        expected = 9.0 if wait_for_seed else 100.0
        assert buffer.lookup_transform(
            "map", "odom", rclpy.time.Time()
        ).transform.translation.x == pytest.approx(expected)
        publish_truth(13, 103.0)
        assert odom[-1].pose.pose.position.x == pytest.approx(3.0)
        if wait_for_seed:
            seed.publish(pose)  # Same-sample acknowledgement retry is idempotent.
            spin_until(lambda: len(acks) >= 2)
            count = len(acks)
            pose.pose.pose.position.x = 50.0
            seed.publish(pose)
            drain()
            assert len(acks) == count
        assert buffer.lookup_transform(
            "map", "odom", rclpy.time.Time()
        ).transform.translation.x == pytest.approx(expected)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
