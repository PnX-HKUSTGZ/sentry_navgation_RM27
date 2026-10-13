"""Exercise the actual server's busy/cancel/retry/watchdog contract without sensors."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from global_relocalization_interfaces.action import Relocalize


def test_cancel_retry_timeout_keeps_motor_stopped():
    binary = Path(os.environ["GLOBAL_RELOCALIZATION_NODE"])
    # The test has its own DDS domain and never connects to a robot/simulation.
    domain = str(100 + os.getpid() % 100)
    os.environ["ROS_DOMAIN_ID"] = domain
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "map.pcd"
        points = [(x * .3, y * .3, z * .3) for x in range(5) for y in range(4) for z in range(3)]
        path.write_text(
            "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
            + "".join(f"{x} {y} {z}\n" for x, y, z in points)
        )
        process = subprocess.Popen(
            [str(binary), "--ros-args", "-p", f"map_path:={path}", "-p", "timeout_sec:=1.0",
             "-p", "settle_sec:=0.1"],
            env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True,
        )
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("global_relocalization_contract_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, Relocalize, "/global_relocalization/run")
        commands, poses = [], []
        node.create_subscription(Twist, "/cmd_vel", commands.append, 10)
        node.create_subscription(PoseStamped, "/global_relocalization/pose", poses.append, 10)

        def wait(future, timeout=4):
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            assert future.done(), "ROS action future did not complete"
            return future.result()

        try:
            assert client.wait_for_server(timeout_sec=5)
            first = wait(client.send_goal_async(Relocalize.Goal()))
            assert first.accepted
            assert not wait(client.send_goal_async(Relocalize.Goal())).accepted
            assert wait(first.cancel_goal_async()).goals_canceling
            canceled = wait(first.get_result_async())
            assert canceled.status == 5  # STATUS_CANCELED
            assert not canceled.result.success
            retry = wait(client.send_goal_async(Relocalize.Goal()))
            assert retry.accepted
            failed = wait(retry.get_result_async())
            assert not failed.result.success
            assert "timeout" in failed.result.reason
            assert not poses
            assert commands
            assert all(
                all(value == 0.0 for value in
                    (m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z))
                for m in commands
            )
            # Real subscription order: every cloud arrives before its exact TF.
            # A retry queue must consume it after TF arrives; immediately
            # dropping each cloud must not pass this experiment.
            cloud_pub = node.create_publisher(PointCloud2, "/registered_scan", 10)
            odom_pub = node.create_publisher(Odometry, "/odometry", 10)
            dynamic_tf = TransformBroadcaster(node)
            static_tf = StaticTransformBroadcaster(node)
            mount = TransformStamped()
            mount.header.frame_id = "base_footprint"
            mount.child_frame_id = "left_mid360"
            mount.transform.rotation.w = 1.0
            static_tf.sendTransform(mount)
            sensor_goal = wait(client.send_goal_async(Relocalize.Goal()))
            assert sensor_goal.accepted
            command_start = len(commands)
            for _ in range(4):
                stamp = node.get_clock().now().to_msg()
                odometry = Odometry()
                odometry.header.stamp = stamp
                odometry.header.frame_id = "odom"
                odometry.child_frame_id = "base_footprint"
                odometry.pose.pose.orientation.w = 1.0
                header = odometry.header
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
                odom_pub.publish(odometry)
                delayed = time.monotonic()+.08
                while time.monotonic()<delayed:
                    executor.spin_once(timeout_sec=.01)
                transform = TransformStamped()
                transform.header = header
                transform.child_frame_id = "base_footprint"
                transform.transform.rotation.w = 1.0
                dynamic_tf.sendTransform(transform)
                delivered = time.monotonic()+.08
                while time.monotonic()<delivered:
                    executor.spin_once(timeout_sec=.01)
            assert any(message.angular.z != 0.0 for message in commands[command_start:]), \
                "Clouds arriving before exact-time TF never reached the spinning stage"
            assert wait(sensor_goal.cancel_goal_async()).goals_canceling
            assert wait(sensor_goal.get_result_async()).status == 5
            # Shutdown with an accepted, still active action must stop cleanly,
            # even when ActionServer removes its goals during context shutdown.
            shutdown_goal = wait(client.send_goal_async(Relocalize.Goal()))
            assert shutdown_goal.accepted
            command_start = len(commands)
            spinning = time.monotonic()+.25
            while time.monotonic()<spinning:
                executor.spin_once(timeout_sec=.01)
            assert any(message.angular.z != 0.0 for message in commands[command_start:])
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
            stopped = time.monotonic()+.1
            while time.monotonic()<stopped:
                executor.spin_once(timeout_sec=.01)
            assert commands[-1].angular.z == 0.0
        finally:
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                output = process.communicate(timeout=5)[0]
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                output = process.communicate()[0]
            assert process.returncode == 0, output


def test_raw_imu_calibration_worker_cancel_and_reset():
    """Raw mode operates without odometry, and retries require fresh calibration."""
    import json
    import numpy as np
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    binary = Path(os.environ["GLOBAL_RELOCALIZATION_NODE"])
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "map.pcd"
        points = [(x * .2, y * .2, z * .2) for x in range(10) for y in range(10) for z in range(5)]
        path.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
                        + "".join(f"{x} {y} {z}\n" for x, y, z in points))
        process = subprocess.Popen([str(binary), "--ros-args", "-p", f"map_path:={path}",
            "-p", "input_mode:=raw_imu", "-p", "raw_calibration_sec:=0.4",
            "-p", "settle_sec:=0.1", "-p", "timeout_sec:=4.0"], env=os.environ.copy(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("raw_relocalization_contract_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, Relocalize, "/global_relocalization/run")
        commands, states, poses = [], [], []
        node.create_subscription(Twist, "/cmd_vel", commands.append, 10)
        node.create_subscription(String, "/global_relocalization/status", lambda m: states.append(json.loads(m.data)), 10)
        node.create_subscription(PoseStamped, "/global_relocalization/pose", poses.append, 10)
        cloud_pub = node.create_publisher(PointCloud2, "/livox/lidar", 10)
        static_tf = StaticTransformBroadcaster(node)
        mount = TransformStamped()
        mount.header.frame_id = "base_footprint"
        mount.child_frame_id = "left_mid360"
        mount.transform.rotation.w = 1.0
        static_tf.sendTransform(mount)

        def wait(future, timeout=5):
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert future.done()
            return future.result()

        # One 1.5M-point burst exercises conversion, transformation and the
        # accumulated-cloud voxel filter while the independent producer keeps IMU continuous.
        from std_msgs.msg import Header
        large_cloud = point_cloud2.create_cloud_xyz32(Header(), np.tile(np.asarray(points, dtype=np.float32), (3000, 1)))
        burst_sent = False

        imu_process = subprocess.Popen(["/usr/bin/python3", "-c", """
import rclpy
from sensor_msgs.msg import Imu
rclpy.init()
node = rclpy.create_node('continuous_imu_contract_source')
pub = node.create_publisher(Imu, '/livox/imu', 200)
# Model a stationary sensor sampling at 100 Hz. A delayed publisher emits
# every buffered measurement at its original stamp, rather than inventing a
# missing physical sample when the Python timer scheduler runs late.
last_sample_ns = node.get_clock().now().nanoseconds
def emit():
    global last_sample_ns
    current_ns = node.get_clock().now().nanoseconds
    while last_sample_ns + 10_000_000 <= current_ns:
        last_sample_ns += 10_000_000
        message = Imu()
        message.header.stamp = rclpy.time.Time(nanoseconds=last_sample_ns).to_msg()
        message.header.frame_id = 'left_mid360'
        message.linear_acceleration.z = 9.81
        pub.publish(message)
node.create_timer(.01, emit)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
"""], env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def feed(duration):
            nonlocal burst_sent
            deadline = time.monotonic() + duration
            next_cloud = time.monotonic()
            while time.monotonic() < deadline:
                current = time.monotonic()
                if current >= next_cloud:
                    header = Imu().header
                    header.stamp = node.get_clock().now().to_msg()
                    header.frame_id = "left_mid360"
                    if not burst_sent and commands and commands[-1].angular.z != 0:
                        large_cloud.header = header
                        cloud_pub.publish(large_cloud)
                        burst_sent = True
                    else:
                        cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
                    next_cloud = current + .1
                executor.spin_once(timeout_sec=.002)

        try:
            assert client.wait_for_server(timeout_sec=5)
            deadline = time.monotonic()+5
            while node.count_publishers("/livox/imu")==0 and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.01)
            assert node.count_publishers("/livox/imu") > 0, "Independent continuous source never became ready"
            first = wait(client.send_goal_async(Relocalize.Goal()))
            assert first.accepted
            active_result = first.get_result_async()
            feed(2.2)
            assert burst_sent, json.dumps({"state": states[-1:] , "reason": active_result.result().result.reason if active_result.done() else None, "imu_source_exit": imu_process.poll()})
            assert any(m.angular.z != 0 for m in commands)
            assert not active_result.done(), "Continuous input was rejected during a processing burst"
            assert any(s.get("raw_frontend", {}).get("gravity_initialized") for s in states)
            assert wait(first.cancel_goal_async()).goals_canceling
            assert wait(first.get_result_async()).status == 5
            feed(.1)
            assert commands[-1].angular.z == 0
            final_raw = next(s["raw_frontend"] for s in reversed(states) if s["stage"]=="CANCELED")
            assert final_raw["imu_rejected"] == 0
            assert final_raw["imu_max_received_stamp_gap_sec"] <= .011
            assert final_raw["imu_accepted"] >= 120
            assert final_raw["max_worker_wall_sec"] > .05, "Fixture must exceed default IMU depth at 100 Hz"
            retry = wait(client.send_goal_async(Relocalize.Goal()))
            assert retry.accepted
            start = len(commands)
            feed(.15)
            assert all(m.angular.z == 0 for m in commands[start:])
            assert wait(retry.cancel_goal_async()).goals_canceling
            assert wait(retry.get_result_async()).status == 5
            assert not poses
        finally:
            imu_process.send_signal(signal.SIGINT)
            imu_process.wait(timeout=5)
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            output = process.communicate(timeout=5)[0]
            assert process.returncode == 0, output


@pytest.mark.parametrize("break_stream", [False, True])
def test_searching_uses_live_raw_sensor_health_after_freezing_scan(break_stream):
    """A slow real database search must monitor arrivals beyond the frozen scan stamp."""
    import json
    import math
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    binary = Path(os.environ["GLOBAL_RELOCALIZATION_NODE"])
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "map.pcd"
        points = [(x*.2, y*.2, z*.2) for x in range(10) for y in range(10) for z in range(5)]
        path.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
                        + "".join(f"{x} {y} {z}\n" for x, y, z in points))
        process = subprocess.Popen([str(binary), "--ros-args", "-p", f"map_path:={path}",
            "-p", "input_mode:=raw_imu", "-p", "raw_calibration_sec:=0.4",
            "-p", "settle_sec:=0.1", "-p", "spin_angle:=0.05", "-p", "timeout_sec:=12.0",
            "-p", "data_timeout_sec:=0.3", "-p", "db_grid_resolution:=0.015",
            "-p", "db_min_points:=80"], env=os.environ.copy(), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = rclpy.create_node("raw_search_health_test", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, Relocalize, "/global_relocalization/run")
        commands, states = [], []
        node.create_subscription(Twist, "/cmd_vel", commands.append, 10)
        node.create_subscription(String, "/global_relocalization/status", lambda m: states.append(json.loads(m.data)), 10)
        imu_pub = node.create_publisher(Imu, "/livox/imu", 10)
        cloud_pub = node.create_publisher(PointCloud2, "/livox/lidar", 10)
        static_tf = StaticTransformBroadcaster(node)
        mount = TransformStamped()
        mount.header.frame_id = "base_footprint"
        mount.child_frame_id = "left_mid360"
        mount.transform.rotation.w = 1.0
        static_tf.sendTransform(mount)
        next_imu = next_cloud = previous = time.monotonic()
        actual_yaw = 0.

        def step(feed=True):
            nonlocal next_imu, next_cloud, previous, actual_yaw
            current = time.monotonic()
            rate = commands[-1].angular.z if commands else 0.
            actual_yaw += rate*(current-previous)
            previous = current
            if feed and current >= next_imu:
                imu = Imu()
                imu.header.stamp = node.get_clock().now().to_msg()
                imu.header.frame_id = "left_mid360"
                imu.angular_velocity.z = rate
                imu.linear_acceleration.z = 9.81
                imu_pub.publish(imu)
                next_imu = current + .01
            if feed and current >= next_cloud:
                header = Imu().header
                header.stamp = node.get_clock().now().to_msg()
                header.frame_id = "left_mid360"
                c, s = math.cos(actual_yaw), math.sin(actual_yaw)
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, [(c*x+s*y, -s*x+c*y, z) for x,y,z in points]))
                next_cloud = current + .08
            executor.spin_once(timeout_sec=.001)

        try:
            assert client.wait_for_server(timeout_sec=5)
            sent = client.send_goal_async(Relocalize.Goal())
            deadline = time.monotonic()+5
            while not sent.done() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.01)
            assert sent.done() and sent.result().accepted
            result = sent.result().get_result_async()
            deadline = time.monotonic()+5
            while not any(s["stage"]=="SEARCHING" for s in states) and time.monotonic()<deadline and not result.done():
                step()
            assert any(s["stage"]=="SEARCHING" for s in states), states[-1:]
            search_start = time.monotonic()
            deadline = search_start+10
            while not result.done() and time.monotonic()<deadline:
                step(feed=not break_stream)
            assert result.done()
            reason = result.result().result.reason
            assert time.monotonic()-search_start > .3, "Fixture must exceed the frozen scan timeout"
            if break_stream:
                assert "stale" in reason, reason
            else:
                assert "stale" not in reason, reason
                assert "timeout" not in reason, reason
            assert commands[-1].angular.z == 0.
        finally:
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            output = process.communicate(timeout=5)[0]
            assert process.returncode == 0, output
