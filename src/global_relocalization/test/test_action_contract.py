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
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from global_relocalization_interfaces.action import Relocalize


# Each ROS context gets its own DDS domain. Reusing a domain immediately after
# shutdown can expose stale action discovery from the previous parametrized case.
import itertools
_domain_ids = itertools.count(200)
def _isolated_domain():
    value = next(_domain_ids)
    assert value < 220
    return str(value)


@pytest.fixture(autouse=True)
def controlled_ros_clock():
    """Keep ROS now and sensor stamps on the same monotonic-derived clock.

    WSL may step CLOCK_REALTIME during a test; production correctly rejects the
    resulting source gap. This fixture isolates algorithm contract tests from
    host wall-clock adjustments without changing any production coverage gate.
    """
    os.environ["ROS_DOMAIN_ID"] = _isolated_domain()
    clock_source = subprocess.Popen(["/usr/bin/python3", "-c", """
import time
import rclpy
from rosgraph_msgs.msg import Clock
rclpy.init()
node = rclpy.create_node('contract_monotonic_clock')
pub = node.create_publisher(Clock, '/clock', 10)
origin_ros = time.time_ns()
origin_steady = time.monotonic_ns()
try:
    while rclpy.ok():
        stamp = origin_ros + time.monotonic_ns() - origin_steady
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(stamp, 1_000_000_000)
        pub.publish(message)
        rclpy.spin_once(node, timeout_sec=0)
        time.sleep(.005)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
"""], env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        yield
    finally:
        if clock_source.poll() is None: clock_source.send_signal(signal.SIGINT)
        clock_source.wait(timeout=5)


def _test_node(name, context):
    return rclpy.create_node(name, context=context,
        parameter_overrides=[Parameter('use_sim_time', value=True)])


def _wait_for_clock(node, executor):
    deadline = time.monotonic() + 5
    while node.get_clock().now().nanoseconds == 0 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)
    assert node.get_clock().now().nanoseconds > 0, 'controlled /clock was not received'


def _wait_for_server_clock(states, executor):
    # The client receiving /clock does not prove the server has received it.
    deadline = time.monotonic() + 5
    while not any(s.get('status_ros_ns', 0) > 0 for s in states) and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)
    assert any(s.get('status_ros_ns', 0) > 0 for s in states), 'server /clock was not received'


def test_cancel_retry_timeout_keeps_motor_stopped():
    binary = Path(os.environ["GLOBAL_RELOCALIZATION_NODE"])
    # The test has its own DDS domain and never connects to a robot/simulation.
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "map.pcd"
        points = [(x * .3, y * .3, z * .3) for x in range(5) for y in range(4) for z in range(3)]
        path.write_text(
            "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
            + "".join(f"{x} {y} {z}\n" for x, y, z in points)
        )
        process = subprocess.Popen(
            [str(binary), "--ros-args", "-p", "use_sim_time:=true", "-p", f"map_path:={path}", "-p", "timeout_sec:=1.0",
             "-p", "settle_sec:=0.1"],
            env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True,
        )
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _test_node("global_relocalization_contract_test", context)
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
            _wait_for_clock(node, executor)
            expected_core = os.environ.get("GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY")
            if expected_core:
                mappings = Path(f"/proc/{process.pid}/maps").read_text()
                assert str(Path(expected_core).resolve()) in mappings, mappings
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


@pytest.mark.parametrize("numerical_recovery", [False, True])
def test_raw_imu_calibration_worker_cancel_and_reset(numerical_recovery):
    """Raw mode operates without odometry, and retries require fresh calibration."""
    import json
    import numpy as np
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    binary = Path(os.environ["GLOBAL_RELOCALIZATION_NODE"])
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "map.pcd"
        points = [(x * .2, y * .2, z * .2) for x in range(10) for y in range(10) for z in range(5)]
        path.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
                        + "".join(f"{x} {y} {z}\n" for x, y, z in points))
        process = subprocess.Popen([str(binary), "--ros-args", "-p", "use_sim_time:=true", "-p", f"map_path:={path}",
            "-p", "input_mode:=raw_imu", "-p", "raw_calibration_sec:=0.4",
            "-p", "settle_sec:=0.1", "-p", "timeout_sec:=4.0"] + (["-p", "raw_numerical_recovery:=true", "-p", "raw_deterministic:=true", "-p", "raw_local_max_iterations:=1", "-p", "raw_local_retry_iterations:=0"] if numerical_recovery else []), env=os.environ.copy(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _test_node("raw_relocalization_contract_test", context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, Relocalize, "/global_relocalization/run")
        commands, states, poses = [], [], []
        command_events, state_events = [], []
        def command_event(message):
            commands.append(message); command_events.append((time.monotonic(), message.angular.z))
        node.create_subscription(Twist, "/cmd_vel", command_event, 10)
        node.create_subscription(String, "/global_relocalization/status", lambda m: (states.append(json.loads(m.data)), state_events.append((time.monotonic(), json.loads(m.data)))), 10)
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

        # One 250k-point burst exercises conversion, transformation and the
        # accumulated-cloud voxel filter while the independent producer keeps IMU continuous.
        from std_msgs.msg import Header
        large_points = np.tile(np.asarray(points, dtype=np.float32), (500, 1))
        if numerical_recovery: large_points += np.asarray([.005, .002, .001], dtype=np.float32)
        large_cloud = point_cloud2.create_cloud_xyz32(Header(), large_points)
        burst_sent = False

        imu_process = subprocess.Popen(["/usr/bin/python3", "-c", """
import rclpy
from sensor_msgs.msg import Imu
rclpy.init()
from rclpy.parameter import Parameter
node = rclpy.create_node('continuous_imu_contract_source', parameter_overrides=[Parameter('use_sim_time', value=True)])
while node.get_clock().now().nanoseconds == 0:
    rclpy.spin_once(node, timeout_sec=.01)
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
            _wait_for_clock(node, executor)
            expected_core = os.environ.get("GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY")
            if expected_core:
                mappings = Path(f"/proc/{process.pid}/maps").read_text()
                assert str(Path(expected_core).resolve()) in mappings, mappings
            deadline = time.monotonic()+5
            while node.count_publishers("/livox/imu")==0 and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.01)
            assert node.count_publishers("/livox/imu") > 0, "Independent continuous source never became ready"
            first = wait(client.send_goal_async(Relocalize.Goal()))
            assert first.accepted
            active_result = first.get_result_async()
            _wait_for_server_clock(states, executor)
            feed(2.2)
            assert burst_sent, json.dumps({"state": states[-1:] , "reason": active_result.result().result.reason if active_result.done() else None, "imu_source_exit": imu_process.poll()})
            assert any(m.angular.z != 0 for m in commands)
            assert not active_result.done(), "Continuous input was rejected during a processing burst"
            assert any(s.get("raw_frontend", {}).get("gravity_initialized") for s in states)
            if numerical_recovery:
                recovering = [(t,s) for t,s in state_events if s["stage"] == "RECOVERING"]
                assert recovering, states[-3:]
                stopped_at = recovering[0][0]
                assert any(v == 0 for t,v in command_events if t >= stopped_at - .05)
                resumed = [(t,s) for t,s in state_events if t > stopped_at and s["stage"] == "SPINNING" and s.get("raw_frontend",{}).get("recovered_scan_count") == 1]
                assert resumed, states[-3:]
                assert any(v == 0 for t,v in command_events if stopped_at - .05 <= t <= resumed[0][0])
                assert states[-1]["raw_frontend"]["recovery_attempts"] == 1
                assert resumed[0][1]["stage_wall_sec"]["SPINNING"] > 0
                assert resumed[0][1]["stage_wall_sec"]["RECOVERING"] > 0
            assert wait(first.cancel_goal_async()).goals_canceling
            assert wait(first.get_result_async()).status == 5
            feed(.1)
            assert commands[-1].angular.z == 0
            final_raw = next(s["raw_frontend"] for s in reversed(states) if s["stage"]=="CANCELED")
            if numerical_recovery:
                terminal = next(s for s in reversed(states) if s["stage"] == "CANCELED")
                assert terminal["stage_wall_sec"]["SPINNING"] > resumed[0][1]["stage_wall_sec"]["SPINNING"]
                assert terminal["elapsed_sec"] >= sum(terminal["stage_wall_sec"].values()) - .05
            assert final_raw["imu_rejected"] == 0
            assert final_raw["imu_max_received_stamp_gap_sec"] <= .011
            assert final_raw["imu_accepted"] >= 120
            assert final_raw["max_worker_wall_sec"] > 0
            assert final_raw["queue_dropped"] == 0
            assert len(large_cloud.data) <= final_raw["max_scan_message_bytes"]
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
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / "map.pcd"
        points = [(x*.2, y*.2, z*.2) for x in range(10) for y in range(10) for z in range(5)]
        path.write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
                        + "".join(f"{x} {y} {z}\n" for x, y, z in points))
        process = subprocess.Popen([str(binary), "--ros-args", "-p", "use_sim_time:=true", "-p", f"map_path:={path}",
            "-p", "input_mode:=raw_imu", "-p", "raw_calibration_sec:=0.4",
            "-p", "settle_sec:=0.1", "-p", "spin_angle:=0.05", "-p", "timeout_sec:=12.0",
            "-p", "data_timeout_sec:=0.3", "-p", "db_grid_resolution:=0.015",
            "-p", "db_min_points:=80"], env=os.environ.copy(), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _test_node("raw_search_health_test", context)
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
            # Model a continuously sampled 100 Hz sensor. Python scheduling
            # delay buffers samples at their measured stamps rather than making
            # a false physical IMU gap in this transport/freshness test.
            current_ros_ns = node.get_clock().now().nanoseconds
            while feed and current >= next_imu:
                imu = Imu()
                imu.header.stamp = rclpy.time.Time(nanoseconds=current_ros_ns - int((current-next_imu)*1e9)).to_msg()
                imu.header.frame_id = "left_mid360"
                imu.angular_velocity.z = rate
                imu.linear_acceleration.z = 9.81
                imu_pub.publish(imu)
                next_imu += .01
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
            _wait_for_clock(node, executor)
            expected_core = os.environ.get("GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY")
            if expected_core:
                mappings = Path(f"/proc/{process.pid}/maps").read_text()
                assert str(Path(expected_core).resolve()) in mappings, mappings
            goal_state_index = len(states)
            sent = client.send_goal_async(Relocalize.Goal())
            deadline = time.monotonic()+5
            while not sent.done() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.01)
            assert sent.done() and sent.result().accepted
            result = sent.result().get_result_async()
            # A client clock sample does not establish the server clock. Wait
            # for this action's own status, sent from the server clock, before
            # publishing epoch-stamped sensor data into the future-stamp gate.
            deadline = time.monotonic()+5
            active_run = None
            server_ready = False
            while time.monotonic()<deadline:
                run_states = states[goal_state_index:]
                if active_run is None and run_states:
                    active_run = run_states[0]['run_id']
                server_ready = (active_run is not None and
                    any(s['run_id']==active_run and s['stage']=='SETTLING' and s['status_ros_ns']>0 for s in run_states) and
                    imu_pub.get_subscription_count()>0 and cloud_pub.get_subscription_count()>0 and
                    node.count_subscribers('/tf_static')>0)
                if server_ready:
                    break
                executor.spin_once(timeout_sec=.005)
            assert server_ready, json.dumps({'states':states[-3:], 'action_done':result.done(),
                'action_reason':result.result().result.reason if result.done() else None})
            next_imu = next_cloud = previous = time.monotonic()
            deadline = time.monotonic()+5
            while not any(s["stage"]=="SEARCHING" for s in states) and time.monotonic()<deadline and not result.done():
                step()
            if not any(s['run_id']==active_run and s['stage']=='SEARCHING' for s in states) and result.done():
                # Preserve the actual terminal reason even if its status topic
                # callback is scheduled after the action result callback.
                deadline = time.monotonic()+1
                while not any(s['run_id']==active_run and s['stage'] in ('FAILED','CANCELED') for s in states) and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.005)
            assert any(s['run_id']==active_run and s['stage']=='SEARCHING' for s in states), json.dumps({
                'states':states[-3:], 'action_done':result.done(),
                'action_reason':result.result().result.reason if result.done() else None})
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

@pytest.mark.parametrize('fault', ['oversize', 'overflow'])
def test_raw_ingress_limits_abort_and_publish_zero(fault):
    """The actual server surfaces acquisition faults; it never drops an old frame."""
    import json
    from std_msgs.msg import String
    binary = Path(os.environ['GLOBAL_RELOCALIZATION_NODE'])
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'map.pcd'
        points = [(x * .3, y * .3, z * .3) for x in range(5) for y in range(4) for z in range(3)]
        path.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'
                        + ''.join(f'{x} {y} {z}\n' for x, y, z in points))
        process = subprocess.Popen([str(binary), '--ros-args', '-p', 'use_sim_time:=true', '-p', f'map_path:={path}',
            '-p', 'input_mode:=raw_imu', '-p', 'timeout_sec:=5.0'], env=os.environ.copy(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _test_node('raw_ingress_limits_contract', context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = ActionClient(node, Relocalize, '/global_relocalization/run')
        commands, states = [], []
        node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
        node.create_subscription(String, '/global_relocalization/status', lambda m: states.append(json.loads(m.data)), 10)
        cloud_pub = node.create_publisher(PointCloud2, '/livox/lidar', 40)

        def wait(future):
            deadline = time.monotonic() + 6
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert future.done()
            return future.result()

        try:
            assert client.wait_for_server(timeout_sec=5)
            _wait_for_clock(node, executor)
            expected_core = os.environ.get("GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY")
            if expected_core:
                mappings = Path(f"/proc/{process.pid}/maps").read_text()
                assert str(Path(expected_core).resolve()) in mappings, mappings
            deadline = time.monotonic() + 3
            while not cloud_pub.get_subscription_count() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            assert cloud_pub.get_subscription_count()
            goal = wait(client.send_goal_async(Relocalize.Goal()))
            assert goal.accepted
            result_future = goal.get_result_async()
            _wait_for_server_clock(states, executor)
            for _ in range(1 if fault == 'oversize' else 33):
                header = Odometry().header
                header.stamp = node.get_clock().now().to_msg()
                header.frame_id = 'left_mid360'
                cloud = point_cloud2.create_cloud_xyz32(header, points)
                if fault == 'oversize':
                    cloud.data = bytes(4 * 1024 * 1024 + 1)
                cloud_pub.publish(cloud)
                deadline = time.monotonic() + .01
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.001)
            result = wait(result_future)
            assert not result.result.success
            assert ('4 MiB' if fault == 'oversize' else 'FIFO overflow') in result.result.reason
            deadline = time.monotonic() + .1
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
            terminal = next(s for s in reversed(states) if s['stage'] == 'FAILED')
            raw = terminal['raw_frontend']
            assert raw['queue_dropped'] == 0
            assert raw['oversized_scan_count' if fault == 'oversize' else 'queue_overflow_count'] == 1
            assert commands and commands[-1].angular.z == 0
        finally:
            node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            output = process.communicate(timeout=5)[0]
            assert process.returncode == 0, output


@pytest.mark.parametrize('enabled', [False, True])
def test_visualization_is_opt_in_and_cloud_snapshots_are_bounded(enabled):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from visualization_msgs.msg import MarkerArray
    binary = Path(os.environ['GLOBAL_RELOCALIZATION_NODE'])
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'map.pcd'
        points = [(x * .2, y * .2, z * .2) for x in range(10) for y in range(10) for z in range(4)]
        path.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'
                        + ''.join(f'{x} {y} {z}\n' for x, y, z in points))
        arguments = [str(binary), '--ros-args', '-p', 'use_sim_time:=true', '-p', f'map_path:={path}',
                     '-p', 'timeout_sec:=0.8', '-p', 'settle_sec:=0.1']
        if enabled:
            arguments += ['-p', 'visualization_enabled:=true', '-p', 'visualization_max_points:=100']
        process = subprocess.Popen(arguments, env=os.environ.copy(), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _test_node('observer_contract_test', context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        clouds, markers = [], []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(PointCloud2, '/global_relocalization/visual/map', clouds.append, qos)
        node.create_subscription(MarkerArray, '/global_relocalization/visual/markers', markers.append, qos)
        client = ActionClient(node, Relocalize, '/global_relocalization/run')
        try:
            assert client.wait_for_server(timeout_sec=5)
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and (enabled and not clouds):
                executor.spin_once(timeout_sec=.02)
            if enabled:
                assert clouds and clouds[-1].width <= 100
                assert clouds[-1].header.frame_id == 'map'
            else:
                assert not node.get_publishers_info_by_topic('/global_relocalization/visual/map')
                assert not node.get_publishers_info_by_topic('/global_relocalization/visual/markers')
            _wait_for_clock(node, executor)
            future = client.send_goal_async(Relocalize.Goal())
            deadline = time.monotonic() + 4
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            handle = future.result()
            assert handle.accepted
            result = handle.get_result_async()
            deadline = time.monotonic() + 4
            while not result.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            assert result.done() and not result.result().result.success
            if enabled:
                # Observer publication runs on its own timer after action completion.
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline and not any(
                        'FAILED' in marker.text for message in markers for marker in message.markers):
                    executor.spin_once(timeout_sec=.02)
                assert markers
                assert any('FAILED' in marker.text for message in markers for marker in message.markers)
            assert process.poll() is None
        finally:
            client.destroy()
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            context.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=5)
