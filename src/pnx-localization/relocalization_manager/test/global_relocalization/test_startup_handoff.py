"""Real manager + coordinator protocol; inject an Action result, never a GT pose."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future
from geometry_msgs.msg import TransformStamped, Twist
from global_relocalization_interfaces.action import Relocalize
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, TransformException

spec = importlib.util.spec_from_file_location(
    'integrated_startup', Path(__file__).parents[2] / 'scripts/relocalize_then_navigate.py')
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


@pytest.mark.parametrize('success', [True, False])
def test_action_result_handoff_ack_and_navigation_gate(tmp_path, success):
    # The global numerical core is covered separately. Here a protocol peer
    # supplies success/failure to exercise the actual C++ handoff and Nav2 gate.
    pcd = tmp_path / 'map.pcd'
    points = [(x*.3, y*.3, z*.3) for x in range(5) for y in range(4) for z in range(3)]
    pcd.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                   f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'
                   + ''.join(f'{x} {y} {z}\n' for x, y, z in points))
    environment = dict(os.environ, ROS_DOMAIN_ID=str(194 if success else 195))
    process = subprocess.Popen([
        os.environ['RELOCALIZATION_TEST_EXECUTABLE'], '--ros-args',
        '-p', f'prior_pcd_file:={pcd}', '-p', 'wait_for_initial_pose:=true',
        '-p', 'base_frame:=base_footprint', '-p', 'robot_base_frame:=base_footprint',
        '-p', 'enable_stationary_refinement:=false',
    ], env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
       text=True, start_new_session=True)
    rclpy.init(domain_id=int(environment['ROS_DOMAIN_ID']))
    peer = rclpy.create_node('startup_protocol_peer')
    coordinator = startup.Startup(parameter_overrides=[Parameter('deployment', value='reality')])
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(peer)
    executor.add_node(coordinator)
    commands, navigation_requests, result_stamps, retirements = [], [], [], []
    peer.create_subscription(Twist, '/cmd_vel', commands.append, 10)
    command_pub = peer.create_publisher(Twist, '/relocalization/cmd_vel', 10)
    lidar = peer.create_publisher(PointCloud2, '/livox/lidar', 10)
    imu = peer.create_publisher(Imu, '/livox/imu', 10)
    odom = peer.create_publisher(Odometry, '/odometry', 10)
    broadcaster = TransformBroadcaster(peer)
    completion = Future()
    group = ReentrantCallbackGroup()

    def map_state(_request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        return response

    def start_match(_request, response):
        completion.set_result(True)
        response.success = True
        return response

    def retire(_request, response):
        assert coordinator.state == "RETIRING_GLOBAL"
        assert coordinator.command_pub is coordinator.command_sub is None
        retirements.append(True)
        response.success = True
        return response

    def navigation(request, response):
        assert retirements == [True]
        navigation_requests.append(request.command)
        assert coordinator.state == 'STARTING_NAVIGATION'
        assert coordinator.command_pub is coordinator.command_sub is None
        response.success = True
        return response

    async def execute(goal):
        assert not navigation_requests
        command = Twist()
        command.angular.z = .2
        command_pub.publish(command)
        goal.publish_feedback(Relocalize.Feedback(stage='ARMED'))
        await completion
        reply = Relocalize.Result()
        reply.success = success
        reply.reason = 'confirmed' if success else 'ambiguous candidate'
        reply.pose.header.frame_id = 'map'
        reply.pose.header.stamp = (peer.get_clock().now() - Duration(seconds=1.)).to_msg()
        result_stamps.append(reply.pose.header.stamp)
        reply.pose.pose.position.x = 10.
        reply.pose.pose.position.y = 4.
        reply.pose.pose.orientation.w = 1.
        if success:
            goal.succeed()
        else:
            goal.abort()
        return reply

    peer.create_service(GetState, '/map_server/get_state', map_state)
    peer.create_service(Trigger, '/global_relocalization/start_match', start_match,
                        callback_group=group)
    peer.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', navigation)
    peer.create_service(Trigger, "/global_relocalization/retire", retire)
    server = ActionServer(peer, Relocalize, '/global_relocalization/run', execute,
                          callback_group=group)
    started = time.monotonic()
    observed_transform = None
    try:
        while time.monotonic()-started < 15:
            stamp = peer.get_clock().now().to_msg()
            cloud = PointCloud2()
            cloud.header.stamp = stamp
            lidar.publish(cloud)
            measurement = Imu()
            measurement.header.stamp = stamp
            imu.publish(measurement)
            measurement = Odometry()
            measurement.header.stamp = stamp
            measurement.header.frame_id = 'odom'
            measurement.child_frame_id = 'base_footprint'
            measurement.pose.pose.orientation.w = 1.
            odom.publish(measurement)
            transforms = []
            for offset in (-2., 0.):
                transform = TransformStamped()
                transform.header.stamp = (peer.get_clock().now() + Duration(seconds=offset)).to_msg()
                transform.header.frame_id = 'odom'
                transform.child_frame_id = 'base_footprint'
                transform.transform.translation.x = 1.
                transform.transform.rotation.w = 1.
                transforms.append(transform)
            broadcaster.sendTransform(transforms)
            executor.spin_once(timeout_sec=.02)
            # The lifecycle reply and map->odom arrive on independent DDS
            # streams. Observe the actual transform within the same deadline;
            # an arbitrary post-READY sleep is not a delivery barrier.
            if coordinator.state == 'READY':
                try:
                    observed_transform = coordinator.tf_buffer.lookup_transform(
                        'map', 'odom', rclpy.time.Time())
                    break
                except TransformException:
                    pass
            elif coordinator.state == 'FAILED' and commands and commands[-1].angular.z == 0.:
                break
            time.sleep(.005)
        assert coordinator.state == ('READY' if success else 'FAILED')
        assert bool(navigation_requests) == success
        assert bool(retirements) == success
        assert commands and all(c.linear.x == c.linear.y == 0. for c in commands)
        if success:
            assert coordinator.seed.header.stamp == result_stamps[0]
            assert observed_transform is not None
            assert observed_transform.transform.translation.x == pytest.approx(9.)
            assert observed_transform.transform.translation.y == pytest.approx(4.)
        else:
            assert coordinator.seed is None
            assert coordinator.command_pub is not None
            assert commands[-1].angular.z == 0.
    finally:
        executor.shutdown()
        server.destroy()
        coordinator.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        output = process.communicate(timeout=5)[0]
        assert process.returncode == 0, output


def test_coordinator_exits_cleanly_on_sigint():
    process = subprocess.Popen(
        [sys.executable, str(Path(startup.__file__)), '--ros-args', '-p', 'deployment:=reality'],
        env=dict(os.environ, ROS_DOMAIN_ID='193'), stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        # The first status is emitted after rclpy installed its signal handlers.
        output = process.stdout.readline()
        assert 'STARTING_MAP' in output
        time.sleep(.1)
        os.killpg(process.pid, signal.SIGINT)
        output += process.communicate(timeout=5)[0]
        assert process.returncode == 0, output
        assert 'Traceback' not in output
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
