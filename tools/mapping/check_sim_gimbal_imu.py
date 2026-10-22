#!/usr/bin/env python3
"""Exercise articulated IMU / raw Point-LIO in an isolated Gazebo simulation."""

import argparse
from collections import deque
from functools import partial
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def matrix(position, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]).as_matrix()
    result[:3, 3] = [position.x, position.y, position.z]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.0,
                        help='Stationary rotation duration in simulation seconds')
    parser.add_argument('--output', type=Path, default=Path('/tmp/rm27_gimbal_imu_check.json'))
    args = parser.parse_args()
    if os.environ.get('GZ_PARTITION') != 'rmul2027-map-validation':
        raise RuntimeError('Use the isolated rmul2027-map-validation partition')
    rclpy.init()
    node = rclpy.create_node('check_sim_gimbal_imu', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    histories = {key: deque(maxlen=4000) for key in ('truth', 'lio', 'imu', 'joints')}
    for key, msg, topic in [('truth', Odometry, '/ground_truth/odometry'),
                            ('lio', Odometry, '/aft_mapped_to_init'),
                            ('imu', Imu, '/livox/imu'), ('joints', JointState, '/joint_states')]:
        node.create_subscription(msg, topic,
                                 partial(lambda queue, message: queue.append(message), histories[key]),
                                 qos_profile_sensor_data)
    state = {}
    node.create_subscription(String, '/sim/gimbal/state',
                             lambda msg: state.update(json.loads(msg.data)), 1)
    clients = {key: node.create_client(SetBool, '/sim/gimbal/set_' + key)
               for key in ('upright', 'spin')}
    commands = node.create_publisher(Twist, '/cmd_vel', 1)
    report = dict(passed=False, measurements=[])

    def wait(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise RuntimeError('Timed out waiting for simulation feedback')

    def call(key, value):
        assert clients[key].wait_for_service(timeout_sec=3)
        future = clients[key].call_async(SetBool.Request(data=value))
        wait(future.done)
        assert future.result().success, future.result().message

    def pair():
        # Allow joint TF to arrive, then compare at the LIO measurement time.
        cutoff = node.get_clock().now().nanoseconds * 1e-9 - .15
        odom = next(m for m in reversed(histories['lio']) if stamp(m) <= cutoff)
        truth = min(histories['truth'], key=lambda m: abs(stamp(m) - stamp(odom)))
        assert abs(stamp(truth) - stamp(odom)) < .025
        assert cutoff - stamp(odom) < 1.0, 'Point-LIO stopped producing fresh odometry'
        tf = buffer.lookup_transform('base_link', 'imu_link',
                                     Time.from_msg(odom.header.stamp)).transform
        world_sensor = matrix(truth.pose.pose.position, truth.pose.pose.orientation) @ matrix(tf.translation, tf.rotation)
        return world_sensor, matrix(odom.pose.pose.position, odom.pose.pose.orientation), cutoff + .15 - stamp(odom)

    def check(label, alignment):
        expected, estimated, age = pair()
        delta = np.linalg.inv(expected) @ alignment @ estimated
        error = float(np.linalg.norm(delta[:3, 3]))
        angle = float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))
        imu = histories['imu'][-1]
        joint = min(histories['joints'], key=lambda m: abs(stamp(m) - stamp(imu)))
        gyro = imu.angular_velocity
        norm = float(np.linalg.norm([gyro.x, gyro.y, gyro.z]))
        rate = abs(joint.velocity[joint.name.index('gimbal_yaw_joint')])
        result = dict(stage=label, sim_time=stamp(imu), raw_lio_error_m=error,
                      raw_lio_angle_deg=angle, sample_age_s=age,
                      gyro_norm_rad_s=norm, yaw_rate_rad_s=rate)
        report['measurements'].append(result)
        print(json.dumps(result), flush=True)
        assert error < .06 and angle < 2.0, result
        if rate > 1.0:
            assert abs(norm - rate) < .05, result

    def run_stage(label, duration, alignment, vx=0.0):
        start = node.get_clock().now().nanoseconds * 1e-9
        last_check = start
        last_command = start - 1.0
        deadline = time.monotonic() + max(60, duration * 12)
        while node.get_clock().now().nanoseconds * 1e-9 < start + duration:
            assert time.monotonic() < deadline, 'Simulation clock stalled'
            message = Twist()
            message.linear.x = float(vx)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now - last_command >= .02:
                commands.publish(message)
                last_command = now
            rclpy.spin_once(node, timeout_sec=.02)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now >= last_check + 1.0:
                check(label, alignment)
                last_check = now

    try:
        wait(lambda: all(histories.values()) and buffer.can_transform('base_link', 'imu_link', Time()))
        wait(lambda: len(histories['lio']) > 30)
        expected, estimated, _ = pair()
        alignment = expected @ np.linalg.inv(estimated)
        call('upright', True)
        wait(lambda: state.get('upright'))
        call('spin', True)
        run_stage('continuous_rotation', args.duration, alignment)
        run_stage('rotating_and_forward', 5, alignment, .12)
        run_stage('rotating_and_backward', 5, alignment, -.12)
        call('spin', False)
        run_stage('returning_home', 7, alignment)
        call('upright', False)
        run_stage('folding', 4, alignment)
        report['passed'] = True
    finally:
        commands.publish(Twist())
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        if clients['upright'].service_is_ready():
            call('upright', False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
