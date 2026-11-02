#!/usr/bin/env python3
"""Wait for the physics loop before inserting a robot into a CAD world."""

import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    rclpy.init()
    node = rclpy.create_node('wait_for_sim_clock')
    ready = [False]
    node.create_subscription(Clock, '/clock',
                             lambda msg: ready.__setitem__(0, msg.clock.sec > 0 or msg.clock.nanosec > 0),
                             qos_profile_sensor_data)
    timeout = node.declare_parameter('timeout', 180.0).value
    started = time.monotonic()
    deadline = started+timeout
    next_report = started+10.0
    node.get_logger().info('Waiting for Gazebo physics before spawning the robot')
    try:
        while rclpy.ok() and not ready[0] and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
            now = time.monotonic()
            if not ready[0] and now >= next_report:
                node.get_logger().info(
                    f'Gazebo is still loading: {now-started:.0f}s elapsed; waiting for physics clock. '
                    'If the world was started paused, press Play to continue.')
                next_report = now+10.0
        if not ready[0]:
            raise RuntimeError('Gazebo did not start its physics clock before the timeout')
        node.get_logger().info('Gazebo clock is ready')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
