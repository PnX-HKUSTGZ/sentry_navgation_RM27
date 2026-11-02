#!/usr/bin/env python3
"""Demonstrate actual DDS history retention while a mapping executor is busy."""
import json
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def run():
    rclpy.init()
    node = rclpy.create_node('pointlio_imu_buffer_integration_test')
    shallow, buffered = [], []
    qos = lambda depth: QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                                  reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Imu, '/pointlio_buffer_test/imu',
                             lambda m: shallow.append(m.header.stamp.nanosec), qos(5))
    node.create_subscription(Imu, '/pointlio_buffer_test/imu',
                             lambda m: buffered.append(m.header.stamp.nanosec), qos(2000))
    publisher = node.create_publisher(Imu, '/pointlio_buffer_test/imu', qos(2000))
    try:
        deadline = time.monotonic()+5
        while publisher.get_subscription_count() < 2 and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert publisher.get_subscription_count() == 2
        # A synchronous registration/publish section prevents callback execution.
        for sequence in range(100):
            msg=Imu()
            msg.header.stamp.nanosec=sequence
            publisher.publish(msg)
            time.sleep(.01)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        assert shallow == list(range(95,100)), shallow
        assert buffered == list(range(100)), buffered
        result={'published_imu_samples':100,'executor_stall_sec':1,
                'default_depth_5_received':len(shallow),'depth_2000_received':len(buffered),
                'default_depth_5_sequence':shallow,'buffered_sequence_complete':True}
        print(json.dumps(result))
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    run()
