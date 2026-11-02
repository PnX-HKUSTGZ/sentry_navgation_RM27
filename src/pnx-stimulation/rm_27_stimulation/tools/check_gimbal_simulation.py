#!/usr/bin/env python3
"""Run against an isolated, running simulation; never publishes chassis velocity."""

import json
import argparse
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spin-radians', type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('check_sim_gimbal')
    tf = Buffer()
    listener = TransformListener(tf, node)
    latest = {}
    lidar = []
    node.create_subscription(String, '/sim/gimbal/state',
                             lambda msg: latest.update(json.loads(msg.data)), 1)
    node.create_subscription(Odometry, '/lidar_odometry', lambda msg: lidar.append(msg), 10)
    clients = {k: node.create_client(SetBool, '/sim/gimbal/set_'+k) for k in ('spin', 'upright')}

    def wait_for(predicate, description, timeout=35):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                print(description, json.dumps(latest), flush=True)
                return
        raise RuntimeError(description+' timed out: '+json.dumps(latest))

    def call(key, value, success=True):
        assert clients[key].wait_for_service(timeout_sec=5), key
        future = clients[key].call_async(SetBool.Request(data=value))
        wait_for(future.done, f'{key}={value}', 5)
        result = future.result()
        assert result.success == success, result.message

    def mount():
        t = tf.lookup_transform('base_link', 'left_mid360', Time()).transform
        return (t.translation.x, t.translation.y, t.translation.z,
                t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)

    wait_for(lambda: latest.get('connected') and latest.get('status') == 'Folded', 'Initial folded')
    call('spin', True, success=False)
    wait_for(lambda: tf.can_transform('base_link', 'left_mid360', Time()), 'Mount TF available')
    before = mount()
    call('upright', True)
    wait_for(lambda: latest.get('upright'), 'Head upright')
    after = mount()
    assert max(abs(a-b) for a,b in zip(before, after)) < .002, (before, after)
    call('spin', True)
    first_target = latest['yaw']+args.spin_radians
    wait_for(lambda: latest.get('yaw', 0) > first_target, 'Yaw rotates',
             max(35, args.spin_radians/.3+15))
    # The sensor odometry and TF must agree at acquisition time, including yaw.
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=.05)
    msg = lidar[-2]
    t = tf.lookup_transform('odom', 'left_mid360', Time.from_msg(msg.header.stamp)).transform
    p = msg.pose.pose.position
    error = math.sqrt((t.translation.x-p.x)**2+(t.translation.y-p.y)**2+(t.translation.z-p.z)**2)
    assert error < .001, error
    print('Scan-time lidar TF position error:', error, flush=True)
    call('spin', False)
    wait_for(lambda: latest.get('status') == 'Upright' and
             abs(math.atan2(math.sin(latest['yaw']), math.cos(latest['yaw']))) < .025,
             'Stop returns forward and stays upright')
    second_target = latest['yaw']+.6
    call('spin', True)
    wait_for(lambda: latest.get('yaw', 0) > second_target, 'Second rotation')
    call('upright', False)
    wait_for(lambda: latest.get('status') == 'Folded' and
             abs(math.atan2(math.sin(latest['yaw']), math.cos(latest['yaw']))) < .025
             and abs(latest['roll']) < .025, 'Automatic home then fold')
    call('spin', True, success=False)
    print('PASS: joint sequence, folding interlock, lidar mount and scan-time TF', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
