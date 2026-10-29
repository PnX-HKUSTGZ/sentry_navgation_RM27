#!/usr/bin/env python3
"""Reproduce RViz placement and navigation, recording timestamped pose evidence.

Run in an isolated ROS domain with the normal global navigation launch running.
Truth is used only by this observer, never to initialize localization.
"""
import argparse
import json
import math
import os
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from observe import Observer, seconds, write_json


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--spawn', default='[-5.935,-0.946,0.0]')
    parser.add_argument('--goals', default='[[7.44,1.673],[-7.231,1.55]]')
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--wall-timeout', type=float, default=240.)
    parser.add_argument('--capture-scans', action='store_true')
    parser.add_argument('--goal-dwell', type=float, default=0.,
                        help='Simulation seconds to observe standstill after each successful goal')
    args = parser.parse_args()
    args.mode = 'full_chain'
    os.makedirs(args.output, exist_ok=False)
    rclpy.init()
    node = Observer(args)
    state = {}
    history = deque(maxlen=4000)
    report = {'states': [], 'pose_matches': []}
    scans, truth_samples = deque(maxlen=800), []
    if args.capture_scans:
        import numpy as np
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py.point_cloud2 import read_points_numpy
        def scan(msg):
            # Keep the most recent 80 seconds so a late localization failure
            # is retained instead of exhausting storage during startup.
            points = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            scans.append((seconds(msg.header.stamp), points[::10].copy()))
        def truth_sample(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            truth_samples.append([seconds(msg.header.stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        node.create_subscription(PointCloud2, '/livox/lidar', scan, qos_profile_sensor_data)
        node.create_subscription(Odometry, '/ground_truth/odometry', truth_sample, qos_profile_sensor_data)
    def status(msg):
        state.update(json.loads(msg.data))
        report['states'].append(dict(state))
        write_json(os.path.join(args.output, 'startup.json'), report)
        print('STARTUP', state, flush=True)
    def pose(msg):
        stamp = seconds(msg.header.stamp)
        if history:
            truth = min(history, key=lambda m: abs(seconds(m.header.stamp)-stamp))
            p, t = msg.pose.position, truth.pose.pose.position
            error = math.atan2(math.sin(yaw(msg.pose.orientation)-yaw(truth.pose.pose.orientation)),
                               math.cos(yaw(msg.pose.orientation)-yaw(truth.pose.pose.orientation)))
            row = {'stamp': stamp, 'truth_stamp_gap': abs(seconds(truth.header.stamp)-stamp),
                   'estimate_xyz': [p.x,p.y,p.z], 'truth_xyz': [t.x,t.y,t.z],
                   'estimate_yaw': yaw(msg.pose.orientation), 'truth_yaw': yaw(truth.pose.pose.orientation),
                   'xy_error': math.hypot(p.x-t.x,p.y-t.y), 'yaw_error': error,
                   'quaternion': [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]}
            report['pose_matches'].append(row)
            write_json(os.path.join(args.output, 'startup.json'), report)
            print('POSE', row, flush=True)
    node.create_subscription(String, '/relocalize_then_navigate/status', status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry, '/ground_truth/odometry', lambda m: history.append(m), qos_profile_sensor_data)
    node.create_subscription(PoseStamped, '/global_relocalization/pose', pose, 10)
    status_stream = open(os.path.join(args.output, 'relocalization.jsonl'), 'w')
    node.create_subscription(String, '/global_relocalization/status', lambda m: status_stream.write(m.data+'\n'), 10)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    def wait(predicate):
        deadline = time.monotonic()+args.wall_timeout
        while not predicate():
            if state.get('state') == 'FAILED' or time.monotonic()>deadline:
                raise RuntimeError(f'Startup failed or timed out: {state}')
            rclpy.spin_once(node, timeout_sec=.05)
    try:
        wait(lambda: state.get('state') == 'WAITING_FOR_INITIAL_POSE' and pub.get_subscription_count()>0)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        x,y,a = json.loads(args.spawn)
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x),float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(a/2),math.cos(a/2)
        pub.publish(msg)
        wait(lambda: state.get('state') == 'READY')
        node.runtime(time.monotonic()+30)
        start = node.clock_time
        wait(lambda: node.clock_time-start >= 3.)
        report['ready_truth_yaw'] = yaw(node.truth.pose.pose.orientation)
        report['goals'] = []
        report['stationary_dwells'] = []
        for i, goal in enumerate(json.loads(args.goals), 1):
            ok = node.run_goal(i, goal)
            report['goals'].append(ok)
            write_json(os.path.join(args.output, 'startup.json'), report)
            if not ok:
                raise RuntimeError(f'Goal {i} failed')
            if args.goal_dwell > 0:
                node.run = {'goal': goal, 'active': False, 'samples': [], 'events': []}
                start = node.clock_time
                wait(lambda: node.clock_time - start >= args.goal_dwell)
                dwell = node.run
                node.run = None
                write_json(os.path.join(args.output, f'dwell_{i:02d}.json'), dwell)
                report['stationary_dwells'].append({'goal_index': i, 'duration': args.goal_dwell})
        report['passed'] = True
    except (RuntimeError, KeyboardInterrupt) as exc:
        report.update(passed=False, error=str(exc) or type(exc).__name__)
        raise
    finally:
        write_json(os.path.join(args.output, 'startup.json'), report)
        if args.capture_scans:
            np.savez_compressed(os.path.join(args.output, 'scan_samples.npz'),
                                truth=np.asarray(truth_samples), stamps=np.asarray([s for s, _ in scans]),
                                **{f'scan_{i}': p for i, (_, p) in enumerate(scans)})
        status_stream.close()
        node.audit_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
