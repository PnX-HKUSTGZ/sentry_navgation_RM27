#!/usr/bin/env python3
"""Exercise live Point-LIO/GICP and Nav2; ground truth is scoring only."""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import re
import time

import numpy as np
import psutil
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from generate_rmuc2027_pcd import digest, read_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--startup-timeout', type=float, default=100)
    parser.add_argument('--partition', default='rmuc2027-world-validation-20261105',
        help='Expected isolated Gazebo partition; must match GZ_PARTITION')
    parser.add_argument('--routes', type=Path, help='JSON list of x/y goals and optional timeout seconds')
    args = parser.parse_args()
    if not args.partition.startswith('rmuc2027-') or os.environ.get('GZ_PARTITION') != args.partition:
        raise RuntimeError('Use the explicitly selected isolated rmuc2027- partition')
    rclpy.init()
    node = rclpy.create_node('check_rmuc2027_navigation',
        parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    state = dict(localized=False, acceptances=0, rejections=0)
    truths = deque(maxlen=500)
    scans = deque(maxlen=30)
    source = cKDTree(read_pcd(args.pcd))
    report = dict(passed=False, ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
        gz_partition=os.environ.get('GZ_PARTITION'), pcd_sha256=digest(args.pcd),
        localization_source='Point-LIO + local GICP; configured spawn seed',
        ground_truth_used_for_estimation_or_control=False, measurements=[], goals=[])
    initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    goal_handle = None

    def logging(message):
        if message.name != 'relocalization_manager':
            return
        text = re.sub(r'\x1b\[[0-9;]*m', '', message.msg)
        if 'GICP converge successfully' in text:
            state['acceptances'] += 1
        if 'GICP converged but rejected' in text or 'GICP did not converge' in text:
            state['rejections'] += 1
        if ' -> LOCALIZED ' in text or 'Localization state: LOCALIZED' in text:
            state['localized'] = True
        if ' -> LOCALIZING ' in text:
            state['localized'] = False

    node.create_subscription(Log, '/rosout', logging, qos_profile_sensor_data)
    def truth_received(message):
        truths.append(message)
        if state.get('goal_trace') is not None:
            p = message.pose.pose.position
            state['goal_trace'].append([p.x, p.y, p.z])

    node.create_subscription(Odometry, '/ground_truth/odometry', truth_received, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/registered_scan',
        lambda m: scans.append(m), qos_profile_sensor_data)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)

    def wait(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.03)
            if predicate():
                return
        raise RuntimeError('Timeout: '+label)

    def measure(label):
        errors = []
        for _ in range(30):
            spin(.1)
            if not truths:
                continue
            truth = next((m for m in reversed(truths)
                if buffer.can_transform('map', 'base_footprint', Time.from_msg(m.header.stamp))), None)
            if truth is None:
                continue
            stamp = Time.from_msg(truth.header.stamp)
            if (node.get_clock().now()-stamp).nanoseconds > 1_000_000_000:
                continue
            tf = buffer.lookup_transform('map', 'base_footprint', stamp).transform
            actual = truth.pose.pose
            p, q, t = tf.translation, tf.rotation, actual.position
            a = actual.orientation
            angle = (Rotation.from_quat([q.x,q.y,q.z,q.w]).inv()*
                     Rotation.from_quat([a.x,a.y,a.z,a.w])).magnitude()
            errors.append([math.hypot(p.x-t.x,p.y-t.y), abs(p.z-t.z), math.degrees(angle)])
        if len(errors) < 5:
            raise RuntimeError('Insufficient same-timestamp TF/truth pairs: '+label)
        median = np.median(errors, axis=0)
        result = dict(stage=label, samples=len(errors), xy_error_m=float(median[0]),
            z_error_m=float(median[1]), orientation_error_deg=float(median[2]))
        scan = next((m for m in reversed(scans) if buffer.can_transform(
            'map', m.header.frame_id, Time.from_msg(m.header.stamp))), None)
        if scan is not None:
            transform = buffer.lookup_transform('map', scan.header.frame_id,
                Time.from_msg(scan.header.stamp)).transform
            q, p = transform.rotation, transform.translation
            xyz = read_points_numpy(scan, field_names=('x','y','z'), skip_nans=True)[::4]
            xyz = xyz @ Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix().T + [p.x,p.y,p.z]
            distances, _ = source.query(xyz, workers=2)
            result['live_scan_to_pcd'] = dict(points=len(xyz),
                median_m=float(np.median(distances)), p90_m=float(np.quantile(distances,.9)),
                within_10cm=float(np.mean(distances <= .1)))
        report['measurements'].append(result)
        print(json.dumps(result), flush=True)
        if not (median[0] < .08 and median[1] < .05 and median[2] < 3):
            raise RuntimeError('Pose accuracy check failed: '+label)

    started = time.monotonic()
    try:
        wait(lambda: state['localized'] and state['acceptances'] >= 3 and truths
             and client.server_is_ready(), args.startup_timeout, 'GICP localization and Nav2')
        simulators = []
        for process in psutil.process_iter(['pid', 'cmdline']):
            try:
                if (process.info['cmdline'] and process.info['cmdline'][0].startswith('gz sim ')
                        and process.environ().get('GZ_PARTITION') == os.environ['GZ_PARTITION']):
                    simulators.append(process.pid)
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                continue
        report['gazebo_server_pids'] = simulators
        if len(simulators) != 1 or node.count_publishers('/ground_truth/odometry') != 1:
            raise RuntimeError('Expected exactly one isolated simulator and truth publisher')
        report['initial_localization_wall_sec'] = time.monotonic()-started
        measure('initial_localization')
        # Perturb the current estimate, never use the scorer's truth as a seed.
        tf = buffer.lookup_transform('map','base_footprint',Time()).transform
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = 'map'
        seed.header.stamp = node.get_clock().now().to_msg()
        seed.pose.pose.position.x = tf.translation.x+.15
        seed.pose.pose.position.y = tf.translation.y-.10
        seed.pose.pose.position.z = tf.translation.z
        q = tf.rotation
        perturbed = Rotation.from_euler('z',3,degrees=True)*Rotation.from_quat([q.x,q.y,q.z,q.w])
        target = seed.pose.pose.orientation
        target.x,target.y,target.z,target.w = perturbed.as_quat().tolist()
        before = state['acceptances']
        state['localized'] = False
        initial.publish(seed)
        wait(lambda: state['localized'] and state['acceptances'] >= before+3, 45,
             'relocalization from +15 cm, -10 cm, +3 degrees')
        measure('perturbed_seed_relocalized')
        routes = (json.loads(args.routes.read_text()) if args.routes else
                  [dict(x=-10., y=3.1), dict(x=-11.7, y=3.1)])
        for route in routes:
            x, y = float(route['x']), float(route['y'])
            state['goal_trace'] = []
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y = x,y
            goal.pose.pose.orientation.w = 1.
            future = client.send_goal_async(goal)
            wait(future.done, 10, 'goal acceptance')
            goal_handle = future.result()
            if not goal_handle.accepted:
                raise RuntimeError('Navigation goal rejected')
            result = goal_handle.get_result_async()
            wait(result.done, float(route.get('timeout', 75)), 'navigation completion')
            status = result.result().status
            truth = truths[-1].pose.pose.position
            error = math.hypot(truth.x-x,truth.y-y)
            trace = np.asarray(state.pop('goal_trace'))
            entry = dict(target_xy=[x,y], status=status, truth_goal_error_m=error,
                truth_path_length_m=float(np.linalg.norm(np.diff(trace[:,:2], axis=0), axis=1).sum()) if len(trace) > 1 else 0.,
                truth_base_z_range_m=[float(trace[:,2].min()),float(trace[:,2].max())] if len(trace) else [])
            report['goals'].append(entry)
            print(json.dumps(entry), flush=True)
            goal_handle = None
            if status != 4 or error > .25:
                raise RuntimeError('Navigation goal did not succeed accurately')
            spin(1)
            measure('after_navigation_'+str(len(report['goals'])))
        report['passed'] = True
    except Exception as error:
        report['failure'] = str(error)
        raise
    finally:
        if goal_handle is not None:
            cancelled = goal_handle.cancel_goal_async()
            deadline = time.monotonic()+3
            while not cancelled.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        report['gicp_acceptances'] = state['acceptances']
        report['gicp_rejections'] = state['rejections']
        report['wall_sec'] = time.monotonic()-started
        report['last_sim_time_sec'] = node.get_clock().now().nanoseconds*1e-9
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
