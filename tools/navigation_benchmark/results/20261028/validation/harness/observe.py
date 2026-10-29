#!/usr/bin/env python3
"""Observe a Nav2 route using Gazebo truth; never publish localization."""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log, ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_interfaces.msg import MpcPositionCommand
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def parameter_value(value):
    fields = {ParameterType.PARAMETER_BOOL: 'bool_value', ParameterType.PARAMETER_INTEGER: 'integer_value',
              ParameterType.PARAMETER_DOUBLE: 'double_value', ParameterType.PARAMETER_STRING: 'string_value',
              ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array_value', ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
              ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value', ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
              ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value'}
    result = getattr(value, fields[value.type]) if value.type in fields else None
    if value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        return [int.from_bytes(x, 'little') if isinstance(x, bytes) else int(x) for x in result]
    if value.type in (ParameterType.PARAMETER_BOOL_ARRAY, ParameterType.PARAMETER_INTEGER_ARRAY,
                      ParameterType.PARAMETER_DOUBLE_ARRAY, ParameterType.PARAMETER_STRING_ARRAY):
        return list(result)
    return result


def write_json(path, data):
    temp = path + '.tmp'
    with open(temp, 'w', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')
    os.replace(temp, path)


class Observer(Node):
    def __init__(self, args):
        super().__init__('navigation_benchmark_observer')
        self.args = args
        self.clock_time = None
        self.truth = None
        self.odom = None
        self.command = None
        self.command_stamp = None
        self.run = None
        self.last_sample = None
        self.trajectory_count = 0
        self.mpc_count = 0
        self.mpc_nonzero_count = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odometry', self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(MpcPositionCommand, '/minco/opt_path', self.on_trajectory, 10)
        self.create_subscription(Twist, '/minco/cmd_vel_mpc', self.on_mpc, 10)

    def on_clock(self, msg):
        self.clock_time = seconds(msg.clock)

    def on_odom(self, msg):
        self.odom = msg

    def on_cmd(self, msg):
        self.command = msg
        self.command_stamp = self.clock_time

    def on_trajectory(self, msg):
        self.trajectory_count += 1

    def on_mpc(self, msg):
        self.mpc_count += 1
        if math.hypot(msg.linear.x, msg.linear.y) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.mpc_nonzero_count += 1

    def on_log(self, msg):
        if self.run is not None:
            self.run['events'].append({'stamp': seconds(msg.stamp), 't': self.clock_time, 'receipt_wall': time.time(), 'name': msg.name, 'msg': msg.msg})

    def on_truth(self, msg):
        self.truth = msg
        if self.run is None or self.clock_time is None:
            return
        t = seconds(msg.header.stamp)
        if self.last_sample is not None and t >= self.last_sample and t - self.last_sample < 0.05 - 1e-9:
            return
        self.last_sample = t
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x))))
        v = msg.twist.twist.linear
        cmd = self.command
        odom = self.odom
        odom_pos = odom.pose.pose.position if odom else None
        loc = None
        loc_stamp = None
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            loc = transform.transform.translation
            loc_stamp = seconds(transform.header.stamp)
        except TransformException:
            pass
        self.run['samples'].append({'t': t, 'x': p.x, 'y': p.y, 'z': p.z,
            'vx': v.x, 'vy': v.y, 'cmd_vx': cmd.linear.x if cmd else None,
            'cmd_vy': cmd.linear.y if cmd else None, 'cmd_stamp': self.command_stamp,
            'goal_distance': math.hypot(self.run['goal'][0]-p.x, self.run['goal'][1]-p.y),
            'active': self.run['active'], 'pitch': pitch, 'roll': roll,
            'odom_x': odom_pos.x if odom_pos else None, 'odom_y': odom_pos.y if odom_pos else None,
            'odom_z': odom_pos.z if odom_pos else None, 'odom_frame': odom.header.frame_id if odom else None,
            'odom_child_frame': odom.child_frame_id if odom else None,
            'odom_stamp': seconds(odom.header.stamp) if odom else None,
            'loc_x': loc.x if loc else None, 'loc_y': loc.y if loc else None,
            'loc_z': loc.z if loc else None, 'loc_transform_stamp': loc_stamp,
            'loc_xy_error_vs_truth': math.hypot(loc.x - p.x, loc.y - p.y) if loc else None})

    def wait_future(self, future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result() if future.done() else None

    def service(self, node, service_type, suffix, request, deadline):
        client = self.create_client(service_type, '/' + node + '/' + suffix)
        try:
            while rclpy.ok() and time.monotonic() < deadline and not client.service_is_ready():
                rclpy.spin_once(self, timeout_sec=0.1)
            if not client.service_is_ready():
                return None
            return self.wait_future(client.call_async(request), deadline)
        finally:
            self.destroy_client(client)

    def runtime(self, deadline):
        runtime = {'mode_label': self.args.mode, 'mode_label_is_verified': False,
                   'nodes': {}, 'topic_publishers': {}}
        queries = {
            'planner_server': ['planner_plugins', 'MincoPlanner.plugin', 'MincoPlanner.odom_topic', 'MincoPlanner.trajectory_topic', 'MincoPlanner.rog_map.ros_callback.cloud_topic'],
            'controller_server': ['controller_plugins', 'MincoMpc.plugin', 'MincoMpc.odom_topic', 'MincoMpc.opt_path_topic', 'MincoMpc.cmd_vel_mpc_topic'],
            'rm27_ground_truth_localizer': ['ground_truth_odom_topic'],
        }
        for name in ('planner_server', 'controller_server', 'bt_navigator', 'velocity_smoother'):
            response = None
            while time.monotonic() < deadline:
                response = self.service(name, GetState, 'get_state', GetState.Request(), deadline)
                if response is not None and response.current_state.label == 'active':
                    break
                rclpy.spin_once(self, timeout_sec=0.2)
            if response is None or response.current_state.label != 'active':
                raise RuntimeError(f'{name} lifecycle state is not active')
            runtime['nodes'][name] = {'lifecycle_state': response.current_state.label}
        for name, names in queries.items():
            # The optional truth localizer may not exist on the full chain.
            end = min(deadline, time.monotonic() + (2 if name == 'rm27_ground_truth_localizer' else 10))
            req = GetParameters.Request()
            req.names = names
            response = self.service(name, GetParameters, 'get_parameters', req, end)
            if response is None:
                if name != 'rm27_ground_truth_localizer':
                    raise RuntimeError(f'cannot read parameters from {name}')
                continue
            runtime['nodes'].setdefault(name, {})['parameters'] = dict(zip(names, map(parameter_value, response.values)))
        p = runtime['nodes']['planner_server']['parameters']
        c = runtime['nodes']['controller_server']['parameters']
        for name in ('planner_server', 'controller_server'):
            listing = self.service(name, ListParameters, 'list_parameters', ListParameters.Request(), min(deadline, time.monotonic() + 10))
            if listing is None:
                raise RuntimeError(f'cannot list parameters from {name}')
            req = GetParameters.Request()
            req.names = list(listing.result.names)
            response = self.service(name, GetParameters, 'get_parameters', req, min(deadline, time.monotonic() + 10))
            if response is None:
                raise RuntimeError(f'cannot dump parameters from {name}')
            runtime['nodes'][name]['all_parameters'] = dict(zip(req.names, map(parameter_value, response.values)))
        runtime['graph_nodes'] = [f'{namespace.rstrip("/")}/{name}' for name, namespace in self.get_node_names_and_namespaces()]
        runtime['localization_error_assumption'] = ('map and Gazebo world share the known map coordinates; '
            'base_link and the ground-truth odometry entity represent the same physical reference point. '
            'Latest available TF and truth samples may have different timestamps.')
        runtime['ground_truth_frame'] = self.truth.header.frame_id
        runtime['ground_truth_child_frame'] = self.truth.child_frame_id
        for topic in ('/odometry', '/ground_truth/odometry', '/cloud_registered_full', '/registered_scan', '/lidar_odometry', '/minco/opt_path', '/minco/cmd_vel_mpc', '/cmd_vel'):
            runtime['topic_publishers'][topic] = [f'{x.node_namespace.rstrip("/")}/{x.node_name}' for x in self.get_publishers_info_by_topic(topic)]
        odom_publishers = runtime['topic_publishers']['/odometry']
        pointlio = [n for n in runtime['graph_nodes'] if 'point_lio' in n.lower() or 'pointlio' in n.lower()]
        runtime['mode_evidence'] = {'odometry_publishers': odom_publishers, 'pointlio_nodes': pointlio}
        if self.args.mode == 'gt':
            runtime['mode_label_is_verified'] = (odom_publishers == ['/rm27_ground_truth_localizer'] and not pointlio)
            runtime['mode_verification'] = 'verified' if runtime['mode_label_is_verified'] else 'failed'
        elif self.args.mode == 'full_chain':
            scans = runtime['topic_publishers']['/registered_scan']
            lidar_odometry = runtime['topic_publishers']['/lidar_odometry']
            truth_nodes = [n for n in runtime['graph_nodes'] if n.rsplit('/', 1)[-1] == 'rm27_ground_truth_localizer']
            runtime['mode_evidence'].update({'registered_scan_publishers': scans, 'lidar_odometry_publishers': lidar_odometry,
                                             'truth_localizer_nodes': truth_nodes})
            runtime['mode_label_is_verified'] = (bool(pointlio) and odom_publishers == ['/sensor_scan_generation'] and
                lidar_odometry == ['/loam_interface'] and scans == ['/loam_interface'] and not truth_nodes)
            runtime['mode_verification'] = ('topology verified; localization accuracy is not established' if runtime['mode_label_is_verified']
                                            else 'topology verification failed')
        else:
            runtime['mode_verification'] = 'pending: hybrid graph is not verified'
        write_json(os.path.join(self.args.output, 'runtime.json'), runtime)
        if self.args.mode in ('gt', 'full_chain') and not runtime['mode_label_is_verified']:
            raise RuntimeError(f'{self.args.mode} localization topology verification failed; see runtime.json')
        if p.get('planner_plugins') != ['MincoPlanner'] or p.get('MincoPlanner.plugin') != 'minco_planner/MincoPlanner':
            raise RuntimeError('active planner configuration is not MincoPlanner')
        if c.get('controller_plugins') != ['MincoMpc'] or c.get('MincoMpc.plugin') != 'minco_controller::MincoMpcController':
            raise RuntimeError('active controller configuration is not MincoMpc')
        if c.get('MincoMpc.opt_path_topic', '').lstrip('/') != 'minco/opt_path' or c.get('MincoMpc.cmd_vel_mpc_topic', '').lstrip('/') != 'minco/cmd_vel_mpc':
            raise RuntimeError('configured MINCO topics differ from observed topics')
        return runtime

    def run_goal(self, index, xy):
        path = os.path.join(self.args.output, f'run_{index:02d}.json')
        self.run = {'goal': xy, 'samples': [], 'events': [], 'active': False, 'status': 'sending'}
        self.last_sample = None
        baseline = (self.trajectory_count, self.mpc_count, self.mpc_nonzero_count)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp.sec = int(self.clock_time)
        goal.pose.header.stamp.nanosec = int((self.clock_time % 1) * 1e9)
        goal.pose.pose.position.x, goal.pose.pose.position.y = xy
        goal.pose.pose.orientation.w = 1.0
        start_sim = self.clock_time
        wall_deadline = time.monotonic() + self.args.wall_timeout
        handle = None
        result = None
        try:
            handle = self.wait_future(self.action.send_goal_async(goal), wall_deadline)
            if handle is None or not handle.accepted:
                self.run['status'] = 'send_timeout' if handle is None else 'rejected'
                return False
            self.run['active'] = True
            self.run['status'] = 'active'
            future = handle.get_result_async()
            while rclpy.ok() and not future.done() and time.monotonic() < wall_deadline:
                if self.clock_time is not None and (self.clock_time < start_sim or self.clock_time - start_sim >= self.args.timeout):
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            result = future.result() if future.done() else None
            if result is None:
                self.run['status'] = 'timeout_or_clock_reset'
                self.wait_future(handle.cancel_goal_async(), time.monotonic() + 3.0)
                return False
            self.run['action_status'] = result.status
            evidence = self.trajectory_count > baseline[0] and self.mpc_nonzero_count > baseline[2]
            self.run['status'] = 'succeeded' if result.status == GoalStatus.STATUS_SUCCEEDED and evidence else 'failed'
            self.run['motion_evidence_passed'] = evidence
            return self.run['status'] == 'succeeded'
        finally:
            if handle is not None and handle.accepted and result is None and self.run['status'] != 'timeout_or_clock_reset':
                self.wait_future(handle.cancel_goal_async(), time.monotonic() + 3.0)
            self.run['active'] = False
            self.run['start_sim_time'] = start_sim
            self.run['end_sim_time'] = self.clock_time
            self.run['trajectory_messages'] = self.trajectory_count - baseline[0]
            self.run['mpc_messages'] = self.mpc_count - baseline[1]
            self.run['mpc_nonzero_messages'] = self.mpc_nonzero_count - baseline[2]
            stamps = [x['t'] for x in self.run['samples']]
            self.run['max_sample_gap'] = max((b-a for a, b in zip(stamps, stamps[1:])), default=None)
            write_json(path, self.run)
            print(f'{path}: {self.run["status"]}, samples={len(stamps)}, trajectory={self.run["trajectory_messages"]}, mpc={self.run["mpc_messages"]}', flush=True)
            self.run = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--goals', required=True, help='JSON list of [x, y] map-frame goals')
    parser.add_argument('--timeout', type=float, default=120.0, help='simulation seconds per goal')
    parser.add_argument('--wall-timeout', type=float, default=240.0, help='wall seconds per goal and startup')
    parser.add_argument('--mode', choices=['gt', 'full_chain', 'hybrid'], default='gt', help='unverified label')
    args = parser.parse_args()
    try:
        goals = json.loads(args.goals)
        if not isinstance(goals, list) or not goals or any(not isinstance(g, list) or len(g) != 2 or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in g) for g in goals):
            raise ValueError('goals must be a nonempty JSON list of finite [x, y] pairs')
        if not math.isfinite(args.timeout) or not math.isfinite(args.wall_timeout) or min(args.timeout, args.wall_timeout) <= 0:
            raise ValueError('timeouts must be finite and positive')
    except (ValueError, TypeError) as exc:
        parser.error(str(exc))
    os.makedirs(args.output, exist_ok=True)
    if any(name.startswith('run_') and name.endswith('.json') for name in os.listdir(args.output)) or os.path.exists(os.path.join(args.output, 'final.json')):
        parser.error('output already contains run or final files; refusing to overwrite')
    final = {'status': 'startup_failure', 'mode': args.mode, 'completed_goals': 0}
    rclpy.init()
    node = Observer(args)
    try:
        deadline = time.monotonic() + args.wall_timeout
        print('Waiting for clock, ground truth and NavigateToPose...', flush=True)
        while rclpy.ok() and time.monotonic() < deadline and (node.clock_time is None or node.truth is None or not node.action.server_is_ready()):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.clock_time is None or node.truth is None or not node.action.server_is_ready():
            raise RuntimeError('startup timed out waiting for clock, ground truth or action server')
        node.runtime(deadline)
        # Lifecycle activation precedes a populated rolling map. Use a fixed
        # simulation-time warmup for every repeat, bounded by the startup deadline.
        warmup_start = node.clock_time
        while node.clock_time - warmup_start < 3.0:
            if time.monotonic() >= deadline:
                raise RuntimeError('clock did not advance during map warmup')
            rclpy.spin_once(node, timeout_sec=0.1)
        print('MINCO planner/controller configuration verified; starting route.', flush=True)
        for index, goal in enumerate(goals, 1):
            if not node.run_goal(index, goal):
                final.update({'status': 'goal_failure', 'failed_goal': index})
                return 1
            final['completed_goals'] = index
        final['status'] = 'succeeded'
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        final['error'] = str(exc)
        print(f'Observation stopped: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        write_json(os.path.join(args.output, 'final.json'), final)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
