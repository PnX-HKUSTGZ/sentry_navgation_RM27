#!/usr/bin/env python3
"""Drive a timed simulation rotation; GT and IMU are observation-only evidence."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Check(Node):
    def __init__(self, speed, duration):
        super().__init__('physical_rotation_check')
        self.speed, self.duration = speed, duration
        self.sim = None
        self.start_sim = None
        self.start_wall = time.monotonic()
        self.rows = []
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odometry', self.gt, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_timer(.05, self.command)

    def clock(self, message):
        self.sim = message.clock.sec + message.clock.nanosec * 1e-9
        if self.start_sim is None:
            self.start_sim = self.sim

    def stage(self):
        elapsed = 0 if self.start_sim is None else self.sim - self.start_sim
        return 'static' if elapsed < 3 else 'spin' if elapsed < 3 + self.duration else 'stop'

    def command(self):
        message = Twist()
        if self.stage() == 'spin':
            message.angular.z = self.speed
        self.publisher.publish(message)

    def gt(self, message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        p, v = message.pose.pose.position, message.twist.twist
        self.rows.append({'source':'gt', 'stamp':message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
                          'stage':self.stage(), 'yaw':yaw, 'xyz':[p.x,p.y,p.z],
                          'angular_norm':math.sqrt(v.angular.x**2+v.angular.y**2+v.angular.z**2),
                          'wz':v.angular.z})

    def imu(self, message):
        v = message.angular_velocity
        self.rows.append({'source':'imu', 'stamp':message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
                          'stage':self.stage(), 'angular_norm':math.sqrt(v.x**2+v.y**2+v.z**2)})


def summarize(node):
    result = {'wall_sec':time.monotonic()-node.start_wall, 'command_rad_sec':node.speed,
              'command_duration_sim_sec':node.duration, 'gt_used_for_control':False, 'stages':{}}
    gt = [row for row in node.rows if row['source'] == 'gt']
    for a, b in zip(gt,gt[1:]):
        dt = b['stamp']-a['stamp']
        if dt > 0:
            b['yaw_rate_from_pose'] = math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw']))/dt
    yaw = sum(math.atan2(math.sin(b['yaw']-a['yaw']), math.cos(b['yaw']-a['yaw'])) for a,b in zip(gt,gt[1:]))
    result['gt_accumulated_yaw_rad'] = yaw
    if gt:
        result['gt_net_translation_m'] = math.dist(gt[0]['xyz'],gt[-1]['xyz'])
        result['gt_max_translation_m'] = max(math.dist(gt[0]['xyz'],row['xyz']) for row in gt)
    for stage in ('static','spin','stop'):
        result['stages'][stage] = {}
        for source in ('gt','imu'):
            rows = [row for row in node.rows if row['source']==source and row['stage']==stage]
            # Skip the first second after command transitions when evaluating rest.
            if rows and stage != 'spin':
                rows = [row for row in rows if row['stamp'] >= rows[0]['stamp']+1.0]
            norms = sorted(row['angular_norm'] for row in rows)
            result['stages'][stage][source] = {'samples':len(rows),
                'angular_norm_median':norms[len(norms)//2] if norms else None,
                'angular_norm_max':max(norms) if norms else None}
            if source == 'gt':
                rates = [abs(row['yaw_rate_from_pose']) for row in rows if 'yaw_rate_from_pose' in row]
                result['stages'][stage][source]['yaw_rate_from_pose_max'] = max(rates) if rates else None
    stopped = result['stages']['stop']
    result['valid'] = bool(yaw * math.copysign(1,node.speed) >= 2*math.pi and
        result.get('gt_max_translation_m',math.inf) < .3 and stopped['gt']['samples'] > 0 and
        stopped['gt']['yaw_rate_from_pose_max'] is not None and stopped['gt']['yaw_rate_from_pose_max'] < .06 and
        stopped['imu']['samples'] > 0 and stopped['imu']['angular_norm_max'] < .06)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--speed', type=float, default=.5)
    parser.add_argument('--duration', type=float, default=22.0)
    args = parser.parse_args()
    rclpy.init()
    node = Check(args.speed,args.duration)
    try:
        while time.monotonic()-node.start_wall < 90:
            rclpy.spin_once(node,timeout_sec=.1)
            if node.start_sim is not None and node.sim-node.start_sim >= args.duration+7:
                break
    finally:
        node.publisher.publish(Twist())
    output = Path(args.output)
    output.parent.mkdir(parents=True,exist_ok=True)
    result = summarize(node)
    output.with_suffix('.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in node.rows))
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
