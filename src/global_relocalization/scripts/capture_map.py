#!/usr/bin/env python3
"""Fuse simulated sensor returns using timestamp-interpolated GT and real TF.
GT is used only by this map preparation tool, never by the online estimator.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    return t


def transform_matrix(transform):
    p, q = transform.translation, transform.rotation
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    return t


class Collector(Node):
    def __init__(self, args):
        super().__init__('global_relocalization_map_collector', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.args = args
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth/odometry', self.pose, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/livox/lidar', self.cloud, qos_profile_sensor_data)
        self.create_timer(.1, self.tick)
        self.poses, self.times, self.pending, self.chunks = [], [], [], []
        self.latest, self.child, self.parent, self.start_ns = None, None, None, None
        self.wall_start = time.monotonic()
        self.scans = self.used = self.dropped = self.points = 0
        self.transforms = {}
        self.done = False
        self.route = json.loads(Path(args.route).read_text())['waypoints'] if args.route else []
        self.target = 0
        self.trajectory = []

    def pose(self, msg):
        stamp = ns(msg.header.stamp)
        if self.times and stamp <= self.times[-1]:
            return
        mat = pose_matrix(msg.pose.pose)
        if not np.isfinite(mat).all():
            return
        if self.child and (msg.child_frame_id != self.child or msg.header.frame_id != self.parent):
            raise RuntimeError('GT frame changed during capture')
        self.child, self.parent = msg.child_frame_id, msg.header.frame_id
        self.times.append(stamp)
        self.poses.append(mat)
        self.latest = mat
        if self.start_ns is None:
            self.start_ns = stamp
        if len(self.trajectory) == 0 or stamp - self.trajectory[-1]['stamp_ns'] >= 1000000000:
            self.trajectory.append({'stamp_ns': stamp, 'T_world_base': mat.tolist()})
        self.flush()

    def cloud(self, msg):
        self.scans += 1
        if self.scans % self.args.scan_stride:
            return
        self.pending.append(msg)
        self.flush()

    def flush(self):
        if len(self.times) < 2:
            return
        keep = []
        for msg in self.pending:
            stamp = ns(msg.header.stamp)
            if stamp > self.times[-1]:
                keep.append(msg)
                continue
            idx = bisect.bisect_left(self.times, stamp)
            if idx == 0 or self.times[idx] - self.times[idx-1] > self.args.max_gap_ms * 1000000:
                self.dropped += 1
                continue
            try:
                tf = self.tf.lookup_transform(self.child, msg.header.frame_id, Time.from_msg(msg.header.stamp))
            except Exception:
                if self.times[-1] - stamp < 2000000000:
                    keep.append(msg)
                else:
                    self.dropped += 1
                continue
            self.transforms[msg.header.frame_id] = transform_matrix(tf.transform).tolist()
            if np.linalg.norm(self.poses[idx][:3,3]-self.poses[idx-1][:3,3]) > .20 or Rotation.from_matrix(self.poses[idx-1][:3,:3].T@self.poses[idx][:3,:3]).magnitude() > .30:
                self.dropped += 1
                continue
            a, b = self.times[idx-1], self.times[idx]
            fraction = (stamp-a)/(b-a)
            interpolated = np.eye(4)
            interpolated[:3, 3] = self.poses[idx-1][:3, 3]*(1-fraction) + self.poses[idx][:3, 3]*fraction
            rotations = Rotation.from_matrix(np.stack([self.poses[idx-1][:3, :3], self.poses[idx][:3, :3]]))
            interpolated[:3, :3] = Slerp([0., 1.], rotations)([fraction]).as_matrix()[0]
            world_sensor = interpolated @ np.asarray(self.transforms[msg.header.frame_id])
            raw = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
            xyz = np.stack([raw[n].reshape(-1) for n in ('x', 'y', 'z')], axis=1)
            finite = np.isfinite(xyz).all(axis=1)
            ranges = np.linalg.norm(xyz, axis=1)
            finite &= (ranges >= self.args.min_range) & (ranges <= self.args.max_range)
            xyz = xyz[finite]
            mapped = xyz @ world_sensor[:3, :3].T + world_sensor[:3, 3]
            if not len(mapped):
                self.dropped += 1
                continue
            _, unique = np.unique(np.floor(mapped/self.args.voxel).astype(np.int32), axis=0, return_index=True)
            self.chunks.append(mapped[unique].astype('<f4'))
            self.points += len(unique)
            self.used += 1
            if len(self.chunks) >= 30:
                self.compact()
        self.pending = keep

    def compact(self):
        if not self.chunks:
            return
        merged = np.concatenate(self.chunks)
        _, indices = np.unique(np.floor(merged/self.args.voxel).astype(np.int32), axis=0, return_index=True)
        self.chunks = [merged[indices]]

    def tick(self):
        if self.done:
            return
        if self.start_ns is None:
            if time.monotonic() - self.wall_start > 90:
                self.finish('no GT data within 90 wall seconds')
            return
        elapsed = (self.times[-1]-self.start_ns)*1e-9
        if elapsed >= self.args.duration or time.monotonic()-self.wall_start > self.args.wall_timeout:
            self.finish('duration reached' if elapsed >= self.args.duration else 'wall timeout')
            return
        cmd = Twist()
        if elapsed > 3 and not self.args.stationary:
            if self.target < len(self.route):
                target = np.array(self.route[self.target][:2], float)
                delta = target-self.latest[:2, 3]
                if np.linalg.norm(delta) < .12:
                    self.target += 1
                else:
                    yaw = math.atan2(self.latest[1,0], self.latest[0,0])
                    velocity = delta/max(np.linalg.norm(delta), 1e-9)*self.args.speed
                    cmd.linear.x = math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1]
                    cmd.linear.y = -math.sin(yaw)*velocity[0]+math.cos(yaw)*velocity[1]
            else:
                cmd.angular.z = .4
        self.pub.publish(cmd)

    def finish(self, reason):
        if self.done:
            return
        self.done = True
        self.pub.publish(Twist())
        self.flush()
        self.compact()
        points = self.chunks[0] if self.chunks else np.empty((0, 3), '<f4')
        pcd = self.output/'map.pcd'
        if len(points):
            header = f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
            with pcd.open('wb') as stream:
                stream.write(header.encode('ascii'))
                stream.write(points.astype('<f4').tobytes())
        manifest = {'schema_version': 1, 'frame_id': 'map', 'pose_source': 'gazebo_ground_truth_map_preparation_only',
            'input_cloud': '/livox/lidar', 'pose_topic': '/ground_truth/odometry', 'pose_parent_frame': self.parent,
            'pose_child_frame': self.child, 'T_map_world': np.eye(4).tolist(), 'T_pose_sensor_by_frame': self.transforms,
            'interpolation': 'linear_translation_shortest_quaternion_slerp_at_cloud_header_stamp', 'max_pose_gap_ms': self.args.max_gap_ms,
            'per_point_deskew': False, 'scan_model': 'Gazebo instantaneous GPU lidar scan',
            'min_range': self.args.min_range, 'self_filter': 'range>=min_range; .5m excludes complete robot envelope',
            'voxel_size': self.args.voxel, 'scan_stride': self.args.scan_stride, 'max_range': self.args.max_range,
            'received_scans': self.scans, 'fused_scans': self.used, 'dropped_scans': self.dropped, 'pending_scans': len(self.pending),
            'points': len(points), 'map_sha256': hashlib.sha256(pcd.read_bytes()).hexdigest() if pcd.exists() else None,
            'bounds': [points.min(axis=0).tolist(), points.max(axis=0).tolist()] if len(points) else None,
            'capture_ros_sec': (self.times[-1]-self.start_ns)*1e-9 if self.start_ns else 0,
            'capture_wall_sec': time.monotonic()-self.wall_start, 'reason': reason, 'waypoints': self.route,
            'capture_mode': 'stationary_discontinuous' if self.args.stationary else 'cmd_vel_route_then_spin',
            'interpolation_discontinuity_limits': {'translation_m': .20, 'orientation_rad': .30},
            'waypoints_completed': self.target, 'trajectory': self.trajectory, 'prior_pcd_transform': [0.,0.,0.,0.,0.,0.]}
        (self.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        self.get_logger().info(f'Frozen map: {len(points)} points, {self.used} scans in {self.output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--wall-timeout', type=float, default=1200)
    parser.add_argument('--stationary', action='store_true', help='Publish only zero velocity; map station resetter may reposition offline')
    parser.add_argument('--route', help='JSON with waypoints [[x,y], ...] in world/map')
    parser.add_argument('--speed', type=float, default=.25)
    parser.add_argument('--voxel', type=float, default=.03)
    parser.add_argument('--scan-stride', type=int, default=3)
    parser.add_argument('--min-range', type=float, default=.5)
    parser.add_argument('--max-range', type=float, default=30)
    parser.add_argument('--max-gap-ms', type=float, default=200)
    args = parser.parse_args()
    if args.duration <= 0 or args.voxel <= 0 or args.scan_stride < 1:
        parser.error('duration, voxel, scan-stride must be positive')
    rclpy.init(args=[])
    node = Collector(args)
    signal.signal(signal.SIGINT, lambda *_: node.finish('interrupted'))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() - node.wall_start > args.wall_timeout:
                node.finish('wall timeout')
    finally:
        node.finish('shutdown')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.used else 2


if __name__ == '__main__':
    raise SystemExit(main())
