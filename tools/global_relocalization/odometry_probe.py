#!/usr/bin/env python3
"""Score timestamp-aligned relative LIO motion; GT is an evaluator side channel."""
import argparse
import bisect
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


def matrix(p):
    m = np.eye(4)
    m[:3, 3] = [p.position.x, p.position.y, p.position.z]
    m[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    return m


def interpolate(rows, stamp, max_gap):
    stamps = [r[0] for r in rows]
    i = bisect.bisect_left(stamps, stamp)
    if i < len(rows) and rows[i][0] == stamp:
        return rows[i][1]
    if i == 0 or i == len(rows):
        return None
    ta, a = rows[i - 1]
    tb, b = rows[i]
    if tb - ta > max_gap:
        return None
    alpha = (stamp - ta) / (tb - ta)
    pose = np.eye(4)
    pose[:3, 3] = (1 - alpha) * a[:3, 3] + alpha * b[:3, 3]
    pose[:3, :3] = Slerp([0., 1.], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))([alpha]).as_matrix()[0]
    return pose


class Probe(Node):
    def __init__(self):
        super().__init__('lio_side_channel_probe')
        self.rows = {'lio': [], 'gt': []}
        self.frames = {}
        self.create_subscription(Odometry, '/odometry', lambda m: self.add('lio', m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odometry', lambda m: self.add('gt', m), qos_profile_sensor_data)

    def add(self, key, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.frames[key] = {'parent': msg.header.frame_id, 'child': msg.child_frame_id}
        if not self.rows[key] or stamp > self.rows[key][-1][0]:
            self.rows[key].append((stamp, matrix(msg.pose.pose)))


def score(rows, max_gap):
    pairs = [(t, lio, interpolate(rows['gt'], t, max_gap)) for t, lio in rows['lio']]
    pairs = [(t, lio, gt) for t, lio, gt in pairs if gt is not None]
    result = {'valid_aligned_samples': len(pairs), 'max_gt_bracket_gap_sec': max_gap,
              'alignment': 'relative transforms from first shared timestamp; no fitted world transform'}
    if len(pairs) < 2:
        result['failure'] = 'insufficient timestamp-aligned odometry'
        return result
    t0, lio0, gt0 = pairs[0]
    errors = []
    for t, lio, gt in pairs:
        error = np.linalg.inv(np.linalg.inv(gt0) @ gt) @ (np.linalg.inv(lio0) @ lio)
        errors.append({'stamp_ros': t, 'translation_m': float(np.linalg.norm(error[:3, 3])),
                       'orientation_deg': float(np.degrees(Rotation.from_matrix(error[:3, :3]).magnitude()))})
    result.update(start_ros=t0, end_ros=pairs[-1][0], relative_error=errors[-1], errors=errors)
    for name, index in [('lio', 1), ('gt', 2)]:
        transforms = [p[index] for p in pairs]
        delta = np.linalg.inv(transforms[0]) @ transforms[-1]
        accumulated = sum(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()
                          for a, b in zip(transforms, transforms[1:]))
        result[name] = {'relative_transform': delta.tolist(), 'translation_m': float(np.linalg.norm(delta[:3, 3])),
                        'rotation_deg': float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())),
                        'accumulated_rotation_deg': float(np.degrees(accumulated))}
    for key in ['translation_m', 'orientation_deg']:
        values = np.asarray([e[key] for e in errors])
        result[key] = {'p50': float(np.percentile(values, 50)), 'p95': float(np.percentile(values, 95)),
                       'max': float(values.max()), 'rmse': float(np.sqrt(np.mean(values ** 2)))}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=25)
    parser.add_argument('--max-gap', type=float, default=.25)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
        result = {'wall_sec': time.monotonic() - start, 'samples': {k: len(v) for k, v in node.rows.items()},
                  'frames': node.frames, **score(node.rows, args.max_gap)}
        Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({k: v for k, v in result.items() if k != 'errors'}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
