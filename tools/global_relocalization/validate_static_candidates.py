#!/usr/bin/env python3
"""Select supplemental starts using physical GT checks before any localization goal."""
import argparse
import hashlib
import datetime
import json
import math
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from batch_run import reset_pose, ROOT


class Check(Node):
    def __init__(self):
        super().__init__('supplemental_physical_static_evaluator')
        self.sim = None
        self.rows = []
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(.05, lambda: self.pub.publish(Twist()))
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth/odometry', self.gt, qos_profile_sensor_data)

    def clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def gt(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.rows.append({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'xyz': [p.x, p.y, p.z], 'quat': [q.x, q.y, q.z, q.w]})

    def wait_sim(self, duration):
        started, wall = self.sim, time.monotonic()
        while self.sim is None or started is None or self.sim - started < duration:
            rclpy.spin_once(self, timeout_sec=.05)
            if started is None:
                started = self.sim
            if time.monotonic() - wall > duration * 3 + 10:
                raise RuntimeError('Simulation clock did not advance normally')

    def inspect(self, requested):
        xyz = np.asarray([r['xyz'] for r in self.rows])
        angles = Rotation.from_quat([r['quat'] for r in self.rows]).as_euler('xyz', degrees=True)
        record = {'samples': len(self.rows), 'start_ros': self.rows[0]['stamp'], 'end_ros': self.rows[-1]['stamp'],
            'max_translation_m': float(np.linalg.norm(xyz - xyz[0], axis=1).max()),
            'max_abs_roll_deg': float(abs(angles[:, 0]).max()), 'max_abs_pitch_deg': float(abs(angles[:, 1]).max()),
            'height_range_m': float(np.ptp(xyz[:, 2])),
            'max_xy_error_from_requested_m': float(np.linalg.norm(xyz[:, :2] - [requested['x'], requested['y']], axis=1).max())}
        record['physical_valid'] = bool(record['samples'] >= 50 and record['end_ros'] - record['start_ros'] >= 1 and
            record['max_translation_m'] <= .1 and record['max_abs_roll_deg'] <= 2 and record['max_abs_pitch_deg'] <= 2 and
            record['height_range_m'] <= .02 and record['max_xy_error_from_requested_m'] <= .1)
        return record


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidates', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--map-manifest', help='Frozen map manifest for auditable physical precheck provenance')
    p.add_argument('--start-position-index', type=int, default=30)
    p.add_argument('--needed-positions', type=int, default=2)
    p.add_argument('--validate-all', action='store_true', help='Audit every frozen trial; retain failures and do not select/remove positions')
    a = p.parse_args()
    source = json.loads(Path(a.candidates).read_text())
    output = Path(a.output); output.mkdir(parents=True, exist_ok=False)
    trials = source['trials']; checks = []; selected = []
    inputs = [Path(a.candidates).resolve(), ROOT/'tools/global_relocalization/.build/robot_velocity.json']
    controller = json.loads(inputs[1].read_text())
    inputs.append(Path(controller['robot_description']))
    map_manifest = json.loads(Path(a.map_manifest).read_text()) if a.map_manifest else None
    if a.map_manifest:
        inputs.extend([Path(a.map_manifest).resolve(), Path(map_manifest.get('map_path', str(Path(a.map_manifest).resolve().parent/'map.pcd')))])
    fingerprints = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    if fingerprints[controller['robot_description']] != controller['robot_description_sha256']:
        raise ValueError('Physical precheck robot description differs from controller provenance')
    if map_manifest and fingerprints[str(inputs[-1])] != map_manifest['map_sha256']:
        raise ValueError('Physical precheck map bytes differ from frozen map manifest')
    provenance = {'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'matrix_path': str(inputs[0]), 'matrix_sha256': fingerprints[str(inputs[0])],
        'map_sha256': map_manifest['map_sha256'] if map_manifest else None,
        'controller_provenance': controller, 'verified_inputs': fingerprints,
        'source_input_hashes_verified_at_end': False}
    (output/'manifest.json').write_text(json.dumps(provenance, indent=2)+'\n')
    rclpy.init(); node = Check()
    try:
        for index in sorted({t['trial_index'] // 4 for t in trials if a.validate_all or t['trial_index'] // 4 >= a.start_position_index}):
            candidate = [t for t in trials if t['trial_index'] // 4 == index]
            valid = True
            for trial in candidate:
                reset_attempts = []
                reset_pose(trial['requested_pose'], attempts=reset_attempts,
                           log_path=output/(trial['trial_id']+'.reset_attempts.json'))
                node.wait_sim(3)
                node.rows = []
                node.wait_sim(3)
                result = node.inspect(trial['requested_pose'])
                check = {**trial, **result, 'reset_attempts': reset_attempts, 'gt_used_to_choose_localization_candidates': False}
                checks.append(check)
                (output/'checks.json').write_text(json.dumps(checks, indent=2) + '\n')
                (output/(trial['trial_id']+'.gt.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in node.rows))
                print(json.dumps({k: check[k] for k in ['trial_id','physical_valid','max_translation_m','max_abs_roll_deg','max_abs_pitch_deg']}), flush=True)
                valid = valid and result['physical_valid']
            if valid:
                selected.extend(candidate)
            if not a.validate_all and len(selected) == a.needed_positions * 4:
                break
    finally:
        node.pub.publish(Twist()); node.destroy_node(); rclpy.shutdown()
    for path, expected in fingerprints.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
            raise RuntimeError('Physical precheck input changed: '+path)
    provenance['source_input_hashes_verified_at_end'] = True
    (output/'manifest.json').write_text(json.dumps(provenance, indent=2)+'\n')
    if a.validate_all:
        summary = {'planned_trials': len(trials), 'observed_trials': len(checks),
            'physical_valid_trials': sum(c['physical_valid'] for c in checks),
            'all_physical_valid': len(checks) == len(trials) and all(c['physical_valid'] for c in checks),
            'matrix_unchanged': True, 'selection_by_localization_success': False,
            'validation': 'actual settle and static GT observation at every frozen position/yaw; collision footprint observed through actual physics',
            'thresholds': {'translation_m': .1, 'roll_deg': 2, 'pitch_deg': 2, 'height_range_m': .02, 'landing_xy_error_m': .1},
            'ros_domain_id': __import__('os').environ.get('ROS_DOMAIN_ID'),
            'gz_partition': __import__('os').environ.get('GZ_PARTITION')}
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
        return
    if len(selected) != a.needed_positions * 4:
        raise RuntimeError('Insufficient physically validated new positions; do not select by localization success')
    config = {**source, 'count':len(selected), 'trials':selected,
        'purpose':'supplemental_only_new_physically_validated_left_positions_four_yaws',
        'selection_rule':'same-seed candidate continuation; physical static GT checks only before localization; no candidate/localization-success selection',
        'main_120_denominator_unchanged':True}
    (output/'trials.json').write_text(json.dumps(config, indent=2)+'\n')
    print('Selected supplemental trials:',len(selected),flush=True)


if __name__ == '__main__':
    main()
