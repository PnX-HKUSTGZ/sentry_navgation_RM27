#!/usr/bin/env python3
"""Display a recorded mapping run in RViz and move a display-only Gazebo model."""
import argparse
import bisect
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from mapping_observer import Observer
from sensor_mapping import base_sensor


class ReplayObserver(Observer):
    def __init__(self, args):
        super().__init__(args)
        self.display_mode = 'bag replay'
        self.original = args.capture
        manifest = json.loads((self.original / 'manifest.json').read_text())
        self.alignment = np.asarray(manifest['T_world_camera_init'])
        self.map_transform = self.alignment @ np.linalg.inv(base_sensor())
        self.recorded_errors = json.loads((self.original / 'mapping_errors.json').read_text())
        self.error_stamps = [e['stamp'] for e in self.recorded_errors]
        if any(a > b for a, b in zip(self.error_stamps, self.error_stamps[1:])):
            raise ValueError('recorded scoring timestamps are not monotonic')
        self.replay_time = None
        self.latest_truth = None
        self.pose_requests = 0
        self.pose_success = 0
        self.last_pose_request = 0.
        self.create_subscription(Clock, '/clock', self.clock_message, qos_profile_sensor_data)

    def load_result(self):
        # The ordinary observer loads the final map here. Playback must build
        # the visualization from messages as they arrive instead.
        pass

    def odom(self, msg):
        # The saved timestamp-aligned scores are derived from this bag. The
        # full odometry topic is still published by rosbag for inspection.
        pass

    def clock_message(self, msg):
        current = msg.clock.sec + 1e-9 * msg.clock.nanosec
        if self.replay_time is not None and current < self.replay_time - 1.:
            self.gt.clear()
            self.errors.clear()
            self.voxels.clear()
            self.latest_cloud = None
            self.grid_cloud = None
            self.grid_message = None
            self.latest_truth = None
            empty = point_cloud2.create_cloud_xyz32(self.header(), [])
            self.cloud_pub.publish(empty)
            self.scan_pub.publish(empty)
            self.grid_cloud_pub.publish(empty)
            self.markers_pub.publish(MarkerArray(markers=[
                Marker(header=self.header(), ns='mapping_eval', id=i, action=Marker.DELETE)
                for i in (1, 2, 3, 5, 6)]))
        self.replay_time = current

    def truth(self, msg):
        super().truth(msg)
        self.latest_truth = msg.pose.pose

    def score(self):
        if self.replay_time is not None:
            count = bisect.bisect_right(self.error_stamps, self.replay_time)
            self.errors = self.recorded_errors[:count]

    def update(self):
        super().update()
        # set_pose affects only the static, sensor-free visual model in the
        # isolated replay world. It uses the recorded ground-truth pose.
        if self.latest_truth is not None and time.monotonic() - self.last_pose_request >= .1:
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.boolean_pb2 import Boolean
            self.last_pose_request = time.monotonic()
            p = self.latest_truth
            request = Pose(name='recorded_robot')
            request.position.x, request.position.y, request.position.z = p.position.x, p.position.y, p.position.z
            request.orientation.x, request.orientation.y = p.orientation.x, p.orientation.y
            request.orientation.z, request.orientation.w = p.orientation.z, p.orientation.w
            ok, response = self.gz_node.request('/world/default/set_pose', request, Pose, Boolean, 100)
            self.pose_requests += 1
            self.pose_success += int(ok and response.data)
        status_path = self.args.result / 'replay_status.json'
        status_path.write_text(json.dumps({
            'mode': 'recorded bag playback', 'bag_clock_sec': self.replay_time,
            'truth_samples_received': len(self.gt), 'score_samples_available': len(self.errors),
            'last_scored_stamp': self.errors[-1]['stamp'] if self.errors else None,
            'score_is_stale': bool(self.errors and self.replay_time is not None and
                                   self.replay_time - self.errors[-1]['stamp'] > 2),
            'visualized_map_points': len(self.voxels), 'gazebo_markers_available': self.gz_ok,
            'gazebo_pose_requests': self.pose_requests, 'gazebo_pose_success': self.pose_success,
            'truth_xyz': ([self.latest_truth.position.x, self.latest_truth.position.y, self.latest_truth.position.z]
                          if self.latest_truth else None),
        }, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--result', required=True, type=Path, help='Separate replay output directory')
    parser.add_argument('--max-points', type=int, default=60000)
    args = parser.parse_args()
    args.gazebo = True
    args.skip_odom_subscription = True
    rclpy.init(args=[])
    node = ReplayObserver(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
