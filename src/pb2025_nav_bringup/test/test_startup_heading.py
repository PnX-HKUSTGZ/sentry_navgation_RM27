"""Heading recovery must finish before navigation and fail closed on stale data."""
import importlib.util
import math
from pathlib import Path
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy

spec = importlib.util.spec_from_file_location('startup', Path(__file__).parents[1] / 'scripts/relocalize_then_navigate.py')
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


def free_map():
    grid = OccupancyGrid()
    grid.info.resolution = .05
    grid.info.width = grid.info.height = 80
    grid.info.origin.position.x = grid.info.origin.position.y = -2.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 6400
    return grid


def test_rotation_sweep_rejects_unknown_obstacles_and_map_edge():
    grid = free_map()
    assert startup.rotation_clear(grid, 0., 0.)
    # Cell within swept disk, even if current heading's polygon misses it.
    for value in (-1, 100):
        grid.data[40*80+48] = value
        assert not startup.rotation_clear(grid, 0., 0.)
    grid.data[40*80+48] = 0
    assert not startup.rotation_clear(grid, 1.8, 0.)
    assert not startup.rotation_clear(grid, float('nan'), 0.)


@pytest.fixture
def node(monkeypatch):
    rclpy.init(domain_id=196)
    n = startup.Startup()
    n.map = free_map()
    n.seed = PoseWithCovarianceStamped()
    n.seed.pose.pose.orientation.w = 1.
    n.selected_yaw = 0.
    n.odom = Odometry()
    n.odom.header.stamp.sec = 10
    n.sensor_times = dict(lidar=time.monotonic(), imu=time.monotonic())
    n.test_time = 10.
    monkeypatch.setattr(n, 'get_clock', lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(n.test_time*1e9))))
    n.tf = TransformStamped()
    n.tf.header.stamp.sec = 10
    n.tf.transform.rotation.w = 1.
    monkeypatch.setattr(n.tf_buffer, 'lookup_transform', lambda *a: n.tf)
    n.commands = []
    n.command_pub = SimpleNamespace(publish=lambda m: n.commands.append(m))
    n.released = False
    def release():
        n.released = True
    monkeypatch.setattr(n, 'release_control', release)
    n.state = 'ALIGNING_HEADING'
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_alignment_uses_shortest_turn_and_bounded_command(node):
    node.selected_yaw = math.pi - .05
    node.tf.transform.rotation.z = math.sin((-math.pi+.05)/2)
    node.tf.transform.rotation.w = math.cos((-math.pi+.05)/2)
    node.align_heading()
    assert -.2 < node.commands[-1].angular.z < 0.
    node.selected_yaw = 0.
    node.align_heading()
    assert abs(node.commands[-1].angular.z) <= .6
    assert not node.released
    assert node.commands[-1].linear.x == node.commands[-1].linear.y == 0.


def test_stale_feedback_stops_and_resets_settle_confirmation(node):
    node.aligned_since = 9.
    node.test_time = 11.
    node.align_heading()
    assert node.commands[-1].angular.z == 0.
    assert node.aligned_since is None
    assert not node.released


def test_missing_transform_resets_settle_confirmation(node, monkeypatch):
    node.aligned_since = 9.
    def missing(*args):
        raise startup.TransformException('unavailable')
    monkeypatch.setattr(node.tf_buffer, 'lookup_transform', missing)
    node.align_heading()
    assert node.commands[-1].angular.z == 0.
    assert node.aligned_since is None and not node.released


def test_alignment_requires_stationary_dwell_before_handoff(node):
    node.odom.twist.twist.angular.z = .2
    node.align_heading()
    assert node.aligned_since is None and not node.released
    node.odom.twist.twist.angular.z = 0.
    node.align_heading()
    assert node.aligned_since == 10. and not node.released
    node.test_time = 10.4
    node.odom.header.stamp.nanosec = node.tf.header.stamp.nanosec = 400000000
    node.align_heading()
    assert node.released


def test_clearance_loss_aborts_without_navigation(node):
    node.map.data[40*80+40] = 100
    node.align_heading()
    assert node.state == 'FAILED'
    assert not node.released
    assert node.commands[-1].angular.z == 0.


def test_invalid_orientation_aborts_without_turning(node):
    node.tf.transform.rotation.z = float('nan')
    node.align_heading()
    assert node.state == 'FAILED' and not node.released
    assert node.commands[-1].angular.z == 0.


def test_pose_ack_enters_alignment_instead_of_activating_navigation(node):
    node.state = 'HANDOFF'
    node.accepted(node.seed)
    assert node.state == 'ALIGNING_HEADING'
    assert not node.released
