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
import yaml

spec = importlib.util.spec_from_file_location('startup', Path(__file__).parents[3] / 'pnx-localization/relocalization_manager/scripts/relocalize_then_navigate.py')
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


@pytest.mark.parametrize('current,target', [
    (20., 0.), (70., 90.), (133., 90.), (170., 180.),
    (-179., -180.), (-130., -90.), (-20., 0.),
    (45., 90.), (-45., 0.), (135., 180.), (-135., -90.),
])
def test_nearest_axis_heading_selects_shortest_square_chassis_alignment(current, target):
    actual = startup.nearest_axis_heading(math.radians(current))
    error = math.atan2(math.sin(actual-math.radians(target)),
                       math.cos(actual-math.radians(target)))
    assert error == pytest.approx(0., abs=1e-12)


def test_nearest_axis_heading_bounds_turn_over_full_circle_and_rejects_nonfinite():
    for index in range(-1440, 1441):
        current = math.radians(index / 4)
        target = startup.nearest_axis_heading(current)
        error = math.atan2(math.sin(target-current), math.cos(target-current))
        assert abs(error) <= math.pi/4 + 1e-12
        assert abs(math.sin(2*target)) <= 1e-12
    for invalid in (float('nan'), float('inf'), -float('inf')):
        with pytest.raises(ValueError):
            startup.nearest_axis_heading(invalid)


@pytest.fixture
def node(monkeypatch):
    rclpy.init(domain_id=196)
    from rclpy.parameter import Parameter
    n = startup.Startup(parameter_overrides=[Parameter("restore_spawn_heading", value=True)])
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


def test_nearest_axis_uses_fresh_base_feedback_and_latches_target(node):
    node.heading_alignment_mode = 'nearest_axis'
    node.selected_yaw = 0.
    # Confirmed scan is older than current feedback. Do not round its yaw or
    # the operator's spawn selection, and never rewrite the confirmed seed.
    node.seed.header.stamp.sec = 8
    node.seed.pose.pose.orientation.z = math.sin(math.radians(100)/2)
    node.seed.pose.pose.orientation.w = math.cos(math.radians(100)/2)
    node.tf.transform.rotation.z = math.sin(math.radians(136)/2)
    node.tf.transform.rotation.w = math.cos(math.radians(136)/2)
    node.align_heading()
    assert node.alignment_yaw == pytest.approx(math.pi)
    assert 0. < node.commands[-1].angular.z <= .6
    # A disturbance crosses a rounding boundary; keep the latched target.
    node.tf.transform.rotation.z = math.sin(math.radians(134)/2)
    node.tf.transform.rotation.w = math.cos(math.radians(134)/2)
    node.align_heading()
    assert node.alignment_yaw == pytest.approx(math.pi)
    assert node.commands[-1].angular.z > 0.
    assert node.seed.header.stamp.sec == 8
    assert startup.planar_yaw(node.seed.pose.pose.orientation) == pytest.approx(math.radians(100))
    assert not node.released


def test_nearest_axis_waits_for_valid_feedback_before_target_selection(node):
    node.heading_alignment_mode = 'nearest_axis'
    node.test_time = 11.
    node.align_heading()
    assert node.alignment_yaw is None
    assert node.commands[-1].angular.z == 0.
    assert not node.released


def test_nearest_axis_still_requires_settle_and_rejects_obstacles(node):
    node.heading_alignment_mode = 'nearest_axis'
    node.selected_yaw = 0.
    node.tf.transform.rotation.z = node.tf.transform.rotation.w = math.sqrt(.5)
    node.align_heading()
    assert node.alignment_yaw == pytest.approx(math.pi/2)
    assert node.aligned_since == 10. and not node.released
    node.test_time = 10.4
    node.odom.header.stamp.nanosec = node.tf.header.stamp.nanosec = 400000000
    node.map.data[40*80+40] = 100
    node.align_heading()
    assert node.state == 'FAILED' and not node.released
    assert node.commands[-1].angular.z == 0.


def test_nearest_axis_releases_only_after_alignment_and_stationary_dwell(node):
    node.heading_alignment_mode = 'nearest_axis'
    node.tf.transform.rotation.z = node.tf.transform.rotation.w = math.sqrt(.5)
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


def test_result_retains_original_stamp_and_navigation_waits_for_matching_ack(node):
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 8  # Result timestamp must survive delayed delivery.
    pose.pose.orientation.w = 1.
    reply = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                            result=SimpleNamespace(success=True, pose=pose,
                                                   fitness=.1, inlier_ratio=.8))
    node.state = 'RELOCALIZING'
    node.result(SimpleNamespace(result=lambda: reply))
    assert node.state == 'HANDOFF' and not node.released
    assert node.seed.header == pose.header
    wrong = PoseWithCovarianceStamped()
    wrong.header.frame_id = 'map'
    wrong.header.stamp.sec = 9
    wrong.pose.pose = pose.pose
    node.accepted(wrong)
    assert node.state == 'HANDOFF' and not node.released
    node.accepted(node.seed)
    assert node.state == 'ALIGNING_HEADING' and not node.released


def test_failed_action_never_hands_off_pose_or_activates_navigation(node):
    from action_msgs.msg import GoalStatus
    node.state = 'RELOCALIZING'
    reply = SimpleNamespace(status=GoalStatus.STATUS_ABORTED,
                            result=SimpleNamespace(success=False, reason='ambiguous candidate'))
    node.result(SimpleNamespace(result=lambda: reply))
    assert node.state == 'FAILED' and not node.released
    assert node.commands[-1].angular.z == 0.
    node.accepted(node.seed)
    assert node.state == 'FAILED' and not node.released


def test_goal_accepted_after_timeout_is_canceled(node):
    canceled = []
    goal = SimpleNamespace(accepted=True, cancel_goal_async=lambda: canceled.append(True))
    node.state = 'FAILED'
    node.goal_response(SimpleNamespace(result=lambda: goal))
    assert canceled == [True]
    assert not node.released


def test_reality_waits_for_sensors_without_simulation_spawn_contract():
    from rclpy.parameter import Parameter
    rclpy.init(domain_id=197)
    n = startup.Startup(parameter_overrides=[Parameter('deployment', value='reality')])
    try:
        assert n.spawned and n.selected_yaw is None
        assert all('/simulation/' not in sub.topic_name for sub in n.subscriptions)
        n.state = 'WAITING_FOR_INITIAL_POSE'
        n.tick()
        assert n.state == 'WAITING_FOR_SENSORS'
        n.seed = PoseWithCovarianceStamped()
        n.state = 'HANDOFF'
        n.accepted(n.seed)
        assert n.state == 'RETIRING_GLOBAL'
        assert n.command_pub is n.command_sub is None
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_raw_startup_does_not_wait_for_pointlio_odometry(node):
    requested = []
    node.state = 'WAITING_FOR_SENSORS'
    node.odom = None
    node.odom_ready_since = None
    node.action = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda *a, **k: SimpleNamespace(
            add_done_callback=lambda callback: requested.append(callback)))
    node.trigger = SimpleNamespace(service_is_ready=lambda: True)
    node.tick()
    assert node.state == 'RELOCALIZING'
    assert len(requested) == 1
    assert not node.released


def test_handoff_waits_for_fresh_stable_external_odometry(node):
    published = []
    node.pose_pub = SimpleNamespace(publish=published.append)
    node.state = 'HANDOFF'
    node.odom = None
    node.odom_ready_since = None
    node.tick()
    assert not published
    measurement = Odometry()
    node.odometry(measurement)
    node.tick()
    assert not published
    node.odom_ready_since -= 1.1
    node.tick()
    assert published == [node.seed]
    assert not node.released


def test_handoff_rejects_an_odometry_reception_gap(node):
    node.state = 'HANDOFF'
    node.odom_ready_since = time.monotonic() - 2.
    node.sensor_times['odom'] = time.monotonic() - .6
    published = []
    node.pose_pub = SimpleNamespace(publish=published.append)
    node.tick()
    assert not published
    assert not node.released


def test_stopped_confirmation_waits_for_pointlio_ready(node):
    calls = []
    node.state = 'RELOCALIZING'
    node.armed = True
    node.odom = None
    node.odom_ready_since = None
    node.trigger = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: SimpleNamespace(
            add_done_callback=lambda callback: calls.append(request)))
    node.tick()
    assert not calls and not node.triggered
    node.odometry(Odometry())
    node.odom_ready_since -= 1.1
    node.tick()
    assert len(calls) == 1 and node.triggered
    assert not node.released


def test_handoff_without_heading_restoration_uses_confirmed_heading():
    rclpy.init(domain_id=198)
    n = startup.Startup()
    try:
        assert not n.restore_spawn_heading
        n.seed = PoseWithCovarianceStamped()
        n.seed.header.frame_id = 'map'
        n.seed.pose.pose.orientation.z = math.sin(1.2 / 2)
        n.seed.pose.pose.orientation.w = math.cos(1.2 / 2)
        n.selected_yaw = 0.
        n.state = 'HANDOFF'
        n.accepted(n.seed)
        assert n.state == 'RETIRING_ASSIST'
        assert n.command_pub is n.command_sub is None
        assert n.seed.pose.pose.orientation.z == pytest.approx(math.sin(1.2 / 2))
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_simulation_profile_keeps_control_until_nearest_axis_is_aligned():
    from rclpy.parameter import Parameter
    profile = yaml.safe_load((Path(__file__).parents[1] / 'config/simulation/minco_params.yaml').read_text())
    settings = profile['relocalize_then_navigate']['ros__parameters']
    rclpy.init(domain_id=198)
    n = startup.Startup(parameter_overrides=[
        Parameter('heading_alignment_mode', value=settings['heading_alignment_mode'])])
    try:
        n.seed = PoseWithCovarianceStamped()
        n.seed.header.frame_id = 'map'
        n.seed.header.stamp.sec = 10
        n.seed.pose.pose.orientation.z = math.sin(2.13 / 2)
        n.seed.pose.pose.orientation.w = math.cos(2.13 / 2)
        n.selected_yaw = 0.
        n.state = 'HANDOFF'
        n.accepted(n.seed)
        assert n.state == 'ALIGNING_HEADING'
        assert n.heading_alignment_mode == 'nearest_axis'
        assert n.alignment_yaw is None  # First fresh TF chooses the target.
        assert n.command_pub is not None and n.command_sub is not None
        assert n.seed.header.stamp.sec == 10
    finally:
        n.destroy_node()
        rclpy.shutdown()
