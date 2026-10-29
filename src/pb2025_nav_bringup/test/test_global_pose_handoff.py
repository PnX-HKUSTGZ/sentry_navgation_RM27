"""Timestamped Point-LIO handoff must not seed from a later robot pose."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import pytest
import rclpy

spec = importlib.util.spec_from_file_location(
    'handoff', Path(__file__).parents[1] / 'scripts/global_pose_handoff.py')
handoff = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handoff)


@pytest.fixture
def node():
    rclpy.init(domain_id=197)
    n = handoff.GlobalPoseHandoff()
    n.published, n.confirmed = [], []
    n.broadcaster = SimpleNamespace(sendTransform=n.published.append)
    n.ack = SimpleNamespace(publish=n.confirmed.append)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def odom(node, stamp, x):
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'base_footprint'
    tf.header.stamp.sec = stamp
    tf.transform.translation.x = x
    tf.transform.rotation.w = 1.
    node.buffer.set_transform(tf, 'pointlio_test')


def pose(stamp=2):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id, msg.header.stamp.sec = 'map', stamp
    msg.pose.pose.position.x, msg.pose.pose.position.y = 10., 4.
    msg.pose.pose.orientation.z = math.sqrt(.5)
    msg.pose.pose.orientation.w = math.sqrt(.5)
    return msg


def test_interpolates_matching_stamp_and_never_uses_latest(node):
    odom(node, 1, 1.)
    odom(node, 3, 3.)
    msg = pose()
    node.accept(msg)
    assert node.confirmed == [msg]
    t = node.published[0]
    assert t.header.stamp.sec == 2
    assert (t.transform.translation.x, t.transform.translation.y) == pytest.approx((10., 2.))
    assert t.transform.rotation.z == pytest.approx(math.sqrt(.5))


def test_unavailable_timestamp_waits_without_latest_fallback(node):
    odom(node, 3, 3.)
    node.accept(pose())
    assert not node.published and not node.confirmed


def test_retries_ack_but_never_reseed_running_navigation(node):
    odom(node, 1, 1.)
    odom(node, 3, 3.)
    msg = pose()
    node.accept(msg)
    node.accept(msg)
    node.accept(pose(3))
    assert len(node.published) == 1
    assert node.confirmed == [msg, msg]


@pytest.mark.parametrize('invalid', ['frame', 'stamp', 'quaternion', 'translation'])
def test_invalid_pose_is_not_acknowledged(node, invalid):
    odom(node, 1, 1.)
    odom(node, 3, 3.)
    msg = pose()
    if invalid == 'frame':
        msg.header.frame_id = 'odom'
    elif invalid == 'stamp':
        msg.header.stamp.sec = 0
    elif invalid == 'quaternion':
        msg.pose.pose.orientation.w = 0.
    else:
        msg.pose.pose.position.x = float('nan')
    node.accept(msg)
    assert not node.published and not node.confirmed


def test_full_3d_alignment_preserves_pitch_and_height():
    msg = pose()
    # Equal 90-degree pitch in both poses cancels; the odom translation
    # must be subtracted without flattening the global z coordinate.
    msg.pose.pose.orientation.z = 0.
    msg.pose.pose.orientation.y = math.sqrt(.5)
    msg.pose.pose.position.z = 1.5
    tf = TransformStamped().transform
    tf.rotation = msg.pose.pose.orientation
    tf.translation.x, tf.translation.z = 2., .4
    p, q = handoff.alignment(msg.pose.pose, tf)
    assert p == pytest.approx((8., 4., 1.1))
    assert q == pytest.approx((0., 0., 0., 1.))
