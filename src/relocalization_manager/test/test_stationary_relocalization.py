"""Exercise the real C++ backend over DDS, including actual GICP and TF output."""
import math
import os
from pathlib import Path
import subprocess
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


@pytest.fixture
def backend(tmp_path):
    # Three asymmetric surfaces constrain all six registration degrees of freedom.
    rng = np.random.default_rng(27)
    a, b = rng.uniform(-2, 2, (2, 6000))
    points = np.concatenate([np.c_[a, b, np.zeros(6000)],
                             np.c_[a, np.full(6000, 2.), b + 2.],
                             np.c_[np.full(6000, -2.), a, b + 2.]])
    pcd = tmp_path / 'prior.pcd'
    with pcd.open('w') as f:
        f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
        np.savetxt(f, points, fmt='%.6f')
    exe = os.environ.get('RELOCALIZATION_TEST_EXECUTABLE', str(
        Path(__file__).resolve().parents[3] /
        'install/relocalization_manager/lib/relocalization_manager/relocalization_manager_node'))
    params = dict(prior_pcd_file=str(pcd), wait_for_initial_pose='true', base_frame='base_footprint',
                  global_leaf_size='0.08', registered_leaf_size='0.08', min_inliers='100',
                  max_iterations='30', required_consecutive_gicp_acceptances='3', num_threads='2',
                  refinement_reference_frame='odom')
    cmd = [exe, '--ros-args']
    for k, v in params.items():
        cmd += ['-p', f'{k}:={v}']
    log = (tmp_path / 'backend.log').open('w')
    process = subprocess.Popen(cmd, env={**os.environ, 'ROS_DOMAIN_ID': '196'},
                               stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(domain_id=196)
    node = rclpy.create_node('stationary_backend_test')
    node.acks, node.transforms = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped, '/relocalization/initialpose_accepted',
                             node.acks.append, qos)
    node.create_subscription(TFMessage, '/tf', lambda m: node.transforms.extend(
        t for t in m.transforms if t.header.frame_id == 'map' and t.child_frame_id == 'odom'), 100)
    node.seed = node.create_publisher(PoseWithCovarianceStamped, '/relocalization/initialpose', 10)
    node.tf = node.create_publisher(TFMessage, '/tf', 100)
    node.cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    node.odom = node.create_publisher(Odometry, '/odometry', 10)
    node.cloud = node.create_publisher(PointCloud2, '/registered_scan', 10)
    node.points = points
    node.backend_log = tmp_path / 'backend.log'
    deadline = time.monotonic() + 15
    while node.seed.get_subscription_count() == 0 and time.monotonic() < deadline:
        assert process.poll() is None, (tmp_path / 'backend.log').read_text()
        rclpy.spin_once(node, timeout_sec=.05)
    assert node.seed.get_subscription_count() > 0
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        print((tmp_path / 'backend.log').read_text())


def pump(n, duration, callback=None):
    until = time.monotonic() + duration
    while time.monotonic() < until:
        if callback:
            callback()
        rclpy.spin_once(n, timeout_sec=.02)
        time.sleep(.02)


def odom_tf(stamp, x=0.):
    t = TransformStamped()
    t.header = Header(stamp=stamp, frame_id='odom')
    t.child_frame_id = 'base_footprint'
    t.transform.translation.x, t.transform.rotation.w = x, 1.
    return t


def seed(n, stamp, x=0., y=0., yaw=0.):
    p = PoseWithCovarianceStamped()
    p.header = Header(stamp=stamp, frame_id='map')
    p.pose.pose.position.x, p.pose.pose.position.y = x, y
    p.pose.pose.orientation.z, p.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def test_timestamped_handoff_and_no_reseed(backend):
    n = backend
    now = n.get_clock().now().nanoseconds
    stamp = lambda offset: rclpy.time.Time(nanoseconds=now + int(offset * 1e9)).to_msg()
    p = seed(n, stamp(-2), 10., 4., math.pi / 2)
    # Only a later odom sample exists: latest-TF fallback would incorrectly seed.
    pump(n, .5, lambda: n.tf.publish(TFMessage(transforms=[odom_tf(stamp(-1), 3.)])))
    pump(n, .5, lambda: n.seed.publish(p))
    assert not n.acks and not n.transforms
    pump(n, .5, lambda: n.tf.publish(TFMessage(transforms=[
        odom_tf(stamp(-3), 1.), odom_tf(stamp(-1), 3.)])))
    bad = seed(n, stamp(-2))
    bad.pose.pose.orientation.w = 0.
    pump(n, .3, lambda: n.seed.publish(bad))
    assert not n.acks
    pump(n, .5, lambda: n.seed.publish(p))
    assert n.acks and n.transforms
    t = n.transforms[-1].transform
    assert (t.translation.x, t.translation.y) == pytest.approx((10., 2.), abs=1e-6)
    assert t.rotation.z == pytest.approx(math.sqrt(.5))
    count = len(n.acks)
    pump(n, .3, lambda: n.seed.publish(p))
    assert len(n.acks) > count
    pump(n, .3, lambda: n.seed.publish(seed(n, stamp(-1), 50.)))
    assert n.transforms[-1].transform == t


def test_stationary_consensus_motion_and_repeated_stops(backend):
    n = backend
    stamp = n.get_clock().now().to_msg()
    p = seed(n, stamp)
    pump(n, .3, lambda: n.tf.publish(TFMessage(transforms=[odom_tf(stamp)])))
    pump(n, .3, lambda: n.seed.publish(p))
    assert n.acks
    target = np.array([.18, -.09, .04])

    def feed(command_yaw=0., measured_linear=0.):
        current = n.get_clock().now().to_msg()
        cmd = Twist()
        cmd.angular.z = command_yaw
        n.cmd.publish(cmd)
        odom = Odometry()
        odom.header = Header(stamp=current, frame_id='odom')
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        odom.twist.twist.linear.x = measured_linear
        n.odom.publish(odom)
        n.cloud.publish(create_cloud_xyz32(Header(stamp=current, frame_id='odom'),
                                          n.points - target))
        n.tf.publish(TFMessage(transforms=[odom_tf(current)]))

    def position():
        t = n.transforms[-1].transform.translation
        return np.array([t.x, t.y, t.z])

    pump(n, 1.2, lambda: feed(command_yaw=.4))
    assert position() == pytest.approx([0., 0., 0.])
    pump(n, 1.2, lambda: feed(measured_linear=.2))
    assert position() == pytest.approx([0., 0., 0.])
    pump(n, 1.4, feed)  # Settling and one match are insufficient to commit.
    assert position() == pytest.approx([0., 0., 0.])
    deadline = time.monotonic() + 15.
    while not np.allclose(position(), target, atol=.015) and time.monotonic() < deadline:
        pump(n, .2, feed)
    assert position() == pytest.approx(target, abs=.015)
    # Startup retries cannot overwrite an already refined alignment.
    pump(n, .3, lambda: (feed(), n.seed.publish(p)))
    assert position() == pytest.approx(target, abs=.015)
    # New motion permits exactly another stationary refinement episode.
    pump(n, .3, lambda: feed(command_yaw=.4))
    target = np.array([.28, -.12, .06])
    deadline = time.monotonic() + 15.
    while not np.allclose(position(), target, atol=.015) and time.monotonic() < deadline:
        pump(n, .2, feed)
    assert position() == pytest.approx(target, abs=.015)


def test_tilted_registration_cannot_disable_navigation_ground_support(backend):
    n = backend
    stamp = n.get_clock().now().to_msg()
    p = seed(n, stamp)
    pump(n, .3, lambda: n.tf.publish(TFMessage(transforms=[odom_tf(stamp)])))
    pump(n, .3, lambda: n.seed.publish(p))
    assert n.acks
    # A well-conditioned, perfectly matchable 0.04 rad roll is nevertheless
    # incompatible with the navigation projection's 0.01 rad tilt limit.
    c, s = math.cos(.04), math.sin(.04)
    rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    cloud = (n.points - np.array([.1, 0., 0.])) @ rotation

    def feed():
        current = n.get_clock().now().to_msg()
        n.cmd.publish(Twist())
        odom = Odometry()
        odom.header = Header(stamp=current, frame_id='odom')
        odom.pose.pose.orientation.w = 1.
        n.odom.publish(odom)
        n.cloud.publish(create_cloud_xyz32(Header(stamp=current, frame_id='odom'), cloud))

    pump(n, 5., feed)
    assert 'rejected by navigation tilt/TF gate' in n.backend_log.read_text()
    t = n.transforms[-1].transform
    assert (t.translation.x, t.translation.y, t.translation.z) == pytest.approx((0., 0., 0.))
    assert (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w) == pytest.approx((0., 0., 0., 1.))
