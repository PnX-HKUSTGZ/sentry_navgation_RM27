#!/usr/bin/env python3
"""Anchor Point-LIO odometry once using a timestamped global localization result."""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s)


def quaternion(q):
    values = (q.x, q.y, q.z, q.w)
    norm = sum(v*v for v in values)
    if not all(math.isfinite(v) for v in values) or abs(norm-1.0) > 1e-3:
        raise ValueError('Invalid unit quaternion')
    return tuple(v/math.sqrt(norm) for v in values)


def alignment(map_pose, odom_transform):
    """T_map_odom = T_map_base(t) * inverse(T_odom_base(t)), including tilt."""
    qm, qo = quaternion(map_pose.orientation), quaternion(odom_transform.rotation)
    q = multiply(qm, (-qo[0], -qo[1], -qo[2], qo[3]))
    p, o = map_pose.position, odom_transform.translation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, o.x, o.y, o.z)):
        raise ValueError('Invalid translation')
    rotated = multiply(multiply(q, (o.x, o.y, o.z, 0.)), (-q[0], -q[1], -q[2], q[3]))
    return (p.x-rotated[0], p.y-rotated[1], p.z-rotated[2]), q


class GlobalPoseHandoff(Node):
    def __init__(self):
        super().__init__('global_pose_handoff')
        self.buffer = Buffer(cache_time=Duration(seconds=60.))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.accepted_pose = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ack = self.create_publisher(
            PoseWithCovarianceStamped, '/relocalization/initialpose_accepted', qos)
        self.create_subscription(PoseWithCovarianceStamped, '/relocalization/initialpose',
                                 self.accept, 10)

    def accept(self, msg):
        # Startup retries are idempotent. A running navigator cannot be reseeded.
        if self.accepted_pose is not None:
            if msg == self.accepted_pose:
                self.ack.publish(msg)
            return
        stamp = msg.header.stamp
        if (msg.header.frame_id != 'map' or stamp.sec < 0 or
                not 0 <= stamp.nanosec < 1_000_000_000 or
                stamp.sec == 0 and stamp.nanosec == 0):
            return
        try:
            # Never substitute the latest pose: the robot may have rotated
            # between scan acquisition and completion of global matching.
            odom = self.buffer.lookup_transform('odom', 'base_footprint',
                                                Time.from_msg(msg.header.stamp))
            p, q = alignment(msg.pose.pose, odom.transform)
        except TransformException:
            return  # The coordinator retries while timestamped TF arrives.
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = 'odom'
        translation, rotation = transform.transform.translation, transform.transform.rotation
        translation.x, translation.y, translation.z = p
        rotation.x, rotation.y, rotation.z, rotation.w = q
        self.broadcaster.sendTransform(transform)
        self.accepted_pose = msg
        self.ack.publish(msg)
        self.get_logger().info('Initialized map->odom from global pose and timestamped Point-LIO TF')


def main():
    rclpy.init()
    node = GlobalPoseHandoff()
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
