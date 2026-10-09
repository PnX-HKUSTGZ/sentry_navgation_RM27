#!/usr/bin/env python3
"""Geometry/parser checks and optional real rosbag2 round-trip fixture."""
import argparse
from pathlib import Path
import struct
from types import SimpleNamespace as NS
import unittest

import numpy as np

from atomic_directory import commit_directory
from export_bag import match_pose, quaternion_rotation, read_cloud


def cloud_message(points, bigendian=False, organized=False):
    from sensor_msgs.msg import PointCloud2, PointField
    count = len(points)
    height = 2 if organized and count % 2 == 0 else 1
    width = count // height
    point_step = 24
    row_step = width * point_step + 12
    payload = bytearray(height * row_step)
    order = '>' if bigendian else '<'
    for i, point in enumerate(points):
        row, column = divmod(i, width)
        # x offset=4, y=8, z=12, intensity=16; padded point and padded row.
        struct.pack_into(order + 'ffff', payload, row * row_step + column * point_step + 4, *point)
    msg = PointCloud2(height=height, width=width, is_bigendian=bigendian,
                      point_step=point_step, row_step=row_step, is_dense=False, data=bytes(payload))
    msg.fields = [PointField(name=name, offset=4+4*i, datatype=7, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    return msg


class GeometryTests(unittest.TestCase):
    def test_padded_bigendian_and_finite(self):
        values = np.array([[1, 2, 3, 4], [-4, .2, 7, 8], [1, 1, 1, np.nan], [9, 8, 7, 6]], dtype=float)
        # Parser can be tested without ROS message classes.
        def fake(points):
            data = bytearray(2 * 64)
            for i, row in enumerate(points):
                struct.pack_into('>ffff', data, (i//2)*64+(i%2)*24+4, *row)
            return NS(height=2, width=2, point_step=24, row_step=64, data=bytes(data), is_bigendian=True,
                      fields=[NS(name=n,offset=4+4*i,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))])
        msg = fake(values)
        actual, dropped = read_cloud(msg)
        np.testing.assert_allclose(actual, values[[0,1,3]], rtol=1e-6)
        self.assertEqual(dropped, 1)
        msg.row_step = 47
        with self.assertRaises(ValueError): read_cloud(msg)
        msg = fake(values); msg.fields[-1].offset=24
        with self.assertRaises(ValueError): read_cloud(msg)

    def test_matching_is_bounded_and_slerp(self):
        qa = np.array([0,0,0,1.])
        qb = np.array([0,0,1.,0])
        a = np.eye(4); b=np.eye(4)
        b[:3,:3]=quaternion_rotation(qb); b[:3,3]=[2,0,0]
        poses=[(a,qa),(b,qb)]
        actual, report = match_pose([10,30],poses,20,'interpolate',0,20)
        np.testing.assert_allclose(actual[:3,3],[1,0,0])
        np.testing.assert_allclose(actual[:3,:3] @ [1,0,0],[0,1,0],atol=1e-8)
        self.assertEqual(report['alpha'],.5)
        for mode, stamp, tol, gap in [('exact',20,0,0),('nearest',20,9,0),('interpolate',9,0,100),('interpolate',20,0,19)]:
            with self.assertRaises(ValueError): match_pose([10,30],poses,stamp,mode,tol,gap)
        _, report=match_pose([10,30],poses,29,'nearest',1,0)
        self.assertEqual(report['error_ns'],1)

    def test_atomic_no_replace(self):
        import tempfile
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'stage'; destination=Path(temp)/'output'
            source.mkdir(); destination.mkdir()
            with self.assertRaises(FileExistsError): commit_directory(source,destination)
            self.assertTrue(source.is_dir())


def make_bag(output, bad_pose_frame=None):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    output = output.resolve()
    if output.exists(): raise ValueError('Fixture output already exists')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name='/cloud_registered', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(id=1, name='/aft_mapped_to_init', type='nav_msgs/msg/Odometry', serialization_format='cdr'))
    rng = np.random.default_rng(1234)
    ground = np.column_stack([rng.uniform(-9,9,3000),rng.uniform(-9,9,3000),rng.normal(-.45,.008,3000),rng.uniform(1,100,3000)])
    pole = np.column_stack([rng.normal(3,.1,200),rng.normal(2,.1,200),rng.uniform(-.4,1.5,200),rng.uniform(1,100,200)])
    expected = []
    for frame in range(8):
        stamp=1_000_000_000+frame*100_000_000
        yaw=.07*frame
        q=np.array([0,0,np.sin(yaw/2),np.cos(yaw/2)])
        rotation=quaternion_rotation(q)
        translation=np.array([.15*frame,.03*frame,0])
        local=np.vstack([ground,pole]).copy()
        # Dynamic compact box, translated across successive frames.
        box=np.column_stack([rng.normal(1+frame*.25,.15,100),rng.normal(-2,.15,100),rng.uniform(-.4,.7,100),rng.uniform(1,100,100)])
        local=np.vstack([local,box])
        world=local.copy(); world[:,:3]=local[:,:3]@rotation.T+translation
        if frame==0: world=np.vstack([world,[np.nan,1,1,3]])
        cloud=cloud_message(world, bigendian=(frame%2==1), organized=(frame%2==1))
        cloud.header.frame_id='camera_init'; cloud.header.stamp.sec=stamp//1_000_000_000; cloud.header.stamp.nanosec=stamp%1_000_000_000
        from copy import deepcopy
        odom=Odometry(); odom.header=deepcopy(cloud.header); odom.child_frame_id='body'
        odom.pose.pose.position.x=float(translation[0]); odom.pose.pose.position.y=float(translation[1])
        odom.pose.pose.orientation.z=float(q[2]); odom.pose.pose.orientation.w=float(q[3])
        if frame == bad_pose_frame:
            odom.header.stamp.nanosec += 1
        # Receive times differ and odometry precedes cloud: header time is authoritative.
        writer.write('/aft_mapped_to_init',serialize_message(odom),stamp+50_000_000)
        writer.write('/cloud_registered',serialize_message(cloud),stamp+60_000_000)
        expected.append(local.astype('<f4'))
    del writer
    np.savez(output.parent/'expected_local.npz', **{f'frame_{i}':x for i,x in enumerate(expected)})
    print('Synthetic real rosbag2 fixture:', output)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--make-bag',type=Path)
    p.add_argument('--bad-pose-frame',type=int)
    args=p.parse_args()
    if args.make_bag: make_bag(args.make_bag,args.bad_pose_frame)
    else: unittest.main(argv=['test_export.py'])
