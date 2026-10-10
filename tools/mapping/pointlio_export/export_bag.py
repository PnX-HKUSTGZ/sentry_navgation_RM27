#!/usr/bin/env python3
"""Read-only rosbag2 -> ERASOR2 local scans, fixed-world poses and audit manifest."""
import argparse
import bisect
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np
import yaml

from atomic_directory import commit_directory


# sensor_msgs/PointField datatype codes; stride/padding and endianness are explicit.
FIELD_TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def read_cloud(msg, missing_intensity='error'):
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        raise ValueError('Empty/invalid PointCloud2 dimensions')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Invalid PointCloud2 row_step/data length')
    fields = {}
    for field in msg.fields:
        if field.name in fields:
            raise ValueError('Duplicate PointCloud2 field: ' + field.name)
        fields[field.name] = field
    columns = []
    for name in ('x', 'y', 'z', 'intensity'):
        if name not in fields:
            if name == 'intensity' and missing_intensity == 'zero':
                columns.append(np.zeros((msg.height, msg.width)))
                continue
            raise ValueError('Missing PointCloud2 field: ' + name)
        field = fields[name]
        if field.count != 1 or field.datatype not in FIELD_TYPES:
            raise ValueError('Unsupported scalar PointCloud2 field: ' + name)
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + FIELD_TYPES[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > msg.point_step:
            raise ValueError('Field exceeds point_step: ' + name)
        columns.append(np.ndarray((msg.height, msg.width), dtype=dtype,
                                  buffer=bytes(msg.data), offset=field.offset,
                                  strides=(msg.row_step, msg.point_step)).astype(np.float64))
    points = np.stack(columns, axis=-1).reshape(-1, 4)
    finite = np.isfinite(points).all(axis=1)
    return points[finite], int((~finite).sum())


def stamp_ns(msg):
    sec, nanosec = msg.header.stamp.sec, msg.header.stamp.nanosec
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('Invalid header timestamp')
    return sec * 1_000_000_000 + nanosec


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12 or abs(norm - 1) > .01:
        raise ValueError('Nonfinite/invalid pose quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def odometry_pose(msg):
    pose = msg.pose.pose
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    t = np.array([pose.position.x, pose.position.y, pose.position.z])
    if not np.isfinite(t).all():
        raise ValueError('Nonfinite pose translation')
    result = np.eye(4)
    result[:3, :3] = quaternion_rotation(q)
    result[:3, 3] = t
    return result, q / np.linalg.norm(q)


def validate_transform(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Transform must be finite 4x4')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('Invalid homogeneous bottom row')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6):
        raise ValueError('Transform rotation must be a proper rotation')
    return matrix


def load_transform(path, key):
    data = yaml.safe_load(Path(path).read_text())
    return validate_transform(data[key])


def match_pose(times, poses, stamp, mode, tolerance_ns, max_gap_ns):
    index = bisect.bisect_left(times, stamp)
    if index < len(times) and times[index] == stamp:
        return poses[index][0], {'method': 'exact', 'pose_stamp_ns': stamp, 'error_ns': 0}
    if mode == 'exact':
        raise ValueError(f'No exactly matching pose for cloud header stamp {stamp}')
    if mode == 'nearest':
        candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
        best = min(candidates, key=lambda i: abs(times[i] - stamp))
        error = times[best] - stamp
        if abs(error) > tolerance_ns:
            raise ValueError(f'Pose/cloud delta {error} ns exceeds tolerance for {stamp}')
        return poses[best][0], {'method': 'nearest', 'pose_stamp_ns': times[best], 'error_ns': error}
    if not 0 < index < len(times):
        raise ValueError('Interpolation requires bracketing poses; extrapolation forbidden')
    left, right = index - 1, index
    gap = times[right] - times[left]
    if gap > max_gap_ns:
        raise ValueError(f'Interpolation gap {gap} ns exceeds max-gap')
    alpha = (stamp - times[left]) / gap
    a, qa = poses[left]
    b, qb = poses[right]
    dot = float(qa @ qb)
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > .9995:
        q = qa + alpha * (qb - qa)
    else:
        angle = np.arccos(np.clip(dot, -1, 1))
        q = (np.sin((1-alpha)*angle)*qa + np.sin(alpha*angle)*qb) / np.sin(angle)
    result = np.eye(4)
    result[:3, :3] = quaternion_rotation(q / np.linalg.norm(q))
    result[:3, 3] = (1-alpha)*a[:3, 3] + alpha*b[:3, 3]
    return result, {'method': 'interpolate', 'left_ns': times[left], 'right_ns': times[right],
                    'alpha': alpha, 'gap_ns': gap, 'error_ns': 0}


def bag_messages(bag, topics, storage_id):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = set(topics) - types.keys()
    if missing:
        raise ValueError(f'Topics absent from bag: {sorted(missing)}')
    expected = {'cloud': 'sensor_msgs/msg/PointCloud2', 'pose': 'nav_msgs/msg/Odometry'}
    for name, role in topics.items():
        if types[name] != expected[role]:
            raise ValueError(f'Unexpected topic type {name}: {types[name]}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    classes = {t: get_message(types[t]) for t in topics}
    while reader.has_next():
        name, data, receive_ns = reader.read_next()
        yield name, deserialize_message(data, classes[name]), receive_ns


def export(args):
    bag = args.bag.expanduser().resolve(strict=True)
    candidate = args.output.expanduser().absolute()
    output = candidate.parent.resolve() / candidate.name
    if bag == output or bag in output.parents:
        raise ValueError("Output must be outside the source bag directory")
    if os.path.lexists(output):
        raise ValueError('Output already exists; select a new sequence directory')
    if not bag.is_dir():
        raise ValueError('Bag must be a rosbag2 directory with metadata.yaml')
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())
    storage_id = metadata['rosbag2_bagfile_information']['storage_identifier']
    # A virtual scan at body origin is explicitly named; a real lidar origin requires calibration.
    body_to_scan = np.eye(4)
    if args.scan_frame == 'lidar':
        if args.extrinsics is None:
            raise ValueError('--scan-frame lidar requires --extrinsics (T_body_lidar), fixed calibration only')
        body_to_scan = load_transform(args.extrinsics, 'T_body_lidar')
    elif args.extrinsics is not None:
        raise ValueError('--extrinsics is applicable only with --scan-frame lidar')
    alignment = load_transform(args.alignment, 'T_output_world') if args.alignment else np.eye(4)
    if args.output_world_frame and not args.alignment:
        raise ValueError('--output-world-frame requires a fixed --alignment')
    if args.alignment and not args.output_world_frame:
        raise ValueError('--alignment requires explicit --output-world-frame')
    poses_by_time = {}
    for _, msg, _ in bag_messages(bag, {args.pose_topic: 'pose'}, storage_id):
        if msg.header.frame_id != args.world_frame or msg.child_frame_id != args.pose_child_frame:
            raise ValueError(f'Odometry frames must be {args.world_frame}->{args.pose_child_frame}')
        stamp = stamp_ns(msg)
        pose = odometry_pose(msg)
        if stamp in poses_by_time:
            raise ValueError(f'Duplicate pose header timestamp: {stamp}')
        poses_by_time[stamp] = pose
    times = sorted(poses_by_time)
    poses = [poses_by_time[t] for t in times]
    if not times:
        raise ValueError('No poses in bag')
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.' + output.name + '.', dir=output.parent))
    records, previous = [], None
    try:
        (stage / 'velodyne').mkdir()
        with (stage / 'poses_suma_optim.txt').open('w') as pose_file, (stage / 'times.txt').open('w') as time_file:
            for _, msg, receive_ns in bag_messages(bag, {args.cloud_topic: 'cloud'}, storage_id):
                stamp = stamp_ns(msg)
                if msg.header.frame_id != args.world_frame:
                    raise ValueError(f'Cloud must be in fixed world frame {args.world_frame}')
                if previous is not None and stamp <= previous:
                    raise ValueError('Cloud header times must be strictly increasing')
                previous = stamp
                body_pose, report = match_pose(times, poses, stamp, args.match,
                                              round(args.tolerance_ms * 1e6), round(args.max_gap_ms * 1e6))
                world_scan = body_pose @ body_to_scan
                points, removed = read_cloud(msg, args.missing_intensity)
                if not len(points):
                    raise ValueError('No finite points in cloud')
                points[:, :3] = (points[:, :3] - world_scan[:3, 3]) @ world_scan[:3, :3]
                with np.errstate(over='ignore'):
                    encoded = points.astype('<f4')
                if not np.isfinite(encoded).all():
                    raise ValueError('Local point value exceeds float32 range')
                frame = len(records)
                scan_file = stage / 'velodyne' / f'{frame:06d}.bin'
                encoded.tofile(scan_file)
                output_scan = alignment @ world_scan
                pose_file.write(' '.join(f'{x:.12g}' for x in output_scan[:3].ravel()) + '\n')
                time_file.write(f'{stamp // 1_000_000_000}.{stamp % 1_000_000_000:09d}\n')
                records.append({'frame': frame, 'header_stamp_ns': stamp, 'bag_receive_ns': receive_ns,
                                'points': len(points), 'dropped_nonfinite': removed,
                                'sha256': hashlib.sha256(scan_file.read_bytes()).hexdigest(), 'pose_match': report})
        if not records:
            raise ValueError('No clouds in bag')
        manifest = {'schema': 1, 'source_bag': str(bag), 'source_metadata_sha256': hashlib.sha256((bag/'metadata.yaml').read_bytes()).hexdigest(),
                    'storage_id': storage_id, 'cloud_topic': args.cloud_topic, 'pose_topic': args.pose_topic,
                    'world_frame': args.world_frame, 'output_world_frame': args.output_world_frame or args.world_frame,
                    'scan_frame': args.scan_frame, 'pose_child_frame': args.pose_child_frame,
                    'pose_convention': 'T_output_scan, direct 3x4 row-major; no KITTI camera calibration',
                    'point_format': 'little-endian float32 x,y,z,intensity',
                    'body_scan_warning': 'Virtual scan at IMU/body origin; ray origin differs from LiDAR by extrinsics' if args.scan_frame == 'body' else None,
                    'calibration_assumption': 'Fixed T_body_lidar; runtime extrinsic estimation must be disabled or known fixed',
                    'T_body_scan': body_to_scan.tolist(), 'T_output_world': alignment.tolist(),
                    'time_policy': {'method': args.match, 'tolerance_ms': args.tolerance_ms, 'max_gap_ms': args.max_gap_ms,
                                    'clock': 'message header; bag receive time never used for matching'},
                    'frame_count': len(records), 'pose_count': len(poses), 'frames': records}
        (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        # Linux RENAME_NOREPLACE atomically commits and rejects any concurrently created target.
        if os.path.lexists(output):
            raise ValueError('Output appeared during export')
        commit_directory(stage, output)
    except BaseException:
        shutil.rmtree(stage, ignore_errors=True)
        raise
    print(f'Exported {len(records)} frames to {output} (scan origin: {args.scan_frame})')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--cloud-topic', default='/cloud_registered', choices=['/cloud_registered', '/cloud_registered_full'])
    p.add_argument('--pose-topic', default='/aft_mapped_to_init')
    p.add_argument('--world-frame', choices=['camera_init', 'odom'], default='camera_init')
    p.add_argument('--pose-child-frame', default='body')
    p.add_argument('--scan-frame', choices=['body', 'lidar'], default='body')
    p.add_argument('--extrinsics', type=Path, help='YAML with calibrated T_body_lidar; no dynamic extrinsic estimation')
    p.add_argument('--alignment', type=Path, help='YAML containing one fixed T_output_world')
    p.add_argument('--output-world-frame')
    p.add_argument('--match', choices=['exact', 'nearest', 'interpolate'], default='exact')
    p.add_argument('--tolerance-ms', type=float, default=0.0)
    p.add_argument('--max-gap-ms', type=float, default=0.0)
    p.add_argument('--missing-intensity', choices=['error', 'zero'], default='error')
    return p


def main():
    p = parser()
    args = p.parse_args()
    if not np.isfinite([args.tolerance_ms, args.max_gap_ms]).all() or min(args.tolerance_ms, args.max_gap_ms) < 0:
        p.error('Matching bounds must be finite and nonnegative')
    if args.match == 'nearest' and args.tolerance_ms <= 0:
        p.error('nearest requires positive --tolerance-ms')
    if args.match == 'interpolate' and args.max_gap_ms <= 0:
        p.error('interpolate requires positive --max-gap-ms')
    try:
        export(args)
    except (ValueError, OSError, KeyError, RuntimeError) as error:
        p.exit(1, f'Export failed: {error}\n')


if __name__ == '__main__':
    main()
