#!/usr/bin/env python3
"""Conservative mesh screening of time-stamped oriented boxes and swept intervals.

Input JSON has poses: [{t, position: [x,y,z], quaternion_xyzw: [x,y,z,w],
world_from_pose_frame: 4x4}]. Each transform is measured at the pose timestamp;
position and quaternion are in the pose frame. Box bounds are in the body frame.
A clear interval requires a user-supplied upper bound on translation from either
endpoint and the mesh-free enclosing sphere; actual trajectories and rotations
between samples are otherwise unknown. Surface contact is not penetration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/global_relocalization'))
from actual_collision_support import CollisionSupportScene
try:
    from .geometry_audit import WORLD
except ImportError:
    from geometry_audit import WORLD


def intersects_box(tri, low, high):
    """Triangle/AABB separating-axis test, including degenerate/touching cases."""
    center = (low + high) / 2
    half = (high - low) / 2
    v = tri - center
    edges = [v[1] - v[0], v[2] - v[1], v[0] - v[2]]
    axes = list(np.eye(3)) + [np.cross(edges[0], edges[1])]
    axes += [np.cross(edge, axis) for edge in edges for axis in np.eye(3)]
    for axis in axes:
        if np.dot(axis, axis) < 1e-24:
            continue
        proj = v @ axis
        radius = half @ np.abs(axis)
        if proj.min() > radius + 1e-10 or proj.max() < -radius - 1e-10:
            return False
    return True


def rigid_transform(value):
    tf = np.asarray(value, dtype=float)
    if tf.shape != (4, 4) or not np.isfinite(tf).all() or not np.allclose(tf[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('world_from_pose_frame must be a finite homogeneous 4x4 matrix')
    r = tf[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError('transform rotation must be proper and orthonormal')
    return tf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--box-min', type=float, nargs=3, default=[-.31, -.31, -.08])
    parser.add_argument('--box-max', type=float, nargs=3, default=[.31, .31, .16])
    parser.add_argument('--max-between-sample-displacement', type=float, default=None,
                        help='Proven bound in metres on displacement from interval start, including tracking uncertainty')
    args = parser.parse_args()
    low, high = np.array(args.box_min), np.array(args.box_max)
    if not np.isfinite([low, high]).all() or np.any(low >= high):
        parser.error('invalid body box')
    if args.max_between_sample_displacement is not None and (not np.isfinite(args.max_between_sample_displacement) or args.max_between_sample_displacement < 0):
        parser.error('invalid displacement bound')
    poses = json.loads(args.poses.read_text())['poses']
    if not poses:
        parser.error('at least one pose is required')
    scene = CollisionSupportScene.from_world(WORLD)
    radius = float(np.linalg.norm(np.maximum(abs(low), abs(high))))
    results, centers = [], []
    previous_t = None
    for pose in poses:
        t = float(pose['t'])
        if not np.isfinite(t) or (previous_t is not None and t <= previous_t):
            raise ValueError('pose timestamps must be finite and strictly increasing')
        previous_t = t
        tf = rigid_transform(pose['world_from_pose_frame'])
        p = np.asarray(pose['position'], float)
        q = np.asarray(pose['quaternion_xyzw'], float)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1, atol=1e-5):
            raise ValueError('finite position and unit quaternion required')
        center = tf[:3, :3] @ p + tf[:3, 3]
        rotation = tf[:3, :3] @ Rotation.from_quat(q).as_matrix()
        centers.append(center)
        near = np.all(scene.maximum >= center - radius, axis=1) & np.all(scene.minimum <= center + radius, axis=1)
        hits = set()
        for index in np.flatnonzero(near):
            triangle_body = (scene.triangles[index] - center) @ rotation
            if intersects_box(triangle_body, low, high):
                hits.add(scene.names[index].split('#')[0])
        results.append({'t': t, 'world_position': center.tolist(), 'overlap_surface_labels': sorted(hits)})
    intervals = []
    for i in range(len(centers)-1):
        bound = args.max_between_sample_displacement
        item = {'start_t': results[i]['t'], 'end_t': results[i+1]['t'], 'status': 'indeterminate'}
        if bound is not None:
            if np.linalg.norm(centers[i+1] - centers[i]) > bound + 1e-8:
                raise ValueError('displacement bound is smaller than endpoint displacement')
            # This AABB encloses all rotations and all translations within bound.
            extent = radius + bound
            near = np.all(scene.maximum >= centers[i] - extent, axis=1) & np.all(scene.minimum <= centers[i] + extent, axis=1)
            if not np.any(near):
                item['status'] = 'mesh_surface_free_conservative_bound'
            else:
                item['candidate_surface_labels'] = sorted({scene.names[j].split('#')[0] for j in np.flatnonzero(near)})
        intervals.append(item)
    sources = {str(p): v for p, v in scene.source_sha256.items()}
    output = {'pose_input_sha256': hashlib.sha256(args.poses.read_bytes()).hexdigest(),
              'scene_source_sha256': sources, 'body_box_min': low.tolist(), 'body_box_max': high.tolist(),
              'max_between_sample_displacement': args.max_between_sample_displacement,
              'pose_surface_overlaps': results, 'swept_intervals': intervals,
              'limitations': ['Box is a conservative proxy; overlap may be normal floor or slope support, not physical penetration.',
                 'Surface-free status requires externally established motion bound, pose and transform accuracy, and complete watertight geometry; enclosing AABB does not detect a body already inside a closed solid.',
                 'Physics-engine margins and dynamic obstacles are not represented; no inference about sensor return origin or protection necessity.']}
    args.output.write_text(json.dumps(output, indent=2) + '\n')


if __name__ == '__main__':
    main()
