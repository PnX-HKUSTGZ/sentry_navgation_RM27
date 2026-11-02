#!/usr/bin/env python3
"""Outcome-independent stationary placement checks against actual SDF collisions.

The accepted scope is horizontal, supported placement with clearance for a full
stationary turn. Slopes, steps, unsupported placements, and low overhead geometry
are rejected geometrically. High flat platforms use their actual collision height.
"""
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _pose(element):
    result = np.eye(4)
    if element is None or not element.text:
        return result
    if element.get('relative_to'):
        raise ValueError('relative_to SDF poses require explicit frame resolution')
    values = np.fromstring(element.text, sep=' ')
    if len(values) == 7 and element.get('rotation_format') == 'quat_xyzw':
        result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    elif len(values) == 6:
        angles = np.deg2rad(values[3:]) if element.get('degrees') == 'true' else values[3:]
        result[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
    else:
        raise ValueError('unsupported SDF pose encoding')
    result[:3, 3] = values[:3]
    return result


def _urdf_origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return result


def _transform(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def _box(size):
    vertices = np.array([[x, y, z] for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]], float) * np.asarray(size) / 2
    faces = [[0, 3, 2], [0, 1, 3], [4, 6, 7], [4, 7, 5],
             [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
             [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return vertices[np.asarray(faces)]


def _stl(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else -1
    if len(data) != 84 + count * 50:
        raise ValueError('collision mesh must be binary STL: ' + str(path))
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(data, dtype=dtype, offset=84, count=count)['vertices'].astype(float)


def robot_collision_envelope(robot_urdf_path):
    path = Path(robot_urdf_path).resolve()
    robot = ET.parse(path).getroot()
    joints = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    transforms = {}
    def link_transform(name):
        if name not in transforms:
            if name not in joints:
                transforms[name] = np.eye(4)
            else:
                joint = joints[name]
                if joint.get('type') != 'fixed':
                    raise ValueError('stationary envelope requires fixed robot joint poses')
                transforms[name] = link_transform(joint.find('parent').get('link')) @ _urdf_origin(joint.find('origin'))
        return transforms[name]
    minimum_z, maximum_z, radius = float('inf'), -float('inf'), 0.0
    shapes = []
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            transform = link_transform(link.get('name')) @ _urdf_origin(collision.find('origin'))
            geometry = collision.find('geometry')
            if geometry.find('sphere') is not None:
                r = float(geometry.find('sphere').get('radius'))
                center = transform[:3, 3]
                low, high = center[2]-r, center[2]+r
                extent = np.linalg.norm(center[:2]) + r
                kind = 'sphere'
            elif geometry.find('box') is not None:
                size = np.fromstring(geometry.find('box').get('size'), sep=' ')
                points = _transform(_box(size).reshape(-1, 3), transform)
                low, high, extent = points[:, 2].min(), points[:, 2].max(), np.linalg.norm(points[:, :2], axis=1).max()
                kind = 'box'
            elif geometry.find('cylinder') is not None:
                cylinder = geometry.find('cylinder')
                if not np.allclose(np.abs(transform[:3, 2]), [0, 0, 1], atol=1e-7):
                    raise ValueError('tilted robot cylinders require a more general envelope')
                r, length = float(cylinder.get('radius')), float(cylinder.get('length'))
                low, high = transform[2, 3]-length/2, transform[2, 3]+length/2
                extent = np.linalg.norm(transform[:2, 3]) + r
                kind = 'cylinder'
            else:
                raise ValueError('unsupported robot collision primitive')
            minimum_z, maximum_z, radius = min(minimum_z, low), max(maximum_z, high), max(radius, extent)
            shapes.append({'link': link.get('name'), 'kind': kind, 'minimum_z': float(low), 'maximum_z': float(high), 'swept_radius': float(extent)})
    if not shapes or minimum_z >= 0:
        raise ValueError('robot contact plane cannot be established from its collisions')
    return {'robot_urdf_path': str(path), 'robot_urdf_sha256': _sha(path),
            'base_height_above_contact': float(-minimum_z),
            'height_above_contact': float(maximum_z-minimum_z), 'swept_radius': float(radius), 'shapes': shapes}


def _clip_z(polygon, z, above):
    result = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        a_inside = a[2] >= z if above else a[2] <= z
        b_inside = b[2] >= z if above else b[2] <= z
        if a_inside:
            result.append(a)
        if a_inside != b_inside:
            result.append(a + (b-a) * ((z-a[2])/(b[2]-a[2])))
    return np.asarray(result).reshape(-1, 3)


def _xy_polygon_distance(point, polygon):
    vertices = polygon[:, :2] - point
    a, b = vertices, np.roll(vertices, -1, axis=0)
    edges = b-a
    length = (edges*edges).sum(axis=1)
    t = np.clip(-(a*edges).sum(axis=1)/np.maximum(length, 1e-20), 0, 1)
    distance = float(np.linalg.norm(a+t[:, None]*edges, axis=1).min())
    cross = edges[:, 0]*(-a[:, 1])-edges[:, 1]*(-a[:, 0])
    area = abs(float(np.sum(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0])))
    if area > 1e-12 and (np.all(cross >= -1e-10) or np.all(cross <= 1e-10)):
        return 0.0
    return distance


class CollisionSupportScene:
    def __init__(self, triangles, names, *, max_support_height=.6):
        self.triangles = np.asarray(triangles, float).reshape(-1, 3, 3)
        self.names = list(names)
        if len(self.names) != len(self.triangles):
            raise ValueError('collision labels must correspond to triangles')
        self.minimum = self.triangles.min(axis=1)
        self.maximum = self.triangles.max(axis=1)
        normals = np.cross(self.triangles[:, 1]-self.triangles[:, 0], self.triangles[:, 2]-self.triangles[:, 0])
        self.upward = normals[:, 2] > 1e-10
        self.max_support_height = max_support_height
        self.height_cache = {}
        self.offsets_cache = {}
        self.buckets = defaultdict(list)
        for index, (low, high) in enumerate(zip(self.minimum, self.maximum)):
            for x in range(math.floor(low[0]/.5), math.floor(high[0]/.5)+1):
                for y in range(math.floor(low[1]/.5), math.floor(high[1]/.5)+1):
                    self.buckets[x, y].append(index)
        self.source_sha256 = {}

    @classmethod
    def from_world(cls, world_path, model_roots=None, **kwargs):
        world_path = Path(world_path).resolve()
        model_roots = [Path(p).resolve() for p in (model_roots or [world_path.parents[2] / 'meshes'])]
        sources = {str(world_path): _sha(world_path)}
        triangles, names = [], []
        def resolve(uri, relative):
            if uri.startswith('model://'):
                name = uri[len('model://'):]
                candidates = [root/name for root in model_roots]
            elif uri.startswith('file://'):
                candidates = [Path(uri[len('file://'):])]
            else:
                candidates = [relative/uri]
            for candidate in candidates:
                if candidate.exists():
                    return candidate.resolve()
            raise FileNotFoundError('unresolved collision resource: ' + uri)
        def model(element, parent, relative):
            frame = parent @ _pose(element.find('pose'))
            for link in element.findall('link'):
                link_frame = frame @ _pose(link.find('pose'))
                for collision in link.findall('collision'):
                    transform = link_frame @ _pose(collision.find('pose'))
                    geometry = collision.find('geometry')
                    if geometry.find('box') is not None:
                        mesh = _box(np.fromstring(geometry.findtext('box/size'), sep=' '))
                    elif geometry.find('mesh') is not None:
                        path = resolve(geometry.findtext('mesh/uri'), relative)
                        sources[str(path)] = _sha(path)
                        mesh = _stl(path)
                        scale = np.fromstring(geometry.findtext('mesh/scale', '1 1 1'), sep=' ')
                        mesh *= scale
                    else:
                        raise ValueError('unsupported scene collision geometry: ' + collision.get('name', ''))
                    mesh = _transform(mesh, transform)
                    triangles.extend(mesh)
                    label = element.get('name', '')+'/'+collision.get('name', '')+'#'+str(len(names))
                    names.extend([label]*len(mesh))
            for child in element.findall('model'):
                model(child, frame, relative)
        world = ET.parse(world_path).getroot().find('world')
        if world is None:
            raise ValueError('SDF has no world')
        for child in world.findall('model'):
            model(child, np.eye(4), world_path.parent)
        for include in world.findall('include'):
            path = resolve(include.findtext('uri'), world_path.parent)
            path = path/'model.sdf' if path.is_dir() else path
            sources[str(path)] = _sha(path)
            element = ET.parse(path).getroot().find('model')
            if element is None:
                raise ValueError('included SDF has no model')
            model(element, _pose(include.find('pose')), path.parent)
        scene = cls(triangles, names, **kwargs)
        scene.source_sha256 = sources
        return scene

    def height_at(self, xy):
        key = (round(float(xy[0]), 8), round(float(xy[1]), 8))
        if key in self.height_cache:
            return self.height_cache[key]
        indices = np.asarray(self.buckets.get((math.floor(xy[0]/.5), math.floor(xy[1]/.5)), []), int)
        if not len(indices):
            self.height_cache[key] = float('nan')
            return self.height_cache[key]
        triangles = self.triangles[indices]
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        denominator = (b[:, 1]-c[:, 1])*(a[:, 0]-c[:, 0])+(c[:, 0]-b[:, 0])*(a[:, 1]-c[:, 1])
        valid = self.upward[indices] & (abs(denominator)>1e-12)
        u, v = np.zeros(len(indices)), np.zeros(len(indices))
        u[valid] = ((b[:, 1]-c[:, 1])*(xy[0]-c[:, 0])+(c[:, 0]-b[:, 0])*(xy[1]-c[:, 1]))[valid]/denominator[valid]
        v[valid] = ((c[:, 1]-a[:, 1])*(xy[0]-c[:, 0])+(a[:, 0]-c[:, 0])*(xy[1]-c[:, 1]))[valid]/denominator[valid]
        w = 1-u-v
        z = u*a[:, 2]+v*b[:, 2]+w*c[:, 2]
        valid &= (u>=-1e-7)&(v>=-1e-7)&(w>=-1e-7)&(z>=-.01)&(z<=self.max_support_height+1e-8)
        value = float(z[valid].max()) if valid.any() else float('nan')
        self.height_cache[key] = value
        return value

    def contains_point(self, xyz):
        indices = np.asarray(self.buckets.get((math.floor(xyz[0]/.5), math.floor(xyz[1]/.5)), []), int)
        if not len(indices):
            return False
        a, b, c = self.triangles[indices, 0], self.triangles[indices, 1], self.triangles[indices, 2]
        denominator = (b[:, 1]-c[:, 1])*(a[:, 0]-c[:, 0])+(c[:, 0]-b[:, 0])*(a[:, 1]-c[:, 1])
        valid = abs(denominator)>1e-12
        u, v = np.zeros(len(indices)), np.zeros(len(indices))
        u[valid] = ((b[:, 1]-c[:, 1])*(xyz[0]-c[:, 0])+(c[:, 0]-b[:, 0])*(xyz[1]-c[:, 1]))[valid]/denominator[valid]
        v[valid] = ((c[:, 1]-a[:, 1])*(xyz[0]-c[:, 0])+(a[:, 0]-c[:, 0])*(xyz[1]-c[:, 1]))[valid]/denominator[valid]
        w = 1-u-v
        z = u*a[:, 2]+v*b[:, 2]+w*c[:, 2]
        valid &= (u>=-1e-7)&(v>=-1e-7)&(w>=-1e-7)&(z>xyz[2]+1e-8)
        crossings = defaultdict(set)
        for index, height in zip(indices[valid], z[valid]):
            crossings[self.names[index]].add(round(float(height), 7))
        return any(len(heights)%2 == 1 for heights in crossings.values())

    def check_position(self, xy, envelope, *, support_tolerance=.005, sample_resolution=.025,
                       radial_margin=.02, separation_margin=.002):
        xy = np.asarray(xy, float)
        radius = envelope['swept_radius'] + radial_margin
        # Pad the sampled disk by half a cell diagonal to cover its boundary.
        key = (radius, sample_resolution)
        if key not in self.offsets_cache:
            steps = math.ceil(radius/sample_resolution)
            self.offsets_cache[key] = [(x*sample_resolution, y*sample_resolution)
                for x in range(-steps, steps+1) for y in range(-steps, steps+1)
                if math.hypot(x, y)*sample_resolution <= radius+sample_resolution/math.sqrt(2)]
        x0, y0 = map(float, xy)
        heights = [self.height_at((x0+dx, y0+dy)) for dx, dy in self.offsets_cache[key]]
        if not np.isfinite(heights).all():
            return False, float('nan'), 'unsupported_footprint'
        low, high = min(heights), max(heights)
        if high-low > support_tolerance+1e-8:
            return False, float('nan'), 'nonplanar_support'
        lower = high + separation_margin
        upper = high + envelope['height_above_contact'] + separation_margin
        if self.contains_point([x0, y0, (lower+upper)/2]):
            return False, float('nan'), 'inside_collision_solid'
        nearby = np.all(self.maximum[:, :2]>=xy-radius, axis=1)&np.all(self.minimum[:, :2]<=xy+radius, axis=1)
        nearby &= (self.maximum[:, 2]>lower+1e-8)&(self.minimum[:, 2]<upper-1e-8)
        for index in np.flatnonzero(nearby):
            polygon = _clip_z(self.triangles[index], lower, True)
            if not len(polygon):
                continue
            polygon = _clip_z(polygon, upper, False)
            if len(polygon) and _xy_polygon_distance(xy, polygon) < radius-1e-8:
                return False, float('nan'), 'swept_volume_collision'
        return True, float(high), 'accepted'


def filter_supported_positions(points_xy, *, world_path, robot_urdf_path, model_roots=None,
                               max_support_height=.6, support_tolerance=.005,
                               sample_resolution=.025, radial_margin=.02, separation_margin=.002,
                               cache_dir=None):
    points = np.asarray(points_xy, float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('points_xy must be a finite Nx2 array')
    if not (0<sample_resolution<=.05 and 0<support_tolerance<=.01 and
            0<=radial_margin<=.1 and 0<=separation_margin<=.005 and max_support_height>0):
        raise ValueError('invalid geometric support margins or sampling resolution')
    envelope = robot_collision_envelope(robot_urdf_path)
    scene = CollisionSupportScene.from_world(world_path, model_roots, max_support_height=max_support_height)
    source_hashes = {**scene.source_sha256,
        str(Path(robot_urdf_path).resolve()): _sha(robot_urdf_path),
        str(Path(__file__).resolve()): _sha(__file__)}
    settings = dict(max_support_height=max_support_height, support_tolerance=support_tolerance,
        sample_resolution=sample_resolution, radial_margin=radial_margin, separation_margin=separation_margin)
    key_input = {'source_hashes': source_hashes, 'settings': settings, 'point_shape': list(points.shape),
        'point_sha256': hashlib.sha256(np.ascontiguousarray(points, dtype='<f8').tobytes()).hexdigest()}
    cache_key = hashlib.sha256(json.dumps(key_input, sort_keys=True).encode()).hexdigest()
    cache_root = Path(cache_dir) if cache_dir is not None else Path(__file__).parent/'.build/collision_support_cache'
    cache_path = cache_root/(cache_key+'.npz')
    if cache_path.is_file():
        try:
            with np.load(cache_path, allow_pickle=False) as cached:
                mask, heights = cached['mask'], cached['heights']
                metadata = json.loads(str(cached['metadata']))
            if (mask.shape == (len(points),) and heights.shape == (len(points),) and mask.dtype == np.bool_
                    and np.isfinite(heights[mask]).all() and metadata.get('persistent_cache_key') == cache_key
                    and metadata.get('source_hashes') == source_hashes):
                return mask, heights, metadata
        except (OSError, ValueError, KeyError):
            pass  # Recompute if a cache file is incomplete or corrupt.
    mask, heights = np.zeros(len(points), bool), np.full(len(points), np.nan)
    reasons = Counter()
    for index, point in enumerate(points):
        accepted, height, reason = scene.check_position(point, envelope,
            support_tolerance=support_tolerance, sample_resolution=sample_resolution,
            radial_margin=radial_margin, separation_margin=separation_margin)
        mask[index], heights[index] = accepted, height
        reasons[reason] += 1
    metadata = {'method': 'actual static collision support and conservative full-turn swept-volume clearance',
        'scope': 'horizontal stationary placements; unsupported, stepped, sloped, and obstructed poses rejected',
        'source_hashes': source_hashes,
        'robot_envelope': envelope, 'max_support_height': max_support_height,
        'support_tolerance': support_tolerance, 'sample_resolution': sample_resolution,
        'radial_margin': radial_margin, 'separation_margin': separation_margin,
        'candidate_count': len(points), 'accepted_count': int(mask.sum()), 'reason_counts': dict(reasons),
        'accepted_positive_height_count': int(sum(mask & (heights>.05))),
        'yaw_filter': 'none; conservative swept volume permits every yaw and a full turn',
        'ground_truth_used': False, 'trial_outcomes_used': False,
        'cached_surface_sample_count': len(scene.height_cache),
        'persistent_cache_key': cache_key,
        'cache_validation': 'exact source-file hashes, all geometric settings, and ordered input-point bytes'}
    if any(_sha(path) != digest for path, digest in source_hashes.items()):
        raise RuntimeError('collision-support sources changed during filtering; refusing mixed provenance')
    cache_root.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=cache_root, suffix='.npz', delete=False) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, mask=mask, heights=heights, metadata=json.dumps(metadata, sort_keys=True))
        os.replace(temporary, cache_path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return mask, heights, metadata
