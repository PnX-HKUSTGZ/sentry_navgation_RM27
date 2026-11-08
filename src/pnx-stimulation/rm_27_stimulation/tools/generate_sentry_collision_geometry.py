#!/usr/bin/env python3
"""从现用 CAD 表面生成分件凸碰撞体，保留洞口净空，不缩小视觉模型。"""

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.spatial.transform import Rotation


PACKAGE = Path(__file__).resolve().parents[1]
VISUAL = PACKAGE / 'meshes/rm27_sentry_visual'
OUTPUT = PACKAGE / 'meshes/rm27_sentry_collision'
PADDING = 0.0005  # 与轴对齐的 0.5 mm 立方体作 Minkowski 和，最大欧氏外扩 sqrt(3)*PADDING。
BULLET_MARGIN = 0.001
NAV_MARGIN = 0.001
PIVOT = np.array([0.005862518, 0.015803345, 0.087848857])
WHEELS = [(1, -1), (-1, 1), (-1, -1), (1, 1)]
# 米，原始 link 局部坐标。底盘 40 mm 以上是细支撑；yaw 的上下板位于
# 约 60 / 118 mm 两侧。切三角面而非只按顶点分组，避免跨层三角面漏包。
LAYER_CUTS = {'chassis_body': (0.040,), 'yaw_body': (0.060, 0.118)}
MAX_COLLISIONS = 64
STL_DTYPE = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
CORNERS = np.array(list(itertools.product((-1., 1.), repeat=3)))


def triangles(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f'需要二进制 STL：{path}')
    return np.frombuffer(data, STL_DTYPE, count, 84)['v'].astype(float)


def vertices(path):
    return np.unique(triangles(path).reshape(-1, 3), axis=0)


def source_triangle_groups():
    raw = {name: triangles(VISUAL / (name + '.stl')) for name in
           ('chassis_body', 'chassis_armor', 'wheels', 'yaw_body', 'yaw_sensors', 'roll_body')}
    groups = {name: raw[name] for name in raw if name != 'wheels'}
    transforms = {}
    for index, (sx, sy) in enumerate(WHEELS, 1):
        mask = np.all((raw['wheels'][:, :, 0] * sx > 0) &
                      (raw['wheels'][:, :, 1] * sy > 0), axis=1)
        pi = 3.1415926  # 与 URDF 固定关节一致。
        rotation = Rotation.from_euler('xyz', [0, pi / 2, (-1 if index < 3 else 1) * pi / 4]).as_matrix()
        center = np.array([sx * .218, sy * .218, 0])
        groups[f'wheel_{index}'] = (raw['wheels'][mask] - center) @ rotation
        transforms[f'wheel_{index}'] = (rotation, center)
    assert sum(len(groups[f'wheel_{i}']) for i in range(1, 5)) == len(raw['wheels'])
    return raw, groups, transforms


def source_groups():
    raw, groups, transforms = source_triangle_groups()
    def unique(values):
        return {k: np.unique(v.reshape(-1, 3), axis=0) for k, v in values.items()}

    return unique(raw), unique(groups), transforms


def clip_z(surface, height, above):
    """封闭半空间内的三角面片；共边交点使用同一运算次序，保留连通性。"""
    inside = surface[:, :, 2] >= height if above else surface[:, :, 2] <= height
    result = list(surface[np.all(inside, axis=1)])
    for triangle in surface[np.any(inside, axis=1) & ~np.all(inside, axis=1)]:
        polygon = []
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            ia = a[2] >= height if above else a[2] <= height
            ib = b[2] >= height if above else b[2] <= height
            if ia:
                polygon.append(a)
            if ia != ib:
                lo, hi = (a, b) if a[2] < b[2] else (b, a)
                point = lo + (hi - lo) * ((height - lo[2]) / (hi[2] - lo[2]))
                point[2] = height
                polygon.append(point)
        for i in range(1, len(polygon) - 1):
            result.append([polygon[0], polygon[i], polygon[i + 1]])
    result = np.asarray(result, dtype=float).reshape(-1, 3, 3)
    area = np.linalg.norm(np.cross(result[:, 1] - result[:, 0], result[:, 2] - result[:, 0]), axis=1)
    return result[area > 0]


def component_points(surface):
    points, faces = np.unique(surface.reshape(-1, 3), axis=0, return_inverse=True)
    faces = faces.reshape(-1, 3)
    graph = coo_matrix((np.ones(faces.size), (faces.ravel(), faces[:, [1, 2, 0]].ravel())),
                       shape=(len(points), len(points)))
    count, labels = connected_components(graph, directed=False)
    return [points[labels == i] for i in range(count)]


def padded_hull(points, envelope=None):
    # 对薄片/开口 CAD 同样有效，不填洞、不扰动源点、不无声丢弃共面分件。
    if len(points) >= 4 and np.linalg.matrix_rank(points - points.mean(axis=0)) == 3:
        points = points[ConvexHull(points).vertices]
    outer = (points[:, None, :] + CORNERS * PADDING).reshape(-1, 3).astype('<f4').astype(float)
    hull = ConvexHull(np.unique(outer, axis=0))
    if envelope is not None and max_plane_violation(hull.points, envelope) > 0:
        planes = np.vstack([hull.equations, envelope.equations])
        outer = HalfspaceIntersection(planes, points.mean(axis=0)).intersections
        hull = ConvexHull(np.unique(outer.astype('<f4').astype(float), axis=0))
    return hull


def decompose(surface, cuts=(), wheel=False):
    # 每块外扩同时受原整组的 0.5 mm 平面外包络约束，不扩大既有导航足迹。
    original = np.unique(surface.reshape(-1, 3), axis=0)
    planes = np.unique(np.round(ConvexHull(original).equations, 12), axis=0)
    planes[:, 3] -= PADDING
    envelope = ConvexHull(HalfspaceIntersection(planes, original.mean(axis=0)).intersections)
    if wheel:
        fragments = [np.unique(surface.reshape(-1, 3), axis=0)]
    else:
        fragments = []
        for low, high in zip((-np.inf, *cuts), (*cuts, np.inf)):
            layer = surface
            if np.isfinite(low):
                layer = clip_z(layer, low, True)
            if np.isfinite(high):
                layer = clip_z(layer, high, False)
            if len(layer):
                fragments.extend(component_points(layer))
    hulls = [padded_hull(points, envelope) for points in fragments]
    owners = list(range(len(hulls)))
    # 只做对单一凸实体的严格包含去冗余；不能用多个不相连凸体覆盖顶点冒充包含。
    for i in sorted(range(len(hulls)), key=lambda i: hulls[i].volume):
        for j in sorted(range(len(hulls)), key=lambda j: -hulls[j].volume):
            if hulls[i].volume < hulls[j].volume and max_plane_violation(
                    hulls[i].points[hulls[i].vertices], hulls[j]) <= 0:
                owners[i] = j
                break
    kept = sorted(set(owners), key=lambda i: tuple(hulls[i].points.min(axis=0)))
    return [(hulls[i], np.vstack([p for p, owner in zip(fragments, owners) if owner == i])) for i in kept]


def write_stl(path, hull):
    faces = hull.points[hull.simplices].copy()
    cross = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    reverse = np.einsum('ij,ij->i', cross, hull.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, [0, 2, 1]]
    normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    assert np.all(lengths > 0), path
    records = np.zeros(len(faces), dtype=STL_DTYPE)
    records['normal'], records['v'] = normals / lengths[:, None], faces
    path.write_bytes(b'RM27 CAD component convex hull'.ljust(80, b' ') +
                     struct.pack('<I', len(faces)) + records.tobytes())


def max_plane_violation(points, hull):
    return max(float(np.max(batch @ hull.equations[:, :-1].T + hull.equations[:, -1]))
               for batch in np.array_split(points, max(1, (len(points) + 2047) // 2048)))


def roll_swept_radius(points):
    """解析覆盖翻转 0..90°，再绕 yaw 轴整圈旋转，无离散角度漏包。"""
    x, y, z = points.T
    candidates = [y, z]
    angle = np.arctan2(z, y)
    for a in (angle, angle + np.pi, angle - np.pi):
        valid = (a >= 0) & (a <= np.pi / 2)
        candidates.append(np.where(valid, y * np.cos(a) + z * np.sin(a), y))
    largest_y = np.max(np.abs(np.array(candidates) + PIVOT[1]), axis=0)
    return float(np.max(np.hypot(x + PIVOT[0], largest_y)))


def footprint(meshes, transforms):
    fixed = [meshes['chassis_body'], meshes['chassis_armor']]
    for name, (rotation, center) in transforms.items():
        fixed.append(meshes[name] @ rotation.T + center)
    fixed = np.vstack(fixed)[:, :2]
    moving_radius = max(float(np.linalg.norm(meshes[name][:, :2], axis=1).max())
                        for name in ('yaw_body', 'yaw_sensors'))
    moving_radius = max(moving_radius, roll_swept_radius(meshes['roll_body']))
    angles = np.arange(48) * 2 * np.pi / 48
    normals = np.column_stack([np.cos(angles), np.sin(angles)])
    distances = np.maximum((fixed @ normals.T).max(axis=0), moving_radius) + BULLET_MARGIN + NAV_MARGIN
    intersections = HalfspaceIntersection(np.column_stack([normals, -distances]), np.zeros(2)).intersections
    polygon = np.round(intersections[ConvexHull(intersections).vertices], 8)
    return polygon, moving_radius


def collision_xacro(parts):
    ns = 'http://www.ros.org/wiki/xacro'
    ET.register_namespace('xacro', ns)
    root = ET.Element('robot')
    cache = ET.SubElement(root, f'{{{ns}}}macro', name='cad_collision_cache_files', params='')
    for entries in parts.values():
        for entry in entries:
            ET.SubElement(cache, 'mesh_file').text = entry['file']
    for group, entries in parts.items():
        macro = ET.SubElement(root, f'{{{ns}}}macro', name=f'cad_{group}_collisions',
                              params='offset_x:=0.0 offset_y:=0.0 offset_z:=0.0')
        for entry in entries:
            ET.SubElement(macro, f'{{{ns}}}cad_collision', mesh=Path(entry['file']).stem,
                          offset_x='${offset_x}', offset_y='${offset_y}', offset_z='${offset_z}')
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode') + '\n'


def generate(check=False, output=OUTPUT):
    raw, groups, transforms = source_triangle_groups()
    report = {'schema_version': 2, 'padding_kind': 'axis_aligned_cube_minkowski_sum',
              'padding_reference': 'unpadded convex hull of each clipped connected surface',
              'plane_padding_m': PADDING, 'max_padding_distance_m': float(np.sqrt(3) * PADDING),
              'bullet_margin_m': BULLET_MARGIN, 'navigation_margin_m': NAV_MARGIN,
              'layer_cuts_m': LAYER_CUTS, 'max_collisions': MAX_COLLISIONS,
              'parts': {}, 'source_sha256': {}}
    meshes = {}
    # 先在临时目录完成生成与验证；失败不会覆盖已有资产。
    with tempfile.TemporaryDirectory(prefix='rm27_collision_') as temporary:
        stage = Path(temporary)
        for name, surface in groups.items():
            entries, loaded = [], []
            for index, (hull, covered) in enumerate(decompose(
                    surface, LAYER_CUTS.get(name, ()), name.startswith('wheel_'))):
                filename = f'{name}_{index:03}.stl'
                path = stage / filename
                write_stl(path, hull)
                points = vertices(path)
                saved_hull = ConvexHull(points)
                violation = max_plane_violation(covered, saved_hull)
                assert violation <= -PADDING + 2e-7, (name, index, violation)
                loaded.append(points)
                entries.append({'file': filename, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                'covered_fragment_vertices': len(covered), 'collision_vertices': len(points),
                                'collision_volume_m3': saved_hull.volume,
                                'max_signed_outside_m': violation,
                                'bounds_m': [points.min(axis=0).tolist(), points.max(axis=0).tolist()]})
            report['parts'][name] = entries
            meshes[name] = np.vstack(loaded)
        report['collision_count'] = sum(len(entries) for entries in report['parts'].values())
        assert report['collision_count'] <= MAX_COLLISIONS, report['collision_count']
        for name in raw:
            report['source_sha256'][name] = hashlib.sha256((VISUAL / (name + '.stl')).read_bytes()).hexdigest()
        polygon, radius = footprint(meshes, transforms)
        report['footprint_points'] = polygon.tolist()
        report['moving_swept_radius_m'] = radius
        report['footprint_size_m'] = np.ptp(polygon, axis=0).tolist()
        report['coverage'] = ('按高度精确裁切源三角面，再按连通表面分件；每块裁切三角面的全部顶点均在单一保存后凸体内，'
                              '由凸性覆盖该完整面片。不宣称完整非凸 CAD 体积误差已获界定。轮组保持单凸体滚动外轮廓。')
        (stage / 'parts.xacro').write_text(collision_xacro(report['parts']))
        (stage / 'geometry.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        expected = {entry['file'] for entries in report['parts'].values() for entry in entries}
        if check:
            assert {path.name for path in output.glob('*.stl')} == expected, '碰撞目录存在缺失或多余 STL'
            for path in stage.iterdir():
                assert (output / path.name).read_bytes() == path.read_bytes(), f'碰撞资源过期或损坏：{path.name}'
        else:
            output.mkdir(parents=True, exist_ok=True)
            for path in stage.iterdir():
                (output / path.name).write_bytes(path.read_bytes())
            # 仅清除本生成器管理的旧 STL，避免留下可被误接线的整组凸包。
            for path in output.glob('*.stl'):
                if path.name not in expected:
                    path.unlink()
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只校验已生成资源，不修改文件')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    args = parser.parse_args()
    generate(args.check, args.output)
