#!/usr/bin/env python3
"""从现用可视 STL 生成外包凸包；不抽样、不向内简化车体。"""

import argparse
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from scipy.spatial.transform import Rotation


PACKAGE = Path(__file__).resolve().parents[1]
VISUAL = PACKAGE / 'meshes/rm27_sentry_visual'
OUTPUT = PACKAGE / 'meshes/rm27_sentry_collision'
PADDING = 0.0005  # 各凸包平面向外留 0.5 毫米，抵消 STL 浮点舍入。
BULLET_MARGIN = 0.001  # gz-physics7 凸体接触余量，导航包络必须包含。
NAV_MARGIN = 0.001
PIVOT = np.array([0.005862518, 0.015803345, 0.087848857])
WHEELS = [(1, -1), (-1, 1), (-1, -1), (1, 1)]


def vertices(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f'需要二进制 STL：{path}')
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.unique(np.frombuffer(data, dtype, count, 84)['v'].reshape(-1, 3), axis=0).astype(float)


def source_groups():
    raw = {name: vertices(VISUAL / (name + '.stl')) for name in
           ('chassis_body', 'chassis_armor', 'wheels', 'yaw_body', 'yaw_sensors', 'roll_body')}
    # 底盘和装甲分开，避免把两者之间的空隙也包成实体。
    groups = {name: raw[name] for name in
              ('chassis_body', 'chassis_armor', 'yaw_body', 'yaw_sensors', 'roll_body')}
    transforms = {}
    for index, (sx, sy) in enumerate(WHEELS, 1):
        points = raw['wheels'][(raw['wheels'][:, 0] * sx > 0) & (raw['wheels'][:, 1] * sy > 0)]
        # 与 URDF 的固定轮关节使用同一个近似 PI，避免引入外参偏差。
        pi = 3.1415926
        rotation = Rotation.from_euler('xyz', [0, pi / 2, (-1 if index < 3 else 1) * pi / 4]).as_matrix()
        center = np.array([sx * .218, sy * .218, 0])
        groups[f'wheel_{index}'] = (points - center) @ rotation
        transforms[f'wheel_{index}'] = (rotation, center)
    assert sum(len(groups[f'wheel_{i}']) for i in range(1, 5)) == len(raw['wheels'])
    return raw, groups, transforms


def padded_hull(points):
    hull = ConvexHull(points)
    planes = np.unique(np.round(hull.equations, 12), axis=0)
    planes[:, 3] -= PADDING
    outer = HalfspaceIntersection(planes, points[hull.vertices].mean(axis=0)).intersections
    # 在 STL 的实际精度下重新建凸包，避免共面交点舍入后形成零面积三角形。
    outer = np.unique(outer.astype('<f4'), axis=0).astype(float)
    hull = ConvexHull(outer)
    triangles = outer[hull.simplices].copy()
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    reverse = np.einsum('ij,ij->i', cross, hull.equations[:, :3]) < 0
    triangles[reverse] = triangles[reverse][:, [0, 2, 1]]
    return triangles.astype('<f4')


def write_stl(path, triangles):
    precise = triangles.astype(float)
    normals = np.cross(precise[:, 1] - precise[:, 0], precise[:, 2] - precise[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    assert np.all(lengths > 0), path
    normals /= lengths[:, None]
    records = np.zeros(len(triangles), dtype=[('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    records['normal'], records['v'] = normals, triangles
    path.write_bytes(b'RM27 CAD outer convex hull'.ljust(80, b' ') + struct.pack('<I', len(triangles)) + records.tobytes())


def max_plane_violation(points, hull):
    return max(float(np.max(batch @ hull.equations[:, :-1].T + hull.equations[:, -1]))
               for batch in np.array_split(points, max(1, (len(points) + 2047) // 2048)))


def roll_swept_radius(points):
    """解析覆盖翻转 0..90°，再绕 yaw 轴整圈旋转，无离散角度漏包。"""
    # y(a) = pivot_y + y*cos(a) + z*sin(a)，极值只可能在端点或导数零点。
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
    planes = np.column_stack([normals, -distances])
    intersections = HalfspaceIntersection(planes, np.zeros(2)).intersections
    polygon = intersections[ConvexHull(intersections).vertices]
    polygon = np.round(polygon, 8)
    hull = ConvexHull(polygon)
    assert max_plane_violation(fixed, hull) <= -BULLET_MARGIN - NAV_MARGIN + 1e-8
    assert np.all(-hull.equations[:, 2] >= moving_radius + BULLET_MARGIN + NAV_MARGIN - 1e-8)
    return polygon, moving_radius


def generate(check=False):
    raw, groups, transforms = source_groups()
    if not check:
        OUTPUT.mkdir(exist_ok=True)
        for name, points in groups.items():
            write_stl(OUTPUT / (name + '.stl'), padded_hull(points))
    report = {'plane_padding_m': PADDING, 'bullet_margin_m': BULLET_MARGIN,
              'navigation_margin_m': NAV_MARGIN, 'parts': {}, 'source_sha256': {}}
    meshes = {}
    for name, points in groups.items():
        mesh = vertices(OUTPUT / (name + '.stl'))
        hull = ConvexHull(mesh)
        violation = max_plane_violation(points, hull)
        assert violation <= -PADDING + 2e-7, (name, violation)
        meshes[name] = mesh
        report['parts'][name] = {'cad_vertices_checked': len(points), 'collision_vertices': len(mesh),
                                'collision_volume_m3': hull.volume, 'max_signed_outside_m': violation,
                                'bounds_m': [mesh.min(axis=0).tolist(), mesh.max(axis=0).tolist()]}
    for name in raw:
        report['source_sha256'][name] = hashlib.sha256((VISUAL / (name + '.stl')).read_bytes()).hexdigest()
    polygon, radius = footprint(meshes, transforms)
    report['footprint_points'] = polygon.tolist()
    report['moving_swept_radius_m'] = radius
    report['footprint_size_m'] = np.ptp(polygon, axis=0).tolist()
    report['coverage'] = '所有原始可视 STL 顶点均在对应凸包内；同刚体变换保持包含关系。足迹解析覆盖 yaw 整圈和 roll 0..90°。'
    if check:
        saved = json.loads((OUTPUT / 'geometry.json').read_text())
        assert saved['source_sha256'] == report['source_sha256'], '可视模型已变化，请重新生成碰撞体。'
        assert np.allclose(saved['footprint_points'], polygon, atol=1e-9, rtol=0)
        for name, part in report['parts'].items():
            assert part['cad_vertices_checked'] == saved['parts'][name]['cad_vertices_checked']
            assert np.allclose(part['bounds_m'], saved['parts'][name]['bounds_m'], atol=1e-9, rtol=0)
    else:
        (OUTPUT / 'geometry.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只校验已生成资源，不修改文件')
    generate(parser.parse_args().check)
