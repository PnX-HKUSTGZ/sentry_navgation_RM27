#!/usr/bin/env python3
"""从现有 PCD 和导航栅格生成虚拟关键帧地图；不运行 SLAM，不覆盖已有产物。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml


# 以内容哈希绑定输入与地图产物。
def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


# 读取常见 ASCII/binary PCD，保留 XYZ；压缩格式明确报错，不静默误读。
def read_pcd(path):
    with Path(path).open('rb') as f:
        header = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError('PCD missing DATA')
            parts = line.decode('ascii').strip().split()
            if parts and not parts[0].startswith('#'):
                header[parts[0]] = parts[1:]
            if parts and parts[0] == 'DATA':
                break
        names = header['FIELDS']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        if any(counts[names.index(k)] != 1 for k in ('x', 'y', 'z')):
            raise ValueError('XYZ must be scalar')
        if header['DATA'] == ['ascii']:
            data = np.loadtxt(f, ndmin=2)
            offsets = np.cumsum([0] + counts[:-1])
            xyz = data[:, [offsets[names.index(k)] for k in ('x', 'y', 'z')]]
        elif header['DATA'] == ['binary']:
            fields = [(n, '<' + {'F': 'f', 'I': 'i', 'U': 'u'}[t] + str(s), (c,))
                      for n, s, t, c in zip(names, sizes, types, counts)]
            data = np.frombuffer(f.read(), dtype=np.dtype(fields))
            xyz = np.column_stack([data[k][:, 0] for k in ('x', 'y', 'z')])
        else:
            raise ValueError('Convert binary_compressed PCD to binary with PCL first')
    if len(xyz) != int(header['POINTS'][0]):
        raise ValueError('PCD point count mismatch')
    return np.asarray(xyz[np.isfinite(xyz).all(axis=1)], dtype=np.float64)


# 输出兼容 PCL 的二进制 XYZI，所有坐标单位为米。
def write_pcd(path, xyz):
    points = np.column_stack([xyz, np.zeros(len(xyz))]).astype('<f4')
    header = (f'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    with Path(path).open('wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.tobytes())


# 每个角度单元保留最近返回，近似处理完整 PCD 中的遮挡；不声称等价于网格射线追踪。
def visible_cloud(points, origin, radius, angular_deg, elevation, max_points):
    local = points - origin
    distance = np.linalg.norm(local, axis=1)
    elevation_rad = np.arctan2(local[:, 2], np.linalg.norm(local[:, :2], axis=1))
    mask = (distance > .3) & (distance < radius)
    mask &= (elevation_rad >= np.deg2rad(elevation[0])) & (elevation_rad <= np.deg2rad(elevation[1]))
    local, distance, elevation_rad = local[mask], distance[mask], elevation_rad[mask]
    angular = np.deg2rad(angular_deg)
    azimuth = np.arctan2(local[:, 1], local[:, 0])
    bins = np.column_stack([np.floor((azimuth + np.pi) / angular),
                            np.floor((elevation_rad + np.pi / 2) / angular)]).astype(np.int64)
    order = np.argsort(distance, kind='stable')
    _, first = np.unique(bins[order], axis=0, return_index=True)
    result = local[order[first]]
    if len(result) > max_points:
        result = result[np.linspace(0, len(result)-1, max_points, dtype=int)]
    return result


# 独立栅格投影：图像行向下，地图 y 向上，支持 origin 中的 yaw。
def world_to_pixel(xy, origin, resolution, shape):
    rot = Rotation.from_euler('z', origin[2]).as_matrix()[:2, :2]
    local = (xy - np.asarray(origin[:2])) @ rot
    col = np.floor(local[:, 0] / resolution).astype(int)
    row = shape[0] - 1 - np.floor(local[:, 1] / resolution).astype(int)
    valid = (row >= 0) & (row < shape[0]) & (col >= 0) & (col < shape[1])
    return row, col, valid


# 独立控制点验证显式 map←PCD 变换；至少三个非共线点才能确认平面坐标约定。
def check_alignment_pairs(path, transform, tolerance):
    pairs = np.loadtxt(path, delimiter=',', ndmin=2)
    if pairs.shape[1] != 6 or len(pairs) < 3 or not np.isfinite(pairs).all():
        raise ValueError('alignment pairs require at least three finite pcd_xyz,map_xyz rows')
    if np.linalg.matrix_rank(pairs[:, :2] - pairs[:, :2].mean(axis=0)) < 2:
        raise ValueError('alignment control points must not be collinear in XY')
    mapped = pairs[:, :3] @ Rotation.from_euler('xyz', transform[3:]).as_matrix().T + transform[:3]
    error = np.linalg.norm(mapped - pairs[:, 3:], axis=1)
    if not np.isfinite(tolerance) or tolerance <= 0 or error.max() > tolerance:
        raise ValueError(f'alignment control points failed: max error={error.max():.6f} m')
    return dict(verified=True, points=len(pairs), max_error_m=float(error.max()),
                rmse_m=float(np.sqrt(np.mean(error**2))), tolerance_m=tolerance, sha256=digest(path))


# 构造地图包并原子发布，清单保留坐标变换、输入哈希和全部生成参数。
def prepare(args):
    output = Path(args.output).resolve()
    if output.exists():
        raise ValueError(f'refusing to overwrite {output}')
    if args.spacing <= 0 or args.voxel <= 0 or args.height <= 0 or args.range <= 0 or args.angular_deg <= 0:
        raise ValueError('spacing, voxel, height, range and angular resolution must be positive')
    pcd, grid = Path(args.pcd).resolve(), Path(args.grid).resolve()
    meta = yaml.safe_load(grid.read_text())
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError('only trinary navigation grids are supported')
    image_path = (grid.parent / meta['image']).resolve()
    image = np.asarray(Image.open(image_path).convert('L'))
    probability = image / 255. if meta.get('negate', 0) else 1. - image / 255.
    free = probability < meta['free_thresh']
    clearance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1] * meta['resolution']
    xyz = read_pcd(pcd)
    alignment = np.asarray(args.map_from_pcd)
    if not np.isfinite(alignment).all():
        raise ValueError('nonfinite map_from_pcd')
    controls = check_alignment_pairs(args.alignment_pairs, alignment, args.alignment_tolerance) if args.alignment_pairs else None
    xyz = xyz @ Rotation.from_euler('xyz', alignment[3:]).as_matrix().T + alignment[:3]
    _, representatives = np.unique(np.floor(xyz / args.voxel).astype(np.int64), axis=0, return_index=True)
    xyz = xyz[representatives]
    if len(xyz) > 10000000 or len(xyz) < 100:
        raise ValueError('map point budget exceeded or empty')
    tree = cKDTree(xyz[:, :2])
    stride = max(1, int(round(args.spacing / meta['resolution'])))
    rot = Rotation.from_euler('z', meta['origin'][2]).as_matrix()[:2, :2]
    views = []
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=output.name+'.tmp-', dir=output.parent))
    try:
        (temporary/'clouds').mkdir()
        write_pcd(temporary/'map.pcd', xyz)
        for row in range(stride//2, image.shape[0], stride):
            for col in range(stride//2, image.shape[1], stride):
                if clearance[row, col] < args.clearance:
                    continue
                xy = np.array([(col+.5)*meta['resolution'], (image.shape[0]-row-.5)*meta['resolution']]) @ rot.T + meta['origin'][:2]
                neighbors = tree.query_ball_point(xy, .3)
                if len(neighbors) < 3:
                    continue
                floor = float(np.quantile(xyz[neighbors, 2], .1))
                origin = np.array([*xy, floor+args.height])
                cloud = visible_cloud(xyz, origin, args.range, args.angular_deg, args.elevation, args.max_points)
                if len(cloud) < 100:
                    continue
                if len(views) >= args.max_views:
                    raise ValueError('view budget exceeded: increase spacing')
                path = f'clouds/{len(views):05d}.pcd'
                write_pcd(temporary/path, cloud)
                views.append({'pose': [*map(float, origin), 0., 0., 0.], 'cloud': path,
                              'sha256': digest(temporary/path)})
        if not views:
            raise ValueError('no supported free-space views: inspect grid/PCD alignment and clearance')
        rows, cols, inside = world_to_pixel(xyz[:, :2], meta['origin'], meta['resolution'], image.shape)
        overlay = np.repeat(image[:, :, None], 3, axis=2)
        overlay[rows[inside], cols[inside]] = [220, 70, 70]
        vr, vc, valid = world_to_pixel(np.asarray([v['pose'][:2] for v in views]), meta['origin'], meta['resolution'], image.shape)
        overlay[vr[valid], vc[valid]] = [0, 70, 255]
        Image.fromarray(overlay).save(temporary/'alignment.png')
        free_rows, free_cols = np.where(clearance >= args.clearance)
        free_xy = np.column_stack([(free_cols+.5)*meta['resolution'],
                                   (image.shape[0]-free_rows-.5)*meta['resolution']]) @ rot.T + meta['origin'][:2]
        reference_xy = np.asarray([v['pose'][:2] for v in views])
        distances = cKDTree(reference_xy).query(free_xy)[0]
        coverage = dict(p95_m=float(np.percentile(distances, 95)), p99_m=float(np.percentile(distances, 99)),
                        max_m=float(np.max(distances)), free_cells=len(free_rows))
        manifest = {'format': 'rm27_reference_map_v1', 'frame': 'map', 'units': 'm',
                    'alignment_controls': controls, 'coverage': coverage, 'map_from_pcd': alignment.tolist(), 'map': 'map.pcd', 'views': views,
                    'source_pcd_sha256': digest(pcd), 'grid_yaml_sha256': digest(grid),
                    'grid_image_sha256': digest(image_path), 'map_sha256': digest(temporary/'map.pcd'),
                    'grid': meta, 'parameters': vars(args),
                    'descriptor_policy': 'runtime ScanContextDatabase from visibility-filtered reference clouds'}
        manifest['version'] = hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()
        (temporary/'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))
        report = {'coverage': coverage, 'views': len(views), 'points': len(xyz), 'pcd_xy_inside_grid_fraction': float(np.mean(inside)),
                  'coordinate_contract': 'single explicit map_from_pcd; PGM origin/resolution/yaw preserved',
                  'alignment_verified': controls is not None, 'alignment_controls': controls,
                  'review_required': 'Inspect alignment.png and surveyed landmarks; bounds alone cannot prove physical alignment',
                  'visibility': 'angular nearest return approximation; not mesh ray tracing', 'version': manifest['version']}
        (temporary/'alignment_report.json').write_text(json.dumps(report, indent=2))
        temporary.rename(output)
        print(json.dumps(report, indent=2))
    except BaseException:
        shutil.rmtree(temporary)
        raise


# 命令行入口；输入均为已有地图，不要求额外在线建图。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--grid', required=True, help='navigation PGM YAML')
    parser.add_argument('--output', required=True)
    parser.add_argument('--map-from-pcd', type=float, nargs=6, default=[0.]*6)
    parser.add_argument('--alignment-pairs', help='CSV of independently measured pcd_x,y,z,map_x,y,z; no header')
    parser.add_argument('--alignment-tolerance', type=float, default=.03)
    parser.add_argument('--spacing', type=float, default=.75)
    parser.add_argument('--height', type=float, default=.35, help='IMU reference height above local support, metres')
    parser.add_argument('--clearance', type=float, default=.25)
    parser.add_argument('--range', type=float, default=25.)
    parser.add_argument('--angular-deg', type=float, default=.5)
    parser.add_argument('--elevation', type=float, nargs=2, default=[-60., 60.])
    parser.add_argument('--voxel', type=float, default=.1)
    parser.add_argument('--max-points', type=int, default=30000)
    parser.add_argument('--max-views', type=int, default=3000)
    prepare(parser.parse_args())


if __name__ == '__main__':
    main()
