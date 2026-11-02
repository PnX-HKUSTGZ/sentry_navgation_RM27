#!/usr/bin/env python3
"""Split the 922 CAD assembly into rigid yaw / roll links (metres, folded zero)."""

import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from import_rm27_robot_cad import (
    EXCLUDED_NAME_FRAGMENTS, PACKAGE_DIR, cad_to_ros, discover_alignment,
    discover_gimbal_alignment, read_binary_stl, roll_rotation, simplify_part,
    write_binary_stl, yaw_rotation,
)


def generate(archive_path, output):
    output.mkdir(parents=True, exist_ok=True)
    old = json.loads((output / 'import_metadata.json').read_text())
    groups, parts = {}, {}
    with zipfile.ZipFile(archive_path) as archive:
        infos = [i for i in archive.infolist() if i.filename.lower().endswith('.stl')]
        alignment = discover_alignment(archive, infos)
        gimbal = discover_gimbal_alignment(archive, infos, alignment)
        rotation = yaw_rotation(np.deg2rad(gimbal['big_yaw_correction_deg']))
        pivot = rotation @ np.asarray(gimbal['small_yaw_pivot_m'])
        pitch_info = next(i for i in infos if i.filename.endswith(
            '小yaw-标准件-深沟球轴承30-37-4 淘宝CSCD店铺-61706-1.STL'))
        pitch_vertices = cad_to_ros(read_binary_stl(archive, pitch_info).reshape(-1, 3, 3),
                                   alignment['cad_center_x_mm'], alignment['cad_center_z_mm'],
                                   alignment['cad_axle_y_mm'])
        pitch_center = (pitch_vertices.min((0, 1)) + pitch_vertices.max((0, 1))) * .5
        pitch_correction = yaw_rotation(np.deg2rad(
            gimbal['small_yaw_correction_deg'] - gimbal['big_yaw_correction_deg']))
        for info in infos:
            name = Path(info.filename).name
            if any(s in name for s in EXCLUDED_NAME_FRAGMENTS) or '全向轮' in name:
                continue
            raw = read_binary_stl(archive, info)
            triangles = cad_to_ros(simplify_part(raw, 2.0),
                                   alignment['cad_center_x_mm'],
                                   alignment['cad_center_z_mm'], alignment['cad_axle_y_mm'])
            if not len(triangles):
                continue
            if ' - 底盘-' in name or ' - 装甲板组合-' in name:
                group = 'chassis_armor' if '装甲' in name else 'chassis_body'
            elif ' - 小YAW-' in name:
                group = 'roll_body'
                # With the roll folded, the gun's pitch axis is vertical.
                # Straighten only its distal assembly; the roll carrier and
                # motor centres must stay attached to their actual shafts.
                center = (triangles.min((0, 1)) + triangles.max((0, 1))) * .5
                if (center[1] < pitch_center[1] - .025 or
                    any(s in name for s in ('相机', '测速', '摩擦轮', '单发限位', '转动弹道'))):
                    triangles = (triangles - pitch_center) @ pitch_correction.T + pitch_center
                triangles = triangles @ rotation.T
            else:
                group = ('yaw_sensors' if '雷达' in name else
                         'yaw_armor' if '装甲' in name else 'yaw_body')
                triangles = triangles @ rotation.T
            groups.setdefault(group, []).append(triangles)
            parts.setdefault(group, []).append(name)

    groups = {key: np.concatenate(value) for key, value in groups.items()}
    # Preserve a 240 mm folded envelope above the wheel contact plane.
    top = max(float(v[:, :, 2].max()) for k, v in groups.items() if k != 'yaw_sensors')
    z_scale = 0.160 / top
    for key in groups:
        if key != 'yaw_sensors':
            groups[key][:, :, 2] *= z_scale
    pivot[2] *= z_scale

    # The sensor assembly has a different CAD placement from the previous ZIP.
    # Align its rigid envelope to the existing calibrated mount, not the old
    # archive's hard-coded source origin. The mount frame remains (0,.126,.130).
    source_rotation = yaw_rotation(np.pi) @ roll_rotation(np.pi / 6)
    sensor = groups['yaw_sensors'] @ (roll_rotation(-np.pi / 6) @ source_rotation.T).T
    old_bounds = old['groups']['sensors']['bounds_m']
    target_center = (np.array(old_bounds['min']) + old_bounds['max']) * 0.5
    sensor += target_center - (sensor.min((0, 1)) + sensor.max((0, 1))) * 0.5
    sensor[:, :, 2] = .130 + (sensor[:, :, 2] - .130) * (.030 / (.168313 - .130))
    groups['yaw_sensors'] = sensor
    groups['roll_body'] -= pivot
    bounds = {}
    for key, triangles in groups.items():
        write_binary_stl(output / (key + '.stl'), triangles, 'RM27 articulated ' + key)
        lo, hi = triangles.min((0, 1)), triangles.max((0, 1))
        bounds[key] = {'min': lo.tolist(), 'max': hi.tolist(),
                       'center': ((lo+hi)*.5).tolist(), 'size': (hi-lo).tolist()}
    head_up = groups['roll_body'] @ roll_rotation(-np.pi / 2).T + pivot
    metadata = {'source_archive': archive_path.name,
                'source_sha256': hashlib.sha256(archive_path.read_bytes()).hexdigest(),
                'alignment': alignment, 'yaw_alignment_deg': gimbal['big_yaw_correction_deg'],
                'roll_origin': pivot.tolist(), 'roll_axis': [-1, 0, 0],
                'folded_height': .240, 'upright_head_top_height': float(head_up[:, :, 2].max()+.080),
                'z_scale': z_scale, 'bounds': bounds, 'parts': parts}
    (output / 'articulated_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: metadata[k] for k in ('roll_origin','roll_axis','folded_height','upright_head_top_height','bounds')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--output', type=Path, default=PACKAGE_DIR / 'meshes/rm27_sentry_visual')
    args = parser.parse_args()
    generate(args.archive, args.output)
