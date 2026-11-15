#!/usr/bin/env python3
"""独立采样地图场景的恢复基准；固定位置/航向/噪声，保留全部失败分母。"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


# 调用独立采样工具，不复用参考云作为查询云。
def load_map_tools():
    path = Path(__file__).resolve().parents[1]/'mapping/prepare_relocalization_map.py'
    spec = importlib.util.spec_from_file_location('map_tools', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# 在固定偏移位置独立生成查询，调用生产恢复库并逐例保存原始结果。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True)
    parser.add_argument('--probe', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--cases', type=int, default=12)
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--with-hint', action='store_true', help='explicit synthetic coarse pose prior, reported separately')
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    tools = load_map_tools()
    bundle = Path(args.bundle).resolve()
    manifest = yaml.safe_load((bundle/'manifest.yaml').read_text())
    points = tools.read_pcd(bundle/'map.pcd')
    rows = []
    for case, index in enumerate(np.linspace(0, len(manifest['views'])-1, args.cases, dtype=int)):
        position = np.array(manifest['views'][index]['pose'][:3])+[.13, -.09, .02]
        yaw = -np.pi+2*np.pi*case/args.cases
        source = tools.visible_cloud(points, position, 20., .37, [-55., 50.], 20000)
        source = source @ Rotation.from_euler('z', yaw).as_matrix()
        rng = np.random.default_rng(120+case)
        source = source[rng.random(len(source)) > .15]
        source += rng.normal(0, .01, source.shape)
        file = out/f'query_{case}.pcd'
        tools.write_pcd(file, source)
        for repetition in range(args.repetitions):
            started = time.monotonic()
            command = [args.probe, str(bundle), str(file)]
            if args.with_hint:
                command += [str(v) for v in [position[0]+.3, position[1]-.2, position[2]+.05, 0., 0., yaw+.15]]
            proc = subprocess.run(command, capture_output=True, text=True, timeout=40)
            row = dict(case=case, repetition=repetition, with_hint=args.with_hint, query_xyz=position.tolist(), query_yaw=yaw,
                       source_points=len(source), elapsed=time.monotonic()-started, exit_code=proc.returncode)
            (out/f'{case}_{repetition}.stdout').write_text(proc.stdout)
            (out/f'{case}_{repetition}.stderr').write_text(proc.stderr)
            if proc.returncode == 0:
                result = json.loads(proc.stdout)
                row.update(result)
                row['position_error_m'] = float(np.linalg.norm(np.array(result['xyz'])-position))
                row['rotation_error_deg'] = float(np.degrees((Rotation.from_euler('z', yaw).inv()*
                                                             Rotation.from_quat(result['quaternion_xyzw'])).magnitude()))
                row['correct'] = result['success'] and row['position_error_m'] < .25 and row['rotation_error_deg'] < 5
                row['false_accept'] = result['success'] and not row['correct']
            rows.append(row)
            (out/'results.json').write_text(json.dumps(rows, indent=2))
    summary = dict(with_hint=args.with_hint, total=len(rows), correct=sum(r.get('correct', False) for r in rows),
                   false_accept=sum(r.get('false_accept', False) for r in rows),
                   rejected=sum(not r.get('success', False) for r in rows),
                   source='PCD-derived independent viewpoint/noise; not real sensor success rate')
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
