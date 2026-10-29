#!/usr/bin/env python3
"""Compare recorded global geometry and motion; retain failed runs in the report."""
import argparse
import json
import math
from pathlib import Path

from metrics import analyze_run


def geometry(points):
    edges = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
    edges = [(x, y, math.hypot(x, y)) for x, y in edges if math.hypot(x, y) > 1e-6]
    length = sum(e[2] for e in edges)
    angles = [math.acos(max(-1.0, min(1.0, (a[0]*b[0]+a[1]*b[1])/(a[2]*b[2]))))
              for a, b in zip(edges, edges[1:])]
    energy = sum(angle**2 / (0.5*(a[2]+b[2]))
                 for angle, a, b in zip(angles, edges, edges[1:]))
    return {'length_m': length, 'max_turn_deg': max(angles, default=0.0)*180/math.pi,
            'bending_per_m': energy/length if length else 0.0, 'points': len(points)}


def summarize(run):
    paths = [p for p in run.get('global_paths', []) if len(p['points']) >= 2 and
             math.dist(p['points'][-1], run['goal']) < 0.35]
    speeds = []
    for a, b in zip(run['samples'], run['samples'][1:]):
        dt = b['t']-a['t']
        if 0.01 <= dt <= 0.3 and a.get('active') and b.get('active'):
            speeds.append((abs(math.hypot(b['vx'], b['vy']) - math.hypot(a['vx'], a['vy']))/dt, dt))
    duration = sum(dt for _, dt in speeds)
    refs = [r for r in run.get('reference_dynamics', []) if r['flag'] == 1]
    result = {'status': run['status'], 'goal': run['goal'],
              'duration_s': run['end_sim_time']-run['start_sim_time'],
              'truth_goal_distance_m': run.get('final_truth_goal_distance'),
              'first_global_path': geometry(paths[0]['points']) if paths else None,
              'global_path_updates': len(paths),
              'speed_rate_rms_m_s2': math.sqrt(sum(v*v*dt for v, dt in speeds)/duration) if duration else None,
              'reference_jerk_peak_m_s3': max((r['max_jerk'] for r in refs), default=None),
              'reference_acceleration_peak_m_s2': max((r['max_acceleration'] for r in refs), default=None),
              'stop_metrics': analyze_run(run)}
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('roots', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    results = {}
    for root in args.roots:
        results[str(root)] = {str(path.relative_to(root)): summarize(json.loads(path.read_text()))
                             for path in sorted(root.glob('repeat_*/run_*.json'))
                             if not path.name.endswith('_metrics.json')}
    args.output.write_text(json.dumps(results, indent=2) + '\n')
    for root, runs in results.items():
        print(root)
        for name, run in runs.items():
            print(name, run['status'], round(run['duration_s'], 2), run['first_global_path'])


if __name__ == '__main__':
    main()
