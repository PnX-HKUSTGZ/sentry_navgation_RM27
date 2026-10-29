#!/usr/bin/env python3
"""Summarize startup and navigation trials, retaining failed/incomplete runs."""
import argparse
import json
import math
from pathlib import Path

from stability_report import summarize


def report(root, scenario):
    startup = json.loads((root / 'startup.json').read_text())
    rows = []
    for path in sorted(root.glob('run_[0-9][0-9].json')):
        run = json.loads(path.read_text())
        row = summarize(run, scenario)
        row['file'] = path.name
        row['emergency_stops'] = sum('emergency stop published' in e['msg'].lower()
                                     for e in run.get('events', []))
        row['trajectory_rejections'] = sum('Trajectory safety rejected:' in e['msg']
                                          for e in run.get('events', []))
        for crossing in row['crossings']:
            samples = [s for s in run['samples']
                       if crossing['start_time'] <= s['t'] <= crossing['end_time']]
            # World yaw, not tracking error: the trial may request a nonzero yaw.
            crossing['max_abs_world_yaw_deg'] = max(
                abs(math.degrees(s['yaw'])) for s in samples)
        rows.append(row)
    return {'passed': startup.get('passed', False), 'error': startup.get('error'),
            'pose_matches': startup.get('pose_matches', []),
            'ready_truth_yaw': startup.get('ready_truth_yaw'),
            'goals': startup.get('goals', []), 'runs': rows,
            'total_navigation_s': sum(r['duration_s'] for r in rows),
            'total_emergency_stops': sum(r['emergency_stops'] for r in rows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    scenario = json.loads((Path(__file__).parent / 'scenario_south.json').read_text())
    result = {str(root): report(root, scenario) for root in args.roots}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    for root, item in result.items():
        print(Path(root).name, 'passed=', item['passed'],
              'time=', round(item['total_navigation_s'], 2),
              'emergency=', item['total_emergency_stops'],
              'crossings=', [(round(c['duration_s'], 2), round(c['low_speed_s'], 2))
                            for r in item['runs'] for c in r['crossings']])


if __name__ == '__main__':
    main()
