"""Summarize recorded runs without inferring safety necessity from log text."""
import argparse
import csv
import json
from pathlib import Path

try:
    from .metrics import analyze_run
except ImportError:
    from metrics import analyze_run

TERMS = ('ground_unverified', 'headroom', 'static', 'collision', 'block', 'timeout', 'stale')
FIELDS = ('run', 'status', 'duration', 'path_m', 'max_speed', 'stop_count', 'long_count',
          'ramp_count', 'tunnel_count', 'candidate_count', 'raw_stop_count', 'excluded_safety_count', 'protective_stop_count', 'unclassified_count')


def _evidence(result, events):
    for episode in result['episodes']:
        episode['evidence'] = []
        for event in events:
            if not isinstance(event, dict):
                continue
            t = event.get('t')
            if not isinstance(t, (int, float)) or isinstance(t, bool):
                continue
            if not episode['start_time'] - 0.3 <= t <= episode['end_time'] + 0.3:
                continue
            if any(term in json.dumps(event, ensure_ascii=False).lower() for term in TERMS):
                episode['evidence'].append(event)


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def summarize_run(run, scenario=None):
    from math import hypot
    samples = run.get('samples', [])
    main = analyze_run(run, scenario=scenario)
    short = analyze_run(run, config={'stop_min_duration': 0.1}, scenario=scenario)
    for result in (main, short):
        _evidence(result, run.get('events', []))
    valid_times = [s['t'] for s in samples if _number(s.get('t'))]
    speeds = [hypot(s['vx'], s['vy']) for s in samples if _number(s.get('vx')) and _number(s.get('vy'))]
    path = 0.0
    for a, b in zip(samples, samples[1:]):
        if all(_number(s.get(k)) for s in (a, b) for k in ('t', 'x', 'y')) and 0 < b['t'] - a['t'] <= 0.3:
            path += hypot(b['x'] - a['x'], b['y'] - a['y'])
    counted = [e for e in main['episodes'] if not e['excluded_start'] and not e['excluded_goal']]
    candidates = [e for e in counted if e['unexpected_candidate']]
    row = dict(status=run.get('status', 'unknown'), duration=(valid_times[-1] - valid_times[0]) if len(valid_times) > 1 else None,
               path_m=path, max_speed=max(speeds) if speeds else None,
               stop_count=len(candidates), long_count=sum(e['long_stop'] for e in candidates),
               ramp_count=sum(any('ramp' in r for r in e['regions']) for e in candidates),
               tunnel_count=sum(any('tunnel' in r for r in e['regions']) for e in candidates),
               candidate_count=len(candidates), raw_stop_count=main['total_episodes'],
               excluded_safety_count=main['excluded_safety_count'],
               protective_stop_count=sum(bool(e['evidence']) for e in counted),
               unclassified_count=sum(not e['evidence'] and e['safety_status'] == 'unknown' for e in counted))
    return {'summary': row, 'main': main, 'short_stop_sensitivity': short,
            'notes': ['Region labels are approximate snapshots from samples or supplied scenario rectangles.',
                      'Protective evidence is a log association for review, not proof that a stop was necessary.',
                      'Path excludes intervals longer than 0.3 simulation seconds.']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--scenario', type=Path)
    args = parser.parse_args()
    scenario = json.loads(args.scenario.read_text(encoding='utf-8')) if args.scenario else None
    rows = []
    for path in sorted(args.root.rglob('run_??.json')):
        run = json.loads(path.read_text(encoding='utf-8'))
        result = summarize_run(run, scenario)
        result['summary']['run'] = str(path.relative_to(args.root))
        # Per-run names remain distinct when runs share a directory.
        output = path.with_name(path.stem + '_metrics.json')
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        rows.append(result['summary'])
    totals = {k: sum(r[k] for r in rows) for k in ('stop_count', 'long_count', 'ramp_count', 'tunnel_count', 'candidate_count', 'raw_stop_count', 'excluded_safety_count', 'protective_stop_count', 'unclassified_count')}
    summary = {'run_count': len(rows), 'totals': totals, 'runs': rows}
    (args.root / 'summary_metrics.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (args.root / 'summary.csv').open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    main()
