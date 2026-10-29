"""Compare repeat directories; incomplete or currently running batches remain partial."""
import argparse
import json
import math
from pathlib import Path

try:
    from .summarize import summarize_run
except ImportError:
    from summarize import summarize_run


def read(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def visited(samples, scenario):
    names = set()
    for s in samples:
        if s.get('region'):
            names.add(str(s['region']).lower())
        x, y = s.get('x'), s.get('y')
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            for r in scenario.get('regions', []):
                if r['xmin'] <= x <= r['xmax'] and r['ymin'] <= y <= r['ymax']:
                    names.add(r['name'].lower())
        pitch, roll = s.get('pitch'), s.get('roll')
        threshold = scenario.get('ramp_pitch_threshold')
        if isinstance(threshold, (int, float)) and (isinstance(pitch, (int, float)) or isinstance(roll, (int, float))):
            if math.hypot(pitch if isinstance(pitch, (int, float)) else 0, roll if isinstance(roll, (int, float)) else 0) >= threshold:
                names.add('ramp')
    return names


def compare_group(directory, scenario):
    repeats = sorted(p for p in directory.glob('repeat_*') if p.is_dir())
    out = dict(attempted_repeats=len(repeats), startup_failures=0, complete_routes=0,
               attempted_legs=0, succeeded_legs=0, candidate_stops=0, long_stops=0,
               ramp_stops=0, tunnel_stops=0, unknown_stops=0, excluded_safety=0,
               excluded_start=0, excluded_goal=0, short_candidate_stops=0,
               complete_route_duration_s=[], complete_route_path_m=[], region_visited_legs={},
               profiles=[], binary_sha256_set={}, runtime=[], repeats=[], partial=False)
    for repeat in repeats:
        manifest, runtime = read(repeat / 'manifest.json'), read(repeat / 'runtime.json')
        profile = {'repeat': repeat.name, 'params_source': manifest.get('params_source'),
                   'params_sha256': manifest.get('params_sha256')}
        out['profiles'].append(profile)
        for name, digest in manifest.get('file_sha256', {}).items():
            if name.startswith('install/'):
                out['binary_sha256_set'].setdefault(name, [])
                if digest not in out['binary_sha256_set'][name]: out['binary_sha256_set'][name].append(digest)
        nodes = runtime.get('nodes', {})
        planner = nodes.get('planner_server', {}).get('parameters', {})
        controller = nodes.get('controller_server', {}).get('parameters', {})
        out['runtime'].append({'repeat': repeat.name, 'mode': runtime.get('mode_label'),
                               'mode_verified': runtime.get('mode_label_is_verified') is True,
                               'mode_verification': runtime.get('mode_verification', 'unverified'),
                               'minco_planner': planner, 'mpc_controller': controller,
                               'raw': runtime or None})
        runs = []
        for path in sorted(repeat.glob('run_??.json')):
            run = read(path)
            if not run.get('samples'):
                continue
            runs.append((run, summarize_run(run, scenario)))
        status = manifest.get('status', 'unverified')
        # No observations in a running repeat is partial, not yet a startup failure.
        terminal = status in ('completed', 'failed', 'interrupted', 'success', 'succeeded')
        if not runs and terminal: out['startup_failures'] += 1
        if not terminal: out['partial'] = True
        expected = len(scenario.get('goals', [])) or 5
        complete = len(runs) == expected and all(r.get('status') == 'succeeded' for r, _ in runs)
        if complete:
            out['complete_routes'] += 1
            out['complete_route_duration_s'].append(sum(m['summary']['duration'] or 0 for _, m in runs))
            out['complete_route_path_m'].append(sum(m['summary']['path_m'] for _, m in runs))
        out['repeats'].append({'repeat': repeat.name, 'manifest_status': status, 'complete_route': complete,
                               'sampled_legs': len(runs), 'partial': not terminal})
        for run, result in runs:
            out['attempted_legs'] += 1
            out['succeeded_legs'] += run.get('status') == 'succeeded'
            row, main = result['summary'], result['main']
            for dest, source in [('candidate_stops', 'candidate_count'), ('long_stops', 'long_count'),
                                 ('ramp_stops', 'ramp_count'), ('tunnel_stops', 'tunnel_count')]:
                out[dest] += row[source]
            out['unknown_stops'] += main['unexpected_candidate_count']
            out['excluded_safety'] += main['excluded_safety_count']
            out['excluded_start'] += main['excluded_start_count']
            out['excluded_goal'] += main['excluded_goal_count']
            out['short_candidate_stops'] += result['short_stop_sensitivity']['unexpected_candidate_count']
            for name in visited(run['samples'], scenario):
                out['region_visited_legs'][name] = out['region_visited_legs'].get(name, 0) + 1
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--groups', nargs='+', required=True)
    parser.add_argument('--scenario', type=Path, required=True)
    args = parser.parse_args()
    scenario = json.loads(args.scenario.read_text(encoding='utf-8'))
    result = {'notes': ['Partial repeats may still be running; missing observations are not final failures.',
                        'Region exposure is approximate and counts sampled legs.',
                        'Complete-route duration sums observed leg durations; path excludes sample gaps over 0.3s.'],
              'groups': {name: compare_group(args.root / name, scenario) for name in args.groups}}
    output = args.root / 'comparison.json'
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
