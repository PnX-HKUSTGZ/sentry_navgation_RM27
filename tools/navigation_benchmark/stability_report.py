#!/usr/bin/env python3
"""Summarize sampled lateral motion and observed tunnel crossings, retaining failures."""
import argparse
import json
import math
from pathlib import Path

from audit_coverage import crossings
from metrics import analyze_run


def lateral_motion(samples, max_tilt=None):
    """Time-weighted lateral acceleration relative to mean measured velocity.

    Includes deliberate bends: this is a motion variation proxy, not a pure
    oscillation detector. Skip rest, goal approach and gaps. The observer's
    /cmd_vel is body-frame; rotate with sampled truth yaw into world XY as the
    Gazebo drive plugin does (not a timestamp-exact command/pose alignment).
    Optional tilt limit reports level-ground motion separately from ramp
    contact impulses. Reject pairs in place so excluded samples are not bridged.
    """
    energy = command_energy = duration = 0.0
    for a, b in zip(samples, samples[1:]):
        fields = ('t', 'vx', 'vy', 'cmd_vx', 'cmd_vy', 'goal_distance', 'yaw')
        if not all(isinstance(s.get(k), (int, float)) and math.isfinite(s[k])
                   for s in (a, b) for k in fields):
            continue
        if max_tilt is not None and not all(
                isinstance(s.get(k), (int, float)) and math.isfinite(s[k]) and
                abs(s[k]) < max_tilt for s in (a, b) for k in ('pitch', 'roll')):
            continue
        dt = b['t'] - a['t']
        if not (0.01 <= dt <= 0.3 and a.get('active') and b.get('active') and
                min(a['goal_distance'], b['goal_distance']) > 0.35):
            continue
        vx, vy = (a['vx'] + b['vx']) / 2, (a['vy'] + b['vy']) / 2
        speed = math.hypot(vx, vy)
        if speed < 0.15:
            continue
        nx, ny = -vy / speed, vx / speed
        acc = ((b['vx'] - a['vx']) * nx + (b['vy'] - a['vy']) * ny) / dt
        commands = [(math.cos(s['yaw']) * s['cmd_vx'] - math.sin(s['yaw']) * s['cmd_vy'],
                     math.sin(s['yaw']) * s['cmd_vx'] + math.cos(s['yaw']) * s['cmd_vy'])
                    for s in (a, b)]
        cmd_acc = ((commands[1][0] - commands[0][0]) * nx +
                   (commands[1][1] - commands[0][1]) * ny) / dt
        energy += acc * acc * dt
        command_energy += cmd_acc * cmd_acc * dt
        duration += dt
    return {'duration_s': duration,
            'lateral_accel_rms': math.sqrt(energy / duration) if duration else None,
            'command_lateral_rate_rms': math.sqrt(command_energy / duration) if duration else None}


def summarize(run, scenario):
    samples = run.get('samples', [])
    observed = [dict(c, region=r['name']) for r in scenario['regions']
                if 'tunnel' in r['name'] for c in crossings(samples, r)]
    for c in observed:
        inside = [s for s in samples if c['start_time'] <= s['t'] <= c['end_time']]
        c['low_speed_s'] = sum(b['t'] - a['t'] for a, b in zip(inside, inside[1:])
                              if math.hypot(a['vx'], a['vy']) <= 0.2)
    stop = analyze_run(run, scenario=scenario)
    return {'status': run.get('status'),
            'truth_goal_gate_passed': run.get('truth_goal_gate_passed'),
            'truth_goal_distance_m': run.get('final_truth_goal_distance'),
            'duration_s': run['end_sim_time'] - run['start_sim_time'],
            'stops': stop['unexpected_candidate_count'],
            'long_stops': stop['long_stop_count'],
            'tunnel_stops': stop['tunnel_count'],
            'brief_stops': analyze_run(run, {'stop_min_duration': 0.1}, scenario)['unexpected_candidate_count'],
            'lateral_motion': lateral_motion(samples),
            'level_ground_motion': lateral_motion(samples, scenario.get('ramp_pitch_threshold', 0.06)),
            'crossings': [dict(c, duration_s=c['end_time'] - c['start_time']) for c in observed]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    scenario = json.loads((Path(__file__).parent / 'scenario_south.json').read_text())
    result = {}
    for root in args.roots:
        runs = {str(p.relative_to(root)): summarize(json.loads(p.read_text()), scenario)
                for p in sorted(root.glob('repeat_*/run_??.json'))}
        result[str(root)] = runs
        print(root.name, 'segments', len(runs), 'time', round(sum(r['duration_s'] for r in runs.values()), 3),
              'stops', sum(r['stops'] for r in runs.values()),
              'crossings', [round(c['duration_s'], 3) for r in runs.values() for c in r['crossings']])
    args.output.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
