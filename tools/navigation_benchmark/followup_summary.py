"""Create a reproducible matrix from follow-up repeat artifacts."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

from summarize import summarize_run

HERE = Path(__file__).resolve().parent
TERMINAL = {'completed', 'failed', 'interrupted', 'success', 'succeeded'}
PATTERNS = {
    'odometry_rx_stale': 'ODOMETRY_RX_STALE',
    'odometry_stamp_stale': 'ODOMETRY_STAMP_STALE',
    'sync_tolerance': 'SYNC_TOLERANCE',
    'non_horizontal_tf': 'non-horizontal',
}


def read(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def counts(messages):
    result = {name: 0 for name in PATTERNS}
    for message in messages:
        for name, token in PATTERNS.items():
            if token in message:
                result[name] += 1
    return result


def build_repeat(repeat, scenario):
    manifest = read(repeat / 'manifest.json')
    runtime = read(repeat / 'runtime.json')
    status = manifest.get('status', 'missing')
    partial = status not in TERMINAL
    rows = []
    action_messages = []
    for path in sorted(repeat.glob('run_??.json')):
        run = read(path)
        if not run:
            rows.append({'file': path.name, 'unreadable_or_empty': True})
            continue
        result = summarize_run(run, scenario) if run.get('samples') else None
        sample = run.get('samples', [])[-1:] or [{}]
        last_distance = sample[0].get('goal_distance')
        if not isinstance(last_distance, (int, float)) or not math.isfinite(last_distance):
            last_distance = None
        for event in run.get('events', []):
            timestamp = event.get('t')
            if isinstance(timestamp, (int, float)) and isinstance(run.get('start_sim_time'), (int, float)) and isinstance(run.get('end_sim_time'), (int, float)) and run['start_sim_time'] <= timestamp <= run['end_sim_time']:
                action_messages.append(str(event.get('msg', '')))
        summary = result['summary'] if result else None
        rows.append({
            'file': path.name, 'status': run.get('status'), 'goal': run.get('goal'),
            'has_samples': bool(run.get('samples')), 'metrics': summary,
            'short_0_1s_candidates': result['short_stop_sensitivity']['unexpected_candidate_count'] if result else None,
            'recorded_truth_goal_distance_m': run.get('final_truth_goal_distance'),
            'recorded_truth_goal_gate_passed': run.get('truth_goal_gate_passed'),
            'last_sample_truth_goal_distance_m': last_distance,
            'last_sample_distance_le_0_35m': last_distance <= 0.35 if last_distance is not None else None,
            'trajectory_messages': run.get('trajectory_messages'),
            'mpc_messages': run.get('mpc_messages'),
            'mpc_nonzero_messages': run.get('mpc_nonzero_messages'),
            'motion_evidence_passed': run.get('motion_evidence_passed'),
        })
    valid = [r for r in rows if r.get('metrics') is not None]
    mode = runtime.get('mode_label')
    if not mode:
        cmd = ' '.join(manifest.get('launch_command', []))
        mode = 'GT' if 'use_ground_truth_odom:=true' in cmd else ('full_chain' if 'use_ground_truth_odom:=false' in cmd else None)
    try:
        log = (repeat / 'launch.log').read_text(encoding='utf-8', errors='replace').splitlines()
    except OSError:
        log = []
    try:
        observer_log = (repeat / 'observer.log').read_text(encoding='utf-8', errors='replace')
    except OSError:
        observer_log = ''
    harness_input_failure = 'PyFloat_Check(field)' in observer_log
    failure_stage = ('observer_goal_serialization' if harness_input_failure else
                     'before_goal_record' if not rows and status == 'failed' else None)
    keys = ('candidate_count', 'long_count', 'ramp_count', 'tunnel_count')
    return {
        'repeat': repeat.name, 'manifest_status': status, 'partial': partial,
        'failure_stage': failure_stage,
        'startup_failure_inferred': None if partial else (
            False if harness_input_failure or rows else None),
        'seed': manifest.get('seed'), 'mode': mode,
        'mode_verified': runtime.get('mode_label_is_verified'),
        'mode_verification': runtime.get('mode_verification'),
        'mode_evidence': runtime.get('mode_evidence'),
        'planner_parameters': runtime.get('nodes', {}).get('planner_server', {}).get('parameters'),
        'controller_parameters': runtime.get('nodes', {}).get('controller_server', {}).get('parameters'),
        'attempted_legs': len(rows), 'sampled_legs': len(valid),
        'succeeded_legs': sum(r.get('status') == 'succeeded' for r in rows),
        'leg_outcomes': dict(Counter(r.get('status', 'unknown') for r in rows)),
        'expected_legs': len(scenario.get('goals', [])),
        'complete_route': len(rows) == len(scenario.get('goals', [])) and all(r.get('status') == 'succeeded' for r in rows) if not partial else None,
        'sampled_duration_s': sum(r['metrics']['duration'] or 0 for r in valid) if valid else None,
        'sampled_path_m': sum(r['metrics']['path_m'] for r in valid) if valid else None,
        'candidate_counts': {**{key: sum(r['metrics'][key] for r in valid) for key in keys},
                             'short_0_1s': sum(r['short_0_1s_candidates'] for r in valid)},
        'action_event_counts': counts(action_messages),
        'whole_launch_log_counts': counts(log),
        'dds_profile': manifest.get('dds_profile'),
        'dds_profile_sha256': manifest.get('dds_profile_sha256'),
        'params_sha256': manifest.get('params_sha256'),
        'binary_sha256': {k: v for k, v in manifest.get('file_sha256', {}).items() if k.startswith('install/')},
        'resolved_binaries': manifest.get('resolved_binaries', {}),
        'resolved_prior_pcd': manifest.get('resolved_prior_pcd'),
        'resolved_prior_pcd_sha256': manifest.get('resolved_prior_pcd_sha256'),
        'legs': rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=HERE / 'results/20261028_followup')
    parser.add_argument('--final', action='store_true', help='refuse finalization while any repeat is partial')
    args = parser.parse_args()
    scenarios = {key: read(HERE / filename) for key, filename in [('fixed', 'scenario.json'), ('south', 'scenario_south.json')]}
    groups = []
    for group in sorted(args.root.iterdir()):
        if not group.is_dir():
            continue
        repeats = sorted(p for p in group.glob('repeat_*') if p.is_dir())
        if not repeats:
            continue
        scenario_name = 'south' if group.name.startswith('south_') else 'fixed'
        groups.append({'group': group.name, 'scenario': scenario_name,
                       'repeats': [build_repeat(p, scenarios[scenario_name]) for p in repeats]})
    if args.final and any(r['partial'] for g in groups for r in g['repeats']):
        parser.error('cannot finalize: a repeat has no terminal manifest status')
    output = {'provisional': not args.final, 'notes': [
        'Running or missing manifest status is partial; no startup failure or completed route is inferred.',
        'Candidate stops are not proven unintended; action event counts use observer /rosout events between action sim timestamps, whole launch log counts include startup and shutdown.',
        'Recorded truth gate is authoritative when present; last sampled distance is diagnostic and does not retroactively establish a gate.',
        'Null hashes mean not recorded, not a matching or default hash.',
        'Dense configuration returned via a roughly 38 m detour and was not adopted; IMU input true timed out on the first 90 s leg and was not adopted.',
        '4DOF GICP is an experiment, disabled by default; finalization means records are closed, not that unresolved navigation issues are solved.'
    ], 'groups': groups}
    output_path = args.root / 'followup_summary.json'
    output_path.write_text(json.dumps(output, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    print(output_path)


if __name__ == '__main__':
    main()
