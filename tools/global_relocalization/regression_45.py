#!/usr/bin/env python3
"""Run three frozen 15-cell correctness rounds in an already isolated simulation.

No bag recorders or diagnostic point dumps are enabled. Every round has its own
algorithm/evaluator processes and output directory. Existing outputs are refused.
A failed round stops further execution for root-cause repair; remaining cells
stay explicitly unattempted and the planned denominator remains 45.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import signal
import time
from pathlib import Path
import subprocess
import sys
import uuid
import yaml

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def changed_paths(frozen):
    return [path for path, expected in frozen.items() if not Path(path).is_file() or digest(path) != expected]


def write(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')



def valid_cell(row, run_id, repetition):
    if row.get('run_id') != run_id or row.get('repetition') != repetition or row.get('success') is not True:
        return False
    for field, limit in [('position_3d_error_m', .1), ('orientation_geodesic_error_deg', 2.)]:
        value = row.get(field)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or value > limit:
            return False
    return True


def round_passes(report, run_id, repetition, expected_ids):
    rows = report.get('trials', [])
    batch = report.get('summary', {}).get('batch', {})
    return (len(rows) == 15 and batch.get('observed_trials') == 15 and batch.get('success_count') == 15
        and {r.get('trial_id') for r in rows} == set(expected_ids)
        and all(valid_cell(r, run_id, repetition) for r in rows))


def round_cells(folder, report, run_id, repetition, expected_ids):
    ledger_path = folder/'trials.jsonl'
    ledger = [json.loads(line) for line in ledger_path.read_text().splitlines() if line.strip()] if ledger_path.exists() else []
    scored = report.get('trials', [])
    result = []
    for trial_id in expected_ids:
        scores = [r for r in scored if r.get('trial_id') == trial_id]
        raw = next((r for r in ledger if r.get('trial_id') == trial_id), None)
        attempted = bool(scores or raw is not None or (folder/trial_id).exists())
        state = 'unattempted'
        if len(scores) == 1:
            state = 'passed' if valid_cell(scores[0], run_id, repetition) else 'failed'
        elif scores:
            state = 'failed'
        elif raw is not None:
            state = 'failed' if raw.get('action',{}).get('success') is False else 'executed_unscored'
        elif attempted:
            state = 'executed_unscored'
        row = {'run_id':run_id, 'repetition':repetition, 'trial_id':trial_id, 'state':state, 'executed':attempted}
        if len(scores)==1:
            for key in ('position_3d_error_m','orientation_geodesic_error_deg'):
                value=scores[0].get(key)
                row[key]=value if type(value) in (float,int) and math.isfinite(value) else None
        result.append(row)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('map', 'manifest', 'params', 'trials', 'output', 'physical-precheck', 'simulation-audit'):
        p.add_argument('--' + name, required=True)
    p.add_argument('--wall-timeout', type=float, default=180)
    p.add_argument('--ready-timeout', type=float, default=90)
    a = p.parse_args()
    profile = yaml.safe_load(Path(a.params).read_text())['global_relocalization']['ros__parameters']
    cases = json.loads(Path(a.trials).read_text())['trials']
    if len(cases) != 15 or len({t['trial_id'] for t in cases}) != 15:
        p.error('requires exactly 15 unique frozen trial IDs')
    if profile.get('input_mode') != 'raw_imu' or profile.get('diagnostic_output_dir', ''):
        p.error('requires raw_imu with diagnostic_output_dir empty')
    if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('GZ_PARTITION'):
        p.error('requires explicit isolated ROS_DOMAIN_ID and GZ_PARTITION')
    out = Path(a.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    run_id = out.name + '_' + uuid.uuid4().hex[:12]
    inputs = [Path(getattr(a, k)).resolve() for k in ('map', 'manifest', 'params', 'trials', 'simulation_audit')]
    inputs += [Path(a.physical_precheck).resolve()/n for n in ('manifest.json', 'checks.json', 'summary.json')]
    inputs += list((ROOT/'src/global_relocalization/src').glob('*.cpp'))
    inputs += list((ROOT/'src/global_relocalization/include/global_relocalization').glob('*.hpp'))
    inputs += list((ROOT/'tools/global_relocalization').glob('*.py'))
    inputs += list((ROOT/'tools/global_relocalization').glob('*.bash'))
    inputs += list((ROOT/'src/global_relocalization/scripts').glob('*.py'))
    inputs += list((ROOT/'src/global_relocalization/launch').glob('*.py'))
    precheck_manifest=json.loads((Path(a.physical_precheck)/'manifest.json').read_text())
    inputs += [Path(path) for path in precheck_manifest.get('verified_inputs',{})]
    from ament_index_python.packages import get_package_prefix
    prefix = Path(get_package_prefix('global_relocalization'))
    inputs += [(prefix/'lib/global_relocalization/global_relocalization_node').resolve(), (prefix/'lib/libglobal_relocalization_core.so').resolve()]
    frozen = {str(path): digest(path) for path in inputs}
    frozen_stats={path:(Path(path).stat().st_mtime_ns,Path(path).stat().st_size) for path in frozen}
    for name in ('params', 'trials', 'manifest'):
        (out/('frozen_' + name + Path(getattr(a,name)).suffix)).write_bytes(Path(getattr(a,name)).read_bytes())
    evidence = {'run_id': run_id, 'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'planned_trials': 45, 'fixed_cells_per_round': 15, 'repetitions': 3,
        'environment': {'ros_domain_id': os.environ['ROS_DOMAIN_ID'], 'gz_partition': os.environ['GZ_PARTITION']},
        'frozen_sha256': frozen, 'ground_truth_used_for_estimation': False,
        'integrity_observer':{'process':'regression harness, excluded from algorithm process cost','stat_period_wall_sec':1.,'hash_on_file_stat_change':True},
        'performance_capture': {'bags': False, 'point_dumps': False, 'gui': False},
        'scope': '100 percent applies only to this frozen regression set', 'rounds': [],
        'planned_cells': [{'run_id': run_id, 'repetition': r, 'trial_id': t['trial_id'], 'state': 'unattempted'} for r in range(1,4) for t in cases]}
    write(out/'regression_manifest.json', evidence)
    for rep in range(1,4):
        changed = changed_paths(frozen)
        if changed:
            evidence['blocked_reason'] = 'frozen inputs changed: ' + repr(changed)
            break
        folder = out/f'round_{rep:02d}'
        command = ['bash', str(ROOT/'tools/global_relocalization/run_batch.bash')]
        for name in ('map', 'manifest', 'params', 'trials', 'physical_precheck', 'simulation_audit'):
            command += ['--'+name.replace('_','-'), str(Path(getattr(a,name)).resolve())]
        command += ['--output', str(folder), '--input-mode', 'raw_imu', '--run-id', run_id, '--repetition', str(rep), '--wall-timeout', str(a.wall_timeout), '--ready-timeout', str(a.ready_timeout)]
        changed_during_round=[]
        with (out/f'round_{rep:02d}.log').open('w') as stream:
            process=subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,start_new_session=True)
            try:
                while process.poll() is None:
                    time.sleep(1.)
                    for path,expected in frozen.items():
                        target=Path(path)
                        if not target.exists() or ((target.stat().st_mtime_ns,target.stat().st_size)!=frozen_stats[path] and digest(path)!=expected):
                            changed_during_round.append(path)
                    if changed_during_round:
                        os.killpg(process.pid,signal.SIGINT)
                        break
                code=process.wait(timeout=60)
            except (KeyboardInterrupt,subprocess.TimeoutExpired):
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=20)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
                code=130
        summary_path = folder/'results.summary.json'
        report = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        batch = report.get('summary', {}).get('batch', {})
        rows = report.get('trials', [])
        passed = not changed_during_round and code == 0 and round_passes(report, run_id, rep, [t['trial_id'] for t in cases])
        observed_cells=round_cells(folder, report, run_id, rep, [t['trial_id'] for t in cases])
        evidence['planned_cells']=[c for c in evidence['planned_cells'] if c['repetition'] != rep]+observed_cells
        passed = passed and all(c['state']=='passed' for c in observed_cells)
        evidence['rounds'].append({'repetition': rep, 'command': command, 'returncode': code, 'summary': batch, 'frozen_changed_during_round':changed_during_round, 'timely_correct': report.get('summary',{}).get('correct_localization_within_15s'), 'passed': passed})
        write(out/'regression_manifest.json', evidence)
        if not passed:
            evidence['blocked_reason'] = 'correctness round failed; repair before a new frozen regression'
            break
    evidence['passed_cells'] = sum(c['state']=='passed' for c in evidence['planned_cells'])
    evidence['executed_cells'] = sum(c.get('executed',False) for c in evidence['planned_cells'])
    evidence['not_attempted_cells'] = 45-evidence['executed_cells']
    evidence['completed_45'] = evidence['executed_cells']==45
    evidence['success_rate_executed_only'] = evidence['passed_cells']/evidence['executed_cells'] if evidence['executed_cells'] else None
    evidence['passed_45_of_45'] = len(evidence['rounds']) == 3 and all(r['passed'] for r in evidence['rounds'])
    evidence['correct_localization_within_15s_by_round'] = [r.get('timely_correct') for r in evidence['rounds']]
    timely = [r.get('timely_correct') or {} for r in evidence['rounds']]
    count = sum(r.get('confirmed_correct_count',0) for r in timely)
    successes = sum(r.get('action_success_count',0) for r in timely)
    evidence['correct_localization_within_15s'] = {'confirmed_correct_count':count, 'expected_cohort_count':45, 'confirmed_correct_rate_all_cohort':count/45, 'action_success_count':successes, 'confirmed_correct_rate_among_successful_actions':count/successes if successes else None, 'action_elapsed_available_trial_count':sum(r.get('action_elapsed_available_trial_count',0) for r in timely), 'action_elapsed_missing_trial_count':sum(r.get('action_elapsed_missing_trial_count',0) for r in timely), 'unobserved_trial_count':45-evidence['executed_cells']}
    evidence['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    evidence['frozen_inputs_unchanged_at_end'] = not changed_paths(frozen)
    evidence['passed_45_of_45'] &= evidence['frozen_inputs_unchanged_at_end']
    write(out/'regression_manifest.json', evidence)
    print(json.dumps({'output': str(out), 'passed_cells': evidence['passed_cells'], 'planned_trials': 45, 'passed_45_of_45': evidence['passed_45_of_45']}))
    return 0 if evidence['passed_45_of_45'] else 1


if __name__ == '__main__':
    sys.exit(main())
