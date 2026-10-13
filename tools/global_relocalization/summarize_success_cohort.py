#!/usr/bin/env python3
"""Summarize timing/resources for strictly joined, fully scored successes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/global_relocalization/scripts'))
from evaluate_pose import _summarize_scored_trials


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = json.loads(Path(args.report).read_text())
    rows = [row for row in report['trials'] if row['success']]
    summary = _summarize_scored_trials(rows, len(rows))
    fields = ['timing', 'status_stage_timing', 'algorithm_process', 'gazebo_process_load',
              'trial_evaluator_process', 'diagnostic_recorder_process', 'frontend_process',
              'lio_pipeline', 'rtf']
    cohort = {'cohort': 'successful actions with exact timestamp join and full 3D/SO(3) score pass',
              'source_report': str(Path(args.report).resolve()),
              'source_report_sha256': hashlib.sha256(Path(args.report).read_bytes()).hexdigest(),
              'derivation_source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in [Path(__file__).resolve(), ROOT/'src/global_relocalization/scripts/evaluate_pose.py']},
              'selected_trial_ids': [row['trial_id'] for row in rows], 'selected_count': len(rows),
              'full_batch_denominator': report['summary']['batch']['expected_trials'],
              'does_not_replace_full_batch_success_rate': True,
              **{name: summary[name] for name in fields}}
    Path(args.output).write_text(json.dumps(cohort, indent=2)+'\n')


if __name__ == '__main__':
    main()
