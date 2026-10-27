#!/usr/bin/env python3
"""Independent ordered-station experiment; every output directory must be new."""
import argparse
from pathlib import Path
import sys

from pipeline_common import REPO


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='stage', required=True)
    p = commands.add_parser('prepare', help='extract raw/IMU; calibrate and freeze actual gyro full-turn roles')
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--raw-prior', type=Path, required=True)
    p.add_argument('--profile', type=Path, default=REPO/'src/global_relocalization/config/competition.yaml')
    p.add_argument('--bag', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p = commands.add_parser('local', help='run existing isolated native frontend; all scans required')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--native', type=Path, required=True)
    p.add_argument('--support', action='store_true', help='accept a complete bounded-recovery source ledger; registrations remain rejected')
    p.add_argument('--output', type=Path, required=True)
    p = commands.add_parser('support', help='derive heldout-only isolated IMU bridges; never fuse or train rejected scans')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--local', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p = commands.add_parser('seed', help='original LIO seed composition; no post-evaluation inputs')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--local', type=Path, required=True)
    p.add_argument('--support', type=Path, help='explicit completed pose-support directory')
    p.add_argument('--output', type=Path, required=True)
    p = commands.add_parser('domains', help='whole-turn train/heldout caches; freeze every pair domain before fitting')
    p.add_argument('--seed', type=Path, required=True)
    p.add_argument('--support', action='store_true', help='explicitly consume supported heldout poses, retaining the original raw denominator')
    p.add_argument('--output', type=Path, required=True)
    p = commands.add_parser('pairs', help='all unordered station pairs; both directions, fixed observation model')
    p.add_argument('--domains', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p = commands.add_parser('graph', help='full-rank/redundancy audit; guarded dense-exact full SE3 graph')
    p.add_argument('--pairs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = parser.parse_args(argv)
    if a.stage == 'prepare':
        from pipeline_inputs import prepare
        passed = prepare(a.capture, a.raw_prior, a.profile, a.output, a.bag)
    elif a.stage == 'local':
        from pipeline_inputs import local
        passed = local(a.input, a.native, a.output, allow_support=a.support)
    elif a.stage == 'support':
        from pipeline_support import support
        passed = support(a.input, a.local, a.output)
    elif a.stage == 'seed':
        from pipeline_inputs import sensor_seed
        passed = sensor_seed(a.input, a.local, a.output, support_dir=a.support)
    elif a.stage == 'domains':
        from pipeline_pairs import build_domains
        passed = build_domains(a.seed, a.output, allow_support=a.support)
    elif a.stage == 'pairs':
        from pipeline_pairs import fit_pairs
        passed = fit_pairs(a.domains, a.output)
    else:
        from pipeline_graph import solve_graph
        passed = solve_graph(a.pairs, a.output)
    # A complete experiment with a rejected geometry/model is not a passing exit.
    return 0 if passed else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, RuntimeError, KeyError, OSError) as error:
        print('station pipeline rejected: ' + str(error), file=sys.stderr)
        sys.exit(2)
