#!/usr/bin/env python3
"""Plot the retained two-seed RMUC2026 stability comparison."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def aggregate(runs):
    values = list(runs.values())
    def rms(field, metric):
        duration = sum(r[field]['duration_s'] for r in values)
        return math.sqrt(sum(r[field][metric]**2 * r[field]['duration_s']
                             for r in values) / duration)
    return [sum(c['duration_s'] for r in values for c in r['crossings']),
            sum(r['duration_s'] for r in values),
            *[rms(field, k) for field in ('lateral_motion', 'level_ground_motion')
              for k in ('command_lateral_rate_rms', 'lateral_accel_rms')]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = {Path(k).name: v for k, v in json.loads(args.comparison.read_text()).items()}
    pairs = [(f'enabled_seed{s}', f'damping6_gt{s}') for s in (42, 43)] + [
        (f'baseline_south_gt{s}', f'damping6_south_gt{s}') for s in (42, 43)]
    before = [aggregate(data[a]) for a, _ in pairs]
    after = [aggregate(data[b]) for _, b in pairs]
    fig, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
    titles = ['Observed tunnel crossing time (s)', 'Complete route time (s)',
              'All terrain: lateral command change RMS (m/s²)',
              'All terrain: measured lateral acceleration RMS (m/s²)',
              'Level ground: lateral command change RMS (m/s²)',
              'Level ground: measured lateral acceleration RMS (m/s²)']
    for metric, ax in enumerate(axes.flat):
        ax.bar([i-.19 for i in range(4)], [r[metric] for r in before], .38, label='Baseline', color='#637e9a')
        ax.bar([i+.19 for i in range(4)], [r[metric] for r in after], .38, label='Final', color='#e28b35')
        ax.set_xticks(range(4), ['North / 42', 'North / 43', 'South / 42', 'South / 43'])
        ax.set_title(titles[metric])
        ax.grid(axis='y', alpha=.2)
        ax.set_axisbelow(True)
    axes[0, 0].legend()
    fig.suptitle('RMUC2026 ground-truth localization — two seeds per route\n'
                 'Motion metrics include intended turns; crossing coverage is not collision certification.', fontsize=12)
    fig.savefig(args.output, dpi=160)


if __name__ == '__main__':
    main()
