"""Render comparable ground-truth trajectories and candidate stops on the map."""
import argparse
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

try:
    from .metrics import analyze_run
except ImportError:
    from metrics import analyze_run

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--candidate-extra', action='append', type=Path, default=[])
    parser.add_argument('--baseline-extra', action='append', type=Path, default=[])
    parser.add_argument('--baseline-label', default='Baseline')
    parser.add_argument('--candidate-label', default='Candidate')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--scenario', type=Path, default=ROOT / 'tools/navigation_benchmark/scenario.json')
    parser.add_argument('--map', type=Path, default=ROOT / 'src/pb2025_nav_bringup/map/simulation/RMUC2026.pgm')
    args = parser.parse_args()
    scenario = json.loads(args.scenario.read_text(encoding='utf-8'))
    background = plt.imread(args.map)
    height, width = background.shape[:2]
    extent = [-14.6, -14.6 + width * .05, -5.86, -5.86 + height * .05]
    groups = [sorted(p.rglob('run_??.json')) for p in (args.baseline, args.candidate)]
    for directory in args.baseline_extra:
        groups[0].extend(sorted(directory.rglob('run_??.json')))
    for directory in args.candidate_extra:
        groups[1].extend(sorted(directory.rglob('run_??.json')))
    def repeat_label(path):
        match = re.search(r'_seed(\d+)$', path.parent.parent.name)
        return 'seed ' + match.group(1) if match else path.parent.name
    repeats = sorted({repeat_label(p) for group in groups for p in group})
    colors = {name: plt.get_cmap('tab10')(i % 10) for i, name in enumerate(repeats)}
    fig, axes = plt.subplots(1, 2, figsize=(16, 7), sharex=True, sharey=True, constrained_layout=True)
    for ax, title, paths in zip(axes, (args.baseline_label, args.candidate_label), groups):
        ax.imshow(background, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=255)
        for region in scenario.get('regions', []):
            ax.add_patch(Rectangle((region['xmin'], region['ymin']), region['xmax']-region['xmin'],
                                   region['ymax']-region['ymin'], fill=False, edgecolor='orange', alpha=.45, linewidth=1))
        successes = long_count = 0
        for path in paths:
            run = json.loads(path.read_text(encoding='utf-8'))
            successes += run.get('status') == 'succeeded'
            samples = run.get('samples', [])
            # NaNs break lines where ground-truth positions are unavailable.
            xy = [(s.get('x'), s.get('y')) if isinstance(s.get('x'), (int, float)) and isinstance(s.get('y'), (int, float)) else (np.nan, np.nan) for s in samples]
            if xy:
                ax.plot(*zip(*xy), color=colors[repeat_label(path)], linewidth=1.1, alpha=.8)
            metrics = analyze_run(run, scenario=scenario)
            for episode in metrics['episodes']:
                if not episode['unexpected_candidate']:
                    continue
                long_count += episode['long_stop']
                pos = episode['start_position']
                if pos['x'] is not None and pos['y'] is not None:
                    ax.scatter(pos['x'], pos['y'], c='red', s=18, zorder=4)
                    ax.annotate(f"{episode['duration']:.1f}s", (pos['x'], pos['y']), xytext=(3, 3), textcoords='offset points', fontsize=7, color='darkred')
            if run.get('status') != 'succeeded' and xy and np.isfinite(xy[-1]).all():
                ax.scatter(*xy[-1], marker='x', color='black', s=55, linewidths=2, zorder=5)
        ax.set_title(f'{title}: {successes}/{len(paths)} legs succeeded; {long_count} long stops')
        ax.set(xlim=extent[:2], ylim=extent[2:], xlabel='x (m)', ylabel='y (m)')
        ax.set_aspect('equal')
    handles = [plt.Line2D([0], [0], color=color, label=name) for name, color in colors.items()]
    handles += [plt.Line2D([0], [0], marker='o', color='red', linestyle='', label='Candidate stop >= 1s'),
                plt.Line2D([0], [0], marker='x', color='black', linestyle='', label='Failed endpoint')]
    fig.legend(handles=handles, loc='lower center', ncol=min(5, len(handles)), fontsize=8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160, bbox_inches='tight')
    plt.close(fig)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
