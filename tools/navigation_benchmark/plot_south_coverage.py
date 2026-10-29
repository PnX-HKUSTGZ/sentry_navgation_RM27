#!/usr/bin/env python3
"""Plot each full-chain south route with labels from the crossing audit."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'tools/navigation_benchmark/results/20261028_followup'
GROUPS = [('6D, 0.8', 'south_full_chain_retry_seed{}'),
          ('4D, slow 0.6', 'south_gravity_slow_{}seed{}'),
          ('6D, dense', 'south_high_density_seed{}')]


def main():
    audit = json.loads((BASE / 'south_crossing_audit.json').read_text())
    rows = {row['file']: row for row in audit['runs']}
    region = audit['region']
    background = plt.imread(ROOT / 'src/pb2025_nav_bringup/map/simulation/RMUC2026.pgm')
    h, w = background.shape[:2]
    extent = [-14.6, -14.6+w*.05, -5.86, -5.86+h*.05]
    fig, axes = plt.subplots(3, 3, figsize=(15, 11), sharex=True, sharey=True, constrained_layout=True)
    colors = {2: 'tab:blue', 3: 'tab:red'}
    for row, seed in enumerate((42, 43, 44)):
        for col, (label, pattern) in enumerate(GROUPS):
            group = (pattern.format('retry_' if seed == 42 else '', seed) if col == 1
                     else pattern.format(seed))
            ax = axes[row, col]
            ax.imshow(background, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=255)
            ax.add_patch(Rectangle((region['xmin'], region['ymin']), region['xmax']-region['xmin'],
                region['ymax']-region['ymin'], edgecolor='darkorange', facecolor='none', linewidth=1.5))
            descriptions=[]
            for leg in (2, 3):
                relative = f'{group}/repeat_01/run_{leg:02d}.json'
                if relative not in rows:
                    descriptions.append(f'{leg}: unattempted')
                    continue
                data = json.loads((BASE / relative).read_text())
                xy = [(s['x'], s['y']) for s in data['samples'] if s.get('x') is not None and s.get('y') is not None]
                if xy:
                    ax.plot(*zip(*xy), color=colors[leg], linewidth=1.2)
                    ax.scatter(*xy[-1], marker='o' if data['status']=='succeeded' else 'x', color=colors[leg], s=18)
                crossed = bool(rows[relative]['observed_crossings'])
                descriptions.append(f'{leg}: {"crossed" if crossed else "no crossing"}, {data["status"]}')
            ax.set_title(f'{label}, seed {seed}\n'+'\n'.join(descriptions), fontsize=8)
            ax.set_aspect('equal')
            ax.set(xlim=(-7, 9), ylim=(-5.86, 8.3), xlabel='world x (m)', ylabel='world y (m)')
    fig.legend([plt.Line2D([0],[0],color=colors[2]),plt.Line2D([0],[0],color=colors[3])],
               ['leg 2 west to east goal','leg 3 east to west goal'],loc='lower center',ncol=2)
    fig.savefig(BASE / 'south_crossing_paths.png', dpi=150)


if __name__ == '__main__':
    main()
