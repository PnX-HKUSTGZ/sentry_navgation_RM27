#!/usr/bin/env python3
"""Plot recorded first global guides against the same prior map."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import yaml


def first_guide(path):
    run = json.loads(path.read_text())
    return np.array(next(p['points'] for p in run['global_paths']
                         if len(p['points']) > 1 and np.linalg.norm(np.array(p['points'][-1])-run['goal']) < 0.35))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--smoothed', required=True, type=Path)
    parser.add_argument('--map', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    before, after = first_guide(args.baseline), first_guide(args.smoothed)
    config = yaml.safe_load(args.map.read_text())
    raster = np.asarray(Image.open(args.map.parent / config['image']))
    x, y = config['origin'][:2]
    resolution = config['resolution']
    extent = [x, x+raster.shape[1]*resolution, y, y+raster.shape[0]*resolution]
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for ax in axes:
        ax.imshow(raster, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=255)
        ax.plot(before[:, 0], before[:, 1], '.-', ms=2, lw=1.2, color='#3167a1', label='Smoothing off')
        ax.plot(after[:, 0], after[:, 1], '.-', ms=2, lw=1.5, color='#d25828', label='Smoothing on')
        ax.set_aspect('equal')
        ax.set_xlabel('map x (m)')
        ax.set_ylabel('map y (m)')
        ax.legend()
    axes[0].set_title('RMUC2026: first global guide, seed 42')
    axes[0].set_xlim(before[:, 0].min()-1, before[:, 0].max()+1)
    axes[0].set_ylim(before[:, 1].min()-1, before[:, 1].max()+1)
    # Show the largest removed grid zigzag away from endpoint connectors.
    bend = np.linalg.norm(before[:-2]-2*before[1:-1]+before[2:], axis=1)
    candidate = np.arange(5, len(before)-5)
    center = before[candidate[np.argmax(bend[candidate-1])]]
    axes[1].set_xlim(center[0]-0.75, center[0]+0.75)
    axes[1].set_ylim(center[1]-0.75, center[1]+0.75)
    axes[1].set_title('Close-up: constrained corner smoothing')
    figure.savefig(args.output, dpi=180)


if __name__ == '__main__':
    main()
