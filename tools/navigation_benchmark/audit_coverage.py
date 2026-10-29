"""Audit observed crossings of a tunnel's rectangular route region, not clearance."""
import argparse
import json
from pathlib import Path


def crossings(samples, region, max_gap=0.3):
    result = []
    start_side = start_time = previous_time = None
    visited_inside = False
    for sample in samples:
        t, x, y = (sample.get(k) for k in ('t', 'x', 'y'))
        valid = all(isinstance(v, (int, float)) for v in (t, x, y))
        in_band = valid and region['ymin'] <= y <= region['ymax']
        gap = valid and previous_time is not None and not 0 < t - previous_time <= max_gap
        if not in_band or gap:
            start_side = start_time = None
            visited_inside = False
        previous_time = t if valid else None
        if not in_band:
            continue
        side = -1 if x < region['xmin'] else 1 if x > region['xmax'] else 0
        if side == 0:
            visited_inside = start_side is not None
        else:
            if start_side is not None and visited_inside and side != start_side:
                result.append({'direction': 'west_to_east' if side == 1 else 'east_to_west',
                               'start_time': start_time, 'end_time': t})
            start_side, start_time, visited_inside = side, t, False
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    scenario = json.loads((Path(__file__).parent / 'scenario_south.json').read_text())
    region = next(r for r in scenario['regions'] if r['name'] == 'south_tunnel')
    rows = []
    for path in sorted(args.root.glob('south_*/repeat_*/run_??.json')):
        run = json.loads(path.read_text())
        rows.append({'file': str(path.relative_to(args.root)), 'status': run.get('status'),
                     'observed_crossings': crossings(run.get('samples', []), region)})
    output = {'region': region, 'note': 'Requires a contiguous sampled center path within the y band, visiting the interior and both x sides; gaps >0.3s reset evidence. This is route coverage, not swept-body or physical clearance certification.', 'runs': rows}
    destination = args.root / 'south_crossing_audit.json'
    destination.write_text(json.dumps(output, indent=2) + '\n')
    print(destination)


if __name__ == '__main__':
    main()
