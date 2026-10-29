#!/usr/bin/env python3
"""Summarize observer receipt timing without equating it to node receive timing."""
import argparse
import bisect
from collections import defaultdict
import json
import math
from pathlib import Path


def quantiles(values):
    values = sorted(values)
    if not values:
        return None
    return {str(p): values[round((len(values)-1)*p/100)] for p in (0, 50, 95, 99, 100)}


def analyze(root):
    data = defaultdict(list)
    for line in (root / 'input_audit.jsonl').read_text().splitlines():
        item = json.loads(line)
        data[item['topic']].append(item)
    result = {'note': 'Receipt times belong to the independent observer, not planner/controller callbacks. Nearest odometry uses all observed stamps; it is an availability check, not proof of online pairing.', 'topics': {}}
    for topic, records in data.items():
        gaps = [b['receipt_monotonic']-a['receipt_monotonic'] for a,b in zip(records,records[1:])]
        result['topics'][topic] = {'count': len(records), 'wall_receive_gap_seconds': quantiles(gaps), 'wall_receive_gaps_over_025': sum(x > .25 for x in gaps), 'stamp_gap_seconds': quantiles([b['stamp']-a['stamp'] for a,b in zip(records,records[1:])]), 'sim_stamp_age_seconds': quantiles([r['receipt_sim']-r['stamp'] for r in records if r['receipt_sim'] is not None])}
        if records and 'points' in records[0]:
            result['topics'][topic]['point_count'] = quantiles([r['points'] for r in records])
    stamps = sorted(r['stamp'] for r in data['/aft_mapped_to_init'])
    deltas=[]
    for r in data['/cloud_registered']:
        index=bisect.bisect_left(stamps,r['stamp'])
        nearby=stamps[max(0,index-1):index+1]
        if nearby: deltas.append(min(abs(x-r['stamp']) for x in nearby))
    result['cloud_nearest_observed_odom_stamp_delta'] = quantiles(deltas)
    result['cloud_stamp_pairs_over_010'] = sum(d > .1 for d in deltas)
    roll_pitch=[]
    synchronized_xy=[]
    synchronized_z=[]
    for path in sorted(root.glob('run_*.json')):
        if path.name.endswith('_metrics.json'):continue
        samples=json.loads(path.read_text()).get('samples',[])
        truth_stamps=[s['t'] for s in samples]
        for s in samples:
            stamp=s.get('loc_transform_stamp')
            if stamp is not None and s.get('loc_x') is not None:
                j=bisect.bisect_left(truth_stamps,stamp)
                if 0 < j < len(samples):
                    a,b=samples[j-1],samples[j]
                    gap=b['t']-a['t']
                    if 0 < gap <= .3:
                        alpha=(stamp-a['t'])/gap
                        truth=[a[k]+alpha*(b[k]-a[k]) for k in ('x','y','z')]
                        synchronized_xy.append(math.hypot(s['loc_x']-truth[0],s['loc_y']-truth[1]))
                        synchronized_z.append(abs(s['loc_z']-truth[2]))
            t=s.get('transforms_latest',{}).get('map<-camera_init')
            if t:
                x,y,z,w=t['quaternion_xyzw']
                roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
                pitch=math.asin(max(-1,min(1,2*(w*y-z*x))))
                roll_pitch.append(max(abs(roll),abs(pitch)))
    result['synchronized_tf_truth_xy_error_m'] = quantiles(synchronized_xy)
    result['synchronized_tf_truth_z_error_m'] = quantiles(synchronized_z)
    result['synchronized_pose_note'] = 'Stored TF position at loc_transform_stamp compared to linearly interpolated truth at the same timestamp, requiring a bracketing gap <=0.3s. Includes TF/localization errors; interpolation is approximate.'
    result['map_camera_max_abs_roll_pitch_rad'] = quantiles(roll_pitch)
    (root/'input_audit_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);args=parser.parse_args()
    print(json.dumps(analyze(args.root),indent=2))
