#!/usr/bin/env python3
"""Independent post-only GT evaluation of completed arbitrary-N local results.

This module never runs an estimator. The input station windows and all pose rows
are consumed unchanged, including original rejection records. Its output belongs
in a new post_evaluation directory and must never feed an estimation stage.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from evaluation import interpolate, score_pose
from export_dense_sensor_prior import Trajectory
from pipeline_common import rigid


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


class Sources:
    def __init__(self):
        self.hashes = {}

    def read(self, path, expected=None):
        path = Path(path).resolve()
        value = digest(path)
        if expected is not None and value != expected:
            raise ValueError('completed artifact hash mismatch: ' + str(path))
        if str(path) in self.hashes and self.hashes[str(path)] != value:
            raise ValueError('input changed: ' + str(path))
        self.hashes[str(path)] = value
        return json.loads(path.read_text())

    def code(self, path):
        path = Path(path).resolve()
        self.hashes[str(path)] = digest(path)

    def verify(self):
        if any(digest(p) != value for p, value in self.hashes.items()):
            raise ValueError('post-evaluation input changed during scoring')


def summarize(scores):
    good = [s for s in scores if s is not None and s.get('valid')]
    result = {'rows': len(scores), 'valid': len(good),
              'missing_or_invalid': len(scores) - len(good)}
    for key in ('translation_3d_m', 'orientation_so3_deg'):
        a = np.array([s[key] for s in good])
        result[key] = ({'mean': float(a.mean()), 'p95': float(np.percentile(a, 95)),
                        'max': float(a.max()), 'final_valid_row': float(a[-1])}
                       if len(a) else None)
    result['strict_010m_2deg'] = sum(s['strict_correct'] for s in good)
    result['sensor_015m_3deg'] = sum(s['sensor_map_correct'] for s in good)
    return result


def score_or_missing(estimate, truth, accepted=True):
    return score_pose(estimate, truth) if accepted and estimate is not None and truth is not None else None


SCORE_KEYS = (
    'old_relative_score', 'new_relative_score',
    'old_LIO_seed_global_score', 'new_LIO_seed_global_score',
    'original_LIO_relative_score', 'original_LIO_global_score',
)


def summaries(rows):
    return {key: summarize([r[key] for r in rows]) for key in SCORE_KEYS}


def completed_local(directory, sources):
    directory = Path(directory).resolve()
    provenance = sources.read(directory / 'provenance.json')
    if provenance.get('stage') != 'local':
        raise ValueError('local final provenance is absent')
    artifacts = provenance['artifact_sha256']
    summary_path = directory / 'summary.json'
    summary = sources.read(summary_path, artifacts[str(summary_path)])
    native = Path(summary['native_output']).resolve()
    if native != directory / 'native':
        raise ValueError('native output escapes supplied local result directory')
    native_summary = sources.read(native / 'summary.json', artifacts[str(native / 'summary.json')])
    return directory, native, artifacts, summary, native_summary


def evaluate(capture_dir, raw_prior, plan_path, old_local, new_local, output, maximum_gap=.25):
    if not np.isfinite(maximum_gap) or maximum_gap <= 0:
        raise ValueError('invalid interpolation gap')
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    sources = Sources()
    for code in (Path(__file__), HERE/'pipeline_common.py', HERE.parent/'evaluation.py',
                 HERE.parent/'export_dense_sensor_prior.py'):
        sources.code(code)
    capture_dir, raw_prior = Path(capture_dir).resolve(), Path(raw_prior).resolve()
    capture_path = capture_dir/'capture.json'
    capture = sources.read(capture_path)
    manifest = sources.read(raw_prior/'manifest.json')
    plan_path = Path(plan_path).resolve()
    plan = sources.read(plan_path)
    if (capture.get('failure') is not None or capture['completed_waypoints'] != capture['total_waypoints']
            or manifest.get('capture_complete') is not True
            or manifest.get('ground_truth_used_for_scan_fusion') is not False
            or Path(manifest['source_capture_directory']).resolve() != capture_dir
            or Path(plan['source_capture']).resolve() != capture_path
            or plan.get('GT_used_by_estimator') is not False):
        raise ValueError('source capture/prior/estimator provenance mismatch')
    if digest(capture_path) != manifest['source_sha256']['capture.json']:
        raise ValueError('raw prior belongs to changed capture contents')
    bs = rigid(manifest['T_base_lidar']); ibs = np.linalg.inv(bs)
    if not np.allclose(bs, rigid(plan['base_from_sensor']), rtol=0, atol=1e-12):
        raise ValueError('sensor/base convention differs from raw prior')
    world_camera = rigid(manifest['T_world_camera_init'])
    gt = [(r['stamp'], rigid(r['matrix'])) for r in capture['gt']]
    if not gt or np.any(np.diff([x[0] for x in gt]) <= 0):
        raise ValueError('invalid GT timestamp inventory')
    lio = Trajectory(capture['odom'], maximum_gap=maximum_gap)
    old_dir,old_native,old_artifacts,old_summary,old_native_summary = completed_local(old_local,sources)
    new_dir,new_native,new_artifacts,new_summary,new_native_summary = completed_local(new_local,sources)
    ids = [s['id'] for s in plan['stations']]
    if len(set(ids)) != len(ids) or len(ids) != plan['expected_station_count']:
        raise ValueError('invalid planned visit inventory')
    for summary in (old_summary,new_summary,old_native_summary,new_native_summary):
        if [s['id'] for s in summary['stations']] != ids:
            raise ValueError('local result omitted/reordered planned visit')
    station_summaries, all_rows = [], []
    ns = lambda row: round(row['stamp'] * 1e9)
    for station in plan['stations']:
        sid = station['id']
        op, npth = old_native/sid/'poses.json', new_native/sid/'poses.json'
        old = sources.read(op,old_artifacts[str(op)])
        new = sources.read(npth,new_artifacts[str(npth)])
        stamps = [s['stamp_ns'] for s in station['scans']]
        if (not stamps or len(stamps) != station['expected_raw_frames']
                or [ns(r) for r in old] != stamps or [ns(r) for r in new] != stamps):
            raise ValueError('not all fixed station raw rows have matching recorded poses: '+sid)
        old_first = rigid(old[0]['reference_from_sensor'])
        new_first = rigid(new[0]['reference_from_sensor'])
        if not np.allclose(old_first,new_first,rtol=0,atol=1e-12):
            raise ValueError('old/new versions do not share the same first gauge: '+sid)
        first_truth = interpolate(gt,new[0]['stamp'],maximum_gap)
        first_lio,first_association = lio.at(stamps[0])
        first_accepted = bool(old[0]['accepted'] and new[0]['accepted'])
        relative_gauge = np.linalg.inv(old_first@ibs) if first_accepted else None
        truth_inverse = np.linalg.inv(first_truth) if first_truth is not None else None
        seed = world_camera@first_lio@np.linalg.inv(old_first) if first_lio is not None else None
        lio_gauge = np.linalg.inv(first_lio@ibs) if first_lio is not None else None
        rows = []
        for i,(a,b) in enumerate(zip(old,new)):
            truth = interpolate(gt,b['stamp'],maximum_gap)
            truth_rel = truth_inverse@truth if truth_inverse is not None and truth is not None else None
            old_sensor,new_sensor = rigid(a['reference_from_sensor']),rigid(b['reference_from_sensor'])
            for record,sensor in ((a,old_sensor),(b,new_sensor)):
                if not np.allclose(rigid(record['reference_from_base']),sensor@ibs,rtol=0,atol=1e-8):
                    raise ValueError('incorrect sensor/base pose composition: '+sid)
            old_base,new_base = old_sensor@ibs,new_sensor@ibs
            li,association = lio.at(stamps[i])
            row={'station_id':sid,'ordinal':i,'stamp_ns':stamps[i],'stamp':b['stamp'],
                 'fixed_sensor_role':station['scans'][i]['role'],
                 'old_accepted':bool(a['accepted']),'new_accepted':bool(b['accepted']),
                 'common_accepted':bool(a['accepted'] and b['accepted']),
                 'GT_missing':truth is None,'LIO_missing':li is None,
                 'relative_first_GT_missing':truth_inverse is None,
                 'old_reason':a['reason'],'old_failure_class':a['failure_class'],
                 'old_rejected_gates':a['rejected_gates'],'LIO_association':association,
                 'truth_world_from_base':truth.tolist() if truth is not None else None,
                 'truth_initial_base_from_base':truth_rel.tolist() if truth_rel is not None else None,
                 'old_reference_from_sensor':old_sensor.tolist(),'new_reference_from_sensor':new_sensor.tolist()}
            for name,base,accepted in (('old',old_base,a['accepted']),('new',new_base,b['accepted'])):
                row[name+'_relative_score']=score_or_missing(relative_gauge@base if relative_gauge is not None else None,truth_rel,accepted)
                row[name+'_LIO_seed_global_score']=score_or_missing(seed@base if seed is not None else None,truth,accepted)
            row['original_LIO_relative_score']=score_or_missing(lio_gauge@li@ibs if li is not None and lio_gauge is not None else None,truth_rel)
            row['original_LIO_global_score']=score_or_missing(world_camera@li@ibs if li is not None else None,truth)
            rows.append(row)
        common = [r for r in rows if r['common_accepted']]
        original_accepted_indices = [i for i,r in enumerate(rows) if r['old_accepted']]
        station_summaries.append({'station_id':sid,'source_frames':len(rows),
            'source_capture_frames':station.get('expected_capture_raw_frames'),
            'excluded_settle_prefix_frames':len(station.get('excluded_settle_prefix_scans',[])),
            'old_accepted':sum(r['old_accepted']for r in rows),
            'new_accepted':sum(r['new_accepted']for r in rows),
            'old_accepted_is_contiguous_prefix':original_accepted_indices==list(range(len(original_accepted_indices))),
            'same_first_raw_stamp_ns':stamps[0],'same_first_sensor_pose_verified':True,
            'fixed_first_raw_accepted_by_both':first_accepted,
            'first_LIO_association':first_association,
            'map_from_local_from_LIO_seed':seed.tolist() if seed is not None else None,
            'missing_GT_stamps_ns':[r['stamp_ns']for r in rows if r['GT_missing']],
            'missing_LIO_stamps_ns':[r['stamp_ns']for r in rows if r['LIO_missing']],
            'all_station_frames':summaries(rows),'old_new_common_accepted_frames':summaries(common)})
        all_rows.extend(rows)
    common = [r for r in all_rows if r['common_accepted']]
    result={'schema_version':1,'scope':'independent post-estimation trajectory scoring; no map or query acceptance claim',
        'GT_used_by_estimator':False,'GT_used_to_choose_windows_poses_or_radius':False,
        'expected_station_count':len(ids),'all_station_visits_retained':True,
        'old_local_all_stations_passed':old_summary['all_stations_complete'],
        'new_local_all_stations_passed':new_summary['all_stations_complete'],
        'all_pose_rows_retained':True,'source_pose_rows':len(all_rows),
        'old_accepted_pose_rows':sum(r['old_accepted']for r in all_rows),
        'new_accepted_pose_rows':sum(r['new_accepted']for r in all_rows),
        'GT_and_LIO_interpolation_maximum_bracket_gap_sec':maximum_gap,
        'orientation_metric':'full SO3 geodesic degrees',
        'relative_alignment':'Each station uses its fixed shared first raw base gauge; GT relative motion uses that exact timestamp. No alternate anchor is searched.',
        'relative_aggregate_limit':'Pooled errors across different per-station initial base gauges; this is not one global trajectory or global accuracy.',
        'global_alignment':'All stations inherit the same original T_world_camera_init. Per-station seed uses only original LIO sensor at first raw * inverse(first local sensor); no per-station GT reanchoring.',
        'all_station_frames':summaries(all_rows),
        'old_new_common_accepted_frames':summaries(common),
        'missing_GT_rows':[{'station_id':r['station_id'],'stamp_ns':r['stamp_ns']}for r in all_rows if r['GT_missing']],
        'missing_LIO_rows':[{'station_id':r['station_id'],'stamp_ns':r['stamp_ns']}for r in all_rows if r['LIO_missing']],
        'stations':station_summaries,'source_sha256':sources.hashes}
    sources.verify()
    output.mkdir(parents=True,exist_ok=False)
    for filename,value in [('summary.json',result),('rows.json',all_rows),('sources.json',sources.hashes)]:
        (output/filename).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k]for k in ('expected_station_count','source_pose_rows','old_accepted_pose_rows',
          'new_accepted_pose_rows','all_station_frames','old_new_common_accepted_frames')},indent=2))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',required=True,type=Path)
    parser.add_argument('--raw-prior',required=True,type=Path)
    parser.add_argument('--input',required=True,type=Path)
    parser.add_argument('--old-local',required=True,type=Path)
    parser.add_argument('--new-local',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--maximum-gap',type=float,default=.25)
    args=parser.parse_args()
    evaluate(args.capture,args.raw_prior,args.input,args.old_local,args.new_local,args.output,args.maximum_gap)


if __name__=='__main__':
    main()
