#!/usr/bin/env python3
"""One predeclared 50-iteration diagnostic, isolated from the frozen pipeline.

Regenerate the exact experimental frontend into a new build, replace only its
two total-budget clamps30->50, and replay a complete named station from its
original first frame. No production files or existing artifacts are changed.
This is not a passing map/graph or a budget sweep.
"""
import argparse
import copy
import difflib
import os
from pathlib import Path
import subprocess

from pipeline_common import HERE, Inputs, new_directory, read_json, write_json, sha256
from pipeline_inputs import local


def run(source_input, station_id, output):
    inputs = Inputs()
    source_input = inputs.artifact(source_input)
    original = read_json(source_input)
    matches = [s for s in original['stations'] if s['id'] == station_id]
    if len(matches) != 1:
        raise ValueError('diagnostic station must uniquely exist in original input')
    out = new_directory(output)
    inputs.add(Path(__file__))
    for name in ['CMakeLists.txt','generate_frontend.py','offline_station_submaps.cpp']:
        inputs.add(HERE/name)
    build = out/'build'
    environment = dict(os.environ,OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
    with (out/'build.log').open('w') as log:
        subprocess.run(['cmake','-S',str(HERE),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],
                       env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
        generated = build/'generated/experimental_frontend.cpp'
        before = generated.read_text()
        edits = [('std::min(30,i.p.max_iterations)','std::min(50,i.p.max_iterations)'),
                 ('std::max(0,30-first_budget)','std::max(0,50-first_budget)')]
        after = before
        for old,new in edits:
            if after.count(old) != 1:
                raise ValueError('isolated budget source anchor not unique')
            after = after.replace(old,new)
        (out/'generated_before.cpp').write_text(before)
        generated.write_text(after)
        (out/'only_budget_patch.diff').write_text(''.join(difflib.unified_diff(
            before.splitlines(True),after.splitlines(True),fromfile='before.cpp',tofile='iteration50.cpp')))
        subprocess.run(['cmake','--build',str(build),'-j2'],env=environment,
                       stdout=log,stderr=subprocess.STDOUT,check=True)
    selected = copy.deepcopy(original)
    selected['stations'] = copy.deepcopy(matches)
    selected['expected_station_count'] = 1
    selected['all_station_windows_retained'] = False
    selected['params']['max_iterations'] = 50
    selected['diagnostic_only'] = True
    selected['diagnostic_scope'] = {
        'source_input':str(source_input),'source_input_sha256':sha256(source_input),
        'station':station_id,'all_original_station_frames_retained':True,
        'GT_query_or_postscore_read':False,'thresholds_priors_or_gates_changed':False,
        'only_estimator_delta':'max_iterations50 + isolated total-budget clamps50',
        'actual_attempt_schedule':'first=min(50,50)=50; retry=min(original15,50-first)=0',
        'production_or_fullgraph_adoption':False}
    # The immutable raw/IMU/ledger artifacts are passed through, not copied or
    # rewritten. Their original completed-stage provenance remains linked.
    input_dir = out/'input'
    input_dir.mkdir()
    write_json(input_dir/'input.json',selected)
    calibration = inputs.artifact(source_input.parent/'calibration_protocol_check.json')
    (input_dir/'calibration_protocol_check.json').write_bytes(calibration.read_bytes())
    prior = read_json(inputs.add(source_input.parent/'provenance.json'))
    passed_through = dict(prior['artifact_sha256'])
    passed_through[str(input_dir/'input.json')] = sha256(input_dir/'input.json')
    passed_through[str(input_dir/'calibration_protocol_check.json')] = sha256(input_dir/'calibration_protocol_check.json')
    inputs.verify()
    write_json(input_dir/'provenance.json',{'sensor_only':True,'stage':'fixed50 diagnostic input',
        'source_sha256':inputs.hashes,'artifact_sha256':passed_through,'diagnostic_only':True})
    passed = local(input_dir/'input.json',build/'offline_station_submaps',out/'local')
    summary = read_json(out/'local/summary.json')
    poses = read_json(out/'local/native'/station_id/'poses.json')
    failures = [p for p in poses if not p['accepted']]
    first = failures[0] if failures else None
    metrics = ['stamp','reason','failure_class','fitness','inlier_ratio','geometry_eigen_ratio',
               'information_eigen_ratio','translation_step','linear_speed','rotation_prior_error',
               'rejected_gates','registration_attempts']
    report = {'diagnostic_only':True,'source_station':station_id,
        'original_frames':len(matches[0]['scans']),'pose_records':len(poses),
        'accepted':sum(p['accepted']for p in poses),'local_complete':passed,
        'first_failure':{k:first.get(k)for k in metrics}if first else None,
        'actual_iterations_at_first_failure':first.get('registration_attempts')if first else None,
        'all_original_station_frames_retained':len(poses)==len(matches[0]['scans']),
        'no_new_calibration_window_or_warm_start':True,
        'native_sha256':sha256(build/'offline_station_submaps'),
        'isolated_generated_source_sha256':sha256(generated),
        'source_generated_before_sha256':sha256(out/'generated_before.cpp'),
        'production_unchanged':True,'map_or_graph_exported':False,'GT_or_query_used':False,
        'result_is_not_a_full34station_regression':True}
    write_json(out/'summary.json',report)
    inputs.save(out,stage='fixed50 numerical diagnostic',local_complete=passed)
    print(__import__('json').dumps(report,indent=2),flush=True)
    return passed


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-input',type=Path,required=True)
    parser.add_argument('--station-id',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.source_input,args.station_id,args.output)else 1)
