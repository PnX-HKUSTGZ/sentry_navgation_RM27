#!/usr/bin/env python3
"""Fixed3*leaf covariance observation diagnostic; existing15+15/gates unchanged."""
import argparse
import copy
import difflib
import os
from pathlib import Path
import subprocess

from pipeline_common import HERE,REPO,Inputs,new_directory,read_json,write_json,sha256
from pipeline_inputs import local


def run(source_input,station_id,output,resume_contract_build=False):
    inputs=Inputs();source_input=inputs.artifact(source_input);original=read_json(source_input)
    matches=[s for s in original['stations']if s['id']==station_id]
    if len(matches)!=1:raise ValueError('named station must uniquely exist')
    if original['params']['max_iterations']!=15 or original['params']['retry_iterations']!=15:
        raise ValueError('diagnostic requires unchanged original15+15 budgets')
    guard_paths=[REPO/'src/global_relocalization/config/competition.yaml',
        REPO/'tools/global_relocalization/.build/station_submaps/offline_station_submaps',
        REPO/'tools/global_relocalization/.build/ros/install/global_relocalization/lib/libglobal_relocalization_core.so',
        REPO/'tools/global_relocalization/.build/ros/install/global_relocalization/lib/global_relocalization/global_relocalization_node']
    before_guard={str(p):sha256(p)for p in guard_paths}
    if resume_contract_build:
        out=Path(output).resolve()
        if (not (out/'first_build_failure_note.json').is_file() or (out/'input').exists()
                or (out/'local').exists()):
            raise ValueError('resume permitted only after recorded pre-data contract-build failure')
    else:
        out=new_directory(output)
    project=out/'project'
    if not resume_contract_build:project.mkdir()
    build=out/'build'
    for name in ['CMakeLists.txt','generate_frontend.py','offline_station_submaps.cpp',
                 'bounded_covariance.hpp','test_bounded_covariance.cpp']:
        inputs.add(HERE/name)
    inputs.add(Path(__file__))
    project_text=f'''cmake_minimum_required(VERSION 3.16)
project(bounded_covariance_diagnostic)
set(CMAKE_CXX_STANDARD 17)
enable_testing()
add_subdirectory("{HERE}" "${{CMAKE_CURRENT_BINARY_DIR}}/frontend")
add_executable(bounded_covariance_contract "{HERE/'test_bounded_covariance.cpp'}")
target_link_libraries(bounded_covariance_contract PRIVATE experimental_station_frontend)
add_test(NAME bounded_covariance_contract COMMAND bounded_covariance_contract)
'''
    if not resume_contract_build:(project/'CMakeLists.txt').write_text(project_text)
    elif (project/'CMakeLists.txt').read_text()!=project_text:
        raise ValueError('resume project differs from the original single-variable build')
    env=dict(os.environ,OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
    generated=build/'frontend/generated/experimental_frontend.cpp'
    with (out/('contract_build_retry.log'if resume_contract_build else'build.log')).open('w')as log:
        if not resume_contract_build:
            subprocess.run(['cmake','-S',str(project),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],
                env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        before=(out/'generated_before.cpp').read_text()if resume_contract_build else generated.read_text()
        after=before.replace('#include "experimental_frontend.hpp"',
            '#include "experimental_frontend.hpp"\n#include "'+str(HERE/'bounded_covariance.hpp')+'"')
        for name in ['target','source']:
            stock=f'small_gicp::estimate_covariances_omp(*{name},20,threads)'
            bounded=f'station_diagnostic::estimate_bounded_covariances(*{name},3.*i.p.leaf_size,20,threads)'
            if after.count(stock)!=1:raise ValueError('covariance source anchor not unique')
            after=after.replace(stock,bounded)
        if 'std::min(30,i.p.max_iterations)'not in after or 'std::max(0,30-first_budget)'not in after:
            raise ValueError('original budget cap unexpectedly changed')
        if not resume_contract_build:
            (out/'generated_before.cpp').write_text(before);generated.write_text(after)
            (out/'only_covariance_patch.diff').write_text(''.join(difflib.unified_diff(
                before.splitlines(True),after.splitlines(True),fromfile='stock.cpp',tofile='bounded_covariance.cpp')))
        elif generated.read_text()!=after:
            raise ValueError('compiled covariance source changed during test-only build repair')
        command=['cmake','--build',str(build),'-j2']
        if resume_contract_build:command+=['--target','bounded_covariance_contract']
        subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        if resume_contract_build:
            note=read_json(out/'first_build_failure_note.json')
            if sha256(build/'frontend/offline_station_submaps')!=note['native_sha256_before_fix']:
                raise ValueError('native unexpectedly changed during contract-only rebuild')
    with (out/'contracts.log').open('w')as log:
        subprocess.run(['ctest','--test-dir',str(build),'--output-on-failure','-R','bounded_covariance_contract'],
            env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    selected=copy.deepcopy(original);selected['stations']=copy.deepcopy(matches)
    selected['expected_station_count']=1;selected['all_station_windows_retained']=False
    selected['diagnostic_only']=True
    selected['diagnostic_scope']={'source_input':str(source_input),'source_sha256':sha256(source_input),
        'station':station_id,'all_original_station_frames_retained':True,'only_model_delta':'stock covariance K20 bounded to3*leaf',
        'covariance_radius_m':3*original['params']['leaf_size'],'registration_max_correspondence_unchanged':original['params']['max_correspondence'],
        'PCA_regularization_fallback_iteration_budget_gates_unchanged':True,'GT_query_or_postscore_read':False,
        'production_or_fullgraph_adoption':False}
    input_dir=out/'input';input_dir.mkdir();write_json(input_dir/'input.json',selected)
    cal=inputs.artifact(source_input.parent/'calibration_protocol_check.json')
    (input_dir/'calibration_protocol_check.json').write_bytes(cal.read_bytes())
    upstream=read_json(inputs.add(source_input.parent/'provenance.json'))
    artifacts=dict(upstream['artifact_sha256'])
    for path in [input_dir/'input.json',input_dir/'calibration_protocol_check.json']:
        artifacts[str(path)]=sha256(path)
    inputs.verify();write_json(input_dir/'provenance.json',{'sensor_only':True,'diagnostic_only':True,
        'stage':'bounded covariance input','source_sha256':inputs.hashes,'artifact_sha256':artifacts})
    native=build/'frontend/offline_station_submaps'
    passed=local(input_dir/'input.json',native,out/'local')
    poses=read_json(out/'local/native'/station_id/'poses.json');failed=[p for p in poses if not p['accepted']]
    fields=['stamp','reason','failure_class','fitness','inlier_ratio','geometry_eigen_ratio','information_eigen_ratio',
            'translation_step','linear_speed','rotation_prior_error','rejected_gates','registration_attempts']
    unchanged=all(sha256(Path(path))==digest for path,digest in before_guard.items())
    report={'diagnostic_only':True,'observation_model':'fixed3*leaf radius for covariance K20 only',
        'covariance_radius_m':3*original['params']['leaf_size'],'registration_correspondence_m':original['params']['max_correspondence'],
        'original15plus15_budget_unchanged':True,'all_contracts_passed':True,'station':station_id,
        'input_frames':len(matches[0]['scans']),'pose_records':len(poses),'accepted':sum(p['accepted']for p in poses),
        'local_complete':passed,'first_failure':{k:failed[0].get(k)for k in fields}if failed else None,
        'all_original_station_frames_retained':len(poses)==len(matches[0]['scans']),
        'no_new_calibration_window_or_warm_start':True,'native_sha256':sha256(native),
        'generated_source_sha256':sha256(generated),'bounded_header_sha256':sha256(HERE/'bounded_covariance.hpp'),
        'production_and_original_native_unchanged':unchanged,'production_guard_sha256':before_guard,
        'GT_or_query_used':False,'map_or_graph_exported':False,
        'full34_or_historical_raw_regression_not_run':True,'global_alias_or100percent_success_claim':False}
    write_json(out/'summary.json',report)
    code=out/'code';code.mkdir()
    for path in [Path(__file__),HERE/'bounded_covariance.hpp',HERE/'test_bounded_covariance.cpp']:
        (code/path.name).write_bytes(path.read_bytes())
    inputs.save(out,stage='single bounded-covariance model diagnostic',local_complete=passed)
    print(__import__('json').dumps(report,indent=2),flush=True)
    return passed and unchanged


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-input',type=Path,required=True)
    p.add_argument('--station-id',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume-contract-build',action='store_true');a=p.parse_args()
    raise SystemExit(0 if run(a.source_input,a.station_id,a.output,a.resume_contract_build)else 1)
