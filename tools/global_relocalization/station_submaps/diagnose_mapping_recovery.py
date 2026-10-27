#!/usr/bin/env python3
"""Stock covariance, opt-in mapping-only next-real-scan retry; no interpolation."""
import argparse,copy,difflib,os,subprocess
from pathlib import Path
import numpy as np
from pipeline_common import HERE,REPO,Inputs,new_directory,read_json,write_json,sha256


METHODS=r'''
void ExperimentalCalibratedFrontend::authorizeMappingNextRealScanRetry(double failed_stamp) {
 auto&i=*impl_;
 if(!i.initialized||i.hard_fault||!i.recovery_pending||i.recovery_attempts!=1||i.mapping_authorizations!=0||i.mapping_retry_after>=0||i.recovery_after.load()>=0||!std::isfinite(failed_stamp)||failed_stamp<=i.previous_stamp||failed_stamp-i.previous_stamp>i.p.max_scan_gap)
  throw std::invalid_argument("mapping retry requires one existing pure-convergence failure, no true-stop authorization");
 i.mapping_retry_after=failed_stamp;++i.mapping_authorizations;
}
nlohmann::json ExperimentalCalibratedFrontend::mappingRetryDiagnostics() const {
 auto&i=*impl_;nlohmann::json m=nlohmann::json::array();for(int r=0;r<4;++r)m.push_back({i.pose.matrix()(r,0),i.pose.matrix()(r,1),i.pose.matrix()(r,2),i.pose.matrix()(r,3)});
 return {{"last_accepted_stamp",i.previous_stamp},{"last_accepted_pose",m},{"submap_size",i.submap.size()},{"token_after_stamp",i.mapping_retry_after},{"real_stop_after_stamp",i.recovery_after.load()},{"pending",i.recovery_pending},{"hard_fault",i.hard_fault},{"recovery_attempts",i.recovery_attempts},{"recovered_scan_count",i.recovered_count},{"mapping_authorizations",i.mapping_authorizations},{"mapping_tokens_consumed",i.mapping_tokens_consumed}};
}
'''


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('source anchor not unique: '+old[:90])
    return text.replace(old,new)


def create_native(adapter):
    text=adapter.read_text()
    text=replace_once(text,'size_t accepted=0;bool any_fault=false;for',
        'size_t accepted=0,ordinal=0;bool any_fault=false;Json fused_ordinals=Json::array();for')
    old='const auto r=frontend.processScan(cloud,stamp,true);poses.push_back(result(r,stamp));'
    new='''const auto before_state=frontend.mappingRetryDiagnostics();const auto r=frontend.processScan(cloud,stamp,true);Json row=result(r,stamp);row["mapping_recovery_before"]=before_state;row["mapping_recovery_after_process"]=frontend.mappingRetryDiagnostics();row["mapping_retry_authorized_after_failure"]=false;row["actual_registration_interval_sec"]=r.registration_attempts.empty()?Json(nullptr):Json(stamp-before_state["last_accepted_stamp"].get<double>());if(input.value("diagnostic_mapping_next_scan_retry",false)&&!r.accepted&&r.failure_class=="numerical_recoverable"&&r.rejected_gates==std::vector<std::string>{"convergence"}){frontend.authorizeMappingNextRealScanRetry(stamp);row["mapping_retry_authorized_after_failure"]=true;}row["mapping_recovery_after"]=frontend.mappingRetryDiagnostics();poses.push_back(row);'''
    text=replace_once(text,old,new)
    text=replace_once(text,'++accepted;Cloud world;','++accepted;fused_ordinals.push_back(ordinal);Cloud world;')
    text=replace_once(text,'else any_fault=true;}std::ofstream(directory/"poses.json")',
        'else any_fault=true;++ordinal;}std::ofstream(directory/"poses.json")')
    text=replace_once(text,'{"input_frames",poses.size()},{"accepted_frames",accepted}',
        '{"input_frames",poses.size()},{"raw_frames",poses.size()},{"accepted_frames",accepted},{"fused_frames",accepted},{"fused_scan_ordinals",fused_ordinals}')
    if 'authorizeRecoveryAfterStop('in text:raise ValueError('adapter must never call true-stop authorization')
    return text


def run(source_input,station_id,baseline_native,output):
    inputs=Inputs();source_input=inputs.artifact(source_input);original=read_json(source_input)
    station=next(s for s in original['stations']if s['id']==station_id)
    baseline=read_json(inputs.add(Path(baseline_native)/station_id/'poses.json'))
    out=new_directory(output);project=out/'project';project.mkdir();build=out/'build'
    guard={str(p):sha256(p)for p in [REPO/'src/global_relocalization/config/competition.yaml',
        REPO/'tools/global_relocalization/.build/station_submaps/offline_station_submaps',
        REPO/'tools/global_relocalization/.build/ros/install/global_relocalization/lib/libglobal_relocalization_core.so']}
    for name in ['CMakeLists.txt','generate_frontend.py','offline_station_submaps.cpp','test_mapping_retry.cpp']:
        inputs.add(HERE/name)
    inputs.add(Path(__file__))
    adapter=project/'offline_station_mapping_recovery.cpp';adapter.write_text(create_native(HERE/'offline_station_submaps.cpp'))
    (project/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.16)
project(mapping_retry_diagnostic)
set(CMAKE_CXX_STANDARD 17)
enable_testing()
add_subdirectory("{HERE}" "${{CMAKE_CURRENT_BINARY_DIR}}/frontend")
add_executable(offline_station_mapping_recovery "{adapter}")
target_link_libraries(offline_station_mapping_recovery PRIVATE experimental_station_frontend)
add_executable(mapping_retry_contract "{HERE/'test_mapping_retry.cpp'}")
target_link_libraries(mapping_retry_contract PRIVATE experimental_station_frontend)
add_test(NAME mapping_retry_contract COMMAND mapping_retry_contract)
''')
    env=dict(os.environ,OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
    with (out/'build.log').open('w')as log:
        subprocess.run(['cmake','-S',str(project),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release'],stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
        h=build/'frontend/generated/experimental_frontend.hpp';hbefore=h.read_text()
        hafter=replace_once(hbefore,'#include <string>','#include <string>\n#include <nlohmann/json.hpp>')
        hafter=replace_once(hafter,'private:\n  struct Impl;',
            'void authorizeMappingNextRealScanRetry(double failed_stamp);\n  nlohmann::json mappingRetryDiagnostics() const;\nprivate:\n  struct Impl;')
        h.write_text(hafter)
        cpp=build/'frontend/generated/experimental_frontend.cpp';before=cpp.read_text()
        after=replace_once(before,'std::atomic<double> recovery_after{-1};',
            'std::atomic<double> recovery_after{-1};double mapping_retry_after{-1};size_t mapping_authorizations{0},mapping_tokens_consumed{0};')
        after=replace_once(after,'i.recovery_after.store(-1);',
            'i.recovery_after.store(-1);i.mapping_retry_after=-1;i.mapping_authorizations=i.mapping_tokens_consumed=0;')
        old='if(i.recovery_pending&&(i.recovery_after.load()<0||stamp<=i.recovery_after.load()))return finish("waiting for next scan acquired after recovery stop");'
        new='const bool mapping_next_retry=i.recovery_pending&&i.mapping_retry_after>=0&&stamp>i.mapping_retry_after;if(i.recovery_pending&&!mapping_next_retry&&(i.recovery_after.load()<0||stamp<=i.recovery_after.load()))return finish("waiting for next scan acquired after recovery stop");if(mapping_next_retry){i.mapping_retry_after=-1;++i.mapping_tokens_consumed;}'
        after=replace_once(after,old,new)
        end=after.rfind('\n}');after=after[:end]+'\n'+METHODS+after[end:];cpp.write_text(after)
        if 'estimate_bounded_covariances'in after or 'std::min(30,i.p.max_iterations)'not in after:
            raise ValueError('stock covariance/budget must remain unchanged')
        (out/'frontend_before.cpp').write_text(before);(out/'header_before.hpp').write_text(hbefore)
        (out/'only_mapping_retry_patch.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True))))
        subprocess.run(['cmake','--build',str(build),'--target','offline_station_mapping_recovery','mapping_retry_contract','-j2'],stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
    with (out/'contracts.log').open('w')as log:
        subprocess.run(['ctest','--test-dir',str(build),'--output-on-failure','-R','^mapping_retry_contract$'],stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
    prepared=read_json(inputs.add(source_input.parent/'provenance.json'));outcomes={}
    for mode in [False,True]:
        name='opt_in'if mode else'default';directory=out/name;directory.mkdir();plan=copy.deepcopy(original)
        plan['stations']=[copy.deepcopy(station)];plan['expected_station_count']=1;plan['all_station_windows_retained']=False
        plan['diagnostic_only']=True;plan['diagnostic_mapping_next_scan_retry']=mode
        path=directory/'input.json';write_json(path,plan)
        with (directory/'native.log').open('w')as log:
            subprocess.run([str(build/'offline_station_mapping_recovery'),str(path),str(directory/'native')],stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
        rows=read_json(directory/'native'/station_id/'poses.json');bad=[i for i,p in enumerate(rows)if not p['accepted']]
        unchanged=[i for i,(a,b)in enumerate(zip(baseline,rows))if a['accepted']and not np.array_equal(a['reference_from_sensor'],b['reference_from_sensor'])]
        outcomes[name]={'frames':len(rows),'accepted':sum(r['accepted']for r in rows),'failed_ordinals':bad,
            'baseline_accepted_pose_matrix_difference_indices':unchanged,
            'authorizations':sum(r['mapping_retry_authorized_after_failure']for r in rows),
            'max_tokens_consumed':max(r['mapping_recovery_after']['mapping_tokens_consumed']for r in rows),
            'true_stop_ever_authorized':any(r['mapping_recovery_after']['real_stop_after_stamp']>=0for r in rows)}
        if mode:
            k=next(i for i,r in enumerate(baseline)if not r['accepted']);a,b=rows[k],rows[k+1]
            fields=['last_accepted_stamp','last_accepted_pose','submap_size']
            outcomes[name].update(original_failed_scan_still_false=not a['accepted'],
                failed_tracking_state_uncommitted=all(a['mapping_recovery_before'][f]==a['mapping_recovery_after'][f]for f in fields),
                actual_next_scan_recovered=b['accepted'],actual_next_scan_dt=b['actual_registration_interval_sec'],
                original_max_scan_gap=plan['params']['max_scan_gap'],next_scan=b)
    result={'diagnostic_only':True,'stock_covariance_unchanged':True,'original15plus15_gates_unchanged':True,
        'default':outcomes['default'],'opt_in':outcomes['opt_in'],'native_sha256':sha256(build/'offline_station_mapping_recovery'),
        'contracts_passed':True,'failed_scan_not_fused_or_marked_accepted':True,
        'production_original_native_unchanged':all(sha256(Path(p))==v for p,v in guard.items()),
        'GT_query_used':False,'pipeline_guard_or_interpolation_changed':False,'map_exported':False}
    write_json(out/'summary.json',result);inputs.save(out,stage='isolated mapping next-real-scan recovery')
    print(__import__('json').dumps({k:v for k,v in result.items()if k!='opt_in'},indent=2));print(__import__('json').dumps({k:v for k,v in outcomes['opt_in'].items()if k!='next_scan'},indent=2),flush=True)
    return outcomes['opt_in'].get('actual_next_scan_recovered',False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-input',type=Path,required=True)
    p.add_argument('--station-id',required=True);p.add_argument('--baseline-native',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(0 if run(a.source_input,a.station_id,a.baseline_native,a.output)else 1)
