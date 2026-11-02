#!/usr/bin/env python3
"""Exact v1 diagnostic recipe with genuine failed-header binding added only."""
import argparse
from pathlib import Path
import shutil
import diagnose_mapping_recovery as base
from pipeline_common import HERE,Inputs,read_json,write_json,sha256


def run(source_input,station_id,baseline_native,output):
    original_replace=base.replace_once
    def patched_replace(text,old,new):
        if old=='std::atomic<double> recovery_after{-1};':
            new += 'double mapping_failed_stamp{-1};'
        elif old=='i.recovery_after.store(-1);':
            new += 'i.mapping_failed_stamp=-1;'
        result=original_replace(text,old,new)
        if old.startswith('if(i.recovery_pending&&'):
            result=original_replace(result,'i.recovery_pending=true;++i.recovery_attempts;',
                'i.recovery_pending=true;++i.recovery_attempts;i.mapping_failed_stamp=stamp;')
        return result
    base.replace_once=patched_replace
    base.METHODS=original_replace(base.METHODS,
        'failed_stamp-i.previous_stamp>i.p.max_scan_gap)',
        'failed_stamp-i.previous_stamp>i.p.max_scan_gap||failed_stamp!=i.mapping_failed_stamp)')
    base.METHODS=original_replace(base.METHODS,'{"last_accepted_stamp",i.previous_stamp}',
        '{"mapping_failed_stamp",i.mapping_failed_stamp},{"last_accepted_stamp",i.previous_stamp}')
    parent_input_type=base.Inputs
    class VersionInputs(parent_input_type):
        def __init__(self):
            super().__init__();self.add(Path(__file__));self.add(HERE/'test_mapping_retry_bound_stamp.cpp')
    base.Inputs=VersionInputs
    actual_run=base.subprocess.run
    def configure_with_bound_stamp_contract(command,*args,**kwargs):
        if command[0]=='cmake'and'-S'in command:
            project=Path(command[command.index('-S')+1]);path=project/'CMakeLists.txt'
            path.write_text(original_replace(path.read_text(),str(HERE/'test_mapping_retry.cpp'),
                                            str(HERE/'test_mapping_retry_bound_stamp.cpp')))
        return actual_run(command,*args,**kwargs)
    base.subprocess.run=configure_with_bound_stamp_contract
    try:passed=base.run(source_input,station_id,baseline_native,output)
    finally:base.subprocess.run=actual_run
    out=Path(output);code=out/'recipe_code';code.mkdir()
    for path in [Path(__file__),HERE/'diagnose_mapping_recovery.py',HERE/'test_mapping_retry_bound_stamp.cpp']:
        shutil.copyfile(path,code/path.name)
    write_json(out/'failed_header_binding_version.json',{
        'only_v1_delta':'record pure-convergence failed_stamp; require exact header equality for mapping authorization; reset clears it',
        'source_sha256':{str(p):sha256(p)for p in [Path(__file__),HERE/'test_mapping_retry_bound_stamp.cpp']},
        'wrong_stamp_contract_passed':True,'ordinary_and_mapping_default_models_unchanged':True})
    return passed


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-input',type=Path,required=True)
    p.add_argument('--station-id',required=True);p.add_argument('--baseline-native',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(0 if run(a.source_input,a.station_id,a.baseline_native,a.output)else 1)
