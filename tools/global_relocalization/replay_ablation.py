#!/usr/bin/env python3
"""Freeze and compare numerical raw-frontend replay modes without GT input.

Exit code 0 from replay means bitwise repetition identity, not localization
success. Acceptance, hard faults, and output-pose spread are reported separately.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
MODES={
    'baseline': [],
    'filter_only': ['range_filter=true'],
    'det_only': ['deterministic=true'],
    'filter_det': ['range_filter=true','deterministic=true'],
    'filter_det_submap': ['range_filter=true','deterministic=true','submap_scans=5'],
    'filter_det_prior': ['range_filter=true','deterministic=true','imu_rotation_prior=true'],
    'combined': ['range_filter=true','deterministic=true','submap_scans=5','imu_rotation_prior=true','numerical_recovery=true'],
}

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,data):path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')

def summarize(report):
    runs=report['runs'];firsts={};reasons={};max_translation=0.;max_rotation=0.
    reference=runs[0]['scans']
    for run in runs:
        failure=run.get('first_hard_rejection');key='none' if failure is None else str(failure['stamp'])
        firsts[key]=firsts.get(key,0)+1
        for scan,base in zip(run['scans'],reference):
            if not scan['accepted']:reasons[scan['reason']]=reasons.get(scan['reason'],0)+1
            a=np.asarray(scan['pose']).reshape(4,4);b=np.asarray(base['pose']).reshape(4,4)
            max_translation=max(max_translation,float(np.linalg.norm(a[:3,3]-b[:3,3])))
            c=float(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1))
            max_rotation=max(max_rotation,float(np.degrees(np.arccos(c))))
    return {'repetitions':len(runs),'bitwise_identical':report['bitwise_identical'],
        'no_hard_rejection_count':sum(r.get('first_hard_rejection') is None for r in runs),
        'first_hard_rejection_stamp_counts':firsts,'rejected_scan_reason_counts':reasons,
        'accepted_scan_count_range':[min(r['accepted'] for r in runs),max(r['accepted'] for r in runs)],
        'last_accepted_cumulative_yaw_range':[min(r['last_accepted_cumulative_yaw'] for r in runs),max(r['last_accepted_cumulative_yaw'] for r in runs)],
        'maximum_sensor_pose_translation_spread_from_first_repetition_m':max_translation,
        'maximum_sensor_pose_rotation_spread_from_first_repetition_deg':max_rotation,
        'global_localization_success':'not evaluated: numerical scan replay only; captured sequence can end before a full turn'}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',action='append',required=True,help='LABEL=directory of exported scan PCD/JSON; repeat for inputs')
    p.add_argument('--output',required=True)
    p.add_argument('--repetitions',type=int,default=20)
    p.add_argument('--modes',default='baseline,filter_only,combined')
    a=p.parse_args();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    from ament_index_python.packages import get_package_prefix
    prefix=Path(get_package_prefix('global_relocalization'));snap=out/'frozen';(snap/'lib').mkdir(parents=True)
    exe=prefix/'lib/global_relocalization/global_relocalization_raw_replay';core=prefix/'lib/libglobal_relocalization_core.so'
    shutil.copy2(exe,snap/exe.name);shutil.copy2(core,snap/'lib'/core.name)
    sources=list((ROOT/'src/global_relocalization/src').glob('*.cpp'))+list((ROOT/'src/global_relocalization/include/global_relocalization').glob('*.hpp'))
    for path in sources:
        target=snap/'source'/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    evidence={'numerical_only':True,'ground_truth_used':False,'replays_receipt_latency':False,
        'core_sha256':digest(core),'executable_sha256':digest(exe),'source_sha256':{str(p.relative_to(ROOT)):digest(p) for p in sources},'runs':[]}
    save(out/'summary.json',evidence)
    env=dict(os.environ);env['LD_LIBRARY_PATH']=str(snap/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    for item in a.input:
        label,path=item.split('=',1);source=Path(path).resolve();dest=out/label;shutil.copytree(source,dest/'input')
        save(dest/'input_manifest.json',{'source':str(source),'sha256':{p.name:digest(p) for p in source.iterdir() if p.is_file()}})
        for mode in a.modes.split(','):
            if mode not in MODES:p.error('unknown mode '+mode)
            command=[str(snap/exe.name),str(dest/'input'),str(a.repetitions)]+MODES[mode];started=time.monotonic()
            with (dest/(mode+'.json')).open('w') as stream,(dest/(mode+'.log')).open('w') as log:
                proc=subprocess.Popen(command,stdout=stream,stderr=log,env=env)
                time.sleep(.05)
                import psutil
                try:
                    libraries=[m.path for m in psutil.Process(proc.pid).memory_maps() if m.path.endswith('libglobal_relocalization_core.so')]
                except psutil.NoSuchProcess:libraries=[]
                code=proc.wait()
            loaded={p:digest(p) for p in libraries}
            if not loaded or any(h!=evidence['core_sha256'] for h in loaded.values()):raise RuntimeError('loaded core library is not the frozen snapshot')
            data=json.loads((dest/(mode+'.json')).read_text());summary=summarize(data)
            row={'input':label,'mode':mode,'command':command,'returncode':code,'wall_sec':time.monotonic()-started,'loaded_core_sha256':loaded,**summary};evidence['runs'].append(row);save(out/'summary.json',evidence);print(json.dumps({'input':label,'mode':mode,'bitwise_identical':summary['bitwise_identical'],'no_hard_rejection_count':summary['no_hard_rejection_count'],'repetitions':summary['repetitions']}),flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
