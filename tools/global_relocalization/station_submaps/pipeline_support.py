"""Explicit sensor-only pose support; rejected registration rows stay rejected.

An isolated pure-convergence failure may receive a heldout-only IMU bridge.
Neither its candidate pose nor its raw points enter registration, fusion or
training. This stage does not run a frontend or use a cross-station target.
"""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from pipeline_common import Inputs, new_directory, read_json, write_json, rigid, record_code
from gyro_turns import check_imu


SUPPORT_RULES = {
    'schema': 'isolated_mapping_imu_bridge_v1',
    'maximum_consecutive_failed_scans': 1,
    'maximum_bracket_gap_sec': .25,
    'maximum_imu_gap_sec': .035,
    'maximum_gyro_innovation_rad': .08,
    'maximum_base_translation_step_m': .12,
    'maximum_base_speed_m_sec': .35,
    'eligible_failure_class': 'numerical_recoverable',
    'eligible_rejected_gates': ['convergence'],
    'orientation': 'actual timestamp gyro, corrected by fractional full-SO3 accepted-endpoint closure',
    'translation': 'linear accepted-endpoint base position; sensor lever arm reapplied afterwards',
    'failed_candidate_pose_used': False,
    'bridge_eligible_for_fusion': False,
    'bridge_eligible_for_training': False,
    'heldout': 'all original heldout raw scans/points retained; same seed-frozen domain and fixed denominators',
}


def stamp_ns(row):
    return row['stamp_ns'] if 'stamp_ns' in row else round(row['stamp'] * 1e9)


def pure_convergence(row):
    return (row.get('accepted') is False and row.get('failure_class') == 'numerical_recoverable'
            and row.get('rejected_gates') == ['convergence'])


def accepted_pose(row, base_from_sensor):
    if row.get('accepted') is not True:
        raise ValueError('a real accepted pose is required')
    sensor = rigid(row['reference_from_sensor'])
    base = rigid(row['reference_from_base'])
    if not np.allclose(base, sensor @ np.linalg.inv(base_from_sensor), rtol=0, atol=1e-8):
        raise ValueError('accepted base/sensor composition mismatch')
    return sensor, base


def audit_recovery(left, failed, right):
    """Require a real one-token next-scan recovery without failed-state commit."""
    if not pure_convergence(failed) or failed.get('mapping_retry_authorized_after_failure') is not True:
        raise ValueError('only an explicitly authorized pure-convergence recovery is supported')
    before, after = failed['mapping_recovery_before'], failed['mapping_recovery_after']
    rb, ra = right['mapping_recovery_before'], right['mapping_recovery_after']
    if any(c['hard_fault'] for c in (before, after, rb, ra)):
        raise ValueError('hard fault cannot receive pose support')
    for key in ('last_accepted_stamp', 'last_accepted_pose', 'submap_size', 'real_stop_after_stamp'):
        if before[key] != after[key]:
            raise ValueError('failed registration committed state or invoked a real-stop recovery')
    if (abs(before['last_accepted_stamp'] - left['stamp']) > 1e-9
            or not after['pending'] or not rb['pending'] or ra['pending']
            or abs(after['token_after_stamp'] - failed['stamp']) > 1e-9
            or abs(rb['token_after_stamp'] - failed['stamp']) > 1e-9
            or abs(ra['last_accepted_stamp'] - right['stamp']) > 1e-9
            or ra['real_stop_after_stamp'] != before['real_stop_after_stamp']
            or after['mapping_authorizations'] != before['mapping_authorizations'] + 1
            or ra['mapping_authorizations'] != after['mapping_authorizations']
            or ra['mapping_tokens_consumed'] != before['mapping_tokens_consumed'] + 1
            or ra['recovered_scan_count'] != before['recovered_scan_count'] + 1
            or ra['recovery_attempts'] != before['recovery_attempts'] + 1):
        raise ValueError('not exactly one bounded next-real-scan mapping recovery')
    if abs(right['actual_registration_interval_sec'] - (right['stamp']-left['stamp'])) > 1e-9:
        raise ValueError('recovery did not use the actual accepted-to-accepted time interval')
    if not np.allclose(rigid(before['last_accepted_pose']), rigid(left['reference_from_sensor']), rtol=0, atol=1e-8):
        raise ValueError('mapping recovery changed its last accepted anchor')
    if not np.allclose(rigid(ra['last_accepted_pose']), rigid(right['reference_from_sensor']), rtol=0, atol=1e-8):
        raise ValueError('mapping recovery did not commit its real accepted endpoint')


def audit_station(station, rows, summary, base_from_sensor):
    """Inventory and genuine registration/fusion counts, without using failures' poses."""
    if (len(rows) != station['expected_raw_frames'] or len(rows) != len(station['scans'])
            or any(abs(round(r['stamp']*1e9)-s['stamp_ns'])>1 for r,s in zip(rows,station['scans']))
            or summary['input_frames'] != len(rows)):
        raise ValueError('raw/registration complete inventory mismatch')
    indices = [i for i,r in enumerate(rows) if r.get('accepted') is True]
    if (summary['accepted_frames'] != len(indices) or summary['fused_frames'] != len(indices)
            or summary['fused_scan_ordinals'] != indices):
        raise ValueError('native cloud must contain exactly its real accepted scans')
    for i,r in enumerate(rows):
        if r.get('accepted') is True:
            accepted_pose(r, base_from_sensor)
        elif (i == 0 or i == len(rows)-1 or not rows[i-1].get('accepted')
              or not rows[i+1].get('accepted')):
            raise ValueError('only an isolated nonboundary failed scan can be supported')
        else:
            audit_recovery(rows[i-1],r,rows[i+1])
    return indices


def gyro_deltas(times, gyro, bias, left, middle, right):
    if left < times[0] or right > times[-1] or not left < middle < right:
        raise ValueError('bridge lacks complete actual IMU brackets')
    use = (times >= left) & (times <= right)
    # Include the enclosing measurements as well as interpolated exact endpoints.
    a=max(0,int(np.searchsorted(times,left))-1)
    b=min(len(times),int(np.searchsorted(times,right,side='right'))+1)
    if np.max(np.diff(times[a:b])) > 35_000_000:
        raise ValueError('bridge IMU gap exceeds the original sensor limit')
    stamps=np.unique(np.r_[left,middle,right,times[use]])
    x=(times-times[0])*1e-9
    query=(stamps-times[0])*1e-9
    w=np.column_stack([np.interp(query,x,gyro[:,k])for k in range(3)])-bias
    current=np.eye(3); at_middle=None
    for i in range(len(stamps)-1):
        angle=.5*(w[i]+w[i+1])*((int(stamps[i+1])-int(stamps[i]))*1e-9)
        current=current @ Rotation.from_rotvec(angle).as_matrix()
        if stamps[i+1] == middle:
            at_middle=current.copy()
    return at_middle,current


def bridge_pose(left, failed_stamp_ns, right, base_from_sensor, times, gyro, bias):
    """Actual gyro + accepted base endpoints. No failed candidate is an argument."""
    ls,lb=accepted_pose(left,base_from_sensor)
    rs,rb=accepted_pose(right,base_from_sensor)
    lo,hi=stamp_ns(left),stamp_ns(right)
    gap=(hi-lo)*1e-9
    if not 0 < gap <= .25 or not lo < failed_stamp_ns < hi:
        raise ValueError('isolated bridge exceeds the fixed .25s accepted bracket')
    step=float(np.linalg.norm(rb[:3,3]-lb[:3,3])); speed=step/gap
    if step > .12 or speed > .35:
        raise ValueError('accepted endpoint base motion exceeds original gates')
    if any(not np.isfinite(right[k]) or right[k] > bound for k,bound in (
            ('rotation_prior_error',.08),('translation_step',.12),('linear_speed',.35))):
        raise ValueError('real accepted endpoint violates original frontend motion gates')
    middle_delta,end_delta=gyro_deltas(times,gyro,bias,lo,failed_stamp_ns,hi)
    closure=end_delta.T @ ls[:3,:3].T @ rs[:3,:3]
    closure_vector=Rotation.from_matrix(closure).as_rotvec()
    innovation=float(np.linalg.norm(closure_vector))
    if innovation > .08:
        raise ValueError('actual gyro endpoint innovation exceeds original .08rad gate')
    fraction=(failed_stamp_ns-lo)/(hi-lo)
    sensor_rotation=ls[:3,:3] @ middle_delta @ Rotation.from_rotvec(fraction*closure_vector).as_matrix()
    base=np.eye(4)
    base[:3,:3]=sensor_rotation @ base_from_sensor[:3,:3].T
    base[:3,3]=(1-fraction)*lb[:3,3]+fraction*rb[:3,3]
    sensor=base @ base_from_sensor
    rigid(sensor);rigid(base)
    return sensor,base,{'left_stamp_ns':lo,'right_stamp_ns':hi,'fraction':fraction,
        'accepted_bracket_gap_sec':gap,'gyro_endpoint_innovation_rad':innovation,
        'base_endpoint_step_m':step,'base_endpoint_speed_m_sec':speed,
        'closure_correction_rotvec_rad':closure_vector.tolist()}


def normalize_station(station, rows, summary, base_from_sensor, times, gyro, bias):
    accepted_indices=audit_station(station,rows,summary,base_from_sensor)
    normalized=[]
    for i,(scan,row)in enumerate(zip(station['scans'],rows)):
        accepted=row['accepted'] is True
        out={'ordinal':i,'stamp':row['stamp'],'stamp_ns':scan['stamp_ns'],
             'role':scan['role'],'accepted':accepted,'registration_accepted':accepted,
             'pose_supported':True,'pose_source':'registration' if accepted else 'imu_bridge',
             'eligible_for_fusion':accepted,'eligible_for_train':accepted and scan['role']=='train',
             'eligible_for_heldout':scan['role']=='heldout'}
        if accepted:
            sensor,base=accepted_pose(row,base_from_sensor)
        else:
            left={**rows[i-1],'stamp_ns':station['scans'][i-1]['stamp_ns']}
            right={**rows[i+1],'stamp_ns':station['scans'][i+1]['stamp_ns']}
            sensor,base,metadata=bridge_pose(left,scan['stamp_ns'],right,
                                             base_from_sensor,times,gyro,bias)
            out['bridge']=metadata
            out['source_failure']={key:row[key]for key in ('failure_class','rejected_gates','reason')}
        out['reference_from_sensor']=sensor.tolist()
        out['reference_from_base']=base.tolist()
        normalized.append(out)
    counts={'raw_frames':len(rows),'registration_accepted_frames':len(accepted_indices),
            'imu_bridge_frames':len(rows)-len(accepted_indices),'pose_supported_frames':len(rows),
            'fused_frames':len(accepted_indices),
            'train_registration_frames':sum(r['eligible_for_train']for r in normalized),
            'heldout_raw_frames':sum(r['eligible_for_heldout']for r in normalized)}
    return normalized,counts


def validate_supported_rows(station,rows,counts,base_from_sensor):
    """Consumers recheck normalized flags/counts; a bridge cannot become accepted."""
    if len(rows)!=len(station['scans']) or counts['raw_frames']!=len(rows):
        raise ValueError('supported raw inventory differs')
    for scan,row in zip(station['scans'],rows):
        accepted=row.get('accepted') is True
        if (row.get('stamp_ns')!=scan['stamp_ns'] or abs(round(row['stamp']*1e9)-scan['stamp_ns'])>1
                or row.get('role')!=scan['role'] or row.get('pose_supported') is not True
                or row.get('registration_accepted') is not accepted
                or row.get('eligible_for_fusion') is not accepted
                or row.get('eligible_for_train') is not (accepted and scan['role']=='train')
                or row.get('eligible_for_heldout') is not (scan['role']=='heldout')
                or row.get('pose_source')!=('registration' if accepted else 'imu_bridge')):
            raise ValueError('supported pose changed registration, role or eligibility flags')
        sensor,base=rigid(row['reference_from_sensor']),rigid(row['reference_from_base'])
        if not np.allclose(base,sensor@np.linalg.inv(base_from_sensor),rtol=0,atol=1e-8):
            raise ValueError('supported base/sensor composition mismatch')
    expected={'raw_frames':len(rows),'registration_accepted_frames':sum(r['accepted']for r in rows),
        'imu_bridge_frames':sum(not r['accepted']for r in rows),'pose_supported_frames':len(rows),
        'fused_frames':sum(r['eligible_for_fusion']for r in rows),
        'train_registration_frames':sum(r['eligible_for_train']for r in rows),
        'heldout_raw_frames':sum(r['eligible_for_heldout']for r in rows)}
    if counts!=expected:raise ValueError('supported pose counts differ from complete row ledger')


def validate_calibration(plan,calibration,prepared,actual_imu_sha):
    if (calibration['source_sha256']!=plan['calibration_source_sha256']
            or actual_imu_sha!=plan['calibration_source_sha256']
            or calibration.get('generated_from_actual_accepted_initial_scan') is not True
            or abs(round(calibration['calibration_stamp']*1e9)-plan['calibration_scan']['stamp_ns'])>1
            or prepared.get('stationary_selfcheck_passed') is not True
            or not np.allclose(calibration['gyro_bias'],prepared['gyro_bias_rad_sec'],rtol=0,atol=1e-12)):
        raise ValueError('trusted calibration is not tied to actual raw IMU source')
    bias=np.asarray(calibration['gyro_bias'],float)
    if bias.shape!=(3,)or not np.isfinite(bias).all():raise ValueError('invalid trusted gyro bias')
    return bias


def support(input_path,local_dir,output):
    inputs=Inputs();record_code(inputs)
    input_path=inputs.artifact(input_path);plan=read_json(input_path)
    if (not plan.get('all_gyro_turns_complete') or not plan.get('all_settle_sensor_selfchecks_passed')
            or plan.get('GT_used_by_estimator') is not False):
        raise ValueError('complete sensor-only station protocol required')
    local_dir=Path(local_dir).resolve()
    summary=read_json(inputs.artifact(local_dir/'summary.json'))
    if summary.get('native_flow_returncode') != 0 or Path(summary['source_input']).resolve()!=input_path:
        raise ValueError('closed native execution/input binding required')
    runtime=inputs.artifact(summary['native_runtime_input'],local_dir)
    if (summary.get('diagnostic_mapping_next_scan_retry_requested') is not True
            or inputs.hashes[str(runtime)]!=summary['native_runtime_input_sha256']
            or read_json(runtime)!={**plan,'diagnostic_mapping_next_scan_retry':True}):
        raise ValueError('mapping-only opt-in runtime input differs from frozen prepared source')
    if not summary.get('native_and_python_calibration_bias_agree'):
        raise ValueError('actual trusted calibration did not match')
    native=Path(summary['native_output']).resolve()
    if native!=local_dir/'native':
        raise ValueError('native artifact directory mismatch')
    native_summary=read_json(inputs.artifact(native/'summary.json',local_dir))
    ids=[s['id']for s in plan['stations']]
    if [r['id']for r in native_summary['stations']]!=ids or [r['id']for r in summary['stations']]!=ids:
        raise ValueError('all ordered stations must be retained')
    raw_imu=inputs.artifact(plan['calibration_source']['path'],input_path.parent)
    data=np.load(raw_imu)
    times=np.asarray(data['stamps_ns'],np.int64);gyro=np.asarray(data['angular_velocity'],float)
    check_imu(times,gyro,np.asarray(data['linear_acceleration'],float))
    calibration=read_json(inputs.artifact(native/'trusted_calibration.json',local_dir))
    prepared=read_json(inputs.artifact(input_path.parent/'calibration_protocol_check.json'))
    bias=validate_calibration(plan,calibration,prepared,inputs.hashes[str(raw_imu)])
    bs=rigid(plan['base_from_sensor'])
    if not np.array_equal(rigid(plan['sensor_from_imu']),np.eye(4)):
        raise ValueError('this adapter requires the verified identity sensor/IMU extrinsic')
    out=new_directory(output);write_json(out/'rules.json',SUPPORT_RULES)
    station_records=[];totals={}
    for station,record in zip(plan['stations'],native_summary['stations']):
        sid=station['id'];source=inputs.artifact(native/sid/'poses.json',local_dir)
        poses,counts=normalize_station(station,read_json(source),record,bs,times,gyro,bias)
        cloud_name=record['cloud_file']
        if Path(cloud_name).name!=cloud_name:raise ValueError('invalid native cloud filename')
        cloud=inputs.artifact(native/sid/cloud_name,local_dir)
        folder=out/sid;folder.mkdir();write_json(folder/'poses.json',poses)
        station_records.append({'id':sid,'counts':counts,'poses_path':str(folder/'poses.json'),
            'source_poses_path':str(source),'cloud':str(cloud),'cloud_is_registration_accepted_only':True})
        for key,value in counts.items():totals[key]=totals.get(key,0)+value
    write_json(out/'support.json',{'schema_version':1,'pose_support_complete':True,
        'source_input':str(input_path),'source_local':str(local_dir),'expected_station_count':len(ids),
        'source_raw_registration_flags_preserved':True,'GT_used':False,'crossstation_target_used':False,
        'counts':totals,'stations':station_records,'source_sha256':inputs.hashes,'rules':SUPPORT_RULES})
    inputs.save(out,stage='support',pose_support_complete=True)
    return True
