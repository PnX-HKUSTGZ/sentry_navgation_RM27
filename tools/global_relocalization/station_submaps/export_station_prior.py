#!/usr/bin/env python3
"""Export every raw station scan using local frontend poses and original LIO seeds.

Diagnostic only. The default uses original LIO station seeds; an explicit rejected
partial-graph mode accepts sensor-only candidate node poses while retaining every
failed adoption guard. No GT scoring file or localization query is an estimator
input. Nonstation source frames remain preserved and are explicitly not fused.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from export_dense_sensor_prior import VoxelAccumulator, sha256


def rigid(value, label):
    matrix=np.asarray(value,dtype=float)
    if matrix.shape!=(4,4)or not np.isfinite(matrix).all():raise ValueError('nonfinite/wrong-shape SE3: '+label)
    rotation=matrix[:3,:3]
    if (not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6,rtol=0)
            or not np.isclose(np.linalg.det(rotation),1.,atol=1e-6,rtol=0)
            or not np.allclose(matrix[3],[0,0,0,1],atol=1e-10,rtol=0)):
        raise ValueError('nonrigid SE3: '+label)
    return matrix


def read_binary_xyz(path):
    """Strict small PCD subset used by the frozen original raw scan inputs."""
    fields={}
    with Path(path).open('rb')as f:
        for _ in range(64):
            line=f.readline()
            if not line:raise ValueError('truncated PCD header: '+str(path))
            words=line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):continue
            fields[words[0]]=words[1:]
            if words[0]=='DATA':break
        else:raise ValueError('PCD header too long')
        if (fields.get('FIELDS')!=['x','y','z']or fields.get('SIZE')!=['4','4','4']
                or fields.get('TYPE')!=['F','F','F']or fields.get('COUNT')!=['1','1','1']
                or fields.get('DATA')!=['binary']):raise ValueError('unsupported raw PCD schema: '+str(path))
        count=int(fields['POINTS'][0]);width=int(fields['WIDTH'][0]);height=int(fields['HEIGHT'][0])
        payload=f.read()
    if count<=0 or width*height!=count or len(payload)!=count*12:raise ValueError('raw PCD point/payload count mismatch: '+str(path))
    return np.frombuffer(payload,dtype='<f4').reshape(-1,3)


def write_pcd(path,points):
    points=np.asarray(points,dtype='<f4')
    if points.ndim!=2 or points.shape[1]!=3 or not len(points)or not np.isfinite(points).all():raise ValueError('invalid output points')
    header=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
    with Path(path).open('wb')as f:f.write(header.encode('ascii'));f.write(points.tobytes())


def transform(points,matrix):
    return points@matrix[:3,:3].T+matrix[:3,3]


def station_map_from_sensor(seed,local_pose):
    return rigid(seed,'map_from_local_initial')@rigid(local_pose,'reference_from_sensor')


def check_hash(path,expected,hashes):
    path=Path(path).resolve()
    if any(word in path.name for word in ['post_evaluation','relative_scores']):raise ValueError('post-evaluation input forbidden: '+str(path))
    actual=sha256(path)
    if expected is not None and actual!=expected:raise ValueError('source SHA mismatch: '+str(path))
    if str(path)in hashes and hashes[str(path)]!=actual:raise ValueError('source changed during preflight: '+str(path))
    hashes[str(path)]=actual
    return path


def verify_unchanged(hashes):
    for path,expected in hashes.items():
        if sha256(path)!=expected:raise ValueError('source changed during export: '+path)



def source_stamp_key(stamp,sorted_stamps):
    """Point-LIO get_ros_time may serialize the same instant 1 ns lower."""
    i=int(np.searchsorted(sorted_stamps,stamp))
    candidates=[sorted_stamps[k] for k in [i-1,i] if 0<=k<len(sorted_stamps) and abs(sorted_stamps[k]-stamp)<=1]
    if len(candidates)!=1:raise ValueError('raw stamp has no unique exact-with-1ns source association')
    return candidates[0]


def preflight(input_path,local_dir,seed_path,prior,source):
    hashes={}
    for path in [input_path,seed_path,prior/'manifest.json',source/'capture.json',source/'mapping_params.yaml',source/'mapping_binary_provenance.json',local_dir/'summary.json',Path(__file__).resolve(),
                 Path(__file__).resolve().parents[1]/'export_dense_sensor_prior.py',Path(__file__).resolve().parents[1]/'refine_sensor_prior.py']:
        check_hash(path,None,hashes)
    plan=json.loads(input_path.read_text());seed=json.loads(seed_path.read_text());original=json.loads((prior/'manifest.json').read_text());capture=json.loads((source/'capture.json').read_text());local_summary=json.loads((local_dir/'summary.json').read_text())
    if (plan.get('GT_used_by_estimator')is not False or plan.get('all_station_windows_retained')is not True
            or seed.get('sensor_only')is not True or seed.get('GT_post_evaluation_files_read')is not False or seed.get('query_results_read')is not False
            or local_summary.get('GT_used_by_estimator')is not False):raise ValueError('sensor-only provenance flags invalid')
    if (capture.get('failure')is not None or not capture.get('total_waypoints')or capture.get('completed_waypoints')!=capture['total_waypoints']
            or original.get('capture_complete')is not True or original.get('ground_truth_used_for_scan_fusion')is not False):raise ValueError('source capture incomplete or provenance invalid')
    for path,digest in seed['sources_sha256'].items():check_hash(path,digest,hashes)
    for path,digest in plan['source_input_sha256'].items():check_hash(path,digest,hashes)
    for name in ['trusted_calibration.json','calibration_protocol_check.json','execution_provenance.json']:
        path=local_dir/name
        if path.is_file():check_hash(path,None,hashes)
    provenance_path=local_dir/'execution_provenance.json'
    if provenance_path.is_file():
        execution=json.loads(provenance_path.read_text())
        if execution.get('experiment_flow_returncode')!=0 or execution.get('all10_stations_complete')is not True or execution.get('no_GT_or_LIO_poses_entered_local_frontend')is not True:raise ValueError('local execution provenance incomplete')
        for path,digest in execution['source_sha256'].items():check_hash(path,digest,hashes)
    check_hash(plan['imu_samples_json'],None,hashes)
    calibration=plan['calibration_scan'];check_hash(calibration['path'],calibration['sha256'],hashes)
    if plan['calibration_source_sha256']not in plan['source_input_sha256'].values():raise ValueError('calibration origin hash not linked')
    rigid(plan['sensor_from_imu'],'sensor_from_imu');base_sensor=rigid(plan['base_from_sensor'],'base_from_sensor')
    world=rigid(seed['T_world_camera_init'],'inherited world_camera')
    if not np.allclose(world,rigid(original['T_world_camera_init'],'original world_camera'),atol=1e-10,rtol=0):raise ValueError('initial map gauge differs')
    leaf=float(original['voxel_size'])
    if not np.isfinite(leaf)or leaf<=0 or not np.isclose(leaf,.03,atol=1e-12,rtol=0):raise ValueError('expected original 0.03m voxel leaf')
    if plan['params'].get('min_range')!=.5 or plan['params'].get('max_range')!=30.:raise ValueError('source frontend range differs from requested .5..30m')
    if not np.allclose(base_sensor,rigid(original['T_base_lidar'],'original base_sensor'),atol=1e-9,rtol=0):raise ValueError('sensor extrinsics differ')
    stations=plan['stations'];metadata=capture['station_scans'];ids=[f'station_{i:02d}'for i in range(len(metadata))]
    if not stations or [x['id']for x in stations]!=ids or [x['id']for x in seed['stations']]!=ids or [x['id']for x in local_summary['stations']]!=ids:raise ValueError('station inventory/order differs')
    if any(s.get('status')!='complete'for s in metadata):raise ValueError('source station window incomplete')
    seeds={x['id']:x for x in seed['stations']};summaries={x['id']:x for x in local_summary['stations']}
    odom={round(x['stamp']*1e9):rigid(x['matrix'],'source LIO')for x in capture['odom']};clouds=capture['clouds']
    capture_ns=[round(x['stamp']*1e9)for x in clouds]
    odom_ns=[round(x['stamp']*1e9)for x in capture['odom']]
    if odom_ns!=sorted(odom_ns):raise ValueError('source LIO stamps not monotonic')
    if len(odom)!=len(capture['odom'])or len(capture_ns)!=len(set(capture_ns))or capture_ns!=sorted(capture_ns)or set(capture_ns)!=set(odom):raise ValueError('original LIO/cloud timestamp inventory differs')
    inventory=[];station_records=[];used=set()
    for station,meta in zip(stations,metadata):
        sid=station['id'];scans=station['scans'];summary=summaries[sid];s=seeds[sid]
        if (station['source_label']!=meta['label']or abs(station['start_ns']-round(meta['start_sensor_stamp']*1e9))>1
                or abs(station['end_ns']-round(meta['end_sensor_stamp']*1e9))>1):raise ValueError('station selection differs from original control time metadata')
        if (len(scans)!=station['expected_raw_frames']or len(scans)!=s['source_frames']or len(scans)!=summary['input_frames']
                or summary['accepted_frames']!=len(scans)or summary['complete']is not True):raise ValueError('station frame completeness differs')
        pose_path=check_hash(local_dir/sid/'poses.json',seed['sources_sha256'].get(str((local_dir/sid/'poses.json').resolve())),hashes);poses=json.loads(pose_path.read_text())
        if len(poses)!=len(scans):raise ValueError('pose count differs from raw scans')
        first=rigid(poses[0]['reference_from_sensor'],sid+' first local pose');seed_matrix=rigid(s['map_from_local_initial'],sid+' seed')
        first_stamp=scans[0]['stamp_ns']
        if s['first_raw_stamp_ns']!=first_stamp:raise ValueError('first source timestamp unavailable')
        lio=odom[source_stamp_key(first_stamp,capture_ns)]
        if (not np.allclose(first,rigid(s['local_first_sensor_pose'],'seed local first'),atol=1e-10,rtol=0)
                or not np.allclose(lio,rigid(s['original_LIO_first_sensor_pose'],'seed original LIO first'),atol=1e-10,rtol=0)
                or not np.allclose(seed_matrix@first,world@lio,atol=1e-9,rtol=0)
                or not np.allclose(rigid(s['camera_init_from_local'],'camera_local'),lio@np.linalg.inv(first),atol=1e-9,rtol=0)):
            raise ValueError('seed composition/initial gauge mismatch')
        stamps=[]
        for scan,pose in zip(scans,poses):
            ns=int(scan['stamp_ns']);stamps.append(ns);capture_key=source_stamp_key(ns,capture_ns)
            if capture_key in used or not station['start_ns']-1<=ns<=station['end_ns']+1:raise ValueError('raw station stamp duplicated/outside capture or window')
            used.add(capture_key)
            if abs(round(scan['stamp']*1e9)-ns)>1 or abs(round(pose['stamp']*1e9)-ns)>1:raise ValueError('raw/local pose stamp mismatch')
            if pose.get('accepted')is not True or scan.get('frame')!='left_mid360':raise ValueError('rejected pose or wrong sensor frame')
            local_pose=rigid(pose['reference_from_sensor'],'local sensor pose');map_sensor=station_map_from_sensor(seed_matrix,local_pose)
            if 'reference_from_base'in pose and not np.allclose(rigid(pose['reference_from_base'],'local base pose'),local_pose@np.linalg.inv(base_sensor),atol=1e-8,rtol=0):raise ValueError('local base/sensor convention mismatch')
            path=check_hash(scan['path'],scan['sha256'],hashes)
            inventory.append({'station_id':sid,'source_label':station['source_label'],'stamp_ns':ns,'raw_path':str(path),'expected_raw_points':int(scan['raw_points']),'original_capture_stamp_ns':int(capture_key),
                'map_from_sensor':map_sensor.tolist(),'local_reference_from_sensor':local_pose.tolist()})
        if stamps!=sorted(stamps)or len(stamps)!=len(set(stamps)):raise ValueError('station raw stamps not strictly increasing')
        expected_raw=[t for t in capture_ns if station['start_ns']-1<=t<=station['end_ns']+1]
        if len(stamps)!=len(expected_raw)or any(abs(a-b)>1 for a,b in zip(stamps,expected_raw)):raise ValueError('not all original station frame timestamps retained')
        station_records.append({'id':sid,'label':station['source_label'],'first_stamp_ns':stamps[0],'last_stamp_ns':stamps[-1],'source_frames':len(stamps),'map_from_local_initial':seed_matrix.tolist()})
    remaining=[{'capture_index':i,'stamp_ns':ns,'reason':'outside all pre-existing station turn windows; preserved at original source'}for i,ns in enumerate(capture_ns)if ns not in used]
    if len(remaining)+len(inventory)!=len(capture_ns):raise ValueError('source capture accounting mismatch')
    return {'hashes':hashes,'frames':inventory,'stations':station_records,'remaining':remaining,'world':world,'leaf':leaf,'original':original,'capture_frames':len(capture_ns)}


def rejected_graph_diagnostic(plan,node_path,baseline,seed_path):
    """Validate an explicitly rejected candidate; never reinterpret it as passed."""
    node_path,baseline=Path(node_path).resolve(),Path(baseline).resolve()
    files={name:node_path.parent/name for name in ['summary.json','fixed_scan_holdout_after_graph.json','heldout_failures_compact.json','scope_and_guards.json']}
    check_hash(node_path,None,plan['hashes'])
    for path in files.values():check_hash(path,None,plan['hashes'])
    nodes=json.loads(node_path.read_text());summary=json.loads(files['summary.json'].read_text());heldout=json.loads(files['fixed_scan_holdout_after_graph.json'].read_text());compact=json.loads(files['heldout_failures_compact.json'].read_text())
    if (nodes.get('sensor_only')is not True or nodes.get('partial_diagnostic')is not True
            or nodes.get('global_graph_adopted')is not False or nodes.get('global_geometry_rank54_passed')is not False
            or nodes.get('partial_guards_passed')is not False or nodes.get('rank')!=48
            or nodes.get('graph_nodes')!=[0,1,2,3,4,6,7,8,9]or nodes.get('unconstrained_seed_nodes')!=[5]
            or nodes.get('source_seed_sha256')!=sha256(seed_path)):
        raise ValueError('not the authorized sensor-only rejected nine-node candidate')
    if (summary.get('GT_or_query_read')is not False or summary.get('partial_diagnostic_only')is not True
            or summary.get('partial_guards_passed')is not False or summary.get('fullgraph_adoption_rejected')is not True
            or summary.get('candidate_nodepose_file')!=node_path.name or not summary.get('fullgraph_rejection_reasons')):
        raise ValueError('rejected candidate summary/provenance differs')
    required=['solver_success','solver_not_iteration_cap','finite_valid_SE3','station0_gauge_exact','station5_seed_exact',
              'node_translation_updates_bounded','node_rotation_updates_bounded','objective_not_worse','all_edge_measurements_consistent']
    if (any(summary.get('checks',{}).get(k)is not True for k in required)
            or summary['checks'].get('all_fixed_scan_holdout_preserved')is not False):
        raise ValueError('rejected diagnostic must retain all numerical/geometric guards')
    solver=summary['actual_solver_termination']
    if (summary['graph_solver'].get('success')is not True or solver.get('success')is not True
            or solver.get('status',0)<=0 or solver.get('nfev',0)>=solver.get('maximum_nfev',0)):
        raise ValueError('graph solver not genuinely converged')
    if any(e.get('passes')is not True or e['translation_residual_m']>.03 or e['rotation_residual_deg']>1 for e in summary['edge_consistency']):
        raise ValueError('edge consistency failed')
    failures=[];directions=0
    for edge in heldout:
        for direction,value in edge['directions'].items():
            directions+=1
            if value.get('fixed_domain_count_unchanged')is not True:raise ValueError('heldout denominator changed')
            if value.get('passed')is False:
                failures.append({'i':edge['i'],'j':edge['j'],'direction':direction,'rejected':value['rejected']})
            elif value.get('passed')is not True:raise ValueError('heldout state missing')
    key=lambda x:(x['i'],x['j'],x['direction'],tuple(x['rejected']))
    if directions!=20 or len(failures)!=3 or sorted(map(key,failures))!=sorted(map(key,compact)):
        raise ValueError('authorized three-of-twenty rejected heldout inventory differs')
    expected_ids=[x['id']for x in plan['stations']]
    if len(expected_ids)!=10 or [x['id']for x in nodes['stations']]!=expected_ids or [x['node']for x in nodes['stations']]!=list(range(10)):
        raise ValueError('partial candidate must retain all ten stations')
    for name in ['manifest.json','source_inputs.json','rules.json','raw_frame_inventory.json','nonstation_capture_inventory.json','exporter_source.py','station_prior_export_complete.json']:
        check_hash(baseline/name,None,plan['hashes'])
    bm=json.loads((baseline/'manifest.json').read_text());br=json.loads((baseline/'rules.json').read_text());bh=json.loads((baseline/'source_inputs.json').read_text());bf=json.loads((baseline/'raw_frame_inventory.json').read_text());remaining=json.loads((baseline/'nonstation_capture_inventory.json').read_text())
    if (bm.get('cross_station_graph_optimized')is not False or bm.get('all_station_frames_fused_once')is not True
            or bm['fused_scans']!=len(plan['frames'])or bm['moving_capture_frames_not_fused']!=len(plan['remaining'])
            or bm['voxel_size']!=plan['leaf']or not np.array_equal(np.array(bm['T_world_camera_init']),plan['world'])
            or br.get('minimum_range_m')!=.5 or br.get('maximum_range_m')!=30.
            or br.get('voxel_frame')!='original camera_init, then unchanged initial world_camera gauge once'
            or remaining!=plan['remaining']):raise ValueError('baseline scope/voxel/gauge differs')
    own=str(Path(__file__).resolve())
    for path,digest in bh.items():
        if path!=own and plan['hashes'].get(path)!=digest:raise ValueError('baseline sensor/provenance input differs: '+path)
    if bh.get(own)!=sha256(baseline/'exporter_source.py'):raise ValueError('baseline archived exporter hash mismatch')
    inventory_key=lambda x:(x['station_id'],x['stamp_ns'],x['raw_path'],x['expected_raw_points'])
    if list(map(inventory_key,bf))!=list(map(inventory_key,plan['frames'])):raise ValueError('baseline raw inventory differs')
    new={}
    for i,(station,row)in enumerate(zip(plan['stations'],nodes['stations'])):
        old=rigid(station['map_from_local_initial'],'original node seed');used=rigid(row['map_from_local'],'rejected node pose')
        if i in [0,5]and not np.array_equal(old,used):raise ValueError('station0/5 must remain exactly at original seed')
        if i==5 and(row.get('geometrically_constrained')is not False or row.get('updated_in_partial_component')is not False):raise ValueError('station5 isolation must remain explicit')
        position=float(np.linalg.norm(used[:3,3]-old[:3,3]));angle=float(Rotation.from_matrix(old[:3,:3].T@used[:3,:3]).magnitude()*180/np.pi)
        if position>.5 or angle>10 or not np.isclose(position,summary['node_translation_updates_m'][i],atol=1e-8,rtol=0)or not np.isclose(angle,summary['node_rotation_updates_deg'][i],atol=1e-8,rtol=0):raise ValueError('node update guard/summary differs')
        new[station['id']]=used;station['map_from_local_used']=used.tolist();station['node_pose_source']='rejected partial geometry candidate'if i!=5 else'unchanged isolated original LIO seed'
    for frame in plan['frames']:
        before=frame['map_from_sensor'];frame['baseline_seed_map_from_sensor']=before;frame['map_from_sensor']=station_map_from_sensor(new[frame['station_id']],frame['local_reference_from_sensor']).tolist()
        if frame['station_id']in ['station_00','station_05']and frame['map_from_sensor']!=before:raise ValueError('fixed-station frame transform changed')
    decision={'diagnostic_arrangement_influenced_by_posthoc_GT_trajectory_review':True,'blind_preregistered_diagnostic':False,
              'GT_scoring_files_read_by_exporter':False,'GT_used_by_estimator_or_fusion':False,
              'description':'The authorized diagnostic export was prompted by later GT trajectory improvements. Node estimation and scan fusion remain sensor-only; no GT score file is an exporter input.'}
    return {'node_path':str(node_path),'node_sha256':sha256(node_path),'source_files':files,'summary':summary,'failures':failures,'compact_failures':compact,
            'heldout_directions':directions,'baseline':str(baseline),'baseline_manifest':bm,'baseline_frames':bf,'decision_provenance':decision,
            'shared_sensor_inputs_identical_to_baseline':True,'station0_seed_exact':True,'station5_seed_exact':True}


def export_station_prior(input_path,local_dir,seed_path,prior,source,output,*,rejected_nodeposes=None,baseline_prior=None):
    requested_output=Path(output).absolute()
    if requested_output.exists()or requested_output.is_symlink():raise FileExistsError('output exists: '+str(requested_output))
    input_path,local_dir,seed_path,prior,source,output=[Path(x).resolve()for x in [input_path,local_dir,seed_path,prior,source,output]]
    if output.exists()or output.is_symlink()or any(output==x or output.is_relative_to(x)for x in [local_dir,input_path.parent,prior,source,seed_path.parent]):raise FileExistsError('refuse existing/source-nested output: '+str(output))
    if (rejected_nodeposes is None)!=(baseline_prior is None):raise ValueError('rejected nodeposes and baseline prior must be explicitly paired')
    plan=preflight(input_path,local_dir,seed_path,prior,source)
    diagnostic=rejected_graph_diagnostic(plan,rejected_nodeposes,baseline_prior,seed_path)if rejected_nodeposes is not None else None
    output.mkdir(parents=True);start=time.monotonic()
    (output/'source_inputs.json').write_text(json.dumps(plan['hashes'],indent=2)+'\n');(output/'exporter_source.py').write_bytes(Path(__file__).read_bytes())
    rules={'purpose':'diagnostic station-only prior, not full-route accepted map','station_selection':'all original completed station labels/time windows, no GT/query-based frame selection',
        'point_selection':'every finite original raw return with range .5..30m inclusive; no local_cloud.pcd/feature cloud geometry read','minimum_range_m':.5,'maximum_range_m':30.,
        'voxel_size_m':plan['leaf'],'voxel_frame':'original camera_init, then unchanged initial world_camera gauge once',
        'projection':'map_from_sensor = sensor_seed.map_from_local_initial * local_pose.reference_from_sensor; no extra base/lidar rotation',
        'cross_station_registration_or_optimization':False,'post_evaluation_files_read':False,'expected_stations':len(plan['stations']),'expected_raw_frames':len(plan['frames']),'source_timestamp_serialization_tolerance_ns':1}
    if diagnostic:
        rules.update(purpose='explicitly rejected partial-graph candidate export for diagnosis only',projection='map_from_sensor = rejected_candidate.map_from_local * unchanged local_pose.reference_from_sensor',
                     cross_station_registration_or_optimization=True,node_pose_input=diagnostic['node_path'],failed_heldout_guards_retained=True,diagnostic_decision_provenance=diagnostic['decision_provenance'])
    (output/'rules.json').write_text(json.dumps(rules,indent=2)+'\n');(output/'nonstation_capture_inventory.json').write_text(json.dumps(plan['remaining'],indent=2)+'\n')
    accumulator=VoxelAccumulator(plan['leaf']);camera_world=np.linalg.inv(plan['world']);pending=[];counts=[]
    for i,frame in enumerate(plan['frames']):
        points=read_binary_xyz(frame['raw_path'])
        if len(points)!=frame['expected_raw_points']:raise ValueError('raw frame count differs from producer metadata')
        finite=np.isfinite(points).all(axis=1);q=points[finite].astype(float);ranges=np.linalg.norm(q,axis=1);near=ranges<.5;far=ranges>30.;q=q[~(near|far)]
        if not len(q):raise ValueError('accepted frame has no finite in-range raw points')
        map_sensor=rigid(frame['map_from_sensor'],'per-frame map sensor');camera_sensor=camera_world@map_sensor
        if not np.allclose(plan['world']@camera_sensor,map_sensor,atol=1e-9,rtol=0):raise ValueError('world gauge applied incorrectly')
        pending.append(transform(q,camera_sensor));counts.append({**frame,'raw_points':len(points),'nonfinite_points':int((~finite).sum()),'below_min_range_points':int(near.sum()),'above_max_range_points':int(far.sum()),'fused_raw_points':len(q)})
        if len(pending)>=20:accumulator.add(np.concatenate(pending));pending=[]
        if i%100==0:print(json.dumps({'raw_frames_fused':i+1,'map_points':len(accumulator.points),'wall_sec':time.monotonic()-start}),flush=True)
    if pending:accumulator.add(np.concatenate(pending))
    if len(counts)!=len(plan['frames'])or len(set(x['stamp_ns']for x in counts))!=len(counts):raise ValueError('not all frames fused exactly once')
    points=transform(accumulator.points,plan['world']).astype('<f4');path=output/'map.pcd';write_pcd(path,points)
    verify_unchanged(plan['hashes'])
    if len(read_binary_xyz(path))!=len(points):raise ValueError('output PCD validation failed')
    (output/'raw_frame_inventory.json').write_text(json.dumps(counts,indent=2)+'\n');(output/'station_inventory.json').write_text(json.dumps(plan['stations'],indent=2)+'\n')
    total={k:sum(x[k]for x in counts)for k in ['raw_points','nonfinite_points','below_min_range_points','above_max_range_points','fused_raw_points']}
    if total['raw_points']!=sum(total[k]for k in ['nonfinite_points','below_min_range_points','above_max_range_points','fused_raw_points']):raise ValueError('point accounting does not conserve input')
    if diagnostic:
        fields=list(total)
        if total!=diagnostic['baseline_manifest']['point_counts']or any(any(a[k]!=b[k]for k in fields)for a,b in zip(counts,diagnostic['baseline_frames'])):raise ValueError('point filtering/counts differ from baseline')
    original=plan['original'];manifest={'schema_version':1,'frame_id':original['frame_id'],'T_map_world':rigid(original['T_map_world'],'T_map_world').tolist(),'T_world_camera_init':plan['world'].tolist(),
        'T_base_lidar':original['T_base_lidar'],'voxel_size':plan['leaf'],'capture_complete':False,'source_capture_complete':True,'station_frames_complete':True,'diagnostic_only':True,
        'formal_localization_acceptance_passed':False,'pose_source':'within-station raw frontend; cross-station poses from original LIO initial seeds only',
        'ground_truth_used_for_scan_fusion':False,'ground_truth_used_for_pose_initialization':False,'post_evaluation_files_read':False,'ideal_3d_map_used':False,'query_results_read':False,
        'alignment':'unchanged original prior single initial gauge inherited through sensor_seed; no new GT alignment',
        'cross_station_graph_optimized':False,'optimized_graph_used_for_3d_fusion':False,'two_dimensional_graph_used':False,'slam_toolbox_used':False,
        'source_capture':str(source),'source_raw_prior':str(prior),'source_station_input':str(input_path),'source_local_frontend':str(local_dir),'source_sensor_seed':str(seed_path),
        'source_capture_frames':plan['capture_frames'],'source_stations':len(plan['stations']),'fused_scans':len(counts),'source_station_frames':len(plan['frames']),'all_station_frames_fused_once':True,
        'moving_capture_frames_not_fused':len(plan['remaining']),'moving_frames_preserved_at_source':True,'nonstation_frame_inventory':'nonstation_capture_inventory.json','dropped_station_frame_indices':[],
        'point_counts':total,'points':len(points),'bounds':[points.min(axis=0).tolist(),points.max(axis=0).tolist()],'map_sha256':sha256(path),
        'source_inputs_verified_unchanged':True,'source_hashes':plan['hashes'],'rules_sha256':sha256(output/'rules.json'),'raw_frame_inventory_sha256':sha256(output/'raw_frame_inventory.json'),
        'scan_model':original['scan_model'],'pointlio_instantaneous_scan':original['pointlio_instantaneous_scan'],'timing_model_consistent':original['timing_model_consistent'],
        'mapper_sha256':original['mapper_sha256'],'mapping_params_sha256':sha256(source/'mapping_params.yaml'),'wall_sec':time.monotonic()-start}
    if diagnostic:
        manifest.update(pose_source='within-station raw frontend; rejected partial nine-node geometry candidate with isolated station5 at original LIO seed',
            cross_station_graph_optimized=True,optimized_graph_used_for_3d_fusion=True,partial_graph_only=True,global_graph_adopted=False,global_geometry_rank54_passed=False,
            partial_guards_passed=False,failed_heldout_directions=len(diagnostic['failures']),total_heldout_directions=diagnostic['heldout_directions'],
            rejected_guard_failures=diagnostic['compact_failures'],fullgraph_rejection_reasons=diagnostic['summary']['fullgraph_rejection_reasons'],
            diagnostic_nodeposes=diagnostic['node_path'],diagnostic_nodeposes_sha256=diagnostic['node_sha256'],baseline_station_prior=diagnostic['baseline'],
            unchanged_source_inputs_except_nodeposes=True,station0_original_seed_exact=True,station5_unconstrained_original_seed_exact=True,
            diagnostic_decision_provenance=diagnostic['decision_provenance'])
    for name in ['mapping_params.yaml','mapping_binary_provenance.json']:
        if(source/name).is_file():shutil.copyfile(source/name,output/name)
    validation={'all_source_hashes_before_after_verified':True,'all_raw_frames_fused_exactly_once':True,'all_SE3_valid':True,'all_station_seed_compositions_verified':True,
        'same_original_world_gauge':True,'all_raw_point_counts_conserved':True,'output_PCD_schema_finite_verified':True,'source_and_output_frame_accounting':{'original_capture':plan['capture_frames'],'station_fused':len(counts),'nonstation_not_fused':len(plan['remaining'])},
        'post_evaluation_files_read':False,'aggregate_local_cloud_geometry_read':False,'GT_read_for_pose_initialization_or_fusion':False,'map_sha256':manifest['map_sha256']}
    if diagnostic:
        validation.update(partial_candidate_export_not_guard_adoption=True,all_failed_guards_retained=True,station0_and5_bit_exact_original_seeds=True,
                          same_raw_frames_filter_voxel_and_gauge_as_baseline=True,baseline_point_counts_identical=True,
                          baseline_sensor_input_hashes_identical_except_exporter_implementation=True,diagnostic_decision_provenance=diagnostic['decision_provenance'])
        shutil.copyfile(diagnostic['node_path'],output/'source_rejected_candidate_nodeposes.json')
        for name,path in diagnostic['source_files'].items():shutil.copyfile(path,output/('source_partial_graph_'+name))
        (output/'MAP_QUALITY_REJECTED.json').write_text(json.dumps({'reason':'Explicit rejected-candidate diagnostic only: 3/20 fixed scan-heldout directions failed; full graph not adopted; formal batches forbidden.',
            'formal_batch_allowed':False,'partial_guards_passed':False,'failed_heldout_directions':diagnostic['compact_failures'],
            'fullgraph_rejection_reasons':diagnostic['summary']['fullgraph_rejection_reasons'],'diagnostic_decision_provenance':diagnostic['decision_provenance']},indent=2)+'\n')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(output/'validation_report.json').write_text(json.dumps(validation,indent=2)+'\n')
    reason='rejected partial graph diagnostic; failed adoption guards retained, formal batch forbidden'if diagnostic else'station-only diagnostic; moving frames not fused, inter-station seeds not optimized, no localization acceptance yet'
    (output/'DIAGNOSTIC_MAP_ONLY.json').write_text(json.dumps({'reason':reason,'formal_batch_allowed':False},indent=2)+'\n')
    (output/'station_prior_export_complete.json').write_text(json.dumps({'completed':True,'map_sha256':manifest['map_sha256'],'manifest_sha256':sha256(output/'manifest.json'),'validation_report_sha256':sha256(output/'validation_report.json')},indent=2)+'\n')
    print(json.dumps({k:manifest[k]for k in ['fused_scans','moving_capture_frames_not_fused','point_counts','points','map_sha256','wall_sec']},indent=2),flush=True);return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['input','local','sensor_seed','raw_prior','source_capture','output']:p.add_argument(name,type=Path)
    p.add_argument('--rejected-nodeposes',type=Path,help='explicit rejected nine-node diagnostic only, never adoption')
    p.add_argument('--baseline-prior',type=Path,help='required paired original station LIO seed map for input invariance checks')
    a=p.parse_args();export_station_prior(a.input,a.local,a.sensor_seed,a.raw_prior,a.source_capture,a.output,rejected_nodeposes=a.rejected_nodeposes,baseline_prior=a.baseline_prior)


if __name__=='__main__':main()
