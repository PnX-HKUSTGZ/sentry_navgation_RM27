import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parent))
import export_station_prior as module
from export_station_prior import export_station_prior, read_binary_xyz, rigid, sha256, station_map_from_sensor, source_stamp_key, transform, write_pcd


def matrix(xyz=(0,0,0),rot=(0,0,0)):
    t=np.eye(4);t[:3,:3]=Rotation.from_rotvec(rot).as_matrix();t[:3,3]=xyz;return t


def save(path,obj):path.write_text(json.dumps(obj))


def fixture(tmp_path,station_count=2):
    source,prior,inputs,local,seed_dir=[tmp_path/name for name in ['source','prior','inputs','local','seed']]
    for folder in [source,prior,inputs,local,seed_dir]:folder.mkdir()
    c=matrix((9,-3,.7),(.2,-.1,.3));b=matrix((0,.12,.13),(-.5,0,0));stations=[];seeds=[];summary=[];odom=[];clouds=[];metadata=[];expected=[]
    for i in range(station_count):
        sid=f'station_{i:02d}';folder=local/sid;folder.mkdir();(folder/'local_cloud.pcd').write_text('THIS AGGREGATE CLOUD MUST NEVER BE PARSED')
        (folder/'post_evaluation.json').write_text('FORBIDDEN INVALID GT SCORING FILE')
        a=matrix((5*i,2*i,.2*i));first=matrix(rot=(-.3+.1*i,.1,0));seed=c@a@np.linalg.inv(first);poses=[];scans=[]
        for k in range(2):
            stamp=1.+2*i+.1*k;ns=round(stamp*1e9);lp=first@matrix((.01*k,.005*k,0),(0,0,.01*k));raw=np.array([[1+.2*k,0,0],[1.04+.2*k,0,0],[1.16+.2*k,.11,.1],[.1,0,0],[31,0,0],[np.nan,0,0]],np.float32)
            raw_path=inputs/f'raw_{ns}.pcd'
            # Write source payload directly: output writer deliberately refuses
            # nonfinite data, whereas real raw sensor payloads may contain NaN.
            h=f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 6\nHEIGHT 1\nPOINTS 6\nDATA binary\n'
            raw_path.write_bytes(h.encode()+raw.astype('<f4').tobytes())
            scans.append({'stamp':stamp,'stamp_ns':ns,'path':str(raw_path),'sha256':sha256(raw_path),'frame':'left_mid360','raw_points':6})
            poses.append({'stamp':stamp,'accepted':True,'reference_from_sensor':lp.tolist(),'reference_from_base':(lp@np.linalg.inv(b)).tolist()})
            odom.append({'stamp':stamp,'matrix':(a@matrix((.01*k,0,0))).tolist()});clouds.append({'stamp':stamp,'index':len(clouds)})
            expected.extend(transform(raw[:3].astype(float),seed@lp))
        save(folder/'poses.json',poses);metadata.append({'label':f'route_{i}','status':'complete','start_sensor_stamp':scans[0]['stamp'],'end_sensor_stamp':scans[-1]['stamp'],'start_xyz':'FORBIDDEN GT CONTROL VALUE','measured_yaw_rad':'FORBIDDEN'})
        stations.append({'id':sid,'source_label':f'route_{i}','start_ns':scans[0]['stamp_ns'],'end_ns':scans[-1]['stamp_ns'],'expected_raw_frames':2,'scans':scans})
        seeds.append({'id':sid,'first_raw_stamp_ns':scans[0]['stamp_ns'],'original_LIO_first_sensor_pose':a.tolist(),'local_first_sensor_pose':first.tolist(),'camera_init_from_local':(a@np.linalg.inv(first)).tolist(),'map_from_local_initial':seed.tolist(),'source_frames':2})
        summary.append({'id':sid,'input_frames':2,'accepted_frames':2,'complete':True})
    odom.append({'stamp':2.,'matrix':matrix((2,1,.1)).tolist()});odom.sort(key=lambda x:x['stamp']);clouds.append({'stamp':2.,'index':len(clouds)});clouds.sort(key=lambda x:x['stamp'])
    for i,x in enumerate(clouds):x['index']=i
    save(source/'capture.json',{'failure':None,'completed_waypoints':2,'total_waypoints':2,'station_scans':metadata,'odom':odom,'clouds':clouds,'gt':'UNUSABLE: MUST NOT BE READ'})
    (source/'mapping_params.yaml').write_text('{}');(source/'mapping_binary_provenance.json').write_text('{}')
    save(prior/'manifest.json',{'capture_complete':True,'ground_truth_used_for_scan_fusion':False,'T_world_camera_init':c.tolist(),'T_map_world':np.eye(4).tolist(),'T_base_lidar':b.tolist(),'voxel_size':.03,'frame_id':'map',
        'scan_model':'instantaneous','pointlio_instantaneous_scan':True,'timing_model_consistent':True,'mapper_sha256':'frozen_source_mapper'})
    save(local/'summary.json',{'GT_used_by_estimator':False,'stations':summary})
    calibration=inputs/'calibration.pcd';calibration.write_bytes(Path(stations[0]['scans'][0]['path']).read_bytes());imu=inputs/'raw_imu.npz';imu.write_bytes(b'sensor-only IMU input');samples=inputs/'imu_samples.json';samples.write_text('[]')
    save(inputs/'input.json',{'GT_used_by_estimator':False,'all_station_windows_retained':True,'params':{'min_range':.5,'max_range':30.},'sensor_from_imu':np.eye(4).tolist(),'base_from_sensor':b.tolist(),
        'source_input_sha256':{str(imu):sha256(imu)},'imu_samples_json':str(samples),'calibration_source_sha256':sha256(imu),'calibration_scan':{'path':str(calibration),'sha256':sha256(calibration)},'stations':stations})
    sources=[inputs/'input.json',local/'summary.json',source/'capture.json',prior/'manifest.json']
    for s in stations:sources.extend([local/s['id']/'poses.json',local/s['id']/'local_cloud.pcd'])
    seed_path=seed_dir/'sensor_seed.json';save(seed_path,{'sensor_only':True,'GT_post_evaluation_files_read':False,'query_results_read':False,'T_world_camera_init':c.tolist(),'stations':seeds,'sources_sha256':{str(p):sha256(p)for p in sources}})
    return inputs/'input.json',local,seed_path,prior,source,tmp_path/'derived',np.array(expected)


def test_projection_uses_full_sensor_pose_and_one_world_gauge():
    seed=matrix((4,-2,1),(.3,.2,-.5));local=matrix((.1,.2,.3),(-.4,.1,.2));p=np.array([[1.,2,3]])
    assert np.allclose(transform(p,station_map_from_sensor(seed,local)),transform(transform(p,local),seed))
    assert not np.allclose(transform(p,seed@local@local),transform(p,seed@local))


def test_every_raw_frame_is_fused_with_no_aggregate_or_GT_input(tmp_path):
    *args,expected=fixture(tmp_path);manifest=export_station_prior(*args);out=args[-1];actual=read_binary_xyz(out/'map.pcd')
    assert manifest['fused_scans']==4 and manifest['moving_capture_frames_not_fused']==1
    assert manifest['source_stations']==2 and not manifest['capture_complete'] and manifest['source_capture_complete']
    assert manifest['diagnostic_only'] and not manifest['post_evaluation_files_read']
    assert manifest['point_counts']=={'raw_points':24,'nonfinite_points':4,'below_min_range_points':4,'above_max_range_points':4,'fused_raw_points':12}
    assert len(actual)==12
    from scipy.spatial import cKDTree
    assert cKDTree(actual).query(expected)[0].max()<1e-6
    assert cKDTree(expected).query(actual)[0].max()<1e-6
    assert (out/'station_prior_export_complete.json').is_file()
    inventory=json.loads((out/'raw_frame_inventory.json').read_text());assert len(inventory)==4
    assert (out/'DIAGNOSTIC_MAP_ONLY.json').is_file()
    with pytest.raises(FileExistsError):export_station_prior(*args)


def test_source_hash_tampering_rejects_before_output(tmp_path):
    *args,expected=fixture(tmp_path);plan=json.loads(args[0].read_text());raw=Path(plan['stations'][0]['scans'][0]['path']);raw.write_bytes(raw.read_bytes()+b'bad')
    with pytest.raises(ValueError,match='SHA mismatch'):export_station_prior(*args)
    assert not args[-1].exists()


def test_missing_pose_and_nonrigid_pose_cannot_be_hidden(tmp_path):
    *args,expected=fixture(tmp_path);path=args[1]/'station_00/poses.json';poses=json.loads(path.read_text());poses[1]['reference_from_sensor'][0][0]=2;save(path,poses)
    seed=json.loads(args[2].read_text());seed['sources_sha256'][str(path)]=sha256(path);save(args[2],seed)
    with pytest.raises(ValueError,match='nonrigid'):export_station_prior(*args)
    assert not args[-1].exists()
    poses=poses[:1];save(path,poses);seed['sources_sha256'][str(path)]=sha256(path);save(args[2],seed)
    with pytest.raises(ValueError,match='pose count'):export_station_prior(*args)


def test_source_drift_cannot_receive_completion_marker(tmp_path,monkeypatch):
    *args,expected=fixture(tmp_path);plan=json.loads(args[0].read_text());raw=Path(plan['stations'][0]['scans'][0]['path']);original=module.write_pcd
    def changed(path,points):original(path,points);raw.write_bytes(raw.read_bytes()+b'drift')
    monkeypatch.setattr(module,'write_pcd',changed)
    with pytest.raises(ValueError,match='source changed during export'):export_station_prior(*args)
    assert not(args[-1]/'manifest.json').exists()and not(args[-1]/'station_prior_export_complete.json').exists()


def test_strict_pcd_schema_and_payload_count(tmp_path):
    p=tmp_path/'x.pcd';write_pcd(p,np.array([[1,2,3]],float));p.write_bytes(p.read_bytes()[:-1])
    with pytest.raises(ValueError,match='payload count'):read_binary_xyz(p)
    p.write_text('FIELDS x y z\n')
    with pytest.raises(ValueError,match='truncated'):read_binary_xyz(p)
    with pytest.raises(ValueError,match='SE3'):rigid(np.full((4,4),np.nan),'bad')


def test_source_header_nanosecond_serialization_has_unique_bounded_association():
    assert source_stamp_key(1000000000,[999999999,1100000000])==999999999
    with pytest.raises(ValueError,match='no unique'):source_stamp_key(1000000001,[999999999,1100000000])
    with pytest.raises(ValueError,match='no unique'):source_stamp_key(1000000000,[999999999,1000000001])


def diagnostic_fixture(tmp_path,move=False):
    *args,expected=fixture(tmp_path,station_count=10)
    baseline=export_station_prior(*args)
    seed=json.loads(args[2].read_text());folder=tmp_path/'rejected_graph';folder.mkdir();rows=[];position=[];angle=[]
    for i,s in enumerate(seed['stations']):
        pose=np.array(s['map_from_local_initial']);old=pose.copy()
        if move and i in [1,7]:pose[0,3]+=.04
        rows.append({'node':i,'id':s['id'],'map_from_local':pose.tolist(),'geometrically_constrained':i!=5,'updated_in_partial_component':i!=5})
        position.append(float(np.linalg.norm(pose[:3,3]-old[:3,3])));angle.append(0.)
    path=folder/'rejected_candidate_nodeposes.json'
    save(path,{'sensor_only':True,'partial_diagnostic':True,'global_graph_adopted':False,'global_geometry_rank54_passed':False,'partial_guards_passed':False,'rank':48,
        'graph_nodes':[0,1,2,3,4,6,7,8,9],'unconstrained_seed_nodes':[5],'source_seed_sha256':sha256(args[2]),'stations':rows})
    pairs=[(0,3),(0,9),(1,2),(1,3),(2,3),(3,4),(3,7),(6,7),(6,8),(7,8)];held=[];failures=[]
    for i,j in pairs:
        directions={}
        for direction in ['forward','reverse']:
            failed=direction=='reverse'and(i,j)in [(0,9),(6,8),(7,8)];reasons=['fixed_scan_heldout_worse']if failed else[]
            directions[direction]={'passed':not failed,'fixed_domain_count_unchanged':True,'rejected':reasons}
            if failed:failures.append({'i':i,'j':j,'direction':direction,'rejected':reasons})
        held.append({'i':i,'j':j,'directions':directions})
    checks={k:True for k in ['solver_success','solver_not_iteration_cap','finite_valid_SE3','station0_gauge_exact','station5_seed_exact','node_translation_updates_bounded','node_rotation_updates_bounded','objective_not_worse','all_edge_measurements_consistent']};checks['all_fixed_scan_holdout_preserved']=False
    save(folder/'summary.json',{'GT_or_query_read':False,'partial_diagnostic_only':True,'partial_guards_passed':False,'fullgraph_adoption_rejected':True,'fullgraph_rejection_reasons':['station5 isolated','rank48<54'],
        'candidate_nodepose_file':path.name,'checks':checks,'graph_solver':{'success':True},'actual_solver_termination':{'success':True,'status':2,'nfev':22,'maximum_nfev':600},
        'node_translation_updates_m':position,'node_rotation_updates_deg':angle,'edge_consistency':[{'i':i,'j':j,'passes':True,'translation_residual_m':0.,'rotation_residual_deg':0.}for i,j in pairs]})
    save(folder/'fixed_scan_holdout_after_graph.json',held);save(folder/'heldout_failures_compact.json',failures);save(folder/'scope_and_guards.json',{'partial_only':True})
    (folder/'post_evaluation_rejected_pose_scores.json').write_text('MUST NOT BE READ OR HASHED')
    baseline_dir=args[-1];args[-1]=tmp_path/'partial';return args,path,baseline_dir


def test_rejected_identity_nodes_preserve_baseline_pcd_bytes_and_failures(tmp_path):
    args,path,baseline=diagnostic_fixture(tmp_path)
    result=export_station_prior(*args,rejected_nodeposes=path,baseline_prior=baseline)
    assert result['map_sha256']==sha256(baseline/'map.pcd')
    assert result['partial_graph_only']and not result['global_graph_adopted']and not result['partial_guards_passed']
    assert result['failed_heldout_directions']==3 and result['total_heldout_directions']==20
    assert result['diagnostic_decision_provenance']['diagnostic_arrangement_influenced_by_posthoc_GT_trajectory_review']
    assert not result['diagnostic_decision_provenance']['blind_preregistered_diagnostic']
    assert result['post_evaluation_files_read']is False
    assert (args[-1]/'MAP_QUALITY_REJECTED.json').is_file()and not result['capture_complete']
    assert all('post_evaluation'not in p for p in result['source_hashes'])


def test_partial_changes_only_node_projection_and_keeps_fixed_nodes_exact(tmp_path):
    args,path,baseline=diagnostic_fixture(tmp_path,move=True)
    result=export_station_prior(*args,rejected_nodeposes=path,baseline_prior=baseline)
    bm=json.loads((baseline/'manifest.json').read_text());before=json.loads((baseline/'raw_frame_inventory.json').read_text());after=json.loads((args[-1]/'raw_frame_inventory.json').read_text())
    assert result['map_sha256']!=bm['map_sha256']and result['point_counts']==bm['point_counts']
    assert len(before)==len(after)==20
    for a,b in zip(before,after):
        assert a['raw_path']==b['raw_path']and a['stamp_ns']==b['stamp_ns']
        if a['station_id']in ['station_00','station_05']:assert a['map_from_sensor']==b['map_from_sensor']
        if a['station_id']=='station_01':assert np.allclose(np.array(b['map_from_sensor'])[:3,3]-np.array(a['map_from_sensor'])[:3,3],[.04,0,0])


@pytest.mark.parametrize('node',[0,5])
def test_fixed_nodes_cannot_be_changed_even_in_rejected_mode(tmp_path,node):
    args,path,baseline=diagnostic_fixture(tmp_path)
    data=json.loads(path.read_text());data['stations'][node]['map_from_local'][0][3]+=.001;save(path,data)
    with pytest.raises(ValueError,match='station0/5'):export_station_prior(*args,rejected_nodeposes=path,baseline_prior=baseline)
    assert not args[-1].exists()


@pytest.mark.parametrize('violation',['iteration_cap','hidden_heldout_failure','pose_not_rigid','false_adoption'])
def test_rejected_mode_does_not_bypass_remaining_guards(tmp_path,violation):
    args,path,baseline=diagnostic_fixture(tmp_path);data=json.loads(path.read_text());summary=json.loads((path.parent/'summary.json').read_text())
    if violation=='iteration_cap':summary['actual_solver_termination']['nfev']=600;save(path.parent/'summary.json',summary)
    if violation=='hidden_heldout_failure':
        held=json.loads((path.parent/'fixed_scan_holdout_after_graph.json').read_text());held[1]['directions']['reverse']['passed']=True;save(path.parent/'fixed_scan_holdout_after_graph.json',held)
    if violation=='pose_not_rigid':data['stations'][1]['map_from_local'][0][0]=2.;save(path,data)
    if violation=='false_adoption':data['global_graph_adopted']=True;save(path,data)
    with pytest.raises(ValueError):export_station_prior(*args,rejected_nodeposes=path,baseline_prior=baseline)
    assert not args[-1].exists()
