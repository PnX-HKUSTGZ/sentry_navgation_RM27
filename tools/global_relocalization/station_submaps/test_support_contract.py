"""Physical and fail-closed contracts for mapping-only heldout pose support."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipeline_common import Inputs
from pipeline_inputs import native_dependencies
from pipeline_model import evaluate, freeze_domain, supported_geometry
from pipeline_support import (audit_station, bridge_pose, normalize_station,
                              validate_calibration, validate_supported_rows)


def fixture(role='heldout'):
    bs=np.eye(4)
    bs[:3,:3]=Rotation.from_euler('x',.3).as_matrix()
    bs[:3,3]=[.25,.10,.20]
    stamps=[10_000_000_000,10_100_000_000,10_200_000_000]
    def pose(u):
        base=np.eye(4);base[:3,:3]=Rotation.from_euler('z',3.05+u).as_matrix()
        base[:3,3]=[.1*u,0,0]
        sensor=base@bs
        return {'stamp':10.+u,'accepted':True,'reference_from_sensor':sensor.tolist(),
                'reference_from_base':base.tolist(),'rotation_prior_error':0.,
                'translation_step':.02,'linear_speed':.1}
    left,right=pose(0),pose(.2)
    state={'last_accepted_stamp':left['stamp'],'last_accepted_pose':left['reference_from_sensor'],
           'submap_size':7,'token_after_stamp':-1.,'real_stop_after_stamp':-1.,
           'pending':False,'hard_fault':False,'recovery_attempts':0,
           'recovered_scan_count':0,'mapping_authorizations':0,'mapping_tokens_consumed':0}
    after={**deepcopy(state),'pending':True,'token_after_stamp':10.1,'mapping_authorizations':1}
    failed={'stamp':10.1,'accepted':False,'failure_class':'numerical_recoverable',
            'rejected_gates':['convergence'],'reason':'retained pure convergence failure',
            'reference_from_sensor':'POISON FAILED CANDIDATE MUST NEVER BE READ',
            'reference_from_base':'POISON FAILED CANDIDATE MUST NEVER BE READ',
            'mapping_retry_authorized_after_failure':True,
            'mapping_recovery_before':deepcopy(state),'mapping_recovery_after':deepcopy(after)}
    right['mapping_recovery_before']=deepcopy(after)
    right['mapping_recovery_after']={**deepcopy(after),'last_accepted_stamp':right['stamp'],
        'last_accepted_pose':right['reference_from_sensor'],'pending':False,
        'recovery_attempts':1,'recovered_scan_count':1,'mapping_tokens_consumed':1}
    right['actual_registration_interval_sec']=.2
    rows=[left,failed,right]
    station={'id':'synthetic','expected_raw_frames':3,
        'scans':[{'stamp_ns':s,'role':r}for s,r in zip(stamps,['train',role,'heldout'])]}
    summary={'input_frames':3,'accepted_frames':2,'fused_frames':2,'fused_scan_ordinals':[0,2]}
    times=np.arange(9_990_000_000,10_220_000_000,10_000_000,dtype=np.int64)
    gyro=np.tile(bs[:3,:3].T@[0.,0.,1.],(len(times),1))
    return station,rows,summary,bs,times,gyro,np.zeros(3),pose(.1)


def test_actual_gyro_bridge_preserves_base_lever_arm_across_heading_wrap_and_failed_poison():
    station,rows,summary,bs,t,w,bias,truth=fixture()
    before=deepcopy(rows)
    normalized,counts=normalize_station(station,rows,summary,bs,t,w,bias)
    middle=normalized[1]
    assert np.allclose(middle['reference_from_base'],truth['reference_from_base'],atol=1e-12)
    assert np.allclose(middle['reference_from_sensor'],truth['reference_from_sensor'],atol=1e-12)
    naive=.5*(np.array(rows[0]['reference_from_sensor'])[:3,3]+np.array(rows[2]['reference_from_sensor'])[:3,3])
    assert np.linalg.norm(naive-np.array(middle['reference_from_sensor'])[:3,3])>1e-4
    assert rows==before and middle['accepted'] is False
    assert middle['pose_source']=='imu_bridge' and middle['pose_supported']
    assert not middle['eligible_for_fusion'] and not middle['eligible_for_train']
    assert counts=={'raw_frames':3,'registration_accepted_frames':2,'imu_bridge_frames':1,
        'pose_supported_frames':3,'fused_frames':2,'train_registration_frames':1,'heldout_raw_frames':2}
    validate_supported_rows(station,normalized,counts,bs)


def test_bridge_closure_uses_both_accepted_rotations_but_no_failed_pose():
    station,rows,summary,bs,t,w,bias,_=fixture()
    # A small independent accepted endpoint correction must be distributed over
    # the bridge, rather than discarding gyro or jumping at the right endpoint.
    right_sensor=np.array(rows[2]['reference_from_sensor'])
    correction=Rotation.from_rotvec([.02,-.01,.005]).as_matrix()
    right_sensor[:3,:3]=right_sensor[:3,:3]@correction
    rows[2]['reference_from_sensor']=right_sensor.tolist()
    rows[2]['reference_from_base']=(right_sensor@np.linalg.inv(bs)).tolist()
    sensor,base,meta=bridge_pose(rows[0],station['scans'][1]['stamp_ns'],rows[2],bs,t,w,bias)
    assert meta['gyro_endpoint_innovation_rad']>0
    assert meta['gyro_endpoint_innovation_rad']<.08
    assert np.allclose(sensor,base@bs,atol=1e-12)
    assert np.allclose(base[:3,3],.5*(np.array(rows[0]['reference_from_base'])[:3,3]+
                                        np.array(rows[2]['reference_from_base'])[:3,3]))


@pytest.mark.parametrize('reason',['boundary','adjacent','wrong_gate','hard_fault','committed','stop','tokens','fused'])
def test_no_bridge_for_unsupported_failure_or_untruthful_native_state(reason):
    station,rows,summary,bs,*_=fixture()
    if reason=='boundary':
        rows[0]['accepted']=False;summary.update(accepted_frames=1,fused_frames=1,fused_scan_ordinals=[2])
    elif reason=='adjacent':
        rows[2]['accepted']=False;summary.update(accepted_frames=1,fused_frames=1,fused_scan_ordinals=[0])
    elif reason=='wrong_gate':rows[1]['rejected_gates']=['convergence','inlier_ratio']
    elif reason=='hard_fault':rows[1]['mapping_recovery_after']['hard_fault']=True
    elif reason=='committed':rows[1]['mapping_recovery_after']['submap_size']+=1
    elif reason=='stop':rows[1]['mapping_recovery_after']['real_stop_after_stamp']=10.1
    elif reason=='tokens':rows[2]['mapping_recovery_after']['mapping_tokens_consumed']=2
    else:summary.update(fused_frames=3,fused_scan_ordinals=[0,1,2])
    with pytest.raises(ValueError):audit_station(station,rows,summary,bs)


@pytest.mark.parametrize('reason',['gap','imu_hole','gyro','speed','step'])
def test_existing_sensor_motion_limits_and_new_bridge_gap_are_hard_guards(reason):
    station,rows,_,bs,t,w,bias,_=fixture()
    middle=station['scans'][1]['stamp_ns']
    if reason=='gap':rows[2]['stamp']=10.26
    elif reason=='imu_hole':
        keep=(t<10_040_000_000)|(t>10_090_000_000);t,w=t[keep],w[keep]
    elif reason=='gyro':w=w*0.
    elif reason=='speed':
        base=np.array(rows[2]['reference_from_base']);base[0,3]=.08
        rows[2]['reference_from_base']=base.tolist();rows[2]['reference_from_sensor']=(base@bs).tolist()
    else:rows[2]['translation_step']=.121
    with pytest.raises(ValueError):bridge_pose(rows[0],middle,rows[2],bs,t,w,bias)


def test_stamp_tolerance_does_not_rewrite_integer_sensor_bridge_endpoints():
    station,rows,summary,bs,t,w,bias,_=fixture()
    for r in rows:r['stamp']+=1e-9
    for state in (rows[1]['mapping_recovery_before'],rows[1]['mapping_recovery_after'],
                  rows[2]['mapping_recovery_before'],rows[2]['mapping_recovery_after']):
        state['last_accepted_stamp']+=1e-9
        if state['token_after_stamp']>0:state['token_after_stamp']+=1e-9
    normalized,_=normalize_station(station,rows,summary,bs,t,w,bias)
    assert normalized[1]['bridge']['left_stamp_ns']==10_000_000_000
    assert normalized[1]['bridge']['right_stamp_ns']==10_200_000_000
    rows[1]['stamp']+=2e-9
    with pytest.raises(ValueError):audit_station(station,rows,summary,bs)


def test_train_failure_never_becomes_training_and_heldout_bridge_keeps_point_denominator():
    station,rows,summary,bs,t,w,bias,_=fixture(role='train')
    normalized,counts=normalize_station(station,rows,summary,bs,t,w,bias)
    assert not normalized[1]['eligible_for_train'] and counts['train_registration_frames']==1
    x,y=np.meshgrid(np.arange(5)*.08,np.arange(5)*.08)
    points=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    source={'train':points,'held':np.r_[points,points],'hf':np.r_[np.ones(25,int),np.full(25,2)],
            'rawids':np.tile(np.arange(25),2),'ordinals':[1,2]}
    normals=np.tile([0.,0.,1.],(25,1))
    target={'train':points,'geom':supported_geometry(points,normals,np.ones(25,bool))}
    domain,_=freeze_domain(source,target,np.eye(4))
    fixed={'hi':domain['held_indices']}
    initial=evaluate([source,target],0,1,fixed,np.eye(4))
    transform=np.eye(4);transform[2,3]=.31
    lost=evaluate([source,target],0,1,fixed,transform)
    assert initial['fixed_domain_points']==lost['fixed_domain_points']==50
    assert [r['fixed_denominator']for r in lost['frames']]==[25,25]
    assert lost['post_support_lost_points']==50 and lost['frame_equal_capped_nn_rmse_m']==.3


def test_normalized_bridge_cannot_be_promoted_to_accepted_fusion_or_training():
    station,rows,summary,bs,t,w,bias,_=fixture()
    normalized,counts=normalize_station(station,rows,summary,bs,t,w,bias)
    normalized[1]['eligible_for_fusion']=True
    with pytest.raises(ValueError,match='eligibility'):validate_supported_rows(station,normalized,counts,bs)


@pytest.mark.parametrize('bad',['origin','stamp','bias','source','prepared'])
def test_support_independently_verifies_real_calibration_not_summary_boolean(bad):
    plan={'calibration_source_sha256':'a'*64,'calibration_scan':{'stamp_ns':2_000_000_000}}
    snapshot={'source_sha256':'a'*64,'generated_from_actual_accepted_initial_scan':True,
              'calibration_stamp':2.,'gyro_bias':[.001,-.002,.003]}
    prepared={'stationary_selfcheck_passed':True,'gyro_bias_rad_sec':snapshot['gyro_bias'][:]}
    if bad=='origin':snapshot['generated_from_actual_accepted_initial_scan']=False
    elif bad=='stamp':snapshot['calibration_stamp']+=3e-9
    elif bad=='bias':snapshot['gyro_bias'][0]+=.001
    elif bad=='source':snapshot['source_sha256']='b'*64
    else:prepared['stationary_selfcheck_passed']=False
    with pytest.raises(ValueError):validate_calibration(plan,snapshot,prepared,'a'*64)


def test_native_dependency_layout_supports_named_mapping_adapter_and_relative_static_archive(tmp_path,monkeypatch):
    native=tmp_path/'offline_station_mapping_recovery';native.write_bytes(b'fixture executable')
    archive=tmp_path/'frontend/libexperimental_station_frontend.a';archive.parent.mkdir();archive.write_bytes(b'archive')
    gicp=tmp_path/'libsmall_gicp.so';gicp.write_bytes(b'gicp input')
    link=tmp_path/'CMakeFiles'/ (native.name+'.dir')/'link.txt';link.parent.mkdir(parents=True)
    link.write_text('c++ object.o frontend/libexperimental_station_frontend.a '+str(gicp))
    monkeypatch.setattr('pipeline_inputs.subprocess.run',lambda *a,**k:SimpleNamespace(returncode=0,stdout=''))
    sources=Inputs();audit=native_dependencies(native,sources)
    assert audit['experimental_frontend_link_inputs']==[str(archive)]
    assert str(gicp)in sources.hashes and str(archive)in sources.hashes


def test_closed_support_seed_domains_chain_keeps_failed_heldout_and_excludes_failed_train(tmp_path):
    """Exercise real stage handoffs without a native process, bag, GT or target fit."""
    from pipeline_common import write_json,read_json,sha256
    from pipeline_support import support
    from pipeline_inputs import sensor_seed
    from pipeline_pairs import build_domains,load_domains
    from export_station_prior import write_pcd
    prepared=tmp_path/'prepared';prepared.mkdir()
    local=tmp_path/'local';local.mkdir();native=local/'native';native.mkdir()
    station,rows,record,bs,t,w,bias,_=fixture()
    imu=prepared/'raw_imu_samples.npz'
    np.savez(imu,stamps_ns=t,angular_velocity=w,linear_acceleration=np.tile([0.,0.,9.81],(len(t),1)))
    write_json(prepared/'calibration_protocol_check.json',{'stationary_selfcheck_passed':True,
                'gyro_bias_rad_sec':bias.tolist()})
    capture=tmp_path/'capture.json';manifest=tmp_path/'manifest.json'
    write_json(capture,{'odom':[{'stamp':r['stamp'],'matrix':r['reference_from_sensor']}for r in [rows[0],rows[2]]],
                        'gt':'FORBIDDEN'})
    write_json(manifest,{'T_world_camera_init':np.eye(4).tolist()})
    u,v=np.meshgrid(np.arange(-.8,.81,.08),np.arange(-.8,.81,.08));uv=np.c_[u.ravel(),v.ravel()]
    world=np.r_[np.c_[uv,np.full(len(uv),2.)],np.c_[np.full(len(uv),2.),uv],
                np.c_[uv[:,0],np.full(len(uv),2.),uv[:,1]]]
    stations=[];records=[]
    for sid,failed_role in [('first','train'),('closure','heldout')]:
        st=deepcopy(station);st.update(id=sid,source_label=sid,waypoint_index=0,kind=sid,
            expected_capture_raw_frames=3,excluded_settle_prefix_scans=[],geometry_start_rule='synthetic post-settle')
        st['scans'][1]['role']=failed_role
        folder=native/sid;folder.mkdir();source_rows=deepcopy(rows)
        write_json(folder/'poses.json',source_rows)
        write_pcd(folder/'incomplete_prefix_cloud.pcd',world.astype(np.float32))
        for i,scan in enumerate(st['scans']):
            if i==1:
                sensor=np.array(fixture()[-1]['reference_from_sensor'])
            else:sensor=np.array(rows[i]['reference_from_sensor'])
            raw=(world-sensor[:3,3])@sensor[:3,:3]
            if i==1 and failed_role=='train':raw=np.tile([10.,10.,10.],(20,1))
            raw_path=prepared/(sid+str(i)+'.pcd');write_pcd(raw_path,raw.astype(np.float32))
            scan.update(stamp=scan['stamp_ns']*1e-9,path=str(raw_path),sha256=sha256(raw_path))
        stations.append(st)
        records.append({**record,'id':sid,'cloud_file':'incomplete_prefix_cloud.pcd','complete':False})
    plan={'stations':stations,'expected_station_count':2,'all_gyro_turns_complete':True,
          'all_settle_sensor_selfchecks_passed':True,'GT_used_by_estimator':False,
          'base_from_sensor':bs.tolist(),'sensor_from_imu':np.eye(4).tolist(),
          'calibration_source':{'path':str(imu)},'calibration_source_sha256':sha256(imu),
          'calibration_scan':{'stamp_ns':9_990_000_000},
          'source_capture':str(capture),'source_raw_prior_manifest':str(manifest),
          'source_input_sha256':{str(capture):sha256(capture),str(manifest):sha256(manifest)},
          'params':{'min_range':.5,'max_range':30.}}
    write_json(prepared/'input.json',plan);Inputs().save(prepared,stage='prepare')
    runtime=local/'mapping_runtime_input.json';write_json(runtime,{**plan,'diagnostic_mapping_next_scan_retry':True})
    write_json(native/'trusted_calibration.json',{'source_sha256':sha256(imu),
        'calibration_stamp':9.99,'gyro_bias':bias.tolist(),'generated_from_actual_accepted_initial_scan':True})
    write_json(native/'summary.json',{'stations':records})
    write_json(local/'summary.json',{'stations':records,'all_stations_complete':False,
        'native_output':str(native),'native_flow_returncode':0,
        'native_and_python_calibration_bias_agree':True,'source_input':str(prepared/'input.json'),
        'native_runtime_input':str(runtime),'native_runtime_input_sha256':sha256(runtime),
        'diagnostic_mapping_next_scan_retry_requested':True})
    Inputs().save(local,stage='local')
    with pytest.raises(ValueError,match='all local station'):sensor_seed(prepared/'input.json',local,tmp_path/'strict_seed')
    supported=tmp_path/'support';assert support(prepared/'input.json',local,supported)
    counts=read_json(supported/'support.json')['counts']
    assert counts['raw_frames']==6 and counts['fused_frames']==4 and counts['imu_bridge_frames']==2
    seed=tmp_path/'seed';assert sensor_seed(prepared/'input.json',local,seed,supported)
    with pytest.raises(ValueError,match='explicit --support'):build_domains(seed/'sensor_seed.json',tmp_path/'strict_domains')
    domains=tmp_path/'domains';assert build_domains(seed/'sensor_seed.json',domains,allow_support=True)
    first_train=np.load(domains/'stations/first/train.npy')
    assert np.max(np.linalg.norm(first_train,axis=1))<4., 'rejected TRAIN poison entered the model'
    held_frames=np.load(domains/'stations/closure/held_frame.npy')
    assert np.sum(held_frames==1)==len(world) and np.sum(held_frames==2)==len(world)
    seed_value,_,_=load_domains(domains,Inputs())
    assert seed_value['pose_support_mode'] is True
    split=read_json(domains/'stations/closure/frame_split.json')
    assert split[1]['registration_accepted'] is False and split[1]['pose_source']=='imu_bridge'


@pytest.mark.parametrize('stage,enabled',[('local',False),('local',True),('seed',False),
    ('seed',True),('domains',False),('domains',True),('support',True)])
def test_cli_support_is_explicit_and_default_strict_dispatch_is_unchanged(stage,enabled,monkeypatch):
    from pathlib import Path
    import pipeline
    calls=[]
    def record(*args,**kwargs):calls.append((args,kwargs));return True
    if stage=='local':
        monkeypatch.setattr('pipeline_inputs.local',record)
        argv=['local','--input','input','--native','native','--output','out']
        if enabled:argv+=['--support']
        expected={'allow_support':enabled}
    elif stage=='seed':
        monkeypatch.setattr('pipeline_inputs.sensor_seed',record)
        argv=['seed','--input','input','--local','local','--output','out']
        if enabled:argv+=['--support','supportdir']
        expected={'support_dir':Path('supportdir') if enabled else None}
    elif stage=='domains':
        monkeypatch.setattr('pipeline_pairs.build_domains',record)
        argv=['domains','--seed','seed','--output','out']
        if enabled:argv+=['--support']
        expected={'allow_support':enabled}
    else:
        monkeypatch.setattr('pipeline_support.support',record)
        argv=['support','--input','input','--local','local','--output','out'];expected={}
    assert pipeline.main(argv)==0 and len(calls)==1 and calls[0][1]==expected
