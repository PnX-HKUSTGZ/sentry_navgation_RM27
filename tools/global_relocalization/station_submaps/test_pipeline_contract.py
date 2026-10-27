"""Meaningful numerical/coverage contracts for the generic station pipeline."""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pipeline_common import Inputs, new_directory, read_json, write_json
from gyro_turns import calibrate, integrate, turn_ledger
from pipeline_inputs import station_windows,geometry_after_settle,bind_raw_prior,validate_source_frames
from pipeline_model import Rules, evaluate, fit, freeze_domain, supported_geometry
from pipeline_graph import GraphObjective, audit_graph, dense_solve, post_heldout


def turn_fixture(direction=1, bias=None):
    # Sensor is rolled relative to a horizontal base: |omega| and sensor omega_z
    # are not the bottom-frame heading. Calibrate only the initial static data.
    times = np.arange(0, 18_010_000_000, 10_000_000, dtype=np.int64)
    bs = np.eye(4)
    bs[:3, :3] = Rotation.from_euler('x', -30, degrees=True).as_matrix()
    bias = np.array([.001, -.002, .003]) if bias is None else np.asarray(bias)
    gyro = np.tile(bias, (len(times), 1))
    # Constant speed begins at4s. Both sides of this IMU sample receive the same
    # angular velocity so the synthetic integration has a well-defined start.
    omega_sensor = bs[:3, :3].T @ np.array([0., 0., float(direction)])
    gyro[times >= 4_000_000_000] += omega_sensor
    acceleration = np.tile(bs[:3, :3].T @ [0., 0., 9.81], (len(times), 1))
    calibration = calibrate(times, gyro, acceleration, 2_000_000_000)
    stamps, quat = integrate(times, gyro, calibration)
    row = {'capture_start_ros_ns': 3_000_000_000, 'rotation_command_start_ros_ns': 4_000_000_000,
           'stop_command_ros_ns': 17_000_000_000, 'capture_end_ros_ns': 18_000_000_000,
           'commanded_speed_rad_sec': direction, 'measured_yaw_rad': 'FORBIDDEN',
           'start_xyz': 'FORBIDDEN'}
    return times, gyro, acceleration, stamps, quat, bs, row


@pytest.mark.parametrize('direction', [1, -1])
def test_actual_gyro_two_turns_with_lever_frame_and_complete_raw_tail(direction):
    _, _, _, stamps, quat, bs, row = turn_fixture(direction)
    scans = np.arange(3_000_000_000, 18_000_000_000, 100_000_000, dtype=np.int64)
    ledger = turn_ledger(row, scans, stamps, quat, bs)
    assert ledger['complete']
    assert ledger['direction_from_command_only'] == direction
    assert not ledger['control_measured_yaw_or_XYZ_used']
    assert sum(ledger['role_counts'].values()) == len(scans)
    first, second = ledger['turn_boundaries_ns'][1:]
    assert abs((first - row['rotation_command_start_ros_ns']) * 1e-9 - 2*np.pi) < .006
    assert abs((second - first) * 1e-9 - 2*np.pi) < 1e-8
    assert ledger['role_counts']['aux_pre'] > 0 and ledger['role_counts']['aux_post'] > 0
    # Exact boundary timestamps are never rounded into the previous role.
    augmented = np.sort(np.unique(np.r_[scans, first, second]))
    exact = turn_ledger(row, augmented, stamps, quat, bs)
    roles = {r['stamp_ns']:r['role'] for r in exact['scans']}
    assert roles[first] == 'heldout' and roles[second] == 'aux_post'
    assert roles[first - 1] == 'train' if first - 1 in roles else True


def test_incomplete_gyro_turn_is_not_completed_from_controller_claim():
    _, _, _, stamps, quat, bs, row = turn_fixture()
    row['stop_command_ros_ns'] = 15_000_000_000
    row['measured_yaw_rad'] = 1000.
    scans = np.arange(3_000_000_000, 18_000_000_000, 100_000_000, dtype=np.int64)
    ledger = turn_ledger(row, scans, stamps, quat, bs)
    assert not ledger['complete'] and len(ledger['turn_boundaries_ns']) == 2
    assert ledger['role_counts']['aux_incomplete_turn1'] > 0
    assert 'heldout' not in ledger['role_counts']
    assert sum(ledger['role_counts'].values()) == len(scans)


def test_dynamic_initial_window_cannot_supply_bias_and_moving_accel_does_not_feed_rotation():
    times, gyro, accel, _, _, _, _ = turn_fixture()
    bad = gyro.copy()
    bad[:201, 0] += .2
    result = calibrate(times, bad, accel, 2_000_000_000)
    assert not result['stationary_selfcheck_passed'] and not result['checks']['gyro_norm']
    with pytest.raises(ValueError, match='calibration'):
        integrate(times, bad, result)
    initial = calibrate(times, gyro, accel, 2_000_000_000)
    changed = accel.copy()
    changed[times > 2_000_000_000] = [100, -200, 50]
    same = calibrate(times, gyro, changed, 2_000_000_000)
    assert initial == same
    assert np.array_equal(integrate(times, gyro, initial)[1], integrate(times, gyro, same)[1])


def test_imu_hole_is_not_hidden_by_window_or_scan_assignment():
    times, gyro, accel, *_ = turn_fixture()
    keep = np.ones(len(times), bool)
    keep[1000:1004] = False
    with pytest.raises(ValueError, match='noncontinuous'):
        calibrate(times[keep], gyro[keep], accel[keep], 2_000_000_000)


def test_braking_prefix_is_preserved_outside_stationary_model_before_any_error_is_known():
    ledger={'scans':[{'ordinal':i,'stamp_ns':ns,'role':role}for i,(ns,role)in enumerate(
        [(100,'aux_pre'),(200,'aux_pre'),(300,'train'),(400,'heldout'),(500,'aux_post')])]}
    split=geometry_after_settle(ledger,200)
    assert [x['stamp_ns']for x in split['scans']]==[100,200,300,400,500]
    assert [x['stamp_ns']for x in split['scans']if x['participates_stationary_geometry']]==[300,400,500]
    assert split['excluded_settle_prefix_frames']==2
    assert split['scans'][1]['role']=='aux_settle_prefix'
    assert ledger['scans'][1]['role']=='aux_pre'
    with pytest.raises(ValueError,match='overlaps'):
        geometry_after_settle(ledger,300)


def metadata(sid, kind, waypoint, start):
    return {'station_id':sid, 'kind':kind, 'waypoint_index':waypoint,
        'label':sid, 'status':'complete', 'commanded_speed_rad_sec':1.,
        'capture_start_ros_ns':start, 'settle_zero_command_ros_ns':start,
        'settle_start_sensor_stamp_ns':start+100_000_000,
        'settle_end_sensor_stamp_ns':start+2_600_000_000,
        'rotation_command_start_ros_ns':start+2_700_000_000,
        'stop_command_ros_ns':start+16_000_000_000,
        'capture_end_ros_ns':start+17_000_000_000, 'requested_settle_sec':2.5,
        'station_xy':[0,0], 'start_xyz':'FORBIDDEN', 'measured_yaw_rad':'FORBIDDEN'}


def test_arbitrary_visit_inventory_retains_same_xy_closure_and_ignores_truth():
    rows = [metadata('start', 'initial', -1, 0), metadata('middle', 'arc', 20, 20_000_000_000),
            metadata('return', 'closure', 40, 40_000_000_000)]
    result = station_windows(rows)
    assert [x['station_id'] for x in result] == ['start','middle','return']
    assert not any('station_xy' in x or 'measured_yaw_rad' in x or 'start_xyz' in x for x in result)
    rows[2]['station_id'] = 'start'
    with pytest.raises(ValueError, match='duplicate'):
        station_windows(rows)


def test_artifact_completion_chain_detects_modified_source_and_refuses_overwrite(tmp_path):
    stage = new_directory(tmp_path/'stage')
    write_json(stage/'data.json', {'frames':[1,2,3]})
    Inputs().save(stage, stage='test')
    tracked = Inputs()
    tracked.artifact(stage/'data.json')
    write_json(stage/'data.json', {'frames':[1,2]})
    with pytest.raises(ValueError, match='changed'):
        tracked.verify()
    with pytest.raises(ValueError, match='SHA'):
        Inputs().artifact(stage/'data.json')
    with pytest.raises(FileExistsError):
        new_directory(stage)


def test_raw_prior_must_be_bound_to_exact_source_and_gauge_before_bag_consumption(tmp_path):
    source=tmp_path/'capture';source.mkdir()
    names=['capture.json','mapping_params.yaml','manifest.json','mapping_binary_provenance.json']
    for name in names:(source/name).write_text('{}')
    from pipeline_common import sha256
    manifest={'source_capture_directory':str(source),'source_sha256':{n:sha256(source/n)for n in names},
        'source_completed_waypoints':20,'source_total_waypoints':20,'source_failure':None,
        'T_map_world':np.eye(4).tolist()}
    capture={'completed_waypoints':20,'total_waypoints':20}
    bind_raw_prior(source,capture,manifest,Inputs())
    wrong={**manifest,'source_capture_directory':str(tmp_path/'other')}
    with pytest.raises(ValueError,match='different capture'):
        bind_raw_prior(source,capture,wrong,Inputs())
    (source/'capture.json').write_text('{"changed":true}')
    with pytest.raises(ValueError,match='SHA'):
        bind_raw_prior(source,capture,manifest,Inputs())


def test_sensor_frame_switch_is_rejected_despite_unchanged_numeric_extrinsics():
    assert validate_source_frames({'left_mid360'},{'imu_link'},'left_mid360')['imu']=='imu_link'
    for lidar,imu in [({'left_mid360','changed'},{'imu_link'}),({'left_mid360'},{''}),
                       ({'left_mid360'},{'wrong_axes'})]:
        with pytest.raises(ValueError,match='frame'):
            validate_source_frames(lidar,imu,'left_mid360')


def model_fixture():
    x,y = np.meshgrid(np.arange(5.)*.08, np.arange(5.)*.08)
    points = np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
    normal = np.tile([0.,0.,1.], (len(points)*2,1))
    target = np.r_[points, points+[0,0,.1]]
    good = np.r_[np.ones(len(points),bool),np.zeros(len(points),bool)]
    source = {'train':points, 'held':np.r_[points,points],
        'hf':np.r_[np.full(len(points),100),np.full(len(points),101)],
        'ordinals':[100,101], 'rawids':np.tile(np.arange(len(points)),2), 'full':points}
    target_station = {'train':target, 'geom':supported_geometry(target,normal,good), 'full':points}
    return source,target_station


def test_good_normal_domain_is_frozen_and_true_out_of_range_heldout_remains_capped():
    source,target = model_fixture()
    arrays,meta = freeze_domain(source,target,np.eye(4))
    domain = {'ti':arrays['train_indices'],'hi':arrays['held_indices'],'meta':meta}
    before = domain['hi'].copy()
    z = np.eye(4)
    z[2,3] = .1
    post = evaluate([source,target],0,1,domain,z)
    assert post['frame_equal_coverage']==1 and post['fixed_domain_points']==len(source['held'])
    assert abs(post['frame_equal_plane_rmse_m']-.1)<1e-12
    z[2,3] = .31
    far = evaluate([source,target],0,1,domain,z)
    assert far['post_support_lost_points']==len(source['held'])
    assert abs(far['frame_equal_plane_rmse_m']-.3)<1e-12
    assert abs(far['frame_equal_capped_nn_rmse_m']-.3)<1e-12
    assert np.array_equal(before,domain['hi'])


def test_no_supported_target_normals_is_explicit_failed_pair_not_exception():
    source,target = model_fixture()
    target['geom'] = supported_geometry(target['train'],target['geom'][1],np.zeros(len(target['train']),bool))
    arrays,meta = freeze_domain(source,target,np.eye(4))
    d = {'ti':arrays['train_indices'],'hi':arrays['held_indices'],'meta':meta}
    z,root,result = fit([source,target],Rules(),0,1,d,np.eye(4))
    assert not result['accepted'] and result['information_rank']==0
    assert 'insufficient_train_plane_matches' in result['rejected_gates']
    assert result['scan_heldout_after']['fixed_domain_points']==0
    assert result['scan_heldout_after']['frame_equal_plane_rmse_m']==.3


def test_parameterized_fitter_and_evaluator_are_exact_reviewed_v3_functions():
    """Refactoring IO/global variables must not change a numerical objective/gate."""
    from scipy.spatial import cKDTree
    from refine_sensor_prior import transform,information_root,invert
    source,target=model_fixture()
    # More cells than the frozen training minimum, with three actual normals.
    u,v=np.meshgrid(np.arange(-.8,.81,.16),np.arange(-.8,.81,.16))
    uv=np.c_[u.ravel(),v.ravel()]
    p=np.r_[np.c_[uv,np.full(len(uv),2.)],np.c_[np.full(len(uv),2.),uv],
            np.c_[uv[:,0],np.full(len(uv),2.),uv[:,1]]]
    n=np.r_[np.tile([0,0,1.],(len(uv),1)),np.tile([1.,0,0],(len(uv),1)),np.tile([0,1.,0],(len(uv),1))]
    source={'train':p,'held':np.r_[p,p],'hf':np.r_[np.full(len(p),200),np.full(len(p),201)],
            'ordinals':[200,201],'rawids':np.tile(np.arange(len(p)),2),'full':p}
    target={'train':p,'geom':supported_geometry(p,n,np.ones(len(p),bool)),'full':p}
    stations=[source,target]
    initial=np.eye(4);initial[:3,3]=[.01,-.02,.015]
    arrays,meta=freeze_domain(source,target,initial)
    domain={'ti':arrays['train_indices'],'hi':arrays['held_indices'],'meta':meta}
    file=Path(__file__).with_name('partial_overlap_v2.py')
    functions=[f for f in ast.parse(file.read_text()).body if isinstance(f,ast.FunctionDef)and f.name in ('evaluate','fit')]
    namespace={'np':np,'stations':stations,'rules':Rules(),'transform':transform,
               'information_root':information_root,'Rotation':Rotation,'cKDTree':cKDTree,'invert':invert}
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(file),'exec'),namespace)
    legacy=namespace['fit'](0,1,domain,initial)
    generic=fit(stations,Rules(),0,1,domain,initial)
    assert np.array_equal(legacy[0],generic[0])and np.array_equal(legacy[1],generic[1])
    assert legacy[2]==generic[2] and generic[2]['accepted']


def graph_fixture(n=4, root=None):
    poses = np.tile(np.eye(4), (n,1,1))
    poses[:,:3,3] = np.c_[np.arange(n),np.arange(n)*.2,np.arange(n)*.1]
    poses[:,:3,:3] = Rotation.from_rotvec(np.c_[np.arange(n)*.01,np.arange(n)*.02,np.arange(n)*.03]).as_matrix()
    from refine_sensor_prior import invert
    root = np.eye(6) if root is None else root
    edges = [{'i':i,'j':j,'measurement':(invert(poses[i])@poses[j]).tolist(),'root':root.tolist()}
             for i in range(n) for j in range(i+1,n)]
    return poses,edges


@pytest.mark.parametrize('n',[3,4,6])
def test_full_geometry_rank_scales_with_n_and_real_loops_are_distinct_pairs(n):
    poses,edges=graph_fixture(n)
    report,_=audit_graph(poses,edges)
    assert report['geometry_information']['rank']==6*(n-1)
    assert report['all_structural_guards_passed']
    for edge in report['edge_leave_one_out']:
        assert len(edge['alternate_station_path'])>=3
        assert edge['geometry_rank_without_edge']==6*(n-1)


def test_connectivity_is_not_observability_and_chain_is_not_redundancy():
    poses,edges=graph_fixture(4)
    chain=[e for e in edges if e['j']==e['i']+1]
    report,_=audit_graph(poses,chain)
    assert report['checks']['connected'] and report['checks']['full_geometry_rank']
    assert not report['all_structural_guards_passed'] and not report['checks']['all_edges_redundant']
    # With all rotations unobserved, full connectivity cannot invent orientation information.
    for e in edges:e['root']=np.diag([0,0,0,1,1,1]).tolist()
    report,_=audit_graph(poses,edges)
    assert report['checks']['connected'] and not report['checks']['full_geometry_rank']
    assert report['geometry_information']['rank']<18


def test_generic_dense_residual_is_exact_legacy_objective_and_solves_without_gauge_motion(monkeypatch):
    import refine_sensor_prior as legacy
    poses,edges=graph_fixture(4)
    initial=poses.copy()
    initial[1:,:3,3]+=[.02,-.015,.01]
    initial[1:,:3,:3]=initial[1:,:3,:3]@Rotation.from_rotvec([.003,-.004,.002]).as_matrix()
    objective=GraphObjective(initial,edges)
    probes=[np.zeros(18),np.linspace(-.001,.001,18)]
    captured=[]
    def capture(fun,x0,**kw):
        captured.extend([fun(x) for x in probes])
        return SimpleNamespace(x=x0,success=False,message='capture',nfev=0,cost=0.)
    monkeypatch.setattr(legacy,'least_squares',capture)
    legacy.optimize_graph(initial,edges,Rules())
    assert all(np.array_equal(a,objective(x))for a,x in zip(captured,probes))
    solved,record=dense_solve(initial,edges)
    assert record['success'] and record['status']>0 and record['nfev']<600
    assert np.array_equal(solved[0],initial[0])
    assert np.max(abs(solved-poses))<1e-5


def test_adopted_inverse_must_pass_heldout_even_when_separate_pairfit_passed():
    seed={'fixed_domain_points':100,'every_frame_min15':True,'frame_equal_coverage':1.,
          'frame_equal_plane_rmse_m':.016154,'frame_equal_capped_nn_rmse_m':.059}
    pair={**seed,'frame_equal_plane_rmse_m':.013639}
    adopted_inverse={**seed,'frame_equal_plane_rmse_m':.017110}
    rejected=post_heldout(adopted_inverse,seed,pair)
    assert rejected==['frame_equal_plane_rmse_m_worse_than_pair']


def test_three_visit_synthetic_stage_pipeline_with_raw_gyro_split_and_whole_turn_holdout(tmp_path,monkeypatch):
    """Exercise all six CLIs' implementations, including artifact handoff.

    Only the native process is a fixture: its already-tested numerical frontend
    is not reimplemented here. Synthetic scans/poses have known rigid geometry;
    this verifies pipeline semantics, not actual-data localization quality.
    """
    import json,sys,types
    import pipeline_inputs as pin
    from pipeline_pairs import build_domains,fit_pairs
    from pipeline_graph import solve_graph
    from export_station_prior import write_pcd
    from pipeline_common import REPO
    source=tmp_path/'capture';source.mkdir()
    prior=tmp_path/'prior';prior.mkdir()
    bag=source/'raw_bag';bag.mkdir()
    (bag/'data.mcap').write_bytes(b'synthetic bag fixture; no ROS storage opened')
    visits=[metadata('initial_visit','initial',-1,3_000_000_000),
            metadata('second_visit','arc',10,22_000_000_000),
            metadata('closure_visit','closure',20,41_000_000_000)]
    it=np.arange(0,59_010_000_000,10_000_000,dtype=np.int64)
    rt=np.arange(0,59_000_000_000,100_000_000,dtype=np.int64)
    w=np.zeros((len(it),3))
    for v in visits:w[(it>=v['rotation_command_start_ros_ns'])&(it<=v['stop_command_ros_ns']),2]=1.
    a=np.tile([0.,0.,9.81],(len(it),1))
    cal=calibrate(it,w,a,2_000_000_000)
    integrated_times,q=integrate(it,w,cal)
    from scipy.spatial.transform import Slerp
    orientation=Slerp((integrated_times-integrated_times[0])*1e-9,Rotation.from_quat(q))
    # Rich nonparallel surfaces, all >.5m range. No predetermined walls enter
    # real data: these synthetic planes merely make the rank test identifiable.
    u,v=np.meshgrid(np.arange(-.8,.81,.16),np.arange(-.8,.81,.16))
    uv=np.c_[u.ravel(),v.ravel()]
    world=np.r_[np.c_[uv,np.full(len(uv),2.)],
                np.c_[np.full(len(uv),2.),uv],np.c_[uv[:,0],np.full(len(uv),2.),uv[:,1]]]
    rotations={int(ns):(orientation([(int(ns)-int(integrated_times[0]))*1e-9]).as_matrix()[0]
                           if ns>=integrated_times[0] else np.eye(3))for ns in rt}
    odom=[]
    for ns in rt:
        pose=np.eye(4);pose[:3,:3]=rotations[int(ns)]
        odom.append({'stamp':int(ns)*1e-9,'matrix':pose.tolist()})
    write_json(source/'capture.json',{'failure':None,'completed_waypoints':21,'total_waypoints':21,
        'station_scans':visits,'odom':odom,'gt':'FORBIDDEN estimator field'})
    mapping={'point_lio':{'ros__parameters':{'mapping':{'extrinsic_est_en':False,
        'extrinsic_R':np.eye(3).ravel().tolist(),'extrinsic_T':[0.,0.,0.]},
        'preprocess':{'instantaneous_scan':True}}},
        'ign_sim_pointcloud_tool':{'ros__parameters':{'instantaneous_scan':True}}}
    import yaml
    (source/'mapping_params.yaml').write_text(yaml.safe_dump(mapping))
    protocol={'schema':'ordered_mapping_survey_v1','preserve_waypoint_order':True,
              'stations':[{'id':x['station_id'],'waypoint_index':x['waypoint_index'],'kind':x['kind'],
                           'xy':'FORBIDDEN estimator field'}for x in visits]}
    write_json(source/'survey_route_protocol.json',protocol)
    from pipeline_common import sha256
    write_json(source/'route.json',{'preserve_waypoint_order':True,'protocol_stations':protocol['stations'],
        'survey_route_sha256':sha256(source/'survey_route_protocol.json'),'waypoints':['UNUSED XY']*21})
    write_json(source/'manifest.json',{'source':'synthetic closed capture fixture'})
    write_json(source/'mapping_binary_provenance.json',{'native':'fixture'})
    write_json(prior/'manifest.json',{'capture_complete':True,'ground_truth_used_for_scan_fusion':False,
        'T_base_lidar':np.eye(4).tolist(),'T_base_imu':np.eye(4).tolist(),'T_world_camera_init':np.eye(4).tolist(),
        'source_capture_directory':str(source),'source_completed_waypoints':21,'source_total_waypoints':21,
        'source_failure':None,'T_map_world':np.eye(4).tolist(),
        'source_sha256':{n:sha256(source/n)for n in ['capture.json','mapping_params.yaml','manifest.json','mapping_binary_provenance.json']}})
    info={'rosbag2_bagfile_information':{'storage_identifier':'mcap','topics_with_message_count':[
        {'topic_metadata':{'name':'/livox/lidar'},'message_count':len(rt)},
        {'topic_metadata':{'name':'/livox/imu'},'message_count':len(it)}]}}
    (bag/'metadata.yaml').write_text(yaml.safe_dump(info))
    def fake_messages(unused,topics):
        if '/livox/imu'in topics:
            for ns,ww,aa in zip(it,w,a):
                vector=lambda values:SimpleNamespace(x=float(values[0]),y=float(values[1]),z=float(values[2]))
                yield '/livox/imu',int(ns),SimpleNamespace(header=SimpleNamespace(frame_id='imu_link'),
                    angular_velocity=vector(ww),linear_acceleration=vector(aa))
        if '/livox/lidar'in topics:
            for ns in rt:
                points=world@rotations[int(ns)]
                points=np.r_[points,[[np.nan,np.nan,np.nan]]]
                yield '/livox/lidar',int(ns),SimpleNamespace(header=SimpleNamespace(frame_id='left_mid360'),
                    width=len(points),height=1,points=points)
    monkeypatch.setattr(pin,'bag_messages',fake_messages)
    def read_points(msg,**unused):
        result=np.empty(len(msg.points),dtype=[('x','f4'),('y','f4'),('z','f4')])
        for k,key in enumerate('xyz'):result[key]=msg.points[:,k]
        return result
    monkeypatch.setitem(sys.modules,'sensor_msgs_py',types.SimpleNamespace(point_cloud2=types.SimpleNamespace(read_points=read_points)))
    prepared=tmp_path/'prepared'
    assert pin.prepare(source,prior,REPO/'src/global_relocalization/config/competition.yaml',prepared)
    plan=read_json(prepared/'input.json')
    assert len(plan['stations'])==3 and plan['all_gyro_turns_complete']
    assert plan['all_settle_sensor_selfchecks_passed']
    assert plan['calibration_source_sha256']==sha256(prepared/'raw_imu_samples.npz')
    assert plan['calibration_source_sha256']!=sha256(source/'capture.json')
    assert all(len(s['scans'])>80 for s in plan['stations'])
    assert plan['stations'][-1]['id']=='closure_visit'
    assert sum(len(s['scans'])+len(s['excluded_settle_prefix_scans'])for s in plan['stations'])+sum(
        r['role']=='nonstation'for r in read_json(prepared/'raw_inventory.json'))==len(rt)
    assert all(s['excluded_settle_prefix_scans']and s['scans'][0]['stamp_ns']>s['settle_end_sensor_stamp_ns']
               for s in plan['stations'])
    # Raw extraction keeps the nonfinite point for native preprocessing.
    from export_station_prior import read_binary_xyz
    assert np.isnan(read_binary_xyz(plan['stations'][0]['scans'][0]['path'])[-1]).all()
    native=tmp_path/'offline_adapter';native.write_text('native process fixture')
    link=tmp_path/'CMakeFiles'/ (native.name+'.dir') / 'link.txt';link.parent.mkdir(parents=True)
    gicp=tmp_path/'libsmall_gicp.so';gicp.write_text('frozen link-input fixture, statically eliminated runtime')
    link.write_text('c++ object.o -o offline_adapter '+str(gicp))
    def fake_native(argv,**kwargs):
        if argv[0]=='ldd':return SimpleNamespace(returncode=0,stdout='linux-vdso.so.1 (0x00)\n')
        output=Path(argv[2]);output.mkdir()
        summaries=[]
        for st in plan['stations']:
            directory=output/st['id'];directory.mkdir()
            poses=[]
            for scan in st['scans']:
                pose=np.eye(4);pose[:3,:3]=rotations[scan['stamp_ns']]
                poses.append({'stamp':scan['stamp'],'accepted':True,'reference_from_sensor':pose.tolist(),
                              'reference_from_base':pose.tolist()})
            write_json(directory/'poses.json',poses)
            write_pcd(directory/'local_cloud.pcd',world)
            summaries.append({'id':st['id'],'complete':True,'accepted_frames':len(poses),'input_frames':len(poses)})
        write_json(output/'summary.json',{'stations':summaries})
        write_json(output/'trusted_calibration.json',{'gyro_bias':cal['gyro_bias_rad_sec'],
            'generated_from_actual_accepted_initial_scan':True,'source_sha256':plan['calibration_source_sha256'],
            'calibration_stamp':plan['calibration_scan']['stamp']})
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(pin.subprocess,'run',fake_native)
    local=tmp_path/'local';assert pin.local(prepared/'input.json',native,local)
    seed=tmp_path/'seed';assert pin.sensor_seed(prepared/'input.json',local,seed)
    pure=read_json(seed/'sensor_seed.json')
    assert pure['GT_post_evaluation_files_read']is False and len(pure['stations'])==3
    assert all(np.allclose(s['map_from_local_initial'],np.eye(4),atol=1e-12)for s in pure['stations'])
    domains=tmp_path/'domains';assert build_domains(seed/'sensor_seed.json',domains)
    marker=read_json(domains/'domains_complete_before_any_registration.json')
    assert marker['directions']==6 and len(marker['domain_sha256'])==6
    pair_dir=tmp_path/'pairs';assert fit_pairs(domains,pair_dir)
    pair_summary=read_json(pair_dir/'summary.json')
    assert pair_summary['pairs_attempted']==3 and pair_summary['accepted_edges']==3
    graph=tmp_path/'graph';assert solve_graph(pair_dir,graph)
    result=read_json(graph/'summary.json')
    assert result['global_graph_adopted'] and result['global_map_exported']is False
    assert read_json(graph/'geometry_audit.json')['geometry_information']['dimension']==12
    assert not list(graph.glob('*.pcd'))
    # The historical native executable returns0 after retaining fault records.
    # Wrapper success must therefore depend on all frame/pose statuses.
    def faulting_native(argv,**kwargs):
        if argv[0]=='ldd':return fake_native(argv,**kwargs)
        fake_native(argv,**kwargs)
        directory=Path(argv[2]);report=read_json(directory/'summary.json')
        report['stations'][0]['complete']=False
        report['stations'][0]['accepted_frames']-=1
        write_json(directory/'summary.json',report)
        path=directory/plan['stations'][0]['id']/'poses.json'
        rows=read_json(path);rows[0]['accepted']=False;write_json(path,rows)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(pin.subprocess,'run',faulting_native)
    failed=tmp_path/'failed_local'
    assert pin.local(prepared/'input.json',native,failed)is False
    rejected=read_json(failed/'summary.json')
    assert rejected['all_stations_complete']is False and rejected['native_flow_returncode']==0
    assert len(rejected['stations'])==3
    def invalid_aux_pose_native(argv,**kwargs):
        result=fake_native(argv,**kwargs)
        if argv[0]=='ldd':return result
        path=Path(argv[2])/plan['stations'][0]['id']/'poses.json'
        rows=read_json(path);rows[-1]['reference_from_base'][0][0]=2.
        write_json(path,rows)
        return result
    monkeypatch.setattr(pin.subprocess,'run',invalid_aux_pose_native)
    invalid=tmp_path/'invalid_aux_pose'
    assert pin.local(prepared/'input.json',native,invalid)is False
    assert read_json(invalid/'summary.json')['pose_coverage'][0][
        'every_geometry_pose_finite_rigid_and_base_composition_valid']is False
    # A dynamic last pre-stop window must reject the complete experiment only
    # after retaining raw/ledger for every visit, without replacing its bias.
    w[(it>=24_300_000_000)&(it<=24_600_000_000),0]=.1
    dynamic=tmp_path/'dynamic_settle_inputs'
    assert pin.prepare(source,prior,REPO/'src/global_relocalization/config/competition.yaml',dynamic)is False
    bad_plan=read_json(dynamic/'input.json')
    assert len(bad_plan['stations'])==3 and bad_plan['all_settle_sensor_selfchecks_passed']is False
    assert len(read_json(dynamic/'raw_inventory.json'))==len(rt)
    assert all(Path(scan['path']).is_file()for s in bad_plan['stations']for scan in
               s['scans']+s['excluded_settle_prefix_scans'])
    assert read_json(dynamic/'calibration_protocol_check.json')['gyro_bias_rad_sec']==cal['gyro_bias_rad_sec']
    with pytest.raises(ValueError,match='settle selfchecks'):
        pin.local(dynamic/'input.json',native,tmp_path/'must_not_run_dynamic_native')
