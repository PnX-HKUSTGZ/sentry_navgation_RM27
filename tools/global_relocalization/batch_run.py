#!/usr/bin/env python3
"""Sequential independent-LIO trials against a frozen map, retaining failures.
Run with tools/global_relocalization/run_batch.bash after start_simulation.bash.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import platform
import shutil
import yaml
import psutil

ROOT=Path(__file__).resolve().parents[2]


def failure_record(trial, reason, timeout=False, input_mode='lio', timeout_source=None):
    missing_cost={'pids': [], 'measurement_valid': False, 'cpu_user_delta_sec': None,
        'cpu_sys_delta_sec': None, 'wall_sec': None, 'cpu_percent': None,
        'core_seconds': None, 'peak_rss_bytes': None}
    return {**trial, 'input_mode':input_mode, 'action_window_exists':False,
        'initial_stamp_ns': None, 'final_stamp_ns': None,
        'action': {'success': False, 'reason': reason, 'timeout': timeout, 'timeout_source':timeout_source, 'pose_stamp_ns': None, 'pose': None},
        'timing_sec': {'total_ros': None, 'total_wall': None, **{name: {'ros': None, 'wall': None}
            for name in ['static','sweep','retrieval','gicp','confirmation','stop_settle','search_total']}},
        'algorithm_process': dict(missing_cost), 'lio_pipeline': dict(missing_cost),
        'gazebo_process_load': dict(missing_cost), 'trial_evaluator_process':dict(missing_cost),
        'diagnostic_recorder_process':dict(missing_cost),
        'frontend_process':{**missing_cost,'included_in_algorithm':input_mode=='raw_imu'},
        'raw_input_observation':{'counts_since_client_start':{'lidar':None,'imu':None},
            'counts_during_action':{'lidar':None,'imu':None},'readiness_used_ground_truth':False},
        'rtf': None, 'status_events': [],
        'environment': {'hostname': platform.node(), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'gz_partition': os.environ.get('GZ_PARTITION')}}


def stop(process):
    try:
        os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL)
            process.wait()
    # The launch parent can exit before a Gazebo/ROS descendant; finish the owned group.
    try:
        os.killpg(process.pid,signal.SIGTERM)
    except ProcessLookupError:
        pass


def launch(command,log):
    stream=log.open('w')
    process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    stream.close()
    return process


def reset_pose(pose):
    half=pose['yaw']/2
    request=(f'name: "rm27_sentry", position: {{x: {pose["x"]}, y: {pose["y"]}, z: {pose["z"]}}}, '
             f'orientation: {{x: 0, y: 0, z: {math.sin(half)}, w: {math.cos(half)}}}')
    command=['gz','service','-s','/world/default/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','10000','--req',request]
    result=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=15)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError('Gazebo pose reset failed: '+result.stdout)
    return command,result.stdout


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',required=True)
    parser.add_argument('--manifest',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--trials',default=str(ROOT/'tools/global_relocalization/config/trials_120.json'))
    parser.add_argument('--params',default=str(ROOT/'src/global_relocalization/config/global.yaml'))
    parser.add_argument('--limit',type=int,help='pilot only; never silently count as complete batch')
    parser.add_argument('--wall-timeout',type=float,default=180)
    parser.add_argument('--ready-timeout',type=float,default=90)
    parser.add_argument('--input-mode',choices=['lio','raw_imu'],default='lio')
    parser.add_argument('--record-trials',default='',help='Comma-separated trial IDs, or * for pilot raw sensor bag capture')
    parser.add_argument('--physical-precheck',help='Directory with independent static physical checks for every selected raw trial')
    parser.add_argument('--simulation-audit',help='Passing stream/graph audit from this live isolated session')
    args=parser.parse_args()
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    cases=json.loads(Path(args.trials).read_text());trials=cases['trials'][:args.limit] if args.limit else cases['trials']
    parameters=yaml.safe_load(Path(args.params).read_text())['global_relocalization']['ros__parameters']
    expected_input_mode='raw_imu' if args.input_mode=='raw_imu' else 'registered'
    if parameters.get('input_mode','registered') != expected_input_mode:
        raise ValueError('CLI input mode differs from frozen algorithm profile')
    search_side=parameters.get('search_side','all');boundary=parameters.get('side_boundary_x',0.)
    requested_prior=cases.get('search_prior',{})
    if requested_prior and (requested_prior['search_side']!=search_side or requested_prior['side_boundary_x']!=boundary):
        raise ValueError('Frozen trial prior differs from algorithm parameters')
    if any((search_side=='negative_x' and not t['requested_pose']['x']<boundary) or
           (search_side=='positive_x' and not t['requested_pose']['x']>boundary) for t in trials):
        raise ValueError('Trial start outside the fixed half-field prior')
    manifest=json.loads(Path(args.manifest).read_text());map_path=Path(args.map).resolve()
    if hashlib.sha256(map_path.read_bytes()).hexdigest()!=manifest['map_sha256']:
        raise ValueError('map SHA differs from frozen manifest')
    metadata={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'command':sys.argv,
        'trials_source':str(Path(args.trials).resolve()),'trial_seed':cases['seed'],'planned_trials':len(trials),
        'full_planned_trials':len(cases['trials']),'pilot':bool(args.limit) or 'pilot' in cases.get('purpose','').lower(),'map_path':str(map_path),
        'map_sha256':manifest['map_sha256'],'manifest_path':str(Path(args.manifest).resolve()),
        'parameters':Path(args.params).read_text(),'parameters_sha256':hashlib.sha256(Path(args.params).read_bytes()).hexdigest(),
        'limits':{'wall_timeout':args.wall_timeout,'ready_timeout':args.ready_timeout},'commands':[]}
    metadata['search_prior']={'search_side':search_side,'side_boundary_x':boundary,
        'source':requested_prior.get('source','none'),'fixed_for_entire_batch':True,
        'ground_truth_used_to_select_candidates':False}
    metadata['environment']={'hostname':platform.node(),'platform':platform.platform(),
        'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'gz_partition':os.environ.get('GZ_PARTITION'),
        'gazebo_gui':False}
    metadata['input_mode']=args.input_mode
    if args.input_mode=='raw_imu':
        if not args.simulation_audit:
            raise ValueError('raw batch requires --simulation-audit for the current isolated session')
        audit_path=Path(args.simulation_audit).resolve()
        audit_data=audit_path.read_bytes()
        audit=json.loads(audit_data)
        if not audit.get('passed') or any(audit.get('environment',{}).get(key)!=os.environ.get(env)
            for key,env in [('ros_domain_id','ROS_DOMAIN_ID'),('gz_partition','GZ_PARTITION')]):
            raise ValueError('simulation stream audit failed or belongs to another isolated session')
        (output/'simulation_audit.json').write_bytes(audit_data)
        metadata['simulation_audit']={'path':str(audit_path),'sha256':hashlib.sha256(audit_data).hexdigest()}
        if not args.physical_precheck:
            raise ValueError('raw batch requires --physical-precheck evidence before localization')
        precheck=Path(args.physical_precheck).resolve()
        checks=json.loads((precheck/'checks.json').read_text())
        by_id={row['trial_id']:row for row in checks}
        if len(by_id)!=len(checks):
            raise ValueError('duplicate trial ID in physical precheck')
        for trial in trials:
            check=by_id.get(trial['trial_id'])
            if not check or check['requested_pose']!=trial['requested_pose'] or not check['physical_valid']:
                raise ValueError('raw trial lacks matching valid independent physical precheck: '+trial['trial_id'])
        metadata['physical_precheck']={'directory':str(precheck),'matched_selected_trials':len(trials),
            'localization_success_used_for_selection':False,'files':{}}
        for name in ['summary.json','manifest.json','checks.json']:
            data=(precheck/name).read_bytes()
            target=output/'physical_precheck'/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(data)
            metadata['physical_precheck']['files'][name]=hashlib.sha256(data).hexdigest()
    record_trials=set(args.record_trials.split(',')) if args.record_trials else set()
    metadata['raw_bag_capture']={'selected_trials':sorted(record_trials),
        'estimator_replay_topics':['/livox/lidar','/livox/imu','/tf_static','/clock'],
        'scoring_bag_topics':['/ground_truth/odometry'],
        'ground_truth_bag_separate_from_estimator_replay':True}
    metadata['resource_groups']={'algorithm_process':'algorithm including in-process raw frontend' if args.input_mode=='raw_imu' else 'algorithm',
        'frontend_process':'included_in_algorithm; no separate CPU/RSS attribution' if args.input_mode=='raw_imu' else 'legacy LIO group',
        'lio_pipeline':'disabled; null measurements' if args.input_mode=='raw_imu' else 'Point-LIO and adapters',
        'gazebo_process_load':'private-domain Gazebo server', 'trial_evaluator_process':'GT observation/client; excluded from estimator'}
    execution_sources=[Path(__file__), ROOT/'src/global_relocalization/scripts/run_trial.py',
            ROOT/'src/global_relocalization/scripts/evaluate_pose.py',
            ROOT/'src/global_relocalization/scripts/instantaneous_cloud_converter.py',
            ROOT/'tools/global_relocalization/odometry_probe.py',
            ROOT/'src/global_relocalization/launch/algorithm.launch.py',
            ROOT/'src/global_relocalization/launch/raw_imu.launch.py',
            ROOT/'src/global_relocalization/launch/simulation.launch.py',
            ROOT/'tools/global_relocalization/audit_runtime_parameters.py',
            ROOT/'tools/global_relocalization/audit_simulation.py',
            ROOT/'tools/global_relocalization/summarize_success_cohort.py',
            ROOT/'tools/global_relocalization/environment.bash',
            ROOT/'tools/global_relocalization/start_simulation.bash',
            ROOT/'tools/global_relocalization/prepare_simulation.bash',
            ROOT/'src/global_relocalization/CMakeLists.txt']
    execution_sources.extend(sorted((ROOT/'src/global_relocalization/src').glob('*.cpp')))
    execution_sources.extend(sorted((ROOT/'src/global_relocalization/include/global_relocalization').glob('*.hpp')))
    metadata['execution_source_sha256']={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
        for path in execution_sources}
    for path in execution_sources:
        snapshot=output/'source_snapshot'/path.relative_to(ROOT)
        snapshot.parent.mkdir(parents=True,exist_ok=True)
        snapshot.write_bytes(path.read_bytes())
    from ament_index_python.packages import get_package_prefix
    metadata['executable_provenance']={}
    executables=[('global_relocalization','global_relocalization_node')]
    if args.input_mode=='lio':
        executables.append(('point_lio','pointlio_mapping'))
    for package,executable in executables:
        path=(Path(get_package_prefix(package))/'lib'/package/executable).resolve()
        metadata['executable_provenance'][package]={'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        snapshot=output/'binary_snapshot'/package/executable
        snapshot.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,snapshot)
    core_path=(Path(get_package_prefix('global_relocalization'))/'lib/libglobal_relocalization_core.so').resolve()
    metadata['executable_provenance']['global_relocalization_core']={'path':str(core_path),'sha256':hashlib.sha256(core_path.read_bytes()).hexdigest()}
    shutil.copy2(core_path,output/'binary_snapshot'/core_path.name)
    metadata['simulation_loaded_binary_provenance']=[]
    for process in psutil.process_iter(['pid','cmdline']):
        try:
            command=process.info['cmdline'] or []
            if not command or not command[0].startswith('gz sim '):
                continue
            environment=process.environ()
            if environment.get('ROS_DOMAIN_ID')!=os.environ.get('ROS_DOMAIN_ID') or environment.get('GZ_PARTITION')!=os.environ.get('GZ_PARTITION'):
                continue
            for mapping in process.memory_maps():
                if 'velocity-control-system' not in mapping.path and 'bullet-featherstone-plugin' not in mapping.path:
                    continue
                path=Path(mapping.path).resolve()
                metadata['simulation_loaded_binary_provenance'].append({'pid':process.pid,'path':str(path),
                    'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
                shutil.copy2(path,output/'binary_snapshot'/path.name)
        except (psutil.NoSuchProcess,psutil.AccessDenied):
            continue
    frozen_sources={'trials.json':Path(args.trials), 'global.yaml':Path(args.params),
        'map_manifest.json':Path(args.manifest)}
    if args.input_mode=='lio':
        frozen_sources['lio.yaml']=ROOT/'src/global_relocalization/config/lio.yaml'
    controller_manifest=ROOT/'tools/global_relocalization/.build/robot_velocity.json'
    if controller_manifest.exists():
        frozen_sources['simulation_controller.json']=controller_manifest
        controller=json.loads(controller_manifest.read_text())
        description=Path(controller['robot_description'])
        if hashlib.sha256(description.read_bytes()).hexdigest()!=controller['robot_description_sha256']:
            raise ValueError('simulation description SHA differs from controller provenance')
        frozen_sources['simulation_robot.urdf']=description
    metadata['frozen_input_sha256']={}
    for name,path in frozen_sources.items():
        data=path.read_bytes();(output/name).write_bytes(data)
        metadata['frozen_input_sha256'][name]=hashlib.sha256(data).hexdigest()
    (output/'batch_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    launch_file='raw_imu.launch.py' if args.input_mode=='raw_imu' else 'algorithm.launch.py'
    algorithm_command=['ros2','launch',str(ROOT/'src/global_relocalization/launch'/launch_file),f'map_path:={map_path}',f'manifest_path:={output/"map_manifest.json"}',f'params_file:={output/"global.yaml"}']
    evaluator_command=['/usr/bin/python3',str(ROOT/'src/global_relocalization/scripts/evaluate_pose.py'),'--ros-args','-p',f'output_prefix:={output/"pose_errors"}','-p',f'map_manifest_path:={Path(args.manifest).resolve()}','-p','use_sim_time:=true']
    algorithm=launch(algorithm_command,output/'algorithm.log')
    evaluator=launch(evaluator_command,output/'evaluator.log')
    metadata['commands']=[algorithm_command,evaluator_command]
    lio=None
    client=None
    recorders=[]
    try:
        audit_command=['/usr/bin/python3',str(ROOT/'tools/global_relocalization/audit_runtime_parameters.py'),
            '--params',str(output/'global.yaml'),'--output',str(output/'runtime_parameters.json'),
            '--timeout',str(args.ready_timeout)]
        metadata['commands'].append(audit_command)
        with (output/'runtime_parameters.log').open('w') as audit_log:
            try:
                audit_returncode=subprocess.run(audit_command,stdout=audit_log,stderr=subprocess.STDOUT,
                    timeout=args.ready_timeout+10).returncode
            except subprocess.TimeoutExpired:
                audit_returncode=124
                audit_log.write('\nRuntime parameter audit outer timeout\n')
        for trial in trials:
            folder=output/trial['trial_id'];folder.mkdir()
            pose=trial['requested_pose'];record={**trial}
            try:
                if audit_returncode:
                    raise RuntimeError('frozen runtime parameter audit failed; see runtime_parameters.json/log')
                command,reset_output=reset_pose(pose)
                (folder/'reset.log').write_text(reset_output)
                time.sleep(3)
                if trial['trial_id'] in record_trials or '*' in record_trials:
                    if shutil.disk_usage(output).free < 5*1024**3:
                        raise RuntimeError('Insufficient free disk for diagnostic raw bag; minimum 5 GiB')
                    raw_command=['ros2','bag','record','-o',str(folder/'raw_sensors'),
                        '/livox/lidar','/livox/imu','/tf_static','/clock']
                    scoring_command=['ros2','bag','record','-o',str(folder/'ground_truth_scoring'),'/ground_truth/odometry']
                    recorders=[launch(raw_command,folder/'raw_bag.log'),launch(scoring_command,folder/'scoring_bag.log')]
                    (folder/'bag_commands.json').write_text(json.dumps([raw_command,scoring_command],indent=2)+'\n')
                    time.sleep(1)
                lio_command=[]
                if args.input_mode=='lio':
                    lio_command=['ros2','launch',str(ROOT/'src/global_relocalization/launch/lio.launch.py'),
                        f'lio_params:={output/"lio.yaml"}']
                    lio=launch(lio_command,folder/'lio.log')
                client_command=['/usr/bin/python3',str(ROOT/'src/global_relocalization/scripts/run_trial.py'),
                    '--output',str(folder/'trial.json'),'--trial-id',trial['trial_id'],'--trial-index',str(trial['trial_index']),
                    '--requested-pose',json.dumps(pose),'--timeout',str(args.wall_timeout),'--ready-timeout',str(args.ready_timeout),
                    '--input-mode',args.input_mode,'--raw-cloud-topic',parameters.get('raw_cloud_topic','/livox/lidar'),
                    '--raw-imu-topic',parameters.get('imu_topic','/livox/imu')]
                (folder/'commands.json').write_text(json.dumps([command,lio_command,client_command],indent=2)+'\n')
                client=launch(client_command,folder/'client.log')
                outer_timeout=False
                try:
                    client.wait(timeout=args.wall_timeout+args.ready_timeout+20)
                except subprocess.TimeoutExpired:
                    outer_timeout=True
                    stop(client)
                if (folder/'trial.json').exists():
                    record=json.loads((folder/'trial.json').read_text())
                else:
                    record=failure_record(trial,'outer_wall_timeout' if outer_timeout else 'client_crash',
                        outer_timeout,args.input_mode,'outer_supervisor' if outer_timeout else None)
            except Exception as error:
                record=failure_record(trial,str(error),False,args.input_mode)
            finally:
                if client:
                    stop(client);client=None
                if lio:
                    stop(lio);lio=None
                for recorder in recorders:
                    stop(recorder)
                recorders=[]
            with (output/'trials.jsonl').open('a') as stream:
                stream.write(json.dumps(record,allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
            print(json.dumps({'trial_id':trial['trial_id'],'action':record['action'],'timing_sec':record.get('timing_sec',{})}),flush=True)
            time.sleep(1)
    finally:
        if client:
            stop(client)
        if lio:
            stop(lio)
        for recorder in recorders:
            stop(recorder)
        stop(evaluator);stop(algorithm)
        metadata['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        metadata['observed_trials']=sum(1 for _ in (output/'trials.jsonl').open()) if (output/'trials.jsonl').exists() else 0
        (output/'batch_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    aggregate=['/usr/bin/python3',str(ROOT/'src/global_relocalization/scripts/evaluate_pose.py'),'--aggregate',
        '--trials',str(output/'trials.jsonl'),'--pose-errors',str(output/'pose_errors.jsonl'),
        '--output-prefix',str(output/'results'),'--expected-trials',str(len(trials))]
    aggregate_code=subprocess.run(aggregate).returncode
    if aggregate_code==0:
        cohort=['/usr/bin/python3',str(ROOT/'tools/global_relocalization/summarize_success_cohort.py'),
            '--report',str(output/'results.summary.json'),'--output',str(output/'results.success_cohort.json')]
        return subprocess.run(cohort).returncode
    return aggregate_code


if __name__=='__main__':
    raise SystemExit(main())
