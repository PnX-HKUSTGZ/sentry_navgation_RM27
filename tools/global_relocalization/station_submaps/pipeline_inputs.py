"""Prepare arbitrary ordered visits and run the existing isolated native adapter."""
from pathlib import Path
import json
import re
import shlex
import subprocess
import sys
import time

import numpy as np
import yaml

from pipeline_common import HERE, REPO, Inputs, new_directory, read_json, write_json, safe_id, rigid, sha256, record_code
from gyro_turns import check_imu, calibrate, integrate, turn_ledger, CALIBRATION_RULES
from export_station_prior import write_pcd, read_binary_xyz

sys.path.insert(0, str(HERE.parent))
from export_dense_sensor_prior import fixed_extrinsic, Trajectory


def bag_messages(bag, topics):
    # Lazy ROS imports keep all numerical/interface tests ROS-independent.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=info['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in topics}
    if set(types) != set(topics):
        raise ValueError('requested raw sensor topic absent from bag')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        msg = deserialize_message(payload, types[topic])
        yield topic, int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec), msg


def frontend_parameters(profile):
    p = yaml.safe_load(Path(profile).read_text())['global_relocalization']['ros__parameters']
    if p['raw_scan_model'] != 'instantaneous':
        raise ValueError('existing adapter requires the verified instantaneous source model')
    mapping = {
        'range_filter': 'raw_range_filter', 'deterministic': 'raw_deterministic',
        'imu_rotation_prior': 'raw_imu_rotation_prior', 'numerical_recovery': 'raw_numerical_recovery',
        'stationary_translation_prior': 'raw_stationary_translation_prior',
        'stationary_translation_sigma_floor': 'raw_stationary_translation_sigma_floor',
        'stationary_duration': 'raw_calibration_sec', 'submap_scans': 'raw_submap_scans',
        'leaf_size': 'raw_local_leaf_size', 'max_correspondence': 'raw_local_max_correspondence',
        'max_iterations': 'raw_local_max_iterations', 'max_fitness': 'raw_local_max_fitness',
        'min_inlier_ratio': 'raw_local_min_inlier_ratio', 'min_range': 'raw_min_range',
        'max_range': 'raw_max_range', 'gyro_noise_density': 'raw_gyro_noise_density',
        'rotation_sigma_floor': 'raw_rotation_sigma_floor',
    }
    result = {name: p[key] for name, key in mapping.items()}
    # Defaults of the pinned native RawFrontendParams; record explicitly.
    result.update(stationary_min_samples=40, retry_iterations=15, max_base_linear_speed=.35,
                  max_base_translation_step=.12, max_imu_gap=.035, max_scan_gap=.35,
                  min_geometry_eigen_ratio=.001, min_information_eigen_ratio=1e-6)
    if (result['max_correspondence'] != .3 or result['stationary_duration'] != 2.
            or result['stationary_translation_prior'] is not True):
        raise ValueError('station experiment requires the frozen .3m/2s/soft-base-prior profile')
    return result, p['raw_cloud_topic'], p['imu_topic'], p['lidar_frame']


def validate_source_frames(lidar_frames, imu_frames, expected_lidar):
    # Verified simulation convention: distinct names but co-located axes;
    # equality of the source fixed extrinsics is checked separately.
    if set(lidar_frames) != {expected_lidar} or set(imu_frames) != {'imu_link'} or not expected_lidar:
        raise ValueError('raw sensor frame changed/empty or differs from supported fixed source convention')
    return {'lidar':expected_lidar, 'imu':'imu_link',
            'convention':'configured LiDAR frame and simulator imu_link; equal verified fixed extrinsics'}


def station_windows(metadata):
    if not metadata:
        raise ValueError('no station visits')
    result, seen, previous_end = [], set(), None
    for row in metadata:
        sid = safe_id(row['station_id'])
        if sid in seen or row['status'] != 'complete':
            raise ValueError('duplicate/incomplete station visit')
        seen.add(sid)
        # Explicit allowlist: control XYZ, station_xy and measured_yaw never propagate.
        keep = {key: row[key] for key in (
            'station_id', 'waypoint_index', 'kind', 'label', 'commanded_speed_rad_sec',
            'capture_start_ros_ns', 'capture_end_ros_ns', 'rotation_command_start_ros_ns',
            'stop_command_ros_ns', 'settle_zero_command_ros_ns',
            'settle_start_sensor_stamp_ns', 'settle_end_sensor_stamp_ns', 'requested_settle_sec')}
        for key in ['capture_start_ros_ns', 'capture_end_ros_ns', 'rotation_command_start_ros_ns',
                    'stop_command_ros_ns', 'settle_zero_command_ros_ns',
                    'settle_start_sensor_stamp_ns', 'settle_end_sensor_stamp_ns']:
            if isinstance(keep[key], bool) or not isinstance(keep[key], int):
                raise ValueError('integer ns metadata required: ' + key)
        lo, hi = keep['capture_start_ros_ns'], keep['capture_end_ros_ns']
        zero, begin, end, rotate, stop = [keep[k] for k in [
            'settle_zero_command_ros_ns', 'settle_start_sensor_stamp_ns',
            'settle_end_sensor_stamp_ns', 'rotation_command_start_ros_ns', 'stop_command_ros_ns']]
        if (not lo <= zero < rotate < stop <= hi or not zero < begin <= end <= rotate
                or keep['requested_settle_sec'] < 2.5 or end - begin < 2_500_000_000
                or previous_end is not None and lo < previous_end):
            raise ValueError('invalid/overlapping visit or insufficient sensor-stamped settle')
        previous_end = hi
        result.append(keep)
    return result


def geometry_after_settle(ledger, settle_end_ns):
    """Predeclare stationary support, preserving raw braking/settle records."""
    ledger = {**ledger, 'scans': [dict(row) for row in ledger['scans']]}
    prefix, geometry = [], []
    for row in ledger['scans']:
        included = row['stamp_ns'] > settle_end_ns
        row['participates_stationary_geometry'] = included
        if included:
            row['geometry_ordinal'] = len(geometry)
            geometry.append(row)
        else:
            if row['role'] != 'aux_pre':
                raise ValueError('settle prefix overlaps a gyro full-turn interval')
            row['role'] = 'aux_settle_prefix'
            row['exclusion_reason'] = 'before measured 2.5s settle completion; approach/braking outside stationary model'
            prefix.append(row)
    if not geometry:
        raise ValueError('no stationary geometry remains after declared settle')
    ledger.update(role_counts={key:sum(x['role']==key for x in ledger['scans'])
                               for key in sorted({x['role'] for x in ledger['scans']})},
                  geometry_input_frames=len(geometry), excluded_settle_prefix_frames=len(prefix),
                  geometry_start_rule='first raw header strictly after settle_end_sensor_stamp_ns',
                  settle_end_sensor_stamp_ns=int(settle_end_ns),
                  local_frontend_dependency='all post-settle geometry scans including both turns/overshoot; '
                                            'pre-settle raw retained separately, not fitted or fused')
    return ledger


def bind_raw_prior(capture_dir, capture, manifest, inputs):
    """The inherited gauge/extrinsics must belong to this exact source capture."""
    capture_dir = Path(capture_dir).resolve()
    if Path(manifest['source_capture_directory']).resolve() != capture_dir:
        raise ValueError('raw-prior manifest belongs to a different capture directory')
    expected = manifest['source_sha256']
    required = {'capture.json', 'mapping_params.yaml', 'manifest.json', 'mapping_binary_provenance.json'}
    if not required.issubset(expected):
        raise ValueError('raw-prior original source hash inventory incomplete')
    for name, digest in expected.items():
        if Path(name).name != name:
            raise ValueError('raw-prior source hash is not a direct capture artifact')
        inputs.add(capture_dir / name, digest)
    if (manifest['source_completed_waypoints'] != capture['completed_waypoints']
            or manifest['source_total_waypoints'] != capture['total_waypoints']
            or manifest['source_failure'] is not None):
        raise ValueError('raw-prior completion counters differ from source capture')
    if not np.allclose(rigid(manifest['T_map_world']), np.eye(4), rtol=0, atol=1e-12):
        raise ValueError('this world-gauge adapter requires the recorded identity T_map_world')


def bind_protocol_inventory(capture_dir, capture, windows, inputs):
    """Check planned visit identity/completeness without using planned XYZ."""
    capture_dir = Path(capture_dir)
    protocol_path = inputs.add(capture_dir/'survey_route_protocol.json')
    route = read_json(inputs.add(capture_dir/'route.json'))
    protocol = read_json(protocol_path)
    project = lambda rows: [(x['id'],x['waypoint_index'],x['kind']) for x in rows]
    observed = [(x['station_id'],x['waypoint_index'],x['kind']) for x in windows]
    if (protocol.get('schema') != 'ordered_mapping_survey_v1'
            or protocol.get('preserve_waypoint_order') is not True
            or route.get('preserve_waypoint_order') is not True
            or route['survey_route_sha256'] != sha256(protocol_path)
            or project(protocol['stations']) != observed
            or project(route['protocol_stations']) != observed
            or len(route['waypoints']) != capture['total_waypoints']):
        raise ValueError('completed ordered visit inventory differs from frozen protocol')
    if windows[0]['kind'] != 'initial' or windows[-1]['kind'] != 'closure':
        raise ValueError('ordered initial and final closure visits must both be retained')


def prepare(capture_dir, raw_prior, profile, output, bag=None):
    start_wall = time.monotonic()
    capture_dir, raw_prior = Path(capture_dir).resolve(), Path(raw_prior).resolve()
    bag = Path(bag).resolve() if bag else capture_dir / 'raw_bag'
    out = new_directory(output)
    inputs = Inputs()
    record_code(inputs)
    source_path = inputs.add(capture_dir / 'capture.json')
    manifest_path = inputs.add(raw_prior / 'manifest.json')
    params_path = inputs.add(capture_dir / 'mapping_params.yaml')
    inputs.add(profile)
    for path in [Path(__file__), HERE / 'gyro_turns.py', HERE / 'pipeline_common.py',
                 REPO / 'src/global_relocalization/include/global_relocalization/stationary_raw_frontend.hpp',
                 REPO / 'src/global_relocalization/src/stationary_raw_frontend.cpp']:
        inputs.add(path)
    capture, manifest = read_json(source_path), read_json(manifest_path)
    if (capture.get('failure') is not None or capture.get('completed_waypoints') != capture.get('total_waypoints')
            or not capture.get('total_waypoints') or manifest.get('capture_complete') is not True
            or manifest.get('ground_truth_used_for_scan_fusion') is not False):
        raise ValueError('capture/prior incomplete or non-sensor fusion provenance')
    bind_raw_prior(capture_dir, capture, manifest, inputs)
    # The final raw-prior manifest and rosbag metadata must exist before any bag
    # file is opened/hashed. Never consume an ongoing capture.
    bag_info = yaml.safe_load(inputs.add(bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    for path in sorted(bag.rglob('*')):
        if path.is_file():
            inputs.add(path)
    source_extrinsic = fixed_extrinsic(yaml.safe_load(params_path.read_text()))
    if not np.array_equal(source_extrinsic, np.eye(4)):
        raise ValueError('existing native carried-gauge adapter supports the verified identity sensor/IMU only')
    bs = rigid(manifest['T_base_lidar'])
    if not np.allclose(bs, rigid(manifest['T_base_imu']), rtol=0, atol=1e-12):
        raise ValueError('sensor/IMU frames differ from adapter convention')
    params, lidar_topic, imu_topic, expected_lidar = frontend_parameters(profile)
    expected_counts = {row['topic_metadata']['name']: int(row['message_count'])
                       for row in bag_info['topics_with_message_count']}
    if not expected_counts.get(lidar_topic) or not expected_counts.get(imu_topic):
        raise ValueError('closed bag metadata lacks complete raw sensor counts')
    windows = station_windows(capture['station_scans'])
    bind_protocol_inventory(capture_dir,capture,windows,inputs)
    times, gyro, acceleration, headers, imu_frames = [], [], [], [], set()
    for topic, ns, msg in bag_messages(bag, [lidar_topic, imu_topic]):
        if topic == imu_topic:
            imu_frames.add(msg.header.frame_id)
            times.append(ns)
            gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            acceleration.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        else:
            headers.append({'stamp_ns': ns, 'frame': msg.header.frame_id,
                            'raw_points': int(msg.width) * int(msg.height)})
    times = np.asarray(times, np.int64)
    gyro, acceleration = np.asarray(gyro), np.asarray(acceleration)
    check_imu(times, gyro, acceleration)
    source_frames = validate_source_frames({h['frame'] for h in headers}, imu_frames, expected_lidar)
    if len(times) != expected_counts[imu_topic] or len(headers) != expected_counts[lidar_topic]:
        raise ValueError('sensor messages read do not equal closed bag metadata counts')
    raw_ns = np.array([x['stamp_ns'] for x in headers], np.int64)
    if len(raw_ns) < 2 or np.any(np.diff(raw_ns) <= 0) or np.max(np.diff(raw_ns)) > 350_000_000:
        raise ValueError('raw LiDAR timestamp uniqueness/continuity failed')
    eligible = raw_ns[(raw_ns - times[0] >= 2_000_000_000) & (raw_ns <= times[-1])]
    if not len(eligible):
        raise ValueError('no earliest initial-calibration scan with complete IMU support')
    calibration_ns = int(eligible[0])
    calibration = calibrate(times, gyro, acceleration, calibration_ns)
    write_json(out / 'calibration_protocol_check.json', calibration)
    if not calibration['stationary_selfcheck_passed']:
        raise ValueError('fixed earliest initial calibration failed; no alternative window search')
    if any(w['capture_start_ros_ns'] < calibration_ns for w in windows):
        raise ValueError('station starts before trusted calibration')
    gt, gq = integrate(times, gyro, calibration)
    np.savez(out / 'continuous_gyro.npz', stamps_ns=gt, sensor_quaternions_xyzw=gq,
             bias_rad_sec=calibration['gyro_bias_rad_sec'])
    write_json(out / 'imu_samples.json', np.column_stack([times * 1e-9, gyro, acceleration]).tolist())
    np.savez(out / 'raw_imu_samples.npz', stamps_ns=times, angular_velocity=gyro, linear_acceleration=acceleration)
    imu_source_path = out/'raw_imu_samples.npz'
    imu_source_sha = sha256(imu_source_path)
    selected, station_rows, ledgers, seen = {calibration_ns}, [], [], set()
    inventory = [{'raw_index': i, **h, 'role': 'nonstation', 'station_id': None,
                  'used_for_initial_calibration':h['stamp_ns']==calibration_ns} for i, h in enumerate(headers)]
    for w in windows:
        indices = np.flatnonzero((raw_ns >= w['capture_start_ros_ns']) & (raw_ns < w['capture_end_ros_ns']))
        if not len(indices):
            raise ValueError('visit has no raw LiDAR')
        ledger = geometry_after_settle(turn_ledger(w, raw_ns[indices], gt, gq, bs),
                                      w['settle_end_sensor_stamp_ns'])
        # Sensor proof of the commanded settle interval. This never
        # supplies a new bias or orientation: the actual native trusted initial
        # snapshot remains the only calibration used by every local frontend.
        settle_check = calibrate(times, gyro, acceleration, w['settle_end_sensor_stamp_ns'])
        ledger['settle_sensor_selfcheck'] = settle_check
        ledger['settle_check_used_to_reestimate_bias_or_orientation'] = False
        ledger.update(station_id=w['station_id'], waypoint_index=w['waypoint_index'], kind=w['kind'])
        ledgers.append(ledger)
        scans, excluded_prefix = [], []
        for index, role in zip(indices, ledger['scans']):
            ns = int(raw_ns[index])
            if ns in seen:
                raise ValueError('raw scan assigned to multiple visits')
            seen.add(ns)
            selected.add(ns)
            inventory[index].update(station_id=w['station_id'], role=role['role'],
                                    participates_stationary_geometry=role['participates_stationary_geometry'])
            row = {'stamp': ns * 1e-9, **headers[index], 'role': role['role'], 'raw_index': int(index),
                   'participates_stationary_geometry':role['participates_stationary_geometry']}
            (scans if role['participates_stationary_geometry'] else excluded_prefix).append(row)
        station_rows.append({'id': w['station_id'], 'source_label': w['label'],
                             'waypoint_index': w['waypoint_index'], 'kind': w['kind'],
                             'start_ns': w['capture_start_ros_ns'], 'end_ns': w['capture_end_ros_ns'],
                             'expected_raw_frames': len(scans), 'scans': scans,
                             'expected_capture_raw_frames':len(scans)+len(excluded_prefix),
                             'excluded_settle_prefix_scans':excluded_prefix,
                             'geometry_start_rule':ledger['geometry_start_rule'],
                             'settle_end_sensor_stamp_ns':w['settle_end_sensor_stamp_ns']})
    write_json(out / 'turn_ledger.json', {'stations': ledgers, 'expected_station_count': len(windows),
               'all_stations_complete': all(x['complete'] for x in ledgers),
               'all_settle_sensor_selfchecks_passed': all(x['settle_sensor_selfcheck']['stationary_selfcheck_passed'] for x in ledgers),
               'control_pose_fields_used': False, 'all_raw_tails_retained': True})
    write_json(out / 'raw_inventory.json', inventory)
    raw_dir = out / 'raw'
    raw_dir.mkdir()
    saved = {}
    from sensor_msgs_py import point_cloud2
    for _, ns, msg in bag_messages(bag, [lidar_topic]):
        if ns not in selected:
            continue
        data = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        xyz = np.column_stack([data[key].reshape(-1) for key in ['x', 'y', 'z']]).astype('<f4')
        # Raw nonfinite points must survive extraction; native range filtering owns them.
        path = raw_dir / ('raw_%d.pcd' % ns)
        header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                  'COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
                  % (len(xyz), len(xyz)))
        with path.open('wb') as stream:
            stream.write(header.encode('ascii'))
            stream.write(xyz.tobytes())
        if ns in saved:
            raise ValueError('duplicate raw scan during extraction')
        saved[ns] = {'path': str(path), 'sha256': sha256(path)}
    if set(saved) != selected:
        raise ValueError('not all selected raw frames were extracted')
    for row in station_rows:
        for scan in row['scans'] + row['excluded_settle_prefix_scans']:
            scan.update(saved[scan['stamp_ns']])
    calibration_scan = {**headers[int(np.searchsorted(raw_ns, calibration_ns))],
                        'stamp': calibration_ns * 1e-9, **saved[calibration_ns]}
    plan = {'schema_version': 2, 'GT_used_by_estimator': False, 'all_station_windows_retained': True,
            'expected_station_count': len(station_rows), 'imu_samples_json': str(out / 'imu_samples.json'),
            'params': params, 'sensor_from_imu': source_extrinsic.tolist(), 'base_from_sensor': bs.tolist(),
            'calibration_source_sha256': imu_source_sha,
            'calibration_source':{'kind':'complete_raw_imu_npz_v1','path':str(imu_source_path),
                                  'sha256':imu_source_sha,
                                  'schema':['stamps_ns','angular_velocity','linear_acceleration']},
            'calibration_rule': CALIBRATION_RULES, 'calibration_scan': calibration_scan,
            'global_calibration_is_actual_raw_scan': True, 'stations': station_rows,
            'source_raw_bag_frame_count': len(headers), 'source_capture': str(source_path),
            'source_raw_prior_manifest': str(manifest_path),
            'source_input_sha256': inputs.hashes,
            'turn_ledger_path': str(out / 'turn_ledger.json'),
            'source_frames':source_frames,
            'all_gyro_turns_complete': all(x['complete'] for x in ledgers),
            'all_settle_sensor_selfchecks_passed':all(x['settle_sensor_selfcheck']['stationary_selfcheck_passed'] for x in ledgers)}
    write_json(out / 'input.json', plan)
    inputs.save(out, stage='prepare', raw_lidar=len(headers), raw_imu=len(times),
                closed_bag_sensor_counts_match=True,
                selected_raw_frames=len(selected), elapsed_sec=time.monotonic() - start_wall,
                all_gyro_turns_complete=plan['all_gyro_turns_complete'])
    return plan['all_gyro_turns_complete'] and plan['all_settle_sensor_selfchecks_passed']


def native_dependencies(native, inputs):
    """Distinguish actual DT_NEEDED libraries from link-time template inputs."""
    inspected = subprocess.run(['ldd',str(native)],capture_output=True,text=True,check=False)
    if inspected.returncode != 0 or 'not found' in inspected.stdout:
        raise ValueError('native dynamic dependency resolution failed')
    runtime = {}
    for line in inspected.stdout.splitlines():
        match = re.search(r'^\s*(\S+)\s+=>\s+(.+?)\s+\(0x[0-9a-fA-F]+\)',line)
        if match and match[1].startswith(('libsmall_gicp','libexperimental_station_frontend')):
            path = inputs.add(match[2])
            runtime[match[1]] = str(path)
    link = inputs.add(native.parent/'CMakeFiles'/ (native.name+'.dir') / 'link.txt')
    candidates,adapter_inputs = [],[]
    for word in shlex.split(link.read_text()):
        candidate=Path(word)
        if candidate.name.startswith(('libsmall_gicp','libexperimental_station_frontend')):
            path=inputs.add(candidate if candidate.is_absolute() else native.parent/candidate)
            (candidates if candidate.name.startswith('libsmall_gicp') else adapter_inputs).append(str(path))
    if not candidates:
        raise ValueError('native build link record has no pinned small_gicp input')
    return {'ldd':inspected.stdout,'runtime_registration_libraries':runtime,
            'link_record':str(link),'small_gicp_link_inputs':candidates,
            'experimental_frontend_link_inputs':adapter_inputs,
            'template_code_may_be_static_as_needed_runtime_dependency_absent':not any(
                k.startswith('libsmall_gicp')for k in runtime)}


def local(input_path, native, output, allow_support=False):
    inputs = Inputs()
    record_code(inputs)
    input_path, native = inputs.artifact(input_path), inputs.add(native)
    plan = read_json(input_path)
    if (not plan.get('all_gyro_turns_complete') or not plan.get('all_settle_sensor_selfchecks_passed')
            or plan.get('GT_used_by_estimator') is not False):
        raise ValueError('complete sensor-only two-turn input and all stationary settle selfchecks required')
    inputs.artifact(plan['imu_samples_json'], input_path.parent)
    inputs.artifact(plan['turn_ledger_path'], input_path.parent)
    inputs.artifact(plan['calibration_source']['path'],input_path.parent)
    if inputs.hashes[plan['calibration_source']['path']] != plan['calibration_source_sha256']:
        raise ValueError('trusted calibration source hash is not the actual sensor bundle')
    inputs.add(plan['calibration_scan']['path'], plan['calibration_scan']['sha256'])
    for s in plan['stations']:
        for scan in s['scans']:
            inputs.add(scan['path'], scan['sha256'])
    # Hash the isolated adapter library/generated source, without rebuilding or installing.
    for path in sorted(native.parent.glob('libexperimental_station_frontend*')):
        inputs.add(path)
    for directory in (native.parent/'generated',native.parent/'frontend/generated'):
        for path in sorted(directory.glob('*')):
            if path.is_file():
                inputs.add(path)
    dependencies = native_dependencies(native,inputs)
    out = new_directory(output)
    write_json(out/'native_dependencies.json',dependencies)
    runtime_input=input_path
    if allow_support:
        runtime_input=out/'mapping_runtime_input.json'
        write_json(runtime_input,{**plan,'diagnostic_mapping_next_scan_retry':True})
    with (out / 'native.log').open('w') as log:
        import os
        environment = dict(os.environ, OMP_NUM_THREADS='2')
        completed = subprocess.run([str(native), str(runtime_input), str(out / 'native')],
                                   stdout=log, stderr=subprocess.STDOUT, check=False, env=environment)
    report_path = out / 'native/summary.json'
    report = read_json(report_path) if report_path.exists() else {'stations': []}
    ids = [x['id'] for x in plan['stations']]
    rows = report['stations']
    complete = (completed.returncode == 0 and [x['id'] for x in rows] == ids
                and all(s['complete'] and s['accepted_frames'] == p['expected_raw_frames']
                        for s, p in zip(rows, plan['stations'])))
    support_source_complete = bool(completed.returncode == 0 and [x['id'] for x in rows] == ids)
    pose_checks = []
    if len(rows) == len(ids):
        for s, p in zip(rows, plan['stations']):
            poses = read_json(out / 'native' / s['id'] / 'poses.json')
            coverage_ok = (len(poses) == len(p['scans']) and all(
                x['accepted'] and abs(round(x['stamp'] * 1e9) - y['stamp_ns']) <= 1
                for x, y in zip(poses, p['scans'])))
            valid_pose = True
            for pose in poses:
                if allow_support and pose.get('accepted') is not True:
                    # A rejected candidate is never a source of pose support.
                    continue
                try:
                    sensor = rigid(pose['reference_from_sensor'])
                    base = rigid(pose['reference_from_base'])
                    valid_pose &= bool(np.allclose(base,sensor@np.linalg.inv(rigid(plan['base_from_sensor'])),rtol=0,atol=1e-8))
                except (ValueError,TypeError,KeyError):
                    valid_pose = False
            ok = coverage_ok and valid_pose
            pose_checks.append({'id': s['id'], 'all_input_stamps_and_accepted_poses_match':coverage_ok,
                                'every_geometry_pose_finite_rigid_and_base_composition_valid':valid_pose})
            complete &= ok
            if allow_support:
                from pipeline_support import audit_station
                try:
                    audit_station(p, poses, s, rigid(plan['base_from_sensor']))
                except (ValueError,TypeError,KeyError) as error:
                    support_source_complete = False
                    pose_checks[-1]['support_source_rejected_reason'] = str(error)
    snapshot_path = out / 'native/trusted_calibration.json'
    if snapshot_path.exists():
        snapshot = read_json(snapshot_path)
        expected = read_json(inputs.artifact(input_path.parent / 'calibration_protocol_check.json'))
        bias_matches = np.allclose(snapshot['gyro_bias'], expected['gyro_bias_rad_sec'], rtol=0, atol=1e-12)
        calibration_source_matches = (snapshot['source_sha256'] == plan['calibration_source_sha256']
            and abs(round(snapshot['calibration_stamp']*1e9)-plan['calibration_scan']['stamp_ns']) <= 1)
        complete &= bool(bias_matches and calibration_source_matches and snapshot['generated_from_actual_accepted_initial_scan'])
        support_source_complete &= bool(bias_matches and calibration_source_matches and snapshot['generated_from_actual_accepted_initial_scan'])
    else:
        bias_matches = False
        complete = False
        support_source_complete = False
    write_json(out / 'summary.json', {'expected_station_count': len(ids),
               'all_stations_complete': bool(complete), 'native_flow_returncode': completed.returncode,
               'native_output': str(out / 'native'), 'pose_coverage': pose_checks,
               'native_and_python_calibration_bias_agree': bool(bias_matches),
               'support_mode_requested':bool(allow_support),
               'support_source_complete':bool(allow_support and support_source_complete),
               'all_registrations_accepted':bool(complete),
               'native_runtime_input':str(runtime_input),
               'native_runtime_input_sha256':sha256(runtime_input),
               'native_runtime_input_role':'runtime-only opt-in copy; original prepared input unchanged' if allow_support else 'original prepared input',
               'diagnostic_mapping_next_scan_retry_requested':bool(allow_support),
               'source_raw_frames':sum(len(s['scans'])for s in plan['stations']),
               'registration_accepted_frames':sum(s['accepted_frames']for s in rows),
               'no_GT_or_LIO_poses_entered_local_frontend': True,
               'source_input': str(input_path), 'stations': rows})
    inputs.save(out, stage='local', all_stations_complete=bool(complete),
                support_mode_requested=bool(allow_support),
                support_source_complete=bool(allow_support and support_source_complete))
    return bool(support_source_complete if allow_support else complete)


def sensor_seed(input_path, local_dir, output, support_dir=None):
    inputs = Inputs()
    record_code(inputs)
    input_path = inputs.artifact(input_path)
    plan = read_json(input_path)
    local_dir = Path(local_dir).resolve()
    summary = read_json(inputs.artifact(local_dir / 'summary.json'))
    support_summary = None
    if support_dir is not None:
        from pipeline_support import SUPPORT_RULES
        support_dir=Path(support_dir).resolve()
        support_summary=read_json(inputs.artifact(support_dir/'support.json'))
        if (support_summary.get('pose_support_complete') is not True
                or support_summary.get('rules') != SUPPORT_RULES
                or Path(support_summary['source_input']).resolve()!=input_path
                or Path(support_summary['source_local']).resolve()!=local_dir
                or support_summary.get('GT_used') is not False
                or support_summary.get('crossstation_target_used') is not False):
            raise ValueError('explicit sensor-only support provenance/input binding required')
        for path,digest in support_summary['source_sha256'].items():
            inputs.add(path,digest)
        if [s['id']for s in support_summary['stations']]!=[s['id']for s in plan['stations']]:
            raise ValueError('support ordered station inventory differs')
    elif not summary.get('all_stations_complete'):
        raise ValueError('all local station scans must pass before building LIO-only seeds')
    if not plan.get('all_settle_sensor_selfchecks_passed'):
        raise ValueError('all declared settle sensor checks must pass')
    native = Path(summary['native_output'])
    capture = read_json(inputs.add(plan['source_capture'], plan['source_input_sha256'][plan['source_capture']]))
    manifest = read_json(inputs.add(plan['source_raw_prior_manifest'], plan['source_input_sha256'][plan['source_raw_prior_manifest']]))
    lio = Trajectory(capture['odom'])  # GT, commands and requested XYZ are never accessed.
    world = rigid(manifest['T_world_camera_init'])
    rows = []
    for ordinal,station in enumerate(plan['stations']):
        folder = native / station['id']
        supported = support_summary['stations'][ordinal] if support_summary else None
        poses_path = (inputs.artifact(supported['poses_path'],support_dir) if supported
                      else inputs.artifact(folder/'poses.json',local_dir))
        poses = read_json(poses_path)
        if (len(poses) != len(station['scans'])
                or any(abs(round(p['stamp']*1e9)-s['stamp_ns'])>1 for p,s in zip(poses,station['scans']))
                or not all(p.get('pose_supported') if supported else p['accepted'] for p in poses)):
            raise ValueError('native pose coverage changed')
        if supported and (not poses[0]['registration_accepted']
                or supported['counts']['raw_frames']!=len(poses)
                or supported['counts']['pose_supported_frames']!=len(poses)
                or supported['counts']['registration_accepted_frames']!=sum(p['accepted']for p in poses)
                or supported['counts']['fused_frames']!=sum(p['eligible_for_fusion']for p in poses)):
            raise ValueError('supported seed complete inventory/counts differ')
        if supported:
            from pipeline_support import validate_supported_rows
            validate_supported_rows(station,poses,supported['counts'],rigid(plan['base_from_sensor']))
        first = rigid(poses[0]['reference_from_sensor'])
        first_lio, association = lio.at(station['scans'][0]['stamp_ns'])
        if first_lio is None:
            raise ValueError('first raw station scan lacks bounded LIO association')
        camera_local = first_lio @ np.linalg.inv(first)
        cloud = (inputs.add(supported['cloud'],support_summary['source_sha256'][supported['cloud']])
                 if supported else inputs.artifact(folder/'local_cloud.pcd',local_dir))
        rows.append({'id': station['id'], 'cloud': str(cloud),
                     'waypoint_index': station['waypoint_index'], 'kind': station['kind'],
                     'first_raw_stamp_ns': station['scans'][0]['stamp_ns'],
                     'original_LIO_first_sensor_pose': first_lio.tolist(),
                     'local_first_sensor_pose': first.tolist(),
                     'camera_init_from_local': camera_local.tolist(),
                     'map_from_local_initial': (world @ camera_local).tolist(),
                     'source_frames': len(poses), 'LIO_timestamp_association': association,
                     'source_capture_frames':station['expected_capture_raw_frames'],
                     'excluded_settle_prefix_frames':len(station['excluded_settle_prefix_scans']),
                     'geometry_start_rule':station['geometry_start_rule'],
                     'poses_path': str(poses_path),
                     'pose_support_mode':bool(supported),
                     'support_counts':supported['counts'] if supported else None})
    if support_summary:
        totals={key:sum(s['counts'][key]for s in support_summary['stations'])
                for key in support_summary['counts']}
        if totals!=support_summary['counts']:
            raise ValueError('supported aggregate counts differ')
    out = new_directory(output)
    inputs.verify()
    write_json(out / 'sensor_seed.json', {
        'schema_version': 2, 'sensor_only': True, 'GT_post_evaluation_files_read': False,
        'query_results_read': False, 'expected_station_count': len(rows),
        'T_world_camera_init': world.tolist(), 'stations': rows,
        'source_input': str(input_path), 'sources_sha256': inputs.hashes,
        'world_camera_gauge': 'inherited single original gauge; cancels in relative seeds',
        'pose_support_mode':support_summary is not None,
        'source_support':str(support_dir) if support_summary else None,
        'support_counts':support_summary['counts'] if support_summary else None,
        'repeated_position_visits_merged': False})
    inputs.save(out, stage='seed', expected_station_count=len(rows))
    return True
