#!/usr/bin/env python3
"""Read-only station-submap feasibility, not a frontend run or optimized map.
Every completed station is selected solely by existing label/status/time metadata.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from scipy.integrate import cumulative_trapezoid
from scipy.spatial import cKDTree
from scipy.sparse.csgraph import connected_components
from scipy.sparse import csr_matrix
import yaml
from audit_sensor_plane_identifiability import RULES
from export_dense_sensor_prior import Trajectory,VoxelAccumulator,fixed_extrinsic,sha256
from refine_sensor_prior import transform


def imu_stats(times,omega,acceleration,end,params):
    width=params['stationary_duration'];gap=params['max_imu_gap'];mask=(times>=end-width-gap)&(times<=end+1e-9);t=times[mask];w=omega[mask];a=acceleration[mask]
    if not len(t):return {'passes_validated_frontend_calibration':False,'reason':'no samples'}
    mw,ma=w.mean(axis=0),a.mean(axis=0);maxw=float(np.linalg.norm(w,axis=1).max());ws=float(np.sqrt(np.mean(np.sum((w-mw)**2,axis=1))));astd=float(np.sqrt(np.mean(np.sum((a-ma)**2,axis=1))));maxgap=float(np.diff(t).max())if len(t)>1 else None
    conditions={'minimum_samples':len(t)>=params['stationary_min_samples'],'duration':t[-1]-t[0]>=width-1e-9,'gyro_max':maxw<=params['stationary_max_gyro'],'gyro_vector_std':ws<=params['stationary_gyro_std'],
        'accel_vector_std':astd<=params['stationary_accel_std'],'gravity_norm':abs(np.linalg.norm(ma)-params['gravity_magnitude'])<=params['gravity_tolerance'],'imu_gap':maxgap is not None and maxgap<=gap}
    conditions={k:bool(v)for k,v in conditions.items()}
    return {'passes_validated_frontend_calibration':all(conditions.values()),'conditions':conditions,'samples':len(t),'first_stamp':float(t[0]),'last_stamp':float(t[-1]),'duration_sec':float(t[-1]-t[0]),'max_imu_gap_sec':maxgap,
        'max_gyro_norm_rad_sec':maxw,'gyro_vector_std_rad_sec':ws,'mean_gyro_rad_sec':mw.tolist(),'mean_acceleration_mps2':ma.tolist(),'mean_acceleration_norm_mps2':float(np.linalg.norm(ma)),'acceleration_vector_std_mps2':astd}


def audit(source,prior,frontend_diagnostic,output):
    if output.exists():raise FileExistsError(str(output))
    output.mkdir(parents=True);start=time.monotonic();capture=json.loads((source/'capture.json').read_text());manifest=json.loads((prior/'manifest.json').read_text());p=json.loads(frontend_diagnostic.read_text())['params'];params=yaml.safe_load((source/'mapping_params.yaml').read_text());extrinsic=fixed_extrinsic(params)
    stations=[{k:s[k]for k in ['label','status','start_sensor_stamp','end_sensor_stamp']}for s in capture['station_scans']]
    if len(stations)!=10 or any(s['status']!='complete'for s in stations):raise ValueError('expected all ten completed source stations')
    for s in stations:s['start_ns']=round(s['start_sensor_stamp']*1e9);s['end_ns']=round(s['end_sensor_stamp']*1e9)
    rules={'selection':'all ten source metadata stations; only label/status/start_sensor_stamp/end_sensor_stamp used','GT_control_positions_or_yaws_used':False,'front_end_run':False,
        'calibration':'exact validated frontend past-duration+max_imu_gap window at first turn scan; unchanged thresholds','frontend_params':p,'cloud_voxel_m':p['leaf_size'],
        'overlap':'unregistered full station clouds projected using original LIO ONLY to assess initial shared return coverage; no geometric registration or pose optimization','correspondence_m':p['max_correspondence'],
        'minimum_overlap_existing_gate':p['min_inlier_ratio'],'yaw_coverage_bins':24,'calibration_endpoint_offsets_sec':[0.,-.5,-1.,-2.,-3.],
        'full_circle_verification':'integrate measured gyro minus local calibration mean along measured gravity; no metadata measured_yaw used'}
    (output/'rules.json').write_text(json.dumps(rules,indent=2)+'\n');(output/'auditor_source.py').write_bytes(Path(__file__).read_bytes())
    files=[source/'capture.json',source/'mapping_params.yaml',prior/'manifest.json',frontend_diagnostic,Path(__file__).resolve(),Path(__file__).with_name('export_dense_sensor_prior.py').resolve()]
    files.extend(x for x in(source/'raw_bag').iterdir()if x.is_file());hashes={str(x.resolve()):sha256(x)for x in files};(output/'sources.json').write_text(json.dumps(hashes,indent=2)+'\n')
    trajectory=Trajectory(capture['odom']);world=np.array(manifest['T_world_camera_init']);clouds=[VoxelAccumulator(p['leaf_size'])for _ in stations];scans=[[]for _ in stations];imu=[];pending=[[]for _ in stations]
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2,Imu
    from sensor_msgs_py import point_cloud2
    metadata=yaml.safe_load((source/'raw_bag/metadata.yaml').read_text())['rosbag2_bagfile_information'];reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(source/'raw_bag'),storage_id=metadata['storage_identifier']),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=['/livox/lidar','/livox/imu']))
    raw_count=0
    while reader.has_next():
        topic,payload,_=reader.read_next();msg=deserialize_message(payload,Imu if topic=='/livox/imu'else PointCloud2);stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if topic=='/livox/imu':imu.append((stamp,[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]));continue
        raw_count+=1;indices=[i for i,s in enumerate(stations)if s['start_ns']-1<=stamp<=s['end_ns']+1]
        if not indices:continue
        a=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=False);q=np.column_stack([a[k].reshape(-1)for k in ['x','y','z']]);q=q[np.isfinite(q).all(axis=1)];r=np.linalg.norm(q,axis=1);q=q[(r>=p['min_range'])&(r<=p['max_range'])];pose,association=trajectory.at(stamp)
        if pose is None:raise ValueError('station scan lacks source LIO initialization')
        for i in indices:
            scans[i].append({'stamp_ns':stamp,'finite_in_range_points':len(q),'pose_association':association})
            pending[i].append(transform(q,world@pose@extrinsic))
            if len(pending[i])>=10:clouds[i].add(np.concatenate(pending[i]));pending[i]=[]
    imu.sort(key=lambda x:x[0]);ns=np.array([x[0]for x in imu],np.int64);times=ns*1e-9;omega=np.array([x[1]for x in imu]);acc=np.array([x[2]for x in imu])
    if np.any(np.diff(ns)<=0):raise ValueError('IMU stamps duplicated/nonmonotonic')
    np.savez(output/'raw_imu_samples.npz',stamps_ns=ns,omega=omega,acceleration=acc)
    reports=[]
    for i,s in enumerate(stations):
        if pending[i]:clouds[i].add(np.concatenate(pending[i]))
        if not scans[i]:raise ValueError('empty station')
        lidar=np.array([x['stamp_ns']for x in scans[i]],np.int64);first=lidar[0]*1e-9;last=lidar[-1]*1e-9;stats=imu_stats(times,omega,acc,first,p)
        calibration_alternatives={str(off):imu_stats(times,omega,acc,first+off,p)for off in rules['calibration_endpoint_offsets_sec']}
        use=(times>=first-p['max_imu_gap'])&(times<=last+p['max_imu_gap']);t,w=times[use],omega[use];gyro_bias=np.array(stats['mean_gyro_rad_sec']);gravity=np.array(stats['mean_acceleration_mps2']);gravity/=np.linalg.norm(gravity)
        angular=(w-gyro_bias)@gravity;integral=np.r_[0,cumulative_trapezoid(angular,t)];angle=np.interp(lidar*1e-9,t,integral);angle-=angle[0];norm_integral=float(np.trapz(np.linalg.norm(w-gyro_bias,axis=1),t))
        bins=np.floor(np.mod(angle,2*np.pi)/(2*np.pi/rules['yaw_coverage_bins'])).astype(int);hist=np.bincount(bins,minlength=rules['yaw_coverage_bins'])
        covered=ns[np.searchsorted(ns,lidar,side='left').clip(max=len(ns)-1)]>=lidar;station_imu=ns[(ns>=lidar[0]-35000000)&(ns<=lidar[-1]+35000000)]
        sensor_origin=world@trajectory.at(int(lidar[0]))[0]@extrinsic
        reports.append({**s,'raw_scan_count':len(scans[i]),'first_raw_stamp':first,'last_raw_stamp':last,'raw_scan_max_gap_sec':float(np.diff(lidar).max()*1e-9),
            'raw_imu_samples_covering_turn':len(station_imu),'imu_max_gap_sec':float(np.diff(station_imu).max()*1e-9),'all_scan_stamps_have_IMU_bracket':bool(lidar[0]>=ns[0]and lidar[-1]<=ns[-1]),
            'calibration_at_first_scan':stats,'calibration_endpoint_diagnostics':calibration_alternatives,'sensor_gyro_projected_angle_rad':float(angle[-1]),
            'sensor_gyro_norm_integral_with_margin_rad':norm_integral,'gyro_yaw_bin_counts':hist.tolist(),'populated_yaw_bins':int((hist>0).sum()),'voxel_station_points':len(clouds[i].points),
            'range_points_min_median_max':np.quantile([x['finite_in_range_points']for x in scans[i]],[0,.5,1]).tolist(),'LIO_initialized_sensor_origin_world':sensor_origin[:3,3].tolist()})
        np.save(output/f'station_{i:02d}_lio_initialized_cloud.npy',clouds[i].points)
    pair=[];trees=[cKDTree(c.points)for c in clouds];adj=np.eye(len(stations),dtype=bool)
    for i in range(len(stations)):
        for j in range(i+1,len(stations)):
            a,idx=trees[j].query(clouds[i].points);b,rev=trees[i].query(clouds[j].points);ra=float((a<=p['max_correspondence']).mean());rb=float((b<=p['max_correspondence']).mean());accepted=ra>=p['min_inlier_ratio']and rb>=p['min_inlier_ratio'];adj[i,j]=adj[j,i]=accepted
            mutual=np.arange(len(a))==rev[idx];shared=(a<=p['max_correspondence']);pair.append({'i':i,'j':j,'label_i':stations[i]['label'],'label_j':stations[j]['label'],'source_initial_origin_distance_m':float(np.linalg.norm(np.array(reports[i]['LIO_initialized_sensor_origin_world'])-reports[j]['LIO_initialized_sensor_origin_world'])),
                'source_to_target_coverage':ra,'target_to_source_coverage':rb,'minimum_directional_coverage':min(ra,rb),'mutual_shared_pairs':int((mutual&shared).sum()),
                'source_inlier_rmse_m':float(np.sqrt(np.mean(a[shared]**2)))if shared.any()else None,'both_meet_existing_065_local_overlap_gate':accepted})
    count,labels=connected_components(csr_matrix(adj),directed=False);result={'scope':'feasibility only; no frontend/registration/optimization/GT evaluation performed','station_count':len(stations),'raw_lidar_frames_in_bag':raw_count,'raw_imu_samples_in_bag':len(imu),
        'calibration_first_scan_pass_count':sum(x['calibration_at_first_scan']['passes_validated_frontend_calibration']for x in reports),'all_turns_complete_sensor_gyro_2pi':all(x['sensor_gyro_projected_angle_rad']>=2*np.pi for x in reports),
        'existing_065_overlap_graph_components':int(count),'overlap_component_labels':labels.tolist(),'stations':reports,'cross_station_pairs':pair,'wall_sec':time.monotonic()-start}
    for path,h in hashes.items():
        if sha256(path)!=h:raise ValueError('source changed: '+path)
    result['source_inputs_verified_unchanged']=True;(output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items()if k not in ['stations','cross_station_pairs']},indent=2),flush=True)
    for x in reports:print(x['label'],'scans',x['raw_scan_count'],'calpass',x['calibration_at_first_scan']['passes_validated_frontend_calibration'],'gyroturn',x['sensor_gyro_projected_angle_rad'],'yawbins',x['populated_yaw_bins'],'points',x['voxel_station_points'],flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['source','prior','frontend_diagnostic','output']:p.add_argument(name,type=Path)
    a=p.parse_args();audit(a.source.resolve(),a.prior.resolve(),a.frontend_diagnostic.resolve(),a.output.resolve())


if __name__=='__main__':main()
