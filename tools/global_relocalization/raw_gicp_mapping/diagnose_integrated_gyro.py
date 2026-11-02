#!/usr/bin/env python3
"""Read-only sensor-only SO(3) integration, followed by separate GT evaluation."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read_topic(bag,topic,type_name):
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]));typ=get_message(type_name)
 while reader.has_next():
  _,data,_=reader.read_next();msg=deserialize_message(data,typ);yield msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,msg

def stats(values):
 a=np.asarray(values);return {'count':len(a),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(np.max(a)),'mean':float(np.mean(a)),'rmse':float(np.sqrt(np.mean(a*a)))}
def errors(estimate,truth):
 z=np.broadcast_to([0.,0.,1.],(len(estimate),3));tilt=np.rad2deg(np.arccos(np.clip(np.sum(estimate.apply(z)*truth.apply(z),axis=1),-1,1)));e=(estimate.inv()*truth).magnitude()*180/np.pi;angles=estimate.as_euler('xyz')-truth.as_euler('xyz');angles=(angles+np.pi)%(2*np.pi)-np.pi;return {'so3_deg':e,'body_up_tilt_deg':tilt,'roll_error_deg':np.rad2deg(angles[:,0]),'pitch_error_deg':np.rad2deg(angles[:,1]),'yaw_error_deg':np.rad2deg(angles[:,2])}

def main():
 p=argparse.ArgumentParser();p.add_argument('--bag',type=Path,required=True);p.add_argument('--lio-capture',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False);start=time.monotonic()
 # Fixed rule and thresholds are declared before opening any GT data.
 rules={'window_rule':'first continuous2.0seconds beginning at first rawIMU header stamp; no search/GT selection','duration_sec':2.,'minimum_samples':40,'max_gyro_norm_rad_sec':.08,'max_vector_gyro_std_rad_sec':.015,'max_vector_accel_std_m_sec2':.2,'gravity_magnitude_m_sec2':9.81,'gravity_norm_tolerance_m_sec2':.8,'max_imu_gap_sec':.035}
 (a.output/'declared_window_rule.json').write_text(json.dumps(rules,indent=2)+'\n')
 times=[];gyro=[];acc=[]
 for stamp,m in read_topic(a.bag,'/livox/imu','sensor_msgs/msg/Imu'):
  times.append(stamp);gyro.append([m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]);acc.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
 times=np.array(times,dtype=np.int64);gyro=np.array(gyro);acc=np.array(acc);window=times<=times[0]+2000000000;w=gyro[window];g=acc[window];bias=w.mean(0);am=g.mean(0);stdw=float(np.sqrt(np.mean(np.sum((w-bias)**2,axis=1))));stda=float(np.sqrt(np.mean(np.sum((g-am)**2,axis=1))));maxw=float(np.max(np.linalg.norm(w,axis=1)));gaps=np.diff(times)*1e-9
 checks={'duration':(times[window][-1]-times[0])*1e-9>=2.-1e-9,'samples':window.sum()>=40,'finite':bool(np.isfinite(w).all() and np.isfinite(g).all()),'gyro_norm':maxw<=.08,'gyro_std':stdw<=.015,'accel_std':stda<=.2,'gravity_norm':abs(np.linalg.norm(am)-9.81)<=.8,'continuous_full_imu':bool((gaps>0).all() and gaps.max()<=.035)}
 checks={k:bool(v)for k,v in checks.items()}
 calibration={'rule':rules,'first_stamp':float(times[0]*1e-9),'window_last_stamp':float(times[window][-1]*1e-9),'samples':int(window.sum()),'gyro_bias_rad_sec':bias.tolist(),'max_gyro_norm_rad_sec':maxw,'vector_gyro_std_rad_sec':stdw,'mean_acceleration_m_sec2':am.tolist(),'mean_acceleration_norm_m_sec2':float(np.linalg.norm(am)),'vector_acceleration_std_m_sec2':stda,'checks':checks,'sensor_selfcheck_stationary':all(checks.values()),'GT_read':False};(a.output/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n');print('sensor calibration',json.dumps(calibration),flush=True)
 if not all(checks.values()):raise RuntimeError('fixed initial stationary window failed; do not estimate bias or integrate')
 # Estimation stage reads original LIO orientation only; capture GT fields are ignored.
 capture=json.loads(a.lio_capture.read_text());lio=capture['odom'];lt=np.array([int(round(x['stamp']*1e9))for x in lio],dtype=np.int64);lr=Rotation.from_matrix(np.array([x['matrix']for x in lio])[:,:3,:3]);merged=np.unique(np.r_[times,lt]);relative=(merged-times[0])*1e-9;wt=np.column_stack([np.interp(relative,(times-times[0])*1e-9,gyro[:,i]-bias[i])for i in range(3)]);delta=.5*(wt[:-1]+wt[1:])*np.diff(relative)[:,None];increments=Rotation.from_rotvec(delta).as_quat();current=Rotation.identity();qs=[current.as_quat()]
 for q in increments:current=current*Rotation.from_quat(q);qs.append(current.as_quat())
 cumulative=Rotation.from_quat(qs);indices=np.searchsorted(merged,lt);seed_index=indices[0];all_rotation=lr[0]*cumulative[seed_index].inv()*cumulative;imu_i=np.searchsorted(merged,times);gyro_at_lio=all_rotation[indices];full_gyro=all_rotation[imu_i]
 np.savez(a.output/'sensor_only_integration.npz',imu_stamps_ns=times,imu_reference_quaternions_xyzw=full_gyro.as_quat(),lio_stamps_ns=lt,gyro_at_lio_quaternions_xyzw=gyro_at_lio.as_quat(),lio_quaternions_xyzw=lr.as_quat(),bias_rad_sec=bias)
 print('sensor integration complete, GT not opened, imu',len(times),'lio',len(lt),flush=True)
 # Evaluation begins only here. One initial orientation gauge is shared by both.
 gt_t=[];gt_q=[]
 for t,m in read_topic(a.bag,'/ground_truth/odometry','nav_msgs/msg/Odometry'):
  q=m.pose.pose.orientation;gt_t.append(t);gt_q.append([q.x,q.y,q.z,q.w])
 gt_t=np.array(gt_t,dtype=np.int64);gt_rotation=Rotation.from_quat(gt_q);base_sensor=Rotation.from_matrix(np.array(json.loads(a.manifest.read_text())['T_base_imu'])[:3,:3]);slerp=Slerp((gt_t-gt_t[0])*1e-9,gt_rotation)
 seed_truth=slerp([(lt[0]-gt_t[0])*1e-9])[0];world_reference=seed_truth*base_sensor*lr[0].inv()
 def evaluate(stamps,rot):
  valid=(stamps>=gt_t[0])&(stamps<=gt_t[-1]);t=stamps[valid];truth=slerp((t-gt_t[0])*1e-9);world=world_reference*rot[valid]*base_sensor.inv();return valid,errors(world,truth)
 valid_g,eg=evaluate(lt,gyro_at_lio);valid_l,el=evaluate(lt,lr);vf,ef=evaluate(times,full_gyro);np.savez(a.output/'post_evaluation_errors.npz',lio_stamps_ns=lt[valid_g],**{'gyro_'+k:v for k,v in eg.items()},**{'lio_'+k:v for k,v in el.items()},full_imu_stamps_ns=times[vf],**{'full_gyro_'+k:v for k,v in ef.items()})
 metrics=lambda e:{k:stats(np.abs(v))for k,v in e.items()};output={'sensor_only_estimator':True,'GT_used_for_bias_or_integration':False,'motion_acceleration_used_for_orientation':False,'initial_orientation':'original LIO first frame; identical one-time GT gauge for evaluation only','integration':'rightSO3 midpoint exponential at actual timestamps; full3axis gyro, no LIDAR orientation feedback','imu_count':len(times),'imu_duration_sec':float((times[-1]-times[0])*1e-9),'max_imu_gap_sec':float(gaps.max()),'lio_samples':len(lt),'lio_first_stamp':float(lt[0]*1e-9),'lio_last_stamp':float(lt[-1]*1e-9),'matched_lio_gyro':int(valid_g.sum()),'matched_lio_original':int(valid_l.sum()),'unmatched_lio':int((~valid_g).sum()),'full_imu_GT_matched':int(vf.sum()),'full_imu_GT_unmatched':int((~vf).sum()),'gyro_at_lio':metrics(eg),'original_lio':metrics(el),'gyro_full_IMU_interval':metrics(ef),'gyro_vs_lio_without_GT_SO3_deg':stats((gyro_at_lio.inv()*lr).magnitude()*180/np.pi),'elapsed_sec':time.monotonic()-start,'source_sha256':{str(f.resolve()):hashlib.sha256(f.read_bytes()).hexdigest()for f in [a.bag/'metadata.yaml',a.lio_capture,a.manifest,Path(__file__)]}};(a.output/'summary.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))
if __name__=='__main__':main()
