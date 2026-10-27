#!/usr/bin/env python3
"""Sensor-only turn check using existing continuous gyro integration/bias.
Does not run the stationary frontend or permit an invalid local calibration.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from export_dense_sensor_prior import sha256


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('audit',type=Path);p.add_argument('integration',type=Path);p.add_argument('prior',type=Path);a=p.parse_args()
    original=json.loads((a.audit/'summary.json').read_text());cal=json.loads((a.integration/'calibration.json').read_text());data=np.load(a.integration/'sensor_only_integration.npz');t=data['imu_stamps_ns']*1e-9;r=Rotation.from_quat(data['imu_reference_quaternions_xyzw']).as_matrix()
    gravity=np.array(cal['mean_acceleration_m_sec2']);gravity/=np.linalg.norm(gravity);z=np.array([0.,0,1]);quat=np.r_[np.cross(gravity,z),1+gravity@z];quat/=np.linalg.norm(quat);r0=Rotation.from_quat(quat).as_matrix()
    # Remove the previous integration's arbitrary initial LIO gauge. This
    # single fixed change uses only the first validated accelerometer gravity.
    gauge=r0@r[0].T;bs=np.array(json.loads((a.prior/'manifest.json').read_text())['T_base_lidar'])
    body=gauge@r@bs[:3,:3].T;yaw=np.unwrap(Rotation.from_matrix(body).as_euler('xyz')[:,2]);rows=[]
    for s in original['stations']:
        lo,hi=s['first_raw_stamp'],s['last_raw_stamp'];turn=float(np.interp(hi,t,yaw)-np.interp(lo,t,yaw));rows.append({'label':s['label'],'start':lo,'end':hi,'first_scan_local_calibration_pass':s['calibration_at_first_scan']['passes_validated_frontend_calibration'],
            'continuous_global_calibration_gyro_yaw_rad':turn,'full_2pi_sensor_turn':turn>=2*np.pi,'calibration_endpoint_diagnostics':{k:v['passes_validated_frontend_calibration']for k,v in s['calibration_endpoint_diagnostics'].items()}})
    result={'scope':'sensor-only coverage recheck, no stationary frontend run or GT; continuous gyro integration uses one first-raw-IMU validated bias, initial attitude changed by one accelerometer-gravity gauge; body extrinsic only','global_calibration_selfcheck':cal['sensor_selfcheck_stationary'],
        'all_ten_turns_have_full_sensor_2pi':all(x['full_2pi_sensor_turn']for x in rows),'local_independent_reset_calibration_pass_count':original['calibration_first_scan_pass_count'],
        'global_bias_is_not_automatic_permission_to_reset_frontend_without_its_own_calibration':True,'stations':rows,'source_hashes':{str(x.resolve()):sha256(x)for x in [a.audit/'summary.json',a.integration/'sensor_only_integration.npz',a.integration/'calibration.json',a.prior/'manifest.json',Path(__file__).resolve()]}}
    path=a.audit/'continuous_gyro_station_coverage.json'
    if path.exists():raise FileExistsError(str(path))
    path.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
