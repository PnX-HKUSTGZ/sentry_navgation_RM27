#!/usr/bin/env python3
"""Post-only local relative-motion evaluation; never used by the frontend."""
from pathlib import Path
import json,sys,numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from evaluation import score_pose,interpolate
from export_dense_sensor_prior import Trajectory
ROOT=Path('tools/global_relocalization/data/optimization_20261026');OUT=ROOT/'station_local_submaps_v1';INPUT=ROOT/'station_local_submaps_v1_inputs'
def pcd(path):
 with path.open('rb')as f:
  while not f.readline().startswith(b'DATA binary'):pass
  return np.frombuffer(f.read(),dtype='<f4').reshape(-1,3).astype(float)
def voxel(a,leaf=.05):
 key=np.floor(a/leaf).astype(np.int64);u,inv,c=np.unique(key,axis=0,return_inverse=True,return_counts=True);v=np.zeros((len(u),3));np.add.at(v,inv,a);return(v/c[:,None]).astype(np.float32)
def save(path,a):
 h=f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(a)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(a)}\nDATA binary\n'
 with path.open('wb')as f:f.write(h.encode());f.write(np.asarray(a,dtype='<f4').tobytes())
def transform(a,t):return a@t[:3,:3].T+t[:3,3]
def stats(x):
 a=np.array(x);return{'count':len(a),'p95':float(np.percentile(a,95)),'max':float(a.max()),'mean':float(a.mean())}
def summarize(scores):return{'translation_3d_m':stats([s['translation_3d_m']for s in scores]),'orientation_so3_deg':stats([s['orientation_so3_deg']for s in scores])}
source=json.loads((ROOT/'mapping06/capture.json').read_text());gt=[(r['stamp'],np.array(r['matrix']))for r in source['gt']];lio=Trajectory(source['odom']);plan=json.loads((INPUT/'input.json').read_text());bs=np.array(plan['base_from_sensor']);invbs=np.linalg.inv(bs);summaries=[];all_new=[];all_old=[];total_gt_missing=0
for station in plan['stations']:
 folder=OUT/station['id'];poses=json.loads((folder/'poses.json').read_text());assert len(poses)==len(station['scans']);estimated0=np.array(poses[0]['reference_from_sensor']);old0,mode=lio.at(station['scans'][0]['stamp_ns']);truth0=interpolate(gt,poses[0]['stamp'],.25);newbase0=estimated0@invbs;oldbase0=old0@invbs;new_scores=[];old_scores=[];truth_motion=[];missing=[];parts=[];seed_matrices=[]
 for scan,estimate in zip(station['scans'],poses):
  assert abs(scan['stamp']-estimate['stamp'])<1e-9
  old,_=lio.at(scan['stamp_ns']);truth=interpolate(gt,scan['stamp'],.25)
  if old is None or truth is None:missing.append(scan['stamp']);continue
  points=pcd(Path(scan['path']));r=np.linalg.norm(points,axis=1);points=points[np.isfinite(points).all(1)&(r>=.5)&(r<=30)];parts.append(transform(points,np.linalg.inv(old0)@old));relative_truth=np.linalg.inv(truth0)@truth;truth_motion.append(np.linalg.norm(relative_truth[:3,3]));old_scores.append({'stamp':scan['stamp'],**score_pose(np.linalg.inv(oldbase0)@(old@invbs),relative_truth)})
  if estimate['accepted']:new_scores.append({'stamp':scan['stamp'],**score_pose(np.linalg.inv(newbase0)@(np.array(estimate['reference_from_sensor'])@invbs),relative_truth)})
 old_cloud=voxel(np.concatenate(parts));save(folder/'lio_shape_initial_sensor.pcd',old_cloud);cloud_path=folder/('local_cloud.pcd'if all(r['accepted']for r in poses)else'incomplete_prefix_cloud.pcd');new_cloud=transform(pcd(cloud_path),np.linalg.inv(estimated0));save(folder/'frontend_shape_initial_sensor.pcd',new_cloud);dnew=cKDTree(old_cloud).query(new_cloud,workers=2)[0];dold=cKDTree(new_cloud).query(old_cloud,workers=2)[0]
 worldseed=np.array(json.loads((ROOT/'mapping06_raw_prior/manifest.json').read_text())['T_world_camera_init'])@old0@np.linalg.inv(estimated0)
 result={'id':station['id'],'label':station['source_label'],'source_frames':len(station['scans']),'accepted_frames':sum(r['accepted']for r in poses),'all_frames_preserved':len(poses)==len(station['scans']),'GT_missing':len(missing),'GT_missing_stamps':missing,'frontend_local_relative':summarize(new_scores),'original_LIO_local_relative':summarize(old_scores),'actual_GT_relative_base_translation_max_m':float(max(truth_motion)),'frontend_to_LIO_shape_nn_m':stats(dnew),'LIO_to_frontend_shape_nn_m':stats(dold),'shape_comparison_note':'NNdifference alone does not decide correctness; pose-relativeGT scores are evaluated separately after estimation','frontend_input_GT':False,'map_from_local_reference_initial_from_original_LIO':worldseed.tolist(),'seed_note':'initialization only for later fullSE3 crossstation registration, no GT; not a solved globalpose'};(folder/'post_evaluation.json').write_text(json.dumps(result,indent=2)+'\n');(folder/'relative_scores.json').write_text(json.dumps({'frontend':new_scores,'original_LIO':old_scores},indent=2)+'\n');summaries.append(result);all_new.extend(new_scores);all_old.extend(old_scores);total_gt_missing+=len(missing);print(station['id'],'newmax',result['frontend_local_relative']['translation_3d_m']['max'],result['frontend_local_relative']['orientation_so3_deg']['max'],'oldmax',result['original_LIO_local_relative']['translation_3d_m']['max'],result['original_LIO_local_relative']['orientation_so3_deg']['max'],flush=True)
report={'scope':'post-estimation local-relative motion and localshape only; no globalmap/no232query','all10_station_windows_retained':len(summaries)==10,'station_frames_total':sum(s['source_frames']for s in summaries),'accepted_frames_total':sum(s['accepted_frames']for s in summaries),'all_GT_matched':total_gt_missing==0,'GT_missing_total':total_gt_missing,'frontend_local_relative_aggregate':summarize(all_new),'original_LIO_local_relative_aggregate':summarize(all_old),'stations':summaries};(OUT/'post_evaluation_summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k!='stations'},indent=2))
