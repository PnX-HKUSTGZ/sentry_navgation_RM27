#!/usr/bin/env python3
"""Post-export evaluation only. Ground truth cannot enter the optimizer."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from refine_sensor_prior import Rules, invert, normals, sha256, transform, voxel


def trajectory_scores(source, derived):
    capture = json.loads((source / 'capture.json').read_text())
    manifest = json.loads((derived / 'manifest.json').read_text())
    optimized = json.loads((derived / 'optimized_poses.json').read_text())
    gt = capture['gt']; times = np.array([x['stamp'] for x in gt]); truth = np.array([x['matrix'] for x in gt])
    source_times = np.array([x['stamp'] for x in capture['odom']]); raw = np.array([x['matrix'] for x in capture['odom']])
    refined = np.array([x['matrix'] for x in optimized]); world = np.array(manifest['T_world_camera_init']); sensor_base = invert(np.array(manifest['T_base_lidar']))
    valid = (source_times >= times[0]) & (source_times <= times[-1]); ids = np.searchsorted(times, source_times[valid]); ids = np.clip(ids, 1, len(times)-1)
    bracket = (times[ids] - times[ids-1]) <= .25; valid[np.flatnonzero(valid)[~bracket]] = False
    stamps = source_times[valid]; reference = np.tile(np.eye(4), (len(stamps), 1, 1))
    reference[:, :3, :3] = Slerp(times, Rotation.from_matrix(truth[:, :3, :3]))(stamps).as_matrix()
    for k in range(3): reference[:, k, 3] = np.interp(stamps, times, truth[:, k, 3])
    metrics = {}; rows = {}
    for label, poses in [('raw', raw), ('refined', refined)]:
        estimate = world @ poses[valid] @ sensor_base
        delta = estimate[:, :3, 3] - reference[:, :3, 3]; position = np.linalg.norm(delta, axis=1)
        angle = Rotation.from_matrix(np.transpose(reference[:, :3, :3], (0, 2, 1)) @ estimate[:, :3, :3]).magnitude() * 180 / np.pi
        metrics[label] = {'samples': len(position), 'unmatched': int((~valid).sum()), 'mean_position_m': float(position.mean()),
            'p95_position_m': float(np.quantile(position,.95)), 'max_position_m': float(position.max()), 'p95_so3_deg': float(np.quantile(angle,.95)),
            'max_so3_deg': float(angle.max()), 'strict_failed_samples': int(((position>.1)|(angle>2)).sum()), 'sensor_map_failed_samples': int(((position>.15)|(angle>3)).sum())}
        rows[label] = [{'stamp':float(t),'position_m':float(p),'rotation_deg':float(a)}for t,p,a in zip(stamps,position,angle)]
    return {'scope':'post-export evaluation only; same original raw-prior first SE3 gauge; all common samples; no GT feedback',
        'source_capture_sha256':sha256(source/'capture.json'),'derived_map_sha256':sha256(derived/'map.pcd'),'metrics':metrics,'trajectory_samples':rows}


def heldout_sensor_scores(source, derived):
    capture=json.loads((source/'capture.json').read_text());raw=np.array([x['matrix']for x in capture['odom']]);stamps=np.array([x['stamp']for x in capture['odom']])
    poses=json.loads((derived/'optimized_poses.json').read_text());correction=np.array([x['correction']for x in poses]);optimized=correction@raw
    key=np.load(derived/'optimized_keyframes.npz')['indices'];rules=Rules();rows=[]
    def cloud(i):return np.load(source/'registered_scans'/f'{i:06d}.npy').astype(float)
    def metrics(points, target):
        tree,n,good=normals(target,rules);d,idx=tree.query(points);v=(d<=rules.correspondence_m)&good[idx]
        error=np.einsum('ij,ij->i',points[v]-target[idx[v]],n[idx[v]])
        return {'points':len(points),'plane_supported':int(v.sum()),'coverage':float((d<=rules.correspondence_m).mean()),
            'plane_rmse_m':float(np.sqrt(np.mean(error**2)))if len(error)else None,'capped_distance_rmse_m':float(np.sqrt(np.mean(np.minimum(d,rules.correspondence_m)**2)))}
    for i in range(0,len(raw),17):
        # These source frames were neither own keyframes nor members of ANY
        # +/-2-frame keyframe target submap used to derive graph measurements.
        if min(abs(key-i))<=rules.submap_half_frames:continue
        possible=[]
        for j in key:
            if abs(stamps[j]-stamps[i])<rules.loop_separation_sec:continue
            distance=np.linalg.norm(raw[i,:3,3]-raw[j,:3,3])
            if distance>rules.candidate_distance_m:continue
            angle=Rotation.from_matrix(raw[i,:3,:3].T@raw[j,:3,:3]).magnitude()
            if angle<=np.deg2rad(rules.candidate_rotation_deg):possible.append((distance,int(j)))
        if not possible:continue
        j=min(possible)[1];source_cloud=voxel(cloud(i),rules.match_leaf_m);frames=range(max(0,j-2),min(len(raw),j+3));before_target=voxel(np.concatenate([cloud(k)for k in frames]),rules.match_leaf_m)
        after_target=voxel(np.concatenate([transform(cloud(k),correction[k])for k in frames]),rules.match_leaf_m)
        before=metrics(source_cloud,before_target);after=metrics(transform(source_cloud,correction[i]),after_target)
        rows.append({'source_index':i,'target_keyframe_index':j,'before':before,'after':after})
    summary={}
    for name in ['before','after']:
        v=[x[name]for x in rows if x[name]['plane_rmse_m']is not None];summary[name]={'frames':len(v),'mean_plane_rmse_m':float(np.mean([x['plane_rmse_m']for x in v]))if v else None,
            'median_plane_rmse_m':float(np.median([x['plane_rmse_m']for x in v]))if v else None,'mean_capped_distance_rmse_m':float(np.mean([x['capped_distance_rmse_m']for x in v]))if v else None,'mean_coverage':float(np.mean([x['coverage']for x in v]))if v else None}
    return {'scope':'sensor-only independent non-key source frame holdout, excludes every frame within 2 indices of ANY keyframe; deterministic every17th frame; nearest original-LIO nonlocal keyframe candidate, not chosen by GT or residual improvement', 'summary':summary,'frames':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('derived',type=Path);a=p.parse_args()
    if not(a.derived/'complete.json').is_file():raise ValueError('derived run incomplete')
    gt=trajectory_scores(a.source,a.derived);(a.derived/'post_export_gt_evaluation.json').write_text(json.dumps(gt,indent=2)+'\n');print(json.dumps(gt['metrics'],indent=2),flush=True)
    held=heldout_sensor_scores(a.source,a.derived);(a.derived/'sensor_heldout_evaluation.json').write_text(json.dumps(held,indent=2)+'\n');print(json.dumps(held['summary'],indent=2),flush=True)


if __name__=='__main__':main()
