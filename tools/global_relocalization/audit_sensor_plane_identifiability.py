#!/usr/bin/env python3
"""Finite source-block planar identifiability audit; never a free-space score.

Surfaces use individual original mapping scans and unchanged sensor-only LIO.
No GT, ideal map, world mesh, occupancy or requested pose enters this audit.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from export_dense_sensor_prior import raw_frames, fixed_extrinsic, sha256, Trajectory
from refine_sensor_prior import invert, transform

RULES={'horizontal_unique_beams':359,'horizontal_step_rad':6.2831852/359,
    'vertical_beams':192,'vertical_min_rad':-.12601276984444446,'vertical_step_rad':(.9637707965111111+.12601276984444446)/191,
    'minimum_range_m':.5,'maximum_range_m':30.,'mapping_source_frame_stride':10,'source_time_block_sec':60.,
    'local_fit_grid_width':3,'local_fit_grid_stride':3,'range_noise_sigma_m':.002,'plane_rms_noise_multiplier':3.,
    'plane_max_noise_multiplier':6.,'minimum_middle_largest_eigen_ratio':.01,'maximum_support_cell_diameter_m':.2,
    'query_keyframes':12,'query_horizontal_stride':12,'query_vertical_stride':8,'surface_search_radius_m':.75,
    'minimum_ray_plane_abs_cosine':.1,'minimum_common_blocks':2,'discriminative_sigma_multiplier':3.,'native_duplicate_bin_policy':'first original point, including 0/360 endpoint'}


def native_grid(points):
    h=RULES['horizontal_unique_beams'];v=RULES['vertical_beams'];grid=np.full((v,h,3),np.nan)
    points=points[np.isfinite(points).all(axis=1)];r=np.linalg.norm(points,axis=1);points=points[(r>=RULES['minimum_range_m'])&(r<=RULES['maximum_range_m'])]
    r=np.linalg.norm(points,axis=1);a=np.rint(np.mod(np.arctan2(points[:,1],points[:,0]),2*np.pi)/RULES['horizontal_step_rad']).astype(int)%h
    e=np.rint((np.arctan2(points[:,2],np.linalg.norm(points[:,:2],axis=1))-RULES['vertical_min_rad'])/RULES['vertical_step_rad']).astype(int)
    good=(e>=0)&(e<v);a,e,points=a[good],e[good],points[good];bins=e*h+a;_,first=np.unique(bins,return_index=True)
    grid.reshape(-1,3)[bins[first]]=points[first]
    return grid


def local_cells(points):
    grid=native_grid(points);v,h=grid.shape[:2];rr,cc=np.meshgrid(np.arange(1,v-1,3),np.arange(1,h-1,3),indexing='ij');rr,cc=rr.ravel(),cc.ravel()
    samples=np.stack([grid[rr+dy,cc+dx]for dy in [-1,0,1]for dx in [-1,0,1]],axis=1)
    valid=np.isfinite(samples).all(axis=(1,2));samples=samples[valid]
    if not len(samples):return np.empty((0,4,3)),np.empty((0,3)),{'valid_3x3_patches':0,'plane_patches':0,'support_cells':0}
    center=samples.mean(axis=1);d=samples-center[:,None,:];cov=np.transpose(d,(0,2,1))@d/9;values,vectors=np.linalg.eigh(cov);n=vectors[:,:,0]
    residual=np.einsum('nki,ni->nk',d,n);rms=np.sqrt(np.mean(residual**2,axis=1));sigma=RULES['range_noise_sigma_m']
    valid=(rms<=RULES['plane_rms_noise_multiplier']*sigma)&(abs(residual).max(axis=1)<=RULES['plane_max_noise_multiplier']*sigma)&(values[:,1]/np.maximum(values[:,2],1e-15)>=RULES['minimum_middle_largest_eigen_ratio'])
    count=len(samples);samples,center,n,residual=samples[valid],center[valid],n[valid],residual[valid]
    n[np.einsum('ni,ni->n',center,n)>0]*=-1
    # Project observations onto their fitted local plane. This gives planar
    # measured supports, never a global hull or an angularly inflated footprint.
    projected=samples-n[:,None,:]*np.einsum('nki,ni->nk',samples-center[:,None,:],n)[:,:,None]
    vertices=[];normals=[]
    for corners in [[0,1,4,3],[1,2,5,4],[3,4,7,6],[4,5,8,7]]:
        quad=projected[:,corners,:];diameter=np.max(np.linalg.norm(quad[:,:,None,:]-quad[:,None,:,:],axis=3),axis=(1,2));use=diameter<=RULES['maximum_support_cell_diameter_m']
        vertices.append(quad[use]);normals.append(n[use])
    result=np.concatenate(vertices);normal=np.concatenate(normals)
    return result,normal,{'valid_3x3_patches':count,'plane_patches':int(valid.sum()),'support_cells':len(result)}


def inside_triangle(p,a,b,c):
    ab,ac,ap=b-a,c-a,p-a;d00=np.einsum('ni,ni->n',ab,ab);d01=np.einsum('ni,ni->n',ab,ac);d11=np.einsum('ni,ni->n',ac,ac)
    d20=np.einsum('ni,ni->n',ap,ab);d21=np.einsum('ni,ni->n',ap,ac);den=d00*d11-d01*d01;valid=den>1e-14
    u=np.zeros(len(p));v=u.copy();u[valid]=(d11[valid]*d20[valid]-d01[valid]*d21[valid])/den[valid];v[valid]=(d00[valid]*d21[valid]-d01[valid]*d20[valid])/den[valid]
    return valid&(u>=0)&(v>=0)&(u+v<=1)


def supported_intersections(origin,direction,observed_range,vertices,normals,centers,tree,frame_ids,blocks):
    ids=np.array(tree.query_ball_point(origin+direction*observed_range,RULES['surface_search_radius_m']),dtype=int)
    if not len(ids):return {}
    q=vertices[ids].astype(float);n=normals[ids].astype(float);den=n@direction;front=np.einsum('ni,ni->n',n,origin-q[:,0])>0
    use=(abs(den)>=RULES['minimum_ray_plane_abs_cosine'])&front
    t=np.full(len(ids),np.nan);t[use]=np.einsum('ni,ni->n',n[use],q[use,0]-origin)/den[use]
    use&=t>0;hits=origin+np.nan_to_num(t)[:,None]*direction
    use&=inside_triangle(hits,q[:,0],q[:,1],q[:,2])|inside_triangle(hits,q[:,0],q[:,2],q[:,3]);ids,t=ids[use],t[use]
    if not len(ids):return {}
    # First supported LOCAL cell per original mapping frame; overlapping cells
    # from one measurement never become independent votes.
    f,inv=np.unique(frame_ids[ids],return_inverse=True);pred=np.full(len(f),np.inf);np.minimum.at(pred,inv,t)
    source_blocks=np.array([blocks[int(k)]for k in f]);result={}
    for b in np.unique(source_blocks):
        p=pred[source_blocks==b];result[int(b)]={'depth_m':float(np.median(p)),'source_frames':len(p),'source_frame_depth_mad_m':float(np.median(abs(p-np.median(p))))}
    return result


def distinct_candidates(path):
    raw=json.loads(path.read_text());rows=[x for x in raw['hypotheses']if x['converged']and x['within_search_side']and x['fitness']<=.2 and x['inlier_ratio']>=.65];result=[]
    for row in sorted(rows,key=lambda x:x['fitness']):
        t=np.array(row['map_from_reference'])
        if all(np.linalg.norm(t[:3,3]-x[:3,3])>.75 or Rotation.from_matrix(t[:3,:3].T@x[:3,:3]).magnitude()>np.deg2rad(20)for x in result):result.append(t)
    if len(result)!=2:raise ValueError('fixed audit requires exactly two distinct accepted candidate clusters')
    return result


def query_rays(trial):
    meta=json.loads((trial/'result.json').read_text());start=next(x['status_ros_ns']/1e9 for x in meta['status_events']if x['stage']=='SPINNING');end=next(x['status_ros_ns']/1e9 for x in meta['status_events']if x['stage']=='PREPARING')
    rows=[]
    for path in sorted((trial/'raw_diagnostics').rglob('*.json')):
        j=json.loads(path.read_text())
        if j.get('accepted'):rows.append((path,j))
    rows.sort(key=lambda x:x[1]['stamp']);first=rows[0][1];o=np.eye(4);o[:3,3]=(np.array(first['output_pose']).reshape(4,4)@invert(np.array(first['base_from_sensor']).reshape(4,4)))[:3,3]
    selected=[x for x in rows if start<=x[1]['stamp']<=end];indices=np.unique(np.rint(np.linspace(0,len(selected)-1,min(RULES['query_keyframes'],len(selected)))).astype(int));result=[]
    for i in indices:
        path,j=selected[i]
        with path.with_suffix('.pcd').open('rb')as f:
            while not f.readline().startswith(b'DATA binary'):pass
            points=np.frombuffer(f.read(),dtype='<f4').reshape(-1,3).astype(float)
        grid=native_grid(points);r,c=np.meshgrid(np.arange(0,grid.shape[0],RULES['query_vertical_stride']),np.arange(0,grid.shape[1],RULES['query_horizontal_stride']),indexing='ij');q=grid[r.ravel(),c.ravel()];valid=np.isfinite(q).all(axis=1)
        result.append({'raw_json_path':str(path.resolve()),'raw_pcd_path':str(path.with_suffix('.pcd').resolve()),'stamp':j['stamp'],'sensor':invert(o)@np.array(j['output_pose']).reshape(4,4),'points':q[valid],'native_ids':(r.ravel()*RULES['horizontal_unique_beams']+c.ravel())[valid]})
    return result


def summarize(rows):
    per_block={};ray_details=[];cross_block={};aggregate=[]
    for row in rows:
        pred=row['predictions'];common=sorted(set(pred[0])&set(pred[1]));observed=row['range_m']
        for b in common:
            e=[pred[k][b]['depth_m']-observed for k in [0,1]];per_block.setdefault(b,[]).append((row['query_frame'],*e))
        if len(common)<RULES['minimum_common_blocks']:continue
        a=np.array([[pred[k][b]['depth_m']for b in common]for k in [0,1]])
        med=np.median(a,axis=1);repeat=np.sqrt(np.mean((a-med[:,None])**2,axis=1));gap=float(abs(med[0]-med[1]));sigma=float(np.sqrt(np.sum(repeat**2)+2*RULES['range_noise_sigma_m']**2));sign=np.sign(abs(a[0]-observed)-abs(a[1]-observed))
        item={'query_frame':row['query_frame'],'native_id':row['native_id'],'common_blocks':common,'candidate_median_depth_m':med.tolist(),'candidate_source_repeat_rms_m':repeat.tolist(),
            'prediction_gap_m':gap,'repeat_combined_sigma_m':sigma,'gap_exceeds_repeat':bool(gap>sigma),'gap_exceeds_3sigma':bool(gap>RULES['discriminative_sigma_multiplier']*sigma),
            'winner_consistent_across_blocks':bool(np.all(sign==sign[0])),'block_residuals_m':(a-observed).tolist()};ray_details.append(item)
        for i,b in enumerate(common):
            for j,c in enumerate(common[i+1:],i+1):cross_block.setdefault((b,c),[]).append((a[:,j]-a[:,i]).tolist())
    def stats(x):
        a=np.array(x);return {'paired_rays':len(a),'query_frames':len(np.unique(a[:,0])),'signed_range_medians_m':np.median(a[:,1:],axis=0).tolist(),
            'mean_absolute_range_error_m':np.mean(abs(a[:,1:]),axis=0).tolist(),'range_rmse_m':np.sqrt(np.mean(a[:,1:]**2,axis=0)).tolist(),'absolute_error_winner':int(np.argmin(np.mean(abs(a[:,1:]),axis=0)))}
    block_stats={str(b):stats(x)for b,x in per_block.items()}
    for group,predicate in [('all_repeated',lambda x:True),('gap_exceeds_repeat',lambda x:x['gap_exceeds_repeat']),('gap_exceeds_3sigma',lambda x:x['gap_exceeds_3sigma'])]:
        selected=[x for x in ray_details if predicate(x)];a=[];byblock={}
        for x in selected:
            for k,b in enumerate(x['common_blocks']):
                value=(x['query_frame'],x['block_residuals_m'][0][k],x['block_residuals_m'][1][k]);a.append(value);byblock.setdefault(str(b),[]).append(value)
        aggregate.append({'subset':group,'rays':len(selected),'query_frames':len(set(x['query_frame']for x in selected)),'blocks':sorted(byblock),'stable_winner_rays':sum(x['winner_consistent_across_blocks']for x in selected),
            'pooled_descriptive_only':stats(a)if a else None,'block_statistics':{b:stats(x)for b,x in byblock.items()}})
    return {'query_rays':len(rows),'candidate_supported_rays':[sum(bool(x['predictions'][i])for x in rows)for i in [0,1]],
        'both_supported_rays':sum(bool(set(x['predictions'][0])&set(x['predictions'][1]))for x in rows),'repeated_common_supported_rays':len(ray_details),
        'per_block_common_support':block_stats,'subsets':aggregate,'cross_block_prediction_repeat':{str(k):{'rays':len(x),'candidate_depth_repeat_rms_m':np.sqrt(np.mean(np.array(x)**2,axis=0)).tolist()}for k,x in cross_block.items()},'repeated_ray_details':ray_details}


def audit(source,prior,root,output):
    if output.exists():raise FileExistsError(str(output))
    output.mkdir(parents=True);(output/'rules.json').write_text(json.dumps({'rules':RULES,'scope':'local measured-surface audit near observed hit, NOT first-return/free-space; no GT/ideal/occupancy/position-specific surface rules; blocks are fixed time strata, not independent passes','tool_sha256':sha256(__file__)},indent=2)+'\n');(output/'auditor_source.py').write_bytes(Path(__file__).read_bytes());start=time.monotonic()
    capture=json.loads((source/'capture.json').read_text())
    if capture['failure'] is not None or capture['completed_waypoints']!=capture['total_waypoints']:raise ValueError('source capture incomplete')
    Trajectory(capture['odom'])
    params=yaml.safe_load((source/'mapping_params.yaml').read_text());extrinsic=fixed_extrinsic(params);manifest=json.loads((prior/'manifest.json').read_text());world=np.array(manifest['T_world_camera_init']);stamps=np.array([round(x['stamp']*1e9)for x in capture['odom']],np.int64);poses=np.array([x['matrix']for x in capture['odom']]);selected=set(range(0,len(stamps),RULES['mapping_source_frame_stride']));selected.add(len(stamps)-1)
    cases=[('batch05/holdout_002',root/'analysis_alias055/002_raw_frozen.json'),('batch05/holdout_055',root/'analysis_alias055/raw_frozen.json')]
    prepared=[(name,path,distinct_candidates(path),query_rays(root/name))for name,path in cases]
    files=[source/'capture.json',source/'mapping_params.yaml',prior/'manifest.json',Path(__file__).resolve(),Path(__file__).with_name('export_dense_sensor_prior.py').resolve(),Path(__file__).with_name('refine_sensor_prior.py').resolve()]
    files.extend(p for p in (source/'raw_bag').iterdir()if p.is_file())
    for name,path,candidates,queries in prepared:
        files.extend([path,root/name/'result.json'])
        for q in queries:files.extend([Path(q['raw_json_path']),Path(q['raw_pcd_path'])])
    source_hashes={str(p.resolve()):sha256(p)for p in files}
    (output/'sources.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    lookup={int(stamps[i]):i for i in selected};vparts=[];nparts=[];fparts=[];blocks={};source_stats=[]
    for stamp,frame,points in raw_frames(source/'raw_bag'):
        if stamp not in lookup:continue
        i=lookup[stamp];vertices,n,stats=local_cells(points);pose=world@poses[i]@extrinsic
        if len(vertices):vparts.append(transform(vertices.reshape(-1,3),pose).reshape(-1,4,3).astype(np.float32));nparts.append((n@pose[:3,:3].T).astype(np.float32));fparts.append(np.full(len(vertices),i,np.int32))
        blocks[i]=int((stamp-int(stamps[0]))*1e-9//RULES['source_time_block_sec']);source_stats.append({'source_index':i,'stamp_ns':stamp,'block':blocks[i],**stats})
        if len(source_stats)%50==0:print(json.dumps({'source_frames':len(source_stats),'cells':sum(x['support_cells']for x in source_stats),'wall_sec':time.monotonic()-start}),flush=True)
    if len(source_stats)!=len(selected):raise ValueError('selected source scan missing')
    vertices=np.concatenate(vparts);normals=np.concatenate(nparts);frame_ids=np.concatenate(fparts);centers=vertices.mean(axis=1);tree=cKDTree(centers);(output/'source_patch_inventory.json').write_text(json.dumps(source_stats,indent=2)+'\n')
    reports=[]
    for name,path,candidates,queries in prepared:
        rows=[]
        for qi,q in enumerate(queries):
            for pid,p in zip(q['native_ids'],q['points']):
                length=float(np.linalg.norm(p));direction=p/length;pred=[]
                for t in candidates:
                    sensor=t@q['sensor'];origin=sensor[:3,3];ray=sensor[:3,:3]@direction;pred.append(supported_intersections(origin,ray,length,vertices,normals,centers,tree,frame_ids,blocks))
                rows.append({'query_frame':qi,'stamp':q['stamp'],'native_id':int(pid),'range_m':length,'predictions':pred})
            print(json.dumps({'case':name,'query_frames':qi+1,'query_rays':len(rows),'wall_sec':time.monotonic()-start}),flush=True)
        report={'case':name,'candidate_matrices':[t.tolist()for t in candidates],'candidate_source_sha256':sha256(path),'query_frame_stamps':[q['stamp']for q in queries],**summarize(rows)};reports.append(report)
        (output/(name.replace('/','_')+'_raw_predictions.json')).write_text(json.dumps(rows)+'\n');(output/(name.replace('/','_')+'_summary.json')).write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'case':name,'repeated_common_rays':report['repeated_common_supported_rays'],'subsets':[{k:v for k,v in x.items()if k not in ['block_statistics']}for x in report['subsets']]}),flush=True)
    for path,expected in source_hashes.items():
        if sha256(path)!=expected:raise ValueError('source changed during audit: '+path)
    result={'source_inputs_verified_unchanged':True,'sources_sha256':sha256(output/'sources.json'),'source_frames':len(source_stats),'local_support_cells':len(vertices),'source_time_blocks':sorted(set(blocks.values())),'source_capture_sha256':sha256(source/'capture.json'),
        'scope':'finite local identifiability evidence only; source time blocks and query frames are statistical units, rays/cells are correlated; no complete first-return/free-space claim','case_files':[x['case'].replace('/','_')+'_summary.json'for x in reports],'wall_sec':time.monotonic()-start}
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('prior',type=Path);p.add_argument('root',type=Path);p.add_argument('output',type=Path);a=p.parse_args();audit(a.source.resolve(),a.prior.resolve(),a.root.resolve(),a.output.resolve())


if __name__=='__main__':main()
