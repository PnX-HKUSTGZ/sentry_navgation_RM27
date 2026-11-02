#!/usr/bin/env python3
"""Post-hoc physical trace of one preregistered-audit discriminative ray.
Does not change the baseline rules or produce a localization acceptance gate.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
import yaml
from audit_sensor_plane_identifiability import (RULES,native_grid,local_cells,inside_triangle,query_rays,distinct_candidates)
from export_dense_sensor_prior import raw_frames,fixed_extrinsic,sha256
from refine_sensor_prior import invert,transform


def intersections(origin,direction,vertices,normals,observed):
    center=vertices.mean(axis=1);v=center-origin;along=v@direction;perp=np.linalg.norm(v-along[:,None]*direction,axis=1)
    # Pure conservative cell bounding sphere, used only to accelerate the exact
    # triangle test; never an acceptance footprint or angular aperture.
    radius=np.linalg.norm(vertices-center[:,None,:],axis=2).max(axis=1)
    ids=np.flatnonzero((perp<=radius+1e-10)&(along+radius>0)&(along-radius<RULES['maximum_range_m']))
    if not len(ids):return []
    q=vertices[ids];n=normals[ids];den=n@direction;front=np.einsum('ni,ni->n',n,origin-q[:,0])>0;good=front&(abs(den)>=RULES['minimum_ray_plane_abs_cosine'])
    t=np.full(len(ids),np.nan);t[good]=np.einsum('ni,ni->n',n[good],q[good,0]-origin)/den[good];hit=origin+np.nan_to_num(t)[:,None]*direction
    good&=(t>0)&(t<=RULES['maximum_range_m'])&(inside_triangle(hit,q[:,0],q[:,1],q[:,2])|inside_triangle(hit,q[:,0],q[:,2],q[:,3]))
    result=[]
    for k in np.flatnonzero(good):
        vv=q[k];p=hit[k];margin=edge_margin(p,vv)
        result.append({'cell_index':int(ids[k]),'depth_m':float(t[k]),'range_residual_m':float(t[k]-observed),'ray_plane_abs_cosine':float(abs(den[k])),
            'hit_world':p.tolist(),'vertices_world':vv.tolist(),'normal_world':n[k].tolist(),'hit_to_cell_edge_margin_m':margin,
            'cell_center_to_observed_hit_m':float(np.linalg.norm(center[ids[k]]-(origin+direction*observed))),
            'inside_original_search_radius':bool(np.linalg.norm(center[ids[k]]-(origin+direction*observed))<=RULES['surface_search_radius_m'])})
    return sorted(result,key=lambda x:x['depth_m'])


def edge_margin(p,v):
    a=v;b=np.roll(v,-1,axis=0);ab=b-a;u=np.clip(np.einsum('ij,ij->i',p-a,ab)/np.maximum(np.einsum('ij,ij->i',ab,ab),1e-15),0,1)
    return float(np.linalg.norm(p-a-u[:,None]*ab,axis=1).min())


def query_patch(grid,row,col):
    patch=grid[row-1:row+2,col-1:col+2].reshape(-1,3);result={'all_3x3_valid':bool(np.isfinite(patch).all()),'ranges_5x5_m':np.linalg.norm(grid[row-2:row+3,col-2:col+3],axis=2).tolist()}
    if not result['all_3x3_valid']:return result,None,None
    center=patch.mean(axis=0);values,vectors=np.linalg.eigh(np.cov(patch.T,bias=True));n=vectors[:,0]
    if n@center>0:n=-n
    residual=(patch-center)@n;projected=patch-residual[:,None]*n
    result.update({'native_3x3_points_sensor':patch.tolist(),'rms_m':float(np.sqrt(np.mean(residual**2))),'max_abs_residual_m':float(abs(residual).max()),'eigenvalues':values.tolist(),
        'support_diameter_m':float(np.linalg.norm(projected[:,None]-projected[None,:],axis=2).max())})
    valid=(result['rms_m']<=RULES['plane_rms_noise_multiplier']*RULES['range_noise_sigma_m']and result['max_abs_residual_m']<=RULES['plane_max_noise_multiplier']*RULES['range_noise_sigma_m']and values[1]/values[2]>=RULES['minimum_middle_largest_eigen_ratio']and result['support_diameter_m']<=RULES['maximum_support_cell_diameter_m'])
    result['passes_original_physical_plane_support_rules']=bool(valid)
    return result,projected[[0,2,8,6]]if valid else None,n if valid else None


def source_patch_provenance(world_vertices,source_pose,grid):
    local=transform(np.array(world_vertices),invert(source_pose));raw=grid.reshape(-1,3);good=np.isfinite(raw).all(axis=1);ids=np.flatnonzero(good)
    from scipy.spatial import cKDTree
    tree=cKDTree(raw[good]);dist,nearest=tree.query(local);beam=ids[nearest];rows,cols=np.divmod(beam,RULES['horizontal_unique_beams']);r0=(int(rows.min())//3)*3;c0=(int(cols.min())//3)*3
    p=grid[r0:r0+3,c0:c0+3]
    return {'nearest_raw_corner_native_ids':beam.tolist(),'corner_projection_to_nearest_raw_m':dist.tolist(),'parent_3x3_native_rows':[r0,r0+1,r0+2],'parent_3x3_native_cols':[c0,c0+1,c0+2],
        'parent_3x3_ranges_m':np.linalg.norm(p,axis=2).tolist(),'parent_3x3_points_sensor':p.tolist()}


def reciprocal_source_beams(raw_grid,source_pose,world_quad,world_normal):
    local=transform(world_quad,invert(source_pose));n=source_pose[:3,:3].T@world_normal
    front=float(n@(-local[0]))>0
    if not front:return {'status':'query_patch_backface_from_source','crossings':[]}
    points=raw_grid.reshape(-1,3);beam_ids=np.flatnonzero(np.isfinite(points).all(axis=1));points=points[beam_ids];ranges=np.linalg.norm(points,axis=1);directions=points/ranges[:,None]
    den=directions@n;valid=abs(den)>=RULES['minimum_ray_plane_abs_cosine'];t=np.full(len(points),np.nan);t[valid]=(n@local[0])/den[valid];hit=np.nan_to_num(t)[:,None]*directions
    v=np.tile(local,(len(points),1,1));valid&=(t>0)&(t<=RULES['maximum_range_m'])&(inside_triangle(hit,v[:,0],v[:,1],v[:,2])|inside_triangle(hit,v[:,0],v[:,2],v[:,3]))
    rows=[]
    for i in np.flatnonzero(valid):
        rows.append({'native_beam_id':int(beam_ids[i]),'source_measured_range_m':float(ranges[i]),'query_patch_intersection_range_m':float(t[i]),'measured_after_patch_m':float(ranges[i]-t[i]),
            'query_patch_edge_margin_m':edge_margin(hit[i],local),'ray_plane_abs_cosine':float(abs(den[i])),'source_return_world':transform(points[i:i+1],source_pose)[0].tolist()})
    return {'status':'known_query_patch_geometry_only'if rows else'no_source_beam_within_measured_patch','crossings':rows}


def trace(source,prior,root,baseline,output):
    if output.exists():raise FileExistsError(str(output))
    output.mkdir(parents=True);start=time.monotonic();report=json.loads((baseline/'batch05_holdout_055_summary.json').read_text());chosen=[x for x in report['repeated_ray_details']if x['gap_exceeds_3sigma']]
    if len(chosen)!=1:raise ValueError('this trace is explicitly for the one existing discriminative ray')
    chosen=chosen[0];q=query_rays(root/'batch05/holdout_055')[chosen['query_frame']];native=chosen['native_id'];row,col=divmod(native,RULES['horizontal_unique_beams'])
    with Path(q['raw_pcd_path']).open('rb')as f:
        while not f.readline().startswith(b'DATA binary'):pass
        raw=np.frombuffer(f.read(),'<f4').reshape(-1,3).astype(float)
    grid=native_grid(raw);point=grid[row,col];distance=float(np.linalg.norm(point));patch,quad,qnormal=query_patch(grid,row,col);candidates=distinct_candidates(root/'analysis_alias055/raw_frozen.json');geometry=[]
    for candidate in candidates:
        sensor=candidate@q['sensor'];geometry.append({'origin':sensor[:3,3],'direction':sensor[:3,:3]@(point/distance),'observed_hit':transform(point[None,:],sensor)[0],
            'quad':transform(quad,sensor)if quad is not None else None,'normal':sensor[:3,:3]@qnormal if qnormal is not None else None})
    (output/'trace_rules.json').write_text(json.dumps({'baseline_rules_sha256':sha256(baseline/'rules.json'),'baseline_sources_sha256':sha256(baseline/'sources.json'),
        'selected_ray_reason':'post-hoc trace of sole baseline 3sigma discriminative ray, not new selection threshold','query_frame_index':chosen['query_frame'],'query_stamp':q['stamp'],'native_row_col':[row,col],
        'full_atlas_intersections':'same plane/support/backface/incidence criteria, remove only original .75m center truncation for this one-ray diagnostic',
        'reciprocal_test':'source original beam must intersect measured query patch, then actual source range can be compared; no interpolated beam/aperture; no certified source/query relative pose bound available',
        'range_noise_only_margin_m':float(6*np.sqrt(2)*RULES['range_noise_sigma_m']),'pose_margin_certified':False,'rules_not_changed':True,'tool_sha256':sha256(__file__)},indent=2)+'\n')
    source_hashes=json.loads((baseline/'sources.json').read_text());source_hashes[str(Path(__file__).resolve())]=sha256(__file__)
    for p,h in source_hashes.items():
        if sha256(p)!=h:raise ValueError('source changed since baseline: '+p)
    (output/'sources.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    capture=json.loads((source/'capture.json').read_text());stamps=np.array([round(x['stamp']*1e9)for x in capture['odom']]);poses=np.array([x['matrix']for x in capture['odom']]);world=np.array(json.loads((prior/'manifest.json').read_text())['T_world_camera_init']);ext=fixed_extrinsic(yaml.safe_load((source/'mapping_params.yaml').read_text()));selected=set(range(0,len(stamps),RULES['mapping_source_frame_stride']));selected.add(len(stamps)-1);lookup={int(stamps[i]):i for i in selected}
    vertex=[];normal=[];frame=[];results=[]
    for stamp,frame_id,points in raw_frames(source/'raw_bag'):
        if stamp not in lookup:continue
        i=lookup[stamp];pose=world@poses[i]@ext;v,n,_=local_cells(points);wv=transform(v.reshape(-1,3),pose).reshape(-1,4,3);wn=n@pose[:3,:3].T
        vertex.append(wv.astype(np.float32));normal.append(wn.astype(np.float32));frame.append(np.full(len(v),i,np.int32));sg=native_grid(points);branches=[]
        for g in geometry:
            hits=intersections(g['origin'],g['direction'],wv.astype(np.float32).astype(float),wn.astype(np.float32).astype(float),distance)
            for hit in hits:hit['raw_support']=source_patch_provenance(hit['vertices_world'],pose,sg)
            reciprocal=reciprocal_source_beams(sg,pose,g['quad'],g['normal'])if g['quad']is not None else{'status':'query_patch_not_valid','crossings':[]}
            branches.append({'atlas_intersections':hits,'nearest_full_atlas_depth_m':hits[0]['depth_m']if hits else None,
                'nearest_original_local_depth_m':next((x['depth_m']for x in hits if x['inside_original_search_radius']),None),'reciprocal_source_beams':reciprocal})
        results.append({'source_index':i,'source_stamp_ns':stamp,'source_block':int((stamp-int(stamps[0]))*1e-9//RULES['source_time_block_sec']),'source_sensor_origin_world':pose[:3,3].tolist(),'candidate_branches':branches})
        if len(results)%50==0:print(json.dumps({'source_frames':len(results),'wall_sec':time.monotonic()-start}),flush=True)
    np.savez(output/'source_cells.npz',vertices=np.concatenate(vertex),normals=np.concatenate(normal),source_frame_indices=np.concatenate(frame))
    result={'scope':'one-ray physical trace only, no GT/ideal map, no new ranking gate, same baseline surface rules','query_native_row_col':[row,col],'query_stamp':q['stamp'],'query_observed_range_m':distance,
        'query_local_patch':patch,'candidate_geometry':[{k:v.tolist()if isinstance(v,np.ndarray)else v for k,v in x.items()}for x in geometry],
        'source_frame_count':len(results),'source_frames':results,'source_relative_pose_uncertainty_certified':False,'wall_sec':time.monotonic()-start}
    (output/'trace.json').write_text(json.dumps(result,indent=2)+'\n')
    for p,h in source_hashes.items():
        if sha256(p)!=h:raise ValueError('source changed during trace: '+p)
    summary={'source_inputs_verified_unchanged':True,'source_frames':len(results),'query_patch_valid':quad is not None,'pose_margin_certified':False,'candidate_summary':[],'wall_sec':time.monotonic()-start}
    for k in [0,1]:
        crossing=[(x,y)for x in results for y in x['candidate_branches'][k]['reciprocal_source_beams']['crossings']];noise_margin=6*np.sqrt(2)*RULES['range_noise_sigma_m'];beyond=[(x,y)for x,y in crossing if y['measured_after_patch_m']>noise_margin]
        full=[x for x in results if x['candidate_branches'][k]['nearest_full_atlas_depth_m']is not None];local=[x for x in results if x['candidate_branches'][k]['nearest_original_local_depth_m']is not None]
        summary['candidate_summary'].append({'candidate_index':k,'source_frames_with_full_atlas_intersection':len(full),'source_frames_with_local_intersection':len(local),
            'source_frames_where_full_first_differs_from_local':sum(x['candidate_branches'][k]['nearest_full_atlas_depth_m']!=x['candidate_branches'][k]['nearest_original_local_depth_m']for x in full),
            'source_beams_through_query_patch':len(crossing),'range_noise_only_beyond_patch_beams':len(beyond),'range_noise_only_beyond_blocks':sorted(set(x['source_block']for x,y in beyond)),
            'maximum_beyond_beam_query_patch_edge_margin_m':max((y['query_patch_edge_margin_m']for x,y in beyond),default=None),
            'range_noise_only_does_not_bound_relative_pose':True})
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['source','prior','root','baseline','output']:p.add_argument(name,type=Path)
    a=p.parse_args();trace(a.source.resolve(),a.prior.resolve(),a.root.resolve(),a.baseline.resolve(),a.output.resolve())


if __name__=='__main__':main()
