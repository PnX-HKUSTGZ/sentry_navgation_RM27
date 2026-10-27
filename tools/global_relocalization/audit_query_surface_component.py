#!/usr/bin/env python3
"""Trace actual connected native-cell support around an audited query vertex.
Same local plane and cell thresholds; only remove coarse spatial subsampling.
No global hull, hole filling, new source scoring or localization decision.
"""
import argparse,json
from pathlib import Path
import numpy as np
from audit_sensor_plane_identifiability import RULES,native_grid,query_rays
from export_dense_sensor_prior import sha256


def components(grid,row,col):
    v,h=grid.shape[:2];rr,cc=np.meshgrid(np.arange(1,v-1),np.arange(h),indexing='ij');rr,cc=rr.ravel(),cc.ravel()
    points=np.stack([grid[rr+dy,(cc+dx)%h]for dy in [-1,0,1]for dx in [-1,0,1]],axis=1)
    good=np.isfinite(points).all(axis=(1,2));points,rr,cc=points[good],rr[good],cc[good]
    center=points.mean(axis=1);delta=points-center[:,None,:];val,vec=np.linalg.eigh(np.transpose(delta,(0,2,1))@delta/9);normal=vec[:,:,0]
    error=np.einsum('nki,ni->nk',delta,normal);sigma=RULES['range_noise_sigma_m'];good=(np.sqrt(np.mean(error**2,axis=1))<=RULES['plane_rms_noise_multiplier']*sigma)&(abs(error).max(axis=1)<=RULES['plane_max_noise_multiplier']*sigma)&(val[:,1]/np.maximum(val[:,2],1e-15)>=RULES['minimum_middle_largest_eigen_ratio'])
    points,rr,cc,normal,error=points[good],rr[good],cc[good],normal[good],error[good];projected=points-normal[:,None,:]*error[:,:,None];cells={}
    for offset,corners in [((0,0),[0,1,4,3]),((0,1),[1,2,5,4]),((1,0),[3,4,7,6]),((1,1),[4,5,8,7])]:
        p=projected[:,corners,:];diam=np.linalg.norm(p[:,:,None,:]-p[:,None,:,:],axis=3).max(axis=(1,2));good=diam<=RULES['maximum_support_cell_diameter_m']
        for i in np.flatnonzero(good):
            key=(int(rr[i]-1+offset[0]),int((cc[i]-1+offset[1])%h))
            if key not in cells:cells[key]=p[i]
    seeds=[(r,c%h)for r in [row-1,row]for c in [col-1,col]if (r,c%h)in cells];remaining=set(seeds);result=[]
    while remaining:
        seed=min(remaining);stack=[seed];visited={seed}
        while stack:
            r,c=stack.pop()
            for near in [(r-1,c),(r+1,c),(r,(c-1)%h),(r,(c+1)%h)]:
                if near in cells and near not in visited:visited.add(near);stack.append(near)
        remaining-=visited;edges=[];raw_vertices=[]
        for r,c in visited:
            p=grid[[r,r,r+1,r+1],[c,(c+1)%h,(c+1)%h,c]];raw_vertices.extend(p)
            for neighbor,a,b in [((r-1,c),0,1),((r,(c+1)%h),1,2),((r+1,c),2,3),((r,(c-1)%h),3,0)]:
                if neighbor not in visited:edges.append([p[a],p[b]])
        edges=np.array(edges);query=grid[row,col];ab=edges[:,1]-edges[:,0];u=np.clip(np.einsum('ni,ni->n',query-edges[:,0],ab)/np.maximum(np.einsum('ni,ni->n',ab,ab),1e-15),0,1);distance=np.linalg.norm(query-edges[:,0]-u[:,None]*ab,axis=1)
        raw_vertices=np.unique(np.array(raw_vertices),axis=0);near=int(distance.argmin())
        result.append({'valid_native_cells':len(visited),'original_measured_vertices':len(raw_vertices),'boundary_edges':len(edges),
            'support_extent_sensor_xyz_m':np.ptp(raw_vertices,axis=0).tolist(),'query_hit_to_true_union_boundary_m':float(distance.min()),
            'closest_boundary_edge_sensor':edges[near].tolist(),'component_native_cells':sorted(visited),'union_boundary_edges_sensor':edges.tolist()})
    return {'all_valid_native_cells':len(cells),'query_adjacent_valid_cells':seeds,'query_connected_components':result,
        'scope':'all possible neighboring 3x3 fits with unchanged physical thresholds; each support remains its own actual 2x2 observed cell; 4-neighbor topology uses shared native edges; no hull or unknown-hole fill; boundary distances use measured endpoints'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    if a.output.exists():raise FileExistsError(str(a.output))
    q=query_rays(a.trial)[1];path=Path(q['raw_pcd_path'])
    with path.open('rb')as f:
        while not f.readline().startswith(b'DATA binary'):pass
        points=np.frombuffer(f.read(),'<f4').reshape(-1,3).astype(float)
    report={'scope':'post-hoc geometric support trace of existing native row64/col192; not a changed threshold or acceptance score',
        'query_stamp':q['stamp'],'raw_pcd_sha256':sha256(path),'raw_json_sha256':sha256(q['raw_json_path']),'rules':RULES,'tool_sha256':sha256(__file__),**components(native_grid(points),64,192)}
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k not in ['query_connected_components','rules']},indent=2))
    for c in report['query_connected_components']:print({k:v for k,v in c.items()if k not in ['component_native_cells','union_boundary_edges_sensor']})


if __name__=='__main__':main()
