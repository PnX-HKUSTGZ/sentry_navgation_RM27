#!/usr/bin/env python3
"""All45 sensor-only station registrations under frozen existing map-edge rules.
Reads no GT/post-evaluation/query files. Unknown graph poses stay fullSE3.
"""
from pathlib import Path
from dataclasses import asdict
import json,time,sys,hashlib,numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));from refine_sensor_prior import Rules,voxel,normals,pair_fit,validate_pair,invert,transform,optimize_graph
ROOT=Path('tools/global_relocalization/data/optimization_20261026');OUT=ROOT/'station_cross_registration_v1';start=time.monotonic();rules=Rules();seed=json.loads((OUT/'sensor_seed.json').read_text());assert seed['sensor_only'] and seed['GT_post_evaluation_files_read']is False;inputs=seed['stations'];initial=np.array([s['map_from_local_initial']for s in inputs]);frozen={'rules':asdict(rules),'pair_selection':'all45 unordered pairs, no distance/yaw/GT/query prescreen','directions':'two independent fits each seeded by originalLIO relative pose/inverse','point_holdout':'deterministic1/5 source voxel centroids, existing rule; spatially correlated, not independent rawframes','acceptance':'unchanged coverage.65, cycle.03m/1deg, bounded planar normals, rank>=5, heldout degradation<=.002m, update bounds','loop_cap_per_node':'not used for this10node feasibility; every validated edge retained','fixed_graph_gauge':'station00 originalLIO pose, no GT re-anchor','scope':'registration+smallposegraph only, no globalmap/no232/no new point'};(OUT/'frozen_rules.json').write_text(json.dumps(frozen,indent=2)+'\n');before={str(Path(s['cloud'])):hashlib.sha256(Path(s['cloud']).read_bytes()).hexdigest()for s in inputs}
def pcd(path):
 with Path(path).open('rb')as f:
  while not f.readline().startswith(b'DATA binary'):pass
  return np.frombuffer(f.read(),dtype='<f4').reshape(-1,3).astype(float)
clouds=[];geometry=[]
for s in inputs:
 cloud=voxel(pcd(s['cloud']),rules.match_leaf_m);clouds.append(cloud);geometry.append(normals(cloud,rules));print('normals',s['id'],'points',len(cloud),'planar',int(geometry[-1][2].sum()),'wall',time.monotonic()-start,flush=True)
rows=[];edges=[]
for i in range(10):
 for j in range(i+1,10):
  z=invert(initial[i])@initial[j];forward,fm=pair_fit(clouds[j],clouds[i],geometry[i],z,rules);reverse,rm=pair_fit(clouds[i],clouds[j],geometry[j],invert(z),rules);ok,reason=validate_pair((forward,fm)if forward is not None else None,(reverse,rm)if reverse is not None else None,rules,True);row={'i':i,'j':j,'accepted':ok,'reason':reason,'forward':fm,'reverse':rm,'initial_relative':z.tolist()}
  if forward is not None:row['forward_transform']=forward[0].tolist()
  if reverse is not None:row['reverse_transform']=reverse[0].tolist()
  if forward is not None and reverse is not None:
   cycle=reverse[0]@forward[0];row['cycle_translation_m']=float(np.linalg.norm(cycle[:3,3]));row['cycle_rotation_deg']=float(Rotation.from_matrix(cycle[:3,:3]).magnitude()*180/np.pi)
  if ok:edges.append({'i':i,'j':j,'measurement':forward[0].tolist(),'root':forward[1].tolist()})
  rows.append(row);(OUT/'pair_results.json').write_text(json.dumps(rows,indent=2)+'\n');(OUT/'validated_edges.json').write_text(json.dumps(edges,indent=2)+'\n');print('pair',i,j,'accepted',ok,'reason',reason,'coverage',fm.get('coverage'),rm.get('coverage'),'wall',time.monotonic()-start,flush=True)
rr=[e['i']for e in edges]+[e['j']for e in edges];cc=[e['j']for e in edges]+[e['i']for e in edges];graph=csr_matrix((np.ones(len(rr)),(rr,cc)),shape=(10,10));components,labels=connected_components(graph,directed=False);summary={'all45_pairs_attempted':len(rows)==45,'pair_count':len(rows),'accepted_edges':len(edges),'components':int(components),'component_labels':labels.tolist(),'GT_read':False,'frozen_rules':'frozen_rules.json','graph_optimization_run':False,'global_map_exported':False,'elapsed_sec':time.monotonic()-start};(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if components==1:
 final,fit=optimize_graph(initial,edges,rules);d=np.linalg.norm(final[:,:3,3]-initial[:,:3,3],axis=1);angle=(Rotation.from_matrix(np.transpose(initial[:,:3,:3],(0,2,1))@final[:,:3,:3]).magnitude()*180/np.pi);checks={'solver_success':fit['success'],'initial_gauge_exact':bool(np.allclose(final[0],initial[0],rtol=0,atol=1e-12)),'position_update_bounded':bool(d.max()<=rules.max_cumulative_position_update_m),'rotation_update_bounded':bool(angle.max()<=rules.max_cumulative_rotation_update_deg),'finite':bool(np.isfinite(final).all()),'objective_not_worse':fit['final_normalized_rmse']<=fit['initial_normalized_rmse']+1e-10};held=[]
 def heldout(source,target,geom,t):
  points=transform(source[np.arange(len(source))%5==0],t);dist,idx=geom[0].query(points);valid=(dist<=rules.correspondence_m)&geom[2][idx];res=np.einsum('ij,ij->i',points[valid]-target[idx[valid]],geom[1][idx[valid]]);return{'count':int(valid.sum()),'coverage':float(np.mean(dist<=rules.correspondence_m)),'plane_rmse_m':float(np.sqrt(np.mean(res**2)))if len(res) else 1e9,'capped_distance_rmse_m':float(np.sqrt(np.mean(np.minimum(dist,rules.correspondence_m)**2)))}
 for row in rows:
  if not row['accepted']:continue
  i,j=row['i'],row['j'];z=invert(final[i])@final[j];f=heldout(clouds[j],clouds[i],geometry[i],z);b=heldout(clouds[i],clouds[j],geometry[j],invert(z));valid=True
  for key,val in [('forward',f),('reverse',b)]:
   reference=row[key]['heldout_after'];valid&=val['count']>=rules.min_heldout_matches and val['coverage']>=rules.min_coverage and val['plane_rmse_m']<=reference['plane_rmse_m']+rules.heldout_max_degradation_m and val['capped_distance_rmse_m']<=reference['capped_distance_rmse_m']+rules.heldout_max_degradation_m
  held.append({'i':i,'j':j,'valid_after_graph':bool(valid),'forward':f,'reverse':b})
 checks['every_validated_edge_holdout_retained']=all(x['valid_after_graph']for x in held);summary.update(graph_optimization_run=True,graph_solver=fit,graph_checks=checks,graph_validated=all(checks.values()),max_position_update_m=float(d.max()),max_rotation_update_deg=float(angle.max()));(OUT/'optimized_station_poses.json').write_text(json.dumps({'sensor_only':True,'initial':initial.tolist(),'optimized':final.tolist(),'gauge_station':0,'graph_validated':summary['graph_validated']},indent=2)+'\n');(OUT/'graph_holdout_validation.json').write_text(json.dumps(held,indent=2)+'\n')
for path,expected in before.items():
 if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:raise RuntimeError('localcloud input changed')
summary['elapsed_sec']=time.monotonic()-start;summary['localcloud_source_hashes_unchanged']=True;summary['tool_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
