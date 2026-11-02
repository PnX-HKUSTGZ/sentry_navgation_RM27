"""Read-only v3 edge failure/rank/redundancy audit. No fitting or graph solve."""
from pathlib import Path
import json,numpy as np,sys,collections
from scipy.spatial.transform import Rotation
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components,shortest_path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));from refine_sensor_prior import invert,relative_error
ROOT=Path('tools/global_relocalization/data/optimization_20261026');OUT=ROOT/'station_partial_overlap_v3';summary=json.loads((OUT/'summary.json').read_text());rows=json.loads((OUT/'pair_results.json').read_text());edges=json.loads((OUT/'validated_edges.json').read_text());seed=json.loads((OUT/'sensor_seed.json').read_text());initial=np.array([s['map_from_local_initial']for s in seed['stations']]);active=[i for i in range(10)if summary['component_labels'][i]==summary['component_labels'][0]];free=[i for i in active if i!=0];columns={node:k for k,node in enumerate(free)}
def unpack(x):
 p=initial.copy()
 for node,k in columns.items():p[node,:3,:3]=initial[node,:3,:3]@Rotation.from_rotvec(x[k*6:k*6+3]).as_matrix();p[node,:3,3]=initial[node,:3,3]+x[k*6+3:k*6+6]
 return p
Jparts=[]
for e in edges:
 if e['i']not in active or e['j']not in active:continue
 z=np.array(e['measurement']);root=np.array(e['root']);J=np.zeros((6,6*len(free)))
 for k in range(len(free)*6):
  d=np.zeros(len(free)*6);d[k]=1e-6;a=unpack(d);b=unpack(-d);J[:,k]=root@(relative_error(a[e['i']],a[e['j']],z)-relative_error(b[e['i']],b[e['j']],z))/(2e-6)
 Jparts.append(J)
J=np.concatenate(Jparts);H=J.T@J;values,vectors=np.linalg.eigh((H+H.T)*.5);tol=max(1e-10,values[-1]*1e-8);rank=int(np.sum(values>tol));bridges=[];oldlabels=json.loads((ROOT/'station_cross_registration_v1/summary.json').read_text())['component_labels'];lookup={(e['i'],e['j']):np.array(e['measurement'])for e in edges}
def adjacency(exclude=None):
 a=np.zeros((len(active),len(active)));ix={node:k for k,node in enumerate(active)}
 for e in edges:
  if exclude==(e['i'],e['j']):continue
  a[ix[e['i']],ix[e['j']]]=a[ix[e['j']],ix[e['i']]]=1
 return a,ix
for k,e in enumerate(edges):
 a,ix=adjacency((e['i'],e['j']));nc,_=connected_components(csr_matrix(a),directed=False);hh=H-Jparts[k].T@Jparts[k];vv=np.linalg.eigvalsh((hh+hh.T)*.5);rr=int(np.sum(vv>tol));row={'i':e['i'],'j':e['j'],'crosses_original_v1_components':oldlabels[e['i']]!=oldlabels[e['j']],'components_without_edge_on_existing9nodes':int(nc),'geometry_rank_without_edge':rr,'required_existing_component_rank':len(free)*6,'redundant_connectivity_and_rank':nc==1 and rr==len(free)*6};dist,pred=shortest_path(csr_matrix(a),directed=False,indices=ix[e['i']],return_predecessors=True)
 if np.isfinite(dist[ix[e['j']]]):
  walk=[ix[e['j']]]
  while walk[-1]!=ix[e['i']]:walk.append(int(pred[walk[-1]]))
  nodes=[active[i]for i in walk[::-1]];composed=np.eye(4)
  for u,v in zip(nodes,nodes[1:]):composed=composed@(lookup[(u,v)]if(u,v)in lookup else invert(lookup[(v,u)]))
  closure=invert(np.array(e['measurement']))@composed;row.update(alternate_station_path=nodes,alternate_measurement_cycle_translation_m=float(np.linalg.norm(closure[:3,3])),alternate_measurement_cycle_rotation_deg=float(Rotation.from_matrix(closure[:3,:3]).magnitude()*180/np.pi))
 bridges.append(row)
station5=[]
for r in rows:
 if 5 not in (r['i'],r['j']):continue
 item={'i':r['i'],'j':r['j'],'accepted':r['accepted'],'cycle_translation_m':r['cycle_translation_m'],'cycle_rotation_deg':r['cycle_rotation_deg'],'cycle_translation_pass':r['cycle_translation_m']<=.03,'cycle_rotation_pass':r['cycle_rotation_deg']<=1.,'directions':{}}
 for name in ['forward','reverse']:
  m=r[name];a,b=m['scan_heldout_after'],m['scan_heldout_before'];item['directions'][name]={'rejected_gates':m['rejected_gates'],'train_converged':m['converged'],'iterations':m['iterations'],'train_domain_coverage':m['train_domain_coverage'],'origin_update_m':m['origin_update_m'],'rotation_update_deg':m['rotation_update_deg'],'heldout_plane_before_after_m':[b['frame_equal_plane_rmse_m'],a['frame_equal_plane_rmse_m']],'heldout_cappedNN_before_after_m':[b['frame_equal_capped_nn_rmse_m'],a['frame_equal_capped_nn_rmse_m']],'heldout_coverage':a['frame_equal_coverage'],'post_support_lost_points':a['post_support_lost_points'],'original_fullcloud_coverage_after':m['original_full_cloud_coverage_after']}
 station5.append(item)
weak=[]
for mode in range(min(6,len(values))):
 vec=vectors[:,mode];weak.append({'eigenvalue':float(values[mode]),'per_free_node_world_translation_and_body_rotation':[{ 'node':node,'body_rotation_xyz':vec[k*6:k*6+3].tolist(),'world_translation_xyz':vec[k*6+3:k*6+6].tolist()}for k,node in enumerate(free)]})
report={'scope':'read-only diagnosis, no refit/optimization/export','all45_pairs_attempted':len(rows)==45,'validated_edges':len(edges),'components':summary['components'],'component_labels':summary['component_labels'],'existing_connected_component_nodes':active,'isolated_station':5,'existing9node_geometry_dimension_after_station0_gauge':len(free)*6,'existing9node_geometry_rank':rank,'full10node_geometry_rank_max':rank,'full10node_dimension_after_station0_gauge':54,'eigenvalues':values.tolist(),'rank_threshold':tol,'condition_positive':float(values[-1]/values[values>tol][0]),'fixed_scene_scale':'1m per1rad; actual source-root matrices preserved, no information floor/prior added','edge_leave_one_out':bridges,'weak_modes':weak,'station5_all9_edges':station5,'all90_rejected_gate_counts':dict(collections.Counter(g for r in rows for name in ['forward','reverse']for g in r[name]['rejected_gates'])),'decision':'No graph optimization: station5 isolated. Existing9node component has only one accepted cross-original-component edge3-7; removing it breaks connectivity/rank, so redundancy criterion also fails even before adding station5. No thresholds/domain/refits changed.'};np.savez(OUT/'nine_node_geometry_information.npz',J=J,H=H,eigenvalues=values,free_nodes=np.array(free));(OUT/'failure_graph_audit.json').write_text(json.dumps(report,indent=2)+'\n');print('nodes/rank',active,rank,'/',len(free)*6,'cond',report['condition_positive']);print('bridges',[(r['i'],r['j'],r['redundant_connectivity_and_rank'],r['geometry_rank_without_edge'])for r in bridges]);print('S5',[(x['i'],x['j'],x['cycle_translation_m'],x['directions']['forward']['rejected_gates'],x['directions']['reverse']['rejected_gates'])for x in station5])
