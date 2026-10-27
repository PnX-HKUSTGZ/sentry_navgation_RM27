#!/usr/bin/env python3
"""Frozen-seed potential planar domains + raw contiguous-scan holdout.
Mapping diagnostic only: a potential domain is not certified co-visibility.
No GT/query input. Frontend poses used all802 raw scans; holdout is independent
of crossstation fitting, not an independent test of that local frontend.
"""
from pathlib import Path
from dataclasses import asdict
import json,time,hashlib,sys,numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));from refine_sensor_prior import Rules,voxel,normals,invert,transform,information_root
ROOT=Path('tools/global_relocalization/data/optimization_20261026');OUT=ROOT/'station_partial_overlap_v2';OUT.mkdir(exist_ok=False);rules=Rules();start=time.monotonic();seed_path=ROOT/'station_cross_registration_v1/sensor_seed.json';seed=json.loads(seed_path.read_text());plan_path=ROOT/'station_local_submaps_v1_inputs/input.json';plan=json.loads(plan_path.read_text());assert seed['GT_post_evaluation_files_read']is False
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
FROZEN={'base_rules':asdict(rules),'all45_pairs':True,'scan_split':'middle contiguous floor(N/4) ordinals, start=floor((N-floor(N/4))/2); all other frames train, no droppedtail','potential_domain':'seed transform to target TRAIN cloud; NNdistance<=.30m AND target TRAIN normalvalid; fixed once, saved originalframe/point IDs and hashes; not a visibility proof','post_validity':'same NNdistance<=.30m and target TRAIN normalvalid; invalidnormal/overdistance retained at cap=.30 in both plane andNNloss; no domain expansion/reselection','holdout_gate':'perframe fixed-denominator metrics; frame-equal block RMS/cappedNN <=seed+.002m, domaincoverage>=.65, >=15 uniquevalid target.08cells in everyheldout frame','training_gate':'atleast50 unique target.08cells, >=50normalmatches, domaincoverage>=.65, rank>=5, convergence norm(delta)<1e-5 within20steps; original update/step bounds','patch_counts':'unique .08 targetvoxel cells, not independent nativepatch samples; sensorposes use all802, crossfit holdout limitation explicit','physical_gates':'same bidirectional .03m/1degcycle, update<=.30m/10deg, boundednormal quality, no addedposition/yawprior','graph_rules':{'need_connected':True,'geometry_only_dof_after_station0_gauge':54,'rank_relative_tolerance':1e-8,'scene_m_per_rad':1.,'critical_edges':'accepted edges across originalv1 threecomponents','redundancy':'each criticaledge leave-one-out connected andrank54; alternate path of differentstationpairs; measurementalternatecycle<=.03m/1deg','first_gauge':'station00 original sensor/LIO seed fixed, no GT reanchor','post_graph':'every acceptededge measurementconsistency .03m/1deg, fixedscanheldout <=seed+.002 AND <=pairfit+.002, nodeupdates .5m/10deg, genuine solver.success; no odometry priors','no_map_export':True},'GT_or_query_read':False}
(OUT/'frozen_rules.json').write_text(json.dumps(FROZEN,indent=2)+'\n');(OUT/'sensor_seed.json').write_bytes(seed_path.read_bytes());(OUT/'source_inputs.json').write_text(json.dumps({'seed':sha(seed_path),'plan':sha(plan_path),'tool':sha(__file__),'refine_helper':sha(Path(__file__).resolve().parents[1]/'refine_sensor_prior.py')},indent=2)+'\n')
def pcd(path):
 with Path(path).open('rb')as f:
  while not f.readline().startswith(b'DATA binary'):pass
  return np.frombuffer(f.read(),dtype='<f4').reshape(-1,3).astype(float)
def compact_stats(p,n):
 if not len(p):return{'points':0}
 centered=p-p.mean(0);eig=np.linalg.eigvalsh(centered.T@centered/len(p));nv=np.linalg.eigvalsh(n.T@n/max(1,len(n)))if len(n)else np.zeros(3);return{'points':len(p),'bbox_extent_m':np.ptp(p,axis=0).tolist(),'spatial_eigenvalues_m2':eig.tolist(),'normal_second_moment_eigenvalues':nv.tolist()}
stations=[]
for st,s in zip(plan['stations'],seed['stations']):
 rows=json.loads((ROOT/'station_local_submaps_v1'/st['id']/'poses.json').read_text());N=len(st['scans']);hold_count=N//4;begin=(N-hold_count)//2;H=set(range(begin,begin+hold_count));train=[];held=[];hf=[];rawids=[];split=[]
 for ordinal,(scan,pose)in enumerate(zip(st['scans'],rows)):
  assert pose['accepted'];points=pcd(scan['path']);rr=np.linalg.norm(points,axis=1);valid=np.isfinite(points).all(1)&(rr>=.5)&(rr<=30);ids=np.flatnonzero(valid);q=transform(points[valid],np.array(pose['reference_from_sensor']));isheld=ordinal in H;split.append({'ordinal':ordinal,'stamp_ns':scan['stamp_ns'],'role':'heldout'if isheld else'train','raw_pcd':scan['path'],'raw_sha256':scan['sha256']})
  if isheld:held.append(q);hf.append(np.full(len(q),ordinal,np.int16));rawids.append(ids.astype(np.int32))
  else:train.append(q)
 train=voxel(np.concatenate(train),rules.match_leaf_m);held=np.concatenate(held);hf=np.concatenate(hf);rawids=np.concatenate(rawids);geom=normals(train,rules);directory=OUT/st['id'];directory.mkdir();np.savez(directory/'data.npz',train=train,normal=geom[1],normal_good=geom[2],held=held,held_frame=hf,held_raw_point=rawids,held_ordinals=np.array(sorted(H)),full=voxel(pcd(s['cloud']),rules.match_leaf_m));(directory/'frame_split.json').write_text(json.dumps(split,indent=2)+'\n');stations.append({'id':st['id'],'train':train,'held':held,'hf':hf,'rawids':rawids,'ordinals':sorted(H),'geom':geom,'full':voxel(pcd(s['cloud']),rules.match_leaf_m)});print('station split',st['id'],'trainframes',N-len(H),'heldframes',len(H),'trainpoints',len(train),'heldrawpoints',len(held),'wall',time.monotonic()-start,flush=True)
initial=np.array([s['map_from_local_initial']for s in seed['stations']]);domains={};domain_meta=[]
for i in range(10):
 for j in range(i+1,10):
  for src,tgt,name in [(j,i,'forward'),(i,j,'reverse')]:
   s,t=stations[src],stations[tgt];z=invert(initial[tgt])@initial[src];dt,it=t['geom'][0].query(transform(s['train'],z));dh,ih=t['geom'][0].query(transform(s['held'],z));ti=np.flatnonzero((dt<=.3)&t['geom'][2][it]);hi=np.flatnonzero((dh<=.3)&t['geom'][2][ih]);meta={'i':i,'j':j,'direction':name,'source':src,'target':tgt,'train_domain_points':len(ti),'train_domain_fraction':len(ti)/len(s['train']),'heldout_domain_points':len(hi),'heldout_domain_fraction':len(hi)/len(s['held']),'seed_train_unique_target_cells':len(np.unique(it[ti])),'seed_heldout_unique_target_cells':len(np.unique(ih[hi])),'heldout_frames':[],'train_support':compact_stats(transform(s['train'][ti],z),t['geom'][1][it[ti]]),'heldout_support':compact_stats(transform(s['held'][hi],z),t['geom'][1][ih[hi]])}
   for f in s['ordinals']:
    mask=s['hf'][hi]==f;meta['heldout_frames'].append({'ordinal':f,'domain_points':int(mask.sum()),'unique_seed_target_cells':len(np.unique(ih[hi][mask]))})
   path=OUT/f'domain_{i:02d}_{j:02d}_{name}.npz';np.savez(path,train_indices=ti.astype(np.int32),held_indices=hi.astype(np.int32),held_frame=s['hf'][hi],held_raw_point=s['rawids'][hi],seed_train_target=it[ti].astype(np.int32),seed_held_target=ih[hi].astype(np.int32));meta['domain_file']=path.name;meta['domain_sha256']=sha(path);domains[(i,j,name)]={'ti':ti,'hi':hi,'meta':meta};domain_meta.append(meta)
(OUT/'frozen_domains.json').write_text(json.dumps(domain_meta,indent=2)+'\n');(OUT/'domains_complete_before_any_registration.json').write_text(json.dumps({'complete':True,'domains':len(domain_meta),'all90_domain_sha256':{m['domain_file']:m['domain_sha256']for m in domain_meta},'elapsed_sec':time.monotonic()-start},indent=2)+'\n');print('all90 domains frozen before fits',time.monotonic()-start,flush=True)

def evaluate(src,tgt,domain,z):
 s,t=stations[src],stations[tgt];q=transform(s['held'][domain['hi']],z);d,ids=t['geom'][0].query(q);valid=(d<=.3)&t['geom'][2][ids];plane=np.full(len(q),.3);distance=np.full(len(q),.3);plane[valid]=np.minimum(abs(np.einsum('ij,ij->i',q[valid]-t['train'][ids[valid]],t['geom'][1][ids[valid]])),.3);distance[valid]=np.minimum(d[valid],.3);frames=[]
 for f in s['ordinals']:
  mask=s['hf'][domain['hi']]==f;count=int(mask.sum());frames.append({'ordinal':f,'fixed_denominator':count,'successful':int(valid[mask].sum()),'unique_valid_target_cells':len(np.unique(ids[mask&valid])),'coverage':float(valid[mask].mean())if count else 0.,'plane_mse':float(np.mean(plane[mask]**2))if count else .09,'capped_nn_mse':float(np.mean(distance[mask]**2))if count else .09})
 return{'fixed_domain_points':len(q),'frame_equal_coverage':float(np.mean([f['coverage']for f in frames])),'post_support_lost_points':int((~valid).sum()),'frame_equal_plane_rmse_m':float(np.sqrt(np.mean([f['plane_mse']for f in frames]))),'frame_equal_capped_nn_rmse_m':float(np.sqrt(np.mean([f['capped_nn_mse']for f in frames]))),'pooled_plane_rmse_m':float(np.sqrt(np.mean(plane**2)))if len(q)else .3,'pooled_capped_nn_rmse_m':float(np.sqrt(np.mean(distance**2)))if len(q)else .3,'every_frame_min15':all(f['fixed_denominator']>=15 and f['unique_valid_target_cells']>=15 for f in frames),'frames':frames}

def fit(src,tgt,D,initial_z):
 s,t=stations[src],stations[tgt];source=s['train'][D['ti']];tree,normal,good=t['geom'];z=initial_z.copy();converged=False;reject=[];iterations=0
 for iteration in range(rules.max_iterations):
  p=transform(source,z);d,ids=tree.query(p);valid=(d<=.3)&good[ids];pp,qq,n=p[valid],t['train'][ids[valid]],normal[ids[valid]]
  if len(pp)<rules.min_plane_matches:reject.append('insufficient_train_plane_matches');break
  residual=np.einsum('ij,ij->i',pp-qq,n);J=np.c_[np.cross(pp,n),n];weight=np.minimum(1.,.03/np.maximum(abs(residual),1e-12));h=J.T@(J*weight[:,None]);b=J.T@(weight*residual);v,u=np.linalg.eigh(h);keep=v>max(1e-12,v[-1]*rules.information_relative_floor);delta=-u[:,keep]@((u[:,keep].T@b)/v[keep]);iterations=iteration+1
  if np.linalg.norm(delta[:3])>.1 or np.linalg.norm(delta[3:])>.1:reject.append('unbounded_train_step');break
  update=np.eye(4);update[:3,:3]=Rotation.from_rotvec(delta[:3]).as_matrix();update[:3,3]=delta[3:];z=update@z
  if np.linalg.norm(delta)<1e-5:converged=True;break
 p=transform(source,z);d,ids=tree.query(p);valid=(d<=.3)&good[ids];pp,n=p[valid],normal[ids[valid]];root=np.zeros((6,6));rank=0;eigen=np.zeros(6)
 if len(pp):
  J=np.c_[np.cross(pp,n),n];h=J.T@J/max(1,len(pp))*min(100,len(pp))/.03**2;adj=np.zeros((6,6));adj[:3,:3]=z[:3,:3];adj[3:,3:]=z[:3,:3];v=z[:3,3];skew=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]]);adj[3:,:3]=skew@z[:3,:3];root,rank,eigen=information_root(adj.T@h@adj,rules.information_relative_floor)
 before=evaluate(src,tgt,D,initial_z);after=evaluate(src,tgt,D,z);update=z@invert(initial_z);origin_update=float(np.linalg.norm(z[:3,3]-initial_z[:3,3]));rotation_update=float(Rotation.from_matrix(update[:3,:3]).magnitude()*180/np.pi);coverage=float(valid.mean())if len(valid)else 0.
 if not converged:reject.append('train_not_converged')
 if D['meta']['seed_train_unique_target_cells']<50 or len(np.unique(ids[valid]))<50:reject.append('train_unique_cells<50')
 if coverage<.65:reject.append('train_domain_coverage<.65')
 if rank<5:reject.append('information_rank<5')
 if origin_update>.3 or rotation_update>10:reject.append('relative_update_bound')
 if not before['every_frame_min15']or not after['every_frame_min15']:reject.append('scan_holdout_unique_cells<15_in_some_frame')
 if after['frame_equal_coverage']<.65:reject.append('scan_holdout_domain_coverage<.65')
 if after['frame_equal_plane_rmse_m']>before['frame_equal_plane_rmse_m']+.002:reject.append('scan_holdout_fixed_plane_loss_degraded')
 if after['frame_equal_capped_nn_rmse_m']>before['frame_equal_capped_nn_rmse_m']+.002:reject.append('scan_holdout_fixed_NN_loss_degraded')
 full_seed=stations[tgt]['full'];ftree=cKDTree(full_seed);orig_before=ftree.query(transform(s['full'],initial_z))[0];orig_after=ftree.query(transform(s['full'],z))[0];metrics={'accepted':not reject,'rejected_gates':sorted(set(reject)),'converged':converged,'iterations':iterations,'train_domain_coverage':coverage,'train_normal_matches':int(valid.sum()),'train_unique_target_cells':len(np.unique(ids[valid])),'information_rank':rank,'information_eigenvalues':eigen.tolist(),'origin_update_m':origin_update,'rotation_update_deg':rotation_update,'scan_heldout_before':before,'scan_heldout_after':after,'original_full_cloud_coverage_at_seed':float(np.mean(orig_before<=.3)),'original_full_cloud_coverage_after':float(np.mean(orig_after<=.3))};return z,root,metrics
rows=[];edges=[];old={ (x['i'],x['j']):x for x in json.loads((ROOT/'station_cross_registration_v1/pair_results.json').read_text())}
for i in range(10):
 for j in range(i+1,10):
  initial_z=invert(initial[i])@initial[j];f,fr,fm=fit(j,i,domains[(i,j,'forward')],initial_z);b,br,bm=fit(i,j,domains[(i,j,'reverse')],invert(initial_z));cycle=b@f;ct=float(np.linalg.norm(cycle[:3,3]));cr=float(Rotation.from_matrix(cycle[:3,:3]).magnitude()*180/np.pi);ok=fm['accepted']and bm['accepted']and ct<=.03 and cr<=1.;row={'i':i,'j':j,'accepted':ok,'forward':fm,'reverse':bm,'forward_transform':f.tolist(),'reverse_transform':b.tolist(),'cycle_translation_m':ct,'cycle_rotation_deg':cr,'v1_original_fullcloud_forward_coverage':old[(i,j)]['forward'].get('coverage'),'v1_original_fullcloud_reverse_coverage':old[(i,j)]['reverse'].get('coverage')};rows.append(row)
  if ok:edges.append({'i':i,'j':j,'measurement':f.tolist(),'root':fr.tolist()})
  (OUT/'pair_results.json').write_text(json.dumps(rows,indent=2)+'\n');(OUT/'validated_edges.json').write_text(json.dumps(edges,indent=2)+'\n');print('partialpair',i,j,'accepted',ok,'fgates',fm['rejected_gates'],'rgates',bm['rejected_gates'],'cycle',ct,cr,'wall',time.monotonic()-start,flush=True)
rr=[e['i']for e in edges]+[e['j']for e in edges];cc=[e['j']for e in edges]+[e['i']for e in edges];nc,labels=connected_components(csr_matrix((np.ones(len(rr)),(rr,cc)),shape=(10,10)),directed=False);summary={'pairs_attempted':len(rows),'accepted_edges':len(edges),'components':int(nc),'component_labels':labels.tolist(),'graph_optimization_run':False,'global_map_exported':False,'GT_or_query_read':False,'all_domains_frozen_before_registration':True,'raw_scan_heldout':True,'elapsed_sec':time.monotonic()-start,'tool_sha256':sha(__file__)};(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
