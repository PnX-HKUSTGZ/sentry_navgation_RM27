#!/usr/bin/env python3
"""Dense-exact numerical implementation control, same partial objective. 9-node geometry, station5 unobserved seed.
No GT/query/odometry factors, no map export, no fullgraph adoption claim.
"""
from pathlib import Path
import ast,json,hashlib,time,sys,numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));import refine_sensor_prior as rsp
from supported_tree import GoodNormalTree
ROOT=Path('tools/global_relocalization/data/optimization_20261026');V2=ROOT/'station_partial_overlap_v2';V3=ROOT/'station_partial_overlap_v3';OUT=V3/'nine_node_dense_exact_diagnostic';OUT.mkdir(exist_ok=False);start=time.monotonic();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seed=json.loads((V3/'sensor_seed.json').read_text());initial=np.array([s['map_from_local_initial']for s in seed['stations']]);summary=json.loads((V3/'summary.json').read_text());audit=json.loads((V3/'failure_graph_audit.json').read_text());assert audit['existing9node_geometry_rank']==48 and summary['components']==2;active=audit['existing_connected_component_nodes'];assert active==[0,1,2,3,4,6,7,8,9];index={n:k for k,n in enumerate(active)};edges=json.loads((V3/'validated_edges.json').read_text());subedges=[{**e,'i':index[e['i']],'j':index[e['j']]}for e in edges];rules=rsp.Rules();(OUT/'scope_and_guards.json').write_text(json.dumps({'partial_only':True,'active_nodes':active,'unconstrained_placeholder_node':5,'node5_stays_original_LIO_seed_not_a_geometric_constraint':True,'fullgeometry_rank':48,'fullgeometry_required':54,'full_adoption_redundancy_failed':True,'solver':'same refine_sensor_prior.optimize_graph objective, denseexact linear subproblem; all6DOF/firstposefixed/noaddedpriors','guards':['solver.success positive termination not max-nfev','finiteSE3','station0 gauge exact','station5 seed exact','nodeupdate .5m/10deg','edge measurementconsistency .03m/1deg','fixedrawscanheldout <=seed+.002m AND<=pairfit+.002m with samecaps/counts'], 'global_map_export':False},indent=2)+'\n');solver_record={};original_lsq=rsp.least_squares
# Capture the original optimize_graph residual closure without running LSMR.
from types import SimpleNamespace
baseline={}
probes=[np.zeros(48),np.tile([.001,-.001,.001,.001,-.001,.001],8),np.linspace(-.0001,.0001,48)]
def capture_original_residual(fun,x0,**kwargs):
 baseline['values']=[fun(x).copy()for x in probes];baseline['kwargs']={k:v for k,v in kwargs.items()if k!='jac_sparsity'};baseline['sparsity_shape']=kwargs['jac_sparsity'].shape
 step=1e-6;columns=[]
 for k in range(len(x0)):
  d=np.zeros(len(x0));d[k]=step;columns.append((fun(x0+d)-fun(x0-d))/(2*step))
 baseline['central_J']=np.column_stack(columns)
 return SimpleNamespace(x=x0.copy(),success=False,message='residual capture only, no optimizer run',nfev=0,cost=float(.5*fun(x0)@fun(x0)))
rsp.least_squares=capture_original_residual
try:rsp.optimize_graph(initial[active],[{**e}for e in subedges],rules)
finally:rsp.least_squares=original_lsq
def observed_least_squares(*args,**kwargs):
 fun,x0=args[:2];current=[fun(x)for x in probes];exact=all(np.array_equal(a,b)for a,b in zip(current,baseline['values']));maximum=max(float(np.max(abs(a-b)))for a,b in zip(current,baseline['values']));assert exact,'objective residual changed'
 forward=[];h=np.sqrt(np.finfo(float).eps);r0=fun(x0)
 for k in range(len(x0)):
  d=np.zeros(len(x0));d[k]=h;forward.append((fun(x0+d)-r0)/h)
 J=np.column_stack(forward);difference=J-baseline['central_J'];check={'same_legacy_objective_residual_exact_at_all3probes':exact,'maximum_absolute_residual_difference':maximum,'finite_difference_central_step':1e-6,'finite_difference_forward_step':h,'Jacobian_max_abs_difference':float(np.max(abs(difference))),'Jacobian_relative_frobenius_difference':float(np.linalg.norm(difference)/np.linalg.norm(baseline['central_J'])),'same_loss_tolerances_budget_xscale':{k:v for k,v in kwargs.items()if k!='jac_sparsity'},'old_jac_sparsity_shape':baseline['sparsity_shape'],'only_solver_delta':'removejac_sparsity, tr_solverexact; defaultx_scaleunchanged'};(OUT/'residual_jacobian_equivalence.json').write_text(json.dumps(check,indent=2)+'\n');np.savez(OUT/'residual_jacobian_equivalence.npz',baseline_residual_probes=np.array(baseline['values']),dense_residual_probes=np.array(current),central_J=baseline['central_J'],forward_J=J)
 assert check['Jacobian_relative_frobenius_difference']<1e-5
 kwargs.pop('jac_sparsity');kwargs['tr_solver']='exact'
 fit=original_lsq(*args,**kwargs);solver_record.update(status=int(fit.status),success=bool(fit.success),optimality=float(fit.optimality),nfev=int(fit.nfev),njev=int(fit.njev)if fit.njev is not None else None,message=str(fit.message),cost=float(fit.cost),maximum_nfev=kwargs.get('max_nfev'),tr_solver='exact',x_scale='unchangeddefault');return fit
rsp.least_squares=observed_least_squares
try:solved,fit=rsp.optimize_graph(initial[active],[{**e}for e in subedges],rules)
finally:rsp.least_squares=original_lsq
final=initial.copy();final[active]=solved;delta=np.linalg.norm(final[:,:3,3]-initial[:,:3,3],axis=1);angle=(Rotation.from_matrix(np.transpose(initial[:,:3,:3],(0,2,1))@final[:,:3,:3]).magnitude()*180/np.pi);validr=bool(np.allclose(np.transpose(final[:,:3,:3],(0,2,1))@final[:,:3,:3],np.eye(3),atol=1e-6)and np.allclose(np.linalg.det(final[:,:3,:3]),1.,atol=1e-6));checks={'solver_success':fit['success']and solver_record['status']>0,'solver_not_iteration_cap':solver_record['nfev']<rules.graph_max_nfev,'finite_valid_SE3':bool(np.isfinite(final).all()and validr),'station0_gauge_exact':bool(np.array_equal(final[0],initial[0])),'station5_seed_exact':bool(np.array_equal(final[5],initial[5])),'node_translation_updates_bounded':bool(delta.max()<=.5),'node_rotation_updates_bounded':bool(angle.max()<=10.),'objective_not_worse':fit['final_normalized_rmse']<=fit['initial_normalized_rmse']+1e-10};edgecheck=[]
for e in edges:
 err=rsp.relative_error(final[e['i']],final[e['j']],np.array(e['measurement']));td=float(np.linalg.norm(err[3:]));rd=float(np.linalg.norm(err[:3])*180/np.pi);edgecheck.append({'i':e['i'],'j':e['j'],'translation_residual_m':td,'rotation_residual_deg':rd,'passes':td<=.03 and rd<=1.})
checks['all_edge_measurements_consistent']=all(e['passes']for e in edgecheck)
# Reuse the exact executed fixed-domain evaluator with v3 good-normal trainingtree.
stations=[]
for s in seed['stations']:
 a=np.load(V2/s['id']/'data.npz');stations.append({'train':a['train'],'held':a['held'],'hf':a['held_frame'],'ordinals':a['held_ordinals'].tolist(),'geom':(GoodNormalTree(a['train'],a['normal_good']),a['normal'],a['normal_good'])})
source=Path(__file__).with_name('partial_overlap_v2.py');tree=ast.parse(source.read_text());fn=next(f for f in tree.body if isinstance(f,ast.FunctionDef)and f.name=='evaluate');transform=rsp.transform;exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),globals());pairrows={(r['i'],r['j']):r for r in json.loads((V3/'pair_results.json').read_text())};held=[]
for e in edges:
 i,j=e['i'],e['j'];z=rsp.invert(final[i])@final[j];row={'i':i,'j':j,'directions':{}}
 for name,src,tgt,t in [('forward',j,i,z),('reverse',i,j,rsp.invert(z))]:
  p=V2/f'domain_{i:02d}_{j:02d}_{name}.npz';a=np.load(p);d={'hi':a['held_indices']};post=evaluate(src,tgt,d,t);previous=pairrows[(i,j)][name];seed_before=previous['scan_heldout_before'];pair=previous['scan_heldout_after'];valid=post['every_frame_min15']and post['frame_equal_coverage']>=.65;fail=[]
  if not post['every_frame_min15']:fail.append('insufficient_unique_patch_count')
  for metric in ['frame_equal_plane_rmse_m','frame_equal_capped_nn_rmse_m']:
   if post[metric]>seed_before[metric]+.002:valid=False;fail.append(metric+'_worse_than_seed')
   if post[metric]>pair[metric]+.002:valid=False;fail.append(metric+'_worse_than_pair')
  row['directions'][name]={'passed':bool(valid),'rejected':fail,'fixed_domain_count_unchanged':post['fixed_domain_points']==seed_before['fixed_domain_points'],'seed':seed_before,'individual_pairfit':pair,'partial_graph':post}
 row['passes']=all(v['passed']and v['fixed_domain_count_unchanged']for v in row['directions'].values());held.append(row)
checks['all_fixed_scan_holdout_preserved']=all(h['passes']for h in held);partial_pass=all(checks.values());output={'sensor_only':True,'partial_diagnostic':True,'global_graph_adopted':False,'global_geometry_rank54_passed':False,'rank':48,'graph_nodes':active,'unconstrained_seed_nodes':[5],'partial_guards_passed':partial_pass,'source_seed_sha256':sha(V3/'sensor_seed.json'),'station0_gauge':'unchanged originalLIO firststation seed','stations':[{'node':i,'id':s['id'],'map_from_local':final[i].tolist(),'geometrically_constrained':i!=5,'updated_in_partial_component':i in active}for i,s in enumerate(seed['stations'])]};name='optimized_nodeposes.json'if partial_pass else'rejected_candidate_nodeposes.json';(OUT/name).write_text(json.dumps(output,indent=2)+'\n');report={'partial_diagnostic_only':True,'partial_guards_passed':partial_pass,'fullgraph_adoption_rejected':True,'fullgraph_rejection_reasons':['station5 isolated with0validgeometric edges','fullrank48<54','singlecross-region3-7bridge lacksredundancy'],'graph_solver':fit,'actual_solver_termination':solver_record,'checks':checks,'node_translation_updates_m':delta.tolist(),'node_rotation_updates_deg':angle.tolist(),'edge_consistency':edgecheck,'fixed_scan_holdout_file':'fixed_scan_holdout_after_graph.json','candidate_nodepose_file':name,'global_map_exported':False,'GT_or_query_read':False,'elapsed_sec':time.monotonic()-start,'tool_sha256':sha(__file__)};(OUT/'fixed_scan_holdout_after_graph.json').write_text(json.dumps(held,indent=2)+'\n');(OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
