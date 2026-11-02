#!/usr/bin/env python3
"""Single semantic change from executed v2: fixed valid-normal target tree.
Reuse v2 engine functions verbatim, frozen split/cache/domain indices/parameters.
No GT, query, mask expansion, threshold or iteration-budget changes.
"""
from pathlib import Path
import ast,json,hashlib,time,sys,numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));from refine_sensor_prior import Rules,invert,transform,information_root
from supported_tree import GoodNormalTree
ROOT=Path('tools/global_relocalization/data/optimization_20261026');V2=ROOT/'station_partial_overlap_v2';OUT=ROOT/'station_partial_overlap_v3';OUT.mkdir(exist_ok=False);start=time.monotonic();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();v2summary=json.loads((V2/'summary.json').read_text());source=Path(__file__).with_name('partial_overlap_v2.py');assert sha(source)==v2summary['tool_sha256'];rules=Rules();frozen=json.loads((V2/'frozen_rules.json').read_text());assert frozen['base_rules']==__import__('dataclasses').asdict(rules);(OUT/'frozen_rules.json').write_text(json.dumps({'version':3,'inherits_all_v2_rules_unchanged':frozen,'only_model_delta':'before any optimization, both train/heldout query a fixed tree of validnormal target TRAIN points; query returns originaltargetindices; true>.30 capped; frozen v2 masks notexpanded','v2_rules_sha256':sha(V2/'frozen_rules.json'),'same_evaluator_fitter_source_sha256':sha(source),'good_tree_source_sha256':sha(Path(__file__).with_name('supported_tree.py'))},indent=2)+'\n');(OUT/'sensor_seed.json').write_bytes((V2/'sensor_seed.json').read_bytes());seed=json.loads((OUT/'sensor_seed.json').read_text());initial=np.array([s['map_from_local_initial']for s in seed['stations']]);stations=[];inherited={};metadata=json.loads((V2/'frozen_domains.json').read_text());domains={}
for s in seed['stations']:
 path=V2/s['id']/'data.npz';a=np.load(path);geom=(GoodNormalTree(a['train'],a['normal_good']),a['normal'],a['normal_good']);stations.append({'id':s['id'],'train':a['train'],'held':a['held'],'hf':a['held_frame'],'rawids':a['held_raw_point'],'ordinals':a['held_ordinals'].tolist(),'geom':geom,'full':a['full']});inherited[str(path)]=sha(path);inherited[str(path.with_name('frame_split.json'))]=sha(path.with_name('frame_split.json'))
for m in metadata:
 path=V2/m['domain_file'];assert sha(path)==m['domain_sha256'];a=np.load(path);domains[(m['i'],m['j'],m['direction'])]={'ti':a['train_indices'],'hi':a['held_indices'],'meta':m};inherited[str(path)]=m['domain_sha256']
(OUT/'inherited_frozen_inputs.json').write_text(json.dumps({'source_directory':str(V2.resolve()),'domains':90,'all_v2_masks_reused_exactly':True,'hashes':inherited},indent=2)+'\n');(OUT/'frozen_domains.json').write_bytes((V2/'frozen_domains.json').read_bytes())
# Only execute the identical two function definitions, not v2's top-level IO/fit.
tree=ast.parse(source.read_text());functions=[x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name in ('evaluate','fit')];assert len(functions)==2;exec(compile(ast.Module(body=functions,type_ignores=[]),str(source), 'exec'),globals())
old={(r['i'],r['j']):r for r in json.loads((V2/'pair_results.json').read_text())};rows=[];edges=[];before_differences=[]
for i in range(10):
 for j in range(i+1,10):
  z=invert(initial[i])@initial[j];f,fr,fm=fit(j,i,domains[(i,j,'forward')],z);b,br,bm=fit(i,j,domains[(i,j,'reverse')],invert(z));cycle=b@f;ct=float(np.linalg.norm(cycle[:3,3]));cr=float(Rotation.from_matrix(cycle[:3,:3]).magnitude()*180/np.pi);ok=fm['accepted']and bm['accepted']and ct<=.03 and cr<=1.;row={'i':i,'j':j,'accepted':ok,'forward':fm,'reverse':bm,'forward_transform':f.tolist(),'reverse_transform':b.tolist(),'cycle_translation_m':ct,'cycle_rotation_deg':cr,'v1_original_fullcloud_forward_coverage':old[(i,j)]['v1_original_fullcloud_forward_coverage'],'v1_original_fullcloud_reverse_coverage':old[(i,j)]['v1_original_fullcloud_reverse_coverage']};rows.append(row)
  for name,m in [('forward',fm),('reverse',bm)]:
   for key in ['frame_equal_plane_rmse_m','frame_equal_capped_nn_rmse_m']:before_differences.append(abs(m['scan_heldout_before'][key]-old[(i,j)][name]['scan_heldout_before'][key]))
  if ok:edges.append({'i':i,'j':j,'measurement':f.tolist(),'root':fr.tolist()})
  (OUT/'pair_results.json').write_text(json.dumps(rows,indent=2)+'\n');(OUT/'validated_edges.json').write_text(json.dumps(edges,indent=2)+'\n');print('v3pair',i,j,'accepted',ok,'fgates',fm['rejected_gates'],'rgates',bm['rejected_gates'],'cycle',ct,cr,'wall',time.monotonic()-start,flush=True)
rr=[e['i']for e in edges]+[e['j']for e in edges];cc=[e['j']for e in edges]+[e['i']for e in edges];ncomp,labels=connected_components(csr_matrix((np.ones(len(rr)),(rr,cc)),shape=(10,10)),directed=False)
for path,expected in inherited.items():
 if sha(path)!=expected:raise RuntimeError('inherited frozen input changed')
summary={'model_version':3,'single_semantic_delta_good_normal_target_tree':True,'pairs_attempted':len(rows),'accepted_edges':len(edges),'components':int(ncomp),'component_labels':labels.tolist(),'original_v2_domains_hashes_unchanged':True,'seed_heldout_metrics_max_difference_from_v2':max(before_differences),'graph_optimization_run':False,'global_map_exported':False,'GT_or_query_read':False,'elapsed_sec':time.monotonic()-start,'tool_sha256':sha(__file__)};(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
