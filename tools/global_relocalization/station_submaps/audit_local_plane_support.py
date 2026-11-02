#!/usr/bin/env python3
"""One fixed physical radius3*leaf, sensor-only support audit; no parameter sweep."""
import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from pipeline_common import Inputs,read_json,write_json,new_directory


def scatter(points,ids):
    q=points[ids];center=q.mean(0);d=q-center
    eig,vec=np.linalg.eigh(d.T@d/len(q));residual=abs(d@vec[:,0])
    return {'neighbor_count':len(ids),'raw_eigenvalues_m2':eig.tolist(),
            'RMS_to_fit_plane_m':float(np.sqrt(max(0.,eig[0]))),
            'maximum_plane_residual_m':float(residual.max()),
            'curvature':float(eig[0]/max(eig.sum(),1e-20)),
            'middle_to_largest_ratio':float(eig[1]/max(eig[2],1e-20)),
            'bbox_extent_m':np.ptp(q,axis=0).tolist(),'fit_normal':vec[:,0].tolist()}


def classify(info):
    # Existing bounded mapping normal rules plus the already-declared native
    # source-patch noise test (3sigma RMS/6sigma maximum, sigma=.002m).
    if info['neighbor_count']<5:return 'insufficient_neighbors'
    if info['middle_to_largest_ratio']<.05:return 'line_degenerate'
    if (info['curvature']>.03 or info['RMS_to_fit_plane_m']>.006
            or info['maximum_plane_residual_m']>.012):return 'not_noise_supported_plane'
    return 'noise_supported_plane'


def run(reconstruction,input_path,output):
    inputs=Inputs();root=Path(reconstruction).resolve()
    plan=read_json(inputs.add(input_path));leaf=plan['params']['leaf_size'];radius=3*leaf
    factor=read_json(inputs.add(root/'factors.json'))
    inventories={}
    for name in ['source','target']:
        rows=read_json(inputs.add(root/(name+'_features.json')))
        points=np.array([r['point']for r in rows]).reshape(-1,3)
        tree=cKDTree(points);near=tree.query_ball_point(points,radius)
        audit=[]
        for i,row in enumerate(rows):
            original=scatter(points,np.array(row['neighbors'],int))
            bounded=scatter(points,np.array(near[i],int))
            original.update(radius_m=row['radius_m'],classification=classify(original))
            bounded.update(radius_m=radius,classification=classify(bounded))
            if original['neighbor_count']>=5 and bounded['neighbor_count']>=5:
                bounded['normal_angle_from_K20_deg']=float(np.rad2deg(np.arccos(np.clip(abs(
                    np.dot(original['fit_normal'],bounded['fit_normal'])),0,1))))
            audit.append({'index':i,'xyz':points[i].tolist(),'original_K20':original,'fixed_radius':bounded})
        inventories[name]=audit
    switched=[];groups={}
    for f in factor['points']:
        if not(f['active_a']and f['active_b']and f['target_a']!=f['target_b']):continue
        source=inventories['source'][f['source']];a=inventories['target'][f['target_a']];b=inventories['target'][f['target_b']]
        nn_delta=f['fresh_cost_b']-f['same_nn_new_W_cost_b']
        classes=[x['fixed_radius']['classification']for x in [source,a,b]]
        if 'insufficient_neighbors'in classes:category='at_least_one_insufficient_local_support'
        elif 'line_degenerate'in classes:category='at_least_one_local_line_degenerate'
        elif 'not_noise_supported_plane'in classes:category='at_least_one_local_nonplanar'
        else:category='all_three_locally_supported_planes'
        angle=float(np.rad2deg(np.arccos(np.clip(abs(np.dot(a['original_K20']['fit_normal'],b['original_K20']['fit_normal'])),0,1))))
        switched.append({'source':source,'target_a':a,'target_b':b,'NN_cost_jump':nn_delta,
            'target_K20_normal_angle_deg':angle,'target_point_separation_m':float(np.linalg.norm(np.array(a['xyz'])-b['xyz'])),
            'H_trace_A':f['H_trace_a'],'H_trace_B':f['H_trace_b'],'local_support_category':category})
        g=groups.setdefault(category,{'switch_points':0,'signed_NN_cost_jump':0.,'positive_NN_cost_jump':0.,'H_trace_A':0.,'H_trace_B':0.})
        g['switch_points']+=1;g['signed_NN_cost_jump']+=nn_delta;g['positive_NN_cost_jump']+=max(0.,nn_delta)
        g['H_trace_A']+=f['H_trace_a'];g['H_trace_B']+=f['H_trace_b']
    report={'sensor_only_readonly':True,'GT_query_used':False,'fixed_radius_m':radius,'radius_rule':'3*original local leaf; single value, no sweep',
        'descriptive_existing_support_rules':{'minimum_neighbors':5,'curvature_max':.03,'middle_ratio_min':.05,
            'source_lidar_range_sigma_m':.002,'RMS_max_3sigma_m':.006,'max_residual_6sigma_m':.012},
        'wide_radius_alone_does_not_prove_incorrect_plane':True,'source_counts':{
            kind:dict(Counter(x[kind]['classification']for x in inventories['source']))for kind in ['original_K20','fixed_radius']},
        'target_counts':{kind:dict(Counter(x[kind]['classification']for x in inventories['target']))for kind in ['original_K20','fixed_radius']},
        'all35_switched_group_contributions':groups,'all_switch_points':len(switched),
        'all_switch_signed_cost_jump':sum(x['NN_cost_jump']for x in switched),
        'source721_and737':[x for x in switched if x['source']['index']in [721,737]],
        'all_switches_file':'switches.json','no_covariance_fallback_or_solver_change':True,
        'classification_limit':'noise-inconsistent scatter does not identify a specific surface label; sparse support does not prove a legal wide plane is geometrically false'}
    out=new_directory(output);write_json(out/'summary.json',report);write_json(out/'switches.json',switched)
    write_json(out/'source_support.json',inventories['source']);write_json(out/'target_support.json',inventories['target'])
    inputs.add(Path(__file__));inputs.save(out,stage='one-radius sensor plane support audit')
    print(__import__('json').dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reconstruction',type=Path,required=True)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    run(a.reconstruction,a.input,a.output)
