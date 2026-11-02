#!/usr/bin/env python3
"""Score a read-only pinned-GICP cycle reconstruction, never change a model."""
import argparse
from pathlib import Path
import numpy as np
from pipeline_common import Inputs,read_json,write_json,new_directory


def stats(x):
    x=np.asarray(x,float)
    return {'count':len(x),'min':float(x.min()),'median':float(np.median(x)),
            'p95':float(np.percentile(x,95)),'max':float(x.max())}if len(x)else {'count':0}


def feature_arrays(rows):
    eigen=np.array([r['raw_eigenvalues']for r in rows]).reshape(-1,3)
    return {'radius':np.array([r['radius_m']for r in rows]),
            'curvature':eigen[:,0]/np.maximum(eigen.sum(1),1e-20),
            'middle_ratio':eigen[:,1]/np.maximum(eigen[:,2],1e-20),
            'normal':np.array([r['raw_normal']for r in rows]).reshape(-1,3),
            'xyz':np.array([r['point']for r in rows]).reshape(-1,3),
            'extent':np.array([r['extent_m']for r in rows]).reshape(-1,3)}


def run(reconstruction,original_poses,ordinal,output):
    inputs=Inputs();reconstruction=Path(reconstruction).resolve()
    data=read_json(inputs.add(reconstruction/'factors.json'))
    source_rows=read_json(inputs.add(reconstruction/'source_features.json'))
    target_rows=read_json(inputs.add(reconstruction/'target_features.json'))
    original=read_json(inputs.add(original_poses))[ordinal]
    trace=original['iteration_trace']
    before=next(r for r in trace if r['attempt']==1 and r['iteration']==12 and r['inner']==0)
    after=next(r for r in trace if r['attempt']==1 and r['iteration']==13 and r['inner']==0)
    error={'fresh_A_vs_original_trace':abs(data['fresh_a_total']-before['objective']),
           'fixed_factor_B_vs_original_trace':abs(data['fixed_a_factor_b_total']-before['candidate_objective']),
           'fresh_B_vs_original_next_iteration':abs(data['fresh_b_total']-after['objective']),
           'final_fitness_vs_original':abs(data['final_reconstructed_quality']['fitness']-original['fitness']),
           'final_inlier_ratio_vs_original':abs(data['final_reconstructed_quality']['inlier_ratio']-original['inlier_ratio']),
           'H_B_eigenratio_vs_original_last_linearization':abs(data['information_ratio_b']-original['information_eigen_ratio'])}
    verified=all(v<1e-6 for v in error.values())
    rows=data['points'];active_a=np.array([r['active_a']for r in rows]);active_b=np.array([r['active_b']for r in rows])
    ta=np.array([r['target_a']if r['target_a']is not None else -1 for r in rows])
    tb=np.array([r['target_b']if r['target_b']is not None else -1 for r in rows])
    common=active_a&active_b;switched=common&(ta!=tb);new=~active_a&active_b;lost=active_a&~active_b
    cost=lambda name:np.array([r[name]for r in rows])
    fixed,same,fresh=cost('fixed_factor_cost_b'),cost('same_nn_new_W_cost_b'),cost('fresh_cost_b')
    w_delta=same-fixed
    nn_delta=np.where(common,fresh-same,0.)
    active_delta=np.where(new,fresh,0.)-np.where(lost,same,0.)
    h_a,h_b=cost('H_trace_a'),cost('H_trace_b')
    source,target=feature_arrays(source_rows),feature_arrays(target_rows)
    def support_summary(feature):
        return {'K20_radius_m':stats(feature['radius']),'K20_max_bbox_axis_m':stats(feature['extent'].max(1)),
                'raw_curvature':stats(feature['curvature']),'raw_middle_to_largest_eigenratio':stats(feature['middle_ratio']),
                'radius_gt_existing_mapping_normal_radius_.35m':int((feature['radius']>.35).sum()),
                'curvature_gt_existing_mapping_limit_.03':int((feature['curvature']>.03).sum()),
                'near_line_middle_ratio_lt_existing_.05':int((feature['middle_ratio']<.05).sum()),
                'classification_is_descriptive_no_new_filter':True}
    def group(mask):
        return {'points':int(mask.sum()),'fresh_b_cost':float(fresh[mask].sum()),
                'W_update_cost_delta':float(w_delta[mask].sum()),'NN_update_cost_delta':float(nn_delta[mask].sum()),
                'H_trace_A':float(h_a[mask].sum()),'H_trace_B':float(h_b[mask].sum()),
                'H_trace_units':'rotation scaled at1m/rad; descriptive, not a rank/adoption test'}
    angles=np.rad2deg(np.arccos(np.clip(np.abs(np.sum(target['normal'][ta[switched]]*target['normal'][tb[switched]],axis=1)),0,1)))
    order=np.argsort(abs(nn_delta))[::-1]
    top=[]
    for index in order[:10]:
        i=int(index);a,b=int(ta[i]),int(tb[i])
        top.append({'source_index':i,'source_xyz':source['xyz'][i].tolist(),
            'source_support':source_rows[i], 'active_a':bool(active_a[i]),'active_b':bool(active_b[i]),
            'target_a':a,'target_b':b,'target_a_support':target_rows[a]if a>=0 else None,
            'target_b_support':target_rows[b]if b>=0 else None,
            'fixed_factor_cost_B':float(fixed[i]),'same_NN_new_W_cost_B':float(same[i]),
            'fresh_cost_B':float(fresh[i]),'NN_cost_delta':float(nn_delta[i]),
            'H_trace_A':float(h_a[i]),'H_trace_B':float(h_b[i])})
    report={'sensor_only_readonly_diagnosis':True,'GT_or_query_used':False,
        'reconstruction_verified_against_original':verified,'absolute_reconstruction_errors':error,
        'source_points':data['source_points'],'target_points':data['target_points'],
        'source_K20_support':support_summary(source),'target_K20_support':support_summary(target),
        'source_hard_planar_covariance':'pinned CovarianceSetter always eig(1e-3,1,1) for>=5neighbors, without radius/curvature test',
        'costs_including_same_B_priors':{k:data[k]for k in ['fresh_a_total','fixed_a_factor_b_total','same_nn_new_W_b_total','fresh_b_total']},
        'fixed_source_universe_cost_jump_decomposition':{
            'freshB_minus_fixedA_factorB':float((fresh-fixed).sum()),
            'W_update_old_active_set':float(w_delta.sum()),'NN_switch_common_active':float(nn_delta.sum()),
            'active_set_add_remove':float(active_delta.sum()),
            'identity_residual':float((fresh-fixed).sum()-w_delta.sum()-nn_delta.sum()-active_delta.sum())},
        'groups':{'common_active':group(common),'common_switched_target':group(switched),
                  'common_same_target':group(common&~switched),'new_active':group(new),'lost_active':group(lost)},
        'switch_target_normal_angle_deg':stats(angles),
        'top10_absolute_NN_cost_jumps_descriptive':top,
        'no_iteration_budget_threshold_covariance_or_solver_change':True,
        'conclusion_scope':'attribute this actual cycle only after successful exact reconstruction; no claim that merely increasing iterations or averaging the cycle solves it'}
    out=new_directory(output);inputs.add(Path(__file__));write_json(out/'summary.json',report)
    inputs.save(out,stage='read-only cycle covariance summary',reconstruction_verified=verified)
    print(__import__('json').dumps({k:v for k,v in report.items()if k!='top10_absolute_NN_cost_jumps_descriptive'},indent=2))
    return verified


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reconstruction',type=Path,required=True)
    p.add_argument('--original-poses',type=Path,required=True);p.add_argument('--ordinal',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(0 if run(a.reconstruction,a.original_poses,a.ordinal,a.output)else 1)
