#!/usr/bin/env python3
"""LiDAR-only final-pose information decomposition, no priors/new gates."""
import argparse
from pathlib import Path
import numpy as np
from pipeline_common import Inputs,read_json,write_json,new_directory


def spectrum(h):
    values,vectors=np.linalg.eigh((h+h.T)*.5)
    return {'eigenvalues':values.tolist(),'lambda_min':float(values[0]),'lambda_max':float(values[-1]),
        'trace':float(np.trace(h)),'ratio':float(values[0]/values[-1]),
        'weak_direction_rotation_then_translation':vectors[:,0].tolist()},vectors[:,0]


def point_arrays(rows,radius):
    xyz=np.array([r['point']for r in rows]).reshape(-1,3)
    cov=np.array([r['covariance']for r in rows])
    n=np.array([sum(np.linalg.norm(xyz[r['neighbors']]-xyz[i],axis=1)<=radius)
                for i,r in enumerate(rows)])
    return xyz,cov,n


def information(data,source,target,radius,fallback_only=False):
    p,cs,ns=point_arrays(source,radius);q,ct,nt=point_arrays(target,radius)
    z=np.array(data['state_a']);r=z[:3,:3];t=z[:3,3]
    h=np.zeros((6,6));supported=np.zeros_like(h);fallback=np.zeros_like(h)
    counts={'supported_inliers':0,'fallback_inliers':0,'source_fallback_points':int((ns<5).sum()),
            'target_fallback_points':int((nt<5).sum())}
    if fallback_only:
        cs=cs.copy();ct=ct.copy();cs[ns<5]=np.eye(3);ct[nt<5]=np.eye(3)
    for item in data['points']:
        if not item['active_a']:continue
        i,j=item['source'],item['target_a'];x=p[i]
        skew=np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])
        jac=np.c_[r@skew,-r];precision=np.linalg.inv(ct[j]+r@cs[i]@r.T)
        hi=jac.T@precision@jac;h+=hi
        if ns[i]>=5 and nt[j]>=5:
            supported+=hi;counts['supported_inliers']+=1
        else:
            fallback+=hi;counts['fallback_inliers']+=1
    return h,supported,fallback,counts


def run(bounded,original,poses,ordinal,radius,output):
    inputs=Inputs()
    def load(folder):
        folder=Path(folder);return [read_json(inputs.add(folder/name))for name in (
            'factors.json','source_features.json','target_features.json')]
    new_data,source,target=load(bounded);old_data,old_source,old_target=load(original)
    recorded=read_json(inputs.add(poses))[ordinal]
    h,hs,hf,counts=information(new_data,source,target,radius)
    full,weak=spectrum(h);sup,_=spectrum(hs);fall,_=spectrum(hf)
    expected=np.array(new_data['H_b']);error=float(np.max(abs(h-expected)))
    quality=new_data['final_reconstructed_quality']
    quality_error=max(abs(quality[k]-recorded[k])for k in ['fitness','inlier_ratio'])
    old_h,_,_,old_counts=information(old_data,old_source,old_target,radius)
    counterfactual,_,_,_=information(old_data,old_source,old_target,radius,True)
    old_spectrum,_=spectrum(old_h);counterfactual_spectrum,_=spectrum(counterfactual)
    downgrade_eigen=np.linalg.eigvalsh((old_h-counterfactual+old_h.T-counterfactual.T)*.5)
    weak_full=float(weak@h@weak);weak_fallback=float(weak@hf@weak)
    report={'LiDAR_only_no_priors':True,'final_pose_relinearized':True,
        'registration_tangent_frame':'right sensor-relative rotation radians / translation meters; fixed1m per rad normalization',
        'supported_definition':'both source and selected target have>=5 neighbors inside fixed3leaf; not curvature/planequality validation',
        'radius_m':radius,'counts':counts,'full':full,'supported_only':sup,'fallback_only':fall,
        'weak_direction_fallback_information_fraction':weak_fallback/weak_full,
        'weak_direction_supported_information':float(weak@hs@weak),
        'weak_direction_fallback_information':weak_fallback,'weak_direction_full_information':weak_full,
        'fresh_final_H_vs_actual_pinned_probe_max_abs_error':error,
        'fresh_final_quality_vs_recorded_max_abs_error':quality_error,
        'native_reported_pre_last_update_information_ratio':recorded['information_eigen_ratio'],
        'H_decomposition_identity_max_abs_error':float(np.max(abs(h-hs-hf))),
        'old_geometry_final_A_original_covariance':old_spectrum,
        'fixed_old_geometry_pose_NN_only_unsupported_I_replacement':counterfactual_spectrum,
        'fixed_old_replacement_H_old_minus_H_new_eigenvalues':downgrade_eigen.tolist(),
        'fixed_old_support_counts':old_counts,
        'comparison_caveat':'PSD downgrade statement applies only to fixed old geometry/pose/NN and unsupported-I replacement. '
            'New bounded>=5 PCA normals, earlier local trajectory and target sampling also change. '
            'A larger min/max ratio alone does not imply more absolute information or greater pose accuracy.',
        'no_new_acceptance_threshold_or_solver_change':True,'GT_query_used':False,
        'H_full':h.tolist(),'H_supported':hs.tolist(),'H_fallback':hf.tolist()}
    out=new_directory(output);inputs.add(Path(__file__));write_json(out/'summary.json',report)
    inputs.save(out,stage='final information audit',exact_H_verified=error<1e-7 and quality_error<1e-7)
    print(__import__('json').dumps({k:v for k,v in report.items()if not k.startswith('H_')},indent=2))
    return error<1e-7 and quality_error<1e-7


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bounded',type=Path,required=True)
    p.add_argument('--original',type=Path,required=True);p.add_argument('--poses',type=Path,required=True)
    p.add_argument('--ordinal',type=int,required=True);p.add_argument('--radius',type=float,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    raise SystemExit(0 if run(a.bounded,a.original,a.poses,a.ordinal,a.radius,a.output)else 1)
