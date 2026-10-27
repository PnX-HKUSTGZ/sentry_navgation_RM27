#!/usr/bin/env python3
"""Fresh final-pose GN step, read-only; never apply it or change acceptance."""
import argparse
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from pipeline_common import Inputs,read_json,write_json,new_directory


def skew(x):
    return np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])


def run(reconstruction,input_path,output):
    inputs=Inputs();root=Path(reconstruction).resolve();plan=read_json(inputs.add(input_path))
    data=read_json(inputs.add(root/'factors.json'))
    source=read_json(inputs.add(root/'source_features.json'));target=read_json(inputs.add(root/'target_features.json'))
    p=np.array([x['point']for x in source]).reshape(-1,3);q=np.array([x['point']for x in target]).reshape(-1,3)
    cs=np.array([x['covariance']for x in source]);ct=np.array([x['covariance']for x in target])
    z=np.array(data['state_a']);r,t=z[:3,:3],z[:3,3];rr=np.array(data['guess'])[:3,:3]
    h=np.zeros((6,6));b=np.zeros(6);frozen=[]
    for item in data['points']:
        if not item['active_a']:continue
        i,j=item['source'],item['target_a'];w=np.linalg.inv(ct[j]+r@cs[i]@r.T)
        residual=q[j]-r@p[i]-t;jac=np.c_[r@skew(p[i]),-r]
        h+=jac.T@w@jac;b+=jac.T@w@residual;frozen.append((i,j,w))
    rot_residual=lambda rotation:Rotation.from_matrix(rr.T@rotation).as_rotvec()
    er=rot_residual(r);jr=np.zeros((3,6));epsilon=1e-6
    for k in range(3):
        d=np.eye(3)[k]*epsilon
        jr[:,k]=(rot_residual(r@Rotation.from_rotvec(d).as_matrix())-
                  rot_residual(r@Rotation.from_rotvec(-d).as_matrix()))/(2*epsilon)
    sb=np.linalg.inv(np.array(plan['base_from_sensor']))[:3,3]
    et=t+r@sb-sb;jt=np.c_[-r@skew(sb),r]
    wr,wt=data['rotation_prior_weight'],data['translation_prior_weight']
    hr=wr*jr.T@jr;br=wr*jr.T@er;ht=wt*jt.T@jt;bt=wt*jt.T@et
    total_h=h+hr+ht;total_b=b+br+bt;delta=np.linalg.solve(total_h,-total_b)
    # Independent central gradient check at fixed final NN/precision, matching
    # the stock factor's linearization model. No candidate is accepted/applied.
    def cost(d):
        omega=d[:3];angle=np.linalg.norm(omega);rotation=Rotation.from_rotvec(omega).as_matrix()
        if angle<1e-10:translation=rotation@d[3:]
        else:
            o=skew(omega);v=np.eye(3)+(1-np.cos(angle))/angle**2*o+(angle-np.sin(angle))/angle**3*(o@o)
            translation=v@d[3:]
        rn=r@rotation;tn=t+r@translation;value=0.
        for i,j,w in frozen:
            e=q[j]-rn@p[i]-tn;value+=.5*e@w@e
        eg=rot_residual(rn);eb=tn+rn@sb-sb
        return value+.5*wr*(eg@eg)+.5*wt*(eb@eb)
    gradient=np.array([(cost(np.eye(6)[k]*epsilon)-cost(-np.eye(6)[k]*epsilon))/(2*epsilon)for k in range(6)])
    rotation=float(np.linalg.norm(delta[:3]));translation=float(np.linalg.norm(delta[3:]))
    report={'read_only_fresh_final_linearization':True,'step_applied':False,'GT_query_used':False,
        'fresh_final_pinned_H_max_abs_error':float(np.max(abs(h-np.array(data['H_b'])))),
        'b_vs_frozen_factor_plus_prior_central_gradient_max_abs_error':float(np.max(abs(total_b-gradient))),
        'undamped_GN_increment':delta.tolist(),'rotation_step_rad':rotation,
        'rotation_step_deg':float(np.rad2deg(rotation)),'translation_step_m':translation,
        'original_stop_limits':{'rotation_deg':.1,'translation_m':.001},
        'fresh_step_within_both_original_limits':rotation<=np.deg2rad(.1)and translation<=.001,
        'fresh_LiDAR_b':b.tolist(),'rotation_prior_b':br.tolist(),'base_translation_prior_b':bt.tolist(),
        'LiDAR_H':h.tolist(),'rotation_prior_H':hr.tolist(),'base_translation_prior_H':ht.tolist(),
        'rotation_prior_weight':wr,'translation_prior_weight':wt,
        'relative_total_linear_system_residual':float(np.linalg.norm(total_h@delta+total_b)/max(1.,np.linalg.norm(total_b))),
        'scope':'next undamped GN direction at recorded final pose with freshly associated NN/W; not a further optimization, new output or global convergence proof'}
    out=new_directory(output);write_json(out/'summary.json',report);inputs.add(Path(__file__))
    inputs.save(out,stage='fresh fixed-point audit')
    print(__import__('json').dumps({k:v for k,v in report.items()if not k.endswith('_H')},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reconstruction',type=Path,required=True)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    run(a.reconstruction,a.input,a.output)
