"""Small independent test of the exact wrapper executed in the dense control."""
import ast
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import csr_matrix

def test_dense_wrapper_preserves_objective_options_and_recovers_known_solution(tmp_path):
 source=Path(__file__).with_name('optimize_partial_nine_dense.py').read_text();tree=ast.parse(source);function=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='observed_least_squares')
 A=np.array([[1.,0,0,0],[0,10.,0,0],[0,0,100.,0],[0,0,0,1000.],[1.,2.,0,0],[0,1.,2.,3.]])
 truth=np.array([.03,-.02,.015,-.01]);b=A@truth
 def residual(x):return A@x-b
 probes=[np.zeros(4),np.array([.001,-.001,.001,-.001]),np.linspace(-.0001,.0001,4)];baseline={'values':[residual(x)for x in probes],'central_J':A,'sparsity_shape':A.shape};received={}
 def original_solver(*args,**kwargs):
  received.update(kwargs)
  assert args[0]is residual
  return least_squares(*args,**kwargs)
 record={};namespace={'np':np,'json':json,'OUT':tmp_path,'baseline':baseline,'probes':probes,'original_lsq':original_solver,'solver_record':record};exec(compile(ast.Module(body=[function],type_ignores=[]),'executed_dense_wrapper','exec'),namespace)
 options={'jac_sparsity':csr_matrix(np.ones_like(A)),'loss':'huber','f_scale':2.,'max_nfev':600,'ftol':1e-6,'xtol':1e-6,'gtol':1e-6};fit=namespace['observed_least_squares'](residual,np.zeros(4),**options)
 assert fit.success and np.allclose(fit.x,truth,rtol=0,atol=1e-8)
 assert received['tr_solver']=='exact'and'jac_sparsity'not in received
 assert all(received[k]==options[k]for k in ['loss','f_scale','max_nfev','ftol','xtol','gtol'])
 assert'x_scale'not in received and'jac_sparsity'in options
 check=json.loads((tmp_path/'residual_jacobian_equivalence.json').read_text())
 assert check['same_legacy_objective_residual_exact_at_all3probes']
 assert check['maximum_absolute_residual_difference']==0
 assert check['Jacobian_relative_frobenius_difference']<1e-5
