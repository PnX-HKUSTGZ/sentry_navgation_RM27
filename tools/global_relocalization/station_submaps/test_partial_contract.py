"""Audit the executed evaluator without executing the experiment."""
import ast
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

def test_fixed_domain_keeps_invalid_normal_and_overdistance_at_cap():
 source=Path(__file__).with_name('partial_overlap_v2.py').read_text();tree=ast.parse(source);fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='evaluate');module=ast.Module(body=[fn],type_ignores=[])
 x,y=np.meshgrid(np.arange(4.),np.arange(4.));p=np.c_[x.ravel(),y.ravel(),np.zeros(16)];target=np.r_[p,p+[0,0,.1]];normal=np.tile([0.,0.,1.],(32,1));good=np.r_[np.ones(16,bool),np.zeros(16,bool)];stations=[{'held':np.r_[p,p],'hf':np.r_[np.full(16,30),np.full(16,31)],'ordinals':[30,31]},{'train':target,'geom':(cKDTree(target),normal,good)}];namespace={'np':np,'stations':stations,'transform':lambda p,t:p@t[:3,:3].T+t[:3,3]};exec(compile(module,'executed_evaluator','exec'),namespace);evaluate=namespace['evaluate'];domain={'hi':np.arange(32)};identity=np.eye(4);before=evaluate(0,1,domain,identity);bad=identity.copy();bad[2,3]=.1;after=evaluate(0,1,domain,bad)
 assert before['frame_equal_coverage']==1 and before['every_frame_min15']
 assert before['frame_equal_plane_rmse_m']==0
 assert after['fixed_domain_points']==32 and after['post_support_lost_points']==32
 assert after['frame_equal_coverage']==0 and not after['every_frame_min15']
 assert abs(after['frame_equal_plane_rmse_m']-.3)<1e-12
 assert abs(after['frame_equal_capped_nn_rmse_m']-.3)<1e-12
 bad[2,3]=1.;far=evaluate(0,1,domain,bad)
 assert far['fixed_domain_points']==32 and abs(far['frame_equal_capped_nn_rmse_m']-.3)<1e-12
 assert np.array_equal(domain['hi'],np.arange(32))

def test_contiguous_scan_split_preserves_non80_frame_tail():
 for count in [80,81]:
  n=count//4;start=(count-n)//2;held=set(range(start,start+n));train=set(range(count))-held
  assert len(held)==20 and len(held|train)==count and not(held&train)
  assert count-1 in train

def test_good_normal_tree_uses_supported_surface_and_preserves_true_overrange_cap():
 from supported_tree import GoodNormalTree
 source=Path(__file__).with_name('partial_overlap_v2.py').read_text();parsed=ast.parse(source);fn=next(x for x in parsed.body if isinstance(x,ast.FunctionDef)and x.name=='evaluate')
 x,y=np.meshgrid(np.arange(4.),np.arange(4.));p=np.c_[x.ravel(),y.ravel(),np.zeros(16)];target=np.r_[p,p+[0,0,.1]];good=np.r_[np.ones(16,bool),np.zeros(16,bool)];normal=np.tile([0.,0.,1.],(32,1));tree=GoodNormalTree(target,good);stations=[{'held':np.r_[p,p],'hf':np.r_[np.full(16,30),np.full(16,31)],'ordinals':[30,31]},{'train':target,'geom':(tree,normal,good)}];namespace={'np':np,'stations':stations,'transform':lambda p,t:p@t[:3,:3].T+t[:3,3]};exec(compile(ast.Module(body=[fn],type_ignores=[]),'same_fixed_evaluator','exec'),namespace);evaluate=namespace['evaluate'];D={'hi':np.arange(32)};t=np.eye(4);t[2,3]=.1;distance,original=tree.query(target[16:]);assert np.all(original<16) and np.allclose(distance,.1);r=evaluate(0,1,D,t)
 assert r['fixed_domain_points']==32 and r['frame_equal_coverage']==1 and r['post_support_lost_points']==0
 assert abs(r['frame_equal_plane_rmse_m']-.1)<1e-12
 assert abs(r['frame_equal_capped_nn_rmse_m']-.1)<1e-12
 t[2,3]=.31;r=evaluate(0,1,D,t)
 assert r['fixed_domain_points']==32 and r['frame_equal_coverage']==0 and r['post_support_lost_points']==32
 assert abs(r['frame_equal_plane_rmse_m']-.3)<1e-12 and abs(r['frame_equal_capped_nn_rmse_m']-.3)<1e-12
 assert np.array_equal(D['hi'],np.arange(32))
