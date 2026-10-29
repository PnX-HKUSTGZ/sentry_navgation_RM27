import sys,json,numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,'tools/navigation_benchmark')
from geometry_audit import *
from metrics import analyze_run
s=CollisionSupportScene.from_world(WORLD)
def seg(p,a,b):
 v=b-a; t=np.clip(np.dot(p-a,v)/max(np.dot(v,v),1e-30),0,1);return np.linalg.norm(p-a-t*v)
def dist(p, tri):
 a,b,c=tri; n=np.cross(b-a,c-a); norm=np.dot(n,n)
 if norm>1e-24:
  q=p - n*(np.dot(p-a,n)/norm); v0=b-a;v1=c-a;v2=q-a; d00=np.dot(v0,v0);d01=np.dot(v0,v1);d11=np.dot(v1,v1);d20=np.dot(v2,v0);d21=np.dot(v2,v1); den=d00*d11-d01*d01
  if den>1e-24:
   v=(d11*d20-d01*d21)/den;w=(d00*d21-d01*d20)/den
   if v>=0 and w>=0 and v+w<=1:return np.linalg.norm(p-q)
 return min(seg(p,a,b),seg(p,b,c),seg(p,c,a))
p='tools/navigation_benchmark/results/20261028_followup/full_chain_gravity_seed42/repeat_01/run_01.json'; d=json.load(open(p));sc=json.load(open('tools/navigation_benchmark/scenario.json')); eps=[e for e in analyze_run(d,scenario=sc)['episodes'] if e['unexpected_candidate']]
for e in eps:
 arr=[v for v in d['samples'] if e['start_time']<=v['t']<=e['end_time']]; v=arr[len(arr)//2]; p=np.array([v[k] for k in 'xyz']); r=Rotation.from_quat(v['quaternion_xyzw']).as_matrix(); rows=[]
 for px in [-.218,.218]:
  for py in [-.218,.218]:
   center=p+r@np.array([px,py,0]); near=np.all(s.maximum>=center-.18,axis=1)&np.all(s.minimum<=center+.18,axis=1)
   ds=[(dist(center,s.triangles[i]),s.names[i].split('#')[0]) for i in np.flatnonzero(near) if 'barrier' in s.names[i]]
   if ds: rows.append((min(ds),px,py))
 if rows:
  (distance,name),px,py=min(rows);print(round(v['t'],2),round(distance,5),round(distance-.08,5),name.split('/')[-1],px,py)
