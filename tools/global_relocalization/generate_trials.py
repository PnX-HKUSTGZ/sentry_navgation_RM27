#!/usr/bin/env python3
"""Freeze 30 planar free-space starts x four headings, with a reproducible seed."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--map-yaml',required=True)
p.add_argument('--output',required=True)
p.add_argument('--seed',type=int,default=20261013)
p.add_argument('--positions',type=int,default=30)
p.add_argument('--clearance',type=float,default=.50)
p.add_argument('--search-side',choices=['all','negative_x'],default='all')
p.add_argument('--side-boundary-x',type=float,default=0.)
a=p.parse_args()
path=Path(a.map_yaml).resolve();m=yaml.safe_load(path.read_text());res=m['resolution'];origin=m['origin']
image=np.array(Image.open(path.parent/m['image']))
free=image>=254
mask=distance_transform_edt(free)*res>=a.clearance
if m.get('ground_elevation',{}).get('grid'):
 grid=m['ground_elevation']['grid'];e=np.array(Image.open(path.parent/grid['image']))
 h=e.astype(float)*grid['scale']+grid['offset']
 mask &= (e!=grid['no_data']) & (np.abs(h)<.01)
else:
 h=np.zeros(image.shape)
ys,xs=np.where(mask);rng=np.random.default_rng(a.seed);order=rng.permutation(len(xs));chosen=[]
for i in order:
 x=origin[0]+(int(xs[i])+.5)*res;y=origin[1]+(image.shape[0]-int(ys[i])-.5)*res
 if a.search_side=='negative_x' and not x<a.side_boundary_x:
  continue
 if all(math.hypot(x-px,y-py)>=1. for px,py,_ in chosen):
  chosen.append((x,y,float(h[ys[i],xs[i]])))
 if len(chosen)==a.positions:
  break
if len(chosen)!=a.positions:
 raise SystemExit('Not enough disjoint flat free starts')
trials=[]
for pi,(x,y,z) in enumerate(chosen):
 for yi,yaw in enumerate([0.,math.pi/2,math.pi,-math.pi/2]):
  trials.append({'trial_id':f'p{pi:02d}_yaw{yi}','trial_index':len(trials),'requested_pose':{'x':x,'y':y,'z':z+.08,'yaw':yaw}})
output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True)
output.write_text(json.dumps({'seed':a.seed,'sampling_map':str(path),'clearance':a.clearance,'count':len(trials),
 'search_prior':{'search_side':a.search_side,'side_boundary_x':a.side_boundary_x,
  'source':'user-authorized fixed half-field prior' if a.search_side!='all' else 'none'},'trials':trials},indent=2)+'\n')
print(f'Frozen {len(trials)} trials in {output}')
