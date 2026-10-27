"""Pure helpers for an ordered, geometry-selected mapping survey protocol."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def points_hash(points):
    return hashlib.sha256(np.ascontiguousarray(points,dtype='<f8').tobytes()).hexdigest()


def route_arclength(start,waypoints):
    points=np.asarray([start,*waypoints],dtype=float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all():raise ValueError('route must be a finite ordered Nx2 polyline')
    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]


def select_arc_stations(start,waypoints,supported,heights,spacing=2.5):
    """Nearest existing supported vertex to each fixed arc target, plus closure."""
    if not math.isfinite(spacing)or spacing<=0:raise ValueError('station spacing must be positive')
    points=np.asarray([start,*waypoints],float);arc=route_arclength(start,waypoints);mask=np.asarray(supported,bool);z=np.asarray(heights,float)
    if mask.shape!=(len(points),)or z.shape!=mask.shape or not np.isfinite(z[mask]).all():raise ValueError('support inventory mismatch')
    if not mask[0]or not mask[-1]:raise ValueError('initial and final closure stations need valid full-turn geometry')
    def row(i,kind,target=None):
        return {'id':f'visit_{i:04d}_{kind}','waypoint_index':int(i-1),'kind':kind,'xy':points[i].tolist(),'arc_length_m':float(arc[i]),
                'actual_support_height_m':float(z[i]),'requested_arc_target_m':target,'full_turn_supported':True}
    stations=[row(0,'initial')];used={0,len(points)-1};missed=[]
    eligible=np.flatnonzero(mask);eligible=eligible[(eligible>0)&(eligible<len(points)-1)]
    for target in np.arange(spacing,arc[-1],spacing):
        if not len(eligible):missed.append(float(target));continue
        i=int(min(eligible,key=lambda j:(abs(arc[j]-target),j)))
        if abs(arc[i]-target)>spacing/2 or i in used:missed.append(float(target));continue
        used.add(i);stations.append(row(i,'arc',float(target)))
    stations.sort(key=lambda s:s['waypoint_index']);closure=row(len(points)-1,'closure');closure['revisit_of_station_id']=stations[0]['id'];stations.append(closure)
    return stations,missed


def read_ordered_protocol(path,start):
    data=json.loads(Path(path).read_text())
    if data.get('schema')!='ordered_mapping_survey_v1' or data.get('preserve_waypoint_order')is not True:raise ValueError('not an ordered mapping protocol')
    route=data['waypoints'];arc=route_arclength(start,route)
    if not np.allclose(start,data['start_xy'],atol=1e-12,rtol=0)or points_hash(route)!=data['ordered_waypoints_sha256']:raise ValueError('route start/order/hash differs')
    if not math.isclose(float(arc[-1]),data['length_m'],abs_tol=1e-9):raise ValueError('ordered route length differs')
    stations=data['stations'];ids=[s['id']for s in stations];indices=[s['waypoint_index']for s in stations]
    if len(ids)!=len(set(ids))or indices!=sorted(set(indices))or indices[0]!=-1 or indices[-1]!=len(route)-1:raise ValueError('station identities/indices invalid')
    if stations[0]['kind']!='initial'or stations[-1]['kind']!='closure'or stations[-1].get('revisit_of_station_id')!=stations[0]['id']:raise ValueError('explicit initial/closure visits required')
    for s in stations:
        index=s['waypoint_index'];expected=start if index==-1 else route[index]
        if index< -1 or index>=len(route)or not np.array_equal(np.array(s['xy']),np.array(expected))or s.get('full_turn_supported')is not True or not math.isfinite(s['actual_support_height_m']):raise ValueError('station not an exact supported original route vertex')
    for name,digest in data['source_hashes'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:raise ValueError('protocol source changed: '+name)
    return data


def validate_route_grid(planner,start,route):
    """Check supplied segments in order; never replan or reorder them."""
    previous=planner.cell(start)
    for a,b in zip([start,*route[:-1]],route):
        count=max(1,int(math.ceil(math.dist(a,b)/(planner.resolution/2))))
        for xy in np.linspace(a,b,count+1):
            cell=planner.cell(xy);r,c=cell
            if not(0<=r<planner.safe.shape[0]and 0<=c<planner.safe.shape[1])or not planner.safe[cell]:raise ValueError('ordered route leaves original traversable grid')
            if cell[0]!=previous[0]and cell[1]!=previous[1]and(not planner.safe[previous[0],cell[1]]or not planner.safe[cell[0],previous[1]]):raise ValueError('ordered route cuts an inflated obstacle corner')
            previous=cell


def settle_sensor_complete(start,latest,required):
    if not all(math.isfinite(x)for x in [start,latest,required])or required<0:raise ValueError('invalid station settling clock/duration')
    if latest<start-1e-8:raise ValueError('station sensor clock moved backwards')
    return latest-start+1e-8>=required
