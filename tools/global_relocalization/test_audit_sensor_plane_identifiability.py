import numpy as np
from scipy.spatial import cKDTree
from audit_sensor_plane_identifiability import RULES, inside_triangle, local_cells, supported_intersections


def test_triangle_support_has_no_inflation():
    a=np.tile([0.,0,0],(3,1));b=np.tile([1.,0,0],(3,1));c=np.tile([0.,1,0],(3,1))
    p=np.array([[.2,.2,0],[.9,.9,0],[-1e-6,.1,0]])
    assert inside_triangle(p,a,b,c).tolist()==[True,False,False]


def wall_points():
    az=np.arange(359)*RULES['horizontal_step_rad'];el=RULES['vertical_min_rad']+np.arange(192)*RULES['vertical_step_rad'];a,e=np.meshgrid(az,el)
    direction=np.stack([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)],axis=-1)
    valid=(direction[:,:,0]>.8)&(e<.6)
    p=direction[valid]*(3/direction[:,:,0][valid])[:,None]
    return p


def test_observed_planar_cells_predict_actual_intersections_only():
    vertices,normals,metrics=local_cells(wall_points());assert metrics['support_cells']>100
    centers=vertices.mean(axis=1);tree=cKDTree(centers);frame=np.zeros(len(vertices),int);blocks={0:0}
    az=4.5*RULES['horizontal_step_rad'];el=RULES['vertical_min_rad']+55.5*RULES['vertical_step_rad']
    direction=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]);distance=3/direction[0]
    result=supported_intersections(np.zeros(3),direction,distance,vertices,normals,centers,tree,frame,blocks)
    assert 0 in result and abs(result[0]['depth_m']-distance)<1e-8
    # A ray just outside the observed wall strip must remain unknown even if a
    # nearby same-plane cell could be extrapolated to cover it.
    direction=np.array([1.,.1,1.]);direction/=np.linalg.norm(direction);distance=3/direction[0]
    assert supported_intersections(np.zeros(3),direction,distance,vertices,normals,centers,tree,frame,blocks)=={}
    # A plane observed only from its front cannot certify a query on its back.
    assert supported_intersections(np.array([6.,0,0]),np.array([-1.,0,0]),3,vertices,normals,centers,tree,frame,blocks)=={}


def test_missing_native_observations_do_not_become_a_filled_global_hull():
    points=wall_points();points=points[abs(points[:,1])>.3]
    vertices,normals,_=local_cells(points);centers=vertices.mean(axis=1);tree=cKDTree(centers)
    # Large wall portions remain on both sides, but the actual unsupported hole
    # must never be filled by fitting one global plane/convex hull.
    assert supported_intersections(np.zeros(3),np.array([1.,0,0]),3,vertices,normals,centers,tree,np.zeros(len(vertices),int),{0:0})=={}
