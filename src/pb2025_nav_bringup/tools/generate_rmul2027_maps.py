#!/usr/bin/env python3
"""Generate aligned RMUL2027 localization PCD and uninflated navigation grid.

Uses the CAD Python environment created for the RMUL2027 world importer.
"""

import argparse
from collections import deque
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
SIMULATION = BRINGUP.parent/'rm_27_stimulation'


def load_geometry(path):
    groups = json.loads(path.read_text())['materials']
    triangles = np.concatenate([np.asarray(g['vertices'])[np.asarray(g['triangles'])] for g in groups])
    mesh = trimesh.Trimesh(vertices=triangles.reshape(-1,3),
                           faces=np.arange(len(triangles)*3).reshape(-1,3),process=False)
    return mesh,RayMeshIntersector(mesh)


def terrain_grid(mesh, rays, resolution=.05, subdivisions=5):
    fine = resolution/subdivisions
    origin = np.array([-6.5,-4.5])
    width,height = 260,180
    fw,fh = width*subdivisions,height*subdivisions
    xx,yy = np.meshgrid(origin[0]+(np.arange(fw)+.5)*fine,
                         origin[1]+(np.arange(fh)+.5)*fine)
    positions = np.column_stack((xx.ravel(),yy.ravel(),np.ones(fw*fh)))
    ids = np.full(fw*fh,-1,dtype=int)
    for start in range(0,len(ids),100000):
        p = positions[start:start+100000]
        ids[start:start+len(p)] = rays.intersects_first(p,np.tile([0,0,-1],(len(p),1)))
    known = ids>=0
    heights = np.full(len(ids),np.nan)
    normals = mesh.face_normals[np.maximum(ids,0)]
    first = mesh.triangles[np.maximum(ids,0),0]
    heights[known] = first[known,2] + (
        normals[known,0]*(first[known,0]-positions[known,0])+
        normals[known,1]*(first[known,1]-positions[known,1]))/normals[known,2]
    candidate = known & (np.abs(normals[:,2])>=np.cos(np.deg2rad(28))) & (heights>=-.005)
    heights = heights.reshape(fh,fw)
    candidate = candidate.reshape(fh,fw)
    reached = np.zeros((fh,fw),bool)
    seed = (int(-origin[1]/fine),int(-origin[0]/fine))
    if not candidate[seed]:
        raise RuntimeError('Origin is not on a walkable CAD surface')
    queue=deque([seed]); reached[seed]=True
    # Small markings / mats can be crossed; a 20 cm platform edge cannot.
    while queue:
        y,x=queue.popleft()
        for dy,dx in ((0,1),(0,-1),(1,0),(-1,0)):
            ny,nx=y+dy,x+dx
            if (0<=ny<fh and 0<=nx<fw and candidate[ny,nx] and not reached[ny,nx]
                    and abs(heights[ny,nx]-heights[y,x])<=.020001):
                reached[ny,nx]=True;queue.append((ny,nx))
    occupied = known.reshape(fh,fw)&~reached
    # Rasterize each obstacle once; costmaps / oriented footprints add their
    # own clearance later. Subcells retain narrow walls missed by centre rays.
    any_obstacle = occupied.reshape(height,subdivisions,width,subdivisions).any((1,3))
    all_free = reached.reshape(height,subdivisions,width,subdivisions).all((1,3))
    pixels=np.full((height,width),205,dtype=np.uint8)
    pixels[all_free]=254
    pixels[any_obstacle]=0
    camera_mask=np.zeros_like(reached)
    camera_mask[subdivisions::75,subdivisions::75]=True
    camera_mask &= reached
    cy,cx=np.nonzero(camera_mask)
    cameras=np.column_stack((xx[cy,cx],yy[cy,cx],heights[cy,cx]+.21))
    return np.flipud(pixels),origin,cameras


def surface_samples(mesh, spacing):
    rng=np.random.default_rng(2027)
    chunks=[]
    for triangle,area in zip(mesh.triangles,mesh.area_faces):
        count=max(1,int(np.ceil(area/spacing**2)))
        uv=rng.random((count,2))
        uv[uv.sum(axis=1)>1]=1-uv[uv.sum(axis=1)>1]
        points=triangle[0]+uv[:,0,None]*(triangle[1]-triangle[0])+uv[:,1,None]*(triangle[2]-triangle[0])
        chunks.extend((points,triangle))
    points=np.concatenate(chunks)
    return points[np.isfinite(points).all(axis=1)&(points[:,2]>=-.002)]


def visible_samples(points,cameras,rays):
    tree=cKDTree(cameras[:,:2])
    output=[]
    for start in range(0,len(points),4096):
        chunk=points[start:start+4096]
        _,neighbours=tree.query(chunk[:,:2],k=min(12,len(cameras)))
        visible=np.zeros(len(chunk),bool)
        for k in range(neighbours.shape[1]):
            indices=np.flatnonzero(~visible)
            if not len(indices): break
            origins=cameras[neighbours[indices,k]]
            directions=chunk[indices]-origins
            distances=np.linalg.norm(directions,axis=1)
            directions/=distances[:,None]
            hits,ray_ids,_=rays.intersects_location(origins,directions,multiple_hits=False)
            residual=np.abs(np.linalg.norm(hits-origins[ray_ids],axis=1)-distances[ray_ids])
            visible[indices[ray_ids[residual<.003]]]=True
        output.append(chunk[visible])
    return np.concatenate(output)


def voxel_centroids(points,leaf):
    keys=np.floor(points/leaf).astype(np.int64)
    _,inverse=np.unique(keys,axis=0,return_inverse=True)
    count=np.bincount(inverse)
    return np.column_stack([np.bincount(inverse,weights=points[:,axis])/count for axis in range(3)])


def write_pcd(path,points):
    header=(f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
            f'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
            f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
            f'POINTS {len(points)}\nDATA binary\n')
    with path.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(np.asarray(points,dtype='<f4').tobytes())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--voxel',type=float,default=.05)
    args=parser.parse_args()
    if not .01<=args.voxel<=.10: parser.error('voxel must be between 0.01 and 0.10 metres')
    geometry=SIMULATION/'meshes/RMUL2027/geometry.json'
    mesh,rays=load_geometry(geometry)
    pixels,origin,cameras=terrain_grid(mesh,rays)
    print('Terrain raster complete; visibility viewpoints:',len(cameras),flush=True)
    samples=surface_samples(mesh,.02)
    visible=visible_samples(samples,cameras,rays)
    points=voxel_centroids(visible,args.voxel)
    map_dir=BRINGUP/'map/simulation'; pcd_dir=BRINGUP/'pcd/simulation'
    map_dir.mkdir(parents=True,exist_ok=True);pcd_dir.mkdir(parents=True,exist_ok=True)
    write_pcd(pcd_dir/'RMUL2027.pcd',points)
    image=Image.fromarray(pixels)
    image.save(map_dir/'RMUL2027.pgm')
    image.resize((1040,720),resample=Image.Resampling.NEAREST).save(map_dir/'RMUL2027_preview.png')
    config=dict(image='RMUL2027.pgm',mode='trinary',resolution=.05,
                origin=[float(origin[0]),float(origin[1]),0.0],negate=0,
                occupied_thresh=.65,free_thresh=.196)
    (map_dir/'RMUL2027.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    metadata=dict(geometry_sha256=hashlib.sha256(geometry.read_bytes()).hexdigest(),
                  coordinate_frame='map = Gazebo world',pcd_transform=[0.0]*6,
                  sample_spacing_m=.02,voxel_size_m=args.voxel,sampled_points=len(samples),
                  visible_points=len(visible),pcd_points=len(points),
                  pcd_bounds=[points.min(0).tolist(),points.max(0).tolist()],
                  camera_viewpoints=len(cameras),map_resolution=.05,raster_subsample_resolution=.01,
                  ground_step_limit_m=.02,ground_slope_limit_degrees=28,
                  free_cells=int((pixels==254).sum()),occupied_cells=int((pixels==0).sum()),
                  unknown_cells=int((pixels==205).sum()))
    (map_dir/'RMUL2027_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':
    main()
