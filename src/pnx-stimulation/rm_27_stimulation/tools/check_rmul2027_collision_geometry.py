#!/usr/bin/env python3
"""Compare collision LOD with CAD using dense support rays and horizontal rays."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector


ASSETS = Path(__file__).resolve().parents[1]/'meshes/RMUL2027'


def load_meshes():
    groups = json.loads((ASSETS/'geometry.json').read_text())['materials']
    cad = trimesh.util.concatenate([trimesh.Trimesh(g['vertices'], g['triangles'], process=False)
                                    for g in groups])
    parts = []
    for collision in ET.parse(ASSETS/'RMUL2027_collision/model.sdf').findall('.//collision'):
        geometry = collision.find('geometry')
        if geometry.find('box') is not None:
            mesh = trimesh.creation.box(np.fromstring(geometry.findtext('box/size'), sep=' '))
        else:
            shape = geometry.find('mesh')
            mesh = trimesh.load_mesh(ASSETS/shape.findtext('uri').removeprefix('model://'))
            mesh.apply_scale(np.fromstring(shape.findtext('scale'), sep=' '))
        pose = np.fromstring(collision.findtext('pose'), sep=' ')
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
        transform[:3, 3] = pose[:3]
        mesh.apply_transform(transform)
        parts.append(mesh)
    return cad, trimesh.util.concatenate(parts)


def hit_positions(mesh, origins, directions):
    hits, indices, faces = RayMeshIntersector(mesh).intersects_location(
        origins, directions, multiple_hits=False)
    positions = np.full(origins.shape, np.nan)
    normals = np.full(origins.shape, np.nan)
    positions[indices] = hits
    normals[indices] = mesh.face_normals[faces]
    return positions, normals


def external_cad_mesh():
    from OCP.BRepClass3d import BRepClass3d
    from import_rmul2027_world import (BRepMesh_IncrementalMesh, REPOSITORY, TopAbs_FACE,
                                      TopAbs_SOLID, TopoDS, face_mesh, read_step, shapes, static)
    document, roots, colors = read_step(REPOSITORY/'RMUL2027_3V3_V1.0.0.stp')
    triangles = []
    for root in roots:
        BRepMesh_IncrementalMesh(root,.5,False,.2,True)
        for solid in shapes(root,TopAbs_SOLID):
            shell = static(BRepClass3d,'OuterShell',static(TopoDS,'Solid',solid))
            for face in shapes(shell,TopAbs_FACE):
                vertices,faces = face_mesh(static(TopoDS,'Face',face))
                triangles.extend(vertices[faces])
    triangles = np.asarray(triangles)
    return trimesh.Trimesh(vertices=triangles.reshape(-1,3),
                           faces=np.arange(triangles.size//3).reshape(-1,3),process=False)


def collision_contains(points):
    inside = np.zeros(len(points),dtype=bool)
    for collision in ET.parse(ASSETS/'RMUL2027_collision/model.sdf').findall('.//collision'):
        pose = np.fromstring(collision.findtext('pose'),sep=' ')
        rotation = Rotation.from_quat(pose[3:]).as_matrix()
        shape = collision.find('geometry/box')
        is_box = shape is not None
        if is_box:
            size = np.fromstring(shape.findtext('size'),sep=' ')
            center = pose[:3]
        else:
            size = np.fromstring(collision.findtext('geometry/mesh/scale'),sep=' ')
            center = pose[:3]+rotation@np.array([size[0]/2,size[1]/2,0])
        half = np.abs(rotation)@(size/2)
        indices = np.flatnonzero(np.all(np.abs(points-center)<=half+1e-8,axis=1))
        local = (points[indices]-pose[:3])@rotation
        if is_box:
            selected = np.all(np.abs(local)<=size/2+1e-8,axis=1)
        else:
            normalized = local/size
            selected = ((normalized[:,0]>=-1e-8)&(normalized[:,1]>=-1e-8)&
                        (normalized[:,0]+normalized[:,1]<=1+1e-8)&
                        (np.abs(normalized[:,2])<=.5+1e-8))
        inside[indices[selected]] = True
    return inside


def compare_free_space(cad):
    x,y,z = np.meshgrid(np.arange(-6.23817,6.3,.125),np.arange(-4.23139,4.3,.125),
                        [.005,.025,.075,.15,.225,.24,.26,.325,.375,.39,.41],indexing='ij')
    points = np.column_stack((x.ravel(),y.ravel(),z.ravel()))
    reference = RayMeshIntersector(external_cad_mesh()).contains_points(points)
    actual = collision_contains(points)
    indices = np.flatnonzero(reference!=actual)
    distance = np.array([])
    if len(indices):
        _,distance,_ = trimesh.proximity.closest_point(cad,points[indices])
        bad = distance>.00401
        assert not bad.any(), dict(point=points[indices[bad][0]].tolist(),
                                   cad_solid=bool(reference[indices[bad][0]]),
                                   collision_solid=bool(actual[indices[bad][0]]))
    return dict(volume_samples=len(points),external_free_samples=int((~reference).sum()),
                solid_samples=int(reference.sum()),
                max_volume_disagreement_boundary_distance_m=float(distance.max()) if len(distance) else 0.0)


def compare():
    cad, collision = load_meshes()
    # Unequal offsets avoid placing entire sample rows exactly on 45-degree
    # CAD edges, whose zero-area ownership changes with STL float rounding.
    x, y = np.meshgrid(np.arange(-6.2874137, 6.3, .025), np.arange(-4.2873191, 4.3, .025))
    origins = np.column_stack((x.ravel(), y.ravel(), np.ones(x.size)))
    down = np.tile([0, 0, -1], (len(origins), 1))
    expected, _ = hit_positions(cad, origins, down)
    actual, _ = hit_positions(collision, origins, down)
    supported = np.isfinite(expected[:, 2])
    if not np.isfinite(actual[supported]).all():
        raise AssertionError('Simplified collisions have holes in CAD ground support')
    height_error = np.abs(actual[supported, 2]-expected[supported, 2])
    # Limit includes the 3 mm detail LOD and retained 1 mm inward skins.
    assert height_error.max() < .00401, (height_error.max(), origins[supported][height_error.argmax()])

    origins = np.array([[x, y, z] for x in [-5, -1, 0, 1, 5]
                        for y in [-3, -1, 0, 1, 3] for z in [.05, .15, .23, .35, .41]])
    angles = np.arange(0, 360, 2)*np.pi/180
    directions = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    rays = np.tile(directions, (len(origins), 1))
    origins = np.repeat(origins, len(directions), axis=0)
    expected, normal = hit_positions(cad, origins, rays)
    actual, _ = hit_positions(collision, origins, rays)
    valid = np.isfinite(expected[:, 0]) & (np.abs(normal[:, 2]) < .01)
    assert np.isfinite(actual[valid]).all(), 'An external CAD wall disappeared'
    boundary_error = np.linalg.norm(actual[valid]-expected[valid], axis=1)
    # Grazing rays amplify an unchanged 1 mm collision skin in range. The
    # normal distance measures whether the actual obstacle boundary moved.
    normal_error = np.abs(np.sum((actual[valid]-expected[valid])*normal[valid], axis=1))
    assert normal_error.max() < .00401, (normal_error.max(), origins[valid][normal_error.argmax()])
    return dict(**compare_free_space(cad),ground_samples=int(supported.sum()), max_height_error_m=float(height_error.max()),
                height_error_p99_m=float(np.quantile(height_error, .99)),
                wall_samples=int(valid.sum()), max_wall_ray_error_m=float(boundary_error.max()),
                max_wall_normal_error_m=float(normal_error.max()),
                wall_ray_error_p99_m=float(np.quantile(boundary_error, .99)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = compare()
    print(json.dumps(report, indent=2))
    if args.output:
        args.output.write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
