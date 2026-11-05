#!/usr/bin/env python3
"""Convert the official RMUC2027 STEP assembly to a metre-based map-frame PCD.

Offline dependencies are listed in rmuc2027_requirements.txt. Intermediate CAD
meshes live in the user cache, not in the ROS source tree. See the validation
report for the source, coordinate convention, and limitations of the CAD prior.
"""

import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / 'src/pnx-launch/launch_bringup'
SOURCE_URL = ('https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/'
              '080ae7230f6c469c9bec9d2c892797b61791994813953/RMUC2027_V1.0.0.stp')
SOURCE_MD5 = '3d3d6a411f44e9a4983794261b3f0bf7'
# The existing RM27 model.sdf has (-.02, -1.624344, 1.641344), and its
# RMUC2026_world.world instance has (0, +1.624344, 0). Rotations are identity.
TRANSLATION = np.array([-.02, 0., 1.641344])


def digest(path, algorithm='sha256'):
    h = hashlib.new(algorithm)
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            h.update(block)
    return h.hexdigest()


def read_pcd(path):
    with path.open('rb') as stream:
        header = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError('PCD header has no DATA line')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, _, value = line.partition(' ')
            header[key] = value
            if key == 'DATA':
                break
        if (header['FIELDS'] != 'x y z' or header['SIZE'] != '4 4 4'
                or header['TYPE'] != 'F F F' or header['DATA'] != 'binary'):
            raise ValueError('Expected binary float32 XYZ PCD')
        points = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 3)
    if len(points) != int(header['POINTS']):
        raise ValueError('PCD point count mismatch')
    return points


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (f'# .PCD v0.7 - Point Cloud Data file format\n'
              f'# frame_id map; units metres; CAD transform already applied\n'
              f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    temporary = path.with_suffix('.pcd.tmp')
    with temporary.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(np.asarray(points, dtype='<f4').tobytes())
    temporary.replace(path)


def make_mesh(step, cache):
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS
    from OCP.TopLoc import TopLoc_Location

    if digest(step, 'md5') != SOURCE_MD5:
        raise ValueError('STEP does not match the official RMUC2027 V1.0.0 MD5')
    print('Reading official STEP assembly (all component placements retained)', flush=True)
    reader = STEPControl_Reader()
    if reader.ReadFile(str(step)) != IFSelect_RetDone:
        raise RuntimeError('STEP reader failed')
    print('Transferring STEP roots', flush=True)
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError('Empty CAD assembly')
    print('Tessellating assembly, 2 mm absolute / 0.3 rad deflection', flush=True)
    mesher = BRepMesh_IncrementalMesh(shape, 2.0, False, .3, True)
    if not mesher.IsDone():
        raise RuntimeError('CAD tessellation failed')
    triangles = []
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    faces = 0
    missing = []
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is None:
            # Fine curved CAD details occasionally need a smaller deflection.
            BRepMesh_IncrementalMesh(face, .1, False, .1, False)
            mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is None:
            props = GProp_GProps()
            BRepGProp.SurfaceProperties_s(face, props)
            missing.append(dict(face_index=faces, area_mm2=props.Mass()))
            print('Untriangulated face:', missing[-1], flush=True)
        else:
            vertices = np.array([
                (p.X(), p.Y(), p.Z()) for p in
                (mesh.Node(i).Transformed(location.Transformation())
                 for i in range(1, mesh.NbNodes() + 1))]) * .001 + TRANSLATION
            indices = np.array([mesh.Triangle(i).Get()
                                for i in range(1, mesh.NbTriangles() + 1)]) - 1
            if face.Orientation() == TopAbs_REVERSED:
                indices = indices[:, ::-1]
            triangles.append(vertices[indices].astype(np.float32))
        faces += 1
        if faces % 10000 == 0:
            print('Extracted CAD faces:', faces, flush=True)
        explorer.Next()
    triangles = np.concatenate(triangles)
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache, triangles=triangles)
    info = dict(source_url=SOURCE_URL, source_md5=SOURCE_MD5,
                source_sha256=digest(step), source_bytes=step.stat().st_size,
                cad_faces=faces, triangles=len(triangles),
                untriangulated_faces=missing,
                tessellation_linear_deflection_mm=2.,
                tessellation_angular_deflection_rad=.3,
                cad_to_map_scale=.001, cad_to_map_rotation=np.eye(3).tolist(),
                cad_to_map_translation_m=TRANSLATION.tolist(),
                mesh_bounds_m=[triangles.min((0, 1)).tolist(),
                               triangles.max((0, 1)).tolist()])
    cache.with_suffix('.json').write_text(json.dumps(info, indent=2) + '\n')
    print(json.dumps(info, indent=2), flush=True)
    if any(x['area_mm2'] > 1. for x in missing):
        raise RuntimeError('Nontrivial CAD faces have no triangulation; inspect cache metadata')


def voxel_centroids(points, leaf):
    keys = np.floor(points / leaf).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    return np.column_stack([np.bincount(inverse, weights=points[:, axis]) / counts
                            for axis in range(3)]).astype(np.float32)


def sample_surface(triangles, spacing, bottom_cutoff):
    rng = np.random.default_rng(2027)
    cross = np.cross(triangles[:, 1] - triangles[:, 0],
                     triangles[:, 2] - triangles[:, 0])
    areas = np.linalg.norm(cross, axis=1) * .5
    # Discard only degenerate faces and wholly underground components.
    valid = (areas > 1e-12) & (triangles[:, :, 2].max(1) >= bottom_cutoff)
    triangles, areas = triangles[valid], areas[valid]
    chunks = []
    for start in range(0, len(triangles), 20000):
        t = triangles[start:start+20000]
        counts = np.maximum(1, np.ceil(areas[start:start+len(t)] / spacing**2).astype(int))
        ids = np.repeat(np.arange(len(t)), counts)
        uv = rng.random((len(ids), 2))
        reflected = uv.sum(1) > 1
        uv[reflected] = 1 - uv[reflected]
        points = (t[ids, 0] + uv[:, :1] * (t[ids, 1] - t[ids, 0])
                  + uv[:, 1:] * (t[ids, 2] - t[ids, 0]))
        chunks.append(points[points[:, 2] >= bottom_cutoff].astype(np.float32))
    return np.concatenate(chunks)


def load_mesh(cache):
    import trimesh
    from trimesh.ray.ray_pyembree import RayMeshIntersector
    triangles = np.load(cache)['triangles']
    mesh = trimesh.Trimesh(vertices=triangles.reshape(-1, 3),
                           faces=np.arange(len(triangles)*3).reshape(-1, 3), process=False)
    return mesh, RayMeshIntersector(mesh)


def viewpoints(mesh, rays):
    low, high = mesh.bounds
    x, y = np.meshgrid(np.arange(low[0]+.25, high[0], .75),
                       np.arange(low[1]+.25, high[1], .75))
    origins = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 1.)))
    hits, indices, faces = rays.intersects_location(
        origins, np.tile([0., 0., -1.], (len(origins), 1)), multiple_hits=False)
    valid = ((hits[:, 2] >= low[2]+.003) & (hits[:, 2] <= .65)
             & (np.abs(mesh.face_normals[faces, 2]) >= np.cos(np.deg2rad(30))))
    support = hits[valid]
    # Different sensor heights also retain structures above short tunnels.
    return np.concatenate([support + [0, 0, z] for z in (.25, .8)])


def visible_surface(points, cameras, rays):
    from scipy.spatial import cKDTree
    tree = cKDTree(cameras[:, :2])
    chunks = []
    for start in range(0, len(points), 8192):
        chunk = points[start:start+8192]
        _, neighbours = tree.query(chunk[:, :2], k=min(16, len(cameras)))
        visible = np.zeros(len(chunk), bool)
        for k in range(neighbours.shape[1]):
            ids = np.flatnonzero(~visible)
            if not len(ids):
                break
            origins = cameras[neighbours[ids, k]]
            delta = chunk[ids] - origins
            distance = np.linalg.norm(delta, axis=1)
            safe = distance > 1e-6
            ids, origins, delta, distance = ids[safe], origins[safe], delta[safe], distance[safe]
            hits, ray_ids, _ = rays.intersects_location(
                origins, delta / distance[:, None], multiple_hits=False)
            residual = np.abs(np.linalg.norm(hits-origins[ray_ids], axis=1)-distance[ray_ids])
            visible[ids[ray_ids[residual < .008]]] = True
        chunks.append(chunk[visible])
        if start % (8192*25) == 0:
            print(f'Visibility: {start}/{len(points)}', flush=True)
    return np.concatenate(chunks)


def generate(args):
    mesh, rays = load_mesh(args.cache)
    info = json.loads(args.cache.with_suffix('.json').read_text())
    print('Sampling CAD surfaces', flush=True)
    bottom_cutoff = float(mesh.bounds[0, 2]) + .003
    samples = sample_surface(mesh.triangles, args.spacing, bottom_cutoff)
    print('Surface samples:', len(samples), flush=True)
    # Select one original surface point per small voxel BEFORE visibility;
    # averaging before ray tests can move a sample off its original CAD face.
    _, ids = np.unique(np.floor(samples / (args.voxel*.5)).astype(np.int64),
                       axis=0, return_index=True)
    candidates = samples[ids]
    cameras = viewpoints(mesh, rays)
    if not len(cameras):
        raise RuntimeError('No above-ground viewpoints; check CAD up axis/ground offset')
    print('Visibility candidates / viewpoints:', len(candidates), len(cameras), flush=True)
    visible = visible_surface(candidates, cameras, rays)
    leaf = args.voxel
    while True:
        points = voxel_centroids(visible, leaf)
        if points.nbytes + 1024 < args.max_bytes:
            break
        leaf *= 1.1
    write_pcd(args.output, points)
    assert args.output.stat().st_size < args.max_bytes
    reread = read_pcd(args.output)
    assert np.isfinite(reread).all() and np.array_equal(points, reread)
    info.update(schema_version=1, frame_id='map', units='metres',
                source_type='official_CAD_surface_prior_not_sensor_mapping',
                source_article='https://bbs.robomaster.com/article/1970956?source=1',
                attribution='RoboMaster 高校赛务', license='CC BY-NC-SA 4.0',
                T_map_world=np.eye(4).tolist(),
                viewpoint_semantics='identity sensor pose; coordinates already in map',
                sample_spacing_m=args.spacing, random_seed=2027,
                surface_samples=len(samples), visibility_candidates=len(candidates),
                visibility_viewpoints=len(cameras), visible_samples=len(visible),
                visibility_tolerance_m=.008, voxel_size_m=leaf,
                underground_cutoff_m=bottom_cutoff,
                negative_ground_retained=True,
                pcd_points=len(points), pcd_bytes=args.output.stat().st_size,
                max_bytes=args.max_bytes, pcd_sha256=digest(args.output),
                pcd_bounds_m=[points.min(0).tolist(), points.max(0).tolist()],
                prior_pcd_transform=[0.]*6, transform_prior_map_with_lidar_offset=False)
    info['map_sha256'] = info['pcd_sha256']
    args.output.with_name(args.output.stem+'_manifest.json').write_text(json.dumps(info, indent=2)+'\n')
    print(json.dumps(info, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=Path,
                        default=Path.home()/'.cache/rmuc2027/RMUC2027_V1.0.0.stp')
    parser.add_argument('--cache', type=Path,
                        default=Path.home()/'.cache/rmuc2027/mesh.npz')
    parser.add_argument('--output', type=Path,
                        default=BRINGUP/'pcd/simulation/RMUC2027.pcd')
    parser.add_argument('--stage', choices=['mesh', 'pcd', 'all'], default='all')
    parser.add_argument('--spacing', type=float, default=.02)
    parser.add_argument('--voxel', type=float, default=.04)
    parser.add_argument('--max-bytes', type=int, default=30_000_000)
    args = parser.parse_args()
    if args.spacing <= 0 or args.voxel <= 0 or args.max_bytes <= 1024:
        parser.error('spacing/voxel must be positive; max-bytes must exceed 1024')
    started = time.monotonic()
    if args.stage in ('mesh', 'all'):
        make_mesh(args.step, args.cache)
    if args.stage in ('pcd', 'all'):
        generate(args)
    print(f'Elapsed: {time.monotonic()-started:.1f} seconds', flush=True)


if __name__ == '__main__':
    main()
