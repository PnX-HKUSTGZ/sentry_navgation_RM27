#!/usr/bin/env python3
"""Independently compare the CAD PCD with the existing RM27 world and map."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import trimesh
import yaml

from generate_rmuc2027_pcd import BRINGUP, ROOT, digest, read_pcd, voxel_centroids


def pose_matrix(text):
    values = np.fromstring(text or '0 0 0 0 0 0', sep=' ')
    result = np.eye(4)
    result[:3, 3] = values[:3]
    result[:3, :3] = (Rotation.from_quat(values[3:]).as_matrix() if len(values) == 7
                       else Rotation.from_euler('xyz', values[3:]).as_matrix())
    return result


def world_triangles():
    """Read the existing, independently imported Gazebo visual geometry."""
    package = ROOT/'src/pnx-stimulation/rm_27_stimulation'
    world_path = package/'world/RMUC2027_world/RMUC2027_world.world'
    model_path = package/'meshes/RM27_battlefield_visual/model.sdf'
    world = ET.parse(world_path)
    instance = next(x for x in world.findall('.//include')
                    if x.findtext('uri') == 'model://RM27_battlefield_visual')
    world_pose = pose_matrix(instance.findtext('pose'))
    model = ET.parse(model_path).find('model')
    model_pose = pose_matrix(model.findtext('pose'))
    triangles = []
    for link in model.findall('link'):
        link_pose = pose_matrix(link.findtext('pose'))
        for visual in link.findall('visual'):
            transform = world_pose @ model_pose @ link_pose @ pose_matrix(visual.findtext('pose'))
            geometry = visual.find('geometry/mesh')
            if geometry is None:
                box = visual.find('geometry/box/size')
                cylinder = visual.find('geometry/cylinder')
                if box is not None:
                    primitive_mesh = trimesh.creation.box(extents=np.fromstring(box.text, sep=' '))
                elif cylinder is not None:
                    primitive_mesh = trimesh.creation.cylinder(radius=float(cylinder.findtext('radius')),
                        height=float(cylinder.findtext('length')), sections=64)
                else:
                    raise ValueError(f'Unsupported SDF geometry: {visual.get("name")}')
                primitive_mesh.apply_transform(transform)
                triangles.append(primitive_mesh.triangles)
                continue
            uri = geometry.findtext('uri')
            path = package/'meshes'/uri.removeprefix('model://')
            root = ET.parse(path).getroot()
            ns = {'c': root.tag.split('}')[0][1:]}
            # The reviewed exports contain no additional scene transforms.
            for node in root.findall('.//c:visual_scene//c:node', ns):
                if any(node.find('c:'+tag, ns) is not None
                       for tag in ('matrix', 'rotate', 'scale', 'translate')):
                    raise ValueError(f'Unsupported Collada node placement: {path}')
            scale = np.fromstring(geometry.findtext('scale') or '1 1 1', sep=' ')
            for mesh in root.findall('.//c:mesh', ns):
                sources = {x.get('id'): np.fromstring(x.findtext('c:float_array', namespaces=ns), sep=' ').reshape(-1, 3)
                           for x in mesh.findall('c:source', ns)}
                vertices = {x.get('id'): x.find('c:input', ns).get('source')[1:]
                            for x in mesh.findall('c:vertices', ns)}
                for primitive in mesh.findall('c:triangles', ns):
                    inputs = primitive.findall('c:input', ns)
                    position = next(x for x in inputs if x.get('semantic') == 'VERTEX')
                    stride = max(int(x.get('offset', '0')) for x in inputs) + 1
                    indices = np.fromstring(primitive.findtext('c:p', namespaces=ns), sep=' ', dtype=int)
                    indices = indices.reshape(-1, stride)[:, int(position.get('offset', '0'))].reshape(-1, 3)
                    xyz = sources[vertices[position.get('source')[1:]]] * scale
                    xyz = xyz @ transform[:3, :3].T + transform[:3, 3]
                    triangles.append(xyz[indices])
    return np.concatenate(triangles), dict(world_path=str(world_path.relative_to(ROOT)),
        model_path=str(model_path.relative_to(ROOT)),
        world_sha256=digest(world_path), model_sha256=digest(model_path))


def summary(distances):
    return dict(points=len(distances), median_m=float(np.median(distances)),
                p90_m=float(np.quantile(distances, .90)),
                p95_m=float(np.quantile(distances, .95)),
                within_5cm=float(np.mean(distances <= .05)),
                within_10cm=float(np.mean(distances <= .10)))


def rigid_check(source, reference):
    """Estimate a diagnostic correction; never alter the delivered PCD."""
    source = voxel_centroids(source[source[:, 2] > .10], .08).astype(float)
    reference = voxel_centroids(reference[reference[:, 2] > .10], .06).astype(float)
    tree = cKDTree(reference)
    before, _ = tree.query(source, workers=4)
    transform = np.eye(4)
    moved = source.copy()
    for _ in range(30):
        distances, ids = tree.query(moved, workers=4)
        good = distances < .12
        x, y = moved[good], reference[ids[good]]
        if len(x) < 100:
            raise ValueError('Insufficient reference overlap')
        cx, cy = x.mean(0), y.mean(0)
        u, _, vt = np.linalg.svd((x-cx).T @ (y-cy))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        translation = cy - rotation @ cx
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = rotation, translation
        transform = delta @ transform
        moved = moved @ rotation.T + translation
        if np.linalg.norm(translation) < 1e-6 and Rotation.from_matrix(rotation).magnitude() < 1e-6:
            break
    after, _ = tree.query(moved, workers=4)
    return dict(reference='existing RMUC2026.pcd, already in map coordinates',
                elevated_geometry_only=True, max_correspondence_m=.12,
                before=summary(before), after=summary(after),
                diagnostic_correction_matrix=transform.tolist(),
                diagnostic_translation_m=transform[:3, 3].tolist(),
                diagnostic_rotation_xyz_deg=Rotation.from_matrix(transform[:3, :3]).as_euler('xyz', degrees=True).tolist(),
                correction_applied=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, default=BRINGUP/'pcd/simulation/RMUC2027.pcd')
    parser.add_argument('--cache', type=Path, default=Path.home()/'.cache/rmuc2027/mesh.npz')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'docs/validation/rmuc2027')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    points = read_pcd(args.pcd)
    metadata = json.loads(args.pcd.with_name(args.pcd.stem+'_manifest.json').read_text())
    config_path = BRINGUP/'map/simulation/RMUC2027.yaml'
    config = yaml.safe_load(config_path.read_text())
    image_path = config_path.parent/config['image']
    pixels = np.asarray(Image.open(image_path))
    resolution = config['resolution']
    ox, oy, yaw = config['origin']
    if yaw != 0:
        raise ValueError('Expected current axis-aligned navigation map')
    height, width = pixels.shape
    extent = [ox, ox+width*resolution, oy, oy+height*resolution]
    reference_cloud = read_pcd(BRINGUP/'pcd/simulation/RMUC2026.pcd')
    print('Comparing independent existing world geometry', flush=True)
    triangles, provenance = world_triangles()
    reference_mesh = trimesh.Trimesh(vertices=triangles.reshape(-1, 3),
        faces=np.arange(len(triangles)*3).reshape(-1, 3), process=False)
    from trimesh.ray.ray_pyembree import RayMeshIntersector
    intersector = RayMeshIntersector(reference_mesh)
    cad_triangles = np.load(args.cache)['triangles']
    cad_mesh = trimesh.Trimesh(vertices=cad_triangles.reshape(-1, 3),
        faces=np.arange(len(cad_triangles)*3).reshape(-1, 3), process=False)
    cad_intersector = RayMeshIntersector(cad_mesh)
    terrain_xy = np.array([[-12., 0.], [-8., 0.], [-8., 5.], [0., 0.],
                           [0., 6.], [8., 3.], [8., -2.], [12., 3.]])
    terrain_origins = np.column_stack((terrain_xy, np.ones(len(terrain_xy))))
    supports = []
    for name, caster, geometry in [('official_CAD', cad_intersector, cad_mesh),
                                    ('existing_world', intersector, reference_mesh)]:
        hits, indices, face_ids = caster.intersects_location(terrain_origins,
            np.tile([0., 0., -1.], (len(terrain_xy), 1)), multiple_hits=False)
        lookup = {}
        for hit, i, face in zip(hits, indices, face_ids):
            normal = geometry.face_normals[face]
            lookup[int(i)] = dict(z_m=float(hit[2]),
                slope_deg=float(np.degrees(np.arccos(np.clip(abs(normal[2]), 0, 1)))))
        supports.append((name, lookup))
    terrain_comparison = [dict(xy_m=xy.tolist(), **{name: lookup.get(i)
                            for name, lookup in supports}) for i, xy in enumerate(terrain_xy)]
    rng = np.random.default_rng(27)
    ids = rng.choice(len(points), min(100000, len(points)), replace=False)
    probes = points[ids]
    # Ray probes from six directions measure distance to independent old mesh.
    # Report coverage as well as residual: unmatched probes cannot disappear
    # from the alignment conclusion.
    residuals = np.full(len(probes), np.inf)
    for direction in np.vstack((np.eye(3), -np.eye(3))):
        origins = probes - direction*.15
        hits, ray_ids, _ = intersector.intersects_location(origins,
            np.tile(direction, (len(probes), 1)), multiple_hits=False)
        residuals[ray_ids] = np.minimum(residuals[ray_ids],
            np.linalg.norm(hits-probes[ray_ids], axis=1))
    finite = np.isfinite(residuals)
    mesh_comparison = summary(residuals)
    mesh_comparison['finite_ray_probe_fraction'] = float(finite.mean())
    print('Independent mesh residual:', mesh_comparison, flush=True)
    icp = rigid_check(points, reference_cloud)
    print('Diagnostic rigid correction:', icp['diagnostic_translation_m'], icp['diagnostic_rotation_xyz_deg'], flush=True)
    col = np.floor((points[:, 0]-ox)/resolution).astype(int)
    row = height-1-np.floor((points[:, 1]-oy)/resolution).astype(int)
    inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    occupied = (255-pixels.astype(float))/255 > config['occupied_thresh']
    distance = distance_transform_edt(~occupied)*resolution
    band = inside & (points[:, 2] > .10) & (points[:, 2] < .20)
    grid_distances = distance[row[band], col[band]]
    checks = dict(binary_payload_valid=True,
        manifest_hash_matches=digest(args.pcd) == metadata['map_sha256'],
        manifest_points_match=len(points) == metadata['pcd_points'],
        manifest_frame_is_map=metadata['frame_id'] == 'map',
        manifest_world_transform_is_identity=bool(np.allclose(metadata['T_map_world'], np.eye(4))),
        finite_xyz=bool(np.isfinite(points).all()),
        file_below_30MB=args.pcd.stat().st_size < 30_000_000,
        below_cad_bottom_removed=float(points[:, 2].min()) >= metadata['underground_cutoff_m']-1e-6,
        negative_ground_retained=bool(np.any(points[:, 2] < -.05)),
        mesh_within_5cm_fraction=float(np.mean(residuals <= .05)),
        rigid_translation_below_3cm=bool(np.linalg.norm(icp['diagnostic_translation_m']) < .03),
        rigid_rotation_below_point2deg=bool(np.linalg.norm(icp['diagnostic_rotation_xyz_deg']) < .2))
    checks['passed'] = bool(all([checks['binary_payload_valid'], checks['manifest_hash_matches'],
        checks['manifest_points_match'], checks['manifest_frame_is_map'],
        checks['manifest_world_transform_is_identity'], checks['finite_xyz'],
        checks['file_below_30MB'], checks['below_cad_bottom_removed'], checks['negative_ground_retained'],
        checks['mesh_within_5cm_fraction'] > .95,
        checks['rigid_translation_below_3cm'], checks['rigid_rotation_below_point2deg']]))
    report = dict(checks=checks, pcd_file=str(args.pcd.relative_to(ROOT)),
        pcd_sha256=digest(args.pcd), pcd_points=len(points), pcd_bytes=args.pcd.stat().st_size,
        voxel_size_m=metadata['voxel_size_m'], frame_id='map',
        pcd_bounds_m=[points.min(0).tolist(), points.max(0).tolist()],
        reference_world=provenance, independent_world_mesh_comparison=mesh_comparison,
        terrain_support_at_same_map_xy=terrain_comparison,
        existing_map_pcd_rigid_check=icp,
        occupancy_map=dict(path=str(config_path.relative_to(ROOT)), image_sha256=digest(image_path),
            yaml_sha256=digest(config_path), resolution_m=resolution, origin=config['origin'],
            dimensions=[width, height], cloud_inside_image_fraction=float(inside.mean()),
            height_band_m=[.10, .20], height_band_distance_to_occupied=summary(grid_distances),
            note='Raster distances are descriptive: ramps, platforms and outside-field CAD can differ from the 2D navigation mask.'),
        runtime_validation='offline only; no live ROS TF or localization run performed')
    (args.output_dir/'alignment.json').write_text(json.dumps(report, indent=2)+'\n')
    fig, axes = plt.subplots(2, 1, figsize=(16, 15), constrained_layout=True)
    axes[0].imshow(pixels, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
    shown = points[(points[:, 2] > .10) & (points[:, 2] < .70)][::3]
    axes[0].scatter(shown[:, 0], shown[:, 1], s=.12, c='#e53935', alpha=.55,
                    label='Official RMUC2027 CAD PCD, z = 0.10..0.70 m')
    axes[0].set_title('RMUC2027 PCD over existing RMUC2026 / RM27 navigation map')
    axes[0].legend(loc='upper left', markerscale=10)
    axes[0].set_xlim(extent[:2]); axes[0].set_ylim(extent[2:])
    axes[1].scatter(reference_cloud[::8, 0], reference_cloud[::8, 1], s=.2, c='#1565c0', alpha=.35,
                    label='Existing map-frame PCD')
    axes[1].scatter(shown[:, 0], shown[:, 1], s=.15, c='#e53935', alpha=.5,
                    label='Official RMUC2027 CAD PCD')
    axes[1].set_title('Independent point cloud overlay; no diagnostic ICP correction applied')
    axes[1].legend(loc='upper left', markerscale=10)
    for ax in axes:
        ax.set_aspect('equal'); ax.set_xlabel('map X [m]'); ax.set_ylabel('map Y [m]')
        ax.grid(alpha=.15)
    fig.savefig(args.output_dir/'map_alignment.png', dpi=160)
    plt.close(fig)
    fig = plt.figure(figsize=(15, 8))
    ax = fig.add_subplot(projection='3d')
    shown = points[::max(1, len(points)//140000)]
    ax.scatter(*shown.T, c=shown[:, 2], cmap='viridis', s=.1, vmin=0, vmax=3)
    ax.set(xlabel='map X [m]', ylabel='map Y [m]', zlabel='map Z [m]',
           title='RMUC2027 official CAD surface prior, metres, map frame')
    ax.set_box_aspect(np.ptp(points, axis=0)); ax.view_init(elev=55, azim=-65)
    fig.savefig(args.output_dir/'pointcloud.png', dpi=160, bbox_inches='tight'); plt.close(fig)
    print(json.dumps(report, indent=2))
    if not checks['passed']:
        raise SystemExit('Alignment checks failed; inspect report before using this PCD')


if __name__ == '__main__':
    main()
