"""Offline collision LOD: solid slabs, merged boxes and one shared prism mesh."""

from collections import Counter, defaultdict
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCP.BRepClass3d import BRepClass3d, BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN
from OCP.gp import gp_Pnt
from scipy.spatial.transform import Rotation
from shapely import set_precision
from shapely.geometry import LineString, Polygon, box
from shapely.ops import polygonize, triangulate, unary_union
import trimesh

from import_rmul2027_world import (BRepCheck_Analyzer, TopAbs_FACE, TopAbs_SHELL,
                                  TopAbs_SOLID, TopoDS, add_friction, convex_patches,
                                  face_mesh, numbers, shapes, static, xml_file)


DETAIL_TOLERANCE = .003
COORDINATE_DIGITS = 6


def polygons(geometry):
    if geometry.geom_type == 'Polygon':
        yield geometry
    elif hasattr(geometry, 'geoms'):
        for part in geometry.geoms:
            yield from polygons(part)


def right_triangles(triangle):
    """Split a triangle at the altitude to its longest edge; never bridge gaps."""
    triangle = np.asarray(triangle, float)
    lengths = np.linalg.norm(triangle - np.roll(triangle, -1, axis=0), axis=1)
    i = int(np.argmax(lengths))
    a, b, c = triangle[i], triangle[(i+1) % 3], triangle[(i+2) % 3]
    if lengths[i] < 1e-9:
        return
    foot = a + np.dot(c-a, b-a) / np.dot(b-a, b-a) * (b-a)
    for end in (a, b):
        if np.linalg.norm(end-foot) > 1e-8 and np.linalg.norm(c-foot) > 1e-8:
            yield np.array([foot, end, c])


def decompose_region(region):
    """Exact planar sweep into axis-aligned rectangles and right triangles."""
    rectangles, triangles = [], []
    for polygon in polygons(region):
        polygon = polygon.simplify(1e-7, preserve_topology=True)
        levels = np.unique(np.concatenate([
            np.asarray(ring.coords)[:, 1] for ring in [polygon.exterior, *polygon.interiors]]))
        for low, high in zip(levels[:-1], levels[1:]):
            if high-low < 1e-8:
                continue
            xmin, _, xmax, _ = polygon.bounds
            strip = polygon.intersection(box(xmin-1, low, xmax+1, high))
            for piece in polygons(strip):
                if piece.area < 1e-12:
                    continue
                piece = piece.simplify(1e-8, preserve_topology=True)
                x0, y0, x1, y1 = piece.bounds
                if abs((x1-x0)*(y1-y0)-piece.area) < 1e-10:
                    rectangles.append([x0, y0, x1, y1])
                    continue
                points = np.asarray(piece.exterior.coords)[:-1]
                bottom = points[np.abs(points[:, 1]-y0) < 1e-8, 0]
                top = points[np.abs(points[:, 1]-y1) < 1e-8, 0]
                left, right = max(bottom.min(), top.min()), min(bottom.max(), top.max())
                if left <= right and not piece.interiors:
                    if right-left > 1e-8:
                        rectangles.append([left, y0, right, y1])
                    for v0, v1, corner, lower in (
                            (bottom.min(), top.min(), left, bottom.min() < top.min()),
                            (bottom.max(), top.max(), right, bottom.max() > top.max())):
                        if abs(v0-v1) > 1e-8:
                            triangles.append(np.array([[corner, y0 if lower else y1],
                                                       [v0, y0], [v1, y1]]))
                else:
                    for part in triangulate(piece):
                        if piece.covers(part):
                            triangles.extend(right_triangles(np.asarray(part.exterior.coords)[:3]))
    covered = unary_union([box(*r) for r in rectangles] + [Polygon(t) for t in triangles])
    error = region.symmetric_difference(covered).area
    if error > max(1e-8, region.area*1e-8):
        raise RuntimeError(f'Collision decomposition lost boundary area: {error} m^2')
    return rectangles, triangles


def merge_boxes(boxes):
    boxes = [np.round(b, COORDINATE_DIGITS) for b in boxes]
    for _ in range(20):
        previous = len(boxes)
        for axis in (0, 1, 2):
            groups = defaultdict(list)
            for bounds in boxes:
                key = tuple(v for i, v in enumerate(bounds) if i not in (axis, axis+3))
                groups[key].append(bounds)
            boxes = []
            for group in groups.values():
                group.sort(key=lambda bounds: bounds[axis])
                current = group[0].copy()
                for bounds in group[1:]:
                    if bounds[axis] <= current[axis+3]+1e-7:
                        current[axis+3] = max(current[axis+3], bounds[axis+3])
                    else:
                        boxes.append(current)
                        current = bounds.copy()
                boxes.append(current)
        if len(boxes) == previous:
            break
    return boxes


def merge_prisms(prisms):
    groups = defaultdict(list)
    profiles = {}
    for triangle, low, high in prisms:
        key = tuple(sorted(map(tuple, np.round(triangle, COORDINATE_DIGITS))))
        groups[key].append((low, high))
        profiles.setdefault(key, triangle)
    result = []
    for triangle, intervals in sorted(groups.items()):
        intervals.sort()
        low, high = intervals[0]
        for a, b in intervals[1:]:
            if a <= high+1e-7:
                high = max(high, b)
            else:
                result.append((profiles[triangle], low, high))
                low, high = a, b
        result.append((profiles[triangle], low, high))
    return result


class CollisionWriter:
    def __init__(self, folder):
        self.folder = folder
        self.root = ET.Element('sdf', version='1.9')
        model = ET.SubElement(self.root, 'model', name='RMUL2027_collision')
        ET.SubElement(model, 'static').text = 'true'
        self.link = ET.SubElement(model, 'link', name='field')
        self.kinds = Counter()

    def collision(self, kind):
        element = ET.SubElement(self.link, 'collision', name=f'{kind}_{sum(self.kinds.values()):04d}')
        self.kinds[kind] += 1
        add_friction(element)
        return element, ET.SubElement(element, 'geometry')

    def box(self, center, dimensions, basis=np.eye(3)):
        if min(dimensions) < 1e-8:
            return
        element, geometry = self.collision('box')
        ET.SubElement(element, 'pose', rotation_format='quat_xyzw').text = numbers([
            *center, *Rotation.from_matrix(basis).as_quat()])
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = numbers(dimensions)

    def prism(self, triangle, height):
        # Locate the right angle after canonicalization used to merge layers.
        triangle = np.asarray(triangle, float)
        for i in range(3):
            a, b = triangle[(i+1) % 3]-triangle[i], triangle[(i+2) % 3]-triangle[i]
            if abs(np.dot(a, b)) <= max(1e-12, np.linalg.norm(a)*np.linalg.norm(b)*2e-5):
                break
        else:
            raise RuntimeError('Shared collision mesh requires a right triangle')
        la, lb = np.linalg.norm(a), np.linalg.norm(b)
        if min(la, lb, height) < 1e-8:
            return
        u, v = a/la, b/lb
        basis = np.column_stack((u, v, np.cross(u, v)))
        element, geometry = self.collision('right_prism')
        ET.SubElement(element, 'pose', rotation_format='quat_xyzw').text = numbers([
            *triangle[i], *Rotation.from_matrix(basis).as_quat()])
        mesh = ET.SubElement(geometry, 'mesh', optimization='convex_hull')
        decomposition = ET.SubElement(mesh, 'convex_decomposition')
        ET.SubElement(decomposition, 'max_convex_hulls').text = '1'
        ET.SubElement(decomposition, 'voxel_resolution').text = '200000'
        ET.SubElement(mesh, 'uri').text = 'model://RMUL2027_collision/meshes/unit_right_prism.stl'
        ET.SubElement(mesh, 'scale').text = numbers([la, lb, height])

    def skin(self, polygon, normal, thickness=.001):
        edge = polygon[1]-polygon[0]
        u = edge/np.linalg.norm(edge)
        v = np.cross(normal, u)
        basis = np.column_stack((u, v, normal))
        profile = Polygon(np.column_stack((polygon@u, polygon@v)))
        if abs(profile.minimum_rotated_rectangle.area-profile.area) < 1e-10:
            self.box(polygon.mean(axis=0)-normal*thickness/2,
                     [np.ptp(polygon@u), np.ptp(polygon@v), thickness], basis)
        else:
            for i in range(1, len(polygon)-1):
                for triangle in right_triangles(polygon[[0, i, i+1]]):
                    self.prism(triangle-normal*thickness/2, thickness)

    def save(self):
        (self.folder/'meshes').mkdir(parents=True, exist_ok=True)
        # This canonical convex primitive is reused at any position and scale.
        vertices = np.array([[0,0,-.5], [1,0,-.5], [0,1,-.5],
                             [0,0,.5], [1,0,.5], [0,1,.5]], float)
        trimesh.convex.convex_hull(vertices).export(self.folder/'meshes/unit_right_prism.stl')
        xml_file(self.root, self.folder/'model.sdf')


def generate_collision(roots, folder, detail_tolerance=DETAIL_TOLERANCE):
    solids = [s for root in roots for s in shapes(root, TopAbs_SOLID)]
    if len(solids) != 1:
        raise RuntimeError('RMUL2027 collision conversion expects its one CAD solid')
    solid_shells = list(shapes(solids[0], TopAbs_SHELL))
    outer = static(BRepClass3d, 'OuterShell', static(TopoDS, 'Solid', solids[0]))
    # Closed internal voids are unreachable from outside. Filling them permits
    # volume merging without changing any accessible external opening.
    filled = BRepBuilderAPI_MakeSolid(static(TopoDS, 'Shell', outer)).Solid()
    if not BRepCheck_Analyzer(filled).IsValid():
        raise RuntimeError('The external CAD shell is not a valid closed solid')
    classifier = BRepClass3d_SolidClassifier(filled)
    triangles, slopes = [], []
    for f in shapes(outer, TopAbs_FACE):
        vertices, faces = face_mesh(static(TopoDS, 'Face', f))
        triangles.extend(vertices[faces])
        patches, normal = convex_patches(vertices, faces)
        if 1e-6 < abs(normal[2]) < 1-1e-6:
            for patch in patches:
                low, high = patch[:, 2].min(), patch[:, 2].max()
                bottom = patch[np.abs(patch[:, 2]-low) < 1e-7]
                top = patch[np.abs(patch[:, 2]-high) < 1e-7]
                if normal[2] < 0 or len(bottom) != 2 or len(top) != 2:
                    raise RuntimeError('RMUL2027 slope must be a rectangular upper face')
                a, b = bottom.mean(axis=0), top.mean(axis=0)
                corner = np.array([b[0], b[1], a[2]])
                slopes.append((np.array([corner, a, b]), np.linalg.norm(bottom[1]-bottom[0]),
                               Polygon(patch[:, :2]), low, high))
    triangles = np.asarray(triangles)
    mesh = trimesh.Trimesh(vertices=triangles.reshape(-1, 3),
                           faces=np.arange(triangles.size//3).reshape(-1, 3), process=False)
    upper_levels = set(np.round(triangles[mesh.face_normals[:, 2] > .99, :, 2].mean(axis=1), 7))
    clusters = []
    for height in np.unique(np.round(triangles[:, :, 2], 7)):
        # Flatten relief only ABOVE a supporting surface. A thin mat with
        # air below it must keep its underside and load-bearing top.
        if (clusters and clusters[-1][0] in upper_levels and
                height-clusters[-1][0] <= detail_tolerance+1e-7):
            clusters[-1].append(float(height))
        else:
            clusters.append([float(height)])
    # The highest thin layer can be an overhanging cover, with no support
    # below its outer edge. Preserve it rather than dropping the whole cover.
    if len(clusters[-1]) > 1:
        top = clusters[-1].pop()
        clusters.append([top])
    boxes, prisms, layers = [], [], []
    for lower, upper in zip(clusters[:-1], clusters[1:]):
        low, high = lower[0], upper[0]
        sample_z = (lower[-1]+upper[0])/2
        segments = trimesh.intersections.mesh_plane(mesh, [0,0,1], [0,0,sample_z])
        lines = unary_union([set_precision(LineString(s[:, :2]), 1e-7) for s in segments])
        inside = []
        for polygon in polygonize(lines):
            p = polygon.representative_point()
            classifier.Perform(gp_Pnt(p.x*1000, -sample_z*1000, p.y*1000), 1e-5)
            if classifier.State() == TopAbs_IN:
                inside.append(polygon)
        region = unary_union(inside)
        # Sloping tops are exact right prisms, not steps sampled at slab
        # midpoints. Remove their swept region from slabs before adding them.
        cutouts = [p for _, _, p, a, b in slopes if a < high-1e-8 and b > low+1e-8]
        if cutouts:
            region = region.difference(unary_union(cutouts))
        rectangles, right = decompose_region(region)
        boxes.extend([[r[0], r[1], low, r[2], r[3], high] for r in rectangles])
        prisms.extend((t, low, high) for t in right)
        layers.append(dict(z_min=low, z_max=high, cad_sample_z=sample_z, area_m2=region.area))
    writer = CollisionWriter(folder)
    for bounds in merge_boxes(boxes):
        writer.box((bounds[:3]+bounds[3:])/2, bounds[3:]-bounds[:3])
    for triangle, low, high in merge_prisms(prisms):
        writer.prism(np.column_stack((triangle, np.full(3, (low+high)/2))), high-low)
    for triangle, width, _, _, _ in slopes:
        writer.prism(triangle, width)
    for root in roots:
        for shell in shapes(root, TopAbs_SHELL):
            if any(shell.IsSame(s) for s in solid_shells):
                continue
            for f in shapes(shell, TopAbs_FACE):
                vertices, faces = face_mesh(static(TopoDS, 'Face', f))
                patches, normal = convex_patches(vertices, faces)
                for patch in patches:
                    writer.skin(patch, normal)
    writer.save()
    return dict(collision_kinds=dict(writer.kinds), unique_collision_meshes=1,
                runtime_convex_voxel_resolution=200000, runtime_convex_unique_meshes=1,
                collision_strategy='merged_solid_slabs_and_shared_right_prism',
                collision_detail_tolerance_m=detail_tolerance,
                collision_coordinate_rounding_m=10**-COORDINATE_DIGITS,
                filled_enclosed_voids=len(solid_shells)-1, collision_layers=layers)


def publish_collision(roots, folder):
    """Keep the previous model loadable if CAD conversion fails."""
    folder.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.collision-build-', dir=folder.parent) as temporary:
        staging = Path(temporary)
        info = generate_collision(roots, staging)
        (folder/'meshes').mkdir(exist_ok=True)
        # Publish dependencies before atomically replacing their SDF reference.
        for mesh in (staging/'meshes').iterdir():
            mesh.replace(folder/'meshes'/mesh.name)
        (staging/'model.sdf').replace(folder/'model.sdf')
    for pattern in ('boundary_*.stl', 'hull_*.stl'):
        for obsolete in (folder/'meshes').glob(pattern):
            obsolete.unlink()
    return info
