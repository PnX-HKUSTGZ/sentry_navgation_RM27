#!/usr/bin/env python3
"""Import the official RMUL2027 STEP with its colours and boundary topology.

Run with the dedicated CAD Python environment documented in the package README.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Plane
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.collections import Sequence_TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf, XCAFDoc_ColorGen


PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]


def static(owner, name, *args):
    method = getattr(owner, name + '_s', None) or getattr(owner, name)
    return method(*args)


def shapes(shape, kind):
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


def bounds(shape):
    box = Bnd_Box()
    static(BRepBndLib, 'Add', shape, box)
    low, high = box.CornerMin(), box.CornerMax()
    return [low.X(), low.Y(), low.Z(), high.X(), high.Y(), high.Z()]


def read_step(path):
    doc = TDocStd_Document(TCollection_ExtendedString('RMUL2027'))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise RuntimeError('STEP import failed')
    shape_tool = static(XCAFDoc_DocumentTool, 'ShapeTool', doc.Main())
    color_tool = static(XCAFDoc_DocumentTool, 'ColorTool', doc.Main())
    labels = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(labels)
    roots = [static(shape_tool, 'GetShape', labels.Value(i))
             for i in range(1, labels.Length()+1)]
    return doc, roots, color_tool


def face_color(tool, face, srgb=False):
    color = Quantity_Color()
    for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
        if tool.GetColor(face, kind, color):
            values = color.Values(Quantity_TOC_sRGB) if srgb else (color.Red(), color.Green(), color.Blue())
            return tuple(round(v, 6) for v in values)
    return None


def inspect(roots, colors):
    report = []
    for root in roots:
        palette = Counter(face_color(colors, face) for face in shapes(root, TopAbs_FACE))
        report.append(dict(type=str(root.ShapeType()), valid=BRepCheck_Analyzer(root).IsValid(),
                           bounds_mm=bounds(root),
                           faces=sum(palette.values()), solids=len(list(shapes(root, TopAbs_SOLID))),
                           shells=len(list(shapes(root, TopAbs_SHELL))),
                           palette={str(k): v for k,v in palette.items()}))
    return report


def face_mesh(face):
    location = TopLoc_Location()
    mesh = static(BRep_Tool, 'Triangulation', face, location)
    if mesh is None:
        raise RuntimeError('A CAD face could not be triangulated')
    vertices = []
    for i in range(1, mesh.NbNodes()+1):
        p = mesh.Node(i).Transformed(location.Transformation())
        # CAD -Y is up. This is a proper rotation, with no mirrored winding.
        vertices.append([p.X()*.001, p.Z()*.001, -p.Y()*.001])
    triangles = np.array([mesh.Triangle(i).Get() for i in range(1, mesh.NbTriangles()+1)])-1
    if face.Orientation() == TopAbs_REVERSED:
        triangles = triangles[:, ::-1]
    return np.asarray(vertices), triangles


def convex_patches(vertices, triangles):
    """Merge neighbouring triangles only when their union is exactly convex.

    Checking area prevents a convex hull from bridging any notch or CAD hole.
    This operates on one planar CAD face, not a hull of the entire field.
    """
    vertices, inverse = np.unique(np.round(vertices, 9), axis=0, return_inverse=True)
    triangles = inverse[triangles]
    cross = np.cross(vertices[triangles[:, 1]]-vertices[triangles[:, 0]],
                     vertices[triangles[:, 2]]-vertices[triangles[:, 0]])
    lengths = np.linalg.norm(cross, axis=1)
    triangles, cross, lengths = triangles[lengths > 1e-14], cross[lengths > 1e-14], lengths[lengths > 1e-14]
    if not len(triangles):
        return [], None
    normal = cross[np.argmax(lengths)]/max(lengths)
    first = vertices[triangles[0, 1]]-vertices[triangles[0, 0]]
    u = first/np.linalg.norm(first)
    v = np.cross(normal, u)
    points2 = np.column_stack((vertices @ u, vertices @ v))
    groups = {i: set(t) for i,t in enumerate(triangles)}
    areas = {i: .5*lengths[i] for i in groups}
    parent = list(range(len(triangles)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    edges = {}
    neighbours = []
    for i,t in enumerate(triangles):
        for a,b in zip(t, np.roll(t, -1)):
            edge = tuple(sorted((int(a), int(b))))
            if edge in edges:
                neighbours.append((np.linalg.norm(vertices[a]-vertices[b]), edges[edge], i))
            else:
                edges[edge] = i
    neighbours.sort(reverse=True)
    for _ in range(20):
        changed = False
        for _, a, b in neighbours:
            a,b = root(a),root(b)
            if a == b:
                continue
            union = sorted(groups[a] | groups[b])
            hull = ConvexHull(points2[union])
            if abs(hull.volume-areas[a]-areas[b]) <= max(1e-10, (areas[a]+areas[b])*1e-8):
                parent[b] = a
                groups[a] |= groups.pop(b)
                areas[a] += areas.pop(b)
                changed = True
        if not changed:
            break
    patches = []
    for group in groups.values():
        indices = sorted(group)
        hull = ConvexHull(points2[indices])
        patches.append(vertices[np.asarray(indices)[hull.vertices]])
    return patches, normal


def xml_file(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def numbers(values):
    return ' '.join(f'{float(x):.10g}' for x in values)


def model_config(path, name, description):
    root = ET.Element('model')
    ET.SubElement(root,'name').text = name
    ET.SubElement(root,'version').text = '1.0'
    ET.SubElement(root,'sdf', version='1.9').text = 'model.sdf'
    ET.SubElement(root,'description').text = description
    xml_file(root, path/'model.config')


def add_friction(collision):
    surface = ET.SubElement(collision,'surface')
    friction = ET.SubElement(surface,'friction')
    ode = ET.SubElement(friction,'ode')
    ET.SubElement(ode,'mu').text = '0.8'
    ET.SubElement(ode,'mu2').text = '0.8'
    bullet = ET.SubElement(friction,'bullet')
    ET.SubElement(bullet,'friction').text = '0.8'
    ET.SubElement(bullet,'friction2').text = '0.8'
    ET.SubElement(bullet,'rolling_friction').text = '0'


def make_world(output):
    root = ET.Element('sdf',version='1.9')
    world = ET.SubElement(root,'world',name='RMUL2027')
    for short, name in [('physics','Physics'), ('user-commands','UserCommands'),
                        ('scene-broadcaster','SceneBroadcaster'), ('sensors','Sensors'), ('imu','Imu')]:
        plugin = ET.SubElement(world,'plugin', filename=f'gz-sim-{short}-system',name=f'gz::sim::systems::{name}')
        if short == 'sensors':
            ET.SubElement(plugin,'render_engine').text = 'ogre2'
    physics = ET.SubElement(world,'physics',name='bullet',default='1',type='bullet')
    ET.SubElement(physics,'max_step_size').text = '.001'
    ET.SubElement(physics,'real_time_factor').text = '1'
    ET.SubElement(physics,'real_time_update_rate').text = '1000'
    scene = ET.SubElement(world,'scene')
    ET.SubElement(scene,'ambient').text = '.35 .35 .35 1'
    ET.SubElement(scene,'background').text = '.12 .14 .16 1'
    ET.SubElement(scene,'shadows').text = 'true'
    ET.SubElement(scene,'grid').text = 'false'
    light = ET.SubElement(world,'light',name='sun',type='directional')
    ET.SubElement(light,'pose').text = '0 0 12 0 0 0'
    ET.SubElement(light,'diffuse').text = '.65 .65 .65 1'
    ET.SubElement(light,'specular').text = '.15 .15 .15 1'
    ET.SubElement(light,'direction').text = '-.4 .2 -1'
    ET.SubElement(light,'cast_shadows').text = 'true'
    for suffix in ('visual','collision'):
        include = ET.SubElement(world,'include')
        ET.SubElement(include,'uri').text = f'model://RMUL2027_{suffix}'
        ET.SubElement(include,'name').text = f'field_{suffix}'
        ET.SubElement(include,'pose').text = '0 0 0 0 0 0'
    ET.SubElement(world,'gravity').text = '0 0 -9.81'
    xml_file(root, PACKAGE/'world/RMUL2027_world/RMUL2027.world')
    source = (PACKAGE/'config/gazebo_gui.config').read_text()
    layout = ET.fromstring('<layout>'+source.replace('<?xml version="1.0"?>','',1)+'</layout>')
    scene_plugin = layout.find("plugin[@filename='MinimalScene']")
    scene_plugin.find('ambient_light').text = '.35 .35 .35'
    scene_plugin.find('background_color').text = '.12 .14 .16'
    scene_plugin.find('camera_pose').text = '6.3 -7.7 7 0 0.613 2.257'
    ET.SubElement(scene_plugin,'grid').text = 'false'
    (PACKAGE/'config/rmul2027_gui.config').write_text(
        '<?xml version="1.0"?>\n'+''.join(ET.tostring(child,encoding='unicode') for child in layout)+'\n')


def generate(step_path, roots, colors, output):
    visual = output/'RMUL2027_visual'
    collision = output/'RMUL2027_collision'
    for folder in (visual,collision):
        (folder/'meshes').mkdir(parents=True,exist_ok=True)
    source_info = inspect(roots, colors)
    if not all(r['valid'] for r in source_info):
        raise RuntimeError('Invalid STEP topology; repair it before producing collisions')
    groups, manifest = {}, []
    face_index = 0
    spawn_support = []
    for root in roots:
        BRepMesh_IncrementalMesh(root, .5, False, .2, True)
        for shape in shapes(root, TopAbs_FACE):
            face = static(TopoDS,'Face',shape)
            if BRepAdaptor_Surface(face).GetType() != GeomAbs_Plane:
                raise RuntimeError('This importer requires planar RMUL2027 faces')
            vertices, triangles = face_mesh(face)
            rgb = face_color(colors,face,True) or (.5,.5,.5)
            linear = face_color(colors,face) or (.214041,.214041,.214041)
            key = ','.join(map(str,rgb))
            group = groups.setdefault(key,dict(srgb=rgb,linear=linear,vertices=[],triangles=[]))
            offset = len(group['vertices'])
            group['vertices'].extend(vertices.tolist())
            group['triangles'].extend((triangles+offset).tolist())
            patches, normal = convex_patches(vertices,triangles)
            for polygon in patches:
                if normal[2] > .9:
                    from shapely.geometry import Point, Polygon
                    if Polygon(polygon[:,:2]).covers(Point(0,0)):
                        spawn_support.append(float(np.dot(normal,polygon[0])/normal[2]))
            manifest.append(dict(face=face_index,colour=rgb,triangles=len(triangles)))
            face_index += 1
    from rmul2027_collision import publish_collision
    collision_info = publish_collision(roots, collision)
    palette = list(groups.values())
    visual_xml = ET.Element('sdf',version='1.9')
    model = ET.SubElement(visual_xml,'model',name='RMUL2027_visual')
    ET.SubElement(model,'static').text = 'true'
    link = ET.SubElement(model,'link',name='field')
    for i,group in enumerate(palette):
        group['name'] = f'material_{i:02d}'
        element = ET.SubElement(link,'visual',name=group['name'])
        mesh = ET.SubElement(ET.SubElement(element,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text = f'model://RMUL2027_visual/meshes/{group["name"]}.dae'
        material = ET.SubElement(element,'material')
        for field in ('ambient','diffuse'):
            ET.SubElement(material,field).text = numbers([*group['srgb'],1])
        ET.SubElement(material,'specular').text = '.08 .08 .08 1'
        ET.SubElement(material,'double_sided').text = 'true'
        metal = ET.SubElement(ET.SubElement(material,'pbr'),'metal')
        ET.SubElement(metal,'roughness').text = '.72'
        ET.SubElement(metal,'metalness').text = '0'
        ET.SubElement(element,'cast_shadows').text = 'true'
    xml_file(visual_xml,visual/'model.sdf')
    model_config(visual,'RMUL2027 Visual','Official 3V3 V1.0.0 STEP colours; static visuals only.')
    model_config(collision,'RMUL2027 Collision',
                 'Merged CAD solid layers; 3 mm detail LOD; boxes and one shared convex prism.')
    (output/'geometry.json').write_text(json.dumps(dict(materials=palette),separators=(',',':'))+'\n')
    metadata = dict(source=step_path.name,sha256=hashlib.sha256(step_path.read_bytes()).hexdigest(),
                    coordinate_mapping='ros_x=cad_x/1000, ros_y=cad_z/1000, ros_z=-cad_y/1000',
                    source_structure=source_info,face_count=face_index,
                    visual_triangles=sum(len(g['triangles']) for g in palette),
                    visual_materials=len(palette),**collision_info,
                    collision_skin_m=.001,
                    origin_support_height_m=max(spawn_support) if spawn_support else None,
                    faces=manifest)
    (output/'import_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    make_world(output)
    print(json.dumps({k:v for k,v in metadata.items() if k not in ('faces','source_structure')},indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', type=Path, nargs='?', default=REPOSITORY/'RMUL2027_3V3_V1.0.0.stp')
    parser.add_argument('--inspect-only',action='store_true')
    parser.add_argument('--output',type=Path,default=PACKAGE/'meshes/RMUL2027')
    args = parser.parse_args()
    document, roots, colors = read_step(args.step)
    if args.inspect_only:
        print(json.dumps(inspect(roots, colors), indent=2))
    else:
        generate(args.step,roots,colors,args.output)


if __name__ == '__main__':
    main()
