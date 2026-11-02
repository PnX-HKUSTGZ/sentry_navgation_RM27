#!/usr/bin/env python3
"""Blender 4.5: export CAD-coloured DAE assets and render the editable scene.

blender --background --factory-startup --python render_rmul2027_world.py -- --assets PATH
"""

import argparse
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector


def camera_at(name, position, target, scale):
    data = bpy.data.cameras.new(name)
    data.type = 'ORTHO'
    data.ortho_scale = scale
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.location = position
    obj.rotation_euler = (Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
    return obj


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    assets = args.assets.resolve()
    geometry = json.loads((assets/'geometry.json').read_text())
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1.0
    field = bpy.data.collections.new('RMUL2027 CAD / metres / Z up')
    scene.collection.children.link(field)
    for group in geometry['materials']:
        mesh = bpy.data.meshes.new(group['name'])
        mesh.from_pydata(group['vertices'], [], group['triangles'])
        mesh.update()
        obj = bpy.data.objects.new(group['name'],mesh)
        field.objects.link(obj)
        mat = bpy.data.materials.new('CAD '+group['name'])
        mat.use_nodes = True
        mat.diffuse_color = (*group['linear'],1)
        shader = mat.node_tree.nodes.get('Principled BSDF')
        shader.inputs['Base Color'].default_value = (*group['linear'],1)
        shader.inputs['Roughness'].default_value = .72
        shader.inputs['Metallic'].default_value = 0.0
        obj.data.materials.append(mat)
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        path = assets/'RMUL2027_visual/meshes'/f'{group["name"]}.dae'
        bpy.ops.wm.collada_export(filepath=str(path), selected=True, apply_modifiers=True)

    # Presentation-only floor and lights are not exported into the field model.
    bpy.ops.mesh.primitive_plane_add(size=200, location=(0,0,-.103))
    floor = bpy.context.object
    floor.name = 'Studio floor (render only)'
    mat = bpy.data.materials.new('Studio charcoal')
    mat.use_nodes = True
    mat.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value = (.035,.04,.045,1)
    mat.node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = .85
    floor.data.materials.append(mat)
    world = bpy.data.worlds.new('Neutral studio')
    world.use_nodes = True
    world.node_tree.nodes['Background'].inputs['Color'].default_value = (.65,.65,.65,1)
    world.node_tree.nodes['Background'].inputs['Strength'].default_value = .35
    scene.world = world
    for name, location, energy, size in [('Key',(-3,-5,10),1800,8),('Fill',(6,4,8),1100,7)]:
        light = bpy.data.lights.new(name,'AREA')
        light.energy, light.shape, light.size = energy, 'DISK', size
        obj = bpy.data.objects.new(name,light)
        bpy.context.collection.objects.link(obj)
        obj.location = location
        obj.rotation_euler = (-obj.location).to_track_quat('-Z','Y').to_euler()
    sun = bpy.data.lights.new('Soft daylight','SUN')
    sun.energy, sun.angle = 1.4, .15
    obj = bpy.data.objects.new('Soft daylight',sun)
    bpy.context.collection.objects.link(obj)
    obj.rotation_euler = (.4,-.3,-.4)
    perspective = camera_at('Overview',(10,-14,14),(0,0,.08),17.3)
    overhead = camera_at('Top',(0,0,16),(0,0,0),14.0)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 48
    scene.cycles.use_denoising = True
    scene.render.threads_mode = 'FIXED'
    scene.render.threads = 8
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 1100
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.view_settings.view_transform = 'Standard'
    scene.view_settings.look = 'None'
    scene.view_settings.exposure = -1.3
    renders = assets/'renders'
    renders.mkdir(exist_ok=True)
    for camera, name in [(perspective,'overview'),(overhead,'top')]:
        scene.camera = camera
        scene.render.filepath = str(renders/f'RMUL2027_{name}.png')
        bpy.ops.render.render(write_still=True)
    scene.camera = perspective
    (assets/'blender').mkdir(exist_ok=True)
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(assets/'blender/RMUL2027.blend'))


if __name__ == '__main__':
    main()
