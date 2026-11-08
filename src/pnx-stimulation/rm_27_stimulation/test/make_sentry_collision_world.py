#!/usr/bin/env python3
"""Build a headless contact fixture from the actual spawn SDF and RMUC27 lintels."""

import argparse
import copy
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro

PACKAGE = Path(__file__).resolve().parents[1]


def make_world(output, cache_plugin):
    spec = importlib.util.spec_from_file_location('spawn_robot', PACKAGE / 'launch/spawn_robot.launch.py')
    spawn = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(spawn)
    urdf = xacro.process_file(str(PACKAGE / 'urdf/simulation_waking_robot.xacro')).toxml()
    robot = ET.fromstring(spawn._convex_collision_description(urdf)).find('model')
    # Keep the actual link/joint/inertial/collision representation. Contact probes
    # need neither rendering sensors nor actuation; isolate all transport topics.
    for parent in robot.iter():
        for child in list(parent):
            if child.tag in ('visual', 'sensor') or (
                    child.tag == 'plugin' and child.get('name') != 'rm_27_stimulation::CadCollisionCacheSystem'):
                parent.remove(child)
    for uri in robot.findall('.//collision/geometry/mesh/uri'):
        uri.text = str(PACKAGE / 'meshes/rm27_sentry_collision' / Path(uri.text).name)
    plugin = robot.find("plugin[@name='rm_27_stimulation::CadCollisionCacheSystem']")
    plugin.set('filename', str(cache_plugin))
    plugin.find('mesh_directory').text = str(PACKAGE / 'meshes/rm27_sentry_collision')

    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name='sentry_collision_regression')
    ET.SubElement(world, 'gravity').text = '0 0 0'
    for filename, name in [('gz-sim-physics-system', 'Physics'), ('gz-sim-contact-system', 'Contact')]:
        ET.SubElement(world, 'plugin', filename=filename, name='gz::sim::systems::' + name)
    physics = ET.SubElement(world, 'physics', name='fixed_step', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '.001'
    ET.SubElement(physics, 'real_time_factor').text = '0'
    field = ET.parse(PACKAGE / 'meshes/RM27_battlefield_collision/model.sdf')

    for index, scenario in enumerate(('air_gap', 'solid', 'roof_clear', 'roof_blocked')):
        offset = index * 2.
        car = copy.deepcopy(robot)
        car.set('name', scenario + '_robot')
        ET.SubElement(car, 'pose').text = f'{offset} 0 .0819 0 0 0'
        world.append(car)
        model = ET.SubElement(world, 'model', name=scenario + '_probe')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='probe')
        if scenario in ('air_gap', 'solid'):
            collision = ET.SubElement(link, 'collision', name='probe_collision')
            # Empty chassis/roll clearance vs a CAD wheel-cover surface.
            point = (.15, .10, .1619) if scenario == 'air_gap' else (.296, -.20, .1019)
            ET.SubElement(collision, 'pose').text = f'{point[0] + offset} {point[1]} {point[2]} 0 0 0'
            ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'box'), 'size').text = '.006 .006 .006'
        else:
            for source in field.findall('.//collision'):
                mesh = source.find('geometry/mesh')
                if mesh is None or Path(mesh.findtext('uri')).name not in ('collision_174.stl', 'collision_176.stl'):
                    continue
                collision = copy.deepcopy(source)
                pose = collision.find('pose')
                values = list(map(float, pose.text.split()))
                # Same world transform as RMUC2027_world.world, then center the
                # front lintel and its 797.5 mm opening on this test robot.
                values[0] += offset - .7313
                values[1] += 1.624344 + 4.31435
                if scenario == 'roof_blocked':
                    values[2] -= .020  # Negative control: visibly insufficient clearance.
                pose.text = ' '.join(map(str, values))
                collision.find('geometry/mesh/uri').text = str(
                    PACKAGE / 'meshes/RM27_battlefield_collision/meshes/collision' / Path(mesh.findtext('uri')).name)
                link.append(collision)
        for number, collision in enumerate(link.findall('collision')):
            name = f'{scenario}_collision{number}'
            collision.set('name', name)
            sensor = ET.SubElement(link, 'sensor', name=f'{scenario}_{number}', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'update_rate').text = '1000'
            ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = name
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cache-plugin', type=Path, required=True)
    args = parser.parse_args()
    make_world(args.output, args.cache_plugin)
