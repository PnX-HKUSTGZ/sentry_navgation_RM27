"""Build a flat-ground physics fixture from the actual CAD robot and spawner."""
import argparse
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--actuator-plugin', required=True)
    parser.add_argument('--cad-plugin', required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location(
        'spawn_robot', args.package / 'launch/spawn_robot.launch.py')
    spawn = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(spawn)
    # Support a first build before this package has an ament index entry.
    text = (args.package / 'urdf/simulation_waking_robot.xacro').read_text()
    document = xacro.parse(text.replace('$(find rm_27_stimulation)', str(args.package)))
    xacro.process_doc(document)
    model = ET.fromstring(spawn._convex_collision_description(document.toxml())).find('model')
    assert len(model.findall('.//collision/geometry/mesh')) == 9
    ET.SubElement(model, 'pose').text = '0 0 0.082 0 0 0'
    # Remove sensor/rendering work, retaining all masses, joints, collisions and
    # contact parameters, plus the same two gimbal position controllers.
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            link.remove(sensor)
    for plugin in list(model.findall('plugin')):
        name = plugin.get('name', '')
        if name == 'rm_27_stimulation::CmdVelPoseControlSystem':
            plugin.set('filename', args.actuator_plugin)
        elif name == 'rm_27_stimulation::CadCollisionCacheSystem':
            plugin.set('filename', args.cad_plugin)
        elif name != 'gz::sim::systems::JointPositionController':
            model.remove(plugin)
    world = ET.fromstring('''<world name="cad_actuation">
      <gravity>0 0 -9.81</gravity>
      <physics name="physics" type="ignored">
        <max_step_size>0.001</max_step_size><real_time_factor>0</real_time_factor>
      </physics>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <model name="ground"><static>true</static><link name="ground_link">
        <collision name="ground_collision"><geometry><plane>
          <normal>0 0 1</normal><size>100 100</size>
        </plane></geometry></collision>
      </link></model>
    </world>''')
    world.append(model)
    root = ET.Element('sdf', version='1.9')
    root.append(world)
    ET.ElementTree(root).write(args.output, encoding='unicode')


if __name__ == '__main__':
    main()
