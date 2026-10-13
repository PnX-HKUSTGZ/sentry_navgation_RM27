#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
mkdir -p "$root/tools/global_relocalization/.build"
xacro "$root/src/rm_27_stimulation/urdf/simulation_waking_robot.xacro" > "$root/tools/global_relocalization/.build/robot_velocity.urdf"
/usr/bin/python3 - "$root" <<'PY'
from pathlib import Path
import hashlib,json,sys,xml.etree.ElementTree as ET
root=Path(sys.argv[1]);path=root/'tools/global_relocalization/.build/robot_velocity.urdf'
tree=ET.parse(path);robot=tree.getroot();removed=0
for gazebo in robot.findall('gazebo'):
 for plugin in list(gazebo.findall('plugin')):
  if plugin.get('name') in ('rm_27_stimulation::CmdVelPoseControlSystem','rm_27_stimulation::RelocalizationCmdVelControlSystem'):
   gazebo.remove(plugin);removed+=1
if removed!=1:
 raise RuntimeError(f'Expected one original drive controller, found {removed}')
gazebo=ET.SubElement(robot,'gazebo')
plugin=ET.SubElement(gazebo,'plugin',filename='gz-sim-velocity-control-system',name='gz::sim::systems::VelocityControl')
ET.SubElement(plugin,'topic').text='/cmd_vel'
tree.write(path,encoding='unicode',xml_declaration=True)
manifest={'reason':'The original and isolated wrench controllers received cmd_vel but produced no motion; Gazebo builtin VelocityControl verified actual link rotation in the same ground scene',
 'controller':'gz::sim::systems::VelocityControl','controller_library':'gz-sim-velocity-control-system',
 'actuation':'Gazebo model velocity command, including zero linear velocity constraints; independent simulation only',
 'command_topic':'/cmd_vel','original_wrench_controller_removed':True,
 'robot_description':str(path),'robot_description_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
 'source_xacro':str(root/'src/rm_27_stimulation/urdf/simulation_waking_robot.xacro'),
 'physics_backend':'gz-physics-bullet-featherstone-plugin'}
path.with_suffix('.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(path)
PY
