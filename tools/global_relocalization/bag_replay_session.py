#!/usr/bin/env python3
"""Play a saved mapping bag in isolated Gazebo and RViz windows."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import secrets
import shlex
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from runtime import ROOT, owned_process, start_process, stop_process

HERE = Path(__file__).resolve().parent
STATE = HERE / '.build/bag_replay/session.json'
DEFAULT_CAPTURE = HERE / 'data/full_uc_20261026/run03'


def save(state):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    temporary = STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, indent=2) + '\n')
    temporary.replace(STATE)


def stop():
    if not STATE.exists():
        print('No saved bag-replay session.')
        return
    state = json.loads(STATE.read_text())
    for name in ('playback', 'observer', 'rviz', 'gui', 'simulation'):
        stop_process(state.get('processes', {}).get(name))
    state['stopped'] = True
    save(state)
    print('Stopped this bag-replay session.')


def playback_control(action):
    if not STATE.exists():
        raise SystemExit('No saved bag-replay session.')
    state = json.loads(STATE.read_text())
    if owned_process(state.get('processes', {}).get('playback')) is None:
        raise SystemExit('Bag playback is not running.')
    env = dict(os.environ, ROS_DOMAIN_ID=str(state['domain']), GZ_PARTITION=state['partition'])
    service, kind, request = {
        'pause': ('pause', 'Pause', '{}'),
        'resume': ('resume', 'Resume', '{}'),
        'restart': ('seek', 'Seek', '{time: {sec: 0, nanosec: 9000000}}'),
    }[action]
    # Read the start time from the bag metadata; no epoch assumption is needed.
    if action == 'restart':
        import yaml
        bag_info = yaml.safe_load((Path(state['capture']) / 'raw_bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
        stamp = bag_info['starting_time']['nanoseconds_since_epoch']
        request = f'{{time: {{sec: {stamp // 1000000000}, nanosec: {stamp % 1000000000}}}}}'
    subprocess.run(['ros2', 'service', 'call', '/rosbag2_player/' + service,
                    'rosbag2_interfaces/srv/' + kind, request], env=env, check=True, timeout=20)


def visual_robot(output):
    source = HERE / '.build/robot_velocity.urdf'
    if not source.exists():
        subprocess.run(['bash', str(HERE / 'prepare_simulation.bash')], check=True)
    tree = ET.parse(source)
    robot = tree.getroot()
    for child in list(robot):
        if child.tag == 'gazebo':
            robot.remove(child)
    for link in robot.findall('link'):
        for collision in list(link.findall('collision')):
            link.remove(collision)
    gazebo = ET.SubElement(robot, 'gazebo')
    ET.SubElement(gazebo, 'static').text = 'true'
    tree.write(output, encoding='unicode', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, default=DEFAULT_CAPTURE)
    parser.add_argument('--rate', type=float, default=1.)
    control = parser.add_mutually_exclusive_group()
    control.add_argument('--stop', action='store_true')
    control.add_argument('--pause', action='store_true')
    control.add_argument('--resume', action='store_true')
    control.add_argument('--restart', action='store_true')
    parser.add_argument('--render-mode', choices=('auto', 'gpu', 'cpu'), default='auto')
    args = parser.parse_args()
    if args.stop:
        stop()
        return
    for action in ('pause', 'resume', 'restart'):
        if getattr(args, action):
            playback_control(action)
            return
    if args.rate <= 0:
        parser.error('rate must be positive')
    capture = args.capture.resolve()
    for name in ('raw_bag/metadata.yaml', 'mapping_errors.json', 'manifest.json', 'route.json'):
        if not (capture / name).is_file():
            parser.error(f'missing {capture / name}')
    if STATE.exists() and any(owned_process(p) for p in json.loads(STATE.read_text()).get('processes', {}).values()):
        raise SystemExit('A bag replay session is already open; use --stop first.')
    logs = HERE / '.build/bag_replay' / (datetime.now().strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:6])
    logs.mkdir(parents=True)
    (logs / 'route.json').write_text((capture / 'route.json').read_text())
    gui_config = logs / 'gazebo_replay.config'
    gui_config.write_text((HERE / 'config/mapping_gazebo.config').read_text().replace(
        '0 -24 28 0 0.85 1.57079632679', '0 -18 18 0 0.85 1.57079632679'))
    visual_robot(logs / 'recorded_robot.urdf')
    domain = int(os.environ.get('ROS_DOMAIN_ID', secrets.randbelow(100) + 80))
    partition = 'rm27_bag_replay_' + uuid.uuid4().hex
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), GZ_PARTITION=partition)
    state = {'capture': str(capture), 'logs': str(logs), 'domain': domain, 'partition': partition,
             'rate': args.rate, 'processes': {}}
    (logs / 'environment.bash').write_text('source ' + shlex.quote(str(HERE / 'environment.bash')) + '\n' +
                                           f'export ROS_DOMAIN_ID={domain}\nexport GZ_PARTITION={partition}\n')
    save(state)
    def start(name, command, environment=env):
        state['processes'][name] = start_process(command, logs / (name + '.log'), environment)
        save(state)
    try:
        from rm27_rendering import select_rendering
        rendering = select_rendering(args.render_mode, gui=True, purpose='rviz', environ=env)
        start('simulation', ['ros2', 'launch', 'rm_27_stimulation', 'rm_simulation.launch.py',
                             'gui:=false', 'use_ros_gz_bridge:=false', 'render_mode:=' + args.render_mode])
        start('gui', [*rendering.prefix, 'gz', 'sim', '-g', '--gui-config', str(gui_config)], rendering.environment)
        start('rviz', [*rendering.prefix, 'ros2', 'run', 'rviz2', 'rviz2', '-d',
                       str(HERE / 'config/mapping_evaluation.rviz'), '--ros-args', '-r', '__node:=bag_mapping_view'], rendering.environment)
        spawn = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default', '-name', 'recorded_robot',
                                '-file', str(logs / 'recorded_robot.urdf'), '-x', '-11.7', '-y', '3.1', '-z', '0.08'],
                               env=env, capture_output=True, text=True, timeout=60)
        (logs / 'spawn.log').write_text(spawn.stdout + spawn.stderr)
        if spawn.returncode:
            raise RuntimeError('Failed to spawn recorded robot: ' + spawn.stderr[-1000:])
        start('observer', ['/usr/bin/python3', str(HERE / 'bag_replay_observer.py'), '--capture', str(capture), '--result', str(logs)])
        start('playback', ['ros2', 'bag', 'play', str(capture / 'raw_bag'), '--rate', str(args.rate), '--loop',
                           '--delay', '8', '--disable-keyboard-controls', '--topics', '/clock', '/ground_truth/odometry',
                           '/aft_mapped_to_init', '/cloud_registered', '/map', '/livox/lidar', '/livox/imu'])
        print(f'Bag replay started, looping at {args.rate:g}x.\nLogs and environment: {logs}\nStop: bash tools/global_relocalization/replay_mapping_bag.bash --stop')
    except (Exception, KeyboardInterrupt):
        stop()
        raise


if __name__ == '__main__':
    main()
