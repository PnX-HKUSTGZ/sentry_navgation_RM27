#!/usr/bin/env python3
"""Read live mapping executables, loaded libraries, and simulator description provenance.

No messages or control commands are published. Only parameter read services are
called. ROS_DOMAIN_ID filters /proc evidence as well as ROS parameter discovery.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import time
import xml.etree.ElementTree as ET

import psutil
import yaml


class FileHashes:
    def __init__(self):
        self.cache = {}

    def record(self, filename):
        path = Path(filename)
        try:
            before = path.stat()
            key = (str(path.resolve()), before.st_ino, before.st_size, before.st_mtime_ns)
            if key not in self.cache:
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(4*1024*1024), b''):
                        digest.update(chunk)
                after = path.stat()
                self.cache[key] = {'path': str(path.resolve()), 'sha256': digest.hexdigest(),
                    'size_bytes': before.st_size,
                    'changed_during_hash': (before.st_ino, before.st_size, before.st_mtime_ns) !=
                                           (after.st_ino, after.st_size, after.st_mtime_ns)}
            return self.cache[key]
        except (OSError, ValueError) as error:
            return {'path': str(path), 'error': repr(error)}


def description_record(description):
    result = {'sha256': hashlib.sha256(description.encode()).hexdigest(),
              'size_bytes': len(description.encode())}
    try:
        robot = ET.fromstring(description)
        result['robot_name'] = robot.get('name')
        result['plugins'] = [dict(plugin.attrib) for plugin in robot.iter('plugin')]
    except ET.ParseError as error:
        result['xml_error'] = str(error)
    return result


def parameter_file_records(command, hashes):
    records = []
    for index, token in enumerate(command[:-1]):
        if token != '--params-file':
            continue
        path = Path(command[index+1])
        record = hashes.record(path)
        try:
            data = yaml.safe_load(path.read_text())
            descriptions = []
            def visit(value):
                if isinstance(value, dict):
                    for key, entry in value.items():
                        if key == 'robot_description' and isinstance(entry, str):
                            descriptions.append(description_record(entry))
                        else:
                            visit(entry)
                elif isinstance(value, list):
                    for entry in value:
                        visit(entry)
            visit(data)
            if descriptions:
                record = {**record, 'robot_descriptions': descriptions}
        except (OSError, yaml.YAMLError) as error:
            record = {**record, 'parameter_parse_error': repr(error)}
        records.append(record)
    return records


def argument_tokens(command):
    # Gazebo's Ruby entry point overwrites argv with one display string.
    # Ordinary argv entries (including paths containing spaces) stay intact.
    tokens = []
    for argument in command:
        if argument.startswith(('gz sim ', 'ruby ')):
            try:
                tokens.extend(shlex.split(argument))
            except ValueError:
                tokens.append(argument)
        elif argument:
            tokens.append(argument)
    return tokens


def process_roles(executable, command, library_paths):
    basename = Path(executable).name
    roles = []
    if basename == 'pointlio_mapping':
        roles.append('point_lio')
    if basename == 'ign_sim_pointcloud_tool_node' or any('libign_sim_pointcloud_tool.so' in p for p in library_paths):
        roles.append('pointcloud_converter')
    if basename == 'loam_interface_node' or any('libloam_interface.so' in p for p in library_paths):
        roles.append('loam_interface')
    if basename == 'robot_state_publisher':
        roles.append('robot_description_publisher')
    if basename == 'parameter_bridge':
        roles.append('simulation_bridge')
    simulator_tokens = argument_tokens(command)
    if (basename == 'gz' or basename.startswith('ruby')) and 'sim' in simulator_tokens:
        roles.append('simulator')
    return roles


def collect_processes(domain, hashes):
    records = []
    for process in psutil.process_iter(['pid', 'exe', 'cmdline', 'create_time']):
        try:
            environment = process.environ()
            if environment.get('ROS_DOMAIN_ID', '0') != str(domain):
                continue
            executable = process.info['exe'] or ''
            command = process.info['cmdline'] or []
            # Avoid inspecting every Python/audit process's large maps list.
            basename = Path(executable).name
            if basename not in ('pointlio_mapping', 'ign_sim_pointcloud_tool_node', 'loam_interface_node',
                                'robot_state_publisher', 'parameter_bridge', 'component_container', 'component_container_mt', 'gz') and not basename.startswith('ruby'):
                continue
            libraries = sorted({entry.path for entry in process.memory_maps(grouped=True)
                                if '.so' in entry.path and entry.path.startswith('/')})
            roles = process_roles(executable, command, libraries)
            if not roles:
                continue
            records.append({'pid': process.pid, 'create_time_unix': process.info['create_time'],
                'roles': roles, 'executable': hashes.record(f'/proc/{process.pid}/exe'), 'command': command,
                'startup_parameter_files': parameter_file_records(command, hashes),
                'loaded_libraries': [hashes.record(path) for path in libraries],
                'environment': {key: environment.get(key) for key in
                    ('ROS_DOMAIN_ID', 'GZ_PARTITION', 'RMW_IMPLEMENTATION', 'FASTDDS_DEFAULT_PROFILES_FILE',
                     'FASTRTPS_DEFAULT_PROFILES_FILE', 'OMP_NUM_THREADS')},
                'command_input_files': [hashes.record(token) for token in argument_tokens(command)
                                        if token.endswith(('.world', '.urdf', '.sdf')) and Path(token).is_file()]})
        except (psutil.NoSuchProcess, psutil.AccessDenied, OSError):
            continue
    return records


def runtime_parameters(robot_node, timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient
    rclpy.init(args=[])
    node = Node('mapping_provenance_readonly_audit')
    result = {}
    requests = {robot_node: ['robot_description'], '/ign_sim_pointcloud_tool': ['instantaneous_scan'],
                '/point_lio': ['preprocess.instantaneous_scan', 'prior_pcd.enable', 'mapping.imu_en',
                               'use_imu_as_input', 'input_buffer.imu_depth', 'input_buffer.lidar_depth',
                               'diagnostics.trace_file'],
                '/loam_interface': ['anchor_initial_base_frame']}
    try:
        for name, names in requests.items():
            client = AsyncParameterClient(node, name)
            if not client.wait_for_services(timeout_sec=timeout):
                result[name] = {'error': 'parameter read service unavailable'}
                continue
            future = client.get_parameters(names)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.exception():
                result[name] = {'error': 'parameter read timed out or failed'}
                continue
            values = dict(zip(names, map(parameter_value_to_python, future.result().values)))
            if 'robot_description' in values:
                description = values.pop('robot_description')
                values['robot_description'] = description_record(description) if isinstance(description, str) else None
            result[name] = values
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--robot-node', default='/robot_state_publisher')
    parser.add_argument('--parameter-timeout', type=float, default=3.0)
    parser.add_argument('--require-mapping', action='store_true')
    args = parser.parse_args()
    if args.parameter_timeout <= 0:
        parser.error('parameter timeout must be positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    hashes = FileHashes()
    report = {'schema_version': 1, 'created_unix': time.time(), 'domain_id': args.domain,
              'read_only': True, 'processes': collect_processes(args.domain, hashes)}
    report['runtime_parameters'] = runtime_parameters(args.robot_node, args.parameter_timeout)
    report['finished_unix'] = time.time()
    for process in report['processes']:
        try:
            process['same_process_alive_at_completion'] = (
                psutil.Process(process['pid']).create_time() == process['create_time_unix'])
        except psutil.NoSuchProcess:
            process['same_process_alive_at_completion'] = False
    roles = {role for process in report['processes'] for role in process['roles']}
    report['missing_mapping_roles'] = sorted({'point_lio', 'pointcloud_converter', 'loam_interface'} - roles)
    actual = report['runtime_parameters'].get(args.robot_node, {}).get('robot_description')
    startup = [description for process in report['processes']
               if 'robot_description_publisher' in process['roles']
               for record in process['startup_parameter_files'] for description in record.get('robot_descriptions', [])]
    report['robot_description_runtime_matches_startup'] = (
        any(actual['sha256'] == description['sha256'] for description in startup)
        if actual and startup else None)
    report['interpretation'] = ('Loaded-library paths establish the actual converter/loam implementation; the wrapper executable hash alone is insufficient. '
        'robot_description hashes describe the ROS runtime and startup parameter inputs; they do not independently prove the current Gazebo entity was spawned from that same description.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), 'domain': args.domain,
                      'roles': sorted(roles), 'missing_mapping_roles': report['missing_mapping_roles'],
                      'robot_description_runtime_matches_startup': report['robot_description_runtime_matches_startup']}))
    return 2 if args.require_mapping and report['missing_mapping_roles'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
