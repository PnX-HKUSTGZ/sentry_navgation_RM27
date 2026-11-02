import hashlib

import yaml

from audit_mapping_provenance import FileHashes, description_record, parameter_file_records, process_roles, argument_tokens


def test_converter_roles_come_from_actual_loaded_library_and_simulator_supports_versioned_ruby():
    assert process_roles('/usr/bin/component_container', [], ['/new/libign_sim_pointcloud_tool.so']) == ['pointcloud_converter']
    assert process_roles('/usr/bin/ruby3.2', ['/usr/bin/gz', 'sim', '-s'], []) == ['simulator']
    assert process_roles('/usr/bin/ruby3.2', ['gz sim -r -s', '', ''], []) == ['simulator']
    assert process_roles('/usr/bin/ruby3.2', ['unrelated'], []) == []
    assert argument_tokens(['node', '/path with spaces/robot.urdf']) == ['node', '/path with spaces/robot.urdf']


def test_startup_robot_description_and_file_hash_change_are_recorded(tmp_path):
    robot = '<robot name="test"><gazebo><plugin name="drive" filename="velocity.so"/></gazebo></robot>'
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': robot}}}))
    hashes = FileHashes()
    records = parameter_file_records(['node', '--params-file', str(path)], hashes)
    assert records[0]['robot_descriptions'][0] == description_record(robot)
    assert records[0]['robot_descriptions'][0]['plugins'] == [{'name': 'drive', 'filename': 'velocity.so'}]
    before = records[0]['sha256']
    path.write_text('changed')
    assert hashes.record(path)['sha256'] == hashlib.sha256(b'changed').hexdigest()
    assert hashes.record(path)['sha256'] != before
