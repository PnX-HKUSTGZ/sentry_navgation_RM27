from pathlib import Path
import struct

import numpy as np

from actual_collision_support import (CollisionSupportScene, _box, filter_supported_positions,
                                      robot_collision_envelope)


def robot(tmp_path):
    links = ['<link name="base"><collision><origin xyz="0 0 .06"/>'
             '<geometry><cylinder radius=".28" length=".12"/></geometry></collision>'
             '<collision><origin xyz="-.043 -.016 .14"/>'
             '<geometry><box size=".362 .354 .04"/></geometry></collision></link>']
    for index, (x, y) in enumerate([(.218, -.218), (-.218, .218), (-.218, -.218), (.218, .218)]):
        links.append(f'<link name="wheel{index}"><collision><geometry><sphere radius=".08"/></geometry></collision></link>'
                     f'<joint name="j{index}" type="fixed"><parent link="base"/><child link="wheel{index}"/>'
                     f'<origin xyz="{x} {y} 0"/></joint>')
    path = tmp_path / 'robot.urdf'
    path.write_text('<robot name="test">'+''.join(links)+'</robot>')
    return path


def scene_with_boxes(boxes):
    triangles, labels = [], []
    for index, (size, center) in enumerate(boxes):
        mesh = _box(size)+center
        triangles.extend(mesh)
        labels.extend([str(index)]*len(mesh))
    return CollisionSupportScene(triangles, labels)


def test_actual_robot_envelope_uses_contact_plane_and_preserves_flat_and_high_platforms(tmp_path):
    envelope = robot_collision_envelope(robot(tmp_path))
    assert abs(envelope['swept_radius']-(np.sqrt(2)*.218+.08)) < 1e-12
    assert envelope['base_height_above_contact'] == .08
    assert envelope['height_above_contact'] == .24
    scene = scene_with_boxes([([10, 10, .2], [0, 0, -.1]), ([3, 3, .4], [0, 0, .2])])
    accepted, height, _ = scene.check_position([0, 0], envelope)
    assert accepted and abs(height-.4) < 1e-12
    accepted, height, _ = scene.check_position([-3, 0], envelope)
    assert accepted and abs(height) < 1e-12
    accepted, _, reason = scene.check_position([1.4, 0], envelope)
    assert not accepted and reason == 'nonplanar_support'
    accepted, _, reason = scene.check_position([10, 0], envelope)
    assert not accepted and reason == 'unsupported_footprint'


def test_whole_turn_clearance_rejects_vertical_wall_and_low_overhead_volume(tmp_path):
    envelope = robot_collision_envelope(robot(tmp_path))
    ground = ([10, 10, .2], [0, 0, -.1])
    # Wall top is above drivable support scope. Its vertical projection is a
    # line, so a height-only surface test would miss its collision face.
    wall = scene_with_boxes([ground, ([.02, 2, 1], [.3, 0, .5])])
    accepted, _, reason = wall.check_position([0, 0], envelope)
    assert not accepted and reason == 'swept_volume_collision'
    roof = scene_with_boxes([ground, ([2, 2, 1], [0, 0, .68])])
    accepted, _, reason = roof.check_position([0, 0], envelope)
    assert not accepted and reason == 'swept_volume_collision'
    clear_roof = scene_with_boxes([ground, ([2, 2, 1], [0, 0, 1])])
    assert clear_roof.check_position([0, 0], envelope)[0]
    # A robot entirely inside a solid can avoid every triangle surface while
    # still being physically invalid. Containment must be rejected separately.
    enclosing_solid = scene_with_boxes([ground, ([3, 3, 1], [0, 0, .5])])
    accepted, _, reason = enclosing_solid.check_position([0, 0], envelope)
    assert not accepted and reason == 'inside_collision_solid'


def test_world_include_pose_mesh_hashes_and_corrected_support_height(tmp_path):
    model_root = tmp_path / 'models'
    model = model_root / 'platform'
    model.mkdir(parents=True)
    triangles = _box([3, 3, .4])+[0, 0, .2]
    mesh = model / 'platform.stl'
    with mesh.open('wb') as stream:
        stream.write(bytes(80)+struct.pack('<I', len(triangles)))
        for triangle in triangles:
            stream.write(struct.pack('<12fH', *np.zeros(3), *triangle.reshape(-1), 0))
    sdf = model / 'model.sdf'
    sdf.write_text('<sdf version="1.9"><model name="platform"><static>true</static><link name="l">'
        '<collision name="c"><geometry><mesh><uri>model://platform/platform.stl</uri></mesh></geometry>'
        '</collision></link></model></sdf>')
    world = tmp_path / 'world.sdf'
    world.write_text('<sdf version="1.9"><world name="world"><include><uri>model://platform</uri>'
        '<pose rotation_format="quat_xyzw">2 3 0 0 0 0 1</pose></include></world></sdf>')
    urdf = robot(tmp_path)
    mask, heights, metadata = filter_supported_positions([[2, 3]], world_path=world,
        robot_urdf_path=urdf, model_roots=[model_root], cache_dir=tmp_path/'cache')
    assert mask.tolist() == [True]
    np.testing.assert_allclose(heights, [.4], atol=1e-7)
    assert metadata['accepted_positive_height_count'] == 1
    for path in [world, sdf, mesh, urdf, Path(__file__).with_name('actual_collision_support.py')]:
        assert str(path.resolve()) in metadata['source_hashes']
    assert metadata['ground_truth_used'] is False
    assert metadata['trial_outcomes_used'] is False
    again, same_heights, same_metadata = filter_supported_positions([[2, 3]], world_path=world,
        robot_urdf_path=urdf, model_roots=[model_root], cache_dir=tmp_path/'cache')
    np.testing.assert_array_equal(again, mask)
    np.testing.assert_array_equal(same_heights, heights)
    assert same_metadata == metadata
    changed, _, changed_metadata = filter_supported_positions([[2, 3]], world_path=world,
        robot_urdf_path=urdf, model_roots=[model_root], cache_dir=tmp_path/'cache', radial_margin=.03)
    assert changed[0]
    assert changed_metadata['persistent_cache_key'] != metadata['persistent_cache_key']
