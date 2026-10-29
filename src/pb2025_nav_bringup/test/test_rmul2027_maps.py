import importlib.util
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


BRINGUP=Path(__file__).resolve().parents[1]
MAP=BRINGUP/'map/simulation'


def test_rmuc_default_prior_and_world_profile(monkeypatch):
    source = BRINGUP.parents[1] / 'tools/global_relocalization/assets/RMUC2026'
    prior_dir = BRINGUP / 'pcd/simulation'
    pcd = prior_dir / 'RMUC2026.pcd'
    manifest_path = prior_dir / 'RMUC2026_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    assert manifest_path.read_bytes() == (source / 'manifest.json').read_bytes()
    digest = hashlib.sha256(pcd.read_bytes()).hexdigest()
    assert digest == manifest['map_sha256']
    assert digest == hashlib.sha256((source / 'map.pcd').read_bytes()).hexdigest()
    assert manifest['pose_source'] == 'gazebo_ground_truth_map_preparation_only'
    assert manifest['T_map_world'] == [[1., 0., 0., 0.], [0., 1., 0., 0.],
                                       [0., 0., 1., 0.], [0., 0., 0., 1.]]

    spec = importlib.util.spec_from_file_location(
        'rmuc_entry', BRINGUP / 'launch/rm_navigation_simulation_launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(BRINGUP))
    declarations = {action.name: action for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)}
    context = LaunchContext()
    world = perform_substitutions(context, declarations['world'].default_value)
    assert world == 'RMUC2026'
    context.launch_configurations['world'] = world
    assert Path(perform_substitutions(context, declarations['prior_pcd_file'].default_value)) == pcd

    params_file = BRINGUP / 'config/simulation/minco_params.yaml'
    original = yaml.safe_load(params_file.read_text())
    resolved = Path(module._apply_world_profile(str(params_file), world))
    try:
        result = yaml.safe_load(resolved.read_text())
        params = result['relocalization_manager']['ros__parameters']
        assert params['prior_pcd_transform'] == [0.0] * 6
        assert params['transform_prior_map_with_lidar_offset'] is False
        assert params['init_pose'] == original['relocalization_manager']['ros__parameters']['init_pose']
        assert result['controller_server'] == original['controller_server']
    finally:
        resolved.unlink()

    context.launch_configurations.update({
        'namespace': '', 'use_ground_truth_odom': 'false', 'slam': 'false',
        'navigation_mode': 'minco', 'params_file': str(params_file),
        'prior_pcd_file': str(pcd),
    })
    actions = module._validate_localization_inputs(context)
    assert len(actions) == 1
    actions[0].execute(context)
    selected = Path(context.launch_configurations['params_file'])
    try:
        selected_params = yaml.safe_load(selected.read_text())['relocalization_manager']['ros__parameters']
        assert selected_params['prior_pcd_transform'] == [0.0] * 6
        assert selected_params['transform_prior_map_with_lidar_offset'] is False
    finally:
        selected.unlink()


def read_pcd():
    with (BRINGUP/'pcd/simulation/RMUL2027.pcd').open('rb') as stream:
        header={}
        while True:
            line=stream.readline().decode('ascii').strip()
            if not line or line.startswith('#'): continue
            key,_,value=line.partition(' ')
            header[key]=value
            if key=='DATA': break
        assert header['FIELDS']=='x y z'
        assert header['DATA']=='binary'
        points=np.frombuffer(stream.read(),dtype='<f4').reshape(-1,3)
    assert len(points)==int(header['POINTS'])
    return points


def test_localization_cloud_keeps_walls_and_has_no_hidden_bottom_plane():
    points=read_pcd()
    assert np.isfinite(points).all()
    assert 20000<len(points)<100000
    assert points[:,2].min()>=-.002
    assert points[:,2].max()<=.421
    for axis,coordinate in [(0,-6),(0,6),(1,-4),(1,4)]:
        assert np.count_nonzero((np.abs(points[:,axis]-coordinate)<.015)&(points[:,2]>.10))>100


def test_grid_uses_the_same_world_coordinates_without_preinflation():
    config=yaml.safe_load((MAP/'RMUL2027.yaml').read_text())
    pixels=np.asarray(Image.open(MAP/config['image']))
    assert config['origin']==[-6.5,-4.5,0.0]
    assert config['resolution']==.05
    assert pixels.shape==(180,260)
    def cell(x,y):
        return pixels[len(pixels)-1-int((y+4.5)/.05),int((x+6.5)/.05)]
    for point in [(0,0),(-4,0),(4,0),(-5,3),(5,-3)]:
        assert cell(*point)==254,point
    for point in [(2.5,0),(-2.5,0),(6.15,0),(-6.15,0)]:
        assert cell(*point)==0,point
    assert cell(-6.45,4.45)==205
    # Nav2 must interpret the grey border as UNKNOWN, not FREE.
    assert config['free_thresh']<(255-205)/255<config['occupied_thresh']


@pytest.mark.parametrize('filename',['minco_params.yaml','nav2_params.yaml'])
def test_rmul_registration_profile_is_applied_without_changing_other_worlds(filename):
    path=BRINGUP/'launch/rm_navigation_simulation_launch.py'
    spec=importlib.util.spec_from_file_location('rmul_entry',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source=BRINGUP/'config/simulation'/filename
    if filename == 'nav2_params.yaml':
        assert module._apply_world_profile(str(source),'RMUC2026')==str(source)
    resolved=Path(module._apply_world_profile(str(source),'RMUL2027'))
    try:
        assert not any(isinstance(token,yaml.tokens.AliasToken) for token in yaml.scan(resolved.read_text()))
        original=yaml.safe_load(source.read_text())
        data=yaml.safe_load(resolved.read_text())
        params=data['relocalization_manager']['ros__parameters']
        assert params['prior_pcd_transform']==[0.0]*6
        # Initial pose is user tuning; verify the world override is applied
        # instead of pinning every deployment to the original centre spawn.
        profile=original['simulation_world_profiles']['ros__parameters']['RMUL2027']
        assert params['init_pose']==profile['relocalization_manager']['init_pose']
        assert params['global_leaf_size']==.05
        for gate in ['max_mean_error','min_inlier_ratio','min_inliers']:
            assert params[gate]==original['relocalization_manager']['ros__parameters'][gate]
        assert data['controller_server']==original['controller_server']
        assert data['point_lio']['ros__parameters']['filter_size_map']==.08
        assert data['point_lio']['ros__parameters']['preprocess']['scan_line']==192
        assert data['point_lio']['ros__parameters']['preprocess']['lidar_type']==2
        assert data['ign_sim_pointcloud_tool']['ros__parameters']['instantaneous_scan']
    finally:
        resolved.unlink()
