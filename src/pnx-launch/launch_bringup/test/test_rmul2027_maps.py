import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml


BRINGUP=Path(__file__).resolve().parents[1]
MAP=BRINGUP/'map/simulation'


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
    assert module._apply_world_profile(str(source),'unconfigured_world')==str(source)
    resolved=Path(module._apply_world_profile(str(source),'RMUL2027'))
    try:
        assert not any(isinstance(token,yaml.tokens.AliasToken) for token in yaml.scan(resolved.read_text()))
        original=yaml.safe_load(source.read_text())
        data=yaml.safe_load(resolved.read_text())
        params=data['rm27_backend']['ros__parameters']
        # 标准导航使用后端启动先验；旧定位模块不再出现在场景覆盖中。
        profile=original['simulation_world_profiles']['ros__parameters']['RMUL2027']
        assert 'relocalization_manager' not in profile
        assert params['prior_map.transform']==[0.0]*6
        assert params['startup_pose.enabled'] is True
        assert params['startup_pose.map_from_base']==profile['rm27_backend']['startup_pose.map_from_base']
        assert params['prior_map.seed_source']=='none'
        assert 'relocalization_manager' not in original
        assert 'relocalization_manager' not in data
        assert data['controller_server']==original['controller_server']
        assert data['point_lio']['ros__parameters']['filter_size_map']==.08
        assert data['point_lio']['ros__parameters']['preprocess']['scan_line']==192
        assert data['point_lio']['ros__parameters']['preprocess']['lidar_type']==2
        assert data['ign_sim_pointcloud_tool']['ros__parameters']['instantaneous_scan']
    finally:
        resolved.unlink()


@pytest.mark.parametrize('filename', ['minco_params.yaml', 'nav2_params.yaml'])
@pytest.mark.parametrize('world', ['RMUL2027', 'RMUC2027'])
def test_backend_startup_profiles_use_spawn_reference_and_same_schema(filename, world):
    """固定出生区先验引用正确底盘参考点，模式切换不得丢失后端覆盖。"""
    profile = yaml.safe_load((BRINGUP/'config/simulation'/filename).read_text())[
        'simulation_world_profiles']['ros__parameters'][world]
    spawn_file = BRINGUP.parents[1]/'pnx-stimulation/rm_27_stimulation/config/spawn_poses.yaml'
    spawn = yaml.safe_load(spawn_file.read_text())['spawn_poses'][world]
    backend = profile['rm27_backend']
    assert set(backend) == {'startup_pose.enabled', 'startup_pose.map_from_base', 'prior_map.transform'}
    assert backend['startup_pose.enabled'] is True
    assert backend['startup_pose.map_from_base'] == [spawn[k] for k in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']]
    assert backend['prior_map.transform'] == [0.0]*6
    assert 'relocalization_manager' not in profile
