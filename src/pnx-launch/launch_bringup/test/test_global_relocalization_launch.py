"""Global initialization retains one map-to-odom owner in every mode."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext

BRINGUP_DIR = Path(__file__).resolve().parents[1]


def load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("truth,global_match,owner", [
    ("false", "true", "relocalization_manager_node"),
    ("false", "false", "relocalization_manager_node"),
    ("true", "true", None),
    ("true", "false", None),
    ("1", "true", None),
])
def test_global_handoff_has_one_map_odom_owner(truth, global_match, owner):
    module = load_launch(BRINGUP_DIR / "launch/localization_launch.py", "rm27_localization")
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(use_ground_truth_odom=truth,
                                         global_relocalization=global_match,
                                         use_composition="false")
    nodes = []
    for entity in description.entities:
        if hasattr(entity, "node_package"):
            nodes.append(entity)
        elif hasattr(entity, "get_sub_entities"):
            nodes.extend(e for e in entity.get_sub_entities() if hasattr(e, "node_package"))
    active = {n.node_executable for n in nodes
              if n.node_executable in {"global_pose_handoff.py", "relocalization_manager_node"}
              and n.condition.evaluate(context)}
    assert active == ({owner} if owner else set())
    pointlio = next(n for n in nodes if n.node_executable == "pointlio_mapping")
    assert pointlio.condition.evaluate(context) == (truth == "false")


def configured_context(params, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(
        params_file=str(params), navigation_mode='minco', deployment='simulation',
        namespace='', slam='false', use_ground_truth_odom='false', world='RMUC2027',
        use_sim_time='true', use_composition='false', use_respawn='false',
        prior_pcd_file='/unused.pcd', map='/unused.yaml', autostart='true',
        log_level='info', stationary_relocalization='true')
    context.launch_configurations.update(overrides)
    return context


@pytest.fixture
def profile(tmp_path):
    import yaml
    configuration = yaml.safe_load((BRINGUP_DIR / 'config/simulation/minco_params.yaml').read_text())
    settings = configuration['global_relocalization']['ros__parameters']
    settings['enabled'] = True
    # Use real delivered assets, including launch substitution resolution.
    path = tmp_path / 'minco_params.yaml'
    path.write_text(yaml.safe_dump(configuration))
    return path


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('composition', ['false', 'true'])
def test_yaml_controls_both_global_processes_and_gates_navigation(profile, enabled, composition):
    import yaml
    from launch.actions import OpaqueFunction
    from relocalization_manager_launch.config import configure_global_relocalization
    configuration = yaml.safe_load(profile.read_text())
    configuration['global_relocalization']['ros__parameters']['enabled'] = enabled
    profile.write_text(yaml.safe_dump(configuration))
    # A stale command-line flag must not override the selected YAML.
    context = configured_context(profile, global_relocalization=str(not enabled).lower(),
                                 use_composition=composition)
    for action in configure_global_relocalization(context):
        action.execute(context)
    assert context.launch_configurations['_global_relocalization_enabled'] == str(enabled).lower()
    assert context.launch_configurations['autostart'] == ('false' if enabled else 'true')
    if enabled:
        assert context.launch_configurations['prior_pcd_file'].endswith('RMUC2027.pcd')
        effective = yaml.safe_load(Path(context.launch_configurations['params_file']).read_text())
        assert effective['relocalization_manager']['ros__parameters']['prior_pcd_transform'] == [0.] * 6
    module = load_launch(BRINGUP_DIR / 'launch/localization_launch.py', 'integrated_localization')
    actions = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context) or [])
    processes = {n.node_executable: n for n in actions if hasattr(n, 'node_executable')}
    assert ('global_relocalization_node' in processes) == enabled
    assert ('relocalize_then_navigate.py' in processes) == enabled
    if enabled:
        assert all(n.node_package == 'relocalization_manager' for n in processes.values()
                   if n.node_executable in ('global_relocalization_node', 'relocalize_then_navigate.py'))


@pytest.mark.parametrize('overrides', [
    {'use_ground_truth_odom': 'true'}, {'slam': 'true'},
    {'navigation_mode': 'legacy'}, {'world': 'RMUL2027'},
])
def test_enabled_global_rejects_incompatible_localization_sources(profile, overrides):
    from relocalization_manager_launch.config import configure_global_relocalization
    with pytest.raises(RuntimeError):
        configure_global_relocalization(configured_context(profile, **overrides))


@pytest.mark.parametrize('key,value', [
    ('enabled', 'false'), ('map_path', '/missing.pcd'),
    ('manifest_path', '/missing.json'), ('cmd_vel_topic', '/cmd_vel'),
    ('input_mode', 'registered'), ('competition_mode', False), ('base_frame', 'gimbal_yaw'),
])
def test_bad_global_configuration_fails_before_nodes_start(profile, key, value):
    import yaml
    from relocalization_manager_launch.config import configure_global_relocalization
    configuration = yaml.safe_load(profile.read_text())
    configuration['global_relocalization']['ros__parameters'][key] = value
    profile.write_text(yaml.safe_dump(configuration))
    with pytest.raises((RuntimeError, ValueError)):
        configure_global_relocalization(configured_context(profile))


def test_public_launches_expose_no_global_enable_suffix():
    from launch.actions import DeclareLaunchArgument
    root = BRINGUP_DIR.parents[1]
    paths = [BRINGUP_DIR / 'launch' / name for name in (
        'bringup_launch.py', 'rm_navigation_simulation_launch.py',
        'rm_navigation_reality_launch.py', 'localization_launch.py')]
    paths.append(root / 'pnx-stimulation/rm_27_stimulation/launch/sim_with_nav.launch.py')
    for path in paths:
        module = load_launch(path, path.stem)
        names = {a.name for a in module.generate_launch_description().entities
                 if isinstance(a, DeclareLaunchArgument)}
        assert 'global_relocalization' not in names
        assert 'relocalization_map' not in names
        assert 'relocalization_manifest' not in names


@pytest.mark.parametrize('kind', ['frame', 'prior_transform'])
def test_global_and_local_map_contract_must_match(profile, kind):
    import yaml
    from relocalization_manager_launch.config import configure_global_relocalization
    configuration = yaml.safe_load(profile.read_text())
    if kind == 'frame':
        configuration['relocalization_manager']['ros__parameters']['lidar_frame'] = 'wrong_sensor'
    else:
        configuration['simulation_world_profiles']['ros__parameters']['RMUC2027'][
            'relocalization_manager']['prior_pcd_transform'] = [0., 0., 1.65, 0., 0., 0.]
    profile.write_text(yaml.safe_dump(configuration))
    with pytest.raises(RuntimeError):
        configure_global_relocalization(configured_context(profile))


def test_reality_requires_its_own_map_manifest_before_enabling(tmp_path):
    import yaml
    from relocalization_manager_launch.config import configure_global_relocalization
    configuration = yaml.safe_load((BRINGUP_DIR/'config/reality/minco_params.yaml').read_text())
    configuration['global_relocalization']['ros__parameters']['enabled'] = True
    path = tmp_path/'reality.yaml'
    path.write_text(yaml.safe_dump(configuration))
    with pytest.raises(RuntimeError, match='map_path'):
        configure_global_relocalization(configured_context(path, deployment='reality', world=''))


@pytest.mark.parametrize('deployment', ['reality', 'simulation'])
def test_global_imu_units_match_pointlio_and_driver_cloud_type(deployment):
    import yaml
    configuration = yaml.safe_load((BRINGUP_DIR/f'config/{deployment}/minco_params.yaml').read_text())
    settings = configuration['global_relocalization']['ros__parameters']
    pointlio = configuration['point_lio']['ros__parameters']
    # Both consumers must convert the same raw IMU to SI, independently.
    assert settings['raw_imu_accel_scale'] == pytest.approx(9.81 / pointlio['mapping']['acc_norm'])
    if deployment == 'reality':
        assert configuration['livox_ros_driver2']['ros__parameters']['xfer_format'] == 4
        assert settings['raw_cloud_topic'] == '/' + pointlio['common']['lid_topic'] + '/pointcloud'
        assert settings['imu_topic'] == '/' + pointlio['common']['imu_topic']
