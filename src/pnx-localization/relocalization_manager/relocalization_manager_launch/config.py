"""Read the same MINCO profile at the simulation and navigation entry points."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile


def select_params_file(path, deployment, navigation_mode):
    if navigation_mode != 'minco':
        return path
    share = get_package_share_directory('launch_bringup')
    defaults = {os.path.realpath(os.path.join(share, 'config', d, 'nav2_params.yaml'))
                for d in ('simulation', 'reality')}
    if os.path.realpath(path) in defaults:
        return os.path.join(share, 'config', deployment, 'minco_params.yaml')
    return path


def apply_world_profile(params_file, world):
    """Resolve world-specific geometry/registration settings into one file."""
    if not world:
        return params_file
    with open(params_file, encoding="utf-8") as stream:
        configuration = yaml.safe_load(stream)
    # The profile block is input metadata, not a runtime node. Removing it
    # also avoids YAML aliases when its list values become node parameters;
    # ROS's C parameter parser does not support aliases.
    profile = configuration.pop("simulation_world_profiles", {}).get(
        "ros__parameters", {}).get(world)
    if not profile:
        return params_file
    def merge(target, overrides):
        for key, value in overrides.items():
            if isinstance(value, dict):
                merge(target.setdefault(key, {}), value)
            else:
                target[key] = value
    for node_name, parameters in profile.items():
        merge(configuration.setdefault(node_name, {}).setdefault("ros__parameters", {}), parameters)
    with tempfile.NamedTemporaryFile(mode="w", prefix="rm27_world_params_", suffix=".yaml",
                                     encoding="utf-8", delete=False) as stream:
        yaml.safe_dump(configuration, stream, sort_keys=False)
        return stream.name


def global_settings(context, params_file, navigation_mode, deployment,
                    slam=False, ground_truth=False, world=''):
    # ParameterFile evaluates $(find-pkg-share ...) exactly as runtime nodes do.
    parameter_file = ParameterFile(params_file, allow_substs=True)
    resolved = parameter_file.evaluate(context)
    with open(resolved, encoding='utf-8') as stream:
        configuration = yaml.safe_load(stream) or {}
    settings = configuration.get('global_relocalization', {}).get('ros__parameters', {})
    enabled = settings.get('enabled', False)
    if not isinstance(enabled, bool):
        raise ValueError('global_relocalization.enabled must be a YAML boolean')
    if not enabled:
        return settings
    if navigation_mode != 'minco' or slam or ground_truth:
        raise RuntimeError('global_relocalization.enabled requires MINCO, slam:=false '
                           'and use_ground_truth_odom:=false (Point-LIO)')
    if deployment == 'simulation' and world and world != 'RMUC2027':
        raise RuntimeError('global relocalization simulation requires RMUC2027')
    for key in ('map_path', 'manifest_path'):
        if not os.path.isfile(settings.get(key, '')):
            raise RuntimeError(f'global_relocalization.{key} must name an existing file '
                               f'in minco_params.yaml; got {settings.get(key)!r}')
    if settings.get('input_mode') != 'raw_imu' or settings.get('competition_mode') is not True:
        raise RuntimeError('Integrated startup requires input_mode: raw_imu and competition_mode: true')
    if settings.get('cmd_vel_topic') != '/relocalization/cmd_vel':
        raise RuntimeError('Global commands must use the coordinator input /relocalization/cmd_vel')
    for key, expected in (('map_frame', 'map'), ('odom_frame', 'odom'),
                          ('base_frame', 'base_footprint')):
        if settings.get(key) != expected:
            raise RuntimeError(f'Integrated startup requires {key}: {expected}')
    manager = dict(configuration.get('relocalization_manager', {}).get('ros__parameters', {}))
    if deployment == 'simulation':
        manager.update(configuration.get('simulation_world_profiles', {}).get(
            'ros__parameters', {}).get(world, {}).get('relocalization_manager', {}))
    for key in ('map_frame', 'odom_frame', 'base_frame', 'lidar_frame'):
        if manager.get(key) != settings.get(key):
            raise RuntimeError(f'Global and local relocalization disagree on {key}')
    if (any(manager.get('prior_pcd_transform', [0.0] * 6)) or
            manager.get('transform_prior_map_with_lidar_offset', False)):
        raise RuntimeError('Global and local matching must use the same map-frame PCD; '
                           'remove local prior PCD transforms in the selected world profile')
    return settings


def configure_global_relocalization(context):
    """Internal launch state is always derived from YAML, never a CLI switch."""
    value = lambda name, default: LaunchConfiguration(name, default=default).perform(context)
    mode = value('navigation_mode', 'legacy').lower()
    deployment = value('deployment', 'reality').lower()
    path = select_params_file(value('params_file', ''), deployment, mode)
    world = value('world', '')
    if deployment == 'simulation':
        path = apply_world_profile(path, world)
    settings = global_settings(
        context, path, mode, deployment,
        slam=value('slam', 'false').lower() in ('true', '1', 'yes', 'on'),
        ground_truth=value('use_ground_truth_odom', 'false').lower() in ('true', '1', 'yes', 'on'),
        world=world)
    enabled = settings.get('enabled', False)
    actions = [SetLaunchConfiguration('params_file', path),
               SetLaunchConfiguration('_global_relocalization_enabled', str(enabled).lower())]
    if enabled:
        actions.extend([SetLaunchConfiguration('autostart', 'false'),
                        SetLaunchConfiguration('prior_pcd_file', settings['map_path'])])
    return actions
