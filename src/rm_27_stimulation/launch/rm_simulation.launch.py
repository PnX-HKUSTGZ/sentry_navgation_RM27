#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _world_path(context):
    package_share = get_package_share_directory("rm_27_stimulation")
    config_path = LaunchConfiguration("worlds_config").perform(context)
    world_name = LaunchConfiguration("sim_world").perform(context)

    worlds = _load_yaml(config_path).get("worlds", {})
    if world_name not in worlds:
        valid_worlds = ", ".join(sorted(worlds.keys()))
        raise RuntimeError(f"Unknown world '{world_name}'. Valid worlds: {valid_worlds}")

    relative_path = worlds[world_name].get("path")
    if not relative_path:
        raise RuntimeError(f"World '{world_name}' does not define a 'path' field")

    resolved_path = os.path.join(package_share, "world", relative_path)
    if not os.path.exists(resolved_path):
        raise RuntimeError(f"World file does not exist: {resolved_path}")

    return resolved_path


def _launch_gzserver(context, *args, **kwargs):
    del args, kwargs

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    gzserver_launch = os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_launch),
            launch_arguments={
                "world": _world_path(context),
                "verbose": LaunchConfiguration("verbose").perform(context),
                "pause": LaunchConfiguration("pause").perform(context),
                "extra_gazebo_args": LaunchConfiguration("extra_gazebo_args").perform(context),
            }.items(),
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("rm_27_stimulation")
    package_prefix = get_package_prefix("rm_27_stimulation")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    gazebo_ros_prefix = get_package_prefix("gazebo_ros")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_world",
                default_value="RMUC2026",
                description="World key from config/worlds.yaml",
            ),
            DeclareLaunchArgument(
                "worlds_config",
                default_value=os.path.join(package_share, "config", "worlds.yaml"),
                description="YAML file describing available Gazebo worlds",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start Gazebo GUI client when true",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Run gzserver/gzclient with verbose output",
            ),
            DeclareLaunchArgument(
                "pause",
                default_value="false",
                description="Start Gazebo paused when true",
            ),
            DeclareLaunchArgument(
                "extra_gazebo_args",
                default_value="",
                description="Additional raw arguments passed to Gazebo",
            ),
            AppendEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(package_share, "meshes")),
            AppendEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(package_share, "world")),
            AppendEnvironmentVariable("GAZEBO_RESOURCE_PATH", package_share),
            AppendEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.path.join(package_prefix, "lib")),
            AppendEnvironmentVariable("GAZEBO_PLUGIN_PATH", os.path.join(gazebo_ros_prefix, "lib")),
            OpaqueFunction(function=_launch_gzserver),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("gui")),
                launch_arguments={
                    "verbose": LaunchConfiguration("verbose"),
                    "extra_gazebo_args": LaunchConfiguration("extra_gazebo_args"),
                }.items(),
            ),
        ]
    )
