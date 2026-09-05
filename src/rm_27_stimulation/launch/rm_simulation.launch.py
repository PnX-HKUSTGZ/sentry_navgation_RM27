#!/usr/bin/env python3

import os
import shlex
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _world_file_and_name(context):
    package_share = get_package_share_directory("rm_27_stimulation")
    config_path = LaunchConfiguration("worlds_config").perform(context)
    world_key = LaunchConfiguration("sim_world").perform(context)

    worlds = _load_yaml(config_path).get("worlds", {})
    if world_key not in worlds:
        valid_worlds = ", ".join(sorted(worlds.keys()))
        raise RuntimeError(f"Unknown world '{world_key}'. Valid worlds: {valid_worlds}")

    relative_path = worlds[world_key].get("path")
    if not relative_path:
        raise RuntimeError(f"World '{world_key}' does not define a 'path' field")

    world_path = os.path.join(package_share, "world", relative_path)
    if not os.path.exists(world_path):
        raise RuntimeError(f"World file does not exist: {world_path}")

    root = ET.parse(world_path).getroot()
    world_element = root.find("world")
    if world_element is None or not world_element.get("name"):
        raise RuntimeError(f"World file does not define a named <world>: {world_path}")

    return world_path, world_element.get("name")


def _launch_gz_sim(context, *args, **kwargs):
    del args, kwargs

    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )

    world_path, _ = _world_file_and_name(context)
    gz_args = []

    if _as_bool(LaunchConfiguration("verbose").perform(context)):
        gz_args.extend(["-v", "4"])

    if not _as_bool(LaunchConfiguration("pause").perform(context)):
        gz_args.append("-r")

    if not _as_bool(LaunchConfiguration("gui").perform(context)):
        gz_args.append("-s")
    else:
        gui_config = LaunchConfiguration("gui_config").perform(context).strip()
        if gui_config and os.path.isfile(gui_config):
            gz_args.extend(["--gui-config", gui_config])
        elif gui_config:
            print(
                f"[rm_27_stimulation] GUI config not found: {gui_config}; "
                "using the GUI section from the world SDF."
            )

    physics_engine = LaunchConfiguration("physics_engine").perform(context).strip()
    if physics_engine:
        gz_args.extend(["--physics-engine", physics_engine])

    extra_args = LaunchConfiguration("extra_gazebo_args").perform(context).strip()
    if extra_args:
        gz_args.extend(shlex.split(extra_args))

    gz_args.append(world_path)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                "gz_args": shlex.join(gz_args),
                "gz_version": "8",
                "on_exit_shutdown": "true",
            }.items(),
        )
    ]


def _launch_ros_gz_bridge(context, *args, **kwargs):
    del args, kwargs

    if not _as_bool(LaunchConfiguration("use_ros_gz_bridge").perform(context)):
        return []

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="rm_27_ros_gz_bridge",
            output="screen",
            parameters=[
                {"config_file": LaunchConfiguration("bridge_config").perform(context)}
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("rm_27_stimulation")
    package_prefix = get_package_prefix("rm_27_stimulation")

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
                description="Start Gazebo GUI when true",
            ),
            DeclareLaunchArgument(
                "gui_config",
                default_value=os.path.join(package_share, "config", "gazebo_gui.config"),
                description="Project-local Gazebo GUI configuration",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="false",
                description="Run Gazebo Harmonic with verbose output",
            ),
            DeclareLaunchArgument(
                "pause",
                default_value="false",
                description="Start Gazebo paused when true",
            ),
            DeclareLaunchArgument(
                "physics_engine",
                default_value="gz-physics-dartsim-plugin",
                description=(
                    "Gazebo physics engine plugin. DART is required by the planar "
                    "velocity controller to retain STL terrain contacts."
                ),
            ),
            DeclareLaunchArgument(
                "extra_gazebo_args",
                default_value="",
                description="Additional raw arguments passed to Gazebo Harmonic",
            ),
            DeclareLaunchArgument(
                "use_ros_gz_bridge",
                default_value="true",
                description="Start ros_gz_bridge for clock, cmd_vel, lidar, and IMU topics",
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=os.path.join(package_share, "config", "ros_gz_bridge.yaml"),
                description="ros_gz_bridge YAML configuration file",
            ),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", package_share),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(package_share, "meshes")),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(package_share, "world")),
            AppendEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.path.join(package_prefix, "lib")),
            OpaqueFunction(function=_launch_gz_sim),
            OpaqueFunction(function=_launch_ros_gz_bridge),
        ]
    )
