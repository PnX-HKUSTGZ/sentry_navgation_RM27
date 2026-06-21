#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _launch_sim_stack(context, *args, **kwargs):
    del args, kwargs

    sim_share = get_package_share_directory("rm_27_stimulation")
    bringup_share = get_package_share_directory("pb2025_nav_bringup")
    worlds_config_path = LaunchConfiguration("worlds_config").perform(context)
    world_name = LaunchConfiguration("world").perform(context)
    nav_world = LaunchConfiguration("nav_world").perform(context)
    nav_start_delay = float(LaunchConfiguration("nav_start_delay").perform(context))

    if nav_world == "auto":
        worlds = _load_yaml(worlds_config_path).get("worlds", {})
        if world_name not in worlds:
            valid_worlds = ", ".join(sorted(worlds.keys()))
            raise RuntimeError(f"Unknown world '{world_name}'. Valid worlds: {valid_worlds}")
        nav_world = worlds[world_name].get("nav_world", "")
        if not nav_world:
            raise RuntimeError(
                f"World '{world_name}' does not define nav_world. "
                "Pass nav_world:=<map_or_pcd_basename> explicitly "
                "or add nav_world to config/worlds.yaml."
            )

    nav_launch = os.path.join(
        bringup_share, "launch", "rm_navigation_simulation_launch.py"
    )
    sim_launch = os.path.join(sim_share, "launch", "rm_simulation.launch.py")
    spawn_launch = os.path.join(sim_share, "launch", "spawn_robot.launch.py")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "sim_world": world_name,
                "worlds_config": worlds_config_path,
                "gui": LaunchConfiguration("gui").perform(context),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "sim_world": world_name,
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
            }.items(),
        ),
        TimerAction(
            period=nav_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        "namespace": LaunchConfiguration("namespace"),
                        "slam": LaunchConfiguration("slam"),
                        "world": nav_world,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "params_file": LaunchConfiguration("params_file"),
                        "autostart": LaunchConfiguration("autostart"),
                        "use_composition": LaunchConfiguration("use_composition"),
                        "use_respawn": LaunchConfiguration("use_respawn"),
                        "use_rviz": LaunchConfiguration("use_rviz"),
                    }.items(),
                )
            ],
        )
    ]


def generate_launch_description():
    sim_share = get_package_share_directory("rm_27_stimulation")
    bringup_share = get_package_share_directory("pb2025_nav_bringup")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="RMUC2026",
                description="Simulation world key from config/worlds.yaml",
            ),
            DeclareLaunchArgument(
                "nav_world",
                default_value="auto",
                description="Navigation map key. Use auto to read nav_world from config/worlds.yaml.",
            ),
            DeclareLaunchArgument(
                "worlds_config",
                default_value=os.path.join(sim_share, "config", "worlds.yaml"),
                description="YAML file describing available Gazebo worlds",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Navigation namespace. Keep empty for the current single-robot Gazebo setup.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="False",
                description=(
                    "Run SLAM mode. Keep False for the default RMUC2026 navigation "
                    "simulation with small_gicp relocalization."
                ),
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start Gazebo GUI client when true",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz from pb2025_nav_bringup",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    bringup_share, "config", "simulation", "nav2_params.yaml"
                ),
                description="RM27 navigation parameter file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the Nav2 stack",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="Use composed Nav2 bringup",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Respawn navigation nodes when composition is disabled",
            ),
            DeclareLaunchArgument(
                "nav_start_delay",
                default_value="8.0",
                description="Delay before starting navigation, in seconds",
            ),
            OpaqueFunction(function=_launch_sim_stack),
        ]
    )
