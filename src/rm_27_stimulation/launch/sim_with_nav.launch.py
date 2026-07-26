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

    scan_context_database_path = LaunchConfiguration("scan_context_database_path").perform(context)
    if scan_context_database_path in ("", "auto"):
        source_bringup_dir = os.path.abspath(
            os.path.join(bringup_share, "..", "..", "..", "..", "src", "pb2025_nav_bringup")
        )
        scan_context_database_path = os.path.join(
            source_bringup_dir, "scan_context", "simulation", f"{nav_world}.scdb"
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
                "verbose": LaunchConfiguration("verbose").perform(context),
                "pause": LaunchConfiguration("pause").perform(context),
                "physics_engine": LaunchConfiguration("physics_engine").perform(context),
                "extra_gazebo_args": LaunchConfiguration("extra_gazebo_args").perform(context),
                "use_ros_gz_bridge": LaunchConfiguration("use_ros_gz_bridge").perform(context),
                "bridge_config": LaunchConfiguration("bridge_config").perform(context),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "sim_world": world_name,
                "worlds_config": worlds_config_path,
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
                        "scan_context_database_path": scan_context_database_path,
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
                default_value="gz-physics-bullet-plugin",
                description="Gazebo physics engine plugin. Use an empty value for Gazebo default.",
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
                default_value=os.path.join(sim_share, "config", "ros_gz_bridge.yaml"),
                description="ros_gz_bridge YAML configuration file",
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
                "scan_context_database_path",
                default_value="auto",
                description=(
                    "Scan Context database path. Use auto to store under "
                    "src/pb2025_nav_bringup/scan_context/simulation/<nav_world>.scdb."
                ),
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
