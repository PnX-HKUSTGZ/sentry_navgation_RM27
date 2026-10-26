#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_sim_stack(context, *args, **kwargs):
    del args, kwargs

    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    slam = _as_bool(LaunchConfiguration("slam").perform(context))
    navigation_mode = LaunchConfiguration("navigation_mode").perform(context).lower()
    use_ground_truth_odom = _as_bool(
        LaunchConfiguration("use_ground_truth_odom").perform(context)
    )
    if namespace:
        raise RuntimeError(
            "Non-empty namespace is not supported by the current single-robot "
            "navigation topic contract. Omit the namespace argument and use its "
            "empty default instead."
        )
    if slam and use_ground_truth_odom:
        raise RuntimeError(
            "slam:=true and use_ground_truth_odom:=true are mutually exclusive. "
            "Choose exactly one localization source."
        )
    if slam and navigation_mode != "legacy":
        raise RuntimeError(
            "slam:=true currently supports navigation_mode:=legacy only because "
            "MINCO requires the selected static map as its ROG prior map."
        )

    sim_share = get_package_share_directory("rm_27_stimulation")
    bringup_share = get_package_share_directory("pb2025_nav_bringup")
    worlds_config_path = LaunchConfiguration("worlds_config").perform(context)
    world_name = LaunchConfiguration("world").perform(context)
    nav_world = LaunchConfiguration("nav_world").perform(context)
    spawn_delay = float(LaunchConfiguration("spawn_delay").perform(context))
    requested_nav_start_delay = float(
        LaunchConfiguration("nav_start_delay").perform(context)
    )
    # Give ros_gz_sim create time to receive the robot description and finish insertion.
    nav_start_delay = max(requested_nav_start_delay, spawn_delay + 2.0)

    worlds = _load_yaml(worlds_config_path).get("worlds", {})
    if nav_world == "auto":
        if world_name not in worlds:
            valid_worlds = ", ".join(sorted(worlds.keys()))
            raise RuntimeError(
                f"Unknown world '{world_name}'. Valid worlds: {valid_worlds}"
            )
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

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "sim_world": world_name,
                "worlds_config": worlds_config_path,
                "gui": LaunchConfiguration("gui").perform(context),
                "hardware_acceleration": LaunchConfiguration(
                    "hardware_acceleration"
                ).perform(context),
                "verbose": LaunchConfiguration("verbose").perform(context),
                "pause": LaunchConfiguration("pause").perform(context),
                "physics_engine": LaunchConfiguration("physics_engine").perform(
                    context
                ),
                "extra_gazebo_args": LaunchConfiguration("extra_gazebo_args").perform(
                    context
                ),
                "gui_config": LaunchConfiguration("gui_config").perform(context),
                "use_ros_gz_bridge": LaunchConfiguration("use_ros_gz_bridge").perform(
                    context
                ),
                "bridge_config": LaunchConfiguration("bridge_config").perform(context),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                "sim_world": world_name,
                "worlds_config": worlds_config_path,
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                "spawn_delay": LaunchConfiguration("spawn_delay").perform(context),
                "gimbal_panel": LaunchConfiguration("gimbal_panel").perform(context),
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
                        "rviz_hardware_acceleration": LaunchConfiguration(
                            "rviz_hardware_acceleration"
                        ),
                        "use_ground_truth_odom": LaunchConfiguration(
                            "use_ground_truth_odom"
                        ),
                        "navigation_mode": LaunchConfiguration("navigation_mode"),
                        "enable_legacy_terrain": LaunchConfiguration(
                            "enable_legacy_terrain"
                        ),
                    }.items(),
                )
            ],
        ),
    ]

    if worlds.get(world_name, {}).get("wait_for_clock", False):
        # The create service exists before expensive collision initialization
        # finishes. Wait for a physics tick before inserting the robot.
        clock_ready = Node(package="rm_27_stimulation", executable="wait_for_sim_clock.py",
                           output="screen")

        def start_after_clock(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Gazebo physics clock unavailable"))]
            return [LogInfo(msg="[rm_27_stimulation] World ready; starting robot and navigation"),
                    TimerAction(period=1.0, actions=actions[1:])]

        return [actions[0],
                LogInfo(msg="[rm_27_stimulation] Loading field collisions; robot/navigation wait for physics"),
                RegisterEventHandler(OnProcessExit(target_action=clock_ready,
                                                   on_exit=start_after_clock)),
                clock_ready]
    return actions


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
            DeclareLaunchArgument("gimbal_panel", default_value="true",
                                  description="Open the two-button yaw/roll control window (disabled for headless runs)"),
            DeclareLaunchArgument(
                "hardware_acceleration",
                default_value="true",
                description="Use EGL without GUI, or VirtualGL / EGL for the GUI when available",
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
                default_value="gz-physics-bullet-featherstone-plugin",
                description=(
                    "Gazebo physics engine plugin. Bullet Featherstone supports the "
                    "RMUC2026 convex STL collision proxies and planar force controller."
                ),
            ),
            DeclareLaunchArgument(
                "extra_gazebo_args",
                default_value="",
                description="Additional raw arguments passed to Gazebo Harmonic",
            ),
            DeclareLaunchArgument(
                "gui_config",
                default_value="auto",
                description="Gazebo GUI configuration; auto selects the world-specific view",
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
                "rviz_hardware_acceleration",
                default_value="true",
                description="Use VirtualGL / EGL to accelerate RViz when available",
            ),
            DeclareLaunchArgument(
                "use_ground_truth_odom",
                default_value="true",
                description="Use Gazebo ground truth for simulation navigation localization",
            ),
            DeclareLaunchArgument(
                "navigation_mode",
                default_value="legacy",
                description="Select legacy or minco navigation",
            ),
            DeclareLaunchArgument(
                "enable_legacy_terrain",
                default_value="auto",
                description="Override legacy terrain nodes for the navigation mode",
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
                default_value="5.0",
                description="Delay before starting navigation, in seconds",
            ),
            DeclareLaunchArgument(
                "spawn_delay",
                default_value="3.0",
                description="Delay before spawning the robot, in seconds",
            ),
            OpaqueFunction(function=_launch_sim_stack),
        ]
    )
