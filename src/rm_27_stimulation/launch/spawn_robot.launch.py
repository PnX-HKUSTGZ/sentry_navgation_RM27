#!/usr/bin/env python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def _spawn_robot(context, *args, **kwargs):
    del args, kwargs

    config_path = LaunchConfiguration("spawn_poses_config").perform(context)
    world_name = LaunchConfiguration("sim_world").perform(context)
    entity_name = LaunchConfiguration("entity").perform(context)
    spawn_delay = float(LaunchConfiguration("spawn_delay").perform(context))
    spawn_timeout = LaunchConfiguration("spawn_timeout").perform(context)

    spawn_poses = _load_yaml(config_path).get("spawn_poses", {})
    if world_name not in spawn_poses:
        valid_worlds = ", ".join(sorted(spawn_poses.keys()))
        raise RuntimeError(f"Unknown spawn pose world '{world_name}'. Valid worlds: {valid_worlds}")

    pose = spawn_poses[world_name]
    spawn_arguments = [
        "-entity",
        entity_name,
        "-topic",
        "robot_description",
        "-timeout",
        spawn_timeout,
        "-x",
        str(pose.get("x", 0.0)),
        "-y",
        str(pose.get("y", 0.0)),
        "-z",
        str(pose.get("z", 0.0)),
        "-R",
        str(pose.get("roll", 0.0)),
        "-P",
        str(pose.get("pitch", 0.0)),
        "-Y",
        str(pose.get("yaw", 0.0)),
    ]

    return [
        TimerAction(
            period=spawn_delay,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    output="screen",
                    arguments=spawn_arguments,
                )
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("rm_27_stimulation")
    robot_description_file = LaunchConfiguration("robot_description_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_description_file]),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_world",
                default_value="RMUC2026",
                description="World key used to select the spawn pose",
            ),
            DeclareLaunchArgument(
                "spawn_poses_config",
                default_value=os.path.join(package_share, "config", "spawn_poses.yaml"),
                description="YAML file describing robot spawn poses",
            ),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=os.path.join(
                    package_share, "urdf", "simulation_waking_robot.xacro"
                ),
                description="Robot xacro file to spawn",
            ),
            DeclareLaunchArgument(
                "entity",
                default_value="rm27_sentry",
                description="Gazebo entity name",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time",
            ),
            DeclareLaunchArgument(
                "spawn_delay",
                default_value="2.0",
                description="Delay before spawning the robot, in seconds",
            ),
            DeclareLaunchArgument(
                "spawn_timeout",
                default_value="60.0",
                description="Timeout while waiting for Gazebo spawn service",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, robot_description],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, robot_description],
            ),
            OpaqueFunction(function=_spawn_robot),
        ]
    )
