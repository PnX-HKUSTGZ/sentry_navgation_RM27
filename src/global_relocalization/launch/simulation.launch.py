"""Isolated Gazebo and robot; no navigation or ground-truth localizer."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = get_package_share_directory('rm_27_stimulation')
    world = LaunchConfiguration('world')
    spawn = LaunchConfiguration('spawn_poses_config')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='RMUC2026'),
        DeclareLaunchArgument('render_mode', default_value='auto'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('robot_description_file', default_value=os.path.join(sim, 'urdf', 'simulation_waking_robot.xacro')),
        DeclareLaunchArgument('physics_engine', default_value='gz-physics-bullet-featherstone-plugin'),
        DeclareLaunchArgument('spawn_poses_config', default_value=os.path.join(sim, 'config', 'spawn_poses.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'rm_simulation.launch.py')),
            launch_arguments={'sim_world': world, 'gui': LaunchConfiguration('gui'), 'render_mode': LaunchConfiguration('render_mode'), 'physics_engine': LaunchConfiguration('physics_engine'), 'verbose': LaunchConfiguration('verbose')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'spawn_robot.launch.py')),
            launch_arguments={'sim_world': world, 'spawn_poses_config': spawn, 'robot_description_file': LaunchConfiguration('robot_description_file'), 'spawn_delay': '3.0', 'use_sim_time': 'true'}.items()),
    ])
