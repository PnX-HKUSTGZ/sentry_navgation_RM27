#!/usr/bin/env python3
"""RMUL2027 world and optional robot, without navigation/map dependencies."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rm_27_stimulation')
    clock_ready = Node(package='rm_27_stimulation',executable='wait_for_sim_clock.py',
                       condition=IfCondition(LaunchConfiguration('spawn_robot')),output='screen')
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'sim_world':'RMUL2027',
                          'gimbal_panel':LaunchConfiguration('gimbal_panel'),
                          'spawn_delay':'0.2','use_sim_time':'true'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('hardware_acceleration', default_value='true'),
        DeclareLaunchArgument('spawn_robot', default_value='true'),
        DeclareLaunchArgument('gimbal_panel', default_value='true'),
        DeclareLaunchArgument('pause', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rm_simulation.launch.py')),
            launch_arguments={
                'sim_world': 'RMUL2027',
                'gui': LaunchConfiguration('gui'),
                'hardware_acceleration': LaunchConfiguration('hardware_acceleration'),
                'pause': LaunchConfiguration('pause'),
                'verbose': LaunchConfiguration('verbose'),
                'gui_config': os.path.join(share, 'config', 'rmul2027_gui.config'),
            }.items()),
        RegisterEventHandler(OnProcessExit(target_action=clock_ready,
            on_exit=lambda event, context: [TimerAction(period=1.0,actions=[spawn])]
            if event.returncode == 0 else [EmitEvent(event=Shutdown(
                reason='World clock unavailable; robot was not spawned'))])),
        clock_ready,
    ])
