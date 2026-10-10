"""Standalone RViz map editor. Online reference is an explicitly enabled snapshot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch(context):
    online = LaunchConfiguration('online_reference').perform(context).lower() == 'true'
    reference = LaunchConfiguration('reference_topic').perform(context)
    if online and not reference.startswith('/'):
        raise ValueError('reference_topic must be an absolute ROS topic name')
    remappings = []
    if online:
        remappings.append(('/map_edit/source', reference))
    return [Node(
        package='rviz2', executable='rviz2', name='map_edit_rviz', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('map_edit'),
                                      'rviz', 'default.rviz')],
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'map_file': ParameterValue(LaunchConfiguration('map_file'), value_type=str)}],
        remappings=remappings,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_file', default_value='',
                              description='Optional local map YAML to open on startup'),
        DeclareLaunchArgument('online_reference', default_value='false', choices=['true', 'false'],
                              description='Read the first online reference snapshot; never publish to it'),
        DeclareLaunchArgument('reference_topic', default_value='/map',
                              description='Read-only input when online_reference is true'),
        OpaqueFunction(function=_launch),
    ])
