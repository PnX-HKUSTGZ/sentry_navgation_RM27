"""One-shot global pose estimation; no pose submission to the legacy stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('global_relocalization')
    return LaunchDescription([
        DeclareLaunchArgument('map_path'),
        DeclareLaunchArgument('manifest_path', default_value=''),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'global.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lio.launch.py'))),
        Node(package='global_relocalization', executable='global_relocalization_node', name='global_relocalization', output='screen',
             parameters=[LaunchConfiguration('params_file'), {'map_path': LaunchConfiguration('map_path'), 'manifest_path': LaunchConfiguration('manifest_path'), 'use_sim_time': True}]),
    ])
