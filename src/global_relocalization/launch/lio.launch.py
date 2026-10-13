"""LiDAR + IMU odometry only. Ground truth is never an estimator input."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('global_relocalization')
    params = LaunchConfiguration('lio_params')
    return LaunchDescription([
        DeclareLaunchArgument('lio_params', default_value=os.path.join(share, 'config', 'lio.yaml')),
        Node(package='global_relocalization', executable='instantaneous_cloud_converter.py', name='instantaneous_cloud_converter', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='point_lio', executable='pointlio_mapping', name='point_lio', parameters=[params], output='screen'),
        Node(package='loam_interface', executable='loam_interface_node', name='loam_interface', parameters=[params], output='screen'),
        Node(package='sensor_scan_generation', executable='sensor_scan_generation_node', name='sensor_scan_generation', parameters=[params], output='screen'),
    ])
