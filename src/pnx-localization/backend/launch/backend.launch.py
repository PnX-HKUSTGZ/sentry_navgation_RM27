from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rm27_backend'), 'config', 'backend.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rm27_backend', executable='rm27_backend_node', name='rm27_backend',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen'),
    ])
