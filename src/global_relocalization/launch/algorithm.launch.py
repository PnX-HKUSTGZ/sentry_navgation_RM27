"""Algorithm process only, permitting map/index reuse between independent LIO trials."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=get_package_share_directory('global_relocalization')
    return LaunchDescription([
        DeclareLaunchArgument('map_path'),
        DeclareLaunchArgument('manifest_path', default_value=''),
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','global.yaml')),
        Node(package='global_relocalization',executable='global_relocalization_node',name='global_relocalization',output='screen',
            parameters=[LaunchConfiguration('params_file'),{'use_sim_time':True,'map_path':LaunchConfiguration('map_path'),'manifest_path':LaunchConfiguration('manifest_path')}]),
    ])
