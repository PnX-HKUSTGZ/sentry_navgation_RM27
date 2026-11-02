"""Final raw LiDAR/IMU competition server, with optional observer windows."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("global_relocalization")
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path"),
            DeclareLaunchArgument("manifest_path", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share, "config", "competition.yaml"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument(
                "visualization_enabled", default_value=LaunchConfiguration("rviz")
            ),
            Node(
                package="global_relocalization",
                executable="global_relocalization_node",
                name="global_relocalization",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "map_path": LaunchConfiguration("map_path"),
                        "manifest_path": LaunchConfiguration("manifest_path"),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "visualization_enabled": ParameterValue(
                            LaunchConfiguration("visualization_enabled"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="global_relocalization",
                executable="display_helper.py",
                condition=IfCondition(LaunchConfiguration("visualization_enabled")),
            ),
            *[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name=name,
                    arguments=["-d", os.path.join(share, "rviz", config)],
                    parameters=[
                        {
                            "use_sim_time": ParameterValue(
                                LaunchConfiguration("use_sim_time"), value_type=bool
                            )
                        }
                    ],
                    condition=IfCondition(LaunchConfiguration("rviz")),
                )
                for name, config in [
                    ("relocalization_map_view", "observer.rviz"),
                    ("relocalization_scan_view", "observer_reference.rviz"),
                ]
            ],
        ]
    )
