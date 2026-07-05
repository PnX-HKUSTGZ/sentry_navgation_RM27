import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_robot_description_file = os.path.join(
        get_package_share_directory("sentry_robot_description"),
        "urdf",
        "sentry_robot.urdf.xacro",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description_file = LaunchConfiguration("robot_description_file")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_robot_description_file_cmd = DeclareLaunchArgument(
        "robot_description_file",
        default_value=default_robot_description_file,
        description="Full path to the robot xacro file to publish",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": Command(["xacro ", robot_description_file]),
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_robot_description_file_cmd,
            robot_state_publisher,
        ]
    )
