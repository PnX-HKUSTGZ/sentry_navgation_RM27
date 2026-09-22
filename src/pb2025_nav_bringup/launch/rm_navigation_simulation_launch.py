# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _as_bool(value):
    return value.strip().lower() in {"true", "1", "yes", "on"}


def _validate_localization_inputs(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    use_ground_truth = _as_bool(
        LaunchConfiguration("use_ground_truth_odom").perform(context)
    )
    use_slam = _as_bool(LaunchConfiguration("slam").perform(context))
    navigation_mode = LaunchConfiguration("navigation_mode").perform(context).lower()
    if namespace:
        raise RuntimeError(
            "Non-empty namespace is not supported by the current single-robot "
            "navigation topic contract. Omit the namespace argument and use its "
            "empty default instead."
        )
    if use_ground_truth and use_slam:
        raise RuntimeError(
            "slam:=true and use_ground_truth_odom:=true are mutually exclusive: "
            "both pipelines would publish the map->odom localization transform."
        )
    if navigation_mode not in {"legacy", "minco"}:
        raise RuntimeError("navigation_mode must be one of: legacy, minco")
    if use_slam and navigation_mode != "legacy":
        raise RuntimeError(
            "slam:=true currently supports navigation_mode:=legacy only because "
            "MINCO requires the selected static map as its ROG prior map."
        )
    params_file = LaunchConfiguration("params_file").perform(context)
    actions = []
    if navigation_mode == "minco":
        directory = os.path.join(
            get_package_share_directory("pb2025_nav_bringup"), "config", "simulation"
        )
        if os.path.realpath(params_file) == os.path.realpath(
            os.path.join(directory, "nav2_params.yaml")
        ):
            actions.append(
                SetLaunchConfiguration(
                    "params_file", os.path.join(directory, "minco_params.yaml")
                )
            )
    if use_ground_truth or use_slam:
        return actions

    prior_pcd = LaunchConfiguration("prior_pcd_file").perform(context)
    if not os.path.isfile(prior_pcd):
        world = LaunchConfiguration("world").perform(context)
        raise RuntimeError(
            f"Point-LIO localization for world '{world}' requires a matching prior "
            f"PCD, but '{prior_pcd}' does not exist. Supply prior_pcd_file:=... and "
            "a matching relocalization init_pose, or use_ground_truth_odom:=true."
        )
    return actions


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    cmd_vel_smoothed_topic = LaunchConfiguration("cmd_vel_smoothed_topic")
    use_ground_truth_odom = LaunchConfiguration("use_ground_truth_odom")
    navigation_mode = LaunchConfiguration("navigation_mode")
    enable_legacy_terrain = LaunchConfiguration("enable_legacy_terrain")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description=(
            "Top-level namespace. Keep empty for the current single-robot Gazebo "
            "simulation because Gazebo publishes sensor and cmd_vel topics globally."
        ),
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether to run SLAM instead of localization",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value="RMUC2026",
        description=(
            "Navigation map/PCD key. RMUC2026 is the default validated RM27 "
            "simulation target."
        ),
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "map", "simulation", "")),
            world,
            TextSubstitution(text=".yaml"),
        ],
        description="Full path to map file to load",
    )

    declare_prior_pcd_file_cmd = DeclareLaunchArgument(
        "prior_pcd_file",
        default_value=[
            TextSubstitution(text=os.path.join(bringup_dir, "pcd", "simulation", "")),
            world,
            TextSubstitution(text=".pcd"),
        ],
        description="Full path to prior pcd file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if True",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_cmd_vel_smoothed_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_smoothed_topic",
        default_value="cmd_vel_nav2_result",
        description=(
            "Output topic for nav2_velocity_smoother. fake_vel_transform converts "
            "this world-aligned command to the chassis-frame cmd_vel used by Gazebo."
        ),
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_ground_truth_odom_cmd = DeclareLaunchArgument(
        "use_ground_truth_odom",
        default_value="True",
        description=(
            "Use Gazebo ground truth for the simulation navigation feedback loop. "
            "Set False only when testing Point-LIO localization."
        ),
    )

    declare_navigation_mode_cmd = DeclareLaunchArgument(
        "navigation_mode",
        default_value="legacy",
        description="Select legacy or minco navigation",
    )

    declare_enable_legacy_terrain_cmd = DeclareLaunchArgument(
        "enable_legacy_terrain",
        default_value="auto",
        description="Override legacy terrain nodes for the selected navigation mode",
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="ign_sim_pointcloud_tool",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
        condition=UnlessCondition(use_ground_truth_odom),
    )

    start_ground_truth_localizer = Node(
        package="rm_27_stimulation",
        executable="rm27_ground_truth_localizer",
        name="rm27_ground_truth_localizer",
        output="screen",
        namespace=namespace,
        parameters=[configured_params],
        condition=IfCondition(use_ground_truth_odom),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "slam": slam,
            "map": map_yaml_file,
            "prior_pcd_file": prior_pcd_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "cmd_vel_smoothed_topic": cmd_vel_smoothed_topic,
            "use_ground_truth_odom": use_ground_truth_odom,
            "deployment": "simulation",
            "navigation_mode": navigation_mode,
            "enable_legacy_terrain": enable_legacy_terrain,
        }.items(),
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_prior_pcd_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_cmd_vel_smoothed_topic_cmd)
    ld.add_action(declare_use_ground_truth_odom_cmd)
    ld.add_action(declare_navigation_mode_cmd)
    ld.add_action(declare_enable_legacy_terrain_cmd)
    ld.add_action(OpaqueFunction(function=_validate_localization_inputs))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_velodyne_convert_tool)
    ld.add_action(start_ground_truth_localizer)
    ld.add_action(bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
