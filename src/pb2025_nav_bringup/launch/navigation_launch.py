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
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def _configure_navigation_mode(context, bringup_dir):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    navigation_mode = LaunchConfiguration("navigation_mode").perform(context).lower()
    deployment = LaunchConfiguration("deployment").perform(context).lower()
    legacy_terrain = (
        LaunchConfiguration("enable_legacy_terrain").perform(context).lower()
    )
    use_ground_truth_odom_value = (
        LaunchConfiguration("use_ground_truth_odom").perform(context).lower()
    )
    base_params_file = LaunchConfiguration("params_file").perform(context)
    prior_map_yaml_file = LaunchConfiguration("map").perform(context).strip()

    if namespace:
        raise RuntimeError(
            "This RM27 navigation stack currently requires the namespace launch "
            "argument to keep its empty default. Its "
            "sensor, localization, and chassis topics use the "
            "single-robot global contract; a partial namespace would be unsafe."
        )

    if navigation_mode not in {"legacy", "minco"}:
        raise RuntimeError("navigation_mode must be one of: legacy, minco")
    if deployment not in {"reality", "simulation"}:
        raise RuntimeError("deployment must be either reality or simulation")
    if not prior_map_yaml_file:
        default_map_name = (
            "RMUC2026.yaml" if deployment == "simulation" else "highbay.yaml"
        )
        prior_map_yaml_file = os.path.join(
            bringup_dir, "map", deployment, default_map_name
        )
    if legacy_terrain not in {"auto", "true", "false"}:
        raise RuntimeError("enable_legacy_terrain must be auto, true, or false")
    if use_ground_truth_odom_value not in {
        "true",
        "false",
        "1",
        "0",
        "yes",
        "no",
        "on",
        "off",
    }:
        raise RuntimeError("use_ground_truth_odom must be a boolean value")
    use_ground_truth_odom = use_ground_truth_odom_value in {
        "true",
        "1",
        "yes",
        "on",
    }

    selected_params_file = base_params_file
    if navigation_mode == "minco":
        if not prior_map_yaml_file or not os.path.isfile(prior_map_yaml_file):
            raise RuntimeError(
                "MINCO requires map:=<existing occupancy-map YAML> for "
                f"ROG prior-map fusion; got '{prior_map_yaml_file}'"
            )
        default_base = os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        )
        deployment_base = os.path.join(
            bringup_dir, "config", deployment, "nav2_params.yaml"
        )
        if os.path.realpath(base_params_file) in {
            os.path.realpath(default_base),
            os.path.realpath(deployment_base),
        }:
            selected_params_file = os.path.join(
                bringup_dir, "config", deployment, "minco_params.yaml"
            )
        if not os.path.isfile(selected_params_file):
            raise RuntimeError(f"MINCO profile does not exist: {selected_params_file}")

        profile = yaml.safe_load(Path(selected_params_file).read_text())
        if not isinstance(profile, dict) or (
            profile.get("planner_server", {})
            .get("ros__parameters", {})
            .get("planner_plugins")
            != ["MincoPlanner"]
            or profile.get("controller_server", {})
            .get("ros__parameters", {})
            .get("controller_plugins")
            != ["MincoMpc"]
        ):
            raise RuntimeError(
                "MINCO params_file must be a complete single-file profile"
            )

        if deployment == "simulation" and not use_ground_truth_odom:
            # The optional Point-LIO inputs live in the same simulation YAML.
            # Materialize one resolved parameter file; never stack a second
            # ROS parameter overlay onto the navigation processes.
            try:
                point_lio = profile["minco_input_profiles"]["ros__parameters"][
                    "point_lio"
                ]
            except (KeyError, TypeError) as exc:
                raise RuntimeError(
                    "simulation MINCO profile is missing minco_input_profiles.point_lio"
                ) from exc
            planner = "planner_server.ros__parameters.MincoPlanner"
            rewrites = {
                f"{planner}.frames.rog_frame": point_lio["frames"]["rog_frame"],
                f"{planner}.odom_topic": point_lio["odom_topic"],
                f"{planner}.lidar_offset_x": point_lio["lidar_offset_x"],
                f"{planner}.lidar_offset_y": point_lio["lidar_offset_y"],
                f"{planner}.rog_map.frame_id": point_lio["rog_map"]["frame_id"],
                f"{planner}.rog_map.ros_callback.cloud_topic": point_lio["rog_map"][
                    "ros_callback"
                ]["cloud_topic"],
                f"{planner}.rog_map.ros_callback.odom_topic": point_lio["rog_map"][
                    "ros_callback"
                ]["odom_topic"],
                f"{planner}.rog_map.visualization.frame_id": point_lio["rog_map"][
                    "visualization"
                ]["frame_id"],
            }
            selected_params_file = RewrittenYaml(
                source_file=selected_params_file,
                param_rewrites={key: str(value) for key, value in rewrites.items()},
                convert_types=True,
            ).perform(context)

    if legacy_terrain == "auto":
        legacy_terrain = "false" if navigation_mode == "minco" else "true"

    command_output_topic = LaunchConfiguration("cmd_vel_smoothed_topic").perform(
        context
    )
    planner_server_package = (
        "minco_planner" if navigation_mode == "minco" else "nav2_planner"
    )
    planner_server_executable = (
        "planner_server_mt" if navigation_mode == "minco" else "planner_server"
    )

    return [
        SetLaunchConfiguration("map", prior_map_yaml_file),
        SetLaunchConfiguration("selected_navigation_params_file", selected_params_file),
        SetLaunchConfiguration("resolved_enable_legacy_terrain", legacy_terrain),
        SetLaunchConfiguration("navigation_cmd_vel_output", command_output_topic),
        SetLaunchConfiguration(
            "selected_planner_server_package", planner_server_package
        ),
        SetLaunchConfiguration(
            "selected_planner_server_executable", planner_server_executable
        ),
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml_file = LaunchConfiguration("map")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_ground_truth_odom = LaunchConfiguration("use_ground_truth_odom")
    enable_legacy_terrain = LaunchConfiguration("resolved_enable_legacy_terrain")
    navigation_cmd_vel_output = LaunchConfiguration("navigation_cmd_vel_output")
    planner_server_package = LaunchConfiguration("selected_planner_server_package")
    planner_server_executable = LaunchConfiguration(
        "selected_planner_server_executable"
    )

    def resolve_single_robot_topics(source_file):
        return ReplaceString(
            source_file=source_file,
            replacements={
                "<robot_namespace>": "",
                "<rog_prior_map_yaml>": map_yaml_file,
            },
        )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
        "input_cmd_vel_topic": navigation_cmd_vel_output,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=resolve_single_robot_topics(
                LaunchConfiguration("selected_navigation_params_file")
            ),
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    navigation_parameters = [configured_params]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace. Keep empty for the current single-robot setup.",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "simulation", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value="",
        description=(
            "Occupancy-map YAML used by ROG prior-map fusion in MINCO modes. "
            "Empty selects the deployment-specific default."
        ),
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="False",
        description="Use composed bringup if True",
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    declare_cmd_vel_smoothed_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_smoothed_topic",
        default_value="cmd_vel_nav2_result",
        description="Output topic for nav2_velocity_smoother",
    )

    declare_use_ground_truth_odom_cmd = DeclareLaunchArgument(
        "use_ground_truth_odom",
        default_value="False",
        description="Use simulation ground-truth odometry and registered scans",
    )

    declare_deployment_cmd = DeclareLaunchArgument(
        "deployment",
        default_value="simulation",
        description="Select reality or simulation MINCO topic/frame profile",
    )

    declare_navigation_mode_cmd = DeclareLaunchArgument(
        "navigation_mode",
        default_value="legacy",
        description="Select legacy or minco navigation",
    )

    declare_enable_legacy_terrain_cmd = DeclareLaunchArgument(
        "enable_legacy_terrain",
        default_value="auto",
        description=(
            "Start terrain_analysis and terrain_analysis_ext. auto enables them for "
            "legacy and disables them for active minco."
        ),
    )

    configure_navigation_mode_cmd = OpaqueFunction(
        function=_configure_navigation_mode,
        args=[bringup_dir],
    )

    start_terrain_analysis_cmd = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        condition=IfCondition(enable_legacy_terrain),
    )

    start_terrain_analysis_ext_cmd = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        condition=IfCondition(enable_legacy_terrain),
    )

    load_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package="loam_interface",
                executable="loam_interface_node",
                name="loam_interface",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                condition=UnlessCondition(use_ground_truth_odom),
            ),
            Node(
                package="sensor_scan_generation",
                executable="sensor_scan_generation_node",
                name="sensor_scan_generation",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                condition=UnlessCondition(use_ground_truth_odom),
            ),
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                name="fake_vel_transform",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package=planner_server_package,
                executable=planner_server_executable,
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", navigation_cmd_vel_output),
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", navigation_cmd_vel_output),
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=navigation_parameters,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", navigation_cmd_vel_output),
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ],
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package="loam_interface",
                plugin="loam_interface::LoamInterfaceNode",
                name="loam_interface",
                parameters=[configured_params],
                condition=UnlessCondition(use_ground_truth_odom),
            ),
            ComposableNode(
                package="sensor_scan_generation",
                plugin="sensor_scan_generation::SensorScanGenerationNode",
                name="sensor_scan_generation",
                parameters=[configured_params],
                condition=UnlessCondition(use_ground_truth_odom),
            ),
            ComposableNode(
                package="fake_vel_transform",
                plugin="fake_vel_transform::FakeVelTransform",
                name="fake_vel_transform",
                parameters=navigation_parameters,
            ),
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=navigation_parameters,
                remappings=[("cmd_vel", "cmd_vel_controller")],
            ),
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=navigation_parameters,
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=navigation_parameters,
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=navigation_parameters,
                remappings=[
                    ("cmd_vel", navigation_cmd_vel_output),
                ],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=navigation_parameters,
                remappings=[("cmd_vel", navigation_cmd_vel_output)],
            ),
            ComposableNode(
                package="nav2_waypoint_follower",
                plugin="nav2_waypoint_follower::WaypointFollower",
                name="waypoint_follower",
                parameters=navigation_parameters,
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=navigation_parameters,
                remappings=[
                    ("cmd_vel", "cmd_vel_controller"),  # remap input
                    ("cmd_vel_smoothed", navigation_cmd_vel_output),
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_cmd_vel_smoothed_topic_cmd)
    ld.add_action(declare_use_ground_truth_odom_cmd)
    ld.add_action(declare_deployment_cmd)
    ld.add_action(declare_navigation_mode_cmd)
    ld.add_action(declare_enable_legacy_terrain_cmd)
    ld.add_action(configure_navigation_mode_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_terrain_analysis_cmd)
    ld.add_action(start_terrain_analysis_ext_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
