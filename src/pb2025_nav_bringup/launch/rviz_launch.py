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
import shlex
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_rviz(context, *args, **kwargs):
    del args, kwargs
    prefix = None
    rendering = "desktop OpenGL"
    hardware_acceleration = LaunchConfiguration("rviz_hardware_acceleration").perform(
        context
    ).strip().lower() in {"true", "1", "yes", "on"}
    if hardware_acceleration:
        vglrun = shutil.which("vglrun")
        if not vglrun:
            for candidate in (
                "/usr/NX/scripts/vgl/vglrun",
                "/opt/VirtualGL/bin/vglrun",
            ):
                if os.access(candidate, os.X_OK):
                    vglrun = candidate
                    break
        if vglrun:
            egl_device = (
                "egl" if os.path.realpath(vglrun).startswith("/usr/NX/") else "egl0"
            )
            prefix = shlex.join([vglrun, "-d", egl_device])
            rendering = "VirtualGL / EGL"
        else:
            rendering = "desktop OpenGL (VirtualGL not found)"

    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("namespace"),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        prefix=prefix,
        additional_env={"QT_QPA_PLATFORM": LaunchConfiguration("rviz_qt_platform")},
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
    return [
        LogInfo(msg=f"[pb2025_nav_bringup] RViz rendering: {rendering}"),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_rviz_cmd,
                on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
            ),
        ),
        start_rviz_cmd,
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description=(
            "Top-level namespace. Keep empty for the current single-robot setup; "
            "non-empty values replace the <robot_namespace> keyword in RViz configs."
        ),
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    declare_rviz_qt_platform_cmd = DeclareLaunchArgument(
        "rviz_qt_platform",
        default_value="xcb",
        description=(
            "Qt window-system backend for RViz. XCB keeps saved window placement "
            "predictable in GNOME Wayland and NoMachine sessions."
        ),
    )

    declare_rviz_hardware_acceleration_cmd = DeclareLaunchArgument(
        "rviz_hardware_acceleration",
        default_value="true",
        description=(
            "Use VirtualGL / EGL for RViz when available; otherwise use desktop OpenGL"
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_qt_platform_cmd)
    ld.add_action(declare_rviz_hardware_acceleration_cmd)

    ld.add_action(OpaqueFunction(function=_launch_rviz))

    return ld
