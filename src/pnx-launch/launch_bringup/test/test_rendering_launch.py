import importlib.util
import json
from pathlib import Path
import sys

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

import rm27_rendering


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
SIMULATION_LAUNCH_FILE = (
    REPOSITORY_ROOT
    / "src"
    / "pnx-stimulation"
    / "rm_27_stimulation"
    / "launch"
    / "sim_with_nav.launch.py"
)
GAZEBO_LAUNCH_FILE = (
    REPOSITORY_ROOT
    / "src"
    / "pnx-stimulation"
    / "rm_27_stimulation"
    / "launch"
    / "rm_simulation.launch.py"
)
RVIZ_LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "rviz_launch.py"


def _load_launch(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _launch_argument_map(action):
    return dict(action.launch_arguments)


def test_simulation_entry_forwards_render_mode_to_gazebo_and_navigation(
    monkeypatch, tmp_path
):
    module = _load_launch(SIMULATION_LAUNCH_FILE, "rm27_sim_with_nav_rendering_test")
    sim_share = tmp_path / "rm_27_stimulation"
    bringup_share = tmp_path / "launch_bringup"
    sim_share.mkdir()
    bringup_share.mkdir()
    (tmp_path / "worlds.yaml").write_text("worlds: {}\n")
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda name: str(sim_share if name == "rm_27_stimulation" else bringup_share),
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "namespace": "",
            "slam": "false",
            "navigation_mode": "minco",
            "use_ground_truth_odom": "true",
            "world": "RMUC2027",
            "nav_world": "RMUC2027",
            "worlds_config": str(tmp_path / "worlds.yaml"),
            "spawn_delay": "6.0",
            "nav_start_delay": "8.0",
            "render_mode": "gpu",
            "hardware_acceleration": "false",
            "rviz_hardware_acceleration": "true",
            "gui": "false",
            "verbose": "false",
            "pause": "false",
            "physics_engine": "test-engine",
            "extra_gazebo_args": "",
            "gui_config": "",
            "use_ros_gz_bridge": "false",
            "bridge_config": "",
            "use_sim_time": "true",
            "params_file": "params.yaml",
            "autostart": "true",
            "use_composition": "false",
            "use_respawn": "false",
            "use_rviz": "true",
            "enable_legacy_terrain": "auto",
            "gimbal_panel": "false",
        }
    )

    monkeypatch.setattr(module, "global_settings", lambda *_args: {})
    actions = module._launch_sim_stack(context)

    gazebo_arguments = _launch_argument_map(actions[0])
    assert gazebo_arguments["render_mode"] == "gpu"
    assert gazebo_arguments["hardware_acceleration"] == "false"
    navigation_include = actions[2].actions[0]
    navigation_arguments = _launch_argument_map(navigation_include)
    assert navigation_arguments["render_mode"] == "gpu"
    assert navigation_arguments["rviz_hardware_acceleration"].perform(context) == "true"


def test_rviz_receives_full_child_environment_without_mutating_launch_parent(
    monkeypatch,
):
    module = _load_launch(RVIZ_LAUNCH_FILE, "rm27_rviz_rendering_test")
    context = LaunchContext()
    context.environment["GALLIUM_DRIVER"] = "parent-marker"
    context.environment["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = "NVIDIA"
    context.launch_configurations.update(
        {
            "render_mode": "cpu",
            "simulation_rendering": "true",
            "rviz_hardware_acceleration": "true",
            "rviz_qt_platform": "xcb",
            "namespace": "",
            "rviz_config": "rviz.rviz",
            "use_sim_time": "true",
        }
    )
    calls = []

    def select(mode, *, purpose, environ):
        calls.append((mode, purpose, dict(environ)))
        child = dict(environ)
        child["GALLIUM_DRIVER"] = "llvmpipe"
        child["LIBGL_ALWAYS_SOFTWARE"] = "1"
        child.pop("MESA_D3D12_DEFAULT_ADAPTER_NAME", None)
        return rm27_rendering.RenderingSelection(
            requested_mode=mode,
            mode="cpu",
            backend="glx",
            renderer="llvmpipe (test renderer)",
            environment=child,
            set_env={"GALLIUM_DRIVER": "llvmpipe", "LIBGL_ALWAYS_SOFTWARE": "1"},
            unset_env=("MESA_D3D12_DEFAULT_ADAPTER_NAME",),
        )

    monkeypatch.setattr(module, "select_rendering", select)

    actions = module._launch_rviz(context)
    rviz = next(action for action in actions if isinstance(action, Node))
    rviz_environment = {
        perform_substitutions(context, name): perform_substitutions(context, value)
        for name, value in rviz.env
    }

    assert calls[0][0:2] == ("cpu", "rviz")
    assert calls[0][2]["GALLIUM_DRIVER"] == "parent-marker"
    assert rviz_environment["GALLIUM_DRIVER"] == "llvmpipe"
    assert rviz_environment["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert "MESA_D3D12_DEFAULT_ADAPTER_NAME" not in rviz_environment
    assert context.environment["GALLIUM_DRIVER"] == "parent-marker"
    assert context.environment["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA"


def test_gazebo_environment_changes_are_scoped_to_its_child_process(
    monkeypatch, tmp_path
):
    module = _load_launch(GAZEBO_LAUNCH_FILE, "rm27_gazebo_rendering_test")
    sim_share = tmp_path / "rm_27_stimulation"
    gz_share = tmp_path / "ros_gz_sim"
    (sim_share / "config").mkdir(parents=True)
    (sim_share / "world").mkdir()
    (gz_share / "launch").mkdir(parents=True)
    (sim_share / "config" / "worlds.yaml").write_text(
        "worlds:\n  test_world:\n    path: test.world\n"
    )
    (sim_share / "world" / "test.world").write_text(
        "<sdf version='1.9'><world name='test_world'/></sdf>\n"
    )
    inside_record = tmp_path / "gazebo-child-env.json"
    outside_record = tmp_path / "sibling-env.json"
    child_code = (
        "import json, os, pathlib; pathlib.Path("
        + repr(str(inside_record))
        + ").write_text(json.dumps({'driver': os.environ.get('GALLIUM_DRIVER'), "
        "'software': os.environ.get('LIBGL_ALWAYS_SOFTWARE')}))"
    )
    fake_gz_launch = (
        "from launch import LaunchDescription\n"
        "from launch.actions import ExecuteProcess\n"
        "import sys\n"
        "def generate_launch_description():\n"
        f"    return LaunchDescription([ExecuteProcess(cmd=[sys.executable, '-c', {child_code!r}], output='screen')])\n"
    )
    (gz_share / "launch" / "gz_sim.launch.py").write_text(fake_gz_launch)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda name: str(sim_share if name == "rm_27_stimulation" else gz_share),
    )
    monkeypatch.setenv("GALLIUM_DRIVER", "parent-marker")
    monkeypatch.setenv("LIBGL_ALWAYS_SOFTWARE", "parent-marker")
    selection = rm27_rendering.RenderingSelection(
        requested_mode="cpu",
        mode="cpu",
        backend="glx",
        renderer="llvmpipe (test renderer)",
        environment={},
        set_env={"GALLIUM_DRIVER": "llvmpipe", "LIBGL_ALWAYS_SOFTWARE": "1"},
        unset_env=(),
    )
    monkeypatch.setattr(module, "select_rendering", lambda *_args, **_kwargs: selection)
    context = LaunchContext()
    context.environment["GALLIUM_DRIVER"] = "parent-marker"
    context.environment["LIBGL_ALWAYS_SOFTWARE"] = "parent-marker"
    context.launch_configurations.update(
        {
            "sim_world": "test_world",
            "worlds_config": str(sim_share / "config" / "worlds.yaml"),
            "gui": "false",
            "rendering_config": "unused.yaml",
            "render_mode": "cpu",
            "hardware_acceleration": "true",
            "verbose": "false",
            "pause": "true",
            "physics_engine": "test-engine",
            "extra_gazebo_args": "",
            "gui_config": "",
            "use_ros_gz_bridge": "false",
            "bridge_config": "",
        }
    )
    selection_calls = []

    def select(mode, *, gui, purpose, environ):
        selection_calls.append((mode, gui, purpose, dict(environ)))
        return selection

    monkeypatch.setattr(module, "select_rendering", select)
    actions = module._launch_gz_sim(context)
    assert any(isinstance(action, GroupAction) for action in actions)
    assert selection_calls[0][0:3] == ("cpu", False, "gazebo")
    assert selection_calls[0][3]["GALLIUM_DRIVER"] == "parent-marker"
    assert selection_calls[0][3]["LIBGL_ALWAYS_SOFTWARE"] == "parent-marker"
    outside_code = (
        "import json, os, pathlib; pathlib.Path("
        + repr(str(outside_record))
        + ").write_text(json.dumps({'driver': os.environ.get('GALLIUM_DRIVER'), "
        "'software': os.environ.get('LIBGL_ALWAYS_SOFTWARE')}))"
    )
    launch = LaunchDescription(
        [
            *actions,
            ExecuteProcess(cmd=[sys.executable, "-c", outside_code], output="screen"),
        ]
    )
    service = LaunchService(argv=[], noninteractive=True)
    service.include_launch_description(launch)

    assert service.run() == 0
    inside = json.loads(inside_record.read_text())
    outside = json.loads(outside_record.read_text())
    assert inside == {"driver": "llvmpipe", "software": "1"}
    assert outside == {"driver": "parent-marker", "software": "parent-marker"}


def _default_context(module, **overrides):
    from launch.actions import DeclareLaunchArgument

    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if (
            isinstance(action, DeclareLaunchArgument)
            and action.default_value is not None
        ):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value
            )
    context.launch_configurations.update(overrides)
    return context


def test_selected_minco_yaml_controls_both_renderers(monkeypatch, tmp_path):
    module = _load_launch(SIMULATION_LAUNCH_FILE, "simulation_rendering_yaml_test")
    profile = tmp_path / "minco_params.yaml"
    profile.write_text(
        "simulation_rendering:\n  ros__parameters:\n    render_mode: cpu\n"
    )
    monkeypatch.setattr(module, "global_settings", lambda *_args: {})
    context = _default_context(
        module, navigation_mode="minco", params_file=str(profile)
    )
    actions = module._launch_sim_stack(context)
    assert _launch_argument_map(actions[0])["render_mode"] == "cpu"
    assert _launch_argument_map(actions[2].actions[0])["render_mode"] == "cpu"
    # The legacy per-process override keeps the other renderer on the YAML policy.
    context.launch_configurations["hardware_acceleration"] = "true"
    actions = module._launch_sim_stack(context)
    assert _launch_argument_map(actions[0])["render_mode"] == "auto"
    assert _launch_argument_map(actions[2].actions[0])["render_mode"] == "cpu"


def test_navigation_selects_minco_yaml_before_forwarding_rviz_policy(tmp_path):
    module = _load_launch(
        RVIZ_LAUNCH_FILE.with_name("rm_navigation_simulation_launch.py"),
        "navigation_rendering_yaml_test",
    )
    context = _default_context(
        module, navigation_mode="minco", use_ground_truth_odom="true"
    )
    profile = tmp_path / "minco_params.yaml"
    profile.write_text(
        "simulation_rendering:\n  ros__parameters:\n    render_mode: cpu\n"
    )
    # Exercise launch action order and runtime substitutions; the resolved profile
    # can be a temporary world/input-profile YAML rather than the original file.
    import relocalization_manager_launch.config as config
    from unittest.mock import patch

    with patch.object(config, "select_params_file", return_value=str(profile)):
        for action in module.configure_global_relocalization_for_sim(context):
            action.execute(context)
    for action in module._validate_localization_inputs(context):
        action.execute(context)
    for action in module._configure_simulation_rendering(context):
        action.execute(context)
    include = next(
        action
        for action in module.generate_launch_description().entities
        if hasattr(action, "launch_arguments")
        and "simulation_rendering" in dict(action.launch_arguments)
    )
    arguments = dict(include.launch_arguments)
    assert arguments["render_mode"].perform(context) == "cpu"
    assert arguments["simulation_rendering"] == "true"


def test_standalone_gazebo_yaml_selects_egl_and_headless_flag(monkeypatch, tmp_path):
    import shlex

    module = _load_launch(GAZEBO_LAUNCH_FILE, "gazebo_rendering_yaml_test")
    profile = tmp_path / "rendering.yaml"
    profile.write_text(
        "simulation_rendering:\n  ros__parameters:\n    render_mode: gpu\n"
    )
    context = _default_context(module, rendering_config=str(profile), gui="false")
    calls = []

    def select(mode, **kwargs):
        calls.append((mode, kwargs))
        return rm27_rendering.RenderingSelection(
            mode, "gpu", "egl", "test GPU", {}, {}, ()
        )

    monkeypatch.setattr(module, "select_rendering", select)
    actions = module._launch_gz_sim(context)
    group = next(action for action in actions if isinstance(action, GroupAction))
    gazebo = next(
        action
        for action in group.get_sub_entities()
        if hasattr(action, "launch_arguments")
    )
    assert calls[0][0] == "gpu"
    assert calls[0][1]["gui"] is False
    arguments = shlex.split(dict(gazebo.launch_arguments)["gz_args"])
    assert "-s" in arguments
    assert arguments.count("--headless-rendering") == 1
    # WSLg GLX sensor rendering must never receive the EGL-only flag.
    monkeypatch.setattr(
        module,
        "select_rendering",
        lambda mode, **_kwargs: rm27_rendering.RenderingSelection(
            mode, "gpu", "glx", "D3D12 test", {}, {}, ()
        ),
    )
    context.launch_configurations["extra_gazebo_args"] = "--headless-rendering"
    import pytest

    with pytest.raises(RuntimeError, match="conflicts"):
        module._launch_gz_sim(context)


def test_real_robot_rviz_keeps_existing_desktop_policy(monkeypatch):
    module = _load_launch(RVIZ_LAUNCH_FILE, "rviz_real_robot_policy_test")
    context = _default_context(module, rviz_hardware_acceleration="false")

    def unexpected_probe(*_args, **_kwargs):
        raise AssertionError("real-robot default must not use simulation preflight")

    monkeypatch.setattr(module, "select_rendering", unexpected_probe)
    actions = module._launch_rviz(context)
    rviz = next(action for action in actions if isinstance(action, Node))
    assert rviz.env is None
