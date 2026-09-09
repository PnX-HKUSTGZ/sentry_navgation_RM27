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

import importlib.util
import math
from pathlib import Path
import struct
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext


BRINGUP_DIR = Path(__file__).resolve().parents[1]
LAUNCH_FILE = BRINGUP_DIR / "launch" / "navigation_launch.py"
REALITY_LAUNCH_FILE = BRINGUP_DIR / "launch" / "rm_navigation_reality_launch.py"
SIMULATION_LAUNCH_FILE = BRINGUP_DIR / "launch" / "rm_navigation_simulation_launch.py"
RMUC2026_MAP = BRINGUP_DIR / "map" / "simulation" / "RMUC2026.yaml"
RMUC2026_ELEVATION_GENERATOR = BRINGUP_DIR / "tools" / "generate_rmuc2026_elevation.py"
RMUC2026_STL = (
    BRINGUP_DIR.parent
    / "rm_27_stimulation"
    / "meshes"
    / "RMUC2026_world"
    / "meshes"
    / "RMUC2026.stl"
)
RMUC2026_WORLD = (
    BRINGUP_DIR.parent
    / "rm_27_stimulation"
    / "world"
    / "RMUC2026_world"
    / "RMUC2026_world.world"
)
SIMULATION_ROBOT_XACRO = (
    BRINGUP_DIR.parent / "rm_27_stimulation" / "urdf" / "simulation_waking_robot.xacro"
)
HIGHBAY_MAP = BRINGUP_DIR / "map" / "reality" / "highbay.yaml"
TUNNEL_RAMP_MAP = BRINGUP_DIR / "map" / "simulation" / "tunnel_ramp_test.yaml"
TUNNEL_RAMP_WORLD = (
    BRINGUP_DIR.parent
    / "rm_27_stimulation"
    / "world"
    / "tunnel_ramp_test"
    / "tunnel_ramp_test.world"
)


def load_navigation_launch():
    spec = importlib.util.spec_from_file_location("rm27_navigation_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_launch(path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_rmuc2026_elevation_generator():
    return load_launch(RMUC2026_ELEVATION_GENERATOR, "rmuc2026_elevation_generator")


def make_context(deployment, mode, map_path=""):
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "namespace": "",
            "navigation_mode": mode,
            "deployment": deployment,
            "enable_legacy_terrain": "auto",
            "use_ground_truth_odom": "true",
            "params_file": str(
                BRINGUP_DIR / "config" / deployment / "nav2_params.yaml"
            ),
            "map": map_path,
            "cmd_vel_smoothed_topic": "cmd_vel_nav2_result",
        }
    )
    return context


def execute_configuration(module, context):
    actions = module._configure_navigation_mode(context, str(BRINGUP_DIR))
    for action in actions:
        action.execute(context)


def _sdf_vector(element, expected_size):
    assert element is not None and element.text is not None
    values = tuple(float(value) for value in element.text.split())
    assert len(values) == expected_size
    return values


def _sdf_box(world, model_name):
    model = world.find(f"./model[@name='{model_name}']")
    assert model is not None
    pose = _sdf_vector(model.find("pose"), 6)
    collision = model.find("./link/collision")
    assert collision is not None
    assert collision.find("pose") is None
    size = _sdf_vector(collision.find("./geometry/box/size"), 3)
    return pose, size


def _support_height(patch, x, y=0.0):
    reference_x, reference_y, reference_z = patch["reference"]
    slope_x, slope_y = patch["slope"]
    return reference_z + slope_x * (x - reference_x) + slope_y * (y - reference_y)


def _read_pgm_header_and_raster(path):
    with path.open("rb") as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            assert line
            if not line.lstrip().startswith(b"#"):
                tokens.extend(line.split())
        magic = tokens[0].decode("ascii")
        width, height, max_value = (int(value) for value in tokens[1:4])
        raster = stream.read()
    return magic, width, height, max_value, raster


@pytest.mark.parametrize(
    ("deployment", "expected_name"),
    [("simulation", "RMUC2026.yaml"), ("reality", "highbay.yaml")],
)
def test_empty_map_selects_matching_deployment_default(deployment, expected_name):
    module = load_navigation_launch()
    context = make_context(deployment, "minco")

    execute_configuration(module, context)

    selected = Path(context.launch_configurations["map"])
    assert selected.name == expected_name
    assert selected.parent.name == deployment


@pytest.mark.parametrize("map_path", [RMUC2026_MAP, HIGHBAY_MAP])
def test_flat_default_maps_provide_ground_elevation_support(map_path):
    map_config = yaml.safe_load(map_path.read_text())
    elevation = map_config["ground_elevation"]

    assert elevation["default_height"] == pytest.approx(0.0)
    assert elevation["patches"] == []


def test_rmuc2026_ground_elevation_grid_matches_occupancy_map():
    config = yaml.safe_load(RMUC2026_MAP.read_text())
    elevation = config["ground_elevation"]
    grid = elevation["grid"]
    occupancy_path = RMUC2026_MAP.parent / config["image"]
    elevation_path = RMUC2026_MAP.parent / grid["image"]

    occupancy = _read_pgm_header_and_raster(occupancy_path)
    height_map = _read_pgm_header_and_raster(elevation_path)
    assert occupancy[0] == "P5"
    assert height_map[0] == "P5"
    assert height_map[1:3] == occupancy[1:3] == (583, 300)
    assert height_map[3] == grid["no_data"] == 65535
    assert len(height_map[4]) == 2 * height_map[1] * height_map[2]

    encoded = struct.unpack(f">{height_map[1] * height_map[2]}H", height_map[4])
    supported = [value for value in encoded if value != grid["no_data"]]
    decoded = [grid["offset"] + grid["scale"] * value for value in supported]
    assert len(supported) == 113792
    assert min(decoded) == pytest.approx(0.0, abs=grid["scale"])
    assert max(decoded) == pytest.approx(0.310, abs=grid["scale"])

    def elevation_at(x, y):
        column = math.floor((x - config["origin"][0]) / config["resolution"])
        map_row = math.floor((y - config["origin"][1]) / config["resolution"])
        image_row = height_map[2] - 1 - map_row
        value = encoded[image_row * height_map[1] + column]
        assert value != grid["no_data"]
        return grid["offset"] + grid["scale"] * value

    # These cells are the second tunnel's ramp under an overlapping roof.
    # The correct support is the middle STL surface, not the roof or base.
    assert elevation_at(4.623, 7.035) == pytest.approx(0.198, abs=grid["scale"])
    assert elevation_at(4.667, 7.035) == pytest.approx(0.198, abs=grid["scale"])
    assert elevation_at(4.729, 7.047) == pytest.approx(0.209, abs=grid["scale"])


def test_rmuc2026_ground_elevation_grid_is_current_generator_output():
    generator = load_rmuc2026_elevation_generator()
    config = yaml.safe_load(RMUC2026_MAP.read_text())
    elevation_path = RMUC2026_MAP.parent / config["ground_elevation"]["grid"]["image"]
    args = SimpleNamespace(
        map_yaml=RMUC2026_MAP,
        map_pgm=RMUC2026_MAP.parent / config["image"],
        stl=RMUC2026_STL,
        world=RMUC2026_WORLD,
        max_slope=28.0,
        max_height=0.60,
        seed_tolerance=0.04,
        step_tolerance=0.005,
    )

    output, _, _, _ = generator.build(args)

    assert elevation_path.read_bytes() == output


def test_simulation_minco_global_seed_keeps_ramp_corner_clearance():
    config = yaml.safe_load(
        (BRINGUP_DIR / "config/simulation/minco_params.yaml").read_text()
    )
    planner = config["planner_server"]["ros__parameters"]["MincoPlanner"]
    inflation = config["global_costmap"]["global_costmap"]["ros__parameters"][
        "inflation_layer"
    ]
    smac = planner["smac_2d"]
    ground_edge = planner["priormap"]["ground_edge_avoidance"]
    projection = planner["rog_map"]["projection"]

    assert inflation["inflation_radius"] == pytest.approx(0.35)
    assert inflation["cost_scaling_factor"] == pytest.approx(5.0)
    assert smac["use_esdf_cost"] is False
    assert ground_edge["enable"] is True
    # The global seed may conservatively route around surveyed roughness that
    # the local body-clearance classifier can still call passable. It must
    # never be more permissive than the final local safety gate.
    assert 0.0 < ground_edge["max_step"] <= projection["max_ground_step"]
    assert 0.0 < ground_edge["max_slope_deg"] <= projection["max_ground_slope_deg"]
    assert ground_edge["max_step"] == pytest.approx(0.02)
    assert ground_edge["max_slope_deg"] == pytest.approx(20.0)
    footprint_corner_radius = math.hypot(
        0.5 * planner["safety"]["footprint_length"]
        + planner["safety"]["footprint_margin"],
        0.5 * planner["safety"]["footprint_width"]
        + planner["safety"]["footprint_margin"],
    )
    assert ground_edge["lethal_clearance_radius"] >= footprint_corner_radius + 0.05
    assert ground_edge["clearance_radius"] >= footprint_corner_radius + 0.05
    assert ground_edge["lethal_clearance_radius"] <= ground_edge["clearance_radius"]
    assert 0 < ground_edge["clearance_cost"] < 253


def test_rmuc2026_physics_supports_holonomic_ramp_contacts():
    world_root = ET.parse(RMUC2026_WORLD).getroot()
    physics = world_root.find("./world/physics[@name='default_physics']")

    assert physics is not None
    assert physics.get("default") == "1"
    assert physics.get("type") == "dart"
    assert physics.findtext("./dart/collision_detector") == "bullet"

    robot_root = ET.parse(SIMULATION_ROBOT_XACRO).getroot()
    for wheel_name in ("wheel_1", "wheel_2", "wheel_3", "wheel_4"):
        wheel = robot_root.find(f"./link[@name='{wheel_name}']")
        assert wheel is not None
        assert wheel.find("./collision/geometry/sphere") is not None
        assert wheel.find("./collision/geometry/cylinder") is None

    plugin = robot_root.find(
        "./gazebo/plugin[@name='rm_27_stimulation::CmdVelPoseControlSystem']"
    )
    assert plugin is not None
    assert float(plugin.findtext("stall_assist_command_threshold")) == pytest.approx(
        0.08
    )
    assert float(plugin.findtext("stall_assist_engage_velocity")) == pytest.approx(0.02)
    assert float(plugin.findtext("stall_assist_release_velocity")) == pytest.approx(
        0.10
    )
    assert float(plugin.findtext("max_stall_assist_force")) == pytest.approx(45.0)


def test_reality_global_seed_ignores_rolling_rog_frontier():
    config = yaml.safe_load(
        (BRINGUP_DIR / "config/reality/minco_params.yaml").read_text()
    )
    planner = config["planner_server"]["ros__parameters"]["MincoPlanner"]
    smac = planner["smac_2d"]
    ground_edge = planner["priormap"]["ground_edge_avoidance"]
    projection = planner["rog_map"]["projection"]

    assert smac["use_esdf_cost"] is False
    assert ground_edge["enable"] is True
    assert ground_edge["max_step"] == pytest.approx(projection["max_ground_step"])
    assert ground_edge["max_slope_deg"] == pytest.approx(
        projection["max_ground_slope_deg"]
    )
    footprint_corner_radius = math.hypot(
        0.5 * planner["safety"]["footprint_length"]
        + planner["safety"]["footprint_margin"],
        0.5 * planner["safety"]["footprint_width"]
        + planner["safety"]["footprint_margin"],
    )
    assert ground_edge["lethal_clearance_radius"] >= footprint_corner_radius + 0.05
    assert ground_edge["clearance_radius"] >= footprint_corner_radius + 0.05
    assert ground_edge["lethal_clearance_radius"] <= ground_edge["clearance_radius"]


def test_minco_rejects_missing_prior_map():
    module = load_navigation_launch()
    context = make_context("simulation", "minco", "/does/not/exist.yaml")

    with pytest.raises(RuntimeError, match="require map"):
        module._configure_navigation_mode(context, str(BRINGUP_DIR))


@pytest.mark.parametrize(
    ("mode", "expected_package", "expected_executable"),
    [
        ("legacy", "nav2_planner", "planner_server"),
        ("minco_shadow", "nav2_planner", "planner_server"),
        ("minco", "minco_planner", "planner_server_mt"),
    ],
)
def test_active_minco_selects_multithreaded_planner_server(
    mode, expected_package, expected_executable
):
    module = load_navigation_launch()
    context = make_context("simulation", mode)

    execute_configuration(module, context)

    assert (
        context.launch_configurations["selected_planner_server_package"]
        == expected_package
    )
    assert (
        context.launch_configurations["selected_planner_server_executable"]
        == expected_executable
    )


def test_legacy_does_not_require_prior_map_file():
    module = load_navigation_launch()
    context = make_context("simulation", "legacy", "/does/not/exist.yaml")

    execute_configuration(module, context)

    assert context.launch_configurations["start_minco_shadow"] == "false"
    assert context.launch_configurations["resolved_enable_legacy_terrain"] == "true"


def test_minco_profiles_keep_prior_map_placeholder():
    for deployment in ("simulation", "reality"):
        text = (BRINGUP_DIR / "config" / deployment / "minco_params.yaml").read_text()
        assert 'yaml_path: "<rog_prior_map_yaml>"' in text
        assert "enable: true" in text


def test_active_minco_profiles_expose_only_authoritative_plugins():
    for deployment in ("simulation", "reality"):
        profile = yaml.safe_load(
            (BRINGUP_DIR / "config" / deployment / "minco_params.yaml").read_text()
        )
        assert profile["planner_server"]["ros__parameters"]["planner_plugins"] == [
            "MincoPlanner"
        ]
        assert profile["controller_server"]["ros__parameters"][
            "controller_plugins"
        ] == ["MincoMpc"]


def test_active_minco_profiles_keep_physical_pose_and_bounded_reference_contracts():
    for deployment in ("simulation", "reality"):
        profile = yaml.safe_load(
            (BRINGUP_DIR / "config" / deployment / "minco_params.yaml").read_text()
        )
        legacy = yaml.safe_load(
            (BRINGUP_DIR / "config" / deployment / "nav2_params.yaml").read_text()
        )
        planner = profile["planner_server"]["ros__parameters"]["MincoPlanner"]
        controller = profile["controller_server"]["ros__parameters"]["MincoMpc"]
        nav2_goal_tolerance = legacy["controller_server"]["ros__parameters"][
            "general_goal_checker"
        ]["xy_goal_tolerance"]

        assert planner["frames"]["physical_base_frame"] == "base_link"
        assert planner["rog_map"]["cloud_filter"]["filter_mode"] == "transform_cloud"
        assert 0.0 < controller["reference_progress_max_lead_time"] <= 0.5
        assert 0.0 < planner["minco_optimizer"]["successful_replan_period"] <= 0.5
        assert planner["minco_optimizer"]["enable_yaw_opt"] is True
        assert (
            0.0
            <= controller["slope_slowdown_start_angle"]
            < controller["slope_full_slowdown_angle"]
        )
        assert 0.0 < controller["slope_speed_limit"] <= controller["max_planar_speed"]
        assert (
            0.0
            < planner["minco_optimizer"]["traj_goal_tolerance"]
            < nav2_goal_tolerance
        )


def test_simulated_no_return_rays_match_embedded_rog_range_and_resolution():
    simulation_nav = yaml.safe_load(
        (BRINGUP_DIR / "config" / "simulation" / "nav2_params.yaml").read_text()
    )
    simulation_minco = yaml.safe_load(
        (BRINGUP_DIR / "config" / "simulation" / "minco_params.yaml").read_text()
    )
    localizer = simulation_nav["rm27_ground_truth_localizer"]["ros__parameters"]
    rog = simulation_minco["planner_server"]["ros__parameters"]["MincoPlanner"][
        "rog_map"
    ]

    assert localizer["reconstruct_no_return_rays"] is True
    assert localizer["lidar_horizontal_samples"] == 360
    assert localizer["lidar_vertical_samples"] == 320
    assert localizer["rog_raycast_max_range"] == rog["raycasting"]["ray_range"][1]
    assert localizer["rog_map_resolution"] == rog["resolution"]
    assert localizer["no_return_ray_length"] >= (
        localizer["rog_raycast_max_range"] + 2.0 * localizer["rog_map_resolution"]
    )
    assert localizer["no_return_horizontal_stride"] == 1
    assert localizer["no_return_vertical_stride"] == 2
    assert localizer["cycle_no_return_stride_phase"] is True
    assert rog["map_size"] == [8.0, 8.0, 2.0]
    assert rog["esdf"]["local_update_box"] == rog["map_size"]
    assert rog["raycasting"]["local_update_box"] == rog["map_size"]

    reality_nav = yaml.safe_load(
        (BRINGUP_DIR / "config" / "reality" / "nav2_params.yaml").read_text()
    )
    assert "rm27_ground_truth_localizer" not in reality_nav


def test_simulation_minco_speed_limit_is_faster_but_stays_inside_mpc_envelope():
    simulation = yaml.safe_load(
        (BRINGUP_DIR / "config" / "simulation" / "minco_params.yaml").read_text()
    )
    reality = yaml.safe_load(
        (BRINGUP_DIR / "config" / "reality" / "minco_params.yaml").read_text()
    )

    sim_planner = simulation["planner_server"]["ros__parameters"]["MincoPlanner"]
    sim_optimizer = sim_planner["minco_optimizer"]
    sim_mpc = simulation["controller_server"]["ros__parameters"]["MincoMpc"]
    real_optimizer = reality["planner_server"]["ros__parameters"]["MincoPlanner"][
        "minco_optimizer"
    ]

    assert sim_optimizer["max_velocity"] == pytest.approx(1.0)
    assert real_optimizer["max_velocity"] == pytest.approx(0.5)
    assert sim_optimizer["max_velocity"] <= sim_mpc["vx_max"]
    assert sim_optimizer["max_velocity"] <= sim_mpc["vy_max"]
    assert sim_optimizer["max_velocity"] == pytest.approx(sim_mpc["max_planar_speed"])
    assert sim_mpc["slope_speed_limit"] < sim_mpc["max_planar_speed"]
    assert sim_optimizer["max_acceleration"] <= sim_mpc["ax_max"]
    assert sim_optimizer["max_acceleration"] <= sim_mpc["ay_max"]

    brake_distance = sim_optimizer["max_velocity"] ** 2 / (
        2.0 * sim_optimizer["max_acceleration"]
    )
    assert brake_distance < sim_optimizer["lookahead_dist"]
    sim_projection = sim_planner["rog_map"]["projection"]
    hard_footprint_length = (
        sim_planner["safety"]["footprint_length"]
        + 2.0 * sim_planner["safety"]["footprint_margin"]
    )
    required_forward_reach = 0.5 * hard_footprint_length + brake_distance
    assert (
        0.5 * sim_projection["near_field_prior_fill_length"]
        >= required_forward_reach - 0.05
    )
    assert (
        sim_planner["safety"]["map_timeout"]
        < sim_planner["rog_map"]["decay"]["keep_time"]
    )


def test_simulation_footprint_bootstrap_covers_one_rog_seed_step():
    simulation = yaml.safe_load(
        (BRINGUP_DIR / "config" / "simulation" / "minco_params.yaml").read_text()
    )
    reality = yaml.safe_load(
        (BRINGUP_DIR / "config" / "reality" / "minco_params.yaml").read_text()
    )
    sim_planner = simulation["planner_server"]["ros__parameters"]["MincoPlanner"]
    sim_projection = sim_planner["rog_map"]["projection"]
    sim_safety = sim_planner["safety"]
    seed_step = sim_planner["priormap"]["rog_boundary_sample_step"]

    required_length = (
        sim_safety["footprint_length"]
        + 2.0 * sim_safety["footprint_margin"]
        + 2.0 * seed_step
    )
    required_width = (
        sim_safety["footprint_width"]
        + 2.0 * sim_safety["footprint_margin"]
        + 2.0 * seed_step
    )
    assert sim_projection["robot_footprint_clear_length"] == pytest.approx(
        required_length
    )
    assert sim_projection["robot_footprint_clear_width"] == pytest.approx(
        required_width
    )

    real_projection = reality["planner_server"]["ros__parameters"]["MincoPlanner"][
        "rog_map"
    ]["projection"]
    assert real_projection["robot_footprint_clear_length"] == pytest.approx(0.40)
    assert real_projection["robot_footprint_clear_width"] == pytest.approx(0.30)


def test_active_minco_profiles_disable_unsafe_motion_entry_points():
    for deployment in ("simulation", "reality"):
        profile = yaml.safe_load(
            (BRINGUP_DIR / "config" / deployment / "minco_params.yaml").read_text()
        )
        assert profile["bt_navigator"]["ros__parameters"]["navigators"] == [
            "navigate_to_pose"
        ]
        assert profile["behavior_server"]["ros__parameters"]["behavior_plugins"] == [
            "wait"
        ]
        assert (
            profile["fake_vel_transform"]["ros__parameters"]["enable_cmd_spin"] is False
        )
        assert (
            profile["fake_vel_transform"]["ros__parameters"]["terrain_guard_enabled"]
            is False
        )

        legacy = yaml.safe_load(
            (BRINGUP_DIR / "config" / deployment / "nav2_params.yaml").read_text()
        )
        assert legacy["bt_navigator"]["ros__parameters"]["navigators"] == [
            "navigate_to_pose",
            "navigate_through_poses",
        ]
        assert set(
            legacy["behavior_server"]["ros__parameters"]["behavior_plugins"]
        ) >= {"spin", "backup", "drive_on_heading", "assisted_teleop", "wait"}
        assert (
            legacy["fake_vel_transform"]["ros__parameters"]["terrain_guard_enabled"]
            is True
        )


def test_legacy_costmaps_fail_stale_terrain_inputs_closed():
    expected_timeouts = {"simulation": 1.5, "reality": 0.5}
    for deployment, timeout in expected_timeouts.items():
        legacy = yaml.safe_load(
            (BRINGUP_DIR / "config" / deployment / "nav2_params.yaml").read_text()
        )
        local_source = legacy["local_costmap"]["local_costmap"]["ros__parameters"][
            "intensity_voxel_layer"
        ]["terrain_map"]
        global_source = legacy["global_costmap"]["global_costmap"]["ros__parameters"][
            "intensity_voxel_layer"
        ]["terrain_map_ext"]
        guard = legacy["fake_vel_transform"]["ros__parameters"]

        assert local_source["expected_update_rate"] == pytest.approx(timeout)
        assert global_source["expected_update_rate"] == pytest.approx(timeout)
        assert guard["terrain_timeout"] == pytest.approx(timeout)
        assert guard["terrain_map_topic"] == "terrain_map"


def test_shadow_sidecar_uses_multithreaded_planner_executor():
    launch_text = LAUNCH_FILE.read_text()
    shadow_start = launch_text.index("start_minco_shadow_planner_cmd = Node(")
    shadow_manager = launch_text.index(
        "start_minco_shadow_lifecycle_manager_cmd = Node(", shadow_start
    )
    shadow_node = launch_text[shadow_start:shadow_manager]

    assert 'package="minco_planner"' in shadow_node
    assert 'executable="planner_server_mt"' in shadow_node


def test_legacy_terrain_consumers_use_sensor_data_qos():
    terrain_sources = (
        BRINGUP_DIR.parent / "terrain_analysis" / "src" / "terrainAnalysis.cpp",
        BRINGUP_DIR.parent / "terrain_analysis_ext" / "src" / "terrainAnalysisExt.cpp",
    )
    for source in terrain_sources:
        text = source.read_text()
        assert '"registered_scan", rclcpp::SensorDataQoS().keep_last(1)' in text


def test_rviz_keeps_heavy_scan_optional_and_minco_outputs_visible():
    rviz = yaml.safe_load((BRINGUP_DIR / "rviz" / "nav2_default_view.rviz").read_text())
    displays = rviz["Visualization Manager"]["Displays"]
    rog_group = next(
        display for display in displays if display.get("Name") == "ROG Map"
    )
    registered = next(
        display
        for display in rog_group["Displays"]
        if display.get("Name") == "Registered Scan"
    )
    minco_path = next(
        display
        for display in rog_group["Displays"]
        if display.get("Name") == "MINCO Optimized Path"
    )
    occupied = next(
        display
        for display in rog_group["Displays"]
        if display.get("Name") == "Occupied Voxels"
    )

    assert registered["Enabled"] is False
    assert registered["Value"] is False
    assert registered["Topic"]["Reliability Policy"] == "Best Effort"
    assert registered["Topic"]["Value"] == "/registered_scan"
    assert occupied["Enabled"] is True
    assert occupied["Topic"]["Value"] == "/rog_map/occupied"
    assert minco_path["Topic"]["Value"] == "/opt_path_vis"


def test_minco_multi_goal_paths_are_not_silently_mirrored_or_executed():
    active_tree = (
        BRINGUP_DIR / "behavior_trees" / "navigate_through_poses_w_minco_replanning.xml"
    ).read_text()
    shadow_tree = (
        BRINGUP_DIR / "behavior_trees" / "navigate_through_poses_w_minco_shadow.xml"
    ).read_text()

    assert "<AlwaysFailure" in active_tree
    assert "ComputePathThroughPoses" not in active_tree
    assert "SendMincoShadowGoalsThroughPoses" not in shadow_tree
    assert 'planner_id="GridBased"' in shadow_tree


def test_reality_entry_rejects_slam_with_minco_before_starting_drivers():
    module = load_launch(REALITY_LAUNCH_FILE, "rm27_reality_launch")
    context = LaunchContext()
    context.launch_configurations.update(
        {"namespace": "", "slam": "true", "navigation_mode": "minco"}
    )

    with pytest.raises(RuntimeError, match="supports navigation_mode:=legacy only"):
        module._validate_launch_contract(context)


def test_simulation_entry_rejects_slam_with_minco_before_starting_localization():
    module = load_launch(SIMULATION_LAUNCH_FILE, "rm27_simulation_launch")
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "namespace": "",
            "slam": "true",
            "navigation_mode": "minco_shadow",
            "use_ground_truth_odom": "false",
            "prior_pcd_file": "/does/not/matter.pcd",
            "world": "test",
        }
    )

    with pytest.raises(RuntimeError, match="supports navigation_mode:=legacy only"):
        module._validate_localization_inputs(context)


def test_simulation_entry_rejects_nonempty_namespace_before_starting_localization():
    module = load_launch(SIMULATION_LAUNCH_FILE, "rm27_simulation_namespace_launch")
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "namespace": "robot1",
            "slam": "false",
            "navigation_mode": "legacy",
            "use_ground_truth_odom": "true",
            "prior_pcd_file": "/does/not/matter.pcd",
            "world": "test",
        }
    )

    with pytest.raises(RuntimeError, match="Non-empty namespace") as error:
        module._validate_localization_inputs(context)
    assert "Omit the namespace argument" in str(error.value)


def test_tunnel_ramp_collision_surface_and_support_yaml_are_continuous():
    sdf_root = ET.parse(TUNNEL_RAMP_WORLD).getroot()
    world = sdf_root.find("world")
    assert world is not None

    ramp_pose, ramp_size = _sdf_box(world, "short_ramp")
    platform_pose, platform_size = _sdf_box(world, "ramp_platform")
    ramp_x, ramp_y, ramp_z, ramp_roll, ramp_pitch, ramp_yaw = ramp_pose
    ramp_length, ramp_width, ramp_thickness = ramp_size
    platform_x, platform_y, platform_z, platform_roll, platform_pitch, platform_yaw = (
        platform_pose
    )
    platform_length, platform_width, platform_thickness = platform_size

    assert (ramp_roll, ramp_yaw) == pytest.approx((0.0, 0.0), abs=1.0e-12)
    assert (platform_roll, platform_pitch, platform_yaw) == pytest.approx(
        (0.0, 0.0, 0.0), abs=1.0e-12
    )
    assert ramp_pitch < 0.0

    def ramp_top_endpoint(local_x):
        local_z = 0.5 * ramp_thickness
        return (
            ramp_x + math.cos(ramp_pitch) * local_x + math.sin(ramp_pitch) * local_z,
            ramp_z - math.sin(ramp_pitch) * local_x + math.cos(ramp_pitch) * local_z,
        )

    ramp_start = ramp_top_endpoint(-0.5 * ramp_length)
    ramp_end = ramp_top_endpoint(0.5 * ramp_length)
    platform_start_x = platform_x - 0.5 * platform_length
    platform_end_x = platform_x + 0.5 * platform_length
    platform_top_z = platform_z + 0.5 * platform_thickness

    height_tolerance = 1.0e-3
    edge_tolerance = 1.0e-3
    assert ramp_start[1] == pytest.approx(0.0, abs=height_tolerance)
    assert ramp_end[1] == pytest.approx(platform_top_z, abs=height_tolerance)
    assert 0.0 <= platform_start_x - ramp_end[0] <= edge_tolerance

    map_config = yaml.safe_load(TUNNEL_RAMP_MAP.read_text())
    elevation = map_config["ground_elevation"]
    patches = {patch["name"]: patch for patch in elevation["patches"]}
    ramp_support = patches["ramp"]
    platform_support = patches["platform"]
    ramp_bounds = ramp_support["bounds"]
    platform_bounds = platform_support["bounds"]

    assert elevation["default_height"] == pytest.approx(0.0, abs=height_tolerance)
    assert ramp_bounds == pytest.approx(
        [
            ramp_start[0],
            ramp_y - 0.5 * ramp_width,
            ramp_end[0],
            ramp_y + 0.5 * ramp_width,
        ],
        abs=height_tolerance,
    )
    assert platform_bounds == pytest.approx(
        [
            platform_start_x,
            platform_y - 0.5 * platform_width,
            platform_end_x,
            platform_y + 0.5 * platform_width,
        ],
        abs=height_tolerance,
    )

    geometric_slope = (ramp_end[1] - ramp_start[1]) / (ramp_end[0] - ramp_start[0])
    assert ramp_support["slope"] == pytest.approx([geometric_slope, 0.0], abs=5.0e-4)
    assert _support_height(ramp_support, ramp_bounds[0]) == pytest.approx(
        ramp_start[1], abs=height_tolerance
    )
    assert _support_height(ramp_support, ramp_bounds[2]) == pytest.approx(
        ramp_end[1], abs=height_tolerance
    )
    assert _support_height(platform_support, platform_bounds[0]) == pytest.approx(
        platform_top_z, abs=height_tolerance
    )
    assert _support_height(ramp_support, ramp_bounds[2]) == pytest.approx(
        _support_height(platform_support, platform_bounds[0]),
        abs=height_tolerance,
    )
    assert 0.0 <= platform_bounds[0] - ramp_bounds[2] <= edge_tolerance
    assert platform_bounds[0] - ramp_bounds[2] == pytest.approx(
        platform_start_x - ramp_end[0], abs=height_tolerance
    )
