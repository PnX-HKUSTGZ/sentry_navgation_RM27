"""The navigation footprint must enclose the Gazebo collision geometry."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

SRC = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def geometry():
    robot = ET.fromstring(xacro.process_file(str(
        SRC / "rm_27_stimulation/urdf/simulation_waking_robot.xacro")).toxml())
    config = yaml.safe_load((SRC / "pb2025_nav_bringup/config/simulation/minco_params.yaml").read_text())
    safety = config["planner_server"]["ros__parameters"]["MincoPlanner"]["safety"]
    polygon = np.asarray(safety["footprint_points"]).reshape(-1, 2)
    edges = np.roll(polygon, -1, axis=0) - polygon
    normals = np.column_stack((edges[:, 1], -edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    bounds = np.sum(normals * polygon, axis=1)
    return robot, config, safety, polygon, normals, bounds


def box_corners(collision):
    center = np.fromstring(collision.find("origin").get("xyz"), sep=" ")
    half = np.fromstring(collision.find("geometry/box").get("size"), sep=" ") / 2
    return np.asarray([center + half * [x, y, z]
                       for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])


def test_nav2_and_minco_use_the_same_collision_envelope(geometry):
    _, config, safety, polygon, _, _ = geometry
    for node in ("local_costmap", "global_costmap"):
        actual = json.loads(config[node][node]["ros__parameters"]["footprint"])
        np.testing.assert_allclose(actual, polygon)
    assert safety["footprint_length"] == pytest.approx(np.ptp(polygon[:, 0]))
    assert safety["footprint_width"] == pytest.approx(np.ptp(polygon[:, 1]))


def test_all_wheel_spheres_fit_with_four_mm_margin(geometry):
    robot, _, _, _, normals, bounds = geometry
    for index in range(1, 5):
        origin = robot.find(f"./joint[@name='base_to_wheel{index}']/origin")
        center = np.fromstring(origin.get("xyz"), sep=" ")[:2]
        sphere = robot.find(f"./link[@name='wheel_{index}']/collision/geometry/sphere")
        radius = float(sphere.get("radius"))
        assert np.all(normals @ center + radius + 0.004 <= bounds + 1e-6), index


def test_folded_and_upright_spinning_gimbal_fit(geometry):
    robot, _, _, _, normals, bounds = geometry
    yaw_box = box_corners(robot.find("./link[@name='gimbal_yaw']/collision"))
    roll_box = box_corners(robot.find("./link[@name='gimbal_roll']/collision"))
    joint = robot.find("./joint[@name='gimbal_roll_joint']/origin")
    joint_position = np.fromstring(joint.get("xyz"), sep=" ")
    folded = roll_box + joint_position
    assert np.all(normals @ folded[:, :2].T <= bounds[:, None])
    # The interlock permits yaw spin only after a -pi/2 roll about X.
    rotation = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    upright = roll_box @ rotation.T + joint_position
    swept_radius = max(np.linalg.norm(yaw_box[:, :2], axis=1).max(),
                       np.linalg.norm(upright[:, :2], axis=1).max())
    assert np.all(swept_radius + 0.003 <= bounds)


def test_folded_clearance_height_encloses_the_physical_top(geometry):
    robot, config, _, _, _, _ = geometry
    yaw_box = box_corners(robot.find("./link[@name='gimbal_yaw']/collision"))
    roll_box = box_corners(robot.find("./link[@name='gimbal_roll']/collision"))
    joint = robot.find("./joint[@name='gimbal_roll_joint']/origin")
    roll_box += np.fromstring(joint.get("xyz"), sep=" ")
    wheel = robot.find("./link[@name='wheel_1']/collision/geometry/sphere")
    physical_height = max(yaw_box[:, 2].max(), roll_box[:, 2].max()) + float(wheel.get("radius"))
    projection = config["planner_server"]["ros__parameters"]["MincoPlanner"]["rog_map"]["projection"]
    assert projection["vehicle_height"] + 1e-6 >= physical_height
