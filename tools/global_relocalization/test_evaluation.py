import math
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from evaluation import interpolate, matrix, score_pose, meets_pose_criteria


def pose(z=0, yaw=0, roll=0):
    value = np.eye(4)
    value[2, 3] = z
    value[:3, :3] = Rotation.from_euler("xyz", [roll, 0, yaw], degrees=True).as_matrix()
    return value


def test_interpolation_keeps_height_and_takes_short_rotation_arc():
    result = interpolate([(1, pose(0, 179)), (3, pose(2, -179))], 2, 3)
    assert result[2, 3] == 1
    assert math.isclose(Rotation.from_matrix(result[:3, :3]).magnitude(), math.pi)


def test_interpolation_requires_an_exact_sample_or_a_bounded_bracket():
    rows = [(1, pose()), (3, pose())]
    assert interpolate(rows, 1, 0.1) is rows[0][1]
    assert interpolate(rows, 2, 0.5) is None
    assert interpolate(rows, 0, 3) is None
    assert interpolate(rows, 4, 3) is None
    assert interpolate([], 1, 3) is None


def test_full_pose_score_rejects_height_and_roll_errors():
    assert not score_pose(pose(z=0.11), pose())["strict_correct"]
    assert not score_pose(pose(roll=3), pose())["strict_correct"]
    assert score_pose(pose(z=0.09, roll=1.9), pose())["strict_correct"]


def test_nonfinite_or_nonrigid_pose_never_scores_as_correct():
    for bad in (np.full((4, 4), np.nan), np.zeros((4, 4)), np.eye(3)):
        assert not score_pose(bad, pose())["valid"]


def test_sensor_map_criteria_keep_strict_result_and_full_orientation():
    score = score_pose(pose(z=0.14, roll=2.5), pose())
    assert not score["strict_correct"]
    assert score["sensor_map_correct"]
    assert meets_pose_criteria(score, "sensor_map")
    assert not meets_pose_criteria(score, "ideal")
    assert not meets_pose_criteria(score_pose(pose(roll=3.1), pose()), "sensor_map")
    assert not meets_pose_criteria({"valid": True}, "sensor_map")
    assert not meets_pose_criteria({"valid": True, "translation_3d_m": float("nan"),
                                    "orientation_so3_deg": 0}, "sensor_map")


def test_geometry_message_conversion_retains_xyz_and_xyzw_rotation():
    value = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.3), w=math.cos(0.3)),
    )
    result = matrix(value)
    np.testing.assert_allclose(result[:3, 3], [1, 2, 3])
    assert math.isclose(Rotation.from_matrix(result[:3, :3]).magnitude(), 0.6)


def test_yaw_error_wraps_at_180_and_xyz_error_remains_signed():
    estimate, truth = pose(yaw=-179), pose(yaw=179)
    estimate[:3, 3] = [0.03, -0.04, 0.12]
    score = score_pose(estimate, truth)
    assert math.isclose(score["yaw_error_deg"], 2)
    assert math.isclose(score["translation_xy_m"], 0.05)
    np.testing.assert_allclose(score["translation_error_xyz_m"], [0.03, -0.04, 0.12])
