import math
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from evaluation import interpolate, matrix, score_pose


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


def test_geometry_message_conversion_retains_xyz_and_xyzw_rotation():
    value = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.3), w=math.cos(0.3)),
    )
    result = matrix(value)
    np.testing.assert_allclose(result[:3, 3], [1, 2, 3])
    assert math.isclose(Rotation.from_matrix(result[:3, :3]).magnitude(), 0.6)
