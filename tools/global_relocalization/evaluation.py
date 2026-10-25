"""Timestamp-aligned SE(3) evaluation helpers; no ROS or control output."""

import bisect
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def matrix(p):
    m = np.eye(4)
    m[:3, 3] = [p.position.x, p.position.y, p.position.z]
    m[:3, :3] = Rotation.from_quat(
        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    ).as_matrix()
    return m


def interpolate(rows, stamp, max_gap):
    stamps = [r[0] for r in rows]
    i = bisect.bisect_left(stamps, stamp)
    if i < len(rows) and rows[i][0] == stamp:
        return rows[i][1]
    if i == 0 or i == len(rows):
        return None
    ta, a = rows[i - 1]
    tb, b = rows[i]
    if tb - ta > max_gap:
        return None
    alpha = (stamp - ta) / (tb - ta)
    pose = np.eye(4)
    pose[:3, 3] = (1 - alpha) * a[:3, 3] + alpha * b[:3, 3]
    pose[:3, :3] = Slerp([0.0, 1.0], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))(
        [alpha]
    ).as_matrix()[0]
    return pose


def score_pose(estimate, truth):
    """Score complete SE(3), rejecting invalid transforms before thresholding."""
    for pose in (estimate, truth):
        pose = np.asarray(pose)
        if (
            pose.shape != (4, 4)
            or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1.0, atol=1e-6)
        ):
            return {
                "valid": False,
                "strict_correct": False,
                "reason": "invalid rigid transform",
            }
    distance = float(np.linalg.norm(estimate[:3, 3] - truth[:3, 3]))
    angle = float(
        np.degrees(Rotation.from_matrix(truth[:3, :3].T @ estimate[:3, :3]).magnitude())
    )
    return {
        "valid": True,
        "translation_3d_m": distance,
        "orientation_so3_deg": angle,
        "strict_correct": distance <= 0.1 and angle <= 2,
    }
