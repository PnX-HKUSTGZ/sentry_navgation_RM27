#!/usr/bin/env python3
"""Score global-relocalization pose outputs against timestamped Gazebo truth.

The mathematical helpers in this file intentionally have no ROS dependencies,
so the transform, interpolation, and error calculations can be unit tested with
the system Python.  The optional ROS 2 node subscribes only to the estimate and
ground-truth topics; it never publishes transforms or feeds truth to the
localizer.
"""

from __future__ import annotations

import bisect
import argparse
import csv
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass
from typing import Any, Optional, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w
CORRECT_LOCALIZATION_DEADLINE_SEC = 15.0


def _finite_tuple(values: Sequence[float], size: int, name: str) -> tuple[float, ...]:
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def normalize_quaternion(q: Sequence[float]) -> Quaternion:
    """Return a normalized xyzw quaternion; reject zero and non-finite inputs."""
    values = _finite_tuple(q, 4, "quaternion")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError("quaternion norm is too small")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def quaternion_conjugate(q: Sequence[float]) -> Quaternion:
    x, y, z, w = normalize_quaternion(q)
    return (-x, -y, -z, w)


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> Quaternion:
    """Hamilton product for xyzw quaternions, normalized on return."""
    ax, ay, az, aw = normalize_quaternion(left)
    bx, by, bz, bw = normalize_quaternion(right)
    return normalize_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def quaternion_rotate(q: Sequence[float], vector: Sequence[float]) -> Vector3:
    """Rotate a 3-vector by a unit quaternion without constructing matrices."""
    x, y, z, w = normalize_quaternion(q)
    vx, vy, vz = _finite_tuple(vector, 3, "vector")
    # q * [v, 0] * conjugate(q), expanded for fewer roundoff operations.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_slerp(q0: Sequence[float], q1: Sequence[float], fraction: float) -> Quaternion:
    """Shortest-arc spherical interpolation, including equivalent +/- q pairs."""
    if not math.isfinite(fraction) or fraction < 0.0 or fraction > 1.0:
        raise ValueError("fraction must be finite and within [0, 1]")
    a = normalize_quaternion(q0)
    b = normalize_quaternion(q1)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-component for component in b)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(tuple(
            (1.0 - fraction) * x + fraction * y for x, y in zip(a, b)
        ))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    weight_a = math.sin((1.0 - fraction) * theta) / sin_theta
    weight_b = math.sin(fraction * theta) / sin_theta
    return normalize_quaternion(tuple(
        weight_a * x + weight_b * y for x, y in zip(a, b)
    ))


def quaternion_geodesic_angle(q_est: Sequence[float], q_truth: Sequence[float]) -> float:
    """Return the SO(3) geodesic distance in radians (range [0, pi])."""
    a = normalize_quaternion(q_est)
    b = normalize_quaternion(q_truth)
    absolute_dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.acos(min(1.0, max(0.0, absolute_dot)))


def wrap_angle(angle: float) -> float:
    """Wrap to [-pi, pi), making the +pi boundary deterministic."""
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    # Avoid returning negative zero in machine-readable reports.
    return 0.0 if wrapped == 0.0 else wrapped


def quaternion_to_rpy(q: Sequence[float]) -> tuple[float, float, float]:
    """Convert xyzw quaternion to roll, pitch, yaw (radians)."""
    x, y, z, w = normalize_quaternion(q)
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return (wrap_angle(roll), wrap_angle(pitch), wrap_angle(yaw))


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Create an xyzw quaternion using the ROS fixed-axis RPY convention."""
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("roll, pitch, and yaw must be finite")
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


@dataclass(frozen=True)
class Pose:
    position: Vector3
    orientation: Quaternion

    @classmethod
    def make(cls, position: Sequence[float], orientation: Sequence[float]) -> "Pose":
        return cls(
            _finite_tuple(position, 3, "position"),
            normalize_quaternion(orientation),
        )


@dataclass(frozen=True)
class TimedPose:
    stamp_ns: int
    pose: Pose


@dataclass(frozen=True)
class LookupResult:
    pose: Optional[Pose]
    validity: str
    alpha: Optional[float] = None
    lower_stamp_ns: Optional[int] = None
    upper_stamp_ns: Optional[int] = None


def compose_pose(parent_from_mid: Pose, mid_from_child: Pose) -> Pose:
    """Compose T_parent_mid * T_mid_child."""
    rotated = quaternion_rotate(parent_from_mid.orientation, mid_from_child.position)
    position = tuple(a + b for a, b in zip(parent_from_mid.position, rotated))
    orientation = quaternion_multiply(parent_from_mid.orientation, mid_from_child.orientation)
    return Pose.make(position, orientation)


def rotation_matrix_to_quaternion(matrix: Sequence[Sequence[float]]) -> Quaternion:
    """Convert a checked 3x3 proper rotation matrix to an xyzw quaternion."""
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("rotation matrix must be 3x3")
    r = [[float(value) for value in row] for row in matrix]
    if not all(math.isfinite(value) for row in r for value in row):
        raise ValueError("rotation matrix must contain finite values")
    for i in range(3):
        for j in range(3):
            dot = sum(r[k][i] * r[k][j] for k in range(3))
            if not math.isclose(dot, 1.0 if i == j else 0.0, abs_tol=1e-6, rel_tol=0.0):
                raise ValueError("transform rotation is not orthonormal")
    determinant = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )
    if not math.isclose(determinant, 1.0, abs_tol=1e-6, rel_tol=0.0):
        raise ValueError("transform rotation must have determinant +1")
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = (
            (r[2][1] - r[1][2]) / scale,
            (r[0][2] - r[2][0]) / scale,
            (r[1][0] - r[0][1]) / scale,
            0.25 * scale,
        )
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        scale = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        q = (
            0.25 * scale,
            (r[0][1] + r[1][0]) / scale,
            (r[0][2] + r[2][0]) / scale,
            (r[2][1] - r[1][2]) / scale,
        )
    elif r[1][1] > r[2][2]:
        scale = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        q = (
            (r[0][1] + r[1][0]) / scale,
            0.25 * scale,
            (r[1][2] + r[2][1]) / scale,
            (r[0][2] - r[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        q = (
            (r[0][2] + r[2][0]) / scale,
            (r[1][2] + r[2][1]) / scale,
            0.25 * scale,
            (r[1][0] - r[0][1]) / scale,
        )
    return normalize_quaternion(q)


def pose_from_homogeneous(matrix: Sequence[Sequence[float]]) -> Pose:
    """Parse a 4x4 row-major rigid transform matrix."""
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("homogeneous transform must be 4x4")
    values = [[float(value) for value in row] for row in matrix]
    if not all(math.isfinite(value) for row in values for value in row):
        raise ValueError("homogeneous transform must contain finite values")
    if any(not math.isclose(values[3][i], target, abs_tol=1e-9, rel_tol=0.0)
           for i, target in enumerate((0.0, 0.0, 0.0, 1.0))):
        raise ValueError("homogeneous transform bottom row must be [0, 0, 0, 1]")
    orientation = rotation_matrix_to_quaternion([row[:3] for row in values[:3]])
    return Pose.make([values[0][3], values[1][3], values[2][3]], orientation)


def transforms_match(a: Pose, b: Pose, translation_tolerance_m: float = 1e-6,
                     rotation_tolerance_rad: float = 1e-6) -> bool:
    translation_error = math.sqrt(sum((x - y) ** 2 for x, y in zip(a.position, b.position)))
    return (translation_error <= translation_tolerance_m and
            quaternion_geodesic_angle(a.orientation, b.orientation) <= rotation_tolerance_rad)


def load_alignment_manifest(path: str, map_frame: str, world_frame: str,
                            base_frame: str) -> tuple[Pose, dict[str, Any]]:
    """Load and validate the mapping manifest's authoritative T_map_world."""
    with open(path, "r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    checks = {
        "frame_id": map_frame,
        "pose_parent_frame": world_frame,
        "pose_child_frame": base_frame,
    }
    for key, expected in checks.items():
        actual = manifest.get(key)
        if actual != expected:
            raise ValueError(f"manifest {key}={actual!r}; expected {expected!r}")
    if "T_map_world" not in manifest:
        raise ValueError("manifest is missing authoritative T_map_world")
    pose = pose_from_homogeneous(manifest["T_map_world"])
    declared_hash = manifest.get("map_sha256")
    if not isinstance(declared_hash, str) or not declared_hash:
        raise ValueError("manifest is missing map_sha256")
    map_path = os.path.join(os.path.dirname(os.path.abspath(path)), "map.pcd")
    if not os.path.isfile(map_path):
        raise ValueError(f"manifest map file is missing: {map_path}")
    actual_digest = hashlib.sha256()
    with open(map_path, "rb") as map_stream:
        for chunk in iter(lambda: map_stream.read(1024 * 1024), b""):
            actual_digest.update(chunk)
    actual_hash = actual_digest.hexdigest()
    if actual_hash != declared_hash:
        raise ValueError(
            f"manifest map_sha256 {declared_hash!r} does not match {map_path} ({actual_hash})"
        )
    metadata = {
        "source": "map_manifest",
        "verification": "manifest_transform_used; geometric map validity is not proven by this check",
        "manifest_path": os.path.abspath(path),
        "map_sha256": declared_hash,
        "map_path": map_path,
        "map_frame": map_frame,
        "world_frame": world_frame,
        "base_frame": base_frame,
    }
    return pose, metadata


def interpolate_pose(a: Pose, b: Pose, fraction: float) -> Pose:
    if not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction must be within [0, 1]")
    position = tuple((1.0 - fraction) * x + fraction * y for x, y in zip(a.position, b.position))
    return Pose.make(position, quaternion_slerp(a.orientation, b.orientation, fraction))


class GroundTruthBuffer:
    """Timestamp-ordered GT samples with strict bracketed interpolation only."""

    def __init__(self, max_samples: int = 10000, max_gap_sec: float = 0.25) -> None:
        if max_samples < 2:
            raise ValueError("max_samples must be at least 2")
        if not math.isfinite(max_gap_sec) or max_gap_sec <= 0.0:
            raise ValueError("max_gap_sec must be positive and finite")
        self._max_samples = max_samples
        self.max_gap_ns = int(round(max_gap_sec * 1_000_000_000))
        self._samples: list[TimedPose] = []
        self._stamps: list[int] = []

    def __len__(self) -> int:
        return len(self._samples)

    def insert(self, sample: TimedPose) -> None:
        if sample.stamp_ns < 0:
            raise ValueError("stamp_ns must be non-negative")
        index = bisect.bisect_left(self._stamps, sample.stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == sample.stamp_ns:
            self._samples[index] = sample
            return
        self._stamps.insert(index, sample.stamp_ns)
        self._samples.insert(index, sample)
        overflow = len(self._samples) - self._max_samples
        if overflow > 0:
            del self._samples[:overflow]
            del self._stamps[:overflow]

    def lookup(self, stamp_ns: int) -> LookupResult:
        if stamp_ns < 0:
            return LookupResult(None, "invalid_query_stamp")
        if not self._samples:
            return LookupResult(None, "no_ground_truth_samples")
        index = bisect.bisect_left(self._stamps, stamp_ns)
        if index < len(self._stamps) and self._stamps[index] == stamp_ns:
            sample = self._samples[index]
            return LookupResult(sample.pose, "exact", 0.0, stamp_ns, stamp_ns)
        if index == 0:
            return LookupResult(None, "would_extrapolate_before_first_sample")
        if index >= len(self._stamps):
            return LookupResult(None, "need_future_bracket")
        lower, upper = self._samples[index - 1], self._samples[index]
        gap = upper.stamp_ns - lower.stamp_ns
        if gap <= 0:
            return LookupResult(None, "invalid_ground_truth_time_order")
        if gap > self.max_gap_ns:
            return LookupResult(None, "interpolation_gap_too_large", None, lower.stamp_ns, upper.stamp_ns)
        fraction = (stamp_ns - lower.stamp_ns) / gap
        return LookupResult(
            interpolate_pose(lower.pose, upper.pose, fraction),
            "interpolated",
            fraction,
            lower.stamp_ns,
            upper.stamp_ns,
        )


def calculate_pose_error(estimate_map: Pose, truth_world: Pose, map_from_world: Pose) -> dict[str, float]:
    """Compare map estimate with GT transformed by the explicit T_map_world."""
    truth_map = compose_pose(map_from_world, truth_world)
    dx, dy, dz = tuple(a - b for a, b in zip(estimate_map.position, truth_map.position))
    estimate_rpy = quaternion_to_rpy(estimate_map.orientation)
    truth_rpy = quaternion_to_rpy(truth_map.orientation)
    roll_error = wrap_angle(estimate_rpy[0] - truth_rpy[0])
    pitch_error = wrap_angle(estimate_rpy[1] - truth_rpy[1])
    yaw_error = wrap_angle(estimate_rpy[2] - truth_rpy[2])
    return {
        "dx_m": dx,
        "dy_m": dy,
        "dz_m": dz,
        "position_xy_error_m": math.hypot(dx, dy),
        "position_3d_error_m": math.sqrt(dx * dx + dy * dy + dz * dz),
        "roll_error_rad": roll_error,
        "roll_error_deg": math.degrees(roll_error),
        "pitch_error_rad": pitch_error,
        "pitch_error_deg": math.degrees(pitch_error),
        "yaw_error_rad": yaw_error,
        "yaw_error_deg": math.degrees(yaw_error),
        "orientation_geodesic_error_rad": quaternion_geodesic_angle(
            estimate_map.orientation, truth_map.orientation
        ),
        "orientation_geodesic_error_deg": math.degrees(quaternion_geodesic_angle(
            estimate_map.orientation, truth_map.orientation
        )),
    }


def make_sample_record(
    *,
    sample_index: int,
    pose_stamp_ns: int,
    pose_frame: str,
    interpolation: LookupResult,
    estimate_map: Optional[Pose],
    map_from_world: Pose,
    position_threshold_m: float,
    orientation_threshold_deg: float,
    base_frame: str = "base_footprint",
    failure_reason: Optional[str] = None,
    alignment_metadata: Optional[dict[str, Any]] = None,
    run_id: Optional[str] = None,
    repetition: Optional[int] = None,
    timing_source: str = "unavailable: timing is not present in the two subscribed topics",
) -> dict[str, Any]:
    record: dict[str, Any] = {
        "sample_index": sample_index,
        "pose_stamp_ns": pose_stamp_ns,
        "pose_stamp_sec": pose_stamp_ns / 1_000_000_000.0,
        "pose_frame": pose_frame,
        "estimate_child_frame": base_frame,
        "ground_truth_interpolation_validity": interpolation.validity,
        "ground_truth_interpolation_alpha": interpolation.alpha,
        "ground_truth_lower_stamp_ns": interpolation.lower_stamp_ns,
        "ground_truth_upper_stamp_ns": interpolation.upper_stamp_ns,
        "evaluation_valid": False,
        "success": False,
        "outcome": "failure",
        "failure_reason": failure_reason,
        "position_threshold_m": position_threshold_m,
        "orientation_threshold_deg": orientation_threshold_deg,
        "sweep_duration_sec": None,
        "computation_duration_sec": None,
        "timing_source": timing_source,
        "T_map_world_translation_m": list(map_from_world.position),
        "T_map_world_quaternion_xyzw": list(map_from_world.orientation),
        "alignment_metadata": alignment_metadata or {
            "source": "explicit_parameters",
            "verification": "unverified_alignment_without_map_manifest",
        },
    }
    if run_id is not None or repetition is not None:
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError("pose score repetition requires a non-empty run_id")
        if isinstance(repetition, bool) or not isinstance(repetition, int) or repetition <= 0:
            raise ValueError("pose score run_id requires a positive integer repetition")
        record["run_id"] = run_id
        record["repetition"] = repetition
    if failure_reason is not None:
        record["failure_reason"] = failure_reason
        record["outcome"] = "failure"
        record["success"] = False
        return record
    if interpolation.pose is None:
        record["failure_reason"] = interpolation.validity
        record["evaluation_valid"] = False
        record["success"] = False
        record["outcome"] = "failure"
        return record
    if estimate_map is None:
        record["failure_reason"] = "invalid_estimate_pose"
        record["outcome"] = "failure"
        record["success"] = False
        return record
    error = calculate_pose_error(estimate_map, interpolation.pose, map_from_world)
    record.update(error)
    within_position = error["position_3d_error_m"] <= position_threshold_m
    within_orientation = error["orientation_geodesic_error_deg"] <= orientation_threshold_deg
    passed = within_position and within_orientation
    record.update({
        "evaluation_valid": True,
        "success": passed,
        "outcome": "success" if passed else "failure",
        "failure_reason": None if passed else ";".join(
            reason for reason, condition in (
                ("position_3d_threshold_exceeded", within_position),
                ("orientation_threshold_exceeded", within_orientation),
            ) if not condition
        ),
    })
    return record


CSV_FIELDS = [
    "run_id", "repetition", "sample_index", "pose_stamp_ns", "pose_stamp_sec", "pose_frame", "estimate_child_frame",
    "ground_truth_interpolation_validity", "ground_truth_interpolation_alpha",
    "ground_truth_lower_stamp_ns", "ground_truth_upper_stamp_ns", "evaluation_valid",
    "success", "outcome", "failure_reason", "dx_m", "dy_m", "dz_m",
    "position_xy_error_m", "position_3d_error_m", "roll_error_rad", "roll_error_deg",
    "pitch_error_rad", "pitch_error_deg", "yaw_error_rad", "yaw_error_deg",
    "orientation_geodesic_error_rad", "orientation_geodesic_error_deg",
    "position_threshold_m", "orientation_threshold_deg", "sweep_duration_sec",
    "computation_duration_sec", "timing_source", "T_map_world_translation_m",
    "T_map_world_quaternion_xyzw", "alignment_metadata",
]


class ResultWriter:
    """Append one machine-readable JSONL row and one CSV row per estimate."""

    def __init__(self, prefix: str) -> None:
        if not prefix:
            raise ValueError("output_prefix must not be empty")
        parent = os.path.dirname(os.path.abspath(prefix))
        os.makedirs(parent, exist_ok=True)
        self.jsonl = open(prefix + ".jsonl", "a", encoding="utf-8")
        self.csv_file = open(prefix + ".csv", "a", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS, extrasaction="ignore")
        if self.csv_file.tell() == 0:
            self.csv_writer.writeheader()
            self.csv_file.flush()

    @staticmethod
    def _csv_value(value: Any) -> Any:
        return json.dumps(value, separators=(",", ":")) if isinstance(value, (dict, list)) else value

    def write(self, record: dict[str, Any]) -> None:
        self.jsonl.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        self.jsonl.flush()
        self.csv_writer.writerow({key: self._csv_value(record.get(key)) for key in CSV_FIELDS})
        self.csv_file.flush()

    def close(self) -> None:
        self.jsonl.close()
        self.csv_file.close()


def _read_jsonl(path: str) -> list[dict[str, Any]]:
    records = []
    with open(path, "r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            record = json.loads(line)
            if not isinstance(record, dict):
                raise ValueError(f"{path}:{line_number}: JSONL entry must be an object")
            records.append(record)
    return records


def _percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _distribution(values: Sequence[Optional[float]]) -> dict[str, Optional[float]]:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    return {
        "count": len(finite),
        "mean": sum(finite) / len(finite) if finite else None,
        "p50": _percentile(finite, 0.50),
        "p95": _percentile(finite, 0.95),
        "max": max(finite) if finite else None,
    }


def _error_distribution(values: Sequence[Optional[float]]) -> dict[str, Optional[float]]:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    stats = _distribution(finite)
    stats["rmse"] = math.sqrt(sum(value * value for value in finite) / len(finite)) if finite else None
    return stats


def _bias_and_rmse(values: Sequence[Optional[float]]) -> dict[str, Optional[float]]:
    finite = [float(value) for value in values if value is not None and math.isfinite(float(value))]
    if not finite:
        return {"count": 0, "bias": None, "rmse": None}
    return {
        "count": len(finite),
        "bias": sum(finite) / len(finite),
        "rmse": math.sqrt(sum(value * value for value in finite) / len(finite)),
    }


def _optional_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _nested(record: dict[str, Any], *path: str) -> Any:
    value: Any = record
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def timestamp_timeout_elapsed(query_stamp_ns: int, current_ros_stamp_ns: int,
                              timeout_sec: float) -> bool:
    """Expire a pending bracket wait against ROS time, never callback arrival time."""
    if query_stamp_ns < 0 or current_ros_stamp_ns < 0:
        return False
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive and finite")
    return current_ros_stamp_ns >= query_stamp_ns + int(timeout_sec * 1_000_000_000)


_ERROR_FIELDS = (
    "dx_m", "dy_m", "dz_m", "position_xy_error_m", "position_3d_error_m",
    "roll_error_deg", "pitch_error_deg", "yaw_error_deg",
    "orientation_geodesic_error_deg",
)
_TIMING_STAGES = (
    "static", "sweep", "retrieval", "gicp", "confirmation", "stop_settle",
    "search_total", "capture_total", "computation_total", "total",
)
_TIMING_COMPOSITES = {
    "capture_total": ("static", "sweep", "stop_settle"),
    "computation_total": ("retrieval", "gicp", "confirmation"),
}
_RESOURCE_METRICS = (
    "cpu_user_delta_sec", "cpu_sys_delta_sec", "wall_sec", "cpu_percent",
    "core_seconds", "peak_rss_bytes",
)


def _accuracy_metrics(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    metrics: dict[str, Any] = {"valid_pose_count": len(rows)}
    for field in ("position_xy_error_m", "position_3d_error_m", "orientation_geodesic_error_deg"):
        metrics[field] = _error_distribution([_optional_float(row.get(field)) for row in rows])
    metrics["per_axis_bias_rmse"] = {}
    metrics["per_axis_absolute_error"] = {}
    for axis in ("dx_m", "dy_m", "dz_m", "roll_error_deg", "pitch_error_deg", "yaw_error_deg"):
        values = [_optional_float(row.get(axis)) for row in rows]
        metrics["per_axis_bias_rmse"][axis] = _bias_and_rmse(values)
        metrics["per_axis_absolute_error"][axis] = _distribution([
            abs(value) if value is not None else None for value in values
        ])
    distances = [
        value for value in (_optional_float(row.get("position_3d_error_m")) for row in rows)
        if value is not None
    ]
    metrics["overall_position_rmse_m"] = (
        math.sqrt(sum(value * value for value in distances) / len(distances)) if distances else None
    )
    return metrics


def _trial_pose_stamp(trial: dict[str, Any]) -> Optional[int]:
    action = trial.get("action")
    if not isinstance(action, dict) or action.get("success") is not True or action.get("timeout") is True:
        return None
    candidate = action.get("pose_stamp_ns")
    if not isinstance(candidate, int) or isinstance(candidate, bool):
        return None
    return candidate if candidate >= 0 else None


def _cohort_identity(record: dict[str, Any], description: str) -> Optional[tuple[str, int]]:
    """Read optional round identity; partial or malformed identities are unsafe to join."""
    run_id = record.get("run_id")
    repetition = record.get("repetition")
    if run_id is None and repetition is None:
        return None
    if not isinstance(run_id, str) or not run_id.strip():
        raise ValueError(f"{description} has repetition without a non-empty run_id")
    if isinstance(repetition, bool) or not isinstance(repetition, int) or repetition <= 0:
        raise ValueError(f"{description} has run_id without a positive integer repetition")
    return run_id, repetition


def _action_elapsed_sec(trial: dict[str, Any], terminal_status: dict[str, Any]) -> Optional[float]:
    action = trial.get("action")
    value = action.get("elapsed_sec") if isinstance(action, dict) else None
    if value is None:
        value = terminal_status.get("elapsed_sec")
    return _optional_float(value)


def _registration_attempt_outcome(raw_frontend: Any) -> Optional[str]:
    """Classify the algorithm's local registration history; missing legacy data stays unknown."""
    if not isinstance(raw_frontend, dict):
        return None
    attempts = raw_frontend.get("registration_attempts")
    if not isinstance(attempts, list):
        return None
    if not attempts:
        return "no_registration_attempt"
    if any(not isinstance(attempt, dict) for attempt in attempts):
        return "invalid_attempt_history"

    def accepted(attempt: dict[str, Any]) -> Optional[bool]:
        converged = attempt.get("converged")
        finite = attempt.get("transform_finite")
        gates = attempt.get("rejected_gates")
        if not isinstance(converged, bool) or not isinstance(finite, bool) or not isinstance(gates, list):
            return None
        return converged and finite and not gates

    numbered: list[tuple[int, dict[str, Any]]] = []
    for attempt in attempts:
        number = attempt.get("attempt")
        if isinstance(number, bool) or not isinstance(number, int) or number <= 0:
            return "invalid_attempt_history"
        numbered.append((number, attempt))
    numbered.sort(key=lambda item: item[0])
    if [number for number, _ in numbered] != list(range(1, len(numbered) + 1)):
        return "invalid_attempt_history"
    outcomes = [accepted(attempt) for _, attempt in numbered]
    if any(value is None for value in outcomes):
        return "invalid_attempt_history"
    if len(outcomes) == 1 and outcomes[0] is True:
        return "first_attempt_success"
    if len(outcomes) == 2 and outcomes[0] is False and outcomes[1] is True:
        return "recovered_after_one_retry"
    if not any(outcomes):
        return "registration_failed"
    return "other_attempt_pattern"


def _terminal_status_event(trial: dict[str, Any]) -> dict[str, Any]:
    events = trial.get("status_events")
    if not isinstance(events, list):
        return {}
    for event in reversed(events):
        if isinstance(event, dict) and str(event.get("stage", "")).upper() in {
            "SUCCEEDED", "FAILED", "CANCELED", "CANCELLED",
        }:
            return event
    return next((event for event in reversed(events) if isinstance(event, dict)), {})


_RAW_COUNTER_KEYS = (
    "lidar_received", "lidar_used", "lidar_rejected", "imu_received",
    "imu_accepted", "imu_rejected", "queue_dropped", "queue_overflow_count",
    "oversized_scan_count", "stale_scan_count", "capture_ignored_count",
    "canceled_scan_count",
)

_RAW_INTEGER_FIELDS = (
    "generation", "scan_queue_capacity", "max_scan_message_bytes", "scan_queue_depth",
    "scan_result_depth", "scan_outstanding_count", "scan_queue_bytes",
    "scan_queue_high_watermark", "scan_queue_max_bytes", "queue_overflow_count",
    "oversized_scan_count", "stale_scan_count", "capture_ignored_count",
    "canceled_scan_count", "iterations", "first_attempt_iterations", "retry_count",
    "recovery_attempts", "recovered_scan_count",
)

_RAW_FLOAT_FIELDS = (
    "lidar_latest_source_stamp_sec", "lidar_latest_receipt_wall_sec",
    "lidar_latest_receipt_ros_sec", "lidar_latest_receipt_ros_delay_sec",
    "lidar_max_receipt_ros_delay_sec", "processed_source_stamp_sec",
    "processed_source_age_sec",
)


def _optional_int(value: Any) -> Optional[int]:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _optional_bool(value: Any) -> Optional[bool]:
    return value if isinstance(value, bool) else None


def _string_list(value: Any) -> Optional[list[str]]:
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        return None
    return list(value)


def _normalize_raw_frontend(raw: Any) -> Optional[dict[str, Any]]:
    """Normalize the node's flat raw status while retaining the source event.

    The node reports received and consumed counts separately. In particular,
    LiDAR callbacks continue during SEARCHING while frontend processing stops;
    that difference is not a drop count. Only the explicit bounded-queue
    counter is reported as ``queue_dropped``. Unreported counters and reason
    breakdowns stay null.
    """
    if not isinstance(raw, dict):
        return None

    # Keep support for previously produced/test nested records. The enclosing
    # terminal_status and status_events are always kept verbatim by the caller.
    if isinstance(raw.get("inputs"), dict):
        return dict(raw)

    def measured_count(name: str) -> Any:
        value = raw.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return value if math.isfinite(float(value)) else None

    measurement_valid = any(measured_count(key) is not None for key in _RAW_COUNTER_KEYS)
    measurement_valid = measurement_valid or any(
        _optional_int(raw.get(field)) is not None for field in _RAW_INTEGER_FIELDS
    ) or any(_optional_float(raw.get(field)) is not None for field in _RAW_FLOAT_FIELDS)
    measurement_valid = measurement_valid or any(
        _optional_bool(raw.get(field)) is not None
        for field in ("converged", "transform_finite")
    ) or _string_list(raw.get("rejected_gates")) is not None
    normalized = dict(raw)
    for field in _RAW_INTEGER_FIELDS:
        normalized[field] = _optional_int(raw.get(field))
    for field in _RAW_FLOAT_FIELDS:
        normalized[field] = _optional_float(raw.get(field))
    for field in ("converged", "transform_finite"):
        normalized[field] = _optional_bool(raw.get(field))
    normalized["rejected_gates"] = _string_list(raw.get("rejected_gates"))
    normalized.update({
        "schema": "node_flat_raw_frontend_v1",
        "measurement_valid": measurement_valid,
        "resource_group": "algorithm_process",
        "separate_cpu_rss_accounting": False,
        "inputs": {
            "lidar": {
                "received": measured_count("lidar_received"),
                "accepted": None,
                "used": measured_count("lidar_used"),
                "drop": None,
                "stale": None,
                "rejected": measured_count("lidar_rejected"),
                "rejection_reasons": None,
            },
            "imu": {
                "received": measured_count("imu_received"),
                "accepted": measured_count("imu_accepted"),
                "used": None,
                "drop": None,
                "stale": None,
                "rejected": measured_count("imu_rejected"),
                "rejection_reasons": None,
            },
        },
        "metrics": {
            "queue_dropped": measured_count("queue_dropped"),
            **{field: _optional_int(raw.get(field)) for field in _RAW_INTEGER_FIELDS},
            **{field: _optional_float(raw.get(field)) for field in _RAW_FLOAT_FIELDS},
            "processing_sec": _optional_float(raw.get("processing_sec")),
            "worker_wall_sec": _optional_float(raw.get("worker_wall_sec")),
            "max_worker_wall_sec": _optional_float(raw.get("max_worker_wall_sec")),
            "scan_queue_wait_sec": _optional_float(raw.get("scan_queue_wait_sec")),
            "max_scan_queue_wait_sec": _optional_float(raw.get("max_scan_queue_wait_sec")),
            "fitness": _optional_float(raw.get("fitness")),
            "inlier_ratio": _optional_float(raw.get("inlier_ratio")),
            "translation_step_m": _optional_float(raw.get("translation_step")),
            "rotation_prior_error_rad": _optional_float(raw.get("rotation_prior_error")),
            "linear_speed_m_sec": _optional_float(raw.get("linear_speed")),
            "angular_speed_rad_sec": _optional_float(raw.get("angular_speed")),
            "lidar_max_received_stamp_gap_sec": _optional_float(raw.get("lidar_max_received_stamp_gap_sec")),
            "lidar_max_callback_wall_gap_sec": _optional_float(raw.get("lidar_max_callback_wall_gap_sec")),
            "imu_max_received_stamp_gap_sec": _optional_float(raw.get("imu_max_received_stamp_gap_sec")),
            "imu_max_callback_wall_gap_sec": _optional_float(raw.get("imu_max_callback_wall_gap_sec")),
            "imu_latest_callback_ros_delay_sec": _optional_float(raw.get("imu_latest_callback_ros_delay_sec")),
        },
        "converged": _optional_bool(raw.get("converged")),
        "transform_finite": _optional_bool(raw.get("transform_finite")),
        "rejected_gates": _string_list(raw.get("rejected_gates")),
        "registration_attempts": raw.get("registration_attempts")
        if isinstance(raw.get("registration_attempts"), list) else None,
        "worker_error": raw.get("worker_error") if isinstance(raw.get("worker_error"), str) else None,
        "recovery_pending": _optional_bool(raw.get("recovery_pending")),
        "failure_class": raw.get("failure_class") if isinstance(raw.get("failure_class"), str) else None,
        "recovery_reason": raw.get("recovery_reason") if isinstance(raw.get("recovery_reason"), str) else None,
        "failure_reason": (
            raw.get("reason") if raw.get("accepted") is False and
            isinstance(raw.get("reason"), str) and
            not raw.get("reason", "").startswith("waiting") else None
        ),
    })
    return normalized


def _resource_summary(scored_trials: list[dict[str, Any]], resource_name: str) -> dict[str, Any]:
    resource_summary = {
        metric: _distribution([
            _optional_float(_nested(trial, resource_name, metric)) for trial in scored_trials
        ])
        for metric in _RESOURCE_METRICS
    }
    core_seconds = []
    for trial in scored_trials:
        user = _optional_float(_nested(trial, resource_name, "cpu_user_delta_sec"))
        system = _optional_float(_nested(trial, resource_name, "cpu_sys_delta_sec"))
        core_seconds.append(None if user is None or system is None else user + system)
    resource_summary["derived_core_seconds_from_user_plus_sys"] = _distribution(core_seconds)
    return resource_summary


def _reset_attempt_summary(scored_trials: list[dict[str, Any]], expected_trials: int) -> dict[str, Any]:
    """Summarize recorded reset retries without treating legacy omissions as zero."""
    available = [trial["reset_attempts"] for trial in scored_trials
                 if isinstance(trial.get("reset_attempts"), list)]
    summary: dict[str, Any] = {
        "planned_trial_count": expected_trials,
        "available_trial_count": len(available),
        "attempted_trial_count": None,
        "first_attempt_success_count": None,
        "first_attempt_success_rate": None,
        "recovered_on_retry_count": None,
        "failed_after_reset_count": None,
        "total_attempt_count": None,
    }
    if not available:
        return summary

    attempts_by_trial = [attempts for attempts in available if attempts]
    first_attempts = [
        next((attempt for attempt in attempts if isinstance(attempt, dict) and attempt.get("attempt") == 1), None)
        for attempts in attempts_by_trial
    ]
    first_attempts = [attempt for attempt in first_attempts if attempt is not None]
    first_successes = sum(1 for attempt in first_attempts if attempt.get("success") is True)
    recovered = 0
    failed_after_reset = 0
    for attempts in attempts_by_trial:
        first = next((attempt for attempt in attempts
                      if isinstance(attempt, dict) and attempt.get("attempt") == 1), None)
        retry = next((attempt for attempt in attempts
                      if isinstance(attempt, dict) and attempt.get("attempt") == 2), None)
        if first and retry and first.get("success") is False and retry.get("success") is True:
            recovered += 1
        final = max(
            (attempt for attempt in attempts if isinstance(attempt, dict)),
            key=lambda attempt: attempt.get("attempt", 0)
            if isinstance(attempt.get("attempt"), int) and not isinstance(attempt.get("attempt"), bool) else 0,
            default=None,
        )
        if final is not None and final.get("success") is False:
            failed_after_reset += 1
    summary.update({
        "attempted_trial_count": len(attempts_by_trial),
        "first_attempt_success_count": first_successes,
        "first_attempt_success_rate": first_successes / len(first_attempts) if first_attempts else None,
        "recovered_on_retry_count": recovered,
        "failed_after_reset_count": failed_after_reset,
        "total_attempt_count": sum(len(attempts) for attempts in available),
    })
    return summary


def _timing_with_composites(timing: Any) -> dict[str, Any]:
    result = dict(timing) if isinstance(timing, dict) else {}
    for composite, components in _TIMING_COMPOSITES.items():
        result[composite] = {}
        for clock in ("ros", "wall"):
            values = [
                _optional_float(_nested(result, component, clock))
                for component in components
            ]
            result[composite][clock] = sum(values) if all(value is not None for value in values) else None
    return result


def _registration_attempt_summary(scored_trials: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize only the terminal snapshot's latest processed scan, not the whole action."""
    outcomes = [trial.get("registration_attempt_outcome") for trial in scored_trials]
    available = [outcome for outcome in outcomes if isinstance(outcome, str)]
    counts = {
        name: available.count(name)
        for name in (
            "first_attempt_success", "recovered_after_one_retry", "registration_failed",
            "no_registration_attempt", "invalid_attempt_history", "other_attempt_pattern",
        )
    }
    return {
        "trial_count": len(scored_trials),
        "attempt_history_available_trial_count": len(available),
        "unknown_or_missing_history_trial_count": len(scored_trials) - len(available),
        "classification_denominator": len(available) if available else None,
        **{f"{name}_count": count if available else None for name, count in counts.items()},
    }


def _algorithm_recovery_summary(scored_trials: list[dict[str, Any]]) -> dict[str, Any]:
    outcomes: list[str] = []
    known_attempts: list[int] = []
    recovered_scan_counts: list[int] = []
    recovery_pending_values: list[bool] = []
    failure_class_counts: dict[str, int] = {}
    recovery_reason_counts: dict[str, int] = {}
    for trial in scored_trials:
        raw_frontend = trial.get("raw_frontend")
        if isinstance(raw_frontend, dict):
            recovered_scan_count = _optional_int(raw_frontend.get("recovered_scan_count"))
            if recovered_scan_count is not None:
                recovered_scan_counts.append(recovered_scan_count)
            recovery_pending = _optional_bool(raw_frontend.get("recovery_pending"))
            if recovery_pending is not None:
                recovery_pending_values.append(recovery_pending)
            failure_class = raw_frontend.get("failure_class")
            if isinstance(failure_class, str) and failure_class:
                failure_class_counts[failure_class] = failure_class_counts.get(failure_class, 0) + 1
            recovery_reason = raw_frontend.get("recovery_reason")
            if isinstance(recovery_reason, str) and recovery_reason:
                recovery_reason_counts[recovery_reason] = recovery_reason_counts.get(recovery_reason, 0) + 1
        if trial.get("action_success") is not True:
            outcomes.append("action_failure")
            attempts = _optional_int(_nested(trial, "raw_frontend", "recovery_attempts"))
            if attempts is not None:
                known_attempts.append(attempts)
            continue
        if trial.get("evaluation_valid") is not True:
            outcomes.append("unscored_action_success")
            attempts = _optional_int(_nested(trial, "raw_frontend", "recovery_attempts"))
            if attempts is not None:
                known_attempts.append(attempts)
            continue
        if trial.get("pose_evaluation_success") is not True:
            outcomes.append("wrong_pose_accepted")
            attempts = _optional_int(_nested(trial, "raw_frontend", "recovery_attempts"))
            if attempts is not None:
                known_attempts.append(attempts)
            continue
        attempts = _optional_int(_nested(trial, "raw_frontend", "recovery_attempts"))
        if attempts is None:
            outcomes.append("correct_with_unknown_recovery_history")
        else:
            known_attempts.append(attempts)
            if attempts == 0:
                outcomes.append("correct_without_recovery")
            elif attempts == 1:
                outcomes.append("correct_after_one_recovery")
            else:
                outcomes.append("correct_after_multiple_recoveries")

    counts = {name: outcomes.count(name) for name in (
        "correct_without_recovery", "correct_after_one_recovery", "wrong_pose_accepted",
        "action_failure", "unscored_action_success", "correct_with_unknown_recovery_history",
        "correct_after_multiple_recoveries",
    )}
    classified_correct = (counts["correct_without_recovery"] + counts["correct_after_one_recovery"]
                          + counts["correct_after_multiple_recoveries"])
    return {
        "trial_count": len(scored_trials),
        "recovery_history_available_trial_count": len(known_attempts),
        "recovery_history_missing_trial_count": len(scored_trials) - len(known_attempts),
        "action_success_count": sum(1 for trial in scored_trials if trial.get("action_success") is True),
        "action_failure_count": counts["action_failure"],
        # Legacy success records without this field are unknown; they cannot be
        # counted as first-pass successes.
        "correct_without_recovery_count": counts["correct_without_recovery"] if known_attempts else None,
        "correct_after_one_recovery_count": counts["correct_after_one_recovery"] if known_attempts else None,
        "wrong_pose_accepted_count": counts["wrong_pose_accepted"],
        "unscored_action_success_count": counts["unscored_action_success"],
        "correct_with_unknown_recovery_history_count": counts["correct_with_unknown_recovery_history"],
        "correct_after_multiple_recoveries_count": counts["correct_after_multiple_recoveries"],
        "known_correct_action_count": classified_correct if known_attempts else None,
        "first_correct_rate_among_known_correct_pose_outputs": (
            counts["correct_without_recovery"] / classified_correct if classified_correct else None
        ),
        "recovered_correct_rate_among_known_correct_pose_outputs": (
            counts["correct_after_one_recovery"] / classified_correct if classified_correct else None
        ),
        "recovery_attempt_count": _distribution(known_attempts),
        "recovered_scan_count": _distribution(recovered_scan_counts),
        "recovery_pending": {
            "available_trial_count": len(recovery_pending_values),
            "true_count": sum(1 for value in recovery_pending_values if value),
            "false_count": sum(1 for value in recovery_pending_values if not value),
        },
        "failure_class_counts": failure_class_counts,
        "recovery_reason_counts": recovery_reason_counts,
    }


def _deadline_accuracy_summary(
    scored_trials: list[dict[str, Any]], expected_trials: int,
) -> dict[str, Any]:
    observed = len(scored_trials)
    action_successes = [trial for trial in scored_trials if trial.get("action_success") is True]
    timed_actions = [trial for trial in scored_trials if trial.get("action_elapsed_sec") is not None]
    timed_successes = [trial for trial in action_successes if trial.get("action_elapsed_sec") is not None]
    confirmed = sum(1 for trial in scored_trials if trial.get("correct_within_15s") is True)
    return {
        "deadline_sec": CORRECT_LOCALIZATION_DEADLINE_SEC,
        "expected_cohort_count": expected_trials,
        "observed_trial_count": observed,
        "cohort_complete": observed == expected_trials,
        "unobserved_trial_count": max(0, expected_trials - observed),
        "confirmed_correct_count": confirmed,
        # Missing timing or pose data adds nothing to the numerator; expected_trials
        # remains the denominator for the frozen cohort.
        "confirmed_correct_rate_all_cohort": (
            confirmed / expected_trials if expected_trials > 0 and timed_actions else None
        ),
        "action_elapsed_available_trial_count": len(timed_actions),
        "action_elapsed_missing_trial_count": observed - len(timed_actions),
        "action_success_count": len(action_successes),
        "timed_action_success_count": len(timed_successes),
        "confirmed_correct_success_count": confirmed,
        "confirmed_correct_rate_among_successful_actions": (
            confirmed / len(action_successes) if action_successes else None
        ),
        "valid_pose_score_count": sum(1 for trial in scored_trials if trial.get("evaluation_valid") is True),
        "missing_or_invalid_pose_score_count": sum(
            1 for trial in scored_trials if trial.get("evaluation_valid") is not True
        ),
    }


def _summarize_scored_trials(scored_trials: list[dict[str, Any]], expected_trials: int) -> dict[str, Any]:
    valid_scores = [trial for trial in scored_trials if trial.get("evaluation_valid") is True]
    non_successful_valid_scores = [trial for trial in valid_scores if trial.get("success") is not True]
    successful_valid_scores = [trial for trial in valid_scores if trial.get("success") is True]
    successful = sum(1 for trial in scored_trials if trial.get("success") is True)
    action_successes = sum(1 for trial in scored_trials if trial.get("action_success") is True)
    timeouts = sum(1 for trial in scored_trials if trial.get("timeout") is True)
    invalid_scores = sum(1 for trial in scored_trials if trial.get("evaluation_valid") is not True)
    summary: dict[str, Any] = {
        "batch": {
            "expected_trials": expected_trials,
            "observed_trials": len(scored_trials),
            "complete": len(scored_trials) == expected_trials,
            "success_count": successful,
            "success_rate": successful / len(scored_trials) if scored_trials else None,
            "success_rate_percent": 100.0 * successful / len(scored_trials) if scored_trials else None,
            "failure_count": len(scored_trials) - successful,
            "failure_rate": (len(scored_trials) - successful) / len(scored_trials) if scored_trials else None,
            "failure_rate_percent": (100.0 * (len(scored_trials) - successful) / len(scored_trials)
                                     if scored_trials else None),
            "action_success_count": action_successes,
            "timeout_count": timeouts,
            "invalid_or_missing_score_count": invalid_scores,
            "denominator_includes_action_failures_and_timeouts": True,
        },
        "scored_pose_count": len(valid_scores),
        "accuracy": _accuracy_metrics(valid_scores),
        "accuracy_cohorts": {
            "all_valid_pose_outputs": _accuracy_metrics(valid_scores),
            "valid_pose_outputs_from_non_successful_trials": _accuracy_metrics(non_successful_valid_scores),
            "valid_pose_outputs_from_successful_trials": _accuracy_metrics(successful_valid_scores),
        },
        "correct_localization_within_15s": _deadline_accuracy_summary(scored_trials, expected_trials),
        "terminal_scan_registration_attempt_snapshot": _registration_attempt_summary(scored_trials),
        "algorithm_recovery_outcomes": _algorithm_recovery_summary(scored_trials),
        "timing": {},
        "algorithm_process": {},
        "lio_pipeline": {},
        "gazebo_process_load": {},
        "trial_evaluator_process": {},
        "diagnostic_recorder_process": {},
        "rtf": _distribution([_optional_float(trial.get("rtf")) for trial in scored_trials]),
    }
    input_mode_counts: dict[str, int] = {}
    timeout_source_counts: dict[str, int] = {}
    for trial in scored_trials:
        mode = trial.get("input_mode")
        if mode is not None:
            key = str(mode)
            input_mode_counts[key] = input_mode_counts.get(key, 0) + 1
        source = trial.get("action_timeout_source")
        if source is not None:
            key = str(source)
            timeout_source_counts[key] = timeout_source_counts.get(key, 0) + 1
    summary["input_mode_counts"] = input_mode_counts
    summary["action_window_count"] = sum(1 for trial in scored_trials
                                         if trial.get("action_window_exists") is True)
    summary["action_timeout_source_counts"] = timeout_source_counts
    summary["raw_input_observation"] = {
        "count_semantics": "messages received by the trial client; not estimator acceptance, drops, or rejects",
        "counts_since_client_start": {},
        "counts_during_action": {},
    }
    for scope in ("counts_since_client_start", "counts_during_action"):
        for sensor in ("lidar", "imu"):
            summary["raw_input_observation"][scope][sensor] = _distribution([
                _optional_float(_nested(trial, "raw_input_observation", scope, sensor))
                for trial in scored_trials
            ])
    summary["frontend_process"] = {
        "included_in_algorithm_trial_count": sum(
            1 for trial in scored_trials
            if _nested(trial, "frontend_process", "included_in_algorithm") is True
        ),
        "separate_measurement_valid_trial_count": sum(
            1 for trial in scored_trials
            if _nested(trial, "frontend_process", "measurement_valid") is True
        ),
        "separate_cpu_rss_accounting": False,
    }
    raw_frontend_measured = [
        trial for trial in scored_trials
        if _nested(trial, "raw_frontend", "measurement_valid") is True
    ]
    raw_frontend_summary: dict[str, Any] = {
        "measurement_valid_trial_count": len(raw_frontend_measured),
        "resource_group": "algorithm_process",
        "separate_cpu_rss_accounting": False,
        "scan_metric_scope": "terminal status snapshot; registration metrics and rejected gates describe the latest processed scan",
        "counter_semantics": {
            "lidar_received": "raw cloud callbacks received during the action, including SEARCHING",
            "lidar_used": "raw scans accepted by the local frontend; SEARCHING callbacks are not processed",
            "lidar_drop": "unavailable as a per-sensor counter; use queue_dropped for bounded cloud queue evictions",
            "imu_accepted": "IMU samples accepted by the frontend input validator; not a per-registration use count",
            "queue_dropped": "raw cloud messages evicted from the bounded frontend queue",
            "unknown_measurements": "null; not inferred as zero",
        },
        "inputs": {},
        "rejection_reasons": {},
        "metrics": {},
        "failure_reason_counts": {},
    }
    for sensor in ("lidar", "imu"):
        raw_frontend_summary["inputs"][sensor] = {
            field: _distribution([
                _optional_float(_nested(trial, "raw_frontend", "inputs", sensor, field))
                for trial in raw_frontend_measured
            ])
            for field in ("received", "accepted", "used", "drop", "stale", "rejected")
        }
        available_reasons = []
        for trial in raw_frontend_measured:
            reasons = _nested(trial, "raw_frontend", "inputs", sensor, "rejection_reasons")
            if isinstance(reasons, dict):
                available_reasons.append((trial, reasons))
        if not available_reasons:
            raw_frontend_summary["rejection_reasons"][sensor] = None
        else:
            reason_names = {str(reason) for _, reasons in available_reasons for reason in reasons}
            raw_frontend_summary["rejection_reasons"][sensor] = {
                reason: _distribution([
                    _optional_float(reasons.get(reason)) if isinstance(reasons, dict) else None
                    for trial in raw_frontend_measured
                    for reasons in [
                        _nested(trial, "raw_frontend", "inputs", sensor, "rejection_reasons")
                    ]
                ])
                for reason in sorted(reason_names)
            }

    def raw_metric(trial: dict[str, Any], field: str) -> Optional[float]:
        direct = _nested(trial, "raw_frontend", field)
        if direct is None:
            direct = _nested(trial, "raw_frontend", "metrics", field)
        return _optional_float(direct)

    for field in (
        "queue_depth", "queue_dropped", "local_registration_wall_sec", "local_fitness_m",
        "local_inlier_ratio", "relative_translation_m", "angular_speed_rad_sec",
        "linear_speed_m_sec", "processing_sec", "worker_wall_sec", "max_worker_wall_sec",
        "scan_queue_wait_sec", "max_scan_queue_wait_sec", "fitness", "inlier_ratio",
        "translation_step_m", "rotation_prior_error_rad", "lidar_max_received_stamp_gap_sec",
        "lidar_max_callback_wall_gap_sec", "imu_max_received_stamp_gap_sec",
        "imu_max_callback_wall_gap_sec", "imu_latest_callback_ros_delay_sec",
        "generation", "scan_queue_capacity", "max_scan_message_bytes", "scan_queue_depth",
        "scan_result_depth", "scan_outstanding_count", "scan_queue_bytes",
        "scan_queue_high_watermark", "scan_queue_max_bytes", "queue_overflow_count",
        "oversized_scan_count", "stale_scan_count", "capture_ignored_count",
        "canceled_scan_count", "lidar_latest_receipt_ros_delay_sec",
        "lidar_max_receipt_ros_delay_sec", "processed_source_age_sec", "iterations",
        "first_attempt_iterations", "retry_count",
    ):
        raw_frontend_summary["metrics"][field] = _distribution([
            raw_metric(trial, field) for trial in raw_frontend_measured
        ])
    for field in ("converged", "transform_finite"):
        available = [
            _nested(trial, "raw_frontend", field)
            for trial in raw_frontend_measured
            if isinstance(_nested(trial, "raw_frontend", field), bool)
        ]
        raw_frontend_summary[field] = {
            "available_trial_count": len(available),
            "true_count": sum(1 for value in available if value is True),
            "false_count": sum(1 for value in available if value is False),
        }
    gate_observations = [
        _nested(trial, "raw_frontend", "rejected_gates")
        for trial in raw_frontend_measured
        if isinstance(_nested(trial, "raw_frontend", "rejected_gates"), list)
    ]
    raw_frontend_summary["rejected_gate_observation_trial_count"] = len(gate_observations)
    raw_frontend_summary["rejected_gate_trial_counts"] = {
        gate: sum(1 for observed in gate_observations if gate in observed)
        for gate in sorted({gate for observed in gate_observations for gate in observed
                            if isinstance(gate, str)})
    }
    worker_errors: dict[str, int] = {}
    for trial in raw_frontend_measured:
        reason = _nested(trial, "raw_frontend", "failure_reason")
        if reason is not None:
            key = str(reason)
            raw_frontend_summary["failure_reason_counts"][key] = (
                raw_frontend_summary["failure_reason_counts"].get(key, 0) + 1
            )
        worker_error = _nested(trial, "raw_frontend", "worker_error")
        if isinstance(worker_error, str) and worker_error:
            worker_errors[worker_error] = worker_errors.get(worker_error, 0) + 1
    raw_frontend_summary["worker_error_counts"] = worker_errors
    summary["raw_frontend"] = raw_frontend_summary
    stage_names: set[str] = set()
    for trial in scored_trials:
        for clock in ("ros", "wall"):
            stages = trial.get("stage_ros_sec" if clock == "ros" else "stage_wall_sec")
            if isinstance(stages, dict):
                stage_names.update(str(name) for name in stages)
    summary["status_stage_timing"] = {}
    for stage in sorted(stage_names):
        summary["status_stage_timing"][stage] = {}
        for clock in ("ros", "wall"):
            key = "stage_ros_sec" if clock == "ros" else "stage_wall_sec"
            summary["status_stage_timing"][stage][clock + "_sec"] = _distribution([
                _optional_float(_nested(trial, key, stage)) for trial in scored_trials
            ])
    requested_positions: set[tuple[float, float, float]] = set()
    requested_yaws: set[float] = set()
    requested_cells: set[tuple[float, float, float, float]] = set()
    for trial in scored_trials:
        requested = trial.get("requested_pose")
        if not isinstance(requested, dict):
            continue
        xyz = tuple(_optional_float(requested.get(axis)) for axis in ("x", "y", "z"))
        yaw = _optional_float(requested.get("yaw"))
        if any(value is None for value in xyz) or yaw is None:
            continue
        position = tuple(round(value, 6) for value in xyz if value is not None)
        yaw_key = round(wrap_angle(yaw), 6)
        requested_positions.add(position)  # type: ignore[arg-type]
        requested_yaws.add(yaw_key)
        requested_cells.add((*position, yaw_key))
    summary["batch"]["requested_pose_coverage"] = {
        "unique_positions": len(requested_positions),
        "unique_yaws": len(requested_yaws),
        "unique_position_yaw_cells": len(requested_cells),
        "complete_30_position_x_4_yaw_grid": (
            len(requested_positions) == 30 and len(requested_yaws) == 4 and len(requested_cells) == 120
        ),
    }
    summary["batch"]["reset_summary"] = _reset_attempt_summary(scored_trials, expected_trials)
    summary["failure_reason_counts"] = {}
    summary["action_reason_counts"] = {}
    summary["pose_evaluation_failure_reason_counts"] = {}
    for trial in scored_trials:
        if trial.get("success") is True:
            continue
        reason = str(trial.get("failure_reason") or trial.get("action_reason") or "unspecified_failure")
        summary["failure_reason_counts"][reason] = summary["failure_reason_counts"].get(reason, 0) + 1
        if trial.get("action_success") is not True:
            action_reason = str(trial.get("action_reason") or "unspecified_action_failure")
            summary["action_reason_counts"][action_reason] = summary["action_reason_counts"].get(action_reason, 0) + 1
        if trial.get("evaluation_valid") is True and trial.get("pose_evaluation_success") is not True:
            pose_reason = str(trial.get("pose_evaluation_failure_reason") or "pose_error_threshold_exceeded")
            summary["pose_evaluation_failure_reason_counts"][pose_reason] = (
                summary["pose_evaluation_failure_reason_counts"].get(pose_reason, 0) + 1
            )
    additional_timing_stages: set[str] = set()
    for trial in scored_trials:
        timing = _timing_with_composites(trial.get("timing_sec", {}))
        if not isinstance(timing, dict):
            continue
        for name, value in timing.items():
            if name not in _TIMING_STAGES and isinstance(value, dict) and any(
                clock in value for clock in ("ros", "wall")
            ):
                additional_timing_stages.add(str(name))
    for stage in _TIMING_STAGES:
        stage_entry: dict[str, Any] = {}
        for clock in ("ros", "wall"):
            values = []
            for trial in scored_trials:
                timing = _timing_with_composites(trial.get("timing_sec", {}))
                if not isinstance(timing, dict):
                    continue
                if stage == "total":
                    key = "total_ros" if clock == "ros" else "total_wall"
                    values.append(_optional_float(timing.get(key)))
                else:
                    item = timing.get(stage)
                    if isinstance(item, dict):
                        values.append(_optional_float(item.get(clock)))
                    else:
                        values.append(None)
            stage_entry[clock + "_sec"] = _distribution(values)
        summary["timing"][stage] = stage_entry
    summary["timing"]["additional_stages"] = {}
    for stage in sorted(additional_timing_stages):
        stage_entry = {}
        for clock in ("ros", "wall"):
            stage_entry[clock + "_sec"] = _distribution([
                _optional_float(_nested(trial, "timing_sec", stage, clock))
                for trial in scored_trials
            ])
        summary["timing"]["additional_stages"][stage] = stage_entry
    for resource_name in (
        "algorithm_process", "lio_pipeline", "gazebo_process_load",
        "trial_evaluator_process", "diagnostic_recorder_process",
    ):
        summary[resource_name] = _resource_summary(scored_trials, resource_name)
    environments = []
    for trial in scored_trials:
        environment = trial.get("environment")
        if environment is not None and environment not in environments:
            environments.append(environment)
    summary["environment_variants"] = environments

    yaw_groups: dict[str, list[dict[str, Any]]] = {}
    for trial in scored_trials:
        requested_pose = trial.get("requested_pose")
        if not isinstance(requested_pose, dict):
            continue
        yaw = _optional_float(requested_pose.get("yaw"))
        if yaw is None:
            continue
        key = f"{math.degrees(wrap_angle(yaw)):.3f}deg"
        yaw_groups.setdefault(key, []).append(trial)
    summary["by_requested_yaw"] = {}
    for yaw_key, group in sorted(yaw_groups.items()):
        group_valid = [trial for trial in group if trial.get("evaluation_valid") is True]
        group_pos = [
            value for value in (_optional_float(t.get("position_3d_error_m")) for t in group_valid)
            if value is not None
        ]
        group_rot = [
            value for value in (_optional_float(t.get("orientation_geodesic_error_deg")) for t in group_valid)
            if value is not None
        ]
        n = len(group)
        n_success = sum(1 for trial in group if trial.get("success") is True)
        summary["by_requested_yaw"][yaw_key] = {
            "trial_count": n,
            "success_count": n_success,
            "success_rate": n_success / n if n else None,
            "position_3d_error_m": _distribution(group_pos),
            "orientation_geodesic_error_deg": _distribution(group_rot),
        }
    return summary


def aggregate_batch(trials_path: str, pose_errors_path: str, output_prefix: str,
                    expected_trials: int = 120) -> dict[str, Any]:
    """Join action and pose ledgers without crossing cohort rounds or reusing scores."""
    if expected_trials <= 0:
        raise ValueError("expected_trials must be positive")
    trials = _read_jsonl(trials_path)
    evaluations = _read_jsonl(pose_errors_path)
    trial_identity_modes = {_cohort_identity(trial, f"trial row {index + 1}") is not None
                            for index, trial in enumerate(trials)}
    if len(trial_identity_modes) > 1:
        raise ValueError("trial ledger mixes tagged cohort rows with legacy rows")
    tagged_cohort = trial_identity_modes == {True}
    cohort_run_ids = {
        identity[0] for trial in trials
        if (identity := _cohort_identity(trial, "trial row")) is not None
    }
    if tagged_cohort and len(cohort_run_ids) > 1:
        raise ValueError(f"trial ledger mixes run_id cohorts: {sorted(cohort_run_ids)}")

    score_by_stamp: dict[int, dict[str, Any]] = {}
    score_by_cohort_stamp: dict[tuple[str, int, int], dict[str, Any]] = {}
    for evaluation in evaluations:
        try:
            stamp = int(evaluation["pose_stamp_ns"])
        except (KeyError, TypeError, ValueError):
            raise ValueError("pose error row is missing integer pose_stamp_ns")
        identity = _cohort_identity(evaluation, f"pose evaluation for stamp {stamp}")
        if tagged_cohort:
            if identity is None:
                raise ValueError(f"untagged pose evaluation for tagged cohort stamp {stamp}")
            key = (identity[0], identity[1], stamp)
            if key in score_by_cohort_stamp:
                raise ValueError(f"duplicate pose evaluation for cohort identity/stamp {key}")
            score_by_cohort_stamp[key] = evaluation
        else:
            if identity is not None:
                raise ValueError("tagged pose evaluation cannot be joined to a legacy trial ledger")
            if stamp in score_by_stamp:
                raise ValueError(f"duplicate pose evaluation for stamp {stamp}")
            score_by_stamp[stamp] = evaluation

    trial_ids: set[tuple[Optional[str], Optional[int], str]] = set()
    trial_pose_stamps: set[tuple[Optional[str], Optional[int], int]] = set()
    source_indices_by_repetition: set[tuple[Optional[str], Optional[int], Any]] = set()
    source_index_by_trial_id: dict[str, Any] = {}
    joined_cohort_score_keys: set[tuple[str, int, int]] = set()
    scored_trials: list[dict[str, Any]] = []
    map_hashes: set[str] = set()
    status_map_hashes: set[str] = set()
    transforms: list[Pose] = []
    for index, trial in enumerate(trials):
        trial_id = str(trial.get("trial_id", trial.get("trial_index", index + 1)))
        identity = _cohort_identity(trial, f"trial row {index + 1}")
        run_id, repetition = identity if identity is not None else (None, None)
        trial_key = (run_id, repetition, trial_id)
        if trial_key in trial_ids:
            raise ValueError(f"duplicate trial identity {trial_key!r}")
        trial_ids.add(trial_key)
        source_trial_index = trial.get("source_trial_index", trial.get("trial_index", index + 1))
        if identity is not None:
            if isinstance(source_trial_index, bool) or not isinstance(source_trial_index, int) or source_trial_index < 0:
                raise ValueError(f"tagged trial {trial_id!r} is missing a non-negative source_trial_index")
            source_key = (run_id, repetition, source_trial_index)
            if source_key in source_indices_by_repetition:
                raise ValueError(f"duplicate source_trial_index within cohort repetition {source_key}")
            source_indices_by_repetition.add(source_key)
            prior_source_index = source_index_by_trial_id.setdefault(trial_id, source_trial_index)
            if prior_source_index != source_trial_index:
                raise ValueError(
                    f"trial_id {trial_id!r} maps to inconsistent source_trial_index values across repetitions"
                )
        action = trial.get("action")
        if not isinstance(action, dict):
            action = {}
        action_success = action.get("success") is True
        timeout = action.get("timeout") is True
        stamp = _trial_pose_stamp(trial)
        if stamp is not None:
            stamp_key = (run_id, repetition, stamp)
            if stamp_key in trial_pose_stamps:
                if identity is None:
                    raise ValueError(f"duplicate trial pose stamp {stamp}")
                raise ValueError(f"duplicate trial pose stamp within cohort {stamp_key}")
            trial_pose_stamps.add(stamp_key)
        if stamp is None:
            evaluation = None
        elif tagged_cohort:
            score_key = (str(run_id), int(repetition), stamp)
            evaluation = score_by_cohort_stamp.get(score_key)
            if evaluation is not None:
                evaluation_trial_id = evaluation.get("trial_id")
                if evaluation_trial_id is not None and str(evaluation_trial_id) != trial_id:
                    raise ValueError(
                        f"pose score trial_id {evaluation_trial_id!r} does not match action trial_id {trial_id!r}"
                    )
                evaluation_source_index = evaluation.get("source_trial_index")
                if evaluation_source_index is not None and evaluation_source_index != source_trial_index:
                    raise ValueError(
                        f"pose score source_trial_index {evaluation_source_index!r} does not match "
                        f"action source_trial_index {source_trial_index!r}"
                    )
                joined_cohort_score_keys.add(score_key)
        else:
            evaluation = score_by_stamp.get(stamp)
        for event in trial.get("status_events", []) if isinstance(trial.get("status_events", []), list) else []:
            if isinstance(event, dict) and event.get("map_sha256"):
                status_map_hashes.add(str(event["map_sha256"]))
        evaluation_valid = bool(evaluation and evaluation.get("evaluation_valid") is True)
        evaluation_pass = bool(evaluation and evaluation.get("success") is True and evaluation_valid)
        accepted = action_success and evaluation_pass and not timeout
        terminal_status = _terminal_status_event(trial)
        raw_frontend = _normalize_raw_frontend(terminal_status.get("raw_frontend"))
        action_elapsed_sec = _action_elapsed_sec(trial, terminal_status)
        row: dict[str, Any] = {
            "run_id": run_id,
            "repetition": repetition,
            "trial_id": trial_id,
            "trial_index": trial.get("trial_index", index + 1),
            "source_trial_index": source_trial_index,
            "trial_identity": {"run_id": run_id, "repetition": repetition, "trial_id": trial_id},
            "requested_pose": trial.get("requested_pose"),
            "initial_stamp_ns": trial.get("initial_stamp_ns"),
            "final_stamp_ns": trial.get("final_stamp_ns"),
            "action_goal_start_ros_ns": trial.get("action_goal_start_ros_ns"),
            "action_goal_started_wall_monotonic_sec": trial.get("action_goal_started_wall_monotonic_sec"),
            "pose_stamp_ns": stamp,
            "action_elapsed_sec": action_elapsed_sec,
            "correct_within_15s": (
                None if action_elapsed_sec is None
                else bool(accepted and action_elapsed_sec <= CORRECT_LOCALIZATION_DEADLINE_SEC)
            ),
            "action_success": action_success,
            "action_reason": action.get("reason"),
            "timeout": timeout,
            "evaluation_valid": evaluation_valid,
            "pose_evaluation_success": bool(evaluation and evaluation.get("success") is True and evaluation_valid),
            "pose_evaluation_failure_reason": evaluation.get("failure_reason") if evaluation else None,
            "success": accepted,
            "outcome": "success" if accepted else ("timeout" if timeout else "failure"),
            "failure_reason": None,
            "timing_sec": _timing_with_composites(trial.get("timing_sec", {})),
            "input_mode": trial.get("input_mode"),
            "action_window_exists": trial.get("action_window_exists"),
            "action_timeout_source": action.get("timeout_source"),
            "raw_input_observation": trial.get("raw_input_observation"),
            "reset_attempts": trial.get("reset_attempts"),
            "algorithm_process": trial.get("algorithm_process", {}),
            "lio_pipeline": trial.get("lio_pipeline", trial.get("lio_pipeline_process")),
            "gazebo_process_load": trial.get("gazebo_process_load", {}),
            "trial_evaluator_process": trial.get("trial_evaluator_process", {}),
            "diagnostic_recorder_process": trial.get("diagnostic_recorder_process", {}),
            "frontend_process": trial.get("frontend_process"),
            "status_events": trial.get("status_events", []),
            "terminal_status": terminal_status,
            "raw_frontend": raw_frontend,
            "registration_attempt_outcome": _registration_attempt_outcome(raw_frontend),
            "stage_wall_sec": terminal_status.get("stage_wall_sec"),
            "stage_ros_sec": terminal_status.get("stage_ros_sec"),
            "rtf": trial.get("rtf"),
            "environment": trial.get("environment"),
        }
        if not action_success:
            row["failure_reason"] = action.get("reason", "action_failed")
        elif timeout:
            row["failure_reason"] = action.get("reason", "timeout")
        elif stamp is None:
            row["failure_reason"] = "missing_pose_stamp"
        elif evaluation is None:
            row["failure_reason"] = "missing_pose_evaluation_row"
        elif not evaluation_valid:
            row["failure_reason"] = evaluation.get("failure_reason", "invalid_pose_evaluation")
        elif not evaluation_pass:
            row["failure_reason"] = evaluation.get("failure_reason", "pose_error_threshold_exceeded")
        for field in _ERROR_FIELDS:
            row[field] = _optional_float(evaluation.get(field)) if evaluation else None
        row["pose_interpolation_validity"] = evaluation.get("ground_truth_interpolation_validity") if evaluation else None
        row["position_threshold_m"] = _optional_float(evaluation.get("position_threshold_m")) if evaluation else None
        row["orientation_threshold_deg"] = _optional_float(evaluation.get("orientation_threshold_deg")) if evaluation else None
        alignment = evaluation.get("alignment_metadata", {}) if evaluation else {}
        row["alignment_metadata"] = alignment
        map_hash = alignment.get("map_sha256") if isinstance(alignment, dict) else None
        if map_hash:
            map_hashes.add(str(map_hash))
        if evaluation:
            if evaluation.get("run_id") is not None:
                row["pose_score_run_id"] = evaluation.get("run_id")
                row["pose_score_repetition"] = evaluation.get("repetition")
                row["pose_score_trial_id"] = evaluation.get("trial_id")
            translation = evaluation.get("T_map_world_translation_m")
            quaternion = evaluation.get("T_map_world_quaternion_xyzw")
            if translation is not None and quaternion is not None:
                transforms.append(Pose.make(translation, quaternion))
        scored_trials.append(row)
    if tagged_cohort:
        orphaned_scores = set(score_by_cohort_stamp) - joined_cohort_score_keys
        if orphaned_scores:
            raise ValueError(
                "pose evaluations do not match an action identity/stamp in this cohort: "
                f"{sorted(orphaned_scores)[:5]}"
            )
    if len(map_hashes) > 1:
        raise ValueError(f"cannot aggregate trials scored against different map hashes: {sorted(map_hashes)}")
    if len(status_map_hashes) > 1:
        raise ValueError(f"one or more trials reported changing localization map hashes: {sorted(status_map_hashes)}")
    if map_hashes and status_map_hashes and map_hashes != status_map_hashes:
        raise ValueError(
            f"pose evaluator map hash {sorted(map_hashes)} differs from action status map hash {sorted(status_map_hashes)}"
        )
    if transforms and any(not transforms_match(transforms[0], transform) for transform in transforms[1:]):
        raise ValueError("cannot aggregate trials with inconsistent fixed T_map_world transforms")

    summary = _summarize_scored_trials(scored_trials, expected_trials)
    summary["cohort_identity"] = {
        "mode": "run_id_repetition_trial_id" if tagged_cohort else "legacy_trial_id",
        "unique_trial_identity_count": len(trial_ids),
        "run_ids": sorted({key[0] for key in trial_ids if key[0] is not None}),
        "repetition_counts": {},
    }
    if tagged_cohort:
        for run_id, repetition in sorted({(key[0], key[1]) for key in trial_ids if key[0] is not None}):
            group = [trial for trial in scored_trials
                     if trial.get("run_id") == run_id and trial.get("repetition") == repetition]
            summary["cohort_identity"]["repetition_counts"][str(repetition)] = {
                "run_id": run_id,
                "observed_trial_count": len(group),
                "action_success_count": sum(1 for trial in group if trial.get("action_success") is True),
                "scored_success_count": sum(1 for trial in group if trial.get("success") is True),
                "confirmed_correct_within_15s_count": sum(
                    1 for trial in group if trial.get("correct_within_15s") is True
                ),
            }
    summary["map_sha256"] = next(iter(map_hashes or status_map_hashes)) if (map_hashes or status_map_hashes) else None
    summary["T_map_world"] = (
        {"translation_m": list(transforms[0].position), "quaternion_xyzw": list(transforms[0].orientation)}
        if transforms else None
    )
    report = {
        "schema_version": 1,
        "trials_source": os.path.abspath(trials_path),
        "pose_errors_source": os.path.abspath(pose_errors_path),
        "summary": summary,
        "trials": scored_trials,
    }
    parent = os.path.dirname(os.path.abspath(output_prefix))
    os.makedirs(parent, exist_ok=True)
    json_path = output_prefix + ".summary.json"
    csv_path = output_prefix + ".trials.csv"
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    csv_fields = [
        "run_id", "repetition", "trial_id", "trial_index", "source_trial_index",
        "requested_pose", "initial_stamp_ns", "final_stamp_ns", "action_goal_start_ros_ns",
        "action_goal_started_wall_monotonic_sec", "pose_stamp_ns",
        "action_elapsed_sec", "correct_within_15s", "action_success", "action_reason", "timeout", "evaluation_valid",
        "pose_evaluation_success", "pose_evaluation_failure_reason",
        "registration_attempt_outcome", "pose_score_run_id", "pose_score_repetition", "pose_score_trial_id",
        "success", "outcome", "failure_reason", "pose_interpolation_validity",
        *_ERROR_FIELDS, "position_threshold_m", "orientation_threshold_deg",
        "timing_sec", "input_mode", "action_window_exists", "action_timeout_source",
        "raw_input_observation", "reset_attempts", "algorithm_process", "lio_pipeline", "gazebo_process_load",
        "trial_evaluator_process", "diagnostic_recorder_process", "frontend_process",
        "status_events", "terminal_status", "raw_frontend", "stage_wall_sec", "stage_ros_sec",
        "rtf", "environment",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_fields, extrasaction="ignore")
        writer.writeheader()
        for row in scored_trials:
            writer.writerow({
                key: json.dumps(row.get(key), separators=(",", ":"))
                if isinstance(row.get(key), (dict, list)) else row.get(key)
                for key in csv_fields
            })
    return report


def save_batch_plots(report: dict[str, Any], output_prefix: str) -> list[str]:
    """Write optional PNG reports with a non-interactive backend."""
    try:
        import matplotlib
        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("--plots requires matplotlib") from error

    rows = report.get("trials", [])
    valid_rows = [row for row in rows if row.get("evaluation_valid") is True]
    plot_paths: list[str] = []

    error_specs = (
        ("position_3d_error_m", "3D position error (m)"),
        ("orientation_geodesic_error_deg", "SO(3) orientation error (deg)"),
    )
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for axis, (field, label) in zip(axes, error_specs):
        values = sorted(
            value for row in valid_rows
            if (value := _optional_float(row.get(field))) is not None
        )
        if values:
            cdf = [(index + 1) / len(values) for index in range(len(values))]
            axis.step(values, cdf, where="post")
        else:
            axis.text(0.5, 0.5, "No valid scored poses", ha="center", va="center", transform=axis.transAxes)
        axis.set_xlabel(label)
        axis.set_ylabel("Empirical CDF")
        axis.set_ylim(0.0, 1.02)
        axis.grid(True, alpha=0.3)
    figure.suptitle("Relocalization pose error distributions")
    figure.tight_layout()
    error_path = output_prefix + ".error_cdf.png"
    figure.savefig(error_path, dpi=160)
    plt.close(figure)
    plot_paths.append(error_path)

    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    stage_fields = (
        "static", "sweep", "retrieval", "gicp", "confirmation", "stop_settle", "search_total", "total",
    )
    stage_values = []
    stage_labels = []
    for stage in stage_fields:
        values = []
        for row in rows:
            timing = row.get("timing_sec")
            item = timing.get(stage) if isinstance(timing, dict) else None
            if stage == "total" and isinstance(timing, dict):
                item = {"wall": timing.get("total_wall")}
            value = _optional_float(item.get("wall")) if isinstance(item, dict) else None
            if value is not None:
                values.append(value)
        if values:
            stage_values.append(values)
            stage_labels.append(stage)
    if stage_values:
        axes[0].boxplot(stage_values, labels=stage_labels, showfliers=False)
        axes[0].tick_params(axis="x", rotation=25)
        axes[0].set_ylabel("Wall time (s)")
        axes[0].grid(True, axis="y", alpha=0.3)
    else:
        axes[0].text(0.5, 0.5, "No stage timing samples", ha="center", va="center", transform=axes[0].transAxes)
    resource_specs = (
        ("algorithm_process", "Algorithm"),
        ("lio_pipeline", "Point-LIO pipeline"),
        ("gazebo_process_load", "Gazebo"),
    )
    resource_values = []
    resource_labels = []
    for field, label in resource_specs:
        values = [
            value for row in rows
            if (value := _optional_float(_nested(row, field, "cpu_percent"))) is not None
        ]
        if values:
            resource_values.append(values)
            resource_labels.append(label)
    if resource_values:
        axes[1].boxplot(resource_values, labels=resource_labels, showfliers=False)
        axes[1].tick_params(axis="x", rotation=20)
        axes[1].set_ylabel("CPU use (%) of one core")
        axes[1].grid(True, axis="y", alpha=0.3)
    else:
        axes[1].text(0.5, 0.5, "No process CPU samples", ha="center", va="center", transform=axes[1].transAxes)
    figure.suptitle("Stage wall time and process CPU distributions")
    figure.tight_layout()
    performance_path = output_prefix + ".performance.png"
    figure.savefig(performance_path, dpi=160)
    plt.close(figure)
    plot_paths.append(performance_path)
    return plot_paths


try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # Enables system-Python unit tests on machines without ROS.
    rclpy = None  # type: ignore[assignment]
    PoseStamped = Any  # type: ignore[misc,assignment]
    Odometry = Any  # type: ignore[misc,assignment]
    Node = object  # type: ignore[assignment,misc]
    Parameter = Any  # type: ignore[assignment,misc]
    qos_profile_sensor_data = None


if rclpy is not None:
    @dataclass
    class _PendingEstimate:
        message: Any


    class PoseEvaluationNode(Node):
        def __init__(self) -> None:
            super().__init__("global_relocalization_pose_evaluator")
            self.declare_parameter("pose_topic", "/global_relocalization/pose")
            self.declare_parameter("ground_truth_topic", "/ground_truth/odometry")
            self.declare_parameter("output_prefix", "global_relocalization_evaluation")
            self.declare_parameter("run_id", "")
            self.declare_parameter("repetition", 0)
            self.declare_parameter("map_manifest_path", "")
            self.declare_parameter("world_frame", "world")
            self.declare_parameter("map_frame", "map")
            self.declare_parameter("base_frame", "base_footprint")
            self.declare_parameter("map_world_transform_configured", False)
            self.declare_parameter("map_world_translation", [0.0, 0.0, 0.0])
            self.declare_parameter("map_world_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
            self.declare_parameter("max_interpolation_gap_sec", 0.25)
            self.declare_parameter("pending_gt_timeout_sec", 1.0)
            self.declare_parameter("position_threshold_m", 0.10)
            self.declare_parameter("orientation_threshold_deg", 2.0)
            self.declare_parameter("max_gt_samples", 10000)

            self.map_frame = str(self.get_parameter("map_frame").value)
            self.cohort_run_id = str(self.get_parameter("run_id").value)
            repetition_value = self.get_parameter("repetition").value
            if self.cohort_run_id or repetition_value:
                if not self.cohort_run_id.strip():
                    raise ValueError("repetition requires a non-empty run_id parameter")
                if isinstance(repetition_value, bool) or not isinstance(repetition_value, int) or repetition_value <= 0:
                    raise ValueError("run_id requires a positive integer repetition parameter")
                self.cohort_repetition: Optional[int] = repetition_value
            else:
                self.cohort_repetition = None
            self.world_frame = str(self.get_parameter("world_frame").value)
            self.base_frame = str(self.get_parameter("base_frame").value)
            manifest_path = str(self.get_parameter("map_manifest_path").value)
            explicitly_configured = bool(self.get_parameter("map_world_transform_configured").value)
            configured_transform = Pose.make(
                self.get_parameter("map_world_translation").value,
                self.get_parameter("map_world_quaternion_xyzw").value,
            )
            if manifest_path:
                self.map_from_world, self.alignment_metadata = load_alignment_manifest(
                    manifest_path, self.map_frame, self.world_frame, self.base_frame
                )
                if explicitly_configured and not transforms_match(configured_transform, self.map_from_world):
                    raise ValueError(
                        "explicit T_map_world differs from authoritative map manifest "
                        "(translation tolerance 1e-6 m, rotation tolerance 1e-6 rad)"
                    )
            else:
                if not explicitly_configured:
                    raise ValueError(
                        "provide map_manifest_path, or set map_world_transform_configured:=true and "
                        "explicitly set T_map_world translation/quaternion"
                    )
                self.map_from_world = configured_transform
                self.alignment_metadata = {
                    "source": "explicit_parameters",
                    "verification": "unverified_alignment_without_map_manifest",
                }
            self.alignment_metadata.update({
                "map_frame": self.map_frame,
                "world_frame": self.world_frame,
                "base_frame": self.base_frame,
            })
            self.position_threshold_m = float(self.get_parameter("position_threshold_m").value)
            self.orientation_threshold_deg = float(self.get_parameter("orientation_threshold_deg").value)
            self.pending_timeout_ns = int(float(self.get_parameter("pending_gt_timeout_sec").value) * 1e9)
            if self.position_threshold_m <= 0 or not math.isfinite(self.position_threshold_m):
                raise ValueError("position_threshold_m must be positive and finite")
            if self.orientation_threshold_deg <= 0 or not math.isfinite(self.orientation_threshold_deg):
                raise ValueError("orientation_threshold_deg must be positive and finite")
            if self.pending_timeout_ns <= 0:
                raise ValueError("pending_gt_timeout_sec must be positive")

            self.gt_buffer = GroundTruthBuffer(
                max_samples=int(self.get_parameter("max_gt_samples").value),
                max_gap_sec=float(self.get_parameter("max_interpolation_gap_sec").value),
            )
            self.pending: list[_PendingEstimate] = []
            self.sample_index = 0
            self.writer = ResultWriter(str(self.get_parameter("output_prefix").value))
            self.estimate_sub = self.create_subscription(
                PoseStamped,
                str(self.get_parameter("pose_topic").value),
                self._estimate_callback,
                20,
            )
            self.gt_sub = self.create_subscription(
                Odometry,
                str(self.get_parameter("ground_truth_topic").value),
                self._ground_truth_callback,
                qos_profile_sensor_data,
            )
            self.pending_timer = self.create_timer(0.05, self._resolve_pending)
            self.get_logger().info(
                "Scoring only estimate PoseStamped and GT Odometry; T_map_world="
                f"translation={self.map_from_world.position}, quaternion_xyzw={self.map_from_world.orientation}; "
                f"alignment={self.alignment_metadata}"
            )

        @staticmethod
        def _stamp_ns(header: Any) -> int:
            return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

        @staticmethod
        def _pose_from_ros(position: Any, orientation: Any) -> Pose:
            return Pose.make(
                (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
            )

        def _ground_truth_callback(self, message: Any) -> None:
            if message.header.frame_id != self.world_frame:
                self.get_logger().warning(
                    f"Ignoring GT frame {message.header.frame_id!r}; expected {self.world_frame!r}"
                )
                return
            if message.child_frame_id != self.base_frame:
                self.get_logger().warning(
                    f"Ignoring GT child frame {message.child_frame_id!r}; expected {self.base_frame!r}"
                )
                return
            try:
                sample = TimedPose(
                    self._stamp_ns(message.header),
                    self._pose_from_ros(message.pose.pose.position, message.pose.pose.orientation),
                )
                self.gt_buffer.insert(sample)
            except (TypeError, ValueError) as error:
                self.get_logger().warning(f"Ignoring malformed GT sample: {error}")
                return
            self._resolve_pending()

        def _estimate_callback(self, message: Any) -> None:
            stamp_ns = self._stamp_ns(message.header)
            try:
                estimate = self._pose_from_ros(message.pose.position, message.pose.orientation)
            except (TypeError, ValueError) as error:
                self._write_result(
                    message, stamp_ns, None,
                    LookupResult(None, "not_queried_invalid_estimate"),
                    failure_reason=f"invalid_estimate_pose:{error}",
                )
                return
            if message.header.frame_id != self.map_frame:
                self._write_result(
                    message, stamp_ns, estimate,
                    LookupResult(None, "not_queried_wrong_pose_frame"),
                    failure_reason=f"estimate_frame_mismatch:{message.header.frame_id!r}",
                )
                return
            lookup = self.gt_buffer.lookup(stamp_ns)
            if lookup.validity == "need_future_bracket":
                self.pending.append(_PendingEstimate(message))
                return
            self._write_result(message, stamp_ns, estimate, lookup)

        def _resolve_pending(self) -> None:
            if not self.pending:
                return
            now_ns = self.get_clock().now().nanoseconds
            remaining: list[_PendingEstimate] = []
            for pending in self.pending:
                stamp_ns = self._stamp_ns(pending.message.header)
                lookup = self.gt_buffer.lookup(stamp_ns)
                if lookup.validity == "need_future_bracket":
                    if timestamp_timeout_elapsed(
                        stamp_ns, now_ns, self.pending_timeout_ns / 1_000_000_000.0
                    ):
                        estimate = self._pose_from_ros(
                            pending.message.pose.position, pending.message.pose.orientation
                        )
                        self._write_result(
                            pending.message, stamp_ns, estimate, lookup,
                            failure_reason="ground_truth_future_bracket_timeout_no_extrapolation",
                        )
                    else:
                        remaining.append(pending)
                    continue
                estimate = self._pose_from_ros(
                    pending.message.pose.position, pending.message.pose.orientation
                )
                self._write_result(pending.message, stamp_ns, estimate, lookup)
            self.pending = remaining

        def _write_result(
            self,
            message: Any,
            stamp_ns: int,
            estimate: Optional[Pose],
            lookup: LookupResult,
            failure_reason: Optional[str] = None,
        ) -> None:
            self.sample_index += 1
            record = make_sample_record(
                sample_index=self.sample_index,
                pose_stamp_ns=stamp_ns,
                pose_frame=str(message.header.frame_id),
                interpolation=lookup,
                estimate_map=estimate,
                map_from_world=self.map_from_world,
                position_threshold_m=self.position_threshold_m,
                orientation_threshold_deg=self.orientation_threshold_deg,
                base_frame=self.base_frame,
                failure_reason=failure_reason,
                alignment_metadata=self.alignment_metadata,
                run_id=self.cohort_run_id or None,
                repetition=self.cohort_repetition,
            )
            self.writer.write(record)
            if record["outcome"] == "success":
                self.get_logger().info(
                    f"sample {self.sample_index}: PASS, 3D={record['position_3d_error_m']:.4f} m, "
                    f"SO(3)={record['orientation_geodesic_error_deg']:.3f} deg"
                )
            elif record["outcome"] == "failure":
                self.get_logger().warning(
                    f"sample {self.sample_index}: FAIL, reason={record['failure_reason']}"
                )
            else:
                self.get_logger().error(
                    f"sample {self.sample_index}: INVALID, reason={record['failure_reason']}"
                )

        def destroy_node(self) -> bool:
            for pending in self.pending:
                stamp_ns = self._stamp_ns(pending.message.header)
                estimate = self._pose_from_ros(
                    pending.message.pose.position, pending.message.pose.orientation
                )
                lookup = self.gt_buffer.lookup(stamp_ns)
                self._write_result(
                    pending.message, stamp_ns, estimate, lookup,
                    failure_reason="evaluator_shutdown_before_future_gt_bracket",
                )
            self.pending.clear()
            self.writer.close()
            return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> int:
    argv = list(args) if args is not None else sys.argv[1:]
    if "--aggregate" in argv:
        parser = argparse.ArgumentParser(description="Join pose scores with a batch trial ledger")
        parser.add_argument("--aggregate", action="store_true")
        parser.add_argument("--trials", required=True, help="one JSON object per trial (JSONL)")
        parser.add_argument("--pose-errors", required=True, help="topic evaluator output (JSONL)")
        parser.add_argument("--output-prefix", required=True)
        parser.add_argument("--expected-trials", type=int, default=120)
        parser.add_argument("--plots", action="store_true", help="also create optional matplotlib PNG plots")
        options = parser.parse_args(argv)
        try:
            report = aggregate_batch(
                options.trials, options.pose_errors, options.output_prefix,
                expected_trials=options.expected_trials,
            )
            plot_paths = save_batch_plots(report, options.output_prefix) if options.plots else []
        except Exception as error:
            print(f"batch aggregation failed: {error}", file=sys.stderr)
            return 2
        batch = report["summary"]["batch"]
        print(
            f"aggregated {batch['observed_trials']}/{batch['expected_trials']} trials; "
            f"success rate={batch['success_rate_percent']}; "
            f"outputs={options.output_prefix}.summary.json and {options.output_prefix}.trials.csv"
        )
        if plot_paths:
            print("plots=" + ", ".join(plot_paths))
        return 0
    if rclpy is None:
        print("ROS 2 Python packages are required to run the topic evaluator", file=sys.stderr)
        return 2
    rclpy.init(args=argv)
    node: Optional[PoseEvaluationNode] = None
    try:
        node = PoseEvaluationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"pose evaluation failed to start: {error}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
