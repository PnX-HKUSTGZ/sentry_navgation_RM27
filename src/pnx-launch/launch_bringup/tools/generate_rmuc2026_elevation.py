#!/usr/bin/env python3
# Copyright 2026 PNX Robot Team
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

"""Build RMUC2026's 2.5D drivable-ground prior from its SDF and STL."""

import argparse
from collections import defaultdict, deque
import hashlib
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml


NO_DATA = 65535
VALUE_SCALE = 0.001
VALUE_OFFSET = -1.0


def _read_pgm(path):
    with path.open("rb") as stream:
        tokens = []
        while len(tokens) < 4:
            line = stream.readline()
            if not line:
                raise RuntimeError(f"incomplete PGM header: {path}")
            if not line.lstrip().startswith(b"#"):
                tokens.extend(line.split())
        if tokens[0] != b"P5":
            raise RuntimeError(f"only binary P5 occupancy maps are supported: {path}")
        width, height, max_value = (int(value) for value in tokens[1:4])
        if not 0 < max_value < 256:
            raise RuntimeError(f"occupancy PGM max value must be below 256: {path}")
        pixels = np.frombuffer(stream.read(), dtype=np.uint8)
    if pixels.size != width * height:
        raise RuntimeError(f"PGM raster size does not match its header: {path}")
    return pixels.reshape((height, width)), max_value


def _pose(element):
    if element is None or not element.text:
        return np.zeros(6, dtype=np.float64)
    values = np.fromstring(element.text, sep=" ", dtype=np.float64)
    if values.size != 6:
        raise RuntimeError("SDF pose must contain x y z roll pitch yaw")
    return values


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _transform_points(points, poses):
    transformed = points
    for pose in poses:
        transformed = transformed @ _rotation(*pose[3:]).T + pose[:3]
    return transformed


def _world_geometry(path):
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError(f"SDF has no world: {path}")

    ground = world.find("./model[@name='ground_plane']")
    map_model = world.find("./model[@name='map']")
    if ground is None or map_model is None:
        raise RuntimeError("world must contain ground_plane and map models")

    ground_collision = ground.find("./link/collision")
    plane_size_node = ground_collision.find("./geometry/plane/size")
    plane_normal_node = ground_collision.find("./geometry/plane/normal")
    if plane_size_node is None or plane_normal_node is None:
        raise RuntimeError("ground_plane collision must be an SDF plane")
    plane_size = np.fromstring(plane_size_node.text, sep=" ", dtype=np.float64)
    plane_normal = np.fromstring(plane_normal_node.text, sep=" ", dtype=np.float64)
    if plane_size.size != 2 or plane_normal.size != 3:
        raise RuntimeError("invalid ground_plane size or normal")
    ground_model_pose = _pose(ground.find("pose"))
    ground_link_pose = _pose(ground.find("./link/pose"))
    ground_collision_pose = _pose(ground_collision.find("pose"))
    if not np.allclose(plane_normal, [0.0, 0.0, 1.0], atol=1.0e-9):
        raise RuntimeError("only an upward horizontal world ground plane is supported")
    if not np.allclose(
        np.concatenate(
            [ground_model_pose[3:], ground_link_pose[3:], ground_collision_pose[3:]]
        ),
        0.0,
        atol=1.0e-9,
    ):
        raise RuntimeError("rotated ground planes are not supported")
    ground_center = (
        ground_model_pose[:3] + ground_link_pose[:3] + ground_collision_pose[:3]
    )

    map_link = map_model.find("link")
    collision = map_link.find("collision")
    scale_node = collision.find("./geometry/mesh/scale")
    scale = (
        np.fromstring(scale_node.text, sep=" ", dtype=np.float64)
        if scale_node is not None
        else np.ones(3, dtype=np.float64)
    )
    if scale.size != 3 or np.any(scale <= 0.0):
        raise RuntimeError("map mesh scale must contain three positive values")
    poses = [
        _pose(collision.find("pose")),
        _pose(map_link.find("pose")),
        _pose(map_model.find("pose")),
    ]
    return ground_center, plane_size, scale, poses


def _read_binary_stl(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise RuntimeError(f"binary STL is too small: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangle_count * 50:
        raise RuntimeError(f"STL byte count does not match its header: {path}")
    record = np.dtype(
        [("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    return np.frombuffer(data, dtype=record, offset=84, count=triangle_count)[
        "vertices"
    ].astype(np.float64)


def _rasterize_surface_candidates(
    triangles, known_free, origin, resolution, base_height, max_slope, max_height
):
    """
    Rasterize every drivable STL surface instead of only its extrema.

    RMUC2026 contains short ramps underneath bridge/roof geometry.  Keeping
    only the highest and lowest intersection loses that middle ramp surface,
    which leaves a no-data stripe exactly at an underpass entrance.
    """
    height, width = known_free.shape
    upper = base_height.copy()
    candidate_samples = defaultdict(list)
    edges_a = triangles[:, 1] - triangles[:, 0]
    edges_b = triangles[:, 2] - triangles[:, 0]
    normals = np.cross(edges_a, edges_b)
    normal_lengths = np.linalg.norm(normals, axis=1)
    upward = normals[:, 2] > 0.0
    slope_ok = normals[:, 2] / np.maximum(normal_lengths, 1.0e-12) >= math.cos(
        math.radians(max_slope)
    )
    height_ok = np.max(triangles[:, :, 2], axis=1) <= max_height

    for triangle in triangles[upward & slope_ok & height_ok]:
        min_x, max_x = np.min(triangle[:, 0]), np.max(triangle[:, 0])
        min_y, max_y = np.min(triangle[:, 1]), np.max(triangle[:, 1])
        col_min = max(0, math.floor((min_x - origin[0]) / resolution))
        col_max = min(width - 1, math.floor((max_x - origin[0]) / resolution))
        map_row_min = max(0, math.floor((min_y - origin[1]) / resolution))
        map_row_max = min(height - 1, math.floor((max_y - origin[1]) / resolution))
        if col_max < col_min or map_row_max < map_row_min:
            continue

        cols = np.arange(col_min, col_max + 1)
        map_rows = np.arange(map_row_min, map_row_max + 1)
        x, y = np.meshgrid(
            origin[0] + (cols + 0.5) * resolution,
            origin[1] + (map_rows + 0.5) * resolution,
        )
        a, b, c = triangle
        denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(denominator) < 1.0e-12:
            continue
        weight_a = (
            (b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])
        ) / denominator
        weight_b = (
            (c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])
        ) / denominator
        weight_c = 1.0 - weight_a - weight_b
        inside = (weight_a >= -1.0e-7) & (weight_b >= -1.0e-7) & (weight_c >= -1.0e-7)
        triangle_z = weight_a * a[2] + weight_b * b[2] + weight_c * c[2]
        image_rows = (height - 1 - map_rows)[:, None]
        block = upper[image_rows, cols[None, :]]
        np.maximum(block, np.where(inside, triangle_z, -np.inf), out=block)
        upper[image_rows, cols[None, :]] = block

        local_rows, local_cols = np.nonzero(inside)
        for local_row, local_col in zip(local_rows, local_cols):
            image_row = height - 1 - int(map_rows[local_row])
            image_col = int(cols[local_col])
            if known_free[image_row, image_col]:
                candidate_samples[(image_row, image_col)].append(
                    float(triangle_z[local_row, local_col])
                )

    candidates = {}
    for index, samples in candidate_samples.items():
        samples.sort()
        unique_samples = []
        for sample in samples:
            if not unique_samples or abs(sample - unique_samples[-1]) > 1.0e-4:
                unique_samples.append(sample)
        candidates[index] = tuple(unique_samples)

    upper[~known_free] = np.nan
    return upper, candidates


def _retain_slope_connected_surface(
    upper,
    known_free,
    base_height,
    resolution,
    max_slope,
    seed_tolerance,
    step_tolerance,
):
    height, width = known_free.shape
    reachable = np.zeros((height, width), dtype=bool)
    seeds = known_free & np.isfinite(upper) & np.isfinite(base_height)
    seeds &= np.abs(upper - base_height) <= seed_tolerance
    queue = deque(zip(*np.where(seeds)))
    reachable[seeds] = True
    neighbors = (
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    )
    slope = math.tan(math.radians(max_slope))
    while queue:
        row, col = queue.popleft()
        for delta_row, delta_col in neighbors:
            next_row, next_col = row + delta_row, col + delta_col
            if not (0 <= next_row < height and 0 <= next_col < width):
                continue
            if reachable[next_row, next_col] or not known_free[next_row, next_col]:
                continue
            if not np.isfinite(upper[next_row, next_col]):
                continue
            allowed_delta = (
                slope * resolution * math.hypot(delta_row, delta_col) + step_tolerance
            )
            if abs(upper[next_row, next_col] - upper[row, col]) <= allowed_delta:
                reachable[next_row, next_col] = True
                queue.append((next_row, next_col))
    return reachable


def _recover_connected_surface_candidates(
    upper,
    candidates,
    base_height,
    support,
    known_free,
    resolution,
    max_slope,
    step_tolerance,
):
    """Recover a slope-continuous surface hidden by overlapping geometry."""
    selected = upper.copy()
    recovered = support.copy()
    queue = deque(zip(*np.where(support)))
    neighbors = (
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    )
    slope = math.tan(math.radians(max_slope))
    height, width = known_free.shape
    while queue:
        row, col = queue.popleft()
        for delta_row, delta_col in neighbors:
            next_row, next_col = row + delta_row, col + delta_col
            if not (0 <= next_row < height and 0 <= next_col < width):
                continue
            if recovered[next_row, next_col] or not known_free[next_row, next_col]:
                continue
            cell_candidates = list(candidates.get((next_row, next_col), ()))
            base_candidate = base_height[next_row, next_col]
            if np.isfinite(base_candidate):
                cell_candidates.append(float(base_candidate))
            if not cell_candidates:
                continue
            allowed_delta = (
                slope * resolution * math.hypot(delta_row, delta_col) + step_tolerance
            )
            previous_height = selected[row, col]
            candidate = min(
                cell_candidates,
                key=lambda value: (abs(value - previous_height), -value),
            )
            if abs(candidate - previous_height) > allowed_delta:
                continue
            selected[next_row, next_col] = candidate
            recovered[next_row, next_col] = True
            queue.append((next_row, next_col))
    return selected, recovered


def _encode(height, support_mask):
    encoded = np.full(height.shape, NO_DATA, dtype=np.uint16)
    values = np.rint((height[support_mask] - VALUE_OFFSET) / VALUE_SCALE)
    if np.any(values < 0) or np.any(values >= NO_DATA):
        raise RuntimeError("ground height cannot be represented by configured encoding")
    encoded[support_mask] = values.astype(np.uint16)
    return encoded


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(args):
    config = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    pixels, max_gray = _read_pgm(args.map_pgm)
    occupancy = (
        pixels.astype(np.float64) / max_gray
        if config["negate"]
        else (max_gray - pixels.astype(np.float64)) / max_gray
    )
    known_free = occupancy < float(config["free_thresh"])
    resolution = float(config["resolution"])
    origin = np.asarray(config["origin"], dtype=np.float64)
    if abs(origin[2]) > 1.0e-12:
        raise RuntimeError("this generator currently requires a zero-yaw map origin")

    ground_center, ground_size, mesh_scale, mesh_poses = _world_geometry(args.world)
    rows, cols = np.indices(pixels.shape)
    x = origin[0] + (cols + 0.5) * resolution
    y = origin[1] + (pixels.shape[0] - rows - 0.5) * resolution
    on_ground = (np.abs(x - ground_center[0]) <= 0.5 * ground_size[0] + 1.0e-9) & (
        np.abs(y - ground_center[1]) <= 0.5 * ground_size[1] + 1.0e-9
    )
    base_height = np.where(on_ground, ground_center[2], np.nan)

    triangles = _read_binary_stl(args.stl) * mesh_scale
    triangles = _transform_points(triangles, mesh_poses)
    upper, candidates = _rasterize_surface_candidates(
        triangles,
        known_free,
        origin,
        resolution,
        base_height,
        args.max_slope,
        args.max_height,
    )
    support = _retain_slope_connected_surface(
        upper,
        known_free,
        base_height,
        resolution,
        args.max_slope,
        args.seed_tolerance,
        args.step_tolerance,
    )
    upper, support = _recover_connected_surface_candidates(
        upper,
        candidates,
        base_height,
        support,
        known_free,
        resolution,
        args.max_slope,
        args.step_tolerance,
    )
    encoded = _encode(upper, support)
    comments = (
        "P5\n"
        "# RMUC2026 drivable-ground elevation prior; uint16 big-endian\n"
        f"# pgm_sha256={_sha256(args.map_pgm)}\n"
        f"# stl_sha256={_sha256(args.stl)}\n"
        f"# world_sha256={_sha256(args.world)}\n"
        f"# scale={VALUE_SCALE:.3f} offset={VALUE_OFFSET:.3f} no_data={NO_DATA}\n"
        f"{pixels.shape[1]} {pixels.shape[0]}\n65535\n"
    ).encode("ascii")
    output = comments + encoded.astype(">u2").tobytes()
    return output, upper, support, known_free


def main():
    package = Path(__file__).resolve().parents[1]
    source = package.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map-yaml", type=Path, default=package / "map/simulation/RMUC2026.yaml"
    )
    parser.add_argument(
        "--map-pgm", type=Path, default=package / "map/simulation/RMUC2026.pgm"
    )
    parser.add_argument(
        "--stl",
        type=Path,
        default=source / "rm_27_stimulation/meshes/RMUC2026_world/meshes/RMUC2026.stl",
    )
    parser.add_argument(
        "--world",
        type=Path,
        default=source / "rm_27_stimulation/world/RMUC2026_world/RMUC2026_world.world",
    )
    parser.add_argument(
        "--output", type=Path, default=package / "map/simulation/RMUC2026_elevation.pgm"
    )
    parser.add_argument("--max-slope", type=float, default=28.0)
    parser.add_argument("--max-height", type=float, default=0.60)
    parser.add_argument("--seed-tolerance", type=float, default=0.04)
    parser.add_argument("--step-tolerance", type=float, default=0.005)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    output, height, support, known_free = build(args)
    if args.check:
        if not args.output.exists() or args.output.read_bytes() != output:
            print(f"stale elevation prior: {args.output}", file=sys.stderr)
            return 1
    else:
        args.output.write_bytes(output)
    supported_heights = height[support]
    print(
        f"{args.output}: support={int(support.sum())}/{int(known_free.sum())} "
        f"known-free cells, z=[{supported_heights.min():.3f}, "
        f"{supported_heights.max():.3f}] m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
