#!/usr/bin/env python3
"""Generate a spatially separated randomized holdout set from the field map.

Only historical ``trials*.json`` and ``result.json`` files are read. In
particular, large capture and bag files are never opened. Sampling is based on
map occupancy, known elevation, and geometric distance only; trial outcomes are
not used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
import yaml


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_MAP = HERE / "config/holdout_sampling.yaml"
DEFAULT_HISTORY_ROOTS = [HERE / "data", HERE / "config"]
BASE_FOOTPRINT_OFFSET_M = 0.08
MIN_CLEARANCE_M = 0.5
DEFAULT_MIN_DISTANCE_M = 0.25


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def history_files(roots: list[Path]) -> list[Path]:
    found: set[Path] = set()
    for root in roots:
        if root.is_file():
            if root.name == "result.json" or (root.name.startswith("trials") and root.suffix == ".json"):
                found.add(root.resolve())
            continue
        if not root.is_dir():
            raise FileNotFoundError(f"history root does not exist: {root}")
        for path in root.rglob("*.json"):
            if path.name == "result.json" or (path.name.startswith("trials") and path.suffix == ".json"):
                found.add(path.resolve())
    return sorted(found)


def requested_xy(document: object) -> list[tuple[float, float]]:
    """Extract only requested_pose x/y values, without examining outcomes."""
    poses: list[tuple[float, float]] = []

    def visit(value: object) -> None:
        if isinstance(value, dict):
            pose = value.get("requested_pose")
            if isinstance(pose, dict) and "x" in pose and "y" in pose:
                try:
                    x, y = float(pose["x"]), float(pose["y"])
                except (TypeError, ValueError) as error:
                    raise ValueError(f"invalid requested_pose coordinates: {pose}") from error
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError(f"non-finite requested_pose coordinates: {pose}")
                poses.append((x, y))
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(document)
    return poses


def load_history(roots: list[Path]) -> tuple[np.ndarray, list[dict]]:
    rows: list[tuple[float, float]] = []
    sources = []
    for path in history_files(roots):
        try:
            document = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError(f"cannot read historical pose source {path}: {error}") from error
        poses = requested_xy(document)
        rows.extend(poses)
        sources.append({"path": str(path), "sha256": sha256_file(path), "requested_pose_count": len(poses)})
    points = np.asarray(rows, dtype=np.float64).reshape((-1, 2))
    return points, sources


def map_candidates(map_yaml: Path, min_clearance: float) -> tuple[np.ndarray, np.ndarray, dict]:
    config = yaml.safe_load(map_yaml.read_text())
    resolution = float(config["resolution"])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("map resolution must be positive and finite")
    origin = config["origin"]
    ox, oy, yaw = map(float, origin[:3])
    if abs(yaw) > 1e-9:
        raise ValueError("rotated occupancy grids are not supported by this sampler")

    occupancy_path = (map_yaml.parent / config["image"]).resolve()
    occupancy = np.asarray(Image.open(occupancy_path))[::-1]
    if occupancy.ndim != 2:
        raise ValueError("occupancy image must be single channel")
    elevation = config.get("ground_elevation", {}).get("grid")
    if not elevation:
        raise ValueError("map must contain a known ground_elevation grid")
    elevation_path = (map_yaml.parent / elevation["image"]).resolve()
    height_image = np.asarray(Image.open(elevation_path))[::-1]
    if height_image.shape != occupancy.shape:
        raise ValueError("occupancy and elevation image dimensions differ")
    no_data = int(elevation["no_data"])
    known = height_image != no_data
    # Unknown elevation is not a valid spawn surface and is also treated as a
    # clearance obstacle, so EDT must see it as blocked before distances form.
    free = (occupancy > 250) & known
    free[[0, -1], :] = False
    free[:, [0, -1]] = False
    valid = distance_transform_edt(free) * resolution >= min_clearance

    height, width = occupancy.shape
    rows, cols = np.nonzero(valid)
    xs = ox + (cols + 0.5) * resolution
    ys = oy + (rows + 0.5) * resolution
    negative_x = xs < 0.0
    rows, cols, xs, ys = rows[negative_x], cols[negative_x], xs[negative_x], ys[negative_x]
    scale = float(elevation["scale"])
    offset = float(elevation["offset"])
    ground_z = height_image[rows, cols].astype(np.float64) * scale + offset
    if not np.isfinite(ground_z).all():
        raise ValueError("elevation grid contains non-finite ground heights")
    points = np.column_stack((xs, ys))
    metadata = {
        "occupancy_image": str(occupancy_path),
        "elevation_image": str(elevation_path),
        "map_yaml_sha256": sha256_file(map_yaml),
        "occupancy_sha256": sha256_file(occupancy_path),
        "elevation_sha256": sha256_file(elevation_path),
        "resolution_m": resolution,
        "origin": [ox, oy, yaw],
        "map_shape_rows_cols": [int(height), int(width)],
    }
    geometry_sources = []
    if config.get("collision_support"):
        from actual_collision_support import filter_supported_positions
        options = dict(config["collision_support"])
        for key in ("world_path", "robot_urdf_path"):
            options[key] = (map_yaml.parent / options[key]).resolve()
        if options.get("model_roots") is not None:
            options["model_roots"] = [(map_yaml.parent / value).resolve()
                                      for value in options["model_roots"]]
        supported, actual_heights, geometry = filter_supported_positions(points, **options)
        if abs(float(geometry["robot_envelope"]["base_height_above_contact"]) - BASE_FOOTPRINT_OFFSET_M) > 1e-6:
            raise ValueError("configured spawn offset differs from the actual robot contact geometry")
        points, ground_z = points[supported], actual_heights[supported]
        metadata["collision_support"] = geometry
        metadata["spawn_separation_margin_m"] = float(options.get("separation_margin", 0.002))
        geometry_sources = [Path(name) for name in sorted(geometry["source_hashes"])]
    map_digest = hashlib.sha256()
    for source in (map_yaml, occupancy_path, elevation_path, *geometry_sources):
        relative_name = Path(os.path.relpath(source, map_yaml.parent)).as_posix()
        map_digest.update(relative_name.encode())
        map_digest.update(b"\0")
        with source.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                map_digest.update(block)
    metadata["map_content_sha256"] = map_digest.hexdigest()
    return points, ground_z, metadata


def sample_trials(
    *, count: int, seed: int, map_yaml: Path, history_roots: list[Path], min_distance: float,
) -> dict:
    if isinstance(count, bool) or not isinstance(count, (int, np.integer)) or count < 1:
        raise ValueError("count must be at least 1")
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)):
        raise ValueError("seed must be an integer")
    if seed < 0 or seed > (2**64 - 1):
        raise ValueError("seed must be in the unsigned 64-bit range")
    if not math.isfinite(min_distance) or min_distance < DEFAULT_MIN_DISTANCE_M:
        raise ValueError(f"min-distance must be at least {DEFAULT_MIN_DISTANCE_M:g} m")
    # Keep a tiny numerical guard so serialized floating-point coordinates do
    # not round a nominal exact-boundary separation below the promised floor.
    spacing_floor = min_distance + 1e-9
    map_yaml = map_yaml.resolve()
    historical, history_sources = load_history(history_roots)
    candidates, ground_z, map_metadata = map_candidates(map_yaml, MIN_CLEARANCE_M)
    map_candidate_count = len(candidates)
    if not len(candidates):
        raise ValueError("map has no negative-x cells meeting occupancy, elevation, and clearance constraints")

    # Remove every map cell inside the historical exclusion radius at once.
    if len(historical):
        history_tree = cKDTree(historical)
        nearest, _ = history_tree.query(candidates, k=1, workers=1)
        eligible = nearest >= spacing_floor
    else:
        eligible = np.ones(len(candidates), dtype=bool)
    candidates, ground_z = candidates[eligible], ground_z[eligible]
    if len(candidates) < count:
        raise ValueError(
            f"only {len(candidates)} map cells remain after historical {min_distance:g} m exclusion; "
            f"cannot produce {count} trials"
        )

    rng = np.random.default_rng(seed)
    order = rng.permutation(len(candidates))
    selected: list[int] = []
    selected_tree: cKDTree | None = None
    for index in order:
        point = candidates[index]
        if selected_tree is not None:
            distance, _ = selected_tree.query(point, k=1, workers=1)
            if distance < spacing_floor:
                continue
        selected.append(int(index))
        selected_tree = cKDTree(candidates[selected])
        if len(selected) == count:
            break
    if len(selected) < count:
        raise ValueError(
            f"only {len(selected)} mutually separated locations found at {min_distance:g} m; "
            f"cannot produce {count} trials; spacing was not reduced"
        )

    poses = candidates[selected]
    yaw_values = rng.uniform(-math.pi, math.pi, size=count)
    if (len(yaw_values) != count or not np.isfinite(yaw_values).all()
            or np.any(yaw_values < -math.pi) or np.any(yaw_values >= math.pi)):
        raise RuntimeError("generated yaw values failed finite range validation")
    trials = []
    for index, ((x, y), ground, yaw) in enumerate(zip(poses, ground_z[selected], yaw_values)):
        trials.append({
            "trial_id": f"holdout_{index:03d}",
            "trial_index": index,
            "requested_pose": {
                "x": float(x), "y": float(y),
                "z": float(ground + BASE_FOOTPRINT_OFFSET_M
                           + map_metadata.get("spawn_separation_margin_m", 0.0)),
                "yaw": float(yaw),
            },
        })

    pose_source_count = sum(s["requested_pose_count"] for s in history_sources)
    return {
        "schema_version": 1,
        "generator": "generate_holdout_trials.py",
        "seed": int(seed),
        "count": int(count),
        "sampling_map": str(map_yaml),
        "map_sha256": map_metadata["map_content_sha256"],
        "map_source": map_metadata,
        "history_roots": [str(p.resolve()) for p in history_roots],
        "history_pose_count": int(pose_source_count),
        "history_sources": history_sources,
        "sampling_limits": {
            "search_side": "negative_x",
            "x_boundary_m": 0.0,
            "minimum_clearance_m": MIN_CLEARANCE_M,
            "minimum_distance_from_history_m": float(min_distance),
            "minimum_batch_separation_m": float(min_distance),
            "yaw_sampling": "uniform[-pi, pi)",
            "base_footprint_offset_above_ground_m": BASE_FOOTPRINT_OFFSET_M,
            "spawn_separation_margin_m": map_metadata.get("spawn_separation_margin_m", 0.0),
            "actual_collision_support_checked": "collision_support" in map_metadata,
            "occupancy_is_free_when_pixel_gt": 250,
            "unknown_and_no_data_elevation_excluded": True,
            "algorithm_outcomes_read": False,
        },
        "eligible_map_cells_before_history_exclusion": int(map_candidate_count),
        "eligible_map_cells_after_history_exclusion": int(len(candidates)),
        "trials": trials,
    }


def write_new_file(path: Path, payload: dict) -> None:
    path = path.resolve()
    if not path.name.startswith("trials") or path.suffix != ".json":
        raise ValueError("holdout filename must be trials*.json so future batches discover and exclude it")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x") as stream:
            json.dump(payload, stream, indent=2, sort_keys=False)
            stream.write("\n")
    except FileExistsError as error:
        raise FileExistsError(f"refusing to overwrite existing output: {path}") from error


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=120)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--history-root", action="append", nargs="+", dest="history_roots")
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--min-distance", type=float, default=DEFAULT_MIN_DISTANCE_M)
    args = parser.parse_args(argv)
    roots = [Path(item) for group in (args.history_roots or []) for item in group]
    if not roots:
        roots = DEFAULT_HISTORY_ROOTS.copy()
    try:
        payload = sample_trials(count=args.count, seed=args.seed, map_yaml=args.map,
                                history_roots=roots, min_distance=args.min_distance)
        write_new_file(args.output, payload)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f"holdout generation failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"output": str(args.output.resolve()), "count": payload["count"],
                      "seed": payload["seed"], "map_sha256": payload["map_sha256"],
                      "history_pose_count": payload["history_pose_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
