#!/usr/bin/env python3
"""Verify that per-trial executed inputs match the frozen batch provenance."""
import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import math
import tempfile
import numpy as np
import yaml
from scipy.spatial import cKDTree
from generate_holdout_trials import (
    BASE_FOOTPRINT_OFFSET_M,
    DEFAULT_MIN_DISTANCE_M,
    MIN_CLEARANCE_M,
    map_candidates,
    requested_xy,
)
from evaluation import POSE_CRITERIA, meets_pose_criteria


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def archived_input_index(batch):
    """Map frozen original paths to verified archive copies, if available."""
    index_path = Path(batch) / "frozen_inputs" / "index.json"
    if not index_path.is_file():
        return {}, [], []
    archives, problems = {}, []
    for item in json.loads(index_path.read_text()):
        original = Path(item["original"]).resolve()
        snapshot = index_path.parent / item["snapshot"]
        expected = item.get("sha256")
        snapshot_hash = file_sha256(snapshot) if snapshot.is_file() else None
        if not expected or snapshot_hash != expected:
            problems.append({"kind": "frozen_snapshot_hash_mismatch", "path": str(snapshot),
                             "expected_sha256": expected, "actual_sha256": snapshot_hash})
            continue
        archives[original] = snapshot
    return archives, problems


def frozen_source_path(path, expected, archives, problems):
    """Prefer a validated archive over a changed runtime path."""
    original = Path(path).resolve()
    snapshot = archives.get(original)
    candidate = snapshot if snapshot is not None else original
    actual = file_sha256(candidate) if candidate.is_file() else None
    if actual != expected:
        problems.append({"kind": "current_frozen_input_changed", "path": str(original),
                         "checked_source": str(candidate), "expected_sha256": expected,
                         "actual_sha256": actual})
        return None
    return candidate


@contextmanager
def archived_map_view(map_yaml, snapshot, archives=None, map_source=None):
    """Use runtime map when frozen-identical; otherwise build a temp view of frozen assets."""
    map_yaml = Path(map_yaml).resolve()
    snapshot = Path(snapshot).resolve() if snapshot is not None else None
    archives = archives or {}
    map_source = map_source or {}
    frozen_resources = []
    for name, expected in ((map_source.get("occupancy_image"), map_source.get("occupancy_sha256")),
                           (map_source.get("elevation_image"), map_source.get("elevation_sha256"))):
        if name and expected:
            frozen_resources.append((Path(name).resolve(), expected))
    collision = map_source.get("collision_support", {})
    geometry_hashes = collision.get("source_hashes", collision.get("source_sha256", {}))
    frozen_resources.extend((Path(name).resolve(), expected) for name, expected in geometry_hashes.items())
    changed_runtime_resource = any(
        not path.is_file() or file_sha256(path) != expected for path, expected in frozen_resources)
    map_yaml_changed = snapshot is not None and map_yaml.is_file() and file_sha256(map_yaml) != file_sha256(snapshot)
    if (snapshot is None or snapshot == map_yaml
            or (not map_yaml_changed and not changed_runtime_resource)):
        yield Path(map_yaml)
        return
    document = yaml.safe_load(Path(snapshot).read_text())
    with tempfile.TemporaryDirectory(prefix="holdout-map-audit-") as temporary:
        root = Path(temporary)
        def frozen_resource(relative, *, directory=False):
            source = Path(relative)
            if not source.is_absolute():
                source = map_yaml.parent / source
            source = archives.get(source.resolve(), source.resolve())
            exists = source.is_dir() if directory else source.is_file()
            if not exists:
                raise FileNotFoundError(f"frozen map resource unavailable: {source}")
            return str(source.resolve())

        if document.get("image"):
            document["image"] = frozen_resource(document["image"])
        grid = document.get("ground_elevation", {}).get("grid", {})
        if grid.get("image"):
            grid["image"] = frozen_resource(grid["image"])
        support = document.get("collision_support", {})
        for key in ("world_path", "robot_urdf_path"):
            if support.get(key):
                support[key] = frozen_resource(support[key])
        if support.get("model_roots"):
            roots = []
            for root_index, item in enumerate(support["model_roots"]):
                original_root = Path(item)
                if not original_root.is_absolute():
                    original_root = map_yaml.parent / original_root
                original_root = original_root.resolve()
                temp_root = root / f"model_root_{root_index}"
                temp_root.mkdir()
                for name, expected in geometry_hashes.items():
                    source = Path(name).resolve()
                    try:
                        relative = source.relative_to(original_root)
                    except ValueError:
                        continue
                    archived_source = archives.get(source, source)
                    if not archived_source.is_file() or file_sha256(archived_source) != expected:
                        raise ValueError(f"frozen collision resource unavailable: {source}")
                    link = temp_root / relative
                    link.parent.mkdir(parents=True, exist_ok=True)
                    link.symlink_to(archived_source)
                roots.append(str(temp_root))
            support["model_roots"] = roots
        archived_yaml = root / Path(map_yaml).name
        archived_yaml.write_text(yaml.safe_dump(document, sort_keys=False))
        yield archived_yaml


def verify_generated_geometry_and_seed(trials, generation, history_xy, map_yaml=None):
    """Recheck sample geometry and seeded point/yaw sequence from frozen history."""
    map_yaml = Path(map_yaml or generation.get("sampling_map", ""))
    candidates, ground_z, _ = map_candidates(map_yaml, MIN_CLEARANCE_M)
    xy = np.asarray([requested_xy(trial) for trial in trials], dtype=np.float64)
    if xy.ndim != 3 or xy.shape[1:] != (1, 2):
        raise ValueError("generated trials must each have one requested pose")
    xy = xy[:, 0, :]
    nearest, indices = cKDTree(candidates).query(xy, k=1, workers=1)
    if np.any(nearest > 1e-5):
        raise ValueError("a generated trial is not a free, known, clearance-qualified map cell center")
    for trial, index in zip(trials, indices):
        support_margin = float(generation.get("sampling_limits", {}).get(
            "spawn_separation_margin_m", generation.get("map_source", {}).get(
                "spawn_separation_margin_m", 0.0)))
        expected_z = float(ground_z[index] + BASE_FOOTPRINT_OFFSET_M + support_margin)
        actual_z = float(trial["requested_pose"]["z"])
        if abs(expected_z - actual_z) > 1e-5:
            raise ValueError(f"{trial.get('trial_id')} height does not match the frozen elevation cell")

    seed = generation.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**64:
        raise ValueError("generated seed is not a valid unsigned 64-bit integer")
    distance = float(generation.get("sampling_limits", {}).get(
        "minimum_distance_from_history_m", DEFAULT_MIN_DISTANCE_M))
    spacing_floor = distance + 1e-9
    if len(history_xy):
        history_distance, _ = cKDTree(history_xy).query(candidates, k=1, workers=1)
        eligible = history_distance >= spacing_floor
    else:
        eligible = np.ones(len(candidates), dtype=bool)
    candidates, ground_z = candidates[eligible], ground_z[eligible]
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(candidates))
    selected = []
    selected_tree = None
    for index in order:
        point = candidates[index]
        if selected_tree is not None:
            nearest_selected, _ = selected_tree.query(point, k=1, workers=1)
            if nearest_selected < spacing_floor:
                continue
        selected.append(int(index))
        selected_tree = cKDTree(candidates[selected])
        if len(selected) == len(trials):
            break
    if len(selected) != len(trials):
        raise ValueError("frozen seed cannot reproduce the recorded trial count")
    expected_xy = candidates[selected]
    expected_yaw = rng.uniform(-math.pi, math.pi, size=len(trials))
    support_margin = float(generation.get("sampling_limits", {}).get(
        "spawn_separation_margin_m", generation.get("map_source", {}).get(
            "spawn_separation_margin_m", 0.0)))
    for index, (trial, point, height, yaw) in enumerate(zip(
            trials, expected_xy, ground_z[selected], expected_yaw)):
        pose = trial["requested_pose"]
        if trial.get("trial_id") != f"holdout_{index:03d}" or trial.get("trial_index") != index:
            raise ValueError("generated trial identities do not match the frozen seed sequence")
        if (np.linalg.norm(np.asarray([pose["x"], pose["y"]]) - point) > 1e-8
                or abs(float(pose["z"]) - float(height + BASE_FOOTPRINT_OFFSET_M + support_margin)) > 1e-8
                or abs(float(pose["yaw"]) - float(yaw)) > 1e-12):
            raise ValueError(f"{trial.get('trial_id')} does not match deterministic seed regeneration")
    return len(trials)


def audit_frozen_freshness(frozen, archives=None):
    """Validate only the historical sources frozen at batch creation time."""
    problems = []
    archives = archives or {}
    if frozen.get("schema_version", 0) < 2 or not frozen.get("historical_validation"):
        return {"status": "legacy_missing_holdout_metadata", "acceptance_eligible": False,
                "regression_mode": bool(frozen.get("regression_mode", False)),
                "trial_source": frozen.get("trial_source"), "map_sha256": None,
                "historical_source_count": None, "historical_pose_count": None,
                "minimum_historical_distance_m": None, "minimum_batch_distance_m": None}, problems

    source = frozen.get("trial_source", {})
    generation = frozen.get("generation_metadata", {})
    validation = frozen["historical_validation"]
    regression = bool(frozen.get("regression_mode", False))
    if source.get("kind") not in ("generated", "explicit"):
        problems.append({"kind": "trial_source_metadata_invalid"})
    if source.get("kind") == "explicit":
        path = Path(source.get("path", "")).resolve()
        path = archives.get(path, path)
        if not path.is_file() or file_sha256(path) != source.get("sha256"):
            problems.append({"kind": "explicit_trials_source_changed", "path": str(path)})
    elif source.get("kind") == "generated":
        seed = source.get("seed")
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**64:
            problems.append({"kind": "generated_seed_invalid"})
        if generation.get("seed") != seed:
            problems.append({"kind": "generated_seed_metadata_mismatch"})

    map_source = generation.get("map_source", {})
    sample_map = generation.get("sampling_map")
    if not sample_map or not map_source:
        problems.append({"kind": "sampling_map_metadata_missing"})
    else:
        map_yaml_path = Path(sample_map).resolve()
        collision = map_source.get("collision_support", {})
        geometry_hashes = collision.get("source_hashes", collision.get("source_sha256", {}))
        geometry_items = sorted(geometry_hashes.items())
        sampling_limits = generation.get("sampling_limits", {})
        if sampling_limits.get("actual_collision_support_checked") and not geometry_items:
            problems.append({"kind": "collision_support_hashes_missing"})
        paths = [(map_yaml_path, map_source.get("map_yaml_sha256")),
                 (map_source.get("occupancy_image"), map_source.get("occupancy_sha256")),
                 (map_source.get("elevation_image"), map_source.get("elevation_sha256")),
                 *geometry_items]
        combined = hashlib.sha256()
        combined_valid = True
        for name, expected in paths:
            path = Path(name).resolve() if name else None
            path = archives.get(path, path) if path is not None else None
            if path is None or not path.is_file() or not expected or file_sha256(path) != expected:
                problems.append({"kind": "sampling_map_source_changed", "path": str(path) if path else None})
                combined_valid = False
                continue
            logical_path = Path(name).resolve()
            relative = Path(os.path.relpath(logical_path, map_yaml_path.parent)).as_posix()
            combined.update(relative.encode())
            combined.update(b"\0")
            with path.open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    combined.update(block)
        expected_map_hash = generation.get("map_sha256")
        expected_component_hash = map_source.get("map_content_sha256")
        if expected_map_hash != expected_component_hash:
            problems.append({"kind": "sampling_map_hash_metadata_mismatch"})
        if combined_valid and combined.hexdigest() != expected_map_hash:
            problems.append({"kind": "sampling_map_content_hash_mismatch"})

    historical_sources = validation.get("history_sources", [])
    old_xy = []
    for item in historical_sources:
        path = Path(item.get("path", ""))
        if not path.is_file() or file_sha256(path) != item.get("sha256"):
            problems.append({"kind": "frozen_history_source_changed", "path": str(path)})
            continue
        try:
            poses = requested_xy(json.loads(path.read_text()))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            problems.append({"kind": "frozen_history_source_unreadable", "path": str(path), "detail": str(error)})
            continue
        if len(poses) != item.get("requested_pose_count"):
            problems.append({"kind": "frozen_history_pose_count_mismatch", "path": str(path)})
        old_xy.extend(poses)
    if len(historical_sources) != validation.get("history_source_count"):
        problems.append({"kind": "frozen_history_source_count_mismatch"})
    if len(old_xy) != validation.get("history_pose_count"):
        problems.append({"kind": "frozen_history_pose_count_total_mismatch"})

    regenerated_count = None
    if source.get("kind") == "generated" and sample_map and not problems:
        try:
            archived_yaml = archives.get(map_yaml_path)
            with archived_map_view(map_yaml_path, archived_yaml, archives, map_source) as geometry_yaml:
                regenerated_count = verify_generated_geometry_and_seed(
                    frozen.get("trials", []), generation,
                    np.asarray(old_xy, dtype=np.float64).reshape((-1, 2)), map_yaml=geometry_yaml)
        except (OSError, ValueError, KeyError, TypeError) as error:
            problems.append({"kind": "generated_geometry_or_seed_mismatch", "detail": str(error)})

    positions, invalid_trials = [], []
    for trial in frozen.get("trials", []):
        pose = trial.get("requested_pose", {})
        try:
            x, y, z, yaw = (float(pose["x"]), float(pose["y"]),
                            float(pose["z"]), float(pose["yaw"]))
        except (KeyError, TypeError, ValueError):
            invalid_trials.append(trial.get("trial_id"))
            continue
        if not np.isfinite([x, y, z, yaw]).all() or x >= 0.0 or not -math.pi <= yaw < math.pi:
            invalid_trials.append(trial.get("trial_id"))
            continue
        positions.append((x, y))
    xy = np.asarray(positions, dtype=np.float64).reshape((-1, 2))
    if invalid_trials or len(xy) != len(frozen.get("trials", [])):
        problems.append({"kind": "frozen_trial_pose_invalid", "trial_ids": invalid_trials})
    min_batch = None
    if len(xy) > 1:
        distances = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=2)
        distances[np.eye(len(xy), dtype=bool)] = np.inf
        min_batch = float(distances.min())
        if min_batch < DEFAULT_MIN_DISTANCE_M:
            problems.append({"kind": "frozen_batch_spacing_below_minimum", "minimum_m": min_batch})
    min_history = None
    if len(xy) and len(old_xy):
        min_history = float(cKDTree(np.asarray(old_xy, dtype=np.float64)).query(xy, k=1, workers=1)[0].min())
    minimum_history = float(validation.get("minimum_historical_distance_m", 0))
    minimum_batch = float(validation.get("minimum_batch_distance_m", 0))
    if not regression and minimum_history < DEFAULT_MIN_DISTANCE_M:
        problems.append({"kind": "frozen_historical_spacing_limit_below_minimum"})
    if minimum_batch < DEFAULT_MIN_DISTANCE_M:
        problems.append({"kind": "frozen_batch_spacing_limit_below_minimum"})
    if min_history is not None and min_history < minimum_history - 1e-9 and not regression:
        problems.append({"kind": "historical_distance_recomputed_below_frozen_limit", "minimum_m": min_history})
    if min_batch is not None and min_batch < minimum_batch - 1e-9:
        problems.append({"kind": "batch_distance_recomputed_below_frozen_limit", "minimum_m": min_batch})
    if not regression and min_history is not None and min_history < DEFAULT_MIN_DISTANCE_M:
        problems.append({"kind": "frozen_trials_overlap_historical_data", "minimum_m": min_history})
    if generation.get("sampling_limits", {}).get("minimum_clearance_m", 0) < 0.5:
        problems.append({"kind": "sampling_clearance_below_minimum"})

    return {
        "status": "regression_not_eligible" if regression else "verified",
        "acceptance_eligible": not regression and not problems,
        "regression_mode": regression,
        "trial_source": source,
        "seed": source.get("seed", generation.get("seed")),
        "map_sha256": generation.get("map_sha256"),
        "historical_source_count": len(historical_sources),
        "historical_pose_count": len(old_xy),
        "minimum_historical_distance_m": min_history,
        "minimum_batch_distance_m": min_batch,
        "minimum_clearance_m": generation.get("sampling_limits", {}).get("minimum_clearance_m"),
        "minimum_clearance_verified": (regenerated_count == len(frozen.get("trials", []))
                                        and generation.get("sampling_limits", {}).get(
                                            "minimum_clearance_m", 0) >= 0.5),
        "generated_trials_reproduced": regenerated_count == len(frozen.get("trials", []))
                                      if source.get("kind") == "generated" else None,
    }, problems


def score_batch_results(trials, rows, profile):
    """Recompute both pose criteria and timed selected-profile success."""
    by_id = {row.get("trial_id"): row for row in rows}
    strict = profile_correct = profile_fast = valid = 0
    for trial in trials:
        row = by_id.get(trial.get("trial_id"), {})
        score = row.get("score") or {}
        if not row.get("action", {}).get("success") or not score.get("valid"):
            continue
        valid += 1
        strict += bool(score.get("strict_correct"))
        selected = meets_pose_criteria(score, profile)
        profile_correct += selected
        competition = (row.get("terminal_status") or {}).get("competition", {})
        trigger_to_pose = competition.get("trigger_to_pose_sec", float("inf"))
        profile_fast += selected and isinstance(trigger_to_pose, (int, float)) and 0 <= trigger_to_pose <= 1.0
    planned = len(trials)
    return {"planned": planned, "completed": len(by_id), "valid_scored": valid,
            "strict_correct": strict, "strict_correct_rate_all_planned": strict / planned if planned else None,
            "profile_correct": profile_correct,
            "profile_correct_rate_all_planned": profile_correct / planned if planned else None,
            "profile_correct_and_fast": profile_fast,
            "profile_correct_and_fast_rate_all_planned": profile_fast / planned if planned else None,
            "selected_pose_limits": dict(zip(("translation_3d_m", "orientation_so3_deg"), POSE_CRITERIA[profile])),
            "acceptance_passed": planned > 0 and len(by_id) == planned and profile_fast == planned}


def coordinator_only_inputs(frozen, provenance):
    """Inputs frozen by the coordinator but not consumed by each child trial."""
    metadata = frozen.get("generation_metadata", {})
    map_source = metadata.get("map_source", {})
    paths = {
        Path(name).resolve() for name in provenance
        if Path(name).name in {"sensor_map_batch.py", "generate_holdout_trials.py"}
    }
    source = frozen.get("trial_source", {})
    if source.get("kind") == "explicit" and source.get("path"):
        paths.add(Path(source["path"]).resolve())
    for value in (metadata.get("sampling_map"), map_source.get("occupancy_image"),
                  map_source.get("elevation_image")):
        if value:
            paths.add(Path(value).resolve())
    collision = map_source.get("collision_support", {})
    geometry_hashes = collision.get("source_hashes", collision.get("source_sha256", {}))
    paths.update(Path(name).resolve() for name in geometry_hashes)
    paths.update(Path(name).resolve() for name in provenance
                 if Path(name).name == "actual_collision_support.py")
    return paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", required=True, type=Path)
    parser.add_argument("--require-complete", action="store_true")
    a = parser.parse_args()
    provenance = json.loads((a.batch / "provenance.json").read_text())
    frozen_hashes = provenance["sha256"]
    frozen_document = json.loads((a.batch / "trials.json").read_text())
    trials = frozen_document["trials"]
    archives, archive_problems = archived_input_index(a.batch)
    freshness, freshness_problems = audit_frozen_freshness(frozen_document, archives)
    problems, checked, completed, runtime_unverified = [], 0, 0, []
    missing_trials = []
    problems.extend(archive_problems)
    problems.extend(freshness_problems)
    coordinator_inputs = coordinator_only_inputs(frozen_document, frozen_hashes)
    rows = []
    frozen_sources = {}
    runtime_source_hashes = []
    for name, expected in frozen_hashes.items():
        checked_source = frozen_source_path(name, expected, archives, problems)
        if checked_source is not None:
            frozen_sources[Path(name).resolve()] = checked_source
        runtime_path = Path(name).resolve()
        runtime_hash = file_sha256(runtime_path) if runtime_path.is_file() else None
        runtime_source_hashes.append({
            "path": str(runtime_path), "frozen_sha256": expected,
            "frozen_source": str(checked_source) if checked_source is not None else None,
            "runtime_sha256": runtime_hash, "runtime_matches_frozen": runtime_hash == expected,
        })
    for trial in trials:
        directory = a.batch / trial["trial_id"]
        path = directory / "result.json"
        if not path.exists():
            if a.require_complete:
                missing_trials.append(trial["trial_id"])
            continue
        completed += 1
        row = json.loads(path.read_text())
        rows.append(row)
        if row.get("trial_id") != trial["trial_id"] or row.get("requested_pose") != trial["requested_pose"]:
            problems.append({"kind": "trial_identity_mismatch", "trial_id": trial["trial_id"]})
        executed = row.get("sha256", {})
        # Early infrastructure failures can legitimately have no executed
        # estimator; they remain explicit failures, never scored successes.
        if not executed and row.get("exception"):
            runtime_unverified.append({"trial_id": trial["trial_id"], "reason": "no estimator execution provenance; explicit infrastructure failure"})
            continue
        for name, expected in frozen_hashes.items():
            if Path(name).resolve() in coordinator_inputs:
                continue  # coordinator inputs are frozen once, not child executed inputs
            if Path(name).suffix in (".yaml", ".yml"):
                # Only per-trial diagnostic destinations differ from the
                # frozen algorithm profile; verify the actual derived file.
                derived = (a.batch / (trial["trial_id"] + ".yaml")).resolve()
                expected_path = frozen_sources.get(Path(name).resolve(), Path(name))
                expected_params = yaml.safe_load(expected_path.read_text())
                actual_params = yaml.safe_load(derived.read_text())
                for params in (expected_params, actual_params):
                    fields = params["global_relocalization"]["ros__parameters"]
                    for key in ("diagnostic_output_dir", "raw_diagnostic_directory"):
                        fields.pop(key, None)
                runtime_path = directory / "runtime_parameters.json"
                audit = json.loads(runtime_path.read_text()) if runtime_path.exists() else {}
                if not audit.get("passed"):
                    runtime_unverified.append({"trial_id": trial["trial_id"], "reason": "runtime parameter audit failed or unavailable"})
                    if row.get("action", {}).get("success") or not row.get("exception"):
                        problems.append({"kind": "runtime_parameters_unverified_for_algorithm_result", "trial_id": trial["trial_id"]})
                if (expected_params != actual_params or
                        executed.get(str(derived)) != hashlib.sha256(derived.read_bytes()).hexdigest()):
                    problems.append({"kind": "derived_parameters_mismatch", "trial_id": trial["trial_id"]})
                else:
                    checked += 1
                continue
            if executed.get(name) != expected:
                problems.append({"kind": "executed_input_mismatch", "trial_id": trial["trial_id"], "path": name})
            else:
                checked += 1
        if row.get("action", {}).get("success") and row.get("score", {}).get("valid"):
            if not row.get("ground_truth_pose_matrix") or not row.get("estimated_pose_matrix"):
                problems.append({"kind": "scored_pose_evidence_missing", "trial_id": trial["trial_id"]})
        if row.get("bag_returncode") == 0 and not (directory / "raw_bag/metadata.yaml").exists():
            problems.append({"kind": "recording_evidence_missing", "trial_id": trial["trial_id"]})
    profile = frozen_document.get("evaluation_profile", "ideal")
    if missing_trials:
        problems.append({"kind": "missing_trials", "count": len(missing_trials),
                         "trial_ids": missing_trials})
    metrics = score_batch_results(trials, rows, profile)
    integrity_passed = not problems
    acceptance_passed = (integrity_passed and freshness["acceptance_eligible"]
                         and metrics["acceptance_passed"])
    if a.require_complete and not metrics["acceptance_passed"]:
        problems.append({"kind": "acceptance_threshold_not_met"})
    if a.require_complete and not freshness["acceptance_eligible"]:
        problems.append({"kind": "holdout_freshness_not_verified", "status": freshness["status"]})
    result = {"passed": integrity_passed and (not a.require_complete or acceptance_passed),
              "integrity_passed": integrity_passed, "acceptance_passed": acceptance_passed,
              "acceptance_eligible": freshness["acceptance_eligible"], "freshness": freshness,
              "metrics": metrics, "frozen_input_sources": runtime_source_hashes,
              "planned": len(trials), "completed": completed,
              "matched_executed_hashes": checked, "require_complete": a.require_complete, "problems": problems,
              "scope": "frozen input identity; infrastructure failures without runtime audit are listed separately and cannot support algorithm conclusions",
              "runtime_parameter_audit_failed_or_unavailable": runtime_unverified}
    (a.batch / "integrity_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
