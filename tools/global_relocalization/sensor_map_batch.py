#!/usr/bin/env python3
"""Checkpointed sensor-built-map trials with immutable inputs and failure accounting."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import time

import numpy as np
import yaml

from evaluation import POSE_CRITERIA, meets_pose_criteria
from generate_holdout_trials import (
    DEFAULT_HISTORY_ROOTS,
    DEFAULT_MAP,
    DEFAULT_MIN_DISTANCE_M,
    history_files,
    requested_xy,
    sample_trials,
    sha256_file,
)

HERE = Path(__file__).resolve().parent


def archive_frozen_inputs(out, provenance):
    """Keep reproducible inputs even when the next optimization edits live files."""
    directory = Path(out) / "frozen_inputs"
    directory.mkdir(exist_ok=True)
    index = []
    for source, digest in provenance.items():
        path = Path(source)
        if sha256_file(path) != digest:
            raise ValueError(f"input changed while freezing: {path}")
        destination = directory / (digest[:12] + "_" + path.name)
        if not destination.exists():
            shutil.copy2(path, destination)
        if sha256_file(destination) != digest:
            raise ValueError(f"frozen input snapshot mismatch: {destination}")
        index.append({"original": source, "snapshot": destination.name, "sha256": digest})
    (directory / "index.json").write_text(json.dumps(index, indent=2) + "\n")


def statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"count": 0, "mean": None, "rmse": None, "median": None, "p95": None, "max": None}
    if not np.isfinite(values).all():
        raise ValueError("nonfinite evaluation metric")
    return {"count": len(values), "mean": float(values.mean()),
            "rmse": float(np.sqrt(np.mean(values**2))), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "max": float(values.max())}


def summarize(rows, planned, profile="ideal", regression_mode=False):
    valid = [r for r in rows if r.get("action", {}).get("success") and (r.get("score") or {}).get("valid")]
    strict = [r for r in valid if r["score"]["strict_correct"]]
    selected = [r for r in valid if meets_pose_criteria(r["score"], profile)]
    fast = [r for r in selected if 0 <= (r.get("terminal_status") or {}).get(
        "competition", {}).get("trigger_to_pose_sec", float("inf")) <= 1.0]
    failures = {}
    for row in rows:
        if row not in fast:
            reason = row.get("exception")
            if row in selected:
                reason = "accepted pose missing valid latency or exceeds 1 s"
            elif row in valid:
                position_limit, angle_limit = POSE_CRITERIA[profile]
                reason = f"accepted pose exceeds {position_limit:g} m / {angle_limit:g} deg"
            elif not reason and row.get("action", {}).get("success"):
                reason = (row.get("score") or {}).get("reason") or "accepted output missing valid GT score"
            elif not reason:
                reason = row.get("action", {}).get("reason") or "no algorithm output"
            failures[reason] = failures.get(reason, 0) + 1
    result = {"planned": planned, "completed": len(rows), "accepted": sum(r.get("action", {}).get("success", False) for r in rows),
              "valid_scored": len(valid), "strict_correct": len(strict),
              "strict_correct_rate_all_completed": len(strict) / len(rows) if rows else None,
              "criteria": "translation_3d <= 0.10 m AND SO(3) <= 2 deg",
              "evaluation_profile": profile,
              "regression_mode": bool(regression_mode),
              "selected_pose_limits": dict(zip(("translation_3d_m", "orientation_so3_deg"), POSE_CRITERIA[profile])),
              "pose_correct": len(selected),
              "pose_correct_and_fast": len(fast),
              "pose_correct_and_fast_rate_all_completed": len(fast) / len(rows) if rows else None,
              "trigger_limit_sec": 1.0,
              "acceptance_passed": (not regression_mode and planned > 0 and len(rows) == planned
                                     and len(fast) == planned),
              "error_statistics_population": "all accepted poses with valid synchronized GT; failures have no artificial zero error",
              "failure_reasons": failures}
    for key in ("translation_3d_m", "translation_xy_m", "orientation_so3_deg"):
        result[key] = statistics([r["score"][key] for r in valid])
    result["absolute_yaw_error_deg"] = statistics([abs(r["score"]["yaw_error_deg"]) for r in valid])
    result["trigger_to_pose_sec"] = statistics([
        r["terminal_status"]["competition"]["trigger_to_pose_sec"] for r in valid
        if (r.get("terminal_status") or {}).get("competition", {}).get("trigger_to_pose_sec", -1) >= 0])
    result["groups_by_initial_yaw"] = {}
    for yaw in sorted(set(r["requested_pose"]["yaw"] for r in rows)):
        group = [r for r in rows if r["requested_pose"]["yaw"] == yaw]
        scored = [r for r in valid if r["requested_pose"]["yaw"] == yaw]
        result["groups_by_initial_yaw"][str(yaw)] = {
            "completed": len(group), "valid_scored": len(scored),
            "strict_correct": sum(r["score"]["strict_correct"] for r in scored),
            "translation_3d_m": statistics([r["score"]["translation_3d_m"] for r in scored]),
            "orientation_so3_deg": statistics([r["score"]["orientation_so3_deg"] for r in scored])}
    if regression_mode:
        result["acceptance_block_reason"] = "regression mode cannot be reported as holdout acceptance"
    return result


def write_reports(out, rows, planned, profile="ideal", regression_mode=False):
    summary = summarize(rows, planned, profile, regression_mode)
    temporary = out / "summary.json.tmp"
    temporary.write_text(json.dumps(summary, indent=2) + "\n")
    temporary.replace(out / "summary.json")
    with (out / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["trial_id", "accepted", "strict_correct", "reason",
            "truth_x", "truth_y", "truth_z", "truth_roll_deg", "truth_pitch_deg", "truth_yaw_deg",
            "estimate_x", "estimate_y", "estimate_z", "estimate_roll_deg", "estimate_pitch_deg", "estimate_yaw_deg",
            "position_error_m", "xy_error_m", "orientation_error_deg", "yaw_error_deg", "trigger_to_pose_sec"])
        writer.writeheader()
        for row in rows:
            score = row.get("score") or {}
            record = {"trial_id": row["trial_id"], "accepted": row.get("action", {}).get("success", False),
                      "strict_correct": score.get("strict_correct", False),
                      "reason": row.get("exception") or row.get("action", {}).get("reason", ""),
                      "position_error_m": score.get("translation_3d_m"), "xy_error_m": score.get("translation_xy_m"),
                      "orientation_error_deg": score.get("orientation_so3_deg"), "yaw_error_deg": score.get("yaw_error_deg"),
                      "trigger_to_pose_sec": (row.get("terminal_status") or {}).get("competition", {}).get("trigger_to_pose_sec")}
            for prefix, name in (("truth", "ground_truth_pose"), ("estimate", "estimate_pose")):
                pose = row.get(name, {})
                for key, value in zip(("x", "y", "z"), pose.get("xyz_m", [])):
                    record[prefix + "_" + key] = value
                for key, value in zip(("roll_deg", "pitch_deg", "yaw_deg"), pose.get("rpy_deg", [])):
                    record[prefix + "_" + key] = value
            writer.writerow(record)
    return summary


def validate_modes(evaluation_profile=None, regression_mode=None):
    """Resolve optional CLI modes and reject mixing regression with a profile."""
    if regression_mode and evaluation_profile is not None:
        raise ValueError("--regression cannot be combined with --evaluation-profile")
    return evaluation_profile or "ideal", bool(regression_mode)


def load_historical_xy(history_roots, exclude_path=None):
    excluded = Path(exclude_path).resolve() if exclude_path is not None else None
    points, sources = [], []
    for path in history_files([Path(p) for p in history_roots]):
        if excluded is not None and path.resolve() == excluded:
            continue
        document = json.loads(path.read_text())
        rows = requested_xy(document)
        points.extend(rows)
        sources.append({"path": str(path.resolve()), "sha256": sha256_file(path),
                        "requested_pose_count": len(rows)})
    return np.asarray(points, dtype=np.float64).reshape((-1, 2)), sources


def validate_trial_separation(trials, history_xy, *, allow_historical=False,
                              min_distance=DEFAULT_MIN_DISTANCE_M):
    """Check requested positions only; never inspect trial outcomes."""
    if min_distance < DEFAULT_MIN_DISTANCE_M:
        raise ValueError(f"minimum trial separation cannot be below {DEFAULT_MIN_DISTANCE_M:g} m")
    xy = []
    identifiers = set()
    for trial in trials:
        trial_id = trial.get("trial_id")
        pose = trial.get("requested_pose")
        if not trial_id or trial_id in identifiers or not isinstance(pose, dict):
            raise ValueError("trial matrix needs unique trial_id and requested_pose entries")
        identifiers.add(trial_id)
        try:
            x, y, z = float(pose["x"]), float(pose["y"]), float(pose["z"])
            yaw = float(pose["yaw"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"trial {trial_id} has invalid requested_pose") from error
        if not np.isfinite([x, y, z, yaw]).all() or not (-np.pi <= yaw < np.pi):
            raise ValueError(f"trial {trial_id} has non-finite coordinates or yaw outside [-pi, pi)")
        if x >= 0.0:
            raise ValueError(f"trial {trial_id} is outside the negative-x search half")
        xy.append((x, y))
    positions = np.asarray(xy, dtype=np.float64).reshape((-1, 2))
    if len(positions) > 1:
        pairwise = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        pairwise[np.eye(len(positions), dtype=bool)] = np.inf
        if float(pairwise.min()) < min_distance - 1e-9:
            raise ValueError(f"trial positions must be separated by at least {min_distance:g} m")
    if len(history_xy) and not allow_historical:
        from scipy.spatial import cKDTree
        distances, _ = cKDTree(history_xy).query(positions, k=1, workers=1)
        if float(np.min(distances)) < min_distance - 1e-9:
            raise ValueError(f"trial position overlaps historical data within {min_distance:g} m")
    return positions


def prepare_frozen_trials(out, *, trials_path=None, limit=None, evaluation_profile=None,
                          regression_mode=None, resume=False, seed=None, map_yaml=None,
                          history_roots=None):
    """Load the immutable trial matrix or create one once for a new batch."""
    out = Path(out).resolve()
    frozen_path = out / "trials.json"
    roots = [Path(p).resolve() for p in
             (DEFAULT_HISTORY_ROOTS if history_roots is None else history_roots)]
    if resume:
        if not frozen_path.is_file():
            raise ValueError("resume requires the existing frozen trials.json; refusing to resample")
        frozen = json.loads(frozen_path.read_text())
        saved_profile = frozen["evaluation_profile"]
        saved_regression = bool(frozen.get("regression_mode", False))
        if regression_mode is True and evaluation_profile is not None:
            raise ValueError("--regression cannot be combined with --evaluation-profile")
        if evaluation_profile is not None and evaluation_profile != saved_profile:
            raise ValueError("resume evaluation profile differs from frozen batch")
        if regression_mode is not None and bool(regression_mode) != saved_regression:
            raise ValueError("resume regression mode differs from frozen batch")
        if limit is not None and int(limit) != len(frozen["trials"]):
            raise ValueError("resume limit differs from frozen trial count")
        source = frozen.get("trial_source", {})
        if seed is not None and (source.get("kind") != "generated" or int(seed) != source.get("seed")):
            raise ValueError("resume seed differs from frozen trial generation")
        if trials_path is not None:
            source_path = Path(trials_path).resolve()
            if source.get("kind") != "explicit" or source.get("path") != str(source_path):
                raise ValueError("resume trials input differs from original frozen source")
            if sha256_file(source_path) != source.get("sha256"):
                raise ValueError("resume trials source file changed")
        elif source.get("kind") == "explicit":
            source_path = Path(source["path"])
            if not source_path.is_file() or sha256_file(source_path) != source.get("sha256"):
                raise ValueError("original explicit trials source is missing or changed")
        if bool(frozen.get("regression_mode", False)) and evaluation_profile is not None:
            raise ValueError("--regression cannot be combined with --evaluation-profile")
        return frozen

    profile, regression = validate_modes(evaluation_profile, regression_mode)
    if limit is not None and (isinstance(limit, bool) or int(limit) != limit or int(limit) < 1):
        raise ValueError("--limit must be a positive integer")
    if seed is not None and (isinstance(seed, bool) or not isinstance(seed, (int, np.integer))
                             or seed < 0 or seed > 2**64 - 1):
        raise ValueError("--seed must be an unsigned 64-bit integer")
    input_path = Path(trials_path).resolve() if trials_path is not None else None
    generated = None
    if input_path is None:
        selected_seed = secrets.randbits(64) if seed is None else int(seed)
        generated = sample_trials(
            count=int(limit) if limit is not None else 120,
            seed=selected_seed,
            map_yaml=Path(map_yaml or DEFAULT_MAP),
            history_roots=roots,
            min_distance=DEFAULT_MIN_DISTANCE_M,
        )
        trials = generated["trials"]
        generation_metadata = {key: value for key, value in generated.items() if key != "trials"}
        source = {"kind": "generated", "seed": int(selected_seed),
                  "generator": str((HERE / "generate_holdout_trials.py").resolve())}
        history_xy, history_sources = load_historical_xy(roots)
    else:
        if seed is not None:
            raise ValueError("--seed applies only to automatically generated trials")
        document = json.loads(input_path.read_text())
        all_trials = document.get("trials")
        if not isinstance(all_trials, list) or not all_trials:
            raise ValueError("explicit trials file must contain a nonempty trials list")
        trials = all_trials[:int(limit)] if limit is not None else all_trials
        generation_metadata = {key: value for key, value in document.items() if key != "trials"}
        source = {"kind": "explicit", "path": str(input_path), "sha256": sha256_file(input_path)}
        history_xy, history_sources = load_historical_xy(roots, exclude_path=input_path)
        if generation_metadata.get("history_sources") is None:
            generation_metadata["validated_history_sources"] = history_sources
        if generation_metadata.get("seed") is not None:
            source["seed"] = generation_metadata["seed"]

    positions = validate_trial_separation(
        trials, history_xy, allow_historical=regression, min_distance=DEFAULT_MIN_DISTANCE_M)
    if not len(trials):
        raise ValueError("trial matrix is empty")
    frozen = {
        "schema_version": 2,
        "trials": trials,
        "planned_count": len(trials),
        "evaluation_profile": profile,
        "regression_mode": regression,
        "trial_source": source,
        "generation_metadata": generation_metadata,
        "historical_validation": {
            "history_roots": [str(root) for root in roots],
            "excluded_input_file": str(input_path) if input_path is not None else None,
            "history_source_count": len(history_sources),
            "history_pose_count": int(len(history_xy)),
            "history_sources": history_sources,
            "minimum_historical_distance_m": DEFAULT_MIN_DISTANCE_M,
            "minimum_batch_distance_m": DEFAULT_MIN_DISTANCE_M,
            "regression_bypassed_historical_exclusion": regression,
        },
        "minimum_limit_argument": limit,
        "validated_trial_xy_count": int(len(positions)),
    }
    frozen_path.parent.mkdir(parents=True, exist_ok=True)
    with frozen_path.open("x") as stream:
        json.dump(frozen, stream, indent=2)
        stream.write("\n")
    return frozen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map-dir", type=Path, required=True)
    parser.add_argument("--node", required=True)
    parser.add_argument("--params", type=Path, default=HERE.parents[1] / "src/global_relocalization/config/competition.yaml")
    parser.add_argument("--trials", type=Path,
                        help="explicit trial matrix; positions are checked against all other history")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--seed", type=int, help="optional seed for an automatically generated holdout")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--evaluation-profile", choices=POSE_CRITERIA)
    parser.add_argument("--regression", action="store_true", default=None,
                        help="allow historical positions; such a run can never pass acceptance")
    a = parser.parse_args()
    out = a.output.resolve()
    if a.resume:
        if not out.is_dir():
            raise ValueError("--resume requires an existing batch output directory")
    else:
        out.mkdir(parents=True, exist_ok=False)
    map_dir = a.map_dir.resolve()
    manifest = json.loads((map_dir / "manifest.json").read_text())
    if (map_dir / "MAP_QUALITY_REJECTED.json").exists():
        raise ValueError("map quality rejected; use offline diagnostics, not a formal batch")
    if (map_dir / "DIAGNOSTIC_MAP_ONLY.json").exists():
        raise ValueError("diagnostic-only map; use offline diagnostics, not a formal batch")
    if manifest.get("ground_truth_used_for_scan_fusion") is not False:
        raise ValueError("expected a sensor-built map with no GT scan fusion")
    if manifest.get("capture_complete") is not True:
        raise ValueError("survey incomplete; do not publish a partial map as formal input")
    if hashlib.sha256((map_dir / "map.pcd").read_bytes()).hexdigest() != manifest["map_sha256"]:
        raise ValueError("map hash mismatch")
    frozen_path = out / "trials.json"
    frozen = prepare_frozen_trials(
        out, trials_path=a.trials, limit=a.limit,
        evaluation_profile=a.evaluation_profile, regression_mode=a.regression,
        resume=a.resume, seed=a.seed,
    )
    trials = frozen["trials"]
    profile = frozen["evaluation_profile"]
    regression_mode = bool(frozen.get("regression_mode", False))
    inputs = [Path(a.node).resolve(), Path(a.node).resolve().parents[1] / "libglobal_relocalization_core.so",
              a.params.resolve(), map_dir / "map.pcd", map_dir / "manifest.json", frozen_path,
              HERE / "competition_trial.py", HERE / "evaluation.py", Path(__file__),
              HERE / "generate_holdout_trials.py"]
    if frozen.get("trial_source", {}).get("kind") == "explicit":
        inputs.append(Path(frozen["trial_source"]["path"]))
    generation_metadata = frozen.get("generation_metadata", {})
    if generation_metadata.get("sampling_map"):
        inputs.append(Path(generation_metadata["sampling_map"]))
    map_sources = generation_metadata.get("map_source", {})
    for key in ("occupancy_image", "elevation_image"):
        if map_sources.get(key):
            inputs.append(Path(map_sources[key]))
    if map_sources.get("collision_support"):
        inputs.append(HERE / "actual_collision_support.py")
        inputs.extend(Path(name) for name in map_sources["collision_support"]["source_hashes"])
    provenance = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    provenance_path = out / "provenance.json"
    if a.resume:
        if not provenance_path.is_file():
            raise ValueError("resume requires the original provenance.json")
        if json.loads(provenance_path.read_text())["sha256"] != provenance:
            raise ValueError("resume inputs changed")
    provenance_path.write_text(json.dumps({"sha256": provenance, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
                                          "GZ_PARTITION": os.environ.get("GZ_PARTITION")}, indent=2) + "\n")
    archive_frozen_inputs(out, provenance)
    rows = []
    for trial in trials:
        directory = out / trial["trial_id"]
        if a.resume and (directory / "result.json").exists():
            rows.append(json.loads((directory / "result.json").read_text()))
            continue
        if directory.exists():
            raise ValueError(f"partial trial requires inspection: {directory}")
        params = yaml.safe_load(a.params.read_text())
        config = params["global_relocalization"]["ros__parameters"]
        config["diagnostic_output_dir"] = str(directory / "query_diagnostics")
        config["raw_diagnostic_directory"] = str(directory / "raw_diagnostics")
        params_path = out / (trial["trial_id"] + ".yaml")
        params_path.write_text(yaml.safe_dump(params, sort_keys=False))
        command = ["/usr/bin/python3", str(HERE / "competition_trial.py"), "--node", a.node,
                   "--params", str(params_path), "--map", str(map_dir / "map.pcd"),
                   "--manifest", str(map_dir / "manifest.json"), "--trials", str(frozen_path),
                   "--evaluation-profile", profile,
                   "--trial", trial["trial_id"], "--output", str(directory), "--record-bag"]
        with (out / (trial["trial_id"] + ".log")).open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=220)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    code = process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    process.kill()
                    code = process.wait()
        result_path = directory / "result.json"
        if result_path.exists():
            result = json.loads(result_path.read_text())
        else:
            directory.mkdir(exist_ok=True)
            result = {**trial, "exception": f"trial process exited {code} without result; inspect trial log"}
            result_path.write_text(json.dumps(result, indent=2) + "\n")
        rows.append(result)
        summary = write_reports(out, rows, len(trials), profile, regression_mode)
        print(json.dumps({"completed": len(rows), "planned": len(trials), "trial": trial["trial_id"],
                          "strict_correct": summary["strict_correct"], "score": result.get("score"),
                          "failure": result.get("exception") or result.get("action", {}).get("reason")}), flush=True)
    summary = write_reports(out, rows, len(trials), profile, regression_mode)
    return 0 if summary["acceptance_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
