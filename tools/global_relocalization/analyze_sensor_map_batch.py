#!/usr/bin/env python3
"""Summarize rejection stages and false accepts without changing trial results."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np

from sensor_map_batch import statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--map-dir", type=Path)
    a = parser.parse_args()
    trials = json.loads((a.batch / "trials.json").read_text())["trials"]
    cases, per_position, categories = [], {}, Counter()
    false_fitness, false_inliers, correct_fitness = [], [], []
    for trial in trials:
        directory = a.batch / trial["trial_id"]
        path = directory / "result.json"
        if not path.exists():
            continue
        row = json.loads(path.read_text())
        action, score = row.get("action", {}), row.get("score", {})
        status = row.get("terminal_status") or {}
        frontend = status.get("raw_frontend", {})
        reason = row.get("exception") or action.get("reason", "no algorithm output")
        if row.get("exception"):
            category = "infrastructure_or_client_exception"
        elif action.get("success") and not score.get("valid"):
            category = "accepted_without_valid_gt"
        elif score.get("strict_correct"):
            category = "strict_correct"
            correct_fitness.append(action["fitness"])
        elif action.get("success"):
            category = "false_accept_against_gt"
            false_fitness.append(action["fitness"])
            false_inliers.append(action["inlier_ratio"])
        elif "became stale" in reason:
            category = "sensor_or_tf_staleness"
        elif frontend.get("failure_class") not in (None, "none", "waiting"):
            category = "raw_frontend_rejection"
        elif "independent" in reason.lower() or "confirmation" in reason.lower():
            category = "global_confirmation_rejection"
        else:
            category = "global_search_or_state_rejection"
        categories[category] += 1
        position = trial["trial_id"].split("_yaw")[0]
        group = per_position.setdefault(position, {"completed": 0, "accepted": 0, "strict_correct": 0,
                                                   "reasons": {}, "position_errors_m": [], "orientation_errors_deg": []})
        group["completed"] += 1
        group["accepted"] += bool(action.get("success"))
        group["strict_correct"] += bool(score.get("strict_correct"))
        group["reasons"][category] = group["reasons"].get(category, 0) + 1
        if score.get("valid"):
            group["position_errors_m"].append(score["translation_3d_m"])
            group["orientation_errors_deg"].append(score["orientation_so3_deg"])
        stages = list(dict.fromkeys(s.get("stage") for s in row.get("status_events", [])))
        cases.append({"trial_id": trial["trial_id"], "category": category, "reason": reason,
                      "stage_sequence": stages, "score": score,
                      "global_quality": {k: action.get(k) for k in ("fitness", "inlier_ratio")},
                      "raw_frontend": {k: frontend.get(k) for k in ["accepted", "failure_class", "reason", "fitness",
                          "inlier_ratio", "rejected_gates", "imu_rejected", "queue_overflow_count", "queue_dropped",
                          "lidar_max_received_stamp_gap_sec", "imu_max_received_stamp_gap_sec"]},
                      "result": str(path), "node_log": str(directory / "node.log"),
                      "raw_diagnostics": str(directory / "raw_diagnostics"),
                      "query_diagnostics": str(directory / "query_diagnostics")})
    for group in per_position.values():
        group["translation_3d_m"] = statistics(group.pop("position_errors_m"))
        group["orientation_so3_deg"] = statistics(group.pop("orientation_errors_deg"))
    result = {"completed": len(cases), "category_counts": dict(categories), "by_position": per_position,
              "false_accept_fitness_m": statistics(false_fitness),
              "false_accept_inlier_ratio": statistics(false_inliers),
              "strict_correct_fitness_m": statistics(correct_fitness), "cases": cases,
              "interpretation": "Categories identify observed rejection stages and GT scoring, not a proven causal attribution. Low fitness can coexist with a map-coordinate bias when the prior is warped."}
    false_cases = [c for c in cases if c["category"] == "false_accept_against_gt"]
    if a.map_dir and false_cases:
        mapping = json.loads((a.map_dir / "mapping_summary.json").read_text())
        drift = np.array(mapping["final_error"]["translation_error_xyz_m"])
        examples = []
        for case in sorted(false_cases, key=lambda c: c["score"]["translation_3d_m"], reverse=True)[:3]:
            error = np.array(case["score"]["translation_error_xyz_m"])
            examples.append({"trial_id": case["trial_id"], "position_error_m": float(np.linalg.norm(error)),
                             "error_xyz_m": error.tolist(), "mapping_final_drift_xyz_m": drift.tolist(),
                             "error_minus_mapping_drift_norm_m": float(np.linalg.norm(error - drift)),
                             "direction_cosine": float(np.dot(error, drift) / (np.linalg.norm(error) * np.linalg.norm(drift))),
                             "global_quality": case["global_quality"], "raw_frontend": case["raw_frontend"],
                             "interpretation": "Directional agreement is evidence consistent with prior-map bias, not proof of a unique cause."})
        result["largest_false_accept_examples"] = examples
    (a.batch / "failure_analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"completed": len(cases), "category_counts": dict(categories)}))


if __name__ == "__main__":
    main()
