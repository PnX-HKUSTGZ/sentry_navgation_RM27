#!/usr/bin/env python3
"""Offline evidence of final graph correction and independent trajectory scores."""
import argparse
import json
from pathlib import Path
import numpy as np

from evaluation import interpolate, score_pose
from full_slam_mapping import se2
from sensor_mapping import base_sensor, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True)
    a = parser.parse_args()
    out = a.map_dir
    capture = json.loads((out / "capture.json").read_text())
    graph = json.loads((out / "graph_optimized.json").read_text())
    online = {round(n["stamp"], 6): np.array(n["matrix"]) for n in capture["online_slam_poses"]}
    updates = []
    for n in graph["nodes"]:
        prior = online.get(round(n["stamp"], 6))
        if prior is not None:
            error = score_pose(se2(n["optimized_xy_yaw"]), prior)
            updates.append({"id": n["id"], "stamp": n["stamp"], **error})
    gt = [(n["stamp"], np.array(n["matrix"])) for n in capture["gt"]]
    raw = [(n["stamp"], np.array(n["matrix"])) for n in capture["odom"]]
    first = next((n for n in raw if interpolate(gt, n[0], 0.25) is not None), None)
    if first is None:
        raise RuntimeError("no raw trajectory / GT initial bracket")
    initial_gt = interpolate(gt, first[0], 0.25)
    world_camera = initial_gt @ base_sensor() @ np.linalg.inv(first[1])
    raw_errors = []
    for t, pose in raw:
        truth = interpolate(gt, t, 0.25)
        if truth is not None:
            raw_errors.append({"stamp": t, **score_pose(world_camera @ pose @ np.linalg.inv(base_sensor()), truth)})
    result = {"graph_nodes": len(graph["nodes"]), "graph_edges": len(graph["edges"]),
              "online_final_matched_nodes": len(updates),
              "historical_nodes_updated": sum(u["translation_3d_m"] > 1e-5 or u["orientation_so3_deg"] > 1e-4 for u in updates),
              "retrospective_translation_change_m": stats([u["translation_3d_m"] for u in updates]),
              "retrospective_yaw_change_deg": stats([u["orientation_so3_deg"] for u in updates]),
              "largest_retrospective_updates": sorted(updates, key=lambda u: u["translation_3d_m"], reverse=True)[:10],
              "edges_with_id_separation_over_10": sum(abs(a-b) > 10 for a,b in graph["edges"]),
              "edge_note": "nonconsecutive edges alone do not prove which edges are loop closures; both local links and loop optimization can update history",
              "raw_pointlio_trajectory": {"alignment": "one initial shared GT pose, independent gauge, no trajectory fitting",
                  "translation_3d_m": stats([r["translation_3d_m"] for r in raw_errors]),
                  "orientation_so3_deg": stats([r["orientation_so3_deg"] for r in raw_errors]),
                  "final_error": raw_errors[-1] if raw_errors else None},
              "mapping_input_gt": False, "modified_map": False}
    (out / "graph_correction_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k:v for k,v in result.items() if k not in ["largest_retrospective_updates", "raw_pointlio_trajectory"]}))


if __name__ == "__main__":
    main()
