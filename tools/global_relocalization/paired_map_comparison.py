#!/usr/bin/env python3
"""Paired offline replay: identical captured clouds, parameters and Core, two maps.

Reference-frame scores include the existing raw frontend. For successful online
cases the final base pose is also reconstructed and checked against that run.
Preparation-only cases stay explicitly separate from full confirmed successes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np

from evaluation import interpolate, score_pose
from sensor_map_batch import statistics

HERE = Path(__file__).resolve().parent


def frame_evidence(directory, row):
    frames = []
    for path in sorted((directory / "raw_diagnostics").glob("*/*.json")):
        data = json.loads(path.read_text())
        if data.get("accepted") and "output_pose" in data:
            base = np.array(data["output_pose"]).reshape(4, 4) @ np.linalg.inv(np.array(data["base_from_sensor"]).reshape(4, 4))
            frames.append((data["stamp"], base))
    if not frames:
        raise RuntimeError("no accepted raw reference frame")
    frames.sort(key=lambda f:f[0])
    origin = np.eye(4)
    origin[:3, 3] = frames[0][1][:3, 3]
    gt = [(r["stamp"], np.array(r["matrix"])) for r in row["gt"]]
    initial_truth = interpolate(gt, frames[0][0], .25)
    if initial_truth is None:
        raise RuntimeError("no initial reference / GT bracket")
    true_world_reference = initial_truth @ np.linalg.inv(np.linalg.inv(origin) @ frames[0][1])
    return frames, origin, gt, true_world_reference


def report(out, rows, planned):
    summary = {"planned": planned, "completed": len(rows),
               "comparison_exceptions": sum("exception" in r for r in rows),
               "comparison": "same captured query and available independent clouds; same frozen Core and profile; only map changes",
               "score_frame": "original accumulated-query reference; truth is used after estimation only",
               "full_pipeline_note": "queries from preparation-rejected trials have no stopped confirmation clouds; their results prove preparation behavior only",
               "groups": {}}
    for name in ["sensor_map", "ideal_map"]:
        valid = [r[name] for r in rows if "exception" not in r and name in r and r[name].get("prepared_ready")]
        confirmed = [r[name] for r in rows if "exception" not in r and name in r and r[name].get("success") and r[name].get("confirmation_replayed")]
        all_outputs = [r[name] for r in rows if "exception" not in r and name in r]
        summary["groups"][name] = {"evaluated": len(all_outputs), "prepared_ready": len(valid),
            "reference_strict_correct_when_prepared": sum(r["reference_score"]["strict_correct"] for r in valid),
            "full_confirmation_success": len(confirmed),
            "confirmed_reference_strict_correct": sum(r["reference_score"]["strict_correct"] for r in confirmed),
            "prepared_reference_position_m": statistics([r["reference_score"]["translation_3d_m"] for r in valid]),
            "prepared_reference_orientation_deg": statistics([r["reference_score"]["orientation_so3_deg"] for r in valid])}
    summary["ideal_ready_sensor_rejected"] = sum(r.get("ideal_map", {}).get("prepared_ready", False) and not r.get("sensor_map", {}).get("prepared_ready", False) for r in rows)
    summary["ideal_strict_sensor_not_strict"] = sum(r.get("ideal_map", {}).get("prepared_ready", False) and r["ideal_map"]["reference_score"]["strict_correct"] and
        not (r.get("sensor_map", {}).get("prepared_ready", False) and r["sensor_map"]["reference_score"]["strict_correct"]) for r in rows)
    checks = [r["online_replay_pose_difference_m"] for r in rows if "online_replay_pose_difference_m" in r]
    summary["online_sensor_map_replay_translation_difference_m"] = statistics(checks)
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", type=Path, required=True)
    parser.add_argument("--sensor-map", type=Path, required=True)
    parser.add_argument("--ideal-map", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--limit", type=int)
    a = parser.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    batch = a.batch.resolve()
    trials = json.loads((batch / "trials.json").read_text())["trials"][:a.limit]
    executable = HERE / ".build/map_compare/offline_map_compare"
    core = HERE / ".build/ros/install/global_relocalization/lib/libglobal_relocalization_core.so"
    frozen_core = json.loads((batch / "provenance.json").read_text())["sha256"][str(core.resolve())]
    if hashlib.sha256(core.read_bytes()).hexdigest() != frozen_core:
        raise RuntimeError("comparison Core differs from online batch")
    rows = []
    for trial in trials:
        directory = batch / trial["trial_id"]
        # Coordinator writes its trial log after recorder/node cleanup. Waiting
        # for its checkpoint avoids consuming an in-progress result file.
        while True:
            finished = json.loads((batch / "summary.json").read_text())["completed"]
            if finished > trials.index(trial):
                break
            if not a.watch:
                report(out, rows, len(trials))
                return
            time.sleep(2)
        result_path = out / (trial["trial_id"] + ".json")
        if result_path.exists():
            rows.append(json.loads(result_path.read_text()))
            continue
        row = json.loads((directory / "result.json").read_text())
        pair = {"trial_id": trial["trial_id"], "online_accepted": row.get("action", {}).get("success", False),
                "online_score": row.get("score")}
        try:
            captures = sorted((directory / "query_diagnostics").glob("search_*/retrieval.json"))
            if len(captures) != 1:
                raise RuntimeError(f"expected one frozen query, found {len(captures)}")
            capture = captures[0].parent
            frames, origin, gt, truth_ref = frame_evidence(directory, row)
            metadata = json.loads(captures[0].read_text())
            if not np.allclose(-origin[:3, 3], metadata["sensor_offset"], atol=1e-7):
                raise RuntimeError("raw reference origin differs from query metadata")
            inputs = [capture / "query.pcd", captures[0], batch / (trial["trial_id"] + ".yaml"), executable, core]
            inputs += sorted(capture.glob("independent_*.pcd"))
            pair["inputs_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
            pair["truth_reference_matrix"] = truth_ref.tolist()
            def compare_map(item):
                name, map_path = item
                output = out / (trial["trial_id"] + "_" + name + ".json")
                process = subprocess.run([str(executable), str(map_path.resolve()), str(capture),
                    str(batch / (trial["trial_id"] + ".yaml")), str(output)], capture_output=True, text=True, timeout=90)
                (out / (trial["trial_id"] + "_" + name + ".log")).write_text(process.stdout + process.stderr)
                if process.returncode:
                    raise RuntimeError(f"{name} comparison process exited {process.returncode}")
                value = json.loads(output.read_text())
                value["reference_score"] = score_pose(np.array(value["candidate"]["map_from_reference"]), truth_ref)
                return name, value
            # Each process owns an independent Core. Parallel maps change no
            # algorithm settings and keep the captured clouds read-only.
            with ThreadPoolExecutor(max_workers=2) as pool:
                for name, value in pool.map(compare_map, [("sensor_map", a.sensor_map), ("ideal_map", a.ideal_map)]):
                    pair[name] = value
            # Verify the diagnostic replay reproduces the accepted online pose
            # before treating the pair as controlled evidence.
            if row.get("action", {}).get("success") and pair["sensor_map"]["success"]:
                stamp = row["action"]["pose_stamp_sec"]
                final = next((p for t,p in frames if abs(t-stamp)<1e-7), None)
                if final is None:
                    raise RuntimeError("missing final raw pose for online replay check")
                delta = np.linalg.inv(origin) @ final
                sensor_pose = np.array(pair["sensor_map"]["candidate"]["map_from_reference"]) @ delta
                difference = score_pose(sensor_pose, np.array(row["estimated_pose_matrix"]))
                pair["online_replay_pose_difference_m"] = difference["translation_3d_m"]
                pair["online_replay_orientation_difference_deg"] = difference["orientation_so3_deg"]
                if difference["translation_3d_m"] > 1e-5 or difference["orientation_so3_deg"] > 1e-4:
                    raise RuntimeError("offline replay did not reproduce the online sensor-map pose")
                for name in ["sensor_map", "ideal_map"]:
                    if pair[name]["success"]:
                        candidate_pose = np.array(pair[name]["candidate"]["map_from_reference"]) @ delta
                        pair[name]["current_base_score"] = score_pose(candidate_pose, interpolate(gt, stamp, .25))
        except Exception as error:
            pair["exception"] = repr(error)
        result_path.write_text(json.dumps(pair, indent=2) + "\n")
        rows.append(pair)
        summary = report(out, rows, len(trials))
        print(json.dumps({"completed": len(rows), "trial": trial["trial_id"], "groups": summary["groups"],
                          "exception": pair.get("exception")}), flush=True)
    report(out, rows, len(trials))


if __name__ == "__main__":
    main()
