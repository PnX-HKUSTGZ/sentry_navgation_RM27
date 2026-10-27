#!/usr/bin/env python3
"""Export full-field trajectory, geometry, occupancy and coverage evidence without fitting."""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml

from audit_prior_geometry import read_xyz
from sensor_mapping import base_sensor, stats


def recover_occupancy(run):
    """Retain the actual last SLAM grid even when the driving test aborted."""
    if (run / "occupancy.yaml").exists() or not (run / "raw_bag/metadata.yaml").exists():
        return
    import rosbag2_py
    from nav_msgs.msg import OccupancyGrid
    from rclpy.serialization import deserialize_message
    from scipy.spatial.transform import Rotation
    metadata = yaml.safe_load((run / "raw_bag/metadata.yaml").read_text())["rosbag2_bagfile_information"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(run / "raw_bag"), storage_id=metadata["storage_identifier"]),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/map"]))
    last = None
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic == "/map":
            last = payload
    if last is None:
        return
    msg = deserialize_message(last, OccupancyGrid)
    values = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
    image = np.full(values.shape, 128, np.uint8)
    image[(values >= 0) & (values <= 25)] = 254
    image[values >= 65] = 0
    Image.fromarray(image[::-1]).save(run / "occupancy.pgm")
    origin = msg.info.origin
    yaw = Rotation.from_quat([origin.orientation.x, origin.orientation.y,
                              origin.orientation.z, origin.orientation.w]).as_euler("xyz")[2]
    config = {"image": "occupancy.pgm", "mode": "trinary", "resolution": msg.info.resolution,
              "origin": [origin.position.x, origin.position.y, float(yaw)], "negate": 0,
              "occupied_thresh": .65, "free_thresh": .25}
    (run / "occupancy.yaml").write_text(yaml.safe_dump(config))
    (run / "occupancy_recovery.json").write_text(json.dumps({
        "source": "last recorded /map in raw_bag; no registration or ground-truth reconstruction",
        "frame_id": msg.header.frame_id, "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "unknown_pixel": 128, "reason": "live map saver did not produce occupancy.yaml"}, indent=2) + "\n")


def occupancy_metrics(run, reference_yaml, transform):
    saved = run / "occupancy.yaml"
    if not saved.exists():
        return {"available": False, "reason": "SLAM Toolbox occupancy map was not saved"}
    cfg = yaml.safe_load(saved.read_text())
    ref = yaml.safe_load(reference_yaml.read_text())
    image = np.asarray(Image.open(saved.parent / cfg["image"]))[::-1]
    target = np.asarray(Image.open(reference_yaml.parent / ref["image"]))[::-1]
    def masks(pixels, config):
        probability = pixels / 255. if config.get("negate", 0) else 1. - pixels / 255.
        return probability >= config["occupied_thresh"], probability <= config["free_thresh"]
    occupied, free = masks(image, cfg)
    ref_occ, ref_free = masks(target, ref)
    rows, cols = np.where(occupied | free)
    yaw = cfg["origin"][2]
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    xy = np.column_stack((cols + .5, rows + .5)) * cfg["resolution"] @ rotation.T + cfg["origin"][:2]
    world = np.column_stack((xy, np.zeros(len(xy)))) @ transform[:3, :3].T + transform[:3, 3]
    # Repository RMUC reference has zero map-origin yaw.
    if abs(ref["origin"][2]) > 1e-10:
        raise ValueError("reference map yaw must be zero")
    cells = np.floor((world[:, :2] - ref["origin"][:2]) / ref["resolution"]).astype(int)
    inside = ((cells[:, 0] >= 0) & (cells[:, 0] < target.shape[1]) &
              (cells[:, 1] >= 0) & (cells[:, 1] < target.shape[0]))
    rows, cols, cells = rows[inside], cols[inside], cells[inside]
    known = (ref_occ | ref_free)[cells[:, 1], cells[:, 0]]
    rows, cols, cells = rows[known], cols[known], cells[known]
    a, b = occupied[rows, cols], ref_occ[cells[:, 1], cells[:, 0]]
    tp, fp, fn = int(np.sum(a & b)), int(np.sum(a & ~b)), int(np.sum(~a & b))
    return {"available": True, "comparison_cells": len(a), "true_positive": tp,
            "false_positive": fp, "false_negative": fn,
            "occupied_iou": tp / (tp + fp + fn) if tp + fp + fn else None,
            "occupied_precision": tp / (tp + fp) if tp + fp else None,
            "occupied_recall": tp / (tp + fn) if tp + fn else None,
            "known_reference_area_observed_fraction": len(np.unique(cells, axis=0)) / int(np.sum(ref_occ | ref_free)),
            "interpretation": "One fixed start gauge, no grid registration; 2D height filtering, visibility and unknown areas affect these scores."}


def report(run, reference, map_yaml):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    errors = json.loads((run / "mapping_errors.json").read_text())
    summary = json.loads((run / "mapping_summary.json").read_text())
    manifest = json.loads((run / "manifest.json").read_text())
    capture = json.loads((run / "capture.json").read_text())
    route = json.loads((run / "route.json").read_text())
    if not errors:
        raise ValueError("no timestamp-matched trajectory samples")
    truth = np.asarray([r["truth_matrix"] for r in errors])[:, :3, 3]
    estimate = np.asarray([r["estimated_matrix"] for r in errors])[:, :3, 3]
    all_truth = np.asarray([r["matrix"] for r in capture["gt"]])[:, :3, 3]
    stations = np.asarray(route["stations"])
    station_distances = cKDTree(all_truth[:, :2]).query(stations)[0]
    summary["coverage"] = {"truth_bounds_xyz": [all_truth.min(0).tolist(), all_truth.max(0).tolist()],
                           "stations_visited_within_0_2m": int(np.sum(station_distances <= .2)),
                           "total_stations": len(stations), "station_min_distance_m": station_distances.tolist(),
                           "both_halves_visited": bool(all_truth[:, 0].min() < -10 and all_truth[:, 0].max() > 10),
                           "return_distance_to_spawn_m": float(np.linalg.norm(all_truth[-1, :2] - [-11.7, 3.1])),
                           "truth_path_length_m": float(np.linalg.norm(np.diff(all_truth, axis=0), axis=1).sum())}
    summary["per_half"] = {name: {"translation_3d_m": stats([r["translation_3d_m"] for r, keep in zip(errors, mask) if keep]),
                                  "orientation_so3_deg": stats([r["orientation_so3_deg"] for r, keep in zip(errors, mask) if keep])}
                           for name, mask in (("left", truth[:, 0] < 0), ("right", truth[:, 0] >= 0))}
    profile = yaml.safe_load((run / "mapping_params.yaml").read_text())
    slam = profile["slam_toolbox"]["ros__parameters"]
    summary["slam_toolbox"] = {k: slam[k] for k in ("use_scan_matching", "do_loop_closing", "transform_publish_period")}
    for audit_path in (run / "mapping_runtime_parameters.json", run / "mapping_runtime_with_slam.json"):
        if audit_path.exists():
            audit = json.loads(audit_path.read_text())
            if audit.get("passed") and "slam_toolbox" in audit:
                summary["slam_toolbox"].update(audit["slam_toolbox"]["actual"])
                summary["slam_toolbox"]["runtime_verified"] = True
    summary["control_source"] = manifest["control_source"]
    summary["survey_route_complete"] = capture.get("survey_route_complete", summary["capture_complete"])
    span = capture["gt"][-1]["stamp"] - capture["gt"][0]["stamp"]
    summary["trajectory_scoring"] = {
        "truth_duration_sec": span,
        "scored_first_stamp": errors[0]["stamp"], "scored_last_stamp": errors[-1]["stamp"],
        "scored_time_span_fraction": (errors[-1]["stamp"] - errors[0]["stamp"]) / span if span else 0.,
        "strict_correct_fraction": float(np.mean([r["strict_correct"] for r in errors])),
        "strict_thresholds": "3D position <= 0.10 m and SO(3) orientation <= 2 deg",
        "missing_truth_is_not_zero_error": True,
        "invalid_odometry_messages": capture.get("invalid_odometry_messages", 0)}
    summary["mapping_output_covered_full_survey"] = errors[-1]["stamp"] >= capture["gt"][-1]["stamp"] - 2.
    summary["alignment"] = manifest["alignment"]
    summary["completed_full_field"] = bool(summary["capture_complete"] and summary["mapping_output_covered_full_survey"] and summary["coverage"]["both_halves_visited"] and
                                             np.all(station_distances <= .2))
    prior = read_xyz(run / "map.pcd")
    distances = None
    if reference is not None:
        target = read_xyz(reference)
        distances = cKDTree(target).query(prior, workers=2)[0]
        reverse = cKDTree(prior).query(target, workers=2)[0]
        summary["geometry"] = {"evaluation_only": True, "region": "full", "registration_fitted": False,
                               "reference": str(reference.resolve()),
                               "reference_sha256": hashlib.sha256(reference.read_bytes()).hexdigest(),
                               "map_to_reference_m": stats(distances), "reference_to_map_m": stats(reverse),
                               "map_fraction_within_0_1m": float(np.mean(distances <= .1)),
                               "reference_fraction_within_0_1m": float(np.mean(reverse <= .1)),
                               "interpretation": "Nearest sampled surface disagreement includes visibility and sampling differences; reverse distance includes missing coverage."}
        np.savez_compressed(run / "geometry_errors.npz", xyz=prior, distance_m=distances)
    world_map = np.asarray(manifest["T_world_camera_init"]) @ np.linalg.inv(base_sensor())
    recover_occupancy(run)
    summary["occupancy"] = occupancy_metrics(run, map_yaml, world_map)
    (run / "full_uc_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    with (run / "trajectory_errors.csv").open("w") as stream:
        fields = ["stamp", "translation_3d_m", "translation_xy_m", "orientation_so3_deg", "yaw_error_deg"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: row[k] for k in fields} for row in errors)
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=True)
    cfg = yaml.safe_load(map_yaml.read_text())
    background = np.asarray(Image.open(map_yaml.parent / cfg["image"]))[::-1]
    extent = [cfg["origin"][0], cfg["origin"][0] + background.shape[1] * cfg["resolution"],
              cfg["origin"][1], cfg["origin"][1] + background.shape[0] * cfg["resolution"]]
    ax = axes[0, 0]
    ax.imshow(background, origin="lower", extent=extent, cmap="gray", alpha=.4)
    planned = np.asarray(route["waypoints"])
    ax.plot(planned[:, 0], planned[:, 1], "--", color="gray", lw=.6, label="Planned")
    ax.plot(all_truth[:, 0], all_truth[:, 1], color="green", lw=1.3, label="Gazebo truth (entire capture)")
    ax.plot(estimate[:, 0], estimate[:, 1], color="deepskyblue", lw=1., label="Point-LIO")
    ax.scatter(stations[:, 0], stations[:, 1], s=15, color="orange", label="Stations")
    ax.set(title="Full UC continuous survey (one start-pose alignment)", xlabel="World X [m]", ylabel="World Y [m]")
    ax.set_aspect("equal"); ax.legend(fontsize=8)
    ax.set_xlim(extent[0] - 1, extent[1] + 1)
    ax.set_ylim(extent[2] - 1, extent[3] + 1)
    outside = (estimate[:, 0] < extent[0]) | (estimate[:, 0] > extent[1]) | (estimate[:, 1] < extent[2]) | (estimate[:, 1] > extent[3])
    ax.text(.02, .02, f"Estimates outside field: {np.mean(outside):.1%}", transform=ax.transAxes, fontsize=8)
    elapsed = np.asarray([r["stamp"] for r in errors]) - errors[0]["stamp"]
    for ax, key, title in ((axes[0, 1], "translation_3d_m", "3D translation error [m]"),
                            (axes[1, 0], "orientation_so3_deg", "SO(3) orientation error [deg]")):
        ax.plot(elapsed, [r[key] for r in errors], lw=1.)
        ax.set(title=title, xlabel="Simulation time since alignment [s]")
        ax.grid(alpha=.25)
    ax = axes[1, 1]
    stride = max(1, len(prior) // 90000)
    shown = prior[::stride]
    color = np.minimum(distances[::stride], .5) if distances is not None else shown[:, 2]
    plot = ax.scatter(shown[:, 0], shown[:, 1], c=color, s=.2, cmap="turbo", vmin=0, vmax=.5 if distances is not None else None)
    fig.colorbar(plot, ax=ax, label="Surface disagreement [m], clipped at 0.5" if distances is not None else "Height [m]")
    ax.set_aspect("equal"); ax.set(title="Sensor-estimated map (field view; full points saved)", xlabel="World X [m]", ylabel="World Y [m]")
    ax.set_xlim(extent[0] - 1, extent[1] + 1)
    ax.set_ylim(extent[2] - 1, extent[3] + 1)
    status = "COMPLETE FULL FIELD" if summary["completed_full_field"] else "PARTIAL / FAILED — not a full-field result"
    fig.suptitle(f"{status} | ATE RMSE {summary['translation_3d_m']['rmse']:.3f} m | "
                 f"P95 {summary['translation_3d_m']['p95']:.3f} m")
    fig.savefig(run / "mapping_evaluation.png", dpi=150)
    fig.savefig(run / "mapping_evaluation.svg")
    plt.close(fig)
    content = (f"<!doctype html><meta charset='utf-8'><title>Full UC mapping evaluation</title>"
               "<style>body{font:16px sans-serif;max-width:1400px;margin:32px auto;background:#171d26;color:#e5edf7}"
               "img{width:100%;background:white}pre{white-space:pre-wrap}</style>"
               f"<h1>{html.escape(status)}</h1><p>Point-LIO trajectory and 3D map; SLAM Toolbox 2D occupancy. "
               "Ground truth drives the survey vehicle and scores the result, never the SLAM estimator. "
               "One start-pose alignment; no trajectory/ICP fitting.</p><p>"
               f"SLAM Toolbox scan matching: {slam['use_scan_matching']}; loop closing: {slam['do_loop_closing']}.</p>"
               f"<img src='mapping_evaluation.png'><pre>{html.escape(json.dumps(summary, indent=2))}</pre>")
    (run / "report.html").write_text(content)
    print(json.dumps({k: summary[k] for k in ("completed_full_field", "translation_3d_m", "coverage", "slam_toolbox")}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--route-map", type=Path, default=Path(__file__).resolve().parents[2] / "src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml")
    args = parser.parse_args()
    report(args.result, args.reference, args.route_map)


if __name__ == "__main__":
    main()
