#!/usr/bin/env python3
"""Full UC sensor-only SLAM evaluation with a separate truth-driven survey vehicle.

Gazebo truth drives the test vehicle and scores the result. It is never published
as SLAM odometry or used to register the mapper's scans. Motion is continuous,
including height changes across surveyed ramps; no pose reset service is used.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
import yaml

from sensor_mapping import HERE, ROOT, Route, Survey, export, mapping_profile, stop


def make_route(map_path, waypoint_path, clearance):
    start = [-11.7, 3.1]
    planner = Route(map_path, start, clearance, region="full", terrain="surveyed")
    targets = json.loads(waypoint_path.read_text())["waypoints"]
    route, stations = planner.make(start, targets)
    heights = [float(planner.ground[planner.cell(xy)]) for xy in route]
    return planner, {"waypoints": route, "ground_heights_m": heights,
                     "stations": stations, "requested_stations": targets,
                     "clearance_m": clearance, "region": "full",
                     "control_source": "Gazebo ground truth; evaluation vehicle only",
                     "continuous_motion": True, "pose_resets": 0,
                     "length_m": sum(math.dist(a, b) for a, b in zip([start] + route, route)),
                     "station_snap_distances_m": [min(math.dist(t, s) for s in stations) for t in targets]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mapper", required=True)
    parser.add_argument("--route-map", type=Path, default=ROOT / "src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml")
    parser.add_argument("--survey-waypoints", type=Path, default=HERE / "config/survey_full_uc.json")
    parser.add_argument("--params", type=Path, default=ROOT / "src/pb2025_nav_bringup/config/simulation/nav2_params.yaml")
    parser.add_argument("--pointlio-overrides", type=Path)
    parser.add_argument("--speed", type=float, default=0.35)
    parser.add_argument("--rotation-speed", type=float, default=0.04)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--clearance", type=float, default=0.55)
    parser.add_argument("--max-wall-sec", type=float, default=1800)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--on-mapper-loss", choices=("stop", "continue"), default="continue",
                        help="continue records the full survey even if SLAM fails; failure remains in the report")
    a = parser.parse_args()
    if not (0 < a.speed <= 0.6 and a.voxel > 0 and a.clearance >= 0.45):
        parser.error("require 0 < speed <= 0.6, voxel > 0, clearance >= 0.45")
    a.method, a.control_source = "repository_slam_full_uc", "ground_truth"
    a.source_revision = "repository snapshot; executable SHA256 in mapping audit"
    planner, route = make_route(a.route_map, a.survey_waypoints, a.clearance)
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / "route.json").write_text(json.dumps(route, indent=2) + "\n")
    if a.plan_only:
        print(json.dumps(route, indent=2))
        return
    params = out / "mapping_params.yaml"
    params.write_text(yaml.safe_dump(mapping_profile(a.params, a.pointlio_overrides)))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise InterruptedError("full UC survey interrupted; partial evidence retained")
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = Survey(out)
    processes, failure, completed, mapping_failure = [], None, 0, None
    start_wall = time.monotonic()
    try:
        commands = [
            ("bag", ["ros2", "bag", "record", "--use-sim-time", "-o", str(out / "raw_bag"), "--topics",
                     "/clock", "/livox/lidar", "/livox/imu", "/velodyne_points", "/ground_truth/odometry",
                     "/aft_mapped_to_init", "/cloud_registered", "/cmd_vel", "/map", "/tf", "/tf_static"]),
            ("mapper", ["ros2", "launch", "pb2025_nav_bringup", "rm_navigation_simulation_launch.py",
                        "slam:=true", "use_ground_truth_odom:=false", "navigation_mode:=legacy",
                        "use_rviz:=false", "params_file:=" + str(params)]),
        ]
        for name, command in commands:
            with (out / f"{name}.log").open("w") as log:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 90
        while (len(node.odom) < 20 or len(node.clouds) < 10 or len(node.gt) < 20) and time.monotonic() < deadline:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("mapping child exited during initialization")
            node.command()
            node.spin_for(0.1)
        if len(node.odom) < 20 or len(node.clouds) < 10 or len(node.gt) < 20:
            raise RuntimeError("missing mapping/ground truth streams")
        audit = subprocess.run(["/usr/bin/python3", str(HERE / "audit_mapping.py"), "--output",
                                str(out / "mapping_runtime_parameters.json"), "--expected-mapper", a.mapper, "--include-slam"],
                               capture_output=True, text=True, timeout=40)
        (out / "mapping_audit.log").write_text(audit.stdout + audit.stderr)
        if audit.returncode:
            raise RuntimeError("sensor-only mapper audit failed")
        initial_gt = node.gt[-1][1]
        if np.linalg.norm(initial_gt[:2, 3] - [-11.7, 3.1]) > 0.3:
            raise RuntimeError("survey requires a fresh simulation at the configured spawn")
        for index, target in enumerate(route["waypoints"]):
            segment_wall = time.monotonic()
            while True:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("mapping child exited during survey")
                if time.monotonic() - node.latest_gt_wall > 2:
                    raise RuntimeError("survey control ground truth stopped")
                if time.monotonic() - start_wall > a.max_wall_sec or time.monotonic() - segment_wall > 45:
                    raise TimeoutError(f"survey timed out at waypoint {index}")
                if time.monotonic() - node.latest_wall > 6:
                    mapping_failure = mapping_failure or "Point-LIO odometry stopped during full-field survey"
                    if a.on_mapper_loss == "stop":
                        raise RuntimeError(mapping_failure)
                if a.on_mapper_loss == "stop" and node.gt[-1][0] - node.odom[-1][0] > 2:
                    # Rendering and registration may briefly fall behind. Hold
                    # the vehicle until processing catches up; the wall/stale
                    # limits still turn persistent failures into partial runs.
                    node.command()
                    node.spin_for(0.1)
                    continue
                pose = node.gt[-1][1]
                difference = np.asarray(target) - pose[:2, 3]
                distance = np.linalg.norm(difference)
                if distance < 0.07:
                    break
                velocity = difference * min(4.0, a.speed / distance)
                # Follow surveyed height using a bounded vertical velocity,
                # retaining continuous physical motion through ramp corridors.
                lookahead = pose[:2, 3] + velocity * 0.4
                cell = planner.cell(lookahead)
                if not (0 <= cell[0] < planner.ground.shape[0] and 0 <= cell[1] < planner.ground.shape[1]):
                    raise RuntimeError("vehicle left the surveyed map")
                height = planner.ground[cell] + 0.08
                vertical = float(np.clip(2.0 * (height - pose[2, 3]), -0.25, 0.25))
                body = pose[:3, :3].T @ np.r_[velocity, vertical]
                node.command(body[0], body[1], a.rotation_speed, vz=body[2])
                node.spin_for(0.05)
            completed = index + 1
            (out / "progress.json").write_text(json.dumps({"completed_waypoints": completed,
                "total_waypoints": len(route["waypoints"]), "truth_xyz": pose[:3, 3].tolist(),
                "clouds": len(node.clouds), "wall_sec": time.monotonic() - start_wall}) + "\n")
            if index % 40 == 0:
                print((out / "progress.json").read_text().strip(), flush=True)
        node.command()
        node.spin_for(2)
    except Exception as error:
        failure = repr(error)
    finally:
        for _ in range(5):
            node.command()
            node.spin_for(0.05)
        try:
            save = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(out / "occupancy"),
                                   "--ros-args", "-p", "use_sim_time:=true", "-p", "save_map_timeout:=5.0"],
                                  capture_output=True, text=True, timeout=15)
            (out / "save_occupancy.log").write_text(save.stdout + save.stderr)
        except subprocess.TimeoutExpired:
            (out / "save_occupancy.log").write_text("Map saver timed out; report can recover the last /map from raw_bag.\n")
        for process in reversed(processes):
            stop(process)
        failure = failure or mapping_failure
        evidence = {"completed_waypoints": completed, "total_waypoints": len(route["waypoints"]),
                    "failure": failure, "mapping_failure": mapping_failure,
                    "invalid_odometry_messages": node.invalid_odometry,
                    "survey_route_complete": completed == len(route["waypoints"]),
                    "control_source": a.control_source, "region": "full",
                    "wall_sec": time.monotonic() - start_wall,
                    "odom": [{"stamp": t, "matrix": p.tolist()} for t, p in node.odom],
                    "gt": [{"stamp": t, "matrix": p.tolist()} for t, p in node.gt],
                    "clouds": node.clouds, "commands": node.commands}
        (out / "capture.json").write_text(json.dumps(evidence) + "\n")
        try:
            if node.clouds and node.odom:
                export(node, out, params, a)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)


if __name__ == "__main__":
    main()
