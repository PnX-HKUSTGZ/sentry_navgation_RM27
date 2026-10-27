#!/usr/bin/env python3
"""Repository mapping mode, final SLAM Toolbox graph, and corrected 3-D prior.

GT never enters the mapper or scan fusion. The survey can follow the estimated
map->base pose, or an explicitly selected independent simulator controller.
Offline scoring uses GT and one initial common-time gauge transform.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
from slam_toolbox.srv import Pause, SerializePoseGraph
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
import yaml

from evaluation import interpolate, matrix, score_pose
from reset_pose import reset_pose
from sensor_mapping import HERE, ROOT, Route, Survey, base_sensor, stats, stop, scan_timing_metadata
from survey_protocol import (points_hash, read_ordered_protocol, route_arclength,
                             settle_sensor_complete, validate_route_grid)


def se2(value):
    x, y, yaw = value
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    pose[:2, 3] = [x, y]
    return pose


def tf_matrix(msg):
    value = np.eye(4)
    t, q = msg.transform.translation, msg.transform.rotation
    value[:3, 3] = [t.x, t.y, t.z]
    value[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return value


class GraphCorrectionTimeline:
    """Interpolate map corrections only across continuous sensor odometry.

    SLAM may omit nodes while turning in place. A long graph-node interval is
    not itself a sensor gap; the full 3-D motion still comes from LIO.
    """
    def __init__(self, corrections, odometry, max_odometry_gap=0.25):
        self.rows = corrections
        self.stamps = [t for t, _ in corrections]
        stamps = [t for t, _ in odometry]
        if any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError("non-monotonic sensor odometry")
        self.continuous = []
        for ta, tb in zip(self.stamps, self.stamps[1:]):
            left, right = bisect.bisect_right(stamps, ta), bisect.bisect_left(stamps, tb)
            bracket = [ta] + stamps[left:right] + [tb]
            valid = (interpolate(odometry, ta, max_odometry_gap) is not None and
                     interpolate(odometry, tb, max_odometry_gap) is not None and
                     max(np.diff(bracket)) <= max_odometry_gap + 1e-8)
            self.continuous.append(valid)

    def at(self, stamp):
        index = bisect.bisect_left(self.stamps, stamp)
        if index < len(self.stamps) and self.stamps[index] == stamp:
            return self.rows[index][1]
        if index == 0 or index == len(self.stamps) or not self.continuous[index - 1]:
            return None
        return interpolate(self.rows, stamp, float("inf"))


def merge(target, addition):
    for key, value in addition.items():
        if isinstance(value, dict):
            merge(target.setdefault(key, {}), value)
        else:
            target[key] = value
    return target


class StationTurnTracker:
    """Measure signed yaw travel; commanded time never counts as a turn."""
    def __init__(self, yaw, now, turns, timeout, stall_timeout=8.0):
        self.previous_yaw, self.started = float(yaw), float(now)
        self.required = 2.0 * math.pi * turns
        self.timeout, self.stall_timeout = timeout, stall_timeout
        self.progress, self.motion_progress = 0.0, 0.0
        self.last_motion = float(now)

    def update(self, yaw, now):
        delta = math.atan2(math.sin(yaw - self.previous_yaw), math.cos(yaw - self.previous_yaw))
        self.previous_yaw = float(yaw)
        self.progress += delta
        if self.progress - self.motion_progress >= 0.02:
            self.motion_progress, self.last_motion = self.progress, float(now)
        if now - self.started > self.timeout:
            raise TimeoutError("station scan timed out before measured yaw completed the turn")
        if self.progress >= self.required:
            return True
        if now - self.last_motion > self.stall_timeout:
            raise TimeoutError("station scan has no sustained measured angular motion")
        return False


def station_already_scanned(xy, scanned, radius=0.2):
    return any(math.dist(xy, prior) <= radius for prior in scanned)


def station_visit_already_scanned(xy, scanned, station_id=None, scanned_ids=None):
    if station_id is not None:
        return station_id in (scanned_ids or set())
    return station_already_scanned(xy, scanned)


def wait_station_settle(node, control_pose, required, wall_now=time.monotonic):
    """Hold zero command for a full duration of fresh sensor time, not wall time."""
    if not math.isfinite(required) or required < 0:
        raise ValueError("station settle duration must be finite and nonnegative")
    started = wall_now()
    zero_stamp = node.get_clock().now().nanoseconds
    node.command()
    first = None
    previous = None
    while True:
        node.spin_for(0.05)
        control_pose()  # Existing source freshness/estimator-lag checks apply.
        stamp = node.odom[-1][0]
        if previous is not None and stamp < previous - 1e-8:
            raise RuntimeError("station settle sensor clock moved backwards")
        previous = stamp
        if round(stamp * 1e9) > zero_stamp:
            if first is None:
                first = stamp
            if settle_sensor_complete(first, stamp, required):
                return {"requested_settle_sec": required,
                        "settle_zero_command_ros_ns": zero_stamp,
                        "settle_start_sensor_stamp": first,
                        "settle_end_sensor_stamp": stamp,
                        "settle_start_sensor_stamp_ns": round(first * 1e9),
                        "settle_end_sensor_stamp_ns": round(stamp * 1e9),
                        "settle_sensor_duration_sec": stamp - first,
                        "settle_wall_duration_sec": wall_now() - started}
        if wall_now() - started > max(10.0, 5.0 * required + 5.0):
            raise TimeoutError("station settle sensor clock did not advance sufficiently")
        node.command()


def validate_ordered_controls(protocol, args):
    minimum = protocol["minimum_control"]
    values = [args.station_scan_turns, args.station_settle_sec, args.station_scan_speed, args.speed, args.rotation_speed]
    if (not all(math.isfinite(x) for x in values) or args.speed <= 0 or args.station_scan_speed <= 0 or
            args.station_scan_turns < minimum["station_scan_turns"] or
            args.station_settle_sec < minimum["station_settle_sec"] or
            args.station_scan_speed > minimum["station_scan_speed_rad_sec"] or
            args.speed > minimum["maximum_translation_speed_m_sec"] or
            args.rotation_speed != minimum["rotation_speed_rad_sec"] or
            args.survey_terrain != minimum["survey_terrain"] or
            args.control_source != minimum["control_source"]):
        raise ValueError("ordered protocol requires its declared turn/settle/speed/terrain/controller limits")


def surveyed_vertical_velocity(planner, pose, velocity):
    """Independent simulator vehicle follows the surveyed terrain continuously."""
    lookahead = pose[:2, 3] + np.asarray(velocity) * 0.4
    cell = planner.cell(lookahead)
    if not (0 <= cell[0] < planner.ground.shape[0] and 0 <= cell[1] < planner.ground.shape[1]):
        raise RuntimeError("vehicle left the surveyed ground grid")
    height = float(planner.ground[cell]) + 0.08
    return float(np.clip(2.0 * (height - pose[2, 3]), -0.25, 0.25))


class FullSurvey(Survey):
    def __init__(self, out):
        super().__init__(out)
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.slam_poses, self.last_map = [], None
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.slam_pose, 100)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.create_subscription(OccupancyGrid, "/map", self.map_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def slam_pose(self, msg):
        self.slam_poses.append({"stamp": self.stamp(msg), "matrix": matrix(msg.pose.pose).tolist()})

    def map_callback(self, msg):
        self.last_map = {"stamp": self.stamp(msg), "width": msg.info.width,
                         "height": msg.info.height, "resolution": msg.info.resolution,
                         "known_cells": sum(v >= 0 for v in msg.data)}

    def transform(self, parent, child):
        return tf_matrix(self.buffer.lookup_transform(parent, child, Time()))

    def call(self, service_type, name, request, timeout=30):
        client = self.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f"service unavailable: {name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.command()
            self.spin_for(0.05)
        if not future.done() or future.exception():
            raise RuntimeError(f"service failed: {name}")
        result = future.result()
        self.destroy_client(client)
        return result

    def parameter_audit(self):
        expected = {"slam_toolbox": {"use_sim_time": True, "use_scan_matching": True, "do_loop_closing": True,
                                     "transform_publish_period": 0.02},
                    "loam_interface": {"anchor_initial_base_frame": True},
                    "ign_sim_pointcloud_tool": {"instantaneous_scan": True},
                    "point_lio": {"preprocess.instantaneous_scan": True, "prior_pcd.enable": False},
                    "fake_vel_transform": {"output_cmd_vel_topic": "mapping_nav2_cmd_vel"}}
        report = {}
        for name, checks in expected.items():
            client = AsyncParameterClient(self, "/" + name)
            if not client.wait_for_services(timeout_sec=10):
                raise RuntimeError(f"parameters unavailable: {name}")
            future = client.get_parameters(list(checks))
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                self.command()
                self.spin_for(0.05)
            if not future.done():
                raise RuntimeError(f"parameters timeout: {name}")
            actual = dict(zip(checks, map(parameter_value_to_python, future.result().values)))
            subscriptions = self.get_subscriber_names_and_types_by_node(name, "/")
            report[name] = {"actual": actual, "subscriptions": subscriptions}
            if actual != checks or any("ground_truth" in t for t, _ in subscriptions):
                raise RuntimeError(f"full mapping audit failed: {name} {actual}")
        scan_subscriptions = self.get_subscriber_names_and_types_by_node("pointcloud_to_laserscan", "/")
        if not any(topic == "/registered_scan" for topic, _ in scan_subscriptions):
            raise RuntimeError("mapping LaserScan must subscribe to the current registered_scan")
        report["pointcloud_to_laserscan"] = {"subscriptions": scan_subscriptions}
        return report


def export_optimized(out, capture, voxel):
    """Reconstruct clouds using final, timestamped graph corrections, then score."""
    graph = json.loads((out / "graph_optimized.json").read_text())
    nodes = sorted(graph["nodes"], key=lambda n: n["stamp"])
    if len(nodes) < 10 or any(b["stamp"] <= a["stamp"] for a, b in zip(nodes, nodes[1:])):
        raise RuntimeError("insufficient or non-monotonic final graph")
    corrections = [(n["stamp"], se2(n["optimized_xy_yaw"]) @ np.linalg.inv(se2(n["odometry_xy_yaw"])))
                   for n in nodes]
    gt = [(n["stamp"], np.array(n["matrix"])) for n in capture["gt"]]
    if any(b[0] <= a[0] for a, b in zip(gt, gt[1:])):
        raise RuntimeError("nonmonotonic GT")
    odometry = [(n["stamp"], np.array(n["matrix"])) for n in capture["odom"]]
    correction_timeline = GraphCorrectionTimeline(corrections, odometry)
    odom_camera = np.array(capture["T_odom_camera_init"])
    anchor = next((n for n in nodes if interpolate(gt, n["stamp"], 0.25) is not None
                   and interpolate(odometry, n["stamp"], 0.25) is not None), None)
    if anchor is None:
        raise RuntimeError("no initial common graph / odometry / GT stamp")
    # Align the complete estimated BASE pose used by the actual cloud chain.
    # A planar graph pose omits LIO Z/roll/pitch and cannot define this SE(3)
    # gauge on its own. Exactly one shared pose is used, never a trajectory fit.
    anchor_correction = se2(anchor["optimized_xy_yaw"]) @ np.linalg.inv(se2(anchor["odometry_xy_yaw"]))
    anchor_base = (anchor_correction @ odom_camera @ interpolate(odometry, anchor["stamp"], 0.25)
                   @ np.linalg.inv(base_sensor()))
    world_map = interpolate(gt, anchor["stamp"], 0.25) @ np.linalg.inv(anchor_base)
    points, used, dropped = np.empty((0, 3), dtype=np.float32), [], []
    for cloud in capture["clouds"]:
        correction = correction_timeline.at(cloud["stamp"])
        if correction is None:
            dropped.append(cloud["index"])
            continue
        transform = world_map @ correction @ odom_camera
        raw = np.load(out / "registered_scans" / f'{cloud["index"]:06d}.npy')
        transformed = (raw @ transform[:3, :3].T + transform[:3, 3]).astype(np.float32)
        points = np.concatenate((points, transformed))
        used.append(cloud["index"])
        if len(used) % 30 == 0:
            _, indices = np.unique(np.floor(points / voxel).astype(np.int32), axis=0, return_index=True)
            points = points[np.sort(indices)]
    if not len(points):
        raise RuntimeError("no clouds bracketed by the optimized graph")
    _, indices = np.unique(np.floor(points / voxel).astype(np.int32), axis=0, return_index=True)
    points = points[np.sort(indices)].astype("<f4")
    pcd = out / "map.pcd"
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n")
    with pcd.open("wb") as stream:
        stream.write(header.encode())
        stream.write(points.tobytes())
    errors = []
    for row in capture["odom"]:
        t, pose = row["stamp"], np.array(row["matrix"])
        truth, correction = interpolate(gt, t, 0.25), correction_timeline.at(t)
        if truth is None or correction is None:
            continue
        estimate = world_map @ correction @ odom_camera @ pose @ np.linalg.inv(base_sensor())
        errors.append({"stamp": t, "estimated_matrix": estimate.tolist(), "truth_matrix": truth.tolist(),
                       **score_pose(estimate, truth)})
    if not errors:
        raise RuntimeError("no synchronized corrected trajectory for scoring")
    (out / "mapping_errors.json").write_text(json.dumps(errors) + "\n")
    delta = [score_pose(se2(n["optimized_xy_yaw"]), se2(n["odometry_xy_yaw"])) for n in nodes]
    backend_label = "raw GICP sensor odometry" if capture.get("mapping_backend") == "raw_gicp" else "Point-LIO"
    summary = {"method": backend_label + " + SLAM Toolbox final optimized graph", "trajectory_samples": len(errors),
               "clouds": len(used), "graph_nodes": len(nodes), "graph_edges": len(graph["edges"]),
               "capture_complete": capture["failure"] is None and capture["completed_waypoints"] == capture["total_waypoints"],
               "completed_waypoints": capture["completed_waypoints"], "total_waypoints": capture["total_waypoints"],
               "failure": capture["failure"], "translation_3d_m": stats([e["translation_3d_m"] for e in errors]),
               "orientation_so3_deg": stats([e["orientation_so3_deg"] for e in errors]),
               "graph_translation_correction_m": stats([e["translation_3d_m"] for e in delta]),
               "graph_yaw_correction_deg": stats([e["orientation_so3_deg"] for e in delta]),
               "final_error": errors[-1], "dropped_unbracketed_clouds": len(dropped)}
    (out / "mapping_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    manifest = {"schema_version": 2, "frame_id": "map", "T_map_world": np.eye(4).tolist(),
                "pose_source": summary["method"], "slam_toolbox_used": True,
                "optimized_graph_used_for_3d_fusion": True, "planar_correction": "XY/yaw; sensor odometry retains Z/roll/pitch",
                "mapping_backend": capture.get("mapping_backend", "pointlio"),
                "graph_correction_interpolation": "bounded by graph; sensor odometry continuously brackets each interval with <=0.25s gaps",
                "ground_truth_used_for_scan_fusion": False,
                "ground_truth_used_for_control": capture.get("control_source") == "simulator",
                "ideal_3d_map_used": False, "control_source": capture.get("control_source", "estimated"),
                "alignment": "one initial common-time GT pose, after collection; no trajectory fitting",
                "alignment_stamp": anchor["stamp"], "T_world_slam_map": world_map.tolist(),
                "T_odom_camera_init": odom_camera.tolist(), "T_base_lidar": base_sensor().tolist(),
                "input_cloud": "/livox/lidar", "input_imu": "/livox/imu", "voxel_size": voxel,
                **scan_timing_metadata(out / "mapping_params.yaml"),
                "fused_scans": len(used), "dropped_cloud_indices": dropped, "points": len(points),
                "bounds": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                "map_sha256": hashlib.sha256(pcd.read_bytes()).hexdigest(),
                "capture_complete": summary["capture_complete"],
                "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                                  [out / "mapping_params.yaml", out / "graph_optimized.json",
                                   out / "slam.posegraph", out / "slam.data", Path(__file__),
                                   HERE / "slam_graph_export.cpp", *[p for p in
                                   [out / "route.json", out / "survey_route_protocol.json"] if p.is_file()]]}}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "final_error"}), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mapper", type=Path, required=True)
    parser.add_argument("--mapping-backend", choices=("pointlio", "raw_gicp"), default="pointlio")
    parser.add_argument("--speed", type=float, default=0.18)
    parser.add_argument("--rotation-speed", type=float, default=0.0)
    parser.add_argument("--station-scan-speed", type=float, default=1.0)
    parser.add_argument("--station-scan-turns", type=float, default=1.0,
                        help="measured full turns at initialization and unique route stations; 0 selects legacy survey")
    parser.add_argument("--station-settle-sec", type=float, default=0.5,
                        help="zero command for at least this much fresh sensor time before each station turn")
    survey_input = parser.add_mutually_exclusive_group()
    survey_input.add_argument("--survey-waypoints", type=Path, default=HERE / "config/survey_left_loop.json")
    survey_input.add_argument("--survey-route", type=Path,
                              help="ordered geometry-validated protocol; preserve supplied waypoint order without replanning")
    parser.add_argument("--survey-terrain", choices=("flat", "surveyed"), default="flat")
    parser.add_argument("--max-wall-sec", type=float, default=1600)
    parser.add_argument("--max-estimator-lag", type=float, default=0.5,
                        help="stop survey motion while LIO sensor time trails the simulation clock")
    parser.add_argument("--voxel", type=float, default=0.03)
    parser.add_argument("--control-source", choices=("estimated", "simulator"), default="estimated",
                        help="simulator uses GT only for the independent survey vehicle, never for mapping")
    parser.add_argument("--export-only", action="store_true")
    a = parser.parse_args()
    if not math.isfinite(a.max_estimator_lag) or a.max_estimator_lag <= 0:
        parser.error("max-estimator-lag must be positive and finite")
    if not math.isfinite(a.station_scan_turns) or a.station_scan_turns < 0 or not 0 < a.station_scan_speed <= 2.5:
        parser.error("station turns must be nonnegative and scan speed must be in (0, 2.5] rad/s")
    if not math.isfinite(a.station_settle_sec) or a.station_settle_sec < 0:
        parser.error("station-settle-sec must be finite and nonnegative")
    if a.survey_terrain == "surveyed" and a.control_source != "simulator":
        parser.error("surveyed terrain height control requires the independent simulator controller")
    out = a.output.resolve()
    if a.export_only:
        export_optimized(out, json.loads((out / "capture.json").read_text()), a.voxel)
        return
    out.mkdir(parents=True, exist_ok=False)
    params = merge(yaml.safe_load((ROOT / "src/pb2025_nav_bringup/config/simulation/nav2_params.yaml").read_text()),
                   yaml.safe_load((HERE / "config/full_slam_mapping.yaml").read_text()))
    if a.mapping_backend == "raw_gicp":
        raw_profile = yaml.safe_load((HERE / "raw_gicp_mapping/profile.yaml").read_text())
        merge(params["point_lio"]["ros__parameters"], raw_profile["raw_mapping"]["ros__parameters"])
        params["point_lio"]["ros__parameters"]["frontend"]["diagnostic_directory"] = str(out / "raw_frontend")
    params["point_lio"]["ros__parameters"].setdefault("diagnostics", {})["trace_file"] = str(out / "estimator_trace.csv")
    params_path = out / "mapping_params.yaml"
    params_path.write_text(yaml.safe_dump(params))
    route_map = ROOT / "src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml"
    planner = Route(route_map, [-11.7, 3.1], 0.55, terrain=a.survey_terrain)
    protocol = None
    if a.survey_route:
        protocol = read_ordered_protocol(a.survey_route, [-11.7, 3.1])
        validate_ordered_controls(protocol, a)
        (out / "survey_route_protocol.json").write_bytes(a.survey_route.read_bytes())
        route = protocol["waypoints"]
        validate_route_grid(planner, [-11.7, 3.1], route)
        stations = [s["xy"] for s in protocol["stations"] if s["waypoint_index"] >= 0]
        targets = stations
    else:
        targets = json.loads(a.survey_waypoints.read_text())["waypoints"]
        route, stations = planner.make([-11.7, 3.1], targets)
    (out / "route.json").write_text(json.dumps({"waypoints": route, "stations": stations,
        "requested_stations": targets, "survey_waypoints": str(a.survey_waypoints.resolve()) if protocol is None else None,
        "survey_waypoints_sha256": hashlib.sha256(a.survey_waypoints.read_bytes()).hexdigest() if protocol is None else None,
        "survey_route": str(a.survey_route.resolve()) if protocol is not None else None,
        "survey_route_sha256": hashlib.sha256(a.survey_route.read_bytes()).hexdigest() if protocol is not None else None,
        "preserve_waypoint_order": protocol is not None,
        "ordered_waypoints_sha256": points_hash(route),
        "protocol_stations": protocol["stations"] if protocol is not None else None,
        "survey_terrain": a.survey_terrain,
        "ground_heights_m": [float(planner.ground[planner.cell(xy)]) for xy in route],
        "station_scan_turns": a.station_scan_turns, "station_scan_speed_rad_sec": a.station_scan_speed,
        "station_settle_sec": a.station_settle_sec,
        "control_source": a.control_source, "length_m": float(route_arclength([-11.7,3.1], route)[-1])}, indent=2))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise InterruptedError("complete mapping interrupted; evidence retained")
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    node = FullSurvey(out)
    processes, completed, failure, odom_camera = [], 0, None, None
    station_scans, scanned_stations = [], []
    scanned_station_ids = set()
    ordered_stations = {s["waypoint_index"]: s for s in protocol["stations"]} if protocol else {}
    started = time.monotonic()
    try:
        node.command()
        node.spin_for(0.5)
        reset_pose({"x": -11.7, "y": 3.1, "z": 0.08, "yaw": 0.0}, log_path=out / "reset_attempts.json")
        node.spin_for(1)
        for name, command in [
            ("bag", ["ros2", "bag", "record", "--use-sim-time", "-o", str(out / "raw_bag"), "--topics",
                     "/clock", "/livox/lidar", "/livox/imu", "/velodyne_points", "/ground_truth/odometry",
                     "/aft_mapped_to_init", "/cloud_registered", "/registered_scan", "/odometry", "/obstacle_scan",
                     "/cmd_vel", "/map", "/tf", "/tf_static", "/pose", "/slam_toolbox/graph_visualization"]),
            ("mapper", ["ros2", "launch", "pb2025_nav_bringup", "rm_navigation_simulation_launch.py",
                        "slam:=true", "use_ground_truth_odom:=false", "navigation_mode:=legacy", "use_rviz:=false",
                        "slam_scan_cloud_topic:=registered_scan",
                        "mapping_executable:=" + str(a.mapper.resolve()),
                        "params_file:=" + str(params_path)])]:
            with (out / f"{name}.log").open("w") as log:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("mapping process exited at initialization")
            node.command()
            node.spin_for(0.1)
            # A stationary mapper may publish /pose only for its first node,
            # before discovery completes. The latched occupancy map plus both
            # live TF chains establishes readiness without that one-shot event.
            if len(node.odom) >= 20 and len(node.clouds) >= 10 and node.last_map and node.last_map["known_cells"] > 0:
                try:
                    odom_camera = node.transform("odom", "camera_init")
                    initial_map_base = node.transform("map", "base_footprint")
                    break
                except TransformException:
                    pass
        else:
            raise RuntimeError("complete SLAM initialization missing odometry/cloud/graph/map/TF")
        (out / "full_mapping_audit.json").write_text(json.dumps(node.parameter_audit(), indent=2))
        publishers = node.get_publishers_info_by_topic("/cmd_vel")
        if len(publishers) != 1 or publishers[0].node_name != node.get_name():
            raise RuntimeError("survey requires exclusive cmd_vel control")
        audit = subprocess.run(["/usr/bin/python3", str(HERE / "audit_mapping.py"), "--output",
            str(out / "mapping_runtime_parameters.json"), "--expected-mapper", str(a.mapper), "--include-slam"],
            capture_output=True, text=True, timeout=45)
        (out / "mapping_audit.log").write_text(audit.stdout + audit.stderr)
        if audit.returncode:
            raise RuntimeError("sensor-only mapping audit failed")
        # The subprocess audit does not spin this observer. Drain fresh sensor
        # callbacks before checking the control source or starting a turn.
        node.spin_for(0.2)
        # Estimated control uses known spawn only. The simulator controller is
        # a separate evaluation fixture, explicitly marked in the manifest.
        control_alignment = se2([-11.7, 3.1, 0]) @ np.linalg.inv(initial_map_base)
        def control_pose():
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("mapping process exited during survey")
            if time.monotonic() - started > a.max_wall_sec:
                raise TimeoutError("survey exceeded total wall time")
            if time.monotonic() - node.latest_wall > 5:
                raise RuntimeError("Point-LIO odometry stale")
            catchup_started = time.monotonic()
            while node.get_clock().now().nanoseconds * 1e-9 - node.odom[-1][0] > a.max_estimator_lag:
                # A newly received OLD pose is not fresh feedback. Stop motion
                # until the estimator catches up; never rotate through a queue
                # stall merely because the observer still receives callbacks.
                node.command()
                node.spin_for(0.05)
                if time.monotonic() - catchup_started > 5:
                    raise RuntimeError("Point-LIO sensor timestamp remains behind the simulation clock")
            if a.control_source == "simulator":
                if time.monotonic() - node.latest_gt_wall > 2:
                    raise RuntimeError("independent survey controller GT stale")
                return node.gt[-1][1]
            return control_alignment @ node.transform("map", "base_footprint")

        def scan_station(label, station, station_spec=None):
            station_id = station_spec["id"] if station_spec else None
            already = station_visit_already_scanned(station, scanned_stations, station_id, scanned_station_ids)
            if a.station_scan_turns == 0 or already:
                return
            tracker = None
            record = {"label": label, "station_xy": list(station), "status": "settling",
                      "station_id": station_id, "waypoint_index": station_spec["waypoint_index"] if station_spec else None,
                      "kind": station_spec["kind"] if station_spec else "legacy",
                      "control_source": a.control_source, "target_yaw_rad": 2.0 * math.pi * a.station_scan_turns,
                      "commanded_speed_rad_sec": a.station_scan_speed,
                      "capture_start_ros_ns": node.get_clock().now().nanoseconds,
                      "rotation_command_start_ros_ns": None,
                      "sensor_stamp_source": "latest LIO header, may lag command ROS time",
                      "controller_pose_fields_for_estimation": False}
            station_scans.append(record)
            try:
                record.update(wait_station_settle(node, control_pose, a.station_settle_sec))
                pose = control_pose()
                tracker = StationTurnTracker(math.atan2(pose[1, 0], pose[0, 0]), time.monotonic(),
                    a.station_scan_turns, max(30.0, 4.0 * 2.0 * math.pi * a.station_scan_turns / a.station_scan_speed + 10.0))
                record.update(status="running", start_wall=tracker.started, start_xyz=pose[:3,3].tolist(),
                              start_sensor_stamp=node.odom[-1][0], start_sensor_stamp_ns=round(node.odom[-1][0]*1e9))
                while True:
                    node.spin_for(0.05)
                    pose = control_pose()
                    if station_spec and math.dist(pose[:2,3], station) > protocol["station_selection"]["controller_arrival_tolerance_m"]:
                        raise RuntimeError("vehicle left the geometry-validated station center tolerance")
                    if tracker.update(math.atan2(pose[1, 0], pose[0, 0]), time.monotonic()):
                        break
                    if record["rotation_command_start_ros_ns"] is None:
                        record["rotation_command_start_ros_ns"] = node.get_clock().now().nanoseconds
                    node.command(0.0, 0.0, a.station_scan_speed, vz=0.0)
                record["status"] = "complete"
                scanned_stations.append(list(station))
                if station_id is not None:
                    scanned_station_ids.add(station_id)
            except Exception as error:
                record.update(status="failed", failure=repr(error))
                raise
            finally:
                record["stop_command_ros_ns"] = node.get_clock().now().nanoseconds
                node.command()
                node.spin_for(0.5)
                record.update(measured_yaw_rad=tracker.progress if tracker else None, end_wall=time.monotonic(),
                              end_xyz=pose[:3, 3].tolist() if tracker else None, end_sensor_stamp=node.odom[-1][0],
                              end_sensor_stamp_ns=round(node.odom[-1][0]*1e9), capture_end_ros_ns=node.get_clock().now().nanoseconds)
                (out / "station_scans.json").write_text(json.dumps(station_scans, indent=2) + "\n")

        initial_spec = ordered_stations.get(-1)
        scan_station("initial", initial_spec["xy"] if initial_spec else control_pose()[:2, 3].tolist(), initial_spec)
        for index, target in enumerate(route):
            segment = time.monotonic()
            while True:
                node.spin_for(0.05)
                if time.monotonic() - segment > 50:
                    raise TimeoutError(f"survey timeout at waypoint {index}")
                pose = control_pose()
                difference = np.asarray(target) - pose[:2, 3]
                distance = float(np.linalg.norm(difference))
                vertical = surveyed_vertical_velocity(planner, pose, np.zeros(2)) if a.survey_terrain == "surveyed" else 0.0
                # The centre-cell ground height is not an exact chassis contact
                # height: wheels may bridge adjacent terrain. Never deadlock
                # a reached waypoint by commanding descent into that contact.
                if distance < 0.075 and vertical <= 0.05:
                    break
                velocity = difference * min(1.8, a.speed / max(distance, 1e-6))
                vertical = surveyed_vertical_velocity(planner, pose, velocity) if a.survey_terrain == "surveyed" else 0.0
                body = pose[:3, :3].T @ np.r_[velocity, vertical]
                if time.monotonic() - node.latest_wall > 0.8:
                    node.command()
                else:
                    node.command(body[0], body[1], a.rotation_speed, vz=body[2])
            if protocol:
                if index in ordered_stations:
                    scan_station(f"route_{index}", target, ordered_stations[index])
            elif any(math.dist(target, station) < 1e-6 for station in stations):
                scan_station(f"route_{index}", target)
            completed = index + 1
            progress = {"completed_waypoints": completed, "total_waypoints": len(route),
                        "estimated_xy": pose[:2, 3].tolist(), "clouds": len(node.clouds),
                        "slam_poses": len(node.slam_poses), "map": node.last_map,
                        "completed_station_scans": len(scanned_stations),
                        "wall_sec": time.monotonic() - started}
            (out / "progress.json").write_text(json.dumps(progress) + "\n")
            if index % 20 == 0:
                print(json.dumps(progress), flush=True)
        node.command()
        node.spin_for(8)
        paused = node.call(Pause, "/slam_toolbox/pause_new_measurements", Pause.Request())
        if not paused.status:
            raise RuntimeError("failed to pause graph before final serialization")
        request = SerializePoseGraph.Request()
        request.filename = str(out / "slam")
        response = node.call(SerializePoseGraph, "/slam_toolbox/serialize_map", request)
        if response.result != 0:
            raise RuntimeError("native graph serialization failed")
        request_map = SaveMap.Request()
        request_map.map_topic, request_map.map_url = "/map", str(out / "occupancy")
        request_map.image_format, request_map.map_mode = "pgm", "trinary"
        request_map.free_thresh, request_map.occupied_thresh = 0.25, 0.65
        if not node.call(SaveMap, "/map_saver/save_map", request_map).result:
            raise RuntimeError("occupancy map saving failed")
        export = subprocess.run([str(HERE / ".build/slam_export/slam_graph_export"), str(out / "slam"),
                                 str(out / "graph_optimized.json")], capture_output=True, text=True, timeout=30)
        (out / "graph_export.log").write_text(export.stdout + export.stderr)
        if export.returncode:
            raise RuntimeError("native final graph export failed")
    except Exception as error:
        failure = repr(error)
    finally:
        for _ in range(5):
            node.command()
            node.spin_for(0.05)
        for process in reversed(processes):
            stop(process)
        capture = {"completed_waypoints": completed, "total_waypoints": len(route), "failure": failure,
                   "mapping_backend": a.mapping_backend, "max_estimator_lag_sec": a.max_estimator_lag,
                   "control_source": a.control_source,
                   "survey_terrain": a.survey_terrain, "station_scans": station_scans,
                   "T_odom_camera_init": odom_camera.tolist() if odom_camera is not None else None,
                   "odom": [{"stamp": t, "matrix": p.tolist()} for t, p in node.odom],
                   "gt": [{"stamp": t, "matrix": p.tolist()} for t, p in node.gt],
                   "clouds": node.clouds, "commands": node.commands, "online_slam_poses": node.slam_poses,
                   "last_map": node.last_map, "wall_sec": time.monotonic() - started}
        (out / "capture.json").write_text(json.dumps(capture) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)
    export_optimized(out, capture, a.voxel)


if __name__ == "__main__":
    main()
