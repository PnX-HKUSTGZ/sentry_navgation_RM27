#!/usr/bin/env python3
"""Drive a continuous LiDAR/IMU survey and export a sensor-estimated prior.

The occupancy grid plans the survey route only. Control uses Point-LIO odometry
and the known spawn pose. GT is recorded but read only after stopping mapping,
for a single start-frame alignment and independent trajectory error evaluation.
"""
import argparse
import hashlib
import heapq
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from evaluation import matrix, interpolate, score_pose

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def mapping_profile(path, overrides=None):
    profile = yaml.safe_load(path.read_text())
    def merge(target, addition):
        for key, value in addition.items():
            if isinstance(value, dict):
                merge(target.setdefault(key, {}), value)
            else:
                target[key] = value
    if overrides:
        addition = yaml.safe_load(overrides.read_text())
        if set(addition) != {"point_lio"}:
            raise ValueError("mapping overrides must target Point-LIO only")
        merge(profile, addition)
    return profile


def base_sensor():
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("x", -math.pi / 6).as_matrix()
    value[:3, 3] = [0, 0.126, 0.130]
    return value


def scan_timing_metadata(params):
    """Describe the configured sensor model without claiming legacy deskew is valid."""
    profile = yaml.safe_load(Path(params).read_text())
    if not isinstance(profile, dict):
        return {"scan_model": "unknown; configuration unavailable"}
    converter = profile.get("ign_sim_pointcloud_tool", {}).get("ros__parameters", {})
    pointlio = profile.get("point_lio", {}).get("ros__parameters", {})
    instantaneous = converter.get("instantaneous_scan")
    estimator_instantaneous = pointlio.get("preprocess", {}).get("instantaneous_scan", False)
    return {"scan_model": "instantaneous Gazebo GPU lidar",
            "mapping_converter_timing": ("unspecified; inspect converter runtime audit" if instantaneous is None
                                          else "zero per-point offsets" if instantaneous else "legacy synthetic per-point offsets"),
            "pointlio_instantaneous_scan": estimator_instantaneous,
            "per_point_deskew": not estimator_instantaneous,
            "timing_model_consistent": None if instantaneous is None else bool(instantaneous and estimator_instantaneous)}


def stats(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return {"count": 0}
    return {"count": len(values), "mean": float(np.mean(values)),
            "rmse": float(np.sqrt(np.mean(values**2))),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values))}


class Route:
    def __init__(self, map_yaml, start, clearance, region="left", terrain="flat"):
        config = yaml.safe_load(map_yaml.read_text())
        self.resolution = config["resolution"]
        self.origin = np.array(config["origin"][:2])
        pixels = np.array(Image.open(map_yaml.parent / config["image"]))[::-1]
        free = pixels > 250
        self.ground = np.zeros(free.shape)
        grid = config.get("ground_elevation", {}).get("grid")
        if grid:
            ground_pixels = np.array(Image.open(map_yaml.parent / grid["image"]))[::-1]
            ground = ground_pixels.astype(float) * grid["scale"] + grid["offset"]
            self.ground = ground
            # The survey's velocity adapter holds vertical velocity at zero.
            # Restrict its route to surveyed flat ground, excluding unknown
            # cells and raised surfaces before footprint inflation.
            free &= ground_pixels != grid["no_data"]
            if terrain == "flat":
                free &= np.abs(ground) <= 0.03
        # Treat outside the supplied map as occupied before footprint inflation.
        free[[0, -1], :] = False
        free[:, [0, -1]] = False
        self.safe = distance_transform_edt(free) * self.resolution >= clearance
        # Left-half survey, including the center corridor, matches the fixed
        # negative-x competition search region. Never use the ideal 3D PCD.
        xs = self.origin[0] + (np.arange(self.safe.shape[1]) + 0.5) * self.resolution
        if region == "left":
            self.safe[:, xs > -0.4] = False
        components, _ = label(self.safe, structure=np.ones((3, 3)))
        cell = self.cell(start)
        if not self.safe[cell]:
            raise ValueError("survey start violates clearance")
        self.safe &= components == components[cell]
        self.cells = np.argwhere(self.safe)

    def cell(self, xy):
        col, row = np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)
        return int(row), int(col)

    def xy(self, cell):
        return self.origin + (np.array(cell)[::-1] + 0.5) * self.resolution

    def nearest(self, xy):
        target = np.array(self.cell(xy))
        return tuple(self.cells[np.argmin(np.sum((self.cells - target)**2, axis=1))])

    def astar(self, start, goal):
        queue = [(0.0, start)]
        distances, parent = {start: 0.0}, {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                path = [current]
                while current in parent:
                    current = parent[current]
                    path.append(current)
                return list(reversed(path))
            for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0),
                           (1, 1), (-1, -1), (1, -1), (-1, 1)):
                nxt = current[0] + dy, current[1] + dx
                if not (0 <= nxt[0] < self.safe.shape[0] and 0 <= nxt[1] < self.safe.shape[1]):
                    continue
                if not self.safe[nxt] or not self.safe[current[0], nxt[1]] or not self.safe[nxt[0], current[1]]:
                    continue
                distance = distances[current] + math.hypot(dx, dy)
                if distance < distances.get(nxt, float("inf")):
                    distances[nxt], parent[nxt] = distance, current
                    heapq.heappush(queue, (distance + math.dist(nxt, goal), nxt))
        raise ValueError("no continuous survey path")

    def make(self, start, targets):
        def simplify(path):
            chosen = [path[0]]
            for i in range(1, len(path) - 1):
                before = np.subtract(path[i], path[i - 1])
                after = np.subtract(path[i + 1], path[i])
                if not np.array_equal(before, after) or math.dist(chosen[-1], path[i]) >= 5:
                    chosen.append(path[i])
            chosen.append(path[-1])
            return chosen
        remaining = set(self.nearest(target) for target in targets)
        current, route = self.nearest(start), []
        stations = []
        while remaining:
            goal = min(remaining, key=lambda p: math.dist(p, current))
            path = self.astar(current, goal)
            # Retain corners and intermediate points at <= 0.25 m. Dense path
            # tracking avoids shortcuts across inflated obstacle corners.
            chosen = simplify(path)
            route.extend(self.xy(c).tolist() for c in chosen[1:])
            stations.append(self.xy(goal).tolist())
            current = goal
            remaining.remove(goal)
        path = self.astar(current, self.nearest(start))
        route.extend(self.xy(c).tolist() for c in simplify(path)[1:])
        return route, stations


class Survey(Node):
    def __init__(self, out):
        super().__init__("sensor_mapping_survey")
        self.out = out
        (out / "registered_scans").mkdir()
        self.gt, self.odom, self.clouds, self.commands = [], [], [], []
        self.latest, self.latest_wall = None, 0.0
        self.latest_gt_wall, self.invalid_odometry = 0.0, 0
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/ground_truth/odometry", self.truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/aft_mapped_to_init", self.pose, 100)
        self.create_subscription(PointCloud2, "/cloud_registered", self.cloud, 20)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def truth(self, msg):
        # Observation only: no control or fusion method reads self.gt.
        self.latest_gt_wall = time.monotonic()
        self.gt.append((self.stamp(msg), matrix(msg.pose.pose)))

    def pose(self, msg):
        try:
            pose = matrix(msg.pose.pose)
            if not np.isfinite(pose).all():
                raise ValueError("nonfinite odometry")
        except ValueError:
            self.invalid_odometry += 1
            return
        self.latest, self.latest_wall = pose, time.monotonic()
        if not self.odom or self.stamp(msg) - self.odom[-1][0] >= 0.049:
            self.odom.append((self.stamp(msg), self.latest))

    def cloud(self, msg):
        raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.column_stack([raw[n].reshape(-1) for n in ("x", "y", "z")])
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        index = len(self.clouds)
        np.save(self.out / "registered_scans" / f"{index:06d}.npy", xyz)
        self.clouds.append({"index": index, "stamp": self.stamp(msg),
                            "frame": msg.header.frame_id, "points": len(xyz)})

    def command(self, vx=0, vy=0, omega=0, vz=0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(omega)
        msg.linear.z = float(vz)
        self.publisher.publish(msg)
        self.commands.append({"wall": time.monotonic(), "vx": vx, "vy": vy, "vz": vz, "omega": omega})

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def export(node, out, params, a):
    # Only after mapping has stopped, fix the arbitrary SLAM gauge from ONE
    # timestamp. No trajectory fit, ICP to the ideal PCD, scale correction,
    # per-scan GT fusion, or loop-closure correction is performed.
    if any(b[0] <= p[0] for p, b in zip(node.gt, node.gt[1:])):
        raise RuntimeError("GT timestamp order invalid; refuse misleading map error statistics")
    shared = next(((t, p, interpolate(node.gt, t, 0.25)) for t, p in node.odom
                   if interpolate(node.gt, t, 0.25) is not None), None)
    if shared is None:
        raise RuntimeError("missing initial GT bracket for evaluation alignment")
    stamp, initial, gt = shared
    world_camera = gt @ base_sensor() @ np.linalg.inv(initial)
    points = np.empty((0, 3), np.float32)
    for start in range(0, len(node.clouds), 100):
        chunk = np.concatenate([np.load(out / "registered_scans" / f"{i:06d}.npy")
                                for i in range(start, min(start + 100, len(node.clouds)))])
        points = np.concatenate((points, chunk))
        _, keep = np.unique(np.floor(points / a.voxel).astype(np.int32), axis=0, return_index=True)
        points = points[np.sort(keep)]
    points = (points @ world_camera[:3, :3].T + world_camera[:3, 3]).astype("<f4")
    pcd = out / "map.pcd"
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
              f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(points)}\nDATA binary\n")
    with pcd.open("wb") as stream:
        stream.write(header.encode())
        stream.write(points.tobytes())
    errors = []
    for t, pose in node.odom:
        truth = interpolate(node.gt, t, 0.25)
        if truth is None:
            continue
        estimate = world_camera @ pose @ np.linalg.inv(base_sensor())
        error = score_pose(estimate, truth)
        errors.append({"stamp": t, "estimated_matrix": estimate.tolist(),
                       "truth_matrix": truth.tolist(), **error})
    capture = json.loads((out / "capture.json").read_text())
    manifest = {"schema_version": 1, "frame_id": "map", "T_map_world": np.eye(4).tolist(),
                "pose_source": a.method,
                "input_cloud": "/livox/lidar", "pose_topic": "/aft_mapped_to_init",
                "ground_truth_used_for_scan_fusion": False,
                "ground_truth_used_for_control": getattr(a, "control_source", "pointlio") == "ground_truth",
                "control_source": getattr(a, "control_source", "pointlio"),
                "ideal_3d_map_used": False,
                "route_occupancy_grid_used_for_collision_planning_only": str(a.route_map),
                "capture_mode": "continuous_sensor_odometry_survey",
                "alignment": "single first shared pose after collection; no trajectory fitting",
                "alignment_stamp": stamp, "T_world_camera_init": world_camera.tolist(),
                "T_base_imu": base_sensor().tolist(), "T_base_lidar": base_sensor().tolist(),
                **scan_timing_metadata(params),
                "lidar_range_noise_stddev_m": 0.002, "imu_noise": "Gazebo default; no added noise",
                "voxel_size": a.voxel, "fused_scans": len(node.clouds), "points": len(points),
                "map_sha256": hashlib.sha256(pcd.read_bytes()).hexdigest(),
                "bounds": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                "mapper_sha256": hashlib.sha256(Path(a.mapper).read_bytes()).hexdigest(),
                "mapping_params_sha256": hashlib.sha256(params.read_bytes()).hexdigest(),
                "capture_complete": capture["failure"] is None and capture["completed_waypoints"] == capture["total_waypoints"],
                "mapping_source_revision": a.source_revision,
                "survey_speed_mps": a.speed, "survey_rotation_rad_sec": a.rotation_speed}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out / "mapping_errors.json").write_text(json.dumps(errors) + "\n")
    summary = {"trajectory_samples": len(errors), "clouds": len(node.clouds),
               "capture_complete": manifest["capture_complete"],
               "completed_waypoints": capture["completed_waypoints"],
               "total_waypoints": capture["total_waypoints"],
               "failure": capture["failure"],
               "unmatched_trajectory_samples": len(node.odom) - len(errors),
               "translation_3d_m": stats([r["translation_3d_m"] for r in errors]),
               "orientation_so3_deg": stats([r["orientation_so3_deg"] for r in errors]),
               "final_error": errors[-1] if errors else None,
               "estimated_path_length_m": sum(float(np.linalg.norm(b[:3, 3] - p[:3, 3]))
                                              for (_, p), (_, b) in zip(node.odom, node.odom[1:]))}
    (out / "mapping_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "final_error"}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mapper", required=True)
    a_method = "repository_slam"
    parser.add_argument("--source-revision", default="working tree; executable and source SHA256 recorded")
    parser.add_argument("--route-map", type=Path, default=ROOT / "src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml")
    parser.add_argument("--survey-waypoints", type=Path, default=HERE / "config/survey_left_loop.json")
    parser.add_argument("--params", type=Path, default=ROOT / "src/pb2025_nav_bringup/config/simulation/nav2_params.yaml")
    parser.add_argument("--pointlio-overrides", type=Path)
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--rotation-speed", type=float, default=0.08)
    parser.add_argument("--voxel", type=float, default=0.03)
    parser.add_argument("--clearance", type=float, default=0.55)
    parser.add_argument("--max-wall-sec", type=float, default=1200)
    a = parser.parse_args()
    a.method = a_method
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    params = out / "mapping_params.yaml"
    params.write_text(yaml.safe_dump(mapping_profile(a.params, a.pointlio_overrides)))
    start = np.array([-11.7, 3.1])
    targets = json.loads(a.survey_waypoints.read_text())["waypoints"]
    route, stations = Route(a.route_map, start, a.clearance).make(start, targets)
    (out / "route.json").write_text(json.dumps({"waypoints": route, "stations": stations,
                                               "requested_stations": targets, "clearance_m": a.clearance,
                                               "control_source": "Point-LIO; known spawn gauge"}, indent=2))
    processes = []
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise InterruptedError("survey interrupted; retaining partial capture")
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = Survey(out)
    failure, completed = None, 0
    try:
        for name, command in [
            ("bag", ["ros2", "bag", "record", "--use-sim-time", "-o", str(out / "raw_bag"), "--topics",
                     "/clock", "/livox/lidar", "/livox/imu", "/velodyne_points",
                     "/ground_truth/odometry", "/aft_mapped_to_init", "/cloud_registered", "/cmd_vel", "/map", "/tf", "/tf_static"]),
            ("mapper", ["ros2", "launch", "pb2025_nav_bringup", "rm_navigation_simulation_launch.py",
              "slam:=true", "use_ground_truth_odom:=false", "navigation_mode:=legacy", "use_rviz:=false", "params_file:=" + str(params)])]:
            with (out / f"{name}.log").open("w") as log:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 60
        while (len(node.odom) < 20 or len(node.clouds) < 10) and time.monotonic() < deadline:
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("mapping child exited during initialization; inspect mapper.log")
            node.command()
            node.spin_for(0.1)
        if len(node.odom) < 20 or len(node.clouds) < 10:
            raise RuntimeError("Point-LIO did not initialize")
        subscriptions = node.get_subscriber_names_and_types_by_node("point_lio", "/")
        if any("ground_truth" in topic for topic, _ in subscriptions):
            raise RuntimeError("mapping estimator must not subscribe to ground truth")
        (out / "mapping_subscriptions.json").write_text(json.dumps(subscriptions, indent=2))
        audit = subprocess.run(["/usr/bin/python3", str(HERE / "audit_mapping.py"), "--output",
                                str(out / "mapping_runtime_parameters.json"), "--expected-mapper", a.mapper],
                               capture_output=True, text=True, timeout=35)
        (out / "mapping_audit.log").write_text(audit.stdout + audit.stderr)
        if audit.returncode:
            raise RuntimeError("Point-LIO mapping audit failed")
        # Known initial spawn, independent of the observed GT stream.
        world_base = np.eye(4)
        world_base[:3, 3] = [-11.7, 3.1, 0.08]
        control_alignment = world_base @ base_sensor() @ np.linalg.inv(node.odom[0][1])
        started = time.monotonic()
        for index, target in enumerate(route):
            segment = time.monotonic()
            while True:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError("mapping child process exited")
                if time.monotonic() - node.latest_wall > 6.0:
                    raise RuntimeError("sensor odometry stale")
                if time.monotonic() - started > a.max_wall_sec or time.monotonic() - segment > 30:
                    raise TimeoutError(f"survey timeout at waypoint {index}")
                pose = control_alignment @ node.latest @ np.linalg.inv(base_sensor())
                difference = np.asarray(target) - pose[:2, 3]
                distance = np.linalg.norm(difference)
                if distance < 0.08:
                    break
                world_velocity = difference * min(1.8, a.speed / distance)
                body_velocity = pose[:2, :2].T @ world_velocity
                # Slow continuous rotation exposes the tilted sensor's blind
                # sectors while translation and registration remain continuous.
                if time.monotonic() - node.latest_wall > 0.75:
                    node.command()
                else:
                    node.command(*body_velocity, omega=a.rotation_speed)
                node.spin_for(0.05)
            completed = index + 1
            if index % 25 == 0:
                print(json.dumps({"waypoint": completed, "total": len(route),
                                  "clouds": len(node.clouds), "estimated_xy": pose[:2, 3].tolist()}), flush=True)
        node.command()
        node.spin_for(2)
        save = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(out / "occupancy"),
                               "--ros-args", "-p", "use_sim_time:=true", "-p", "save_map_timeout:=5.0"],
                              capture_output=True, text=True, timeout=12)
        (out / "save_occupancy.log").write_text(save.stdout + save.stderr)
    except Exception as error:
        failure = repr(error)
    finally:
        for _ in range(5):
            node.command()
            node.spin_for(0.05)
        for process in reversed(processes):
            stop(process)
        native = HERE / ".build/mapping_source/PCD/scans.pcd"
        if native.exists():
            shutil.copy2(native, out / "pointlio_native_map.pcd")
        evidence = {"completed_waypoints": completed, "total_waypoints": len(route), "failure": failure,
                    "odom": [{"stamp": t, "matrix": p.tolist()} for t, p in node.odom],
                    "gt": [{"stamp": t, "matrix": p.tolist()} for t, p in node.gt],
                    "clouds": node.clouds, "commands": node.commands}
        (out / "capture.json").write_text(json.dumps(evidence) + "\n")
        if node.clouds and node.odom:
            export(node, out, params, a)
        node.destroy_node()
        rclpy.shutdown()
    if failure:
        raise RuntimeError(failure)


if __name__ == "__main__":
    main()
