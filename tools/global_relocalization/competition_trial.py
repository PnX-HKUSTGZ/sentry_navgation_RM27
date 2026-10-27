#!/usr/bin/env python3
"""One isolated real-simulation competition trial, with observation-only GT scoring."""
import argparse
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import psutil
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from global_relocalization_interfaces.action import Relocalize
from evaluation import matrix, interpolate, score_pose, meets_pose_criteria, POSE_CRITERIA
from reset_pose import reset_pose


class Observe(Node):
    def __init__(self):
        super().__init__("competition_trial_observer")
        self.gt, self.imu, self.states, self.commands = [], [], [], []
        self.gt_error = None
        self.create_subscription(
            Odometry,
            "/ground_truth/odometry",
            self.truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/livox/imu",
            lambda m: self.imu.append(
                {
                    "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    "omega": math.sqrt(
                        m.angular_velocity.x**2
                        + m.angular_velocity.y**2
                        + m.angular_velocity.z**2
                    ),
                }
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/global_relocalization/status", self.status, 100
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            lambda m: self.commands.append(
                {
                    "wall": time.monotonic(),
                    "omega": m.angular.z,
                    "linear": [m.linear.x, m.linear.y, m.linear.z],
                }
            ),
            100,
        )
        self.action = ActionClient(self, Relocalize, "/global_relocalization/run")
        self.trigger = self.create_client(Trigger, "/global_relocalization/start_match")

    def status(self, m):
        self.states.append({"received_wall": time.monotonic(), **json.loads(m.data)})

    def truth(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.gt and stamp <= self.gt[-1][0]:
            self.gt_error = "GT timestamps not strictly increasing"
        self.gt.append((stamp, matrix(message.pose.pose)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("node", "params", "map", "manifest", "output", "trial"):
        p.add_argument("--" + name, required=True)
    p.add_argument(
        "--trials",
        default=str(Path(__file__).parent / "config/trials_left_physical_120.json"),
    )
    p.add_argument("--early", action="store_true")
    p.add_argument("--evaluation-profile", choices=POSE_CRITERIA, default="ideal",
                   help="GT scoring only: ideal 0.10m/2deg; sensor_map 0.15m/3deg; both require <=1s")
    p.add_argument("--record-bag", action="store_true",
                   help="retain independent raw sensors, GT, status, pose and control")
    a = p.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    if (
        hashlib.sha256(Path(a.map).read_bytes()).hexdigest()
        != json.loads(Path(a.manifest).read_text())["map_sha256"]
    ):
        raise ValueError("frozen map hash mismatch")
    trial = next(
        t
        for t in json.loads(Path(a.trials).read_text())["trials"]
        if t["trial_id"] == a.trial
    )
    reset_pose(trial["requested_pose"], log_path=out / "reset_attempts.json")
    bag_process = None
    if a.record_bag:
        with (out / "bag.log").open("w") as bag_log:
            bag_process = subprocess.Popen(
                ["ros2", "bag", "record", "--use-sim-time", "-o", str(out / "raw_bag"), "--topics",
                 "/clock", "/livox/lidar", "/livox/imu", "/ground_truth/odometry",
                 "/cmd_vel", "/global_relocalization/status", "/global_relocalization/pose"],
                stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
    command = [
        a.node,
        "--ros-args",
        "--params-file",
        a.params,
        "-p",
        "map_path:=" + str(Path(a.map).resolve()),
        "-p",
        "manifest_path:=" + str(Path(a.manifest).resolve()),
        "-p",
        "visualization_enabled:=false",
    ]
    log = (out / "node.log").open("w")
    process = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )
    log.close()
    rclpy.init()
    node = Observe()
    cost = psutil.Process(process.pid)
    cpu0 = cost.cpu_times()
    peak = 0
    started = time.monotonic()

    def spin():
        nonlocal peak
        rclpy.spin_once(node, timeout_sec=0.02)
        try:
            peak = max(peak, cost.memory_info().rss)
        except psutil.NoSuchProcess:
            pass

    def wait(future, timeout=150):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("node exited; inspect node.log")
            spin()
        if not future.done():
            raise TimeoutError("future deadline exceeded")
        return future.result()

    paths = [
        Path(a.node),
        Path(a.node).parents[1] / "libglobal_relocalization_core.so",
        Path(a.params),
        Path(a.map),
        Path(a.manifest),
        Path(a.trials),
        Path(__file__),
        Path(__file__).parent / "evaluation.py",
        Path(__file__).parent / "reset_pose.py",
        Path(__file__).parent / "audit_runtime_parameters.py",
    ]
    provenance = {
        str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in paths
    }
    result = {
        "sha256": provenance,
        **trial,
        "early_trigger_test": a.early,
        "evaluation_profile": a.evaluation_profile,
        "pose_limits": dict(zip(("translation_3d_m", "orientation_so3_deg"), POSE_CRITERIA[a.evaluation_profile])),
        "trigger_limit_sec": 1.0,
        "command": command,
        "environment": {
            k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "GZ_PARTITION")
        },
        "gt_used_by_estimator": False,
    }
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def terminate_trial(signum, frame):
        raise InterruptedError("trial terminated; retaining partial evidence")

    signal.signal(signal.SIGTERM, terminate_trial)
    try:
        if not node.action.wait_for_server(
            timeout_sec=15
        ) or not node.trigger.wait_for_service(timeout_sec=5):
            raise RuntimeError("server unavailable")
        mappings = Path("/proc/" + str(process.pid) + "/maps").read_text()
        (out / "node_maps.txt").write_text(mappings)
        expected_library = (
            Path(a.node).parents[1] / "libglobal_relocalization_core.so"
        ).resolve()
        if str(expected_library) not in mappings:
            raise RuntimeError("node loaded unexpected core library")
        audit_command = [
            "/usr/bin/python3",
            str(Path(__file__).parent / "audit_runtime_parameters.py"),
            "--params",
            a.params,
            "--output",
            str(out / "runtime_parameters.json"),
            "--timeout",
            "10",
        ]
        audit = subprocess.run(
            audit_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=15,
        )
        (out / "runtime_parameters.log").write_text(audit.stdout)
        if audit.returncode:
            raise RuntimeError("runtime parameter audit failed")
        until = time.monotonic() + 2
        while time.monotonic() < until:
            spin()
        action_started = time.monotonic()
        cpu0 = cost.cpu_times()
        peak = cost.memory_info().rss
        handle = wait(node.action.send_goal_async(Relocalize.Goal()), 10)
        if not handle.accepted:
            raise RuntimeError("goal rejected")
        future = handle.get_result_async()
        triggered = False
        trigger_wall = None
        trigger_response = None
        armed_wall = None
        deadline = time.monotonic() + 140
        while not future.done() and time.monotonic() < deadline:
            spin()
            if armed_wall is None and any(
                s.get("stage") == "ARMED" for s in node.states
            ):
                armed_wall = time.monotonic()
            if not triggered and (
                a.early
                or (armed_wall is not None and time.monotonic() - armed_wall >= 0.3)
            ):
                triggered = True
                trigger_wall = time.monotonic()
                response = wait(node.trigger.call_async(Trigger.Request()), 5)
                trigger_response = {
                    "success": response.success,
                    "message": response.message,
                }
        response = wait(future, 2)
        pose = response.result.pose
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        result["estimated_pose_matrix"] = (
            matrix(pose.pose).tolist() if response.result.success else None
        )
        result.update(
            action={
                "status": response.status,
                "success": response.result.success,
                "reason": response.result.reason,
                "pose_stamp_sec": stamp,
                "fitness": response.result.fitness,
                "inlier_ratio": response.result.inlier_ratio,
            },
            trigger=trigger_response,
            trigger_wall=trigger_wall,
            action_finished_wall=time.monotonic(),
        )
        if response.result.success:
            # Obtain a future GT bracket; this is observation only, outside latency measurement.
            deadline = time.monotonic() + 1
            while node.gt and node.gt[-1][0] < stamp and time.monotonic() < deadline:
                spin()
            truth = interpolate(node.gt, stamp, 0.25) if node.gt_error is None else None
            if truth is None:
                result["score"] = {"valid": False, "reason": node.gt_error or "missing GT bracket"}
            else:
                estimate = matrix(pose.pose)
                map_world = np.array(
                    json.loads(Path(a.manifest).read_text())["T_map_world"]
                )
                truth = map_world @ truth
                result["ground_truth_pose_matrix"] = truth.tolist()
                result["ground_truth_interpolation"] = {
                    "pose_stamp_sec": stamp, "max_bracket_gap_sec": 0.25,
                    "method": "linear translation and quaternion SLERP; no extrapolation"}
                for name, value in (("estimate", estimate), ("ground_truth", truth)):
                    result[name + "_pose"] = {
                        "xyz_m": value[:3, 3].tolist(),
                        "quaternion_xyzw": Rotation.from_matrix(value[:3, :3]).as_quat().tolist(),
                        "rpy_deg": Rotation.from_matrix(value[:3, :3]).as_euler("xyz", degrees=True).tolist()}
                result["score"] = score_pose(estimate, truth)
                result["evaluation_profile"] = a.evaluation_profile
                result["pose_correct"] = meets_pose_criteria(result["score"], a.evaluation_profile)
        terminal_wall = time.monotonic()
        until = terminal_wall + 1
        while time.monotonic() < until:
            spin()
        result["commands_after_terminal_grace"] = [
            c for c in node.commands if c["wall"] > terminal_wall + 0.2
        ]
        result["terminal_status"] = node.states[-1] if node.states else None
        spin_stamps = [
            s.get("status_ros_ns", 0) * 1e-9
            for s in node.states
            if s.get("stage") == "SPINNING"
        ]
        trigger_ros = (
            (result["terminal_status"] or {})
            .get("competition", {})
            .get("trigger_ros_sec")
        )
        if spin_stamps:
            begin = min(spin_stamps)
            end = trigger_ros if trigger_ros and trigger_ros > 0 else float("inf")
            rows = [(stamp, pose) for stamp, pose in node.gt if begin <= stamp <= end]
            accumulated = 0.0
            full_turn = None
            rates = []
            for (ta, pa), (tb, pb) in zip(rows, rows[1:]):
                angle = float(
                    Rotation.from_matrix(pa[:3, :3].T @ pb[:3, :3]).magnitude()
                )
                if tb > ta:
                    rates.append(angle / (tb - ta))
                if (
                    full_turn is None
                    and accumulated < 2 * math.pi <= accumulated + angle
                ):
                    full_turn = (
                        ta
                        + (2 * math.pi - accumulated) / angle * (tb - ta)
                        - rows[0][0]
                    )
                accumulated += angle
            omegas = [r["omega"] for r in node.imu if begin <= r["stamp"] <= end]
            result["physical_rotation"] = {
                "gt_accumulated_so3_rad": accumulated,
                "measured_first_full_turn_sec": full_turn,
                "gt_pose_rate_median_rad_sec": (
                    float(np.median(rates)) if rates else None
                ),
                "imu_omega_median_rad_sec": (
                    float(np.median(omegas)) if omegas else None
                ),
                "gt_max_translation_m": max(
                    (
                        float(np.linalg.norm(row[1][:3, 3] - rows[0][1][:3, 3]))
                        for row in rows
                    ),
                    default=None,
                ),
            }
        result["wall_sec"] = time.monotonic() - started
        cpu1 = cost.cpu_times()
        resource_wall = time.monotonic() - action_started
        result["algorithm_resource"] = {
            "window": "goal submission through terminal plus one second passive observation",
            "wall_sec": resource_wall,
            "cpu_core_seconds": cpu1.user + cpu1.system - cpu0.user - cpu0.system,
            "cpu_percent": 100
            * (cpu1.user + cpu1.system - cpu0.user - cpu0.system)
            / resource_wall,
            "peak_rss_bytes": peak,
        }
    except Exception as e:
        result["exception"] = repr(e)
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        result["status_events"] = node.states
        result["gt_timestamps_valid"] = node.gt_error is None
        result["commands"] = node.commands
        result["imu"] = node.imu
        result["gt"] = [{"stamp": t, "matrix": m.tolist()} for t, m in node.gt]
        (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        if bag_process is not None:
            if bag_process.poll() is None:
                os.killpg(bag_process.pid, signal.SIGINT)
            try:
                bag_process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(bag_process.pid, signal.SIGKILL)
                bag_process.wait()
            result["bag_returncode"] = bag_process.returncode
            (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(
        json.dumps(
            {
                k: v
                for k, v in result.items()
                if k not in ("gt", "imu", "commands", "status_events")
            },
            indent=2,
        )
    )
    latency = (
        (result.get("terminal_status") or {})
        .get("competition", {})
        .get("trigger_to_pose_sec", -1)
    )
    passed = (
        result.get("action", {}).get("success")
        and meets_pose_criteria(result.get("score", {}), a.evaluation_profile)
        and 0 <= latency <= 1
    )
    early_passed = (
        a.early
        and result.get("trigger", {}).get("success") is False
        and result.get("action", {}).get("status") == 6
        and bool(result.get("commands"))
        and all(
            c["omega"] == 0 and all(x == 0 for x in c["linear"])
            for c in result.get("commands", [])
        )
    )
    return 0 if passed or early_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
