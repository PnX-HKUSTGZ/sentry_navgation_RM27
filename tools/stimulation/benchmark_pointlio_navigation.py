#!/usr/bin/env python3
"""Measure Point-LIO simulation routes using independent Gazebo odometry.

Run after starting the simulation, with the same ROS_DOMAIN_ID. This sends
NavigateToPose goals. Stops are physical speed < 0.05 m/s for >= 0.4 simulated
seconds after movement has begun; terminal parking after success is excluded.
GICP counters cover messages received after this monitor starts.
"""
import fcntl

_run_lock = open("/tmp/pnx_navigation_benchmark.guard", "w")
fcntl.flock(_run_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
import argparse, collections, json, math, re, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
import rclpy.signals
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool
from lifecycle_msgs.srv import GetState

p = argparse.ArgumentParser()
p.add_argument("--output", required=True)
p.add_argument(
    "--routes", default=str(Path(__file__).with_name("pointlio_benchmark_routes.json"))
)
p.add_argument("--tag", default="test")
p.add_argument("--gimbal", choices=["folded", "upright", "spin"], default="folded")
args = p.parse_args()
rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
node = rclpy.create_node(
    "pnx_route_benchmark", parameter_overrides=[Parameter("use_sim_time", value=True)]
)
latest = {}
samples = []
active = False
cmd = (0.0, 0.0, 0.0)
events = collections.Counter()
all_events = collections.Counter()
localized = False
age = []
last_truth = None
gimbal_state = {}
odom_intervals = []
last_odom_rx = None
input_last_rx = {}
input_intervals = collections.defaultdict(list)
subs = []
gicp = {"attempts": 0, "accepted": 0, "rejected": 0}
max_latest_age = 0.0
start = time.monotonic()


def odom(m):
    global latest, last_odom_rx
    rx = time.monotonic()
    if active and last_odom_rx is not None:
        odom_intervals.append(rx - last_odom_rx)
    last_odom_rx = rx
    latest["odom"] = m
    if active:
        age.append(
            node.get_clock().now().nanoseconds * 1e-9
            - m.header.stamp.sec
            - m.header.stamp.nanosec * 1e-9
        )


def truth(m):
    global last_truth
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    pos = m.pose.pose.position
    speed = 0.0
    if last_truth and t > last_truth[0]:
        speed = math.hypot(pos.x - last_truth[1], pos.y - last_truth[2]) / (
            t - last_truth[0]
        )
    last_truth = (t, pos.x, pos.y)
    latest["truth"] = m
    if active:
        q = m.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        samples.append([t, pos.x, pos.y, pos.z, speed,
                        math.hypot(cmd[0], cmd[1]), roll, pitch])


def command(m):
    global cmd
    cmd = (m.linear.x, m.linear.y, m.angular.z)


def gimbal(m):
    global gimbal_state
    gimbal_state = json.loads(m.data)


def input_odometry(topic):
    def receive(_msg):
        now = time.monotonic()
        if active and topic in input_last_rx:
            input_intervals[topic].append(now - input_last_rx[topic])
        input_last_rx[topic] = now
    return receive


def logging(m):
    global localized
    text = re.sub(r"\x1b\[[0-9;]*m", "", m.msg)
    if "relocalization_manager" in m.name:
        if "LOCALIZING" in text and ("state:" in text or " -> LOCALIZING " in text):
            localized = False
        if "LOCALIZED" in text and ("state:" in text or " -> LOCALIZED " in text):
            localized = True
        if "Verifying GICP source:" in text:
            gicp["attempts"] += 1
        if "GICP converge successfully" in text:
            gicp["accepted"] += 1
        if "GICP did not converge" in text or "GICP converged but rejected" in text:
            gicp["rejected"] += 1
    reason = None
    if "fail-closed:" in text:
        reason = text.split("fail-closed:", 1)[1].strip()
    elif "Rejecting velocity command" in text:
        reason = "VELOCITY_ODOM_STALE"
    elif "Trajectory safety rejected:" in text:
        found = re.search(r"reason=(\w+)", text)
        reason = "SAFETY_" + found.group(1) if found else "SAFETY_OTHER"
    elif "Failed to make progress" in text:
        reason = "NO_PROGRESS"
    elif "GICP converged but rejected" in text:
        reason = "GICP_REJECTED"
    elif "ground support is fail-closed" in text:
        reason = "GROUND_TRANSFORM_INVALID"
    elif "MINCO optimizer failed:" in text:
        reason = "OPTIMIZER_FAILED"
    if reason:
        all_events[reason] += 1
        if active:
            events[reason] += 1


for typ, topic, cb in [
    (Odometry, "/odometry", odom),
    (Odometry, "/ground_truth/odometry", truth),
    (Twist, "/cmd_vel", command),
    (Log, "/rosout", logging),
    (String, "/sim/gimbal/state", gimbal),
    (Odometry, "/aft_mapped_to_init", input_odometry("point_lio")),
    (Odometry, "/lidar_odometry", input_odometry("loam_interface")),
]:
    subs.append(node.create_subscription(typ, topic, cb, qos_profile_sensor_data))
client = ActionClient(node, NavigateToPose, "navigate_to_pose")
while time.monotonic() - start < 65 and not (
    localized and "truth" in latest and client.server_is_ready()
):
    rclpy.spin_once(node, timeout_sec=0.03)
report = {
    "tag": args.tag,
    "gimbal_mode": args.gimbal,
    "wait_localized_wall_s": time.monotonic() - start,
    "localized_at_start": localized,
    "gicp_start": dict(gicp),
    "routes": [],
}
print(
    json.dumps(
        {"ready": localized, "wait_s": report["wait_localized_wall_s"], "gicp": gicp}
    ),
    flush=True,
)


def persist():
    report["gicp_total"] = dict(gicp)
    report["events_total"] = dict(all_events)
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n")


if not localized or "truth" not in latest or not client.server_is_ready():
    report["failure"] = (
        "localization, simulator odometry, or navigation action not ready"
    )
    persist()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

request = None
future = None


def set_gimbal(service, enabled):
    service_client = node.create_client(SetBool, service)
    if not service_client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"Gimbal service unavailable: {service}")
    response = service_client.call_async(SetBool.Request(data=enabled))
    deadline = time.monotonic() + 5.0
    while not response.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
    if not response.done() or not response.result().success:
        raise RuntimeError(f"Gimbal command failed: {service}")
    node.destroy_client(service_client)


try:
    # An action server can be discovered while its lifecycle node is still
    # activating. Do not mistake a rejected first goal for a route timeout.
    lifecycle = node.create_client(GetState, "/bt_navigator/get_state")
    if not lifecycle.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("Navigator lifecycle service unavailable")
    deadline = time.monotonic() + 15.0
    navigator_active = False
    while time.monotonic() < deadline and not navigator_active:
        state_future = lifecycle.call_async(GetState.Request())
        while not state_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
        navigator_active = state_future.done() and state_future.result().current_state.id == 3
        if not navigator_active:
            rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_client(lifecycle)
    report["navigator_active_at_start"] = navigator_active
    if not navigator_active:
        raise RuntimeError("Navigator did not activate")
    set_gimbal("/sim/gimbal/set_upright", args.gimbal != "folded")
    deadline = time.monotonic() + 20.0
    wanted = "Folded" if args.gimbal == "folded" else "Upright"
    while gimbal_state.get("status") != wanted and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
    if gimbal_state.get("status") != wanted:
        raise RuntimeError(f"Gimbal did not reach {wanted}: {gimbal_state}")
    if args.gimbal == "spin":
        set_gimbal("/sim/gimbal/set_spin", True)
    report["gimbal_at_start"] = dict(gimbal_state)
    for route in json.loads(Path(args.routes).read_text()):
        if not client.server_is_ready() or "truth" not in latest:
            break
        samples = []
        age = []
        odom_intervals = []
        input_intervals.clear()
        events.clear()
        active = True
        max_latest_age = 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = route["x"]
        goal.pose.pose.position.y = route["y"]
        goal.pose.pose.orientation.w = 1.0
        request = client.send_goal_async(goal)
        future = None
        begin = time.monotonic()
        while time.monotonic() - begin < route["timeout"]:
            rclpy.spin_once(node, timeout_sec=0.025)
            if "odom" in latest:
                m = latest["odom"]
                a = (
                    node.get_clock().now().nanoseconds * 1e-9
                    - m.header.stamp.sec
                    - m.header.stamp.nanosec * 1e-9
                )
                if a >= 0:
                    max_latest_age = max(max_latest_age, a)
            if future is None and request.done():
                if not request.result().accepted:
                    break
                future = request.result().get_result_async()
            if future and future.done():
                break
        active = False
        status = future.result().status if future and future.done() else "timeout"
        if request.done() and not request.result().accepted:
            status = "rejected"
        if future and not future.done():
            cancel = request.result().cancel_goal_async()
            until = time.monotonic() + 3
            while not cancel.done() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.03)
        stop_intervals = []
        low = None
        has_moved = False
        if samples:
            for row in samples:
                if row[4] > 0.1:
                    has_moved = True
                if has_moved and row[4] < 0.05:
                    if low is None:
                        low = []
                    low.append(row)
                elif low is not None:
                    if low[-1][0] - low[0][0] >= 0.4:
                        stop_intervals.append(low)
                    low = None
            if low is not None and status != 4 and low[-1][0] - low[0][0] >= 0.4:
                stop_intervals.append(low)
        stops = [
            {
                "duration_s": round(a[-1][0] - a[0][0], 3),
                "xy": a[0][1:3],
                "mean_command": float(np.mean([r[5] for r in a])),
            }
            for a in stop_intervals
        ]
        distance = sum(
            math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:])
        )
        # Time-weighted planar ground-truth speed, independently of LIO and
        # command values. Flat samples require both ends below 0.04 rad tilt.
        flat_distance = flat_time = moving_distance = moving_time = 0.0
        flat_moving_distance = flat_moving_time = 0.0
        for a, b in zip(samples, samples[1:]):
            dt = b[0] - a[0]
            if dt <= 0:
                continue
            ds = math.hypot(b[1] - a[1], b[2] - a[2])
            moving = ds / dt > 0.05
            if moving:
                moving_distance += ds
                moving_time += dt
            if max(abs(a[6]), abs(a[7]), abs(b[6]), abs(b[7])) < 0.04:
                flat_distance += ds
                flat_time += dt
                if moving:
                    flat_moving_distance += ds
                    flat_moving_time += dt
        result = {
            "name": route["name"],
            "goal": [route["x"], route["y"]],
            "status": status,
            "wall_s": time.monotonic() - begin,
            "sim_s": samples[-1][0] - samples[0][0] if samples else None,
            "path_length_m": distance,
            "mean_speed_sim_mps": distance / (samples[-1][0] - samples[0][0])
                if len(samples) > 1 and samples[-1][0] > samples[0][0] else None,
            "moving_mean_speed_sim_mps": moving_distance / moving_time if moving_time else None,
            "flat_mean_speed_sim_mps": flat_distance / flat_time if flat_time else None,
            "flat_moving_mean_speed_sim_mps": flat_moving_distance / flat_moving_time
                if flat_moving_time else None,
            "flat_sample_time_sim_s": flat_time,
            "physical_speed_p50_p95_max_mps": np.percentile(
                [row[4] for row in samples], [50, 95, 100]).tolist() if samples else None,
            "stop_count": len(stops),
            "stop_time_s": sum(s["duration_s"] for s in stops),
            "stalls_with_command": sum(s["mean_command"] > 0.15 for s in stops),
            "stops": stops,
            "events": dict(events),
            "max_latest_odom_age_s": max_latest_age,
            "odom_receive_interval_p50_p95_max_wall_s": (
                np.percentile(odom_intervals, [50, 95, 100]).tolist()
                if odom_intervals else None
            ),
            "gimbal_at_end": dict(gimbal_state),
            "input_receive_interval_p50_p95_max_wall_s": {
                topic: np.percentile(values, [50, 95, 100]).tolist()
                for topic, values in input_intervals.items() if values
            },
            "arrival_age_p50_p95_max": (
                np.percentile(age, [50, 95, 100]).tolist() if age else None
            ),
            "last_xyz": samples[-1][1:4] if samples else None,
            "gicp": dict(gicp),
        }
        report["routes"].append(result)
        persist()
        print(json.dumps(result), flush=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
except Exception as exc:
    report["failure"] = str(exc)
    raise
finally:
    if (
        rclpy.ok()
        and request is not None
        and request.done()
        and request.result().accepted
        and future is not None
        and not future.done()
    ):
        cancellation = request.result().cancel_goal_async()
        cancel_deadline = time.monotonic() + 2.0
        while (
            rclpy.ok()
            and not cancellation.done()
            and time.monotonic() < cancel_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
    persist()
    node.destroy_node()
    rclpy.shutdown()
expected_count = len(json.loads(Path(args.routes).read_text()))
raise SystemExit(
    0
    if report["localized_at_start"]
    and len(report["routes"]) == expected_count
    and all(r["status"] == 4 for r in report["routes"])
    else 1
)
