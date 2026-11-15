#!/usr/bin/env python3
"""隔离验证人工重定位：固定场景提示、连续点击、错误提示及 TF 预览；GT 仅评分。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import psutil


# 只清理本脚本创建的进程组。
def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


# 按隔离 domain 启动完整仿真并采集后端状态及进程资源。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--domain", type=int, default=183)
    parser.add_argument("--duration", type=float, default=100)
    parser.add_argument("--stamp-mode", choices=["latest", "current"], default="latest")
    parser.add_argument("--navigate", action="store_true")
    parser.add_argument(
        "--retry-after-failure",
        action="store_true",
        help="52 秒时重新发送正确提示，验证失败标记删除",
    )
    parser.add_argument(
        "--continuity-request-at",
        type=float,
        default=0.0,
        help="指定墙钟秒时无提示请求恢复，验证连续性约束",
    )
    parser.add_argument("--goal-time", type=float, default=26.5)
    parser.add_argument("--initial-hint", action="store_true")
    parser.add_argument(
        "--skip-recovery",
        action="store_true",
        help="startup smoke: do not inject a manual recovery request",
    )
    parser.add_argument("--reset", action="store_true")
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["GZ_PARTITION"] = f"rm27_recovery_{os.getpid()}"
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rm27_localization_interfaces.msg import LocalizationStatus
    from rm27_localization_interfaces.srv import ResetFrontend
    from nav_msgs.msg import Odometry
    from visualization_msgs.msg import MarkerArray
    from tf2_ros import Buffer, TransformListener
    from rclpy.time import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from std_srvs.srv import Trigger
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose

    rclpy.init()
    from rclpy.parameter import Parameter

    node = rclpy.create_node(
        "manual_recovery_validation",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    states, estimates, truth, resources, diagnostics = [], [], [], [], []
    markers, displayed = [], []
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    node.create_subscription(
        MarkerArray,
        "/rm27_backend/recovery_markers",
        lambda m: markers.append(
            dict(
                wall=time.monotonic() - start,
                text=[x.text for x in m.markers if x.text],
                actions=[dict(ns=x.ns, id=x.id, action=x.action) for x in m.markers],
            )
        ),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    start = time.monotonic()
    latest_stamp = None
    hint_sent = reset_sent = False
    click_index = 0
    clicks = []
    hint_count = 0

    # 仅作为评分记录，不将真值送回任何估计入口。
    def pose_record(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return [
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            p.x,
            p.y,
            p.z,
            q.x,
            q.y,
            q.z,
            q.w,
        ]

    # 保留每次状态快照及源帧时间，用于分析冻结和恢复耗时。
    def status(msg):
        nonlocal latest_stamp
        latest_stamp = msg.pose_stamp
        states.append(
            dict(
                wall=time.monotonic() - start,
                state=msg.state,
                trusted=msg.global_trusted,
                reason=msg.reason,
                tf=msg.tf_published,
                generation=msg.generation,
                epoch=msg.epoch,
            )
        )

    node.create_subscription(
        LocalizationStatus,
        "/rm27_backend/localization_status",
        status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        Odometry,
        "/rm27_backend/odometry",
        lambda m: estimates.append(pose_record(m)),
        10,
    )
    node.create_subscription(
        Odometry, "/ground_truth/odometry", lambda m: truth.append(pose_record(m)), 10
    )
    node.create_subscription(
        DiagnosticArray,
        "/rm27_backend/diagnostics",
        lambda m: diagnostics.append(
            {v.key: v.value for s in m.status for v in s.values}
        ),
        10,
    )
    navigation = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    goals = []
    invalid_reset_future = None
    invalid_reset_result = None
    invalid_reset_client = node.create_client(ResetFrontend, "/point_lio/reset")
    goal_future = result_future = None
    hint_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    reset = node.create_client(Trigger, "/rm27_backend/reset_frontend")
    recover = node.create_client(Trigger, "/rm27_backend/relocalize")
    launch = subprocess.Popen(
        [
            "ros2",
            "launch",
            "rm_27_stimulation",
            "sim_with_nav.launch.py",
            "world:=RMUC2027",
            "nav_world:=auto",
            "use_ground_truth_odom:=false",
            "navigation_mode:=minco",
            "gui:=false",
            "use_rviz:=false",
            "gimbal_panel:=false",
        ],
        stdout=(out / "launch.log").open("w"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    bag = None
    proc = None
    triggered = False
    last_sample = 0
    discovered_nodes = set()
    tf_publishers = set()
    try:
        bag = subprocess.Popen(
            [
                "ros2",
                "bag",
                "record",
                "--use-sim-time",
                "-o",
                str(out / "bag"),
                "--topics",
                "/point_lio/backend_frame",
                "/tf",
                "/tf_static",
                "/clock",
                "/ground_truth/odometry",
                "/rm27_backend/odometry",
                "/rm27_backend/localization_status",
                "/plan",
                "/cmd_vel",
                "/cmd_vel_nav2_result",
                "/initialpose",
                "/rm27_backend/recovery_markers",
            ],
            stdout=(out / "bag.log").open("w"),
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        while time.monotonic() - start < args.duration and launch.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            try:
                transform = buffer.lookup_transform("map", "base_footprint", Time())
                t = transform.transform.translation
                q = transform.transform.rotation
                displayed.append(
                    dict(
                        wall=elapsed,
                        stamp=transform.header.stamp.sec
                        + transform.header.stamp.nanosec * 1e-9,
                        x=t.x,
                        y=t.y,
                        z=t.z,
                        q=[q.x, q.y, q.z, q.w],
                    )
                )
            except Exception:
                pass
            if elapsed - last_sample > 0.5:
                last_sample = elapsed
                discovered_nodes.update(node.get_node_names())
                tf_publishers.update(
                    info.node_name for info in node.get_publishers_info_by_topic("/tf")
                )
                try:
                    if proc is None:
                        for child in psutil.Process(launch.pid).children(
                            recursive=True
                        ):
                            if child.name() == "rm27_backend_node":
                                proc = child
                                proc.cpu_percent()
                                break
                    if proc is not None:
                        memory = proc.memory_full_info()
                        resources.append(
                            dict(
                                wall=elapsed,
                                cpu=proc.cpu_percent(),
                                rss=memory.rss,
                                pss=memory.pss,
                            )
                        )
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            if (
                invalid_reset_future is None
                and elapsed > 25
                and invalid_reset_client.service_is_ready()
            ):
                invalid = ResetFrontend.Request()
                invalid.instance_id = "deliberately-stale-instance"
                invalid.epoch = 0
                invalid.request_id = 1
                invalid_reset_future = invalid_reset_client.call_async(invalid)
            if (
                invalid_reset_future is not None
                and invalid_reset_future.done()
                and invalid_reset_result is None
            ):
                invalid_reset_result = dict(
                    accepted=invalid_reset_future.result().accepted,
                    reason=invalid_reset_future.result().reason,
                )
            schedule = [
                (15.0, -11.4, 2.9, 0.15),
                (27.0, -5.76, 3.51, 0.645),
                (27.2, -11.4, 2.9, 0.15),
                (42.0, -5.76, 3.51, 0.645),
            ]
            if args.retry_after_failure:
                schedule.append((52.0, -11.4, 2.9, 0.15))
            if (
                args.continuity_request_at > 0
                and not triggered
                and elapsed > args.continuity_request_at
                and recover.service_is_ready()
            ):
                recover.call_async(Trigger.Request())
                triggered = True
            if (
                click_index < len(schedule)
                and elapsed >= schedule[click_index][0]
                and latest_stamp is not None
            ):
                import math

                _, x, y, yaw = schedule[click_index]
                hint = PoseWithCovarianceStamped()
                hint.header.frame_id = "map"
                hint.header.stamp = (
                    node.get_clock().now().to_msg()
                    if args.stamp_mode == "current"
                    else latest_stamp
                )
                hint.pose.pose.position.x = x
                hint.pose.pose.position.y = y
                hint.pose.pose.orientation.z = math.sin(yaw / 2)
                hint.pose.pose.orientation.w = math.cos(yaw / 2)
                hint.pose.covariance[0] = hint.pose.covariance[7] = 0.25
                hint.pose.covariance[35] = 0.06853891909122467
                hint_pub.publish(hint)
                clicks.append(
                    dict(
                        wall=elapsed,
                        position=[x, y],
                        yaw=yaw,
                        stamp=hint.header.stamp.sec + hint.header.stamp.nanosec * 1e-9,
                    )
                )
                click_index += 1
            if (
                args.reset
                and not reset_sent
                and elapsed > args.duration * 0.8
                and reset.service_is_ready()
            ):
                reset.call_async(Trigger.Request())
                reset_sent = True
            if (
                args.navigate
                and not goals
                and elapsed > args.goal_time
                and states
                and states[-1]["trusted"]
                and navigation.server_is_ready()
            ):
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = "map"
                goal.pose.pose.position.x = 5.163
                goal.pose.pose.position.y = 4.891
                goal.pose.pose.orientation.w = 1.0
                goal_future = navigation.send_goal_async(goal)
                goals.append({"wall": elapsed, "target": [5.163, 4.891]})
            if goal_future is not None and goal_future.done() and result_future is None:
                handle = goal_future.result()
                goals[-1]["accepted"] = handle.accepted
                if handle.accepted:
                    result_future = handle.get_result_async()
                else:
                    goal_future = None
            if result_future is not None and result_future.done():
                goals[-1]["status"] = result_future.result().status
                goals[-1]["finished_wall"] = elapsed
                goal_future = result_future = None
            if (
                not args.skip_recovery
                and elapsed > args.duration * 0.55
                and not triggered
                and states
                and states[-1]["trusted"]
                and recover.service_is_ready()
            ):
                recover.call_async(Trigger.Request())
                triggered = True
    finally:
        if bag:
            stop(bag)
        stop(launch)
        node.destroy_node()
        rclpy.shutdown()
        report = dict(
            stamp_mode=args.stamp_mode,
            markers=markers,
            displayed=displayed,
            clicks=clicks,
            nodes=sorted(discovered_nodes),
            tf_publishers=sorted(tf_publishers),
            domain=args.domain,
            duration=time.monotonic() - start,
            manual_recovery_requested=triggered,
            initial_hint_sent=hint_sent,
            initial_hint_count=hint_count,
            reset_requested=reset_sent,
            invalid_reset_result=invalid_reset_result,
            goals=goals,
            states=states,
            estimates=estimates,
            truth=truth,
            resources=resources,
            diagnostics=diagnostics,
        )
        (out / "results.json").write_text(json.dumps(report, indent=2))
        print(
            json.dumps(
                dict(
                    states=len(states),
                    estimates=len(estimates),
                    trusted_samples=sum(s["trusted"] for s in states),
                    last_state=states[-1] if states else None,
                ),
                indent=2,
            )
        )


if __name__ == "__main__":
    main()
