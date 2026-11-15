#!/usr/bin/env python3
"""隔离仿真验证：不发送运动命令，记录未知初值定位、人工恢复、资源与真值。"""
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
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=183)
    parser.add_argument('--duration', type=float, default=100)
    parser.add_argument('--navigate', action='store_true')
    parser.add_argument('--initial-hint', action='store_true')
    parser.add_argument('--skip-recovery', action='store_true', help='startup smoke: do not inject a manual recovery request')
    parser.add_argument('--reset', action='store_true')
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['GZ_PARTITION'] = f'rm27_recovery_{os.getpid()}'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rm27_localization_interfaces.msg import LocalizationStatus
    from rm27_localization_interfaces.srv import ResetFrontend
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from std_srvs.srv import Trigger
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    rclpy.init()
    node = rclpy.create_node('recovery_validation')
    states, estimates, truth, resources, diagnostics = [], [], [], [], []
    start = time.monotonic()
    latest_stamp = None
    hint_sent = reset_sent = False
    hint_count = 0
    # 仅作为评分记录，不将真值送回任何估计入口。
    def pose_record(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return [msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    # 保留每次状态快照及源帧时间，用于分析冻结和恢复耗时。
    def status(msg):
        nonlocal latest_stamp
        latest_stamp = msg.pose_stamp
        states.append(dict(wall=time.monotonic()-start, state=msg.state,
                           trusted=msg.global_trusted, reason=msg.reason, tf=msg.tf_published,
                           generation=msg.generation, epoch=msg.epoch))
    node.create_subscription(LocalizationStatus, '/rm27_backend/localization_status', status,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry, '/rm27_backend/odometry', lambda m: estimates.append(pose_record(m)), 10)
    node.create_subscription(Odometry, '/ground_truth/odometry', lambda m: truth.append(pose_record(m)), 10)
    node.create_subscription(DiagnosticArray, '/rm27_backend/diagnostics',
                             lambda m: diagnostics.append({v.key: v.value for s in m.status for v in s.values}), 10)
    navigation = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    goals = []
    invalid_reset_future = None
    invalid_reset_result = None
    invalid_reset_client = node.create_client(ResetFrontend, "/point_lio/reset")
    goal_future = result_future = None
    hint_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    reset = node.create_client(Trigger, '/rm27_backend/reset_frontend')
    recover = node.create_client(Trigger, '/rm27_backend/relocalize')
    launch = subprocess.Popen(['ros2', 'launch', 'rm_27_stimulation', 'sim_with_nav.launch.py',
                               'world:=RMUC2027', 'nav_world:=auto', 'use_ground_truth_odom:=false',
                               'navigation_mode:=minco', 'gui:=false', 'use_rviz:=false', 'gimbal_panel:=false'],
                              stdout=(out/'launch.log').open('w'), stderr=subprocess.STDOUT, start_new_session=True)
    bag = None
    proc = None
    triggered = False
    last_sample = 0
    discovered_nodes = set()
    tf_publishers = set()
    try:
        bag = subprocess.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-o', str(out/'bag'),
                                '--topics', '/point_lio/backend_frame', '/tf', '/tf_static', '/clock',
                                '/ground_truth/odometry', '/rm27_backend/odometry', '/rm27_backend/localization_status',
                                '/plan', '/cmd_vel', '/cmd_vel_nav2_result'],
                               stdout=(out/'bag.log').open('w'), stderr=subprocess.STDOUT, start_new_session=True)
        while time.monotonic()-start < args.duration and launch.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            elapsed = time.monotonic()-start
            if elapsed-last_sample > .5:
                last_sample = elapsed
                discovered_nodes.update(node.get_node_names())
                tf_publishers.update(info.node_name for info in node.get_publishers_info_by_topic('/tf'))
                try:
                    if proc is None:
                        for child in psutil.Process(launch.pid).children(recursive=True):
                            if child.name() == 'rm27_backend_node':
                                proc = child
                                proc.cpu_percent()
                                break
                    if proc is not None:
                        memory = proc.memory_full_info()
                        resources.append(dict(wall=elapsed, cpu=proc.cpu_percent(), rss=memory.rss, pss=memory.pss))
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            if invalid_reset_future is None and elapsed > 25 and invalid_reset_client.service_is_ready():
                invalid = ResetFrontend.Request()
                invalid.instance_id = 'deliberately-stale-instance'
                invalid.epoch = 0
                invalid.request_id = 1
                invalid_reset_future = invalid_reset_client.call_async(invalid)
            if invalid_reset_future is not None and invalid_reset_future.done() and invalid_reset_result is None:
                invalid_reset_result = dict(accepted=invalid_reset_future.result().accepted,
                                            reason=invalid_reset_future.result().reason)
            if args.initial_hint and latest_stamp is not None and (
                    (hint_count == 0 and elapsed > 15) or
                    (hint_count == 1 and triggered and elapsed > args.duration*.65 and states and not states[-1]['trusted']) or
                    (hint_count == 2 and reset_sent and elapsed > args.duration*.9 and states and not states[-1]['trusted'])):
                hint = PoseWithCovarianceStamped()
                hint.header.frame_id = 'map'
                hint.header.stamp = latest_stamp
                # 固定场景配置中的出生点提示；此场景明确评估人工辅助，不冒充无初值测试。
                hint.pose.pose.position.x = -11.7
                hint.pose.pose.position.y = 3.1
                hint.pose.pose.position.z = .08
                hint.pose.pose.orientation.w = 1.
                hint_pub.publish(hint)
                hint_sent = True
                hint_count += 1
            if args.reset and not reset_sent and elapsed > args.duration*.8 and reset.service_is_ready():
                reset.call_async(Trigger.Request())
                reset_sent = True
            if args.navigate and not goals and elapsed > 20 and states and states[-1]['trusted'] and navigation.server_is_ready():
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = 'map'
                goal.pose.pose.position.x = -10.5
                goal.pose.pose.position.y = 3.1
                goal.pose.pose.orientation.w = 1.
                goal_future = navigation.send_goal_async(goal)
                goals.append({'wall': elapsed, 'target': [-10.5, 3.1]})
            if goal_future is not None and goal_future.done() and result_future is None:
                handle = goal_future.result()
                goals[-1]['accepted'] = handle.accepted
                if handle.accepted:
                    result_future = handle.get_result_async()
                else:
                    goal_future = None
            if result_future is not None and result_future.done():
                goals[-1]['status'] = result_future.result().status
                goals[-1]['finished_wall'] = elapsed
                goal_future = result_future = None
            if not args.skip_recovery and elapsed > args.duration*.55 and not triggered and states and states[-1]['trusted'] and recover.service_is_ready():
                recover.call_async(Trigger.Request())
                triggered = True
    finally:
        if bag:
            stop(bag)
        stop(launch)
        node.destroy_node()
        rclpy.shutdown()
        report = dict(nodes=sorted(discovered_nodes), tf_publishers=sorted(tf_publishers), domain=args.domain, duration=time.monotonic()-start, manual_recovery_requested=triggered, initial_hint_sent=hint_sent, initial_hint_count=hint_count, reset_requested=reset_sent,
                      invalid_reset_result=invalid_reset_result, goals=goals, states=states, estimates=estimates, truth=truth, resources=resources, diagnostics=diagnostics)
        (out/'results.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(dict(states=len(states), estimates=len(estimates),
                              trusted_samples=sum(s['trusted'] for s in states),
                              last_state=states[-1] if states else None), indent=2))


if __name__ == '__main__':
    main()
