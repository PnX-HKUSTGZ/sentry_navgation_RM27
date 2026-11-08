#!/usr/bin/env python3
"""RM27 后端先验地图因子 A/B 实验：录制、漂移注入、回放资源测量与真值评估。

子命令：
  record    无界面 Gazebo 完整导航（Point-LIO + MINCO），按路线发送目标并录制 bag。
  drift     生成注入里程计漂移的派生 bag：位姿与世界系点云同乘漂移变换，帧内几何不变。
  replay    用同一 bag 回放给独立后端进程，分别开/关地图因子，采样 CPU/RSS/PSS。
  evaluate  用 /ground_truth/odometry 与同刻 TF(base→IMU) 评估后端输出误差。

CPU 以 100% = 一个逻辑核计；真值只用于离线评分，不进入任何估计。须使用独立 ROS_DOMAIN_ID。
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
import psutil
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml'
PCD = ROOT / 'src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027.pcd'
RECORD_TOPICS = ['/point_lio/backend_frame', '/tf', '/tf_static', '/ground_truth/odometry', '/clock',
                 '/rm27_backend/diagnostics', '/rm27_backend/odometry']
REPLAY_TOPICS = ['/point_lio/backend_frame', '/tf', '/tf_static']
ROUTE = [(-2.75, 2.54), (3.15, -3.41), (7.25, 3.19), (-2.75, 2.54), (-11.7, 3.1)]


def stop(proc, group=True, timeout=20):
    """先 SIGINT 让 ROS 进程正常退出，超时再强制结束整个进程组。"""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT) if group else proc.send_signal(signal.SIGINT)
        proc.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL) if group else proc.kill()
        except ProcessLookupError:
            pass
        proc.wait()


class Sampler:
    """周期采样匹配进程的 CPU（单核百分比）、RSS 与 PSS（MiB）。"""

    def __init__(self, match, period=.2):
        self.match, self.period, self.samples = match, period, []
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def find(self):
        """按命令行匹配被测进程。"""
        return [p for p in psutil.process_iter(['cmdline'])
                if self.match in ' '.join(p.info['cmdline'] or []) and p.pid != os.getpid()]

    def run(self):
        """首次 cpu_percent 只建立基准，此后每周期记录一次。"""
        procs = {}
        while not self.done.is_set():
            for p in self.find():
                if p.pid not in procs:
                    procs[p.pid] = p
                    try:
                        p.cpu_percent(None)
                    except psutil.Error:
                        pass
            time.sleep(self.period)
            cpu = rss = pss = 0.
            alive = False
            for p in list(procs.values()):
                try:
                    cpu += p.cpu_percent(None)
                    info = p.memory_full_info()
                    rss += info.rss / 2**20
                    pss += info.pss / 2**20
                    alive = True
                except psutil.Error:
                    procs.pop(p.pid, None)
            if alive:
                self.samples.append((time.monotonic(), cpu, rss, pss))

    def start(self):
        self.thread.start()
        return self

    def finish(self, skip=0.):
        """结束采样并汇总；skip 秒内的启动样本不计入 CPU 均值。"""
        self.done.set()
        self.thread.join()
        if not self.samples:
            return {}
        t0 = self.samples[0][0]
        data = np.array([s[1:] for s in self.samples if s[0] - t0 >= skip] or [s[1:] for s in self.samples])
        return dict(samples=len(data), cpu_mean=float(data[:, 0].mean()), cpu_p95=float(np.percentile(data[:, 0], 95)),
                    cpu_max=float(data[:, 0].max()), rss_mean=float(data[:, 1].mean()), rss_max=float(data[:, 1].max()),
                    pss_mean=float(data[:, 2].mean()), pss_max=float(data[:, 2].max()))


def backend_parameters(map_enabled, out_dir, seed='tf'):
    """从仿真 MINCO 部署 YAML 取 rm27_backend 块并合并 RMUC2027 场景 profile，生成回放参数文件。"""
    configuration = yaml.safe_load(CONFIG.read_text())
    params = dict(configuration['rm27_backend']['ros__parameters'])
    profile = configuration['simulation_world_profiles']['ros__parameters']['RMUC2027'].get('rm27_backend', {})
    params.update(profile)
    params.update({'use_sim_time': True, 'prior_map.enabled': bool(map_enabled), 'prior_map.file': str(PCD),
                   'prior_map.seed_source': seed})
    path = Path(out_dir) / f'backend_map_{"on" if map_enabled else "off"}.yaml'
    path.write_text(yaml.safe_dump({'rm27_backend': {'ros__parameters': params}}, sort_keys=False))
    return path


def ros_env(domain):
    """独立 domain，避免与其他仿真或真机互相干扰。"""
    # 只在本机发现，避免同网段其他主机或容器的同 domain 话题串入。
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               RCUTILS_COLORIZED_OUTPUT='0')
    env.pop('DISPLAY', None)
    return env


def record(args):
    """无界面完整导航仿真：等待后端锚定和导航就绪后开始录制，按路线发送目标。"""
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from diagnostic_msgs.msg import DiagnosticArray
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    env = ros_env(args.domain)
    env['GZ_PARTITION'] = f'rm27_prior_map_{os.getpid()}'
    launch_log = (out / 'launch.log').open('w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'rm_27_stimulation', 'sim_with_nav.launch.py', 'world:=RMUC2027', 'nav_world:=auto',
         'use_ground_truth_odom:=false', 'navigation_mode:=minco', 'gui:=false', 'use_rviz:=false',
         'gimbal_panel:=false'], env=env, stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('rm27_prior_map_recorder')
    diagnostics = {}

    def on_diagnostics(msg):
        for status in msg.status:
            if status.name == 'rm27_backend':
                diagnostics.update({v.key: v.value for v in status.values})
    node.create_subscription(DiagnosticArray, '/rm27_backend/diagnostics', on_diagnostics, 10)
    action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    summary = dict(domain=args.domain, gz_partition=env['GZ_PARTITION'], route=ROUTE, laps=args.laps, goals=[])
    try:
        deadline = time.monotonic() + 240
        # C++ 诊断把 bool 输出为 1/0。
        anchored = lambda: diagnostics.get('global_anchor') in ('1', 'true', 'True')
        while time.monotonic() < deadline and not anchored():
            rclpy.spin_once(node, timeout_sec=.5)
        summary['anchored_before_record'] = anchored()
        if not anchored():
            raise RuntimeError(f'backend never anchored: {diagnostics}')
        if not action.wait_for_server(timeout_sec=60):
            raise RuntimeError('navigate_to_pose unavailable')
        # 以仿真时间记录 bag 时间戳，回放 --clock 时与消息 stamp 同一时间基准。
        bag = subprocess.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-o', str(out / 'bag'),
                                '--topics', *RECORD_TOPICS],
                               env=env, stdout=(out / 'bag.log').open('w'), stderr=subprocess.STDOUT,
                               start_new_session=True)
        sampler = Sampler('rm27_backend_node').start()
        time.sleep(3)
        start = time.monotonic()
        for lap, (x, y) in [(lap, point) for lap in range(args.laps) for point in ROUTE]:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.w = 1.
            future = action.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            handle = future.result()
            status, t0 = 'rejected', time.monotonic()
            if handle and handle.accepted:
                result = handle.get_result_async()
                rclpy.spin_until_future_complete(node, result, timeout_sec=args.goal_timeout)
                if result.done():
                    status = int(result.result().status)
                else:
                    status = 'timeout'
                    rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5)
            summary['goals'].append(dict(lap=lap, goal=[x, y], status=status,
                                         seconds=round(time.monotonic() - t0, 2)))
            print(summary['goals'][-1], flush=True)
        time.sleep(3)
        summary['route_wall_seconds'] = round(time.monotonic() - start, 2)
        for _ in range(6):
            rclpy.spin_once(node, timeout_sec=.5)
        summary['live_backend_diagnostics'] = dict(diagnostics)
        summary['live_backend_resources'] = sampler.finish(skip=3)
        stop(bag)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop(launch, timeout=30)
    (out / 'record_summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    print(json.dumps(summary, indent=2, ensure_ascii=False))


def open_reader(bag):
    """打开 rosbag2 只读游标，返回 reader 与话题元数据。"""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader, reader.get_all_topics_and_types()


def drift(args):
    """D(t)=Trans(v·t)·Rz(ω·t) 左乘 camera_init 位姿和完整世界系点云；其余话题原样复制。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from scipy.spatial.transform import Rotation
    reader, topics = open_reader(args.input)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for meta in topics:
        writer.create_topic(meta)
    frame_type = get_message('rm27_localization_interfaces/msg/FrontendFrame')
    t0 = None
    velocity = np.array([args.vx, args.vy, 0.])
    yaw_rate = np.radians(args.yaw_rate_deg)
    count = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == '/point_lio/backend_frame':
            msg = deserialize_message(data, frame_type)
            t = msg.odometry.header.stamp.sec + msg.odometry.header.stamp.nanosec * 1e-9
            t0 = t if t0 is None else t0
            dt = t - t0
            r = Rotation.from_euler('z', yaw_rate * dt).as_matrix()
            shift = velocity * dt
            p = msg.odometry.pose.pose
            q = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            position = r @ np.array([p.position.x, p.position.y, p.position.z]) + shift
            p.position.x, p.position.y, p.position.z = position.tolist()
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (
                Rotation.from_matrix(r) * q).as_quat().tolist()
            cloud = msg.cloud
            offsets = {f.name: f.offset for f in cloud.fields}
            raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8).reshape(-1, cloud.point_step).copy()
            xyz = np.stack([raw[:, offsets[a]:offsets[a] + 4].copy().view('<f4')[:, 0] for a in 'xyz'], axis=1)
            moved = (xyz.astype(np.float64) @ r.T + shift).astype('<f4')
            for i, a in enumerate('xyz'):
                raw[:, offsets[a]:offsets[a] + 4] = moved[:, i:i + 1].view(np.uint8)
            cloud.data = raw.tobytes()
            data = serialize_message(msg)
            count += 1
        writer.write(topic, data, stamp)
    del writer
    info = dict(frames=count, vx=args.vx, vy=args.vy, yaw_rate_deg=args.yaw_rate_deg)
    Path(str(args.output) + '.json').write_text(json.dumps(info, indent=2))
    print(info)


def replay(args):
    """每轮启动全新后端进程，等待地图加载后以 1 倍速回放；记录输出、诊断和资源。"""
    import rclpy
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    env = ros_env(args.domain)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    rclpy.init()
    results = []
    for repetition in range(args.repetitions):
        for map_enabled in (True, False) if repetition % 2 == 0 else (False, True):
            tag = f'{Path(args.bag).name}_map_{"on" if map_enabled else "off"}_r{repetition}'
            params = backend_parameters(map_enabled, out)
            node = rclpy.create_node(f'rm27_prior_map_observer_{repetition}_{int(map_enabled)}')
            outputs, diagnostics = [], {}

            def on_odometry(msg):
                p = msg.pose.pose
                outputs.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.header.frame_id,
                                p.position.x, p.position.y, p.position.z,
                                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

            def on_diagnostics(msg):
                for status in msg.status:
                    if status.name == 'rm27_backend':
                        diagnostics.update({v.key: v.value for v in status.values})
            node.create_subscription(Odometry, '/rm27_backend/odometry', on_odometry, 100)
            node.create_subscription(DiagnosticArray, '/rm27_backend/diagnostics', on_diagnostics, 10)
            stop_spin = threading.Event()

            def spin():
                while not stop_spin.is_set():
                    rclpy.spin_once(node, timeout_sec=.05)
            spinner = threading.Thread(target=spin, daemon=True)
            spinner.start()
            backend = subprocess.Popen(['ros2', 'run', 'rm27_backend', 'rm27_backend_node', '--ros-args',
                                        '--params-file', str(params), '-r', '__node:=rm27_backend'],
                                       env=env, stdout=(out / f'{tag}.log').open('w'), stderr=subprocess.STDOUT,
                                       start_new_session=True)
            sampler = Sampler('rm27_backend_node').start()
            # 地图在后台线程加载；等待加载完成再回放，使两组输入时刻一致。
            load_start = time.monotonic()
            while map_enabled and time.monotonic() - load_start < 60:
                if diagnostics.get('map_state') not in (None, 'loading'):
                    break
                time.sleep(.2)
            load_seconds = time.monotonic() - load_start
            time.sleep(1.)
            play_start = time.monotonic()
            subprocess.run(['ros2', 'bag', 'play', str(args.bag), '--clock', '100', '--topics', *REPLAY_TOPICS],
                           env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            play_seconds = time.monotonic() - play_start
            time.sleep(2.5)
            resources = sampler.finish(skip=load_seconds + 1.)
            stop(backend)
            stop_spin.set()
            spinner.join()
            node.destroy_node()
            result = dict(tag=tag, bag=str(args.bag), map_enabled=map_enabled, repetition=repetition,
                          map_load_wall_seconds=round(load_seconds, 2) if map_enabled else None,
                          play_wall_seconds=round(play_seconds, 2), outputs=len(outputs),
                          diagnostics=dict(diagnostics), resources=resources)
            (out / f'{tag}_odometry.json').write_text(json.dumps(outputs))
            results.append(result)
            print(json.dumps({k: v for k, v in result.items() if k != 'diagnostics'}), flush=True)
    rclpy.shutdown()
    summary = out / f'{Path(args.bag).name}_replay.json'
    summary.write_text(json.dumps(results, indent=2))


def pose_matrix(position, quaternion):
    """位置和 xyzw 四元数转 4×4 齐次矩阵。"""
    from scipy.spatial.transform import Rotation
    m = np.eye(4)
    m[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    m[:3, 3] = position
    return m


def load_truth(bag):
    """读取真值（world=map，child=base_footprint）和全部 TF，返回插值函数所需数据与 TF 缓冲。"""
    from rclpy.serialization import deserialize_message
    from rclpy.duration import Duration
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer
    reader, topics = open_reader(bag)
    types = {t.name: get_message(t.type) for t in topics if t.name in ('/ground_truth/odometry', '/tf', '/tf_static')}
    buffer = Buffer(cache_time=Duration(seconds=3600))
    truth_t, truth_p, truth_q = [], [], []
    child = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == '/ground_truth/odometry':
            child = msg.child_frame_id
            truth_t.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            truth_p.append([p.x, p.y, p.z])
            truth_q.append([q.x, q.y, q.z, q.w])
        else:
            for transform in msg.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'bag')
                else:
                    buffer.set_transform(transform, 'bag')
    order = np.argsort(truth_t)
    return (np.array(truth_t)[order], np.array(truth_p)[order], np.array(truth_q)[order], child, buffer)


def transform_at(buffer, target, source, t):
    """查询 bag 内 target←source 在时刻 t 的变换。"""
    from rclpy.time import Time
    msg = buffer.lookup_transform(target, source, Time(nanoseconds=int(round(t * 1e9))))
    tr, ro = msg.transform.translation, msg.transform.rotation
    return pose_matrix([tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w])


def evaluate(args):
    """真值 IMU 位姿 = 真值 base 位姿 × 同刻 TF(base→imu_link)，与后端 map 系输出比较。
    纯里程计输出位于 camera_init，用首个输出时刻的 map←camera_init（导航定位链 TF）对齐一次。"""
    from scipy.spatial.transform import Rotation, Slerp
    truth_t, truth_p, truth_q, base, buffer = load_truth(args.truth_bag)
    slerp = Slerp(truth_t, Rotation.from_quat(truth_q))
    world = yaml.safe_load(CONFIG.read_text())['rm27_backend']['ros__parameters']['input.world_frame']
    report = []
    for path in sorted(Path(args.replay).glob('*_odometry.json')):
        outputs = json.loads(path.read_text())
        rows, alignment = [], None
        for t, frame, *pose in outputs:
            if t < truth_t[0] or t > truth_t[-1]:
                continue
            try:
                base_imu = transform_at(buffer, base, args.imu_frame, t)
            except Exception:
                continue
            estimate = pose_matrix(pose[:3], pose[3:])
            if frame != 'map':
                if alignment is None:
                    alignment = transform_at(buffer, 'map', world, t)
                estimate = alignment @ estimate
            truth = pose_matrix([np.interp(t, truth_t, truth_p[:, i]) for i in range(3)],
                                slerp([t]).as_quat()[0]) @ base_imu
            error = np.linalg.inv(truth) @ estimate
            yaw = np.degrees(abs(Rotation.from_matrix(error[:3, :3]).as_euler('zyx')[0]))
            rows.append([t, np.linalg.norm(estimate[:3, 3] - truth[:3, 3]),
                         np.linalg.norm(estimate[:2, 3] - truth[:2, 3]), yaw])
        if not rows:
            continue
        data = np.array(rows)
        elapsed = data[:, 0] - data[0, 0]
        bins = {}
        for edge in range(30, int(elapsed[-1]) + 30, 30):
            mask = (elapsed > edge - 30) & (elapsed <= edge)
            if mask.any():
                bins[str(edge)] = round(float(data[mask, 1].mean()), 4)
        report.append(dict(run=path.name.replace('_odometry.json', ''), samples=len(data),
                           first_output_s=round(float(data[0, 0] - truth_t[0]), 2),
                           rmse_m=float(np.sqrt((data[:, 1] ** 2).mean())), mean_m=float(data[:, 1].mean()),
                           p95_m=float(np.percentile(data[:, 1], 95)), max_m=float(data[:, 1].max()),
                           final_m=float(data[-1, 1]), xy_rmse_m=float(np.sqrt((data[:, 2] ** 2).mean())),
                           yaw_mean_deg=float(data[:, 3].mean()), yaw_max_deg=float(data[:, 3].max()),
                           mean_error_by_30s=bins))
        print(json.dumps(report[-1]), flush=True)
    Path(args.output).write_text(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('record')
    p.add_argument('--output', required=True)
    p.add_argument('--domain', type=int, default=91)
    p.add_argument('--goal-timeout', type=float, default=90.)
    p.add_argument('--laps', type=int, default=2)
    p.set_defaults(func=record)
    p = sub.add_parser('drift')
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--vx', type=float, default=.004)
    p.add_argument('--vy', type=float, default=.002)
    p.add_argument('--yaw-rate-deg', type=float, default=.015)
    p.set_defaults(func=drift)
    p = sub.add_parser('replay')
    p.add_argument('--bag', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--domain', type=int, default=92)
    p.add_argument('--repetitions', type=int, default=3)
    p.set_defaults(func=replay)
    p = sub.add_parser('evaluate')
    p.add_argument('--replay', required=True)
    p.add_argument('--truth-bag', required=True)
    p.add_argument('--imu-frame', default='imu_link')
    p.add_argument('--output', required=True)
    p.set_defaults(func=evaluate)
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
