#!/usr/bin/env python3
"""Compare isolated worlds with the same robot, renderer and sensor workload."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


def stop(process):
    for sig, timeout in [(signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        # A launch parent can exit before its Gazebo children.
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
    process.wait(timeout=3)


def cpu_seconds(processes):
    seen, total = set(), 0.0
    for process in processes:
        try:
            children = [psutil.Process(process.pid)]
            children.extend(children[0].children(recursive=True))
        except psutil.NoSuchProcess:
            continue
        for child in children:
            if child.pid in seen:
                continue
            seen.add(child.pid)
            try:
                cpu = child.cpu_times()
                total += cpu.user+cpu.system
            except psutil.NoSuchProcess:
                pass
    return total


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('worlds', nargs='+')
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('GZ_PARTITION') != 'rmul2027-collision-validation':
        raise RuntimeError('Use GZ_PARTITION=rmul2027-collision-validation in an isolated ROS domain')
    rclpy.init()
    node = rclpy.create_node('benchmark_world_runtime')
    state = {}
    def clock(message):
        state['clock'] = message.clock.sec+message.clock.nanosec*1e-9
    def cloud(_):
        state['clouds'] = state.get('clouds', 0)+1
    def odom(_):
        state['odometry'] = state.get('odometry', 0)+1
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/livox/lidar', cloud, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/ground_truth/odometry', odom, qos_profile_sensor_data)
    results = []
    try:
        for world in args.worlds:
            state.clear()
            processes, logs = [], []
            logfile = args.output.with_name(args.output.stem+'_'+world+'.log')
            def launch(file, *arguments):
                stream = logfile.open('a')
                logs.append(stream)
                process = subprocess.Popen(['ros2', 'launch', 'rm_27_stimulation', file, *arguments],
                                           stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                processes.append(process)
            def wait(predicate, timeout=180):
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                    if any(p.poll() is not None for p in processes):
                        raise RuntimeError('Simulation exited; inspect '+str(logfile))
                    if predicate():
                        return
                raise RuntimeError('Simulation feedback timeout; inspect '+str(logfile))
            try:
                started = time.monotonic()
                launch('rm_simulation.launch.py', 'sim_world:='+world, 'gui:=false')
                wait(lambda: state.get('clock', 0) > 0)
                clock_ready = time.monotonic()-started
                print(json.dumps(dict(world=world, physics_ready_s=clock_ready)), flush=True)
                launch('spawn_robot.launch.py', 'sim_world:='+world,
                       'spawn_delay:=0.2', 'gimbal_panel:=false')
                wait(lambda: state.get('clouds', 0) >= 10 and state.get('odometry', 0) >= 20)
                sensors_ready = time.monotonic()-started
                warmup = time.monotonic()+3
                wait(lambda: time.monotonic() >= warmup)
                t0, sim0, clouds0 = time.monotonic(), state['clock'], state['clouds']
                cpu0 = cpu_seconds(processes)
                wait(lambda: time.monotonic() >= t0+args.seconds)
                elapsed, simulated = time.monotonic()-t0, state['clock']-sim0
                cpu = cpu_seconds(processes)-cpu0
                result = dict(world=world, physics_ready_s=clock_ready, sensors_ready_s=sensors_ready,
                              measurement_wall_s=elapsed, measurement_sim_s=simulated,
                              real_time_factor=simulated/elapsed, cpu_seconds=cpu,
                              cpu_seconds_per_sim_second=cpu/simulated,
                              lidar_frames_per_sim_second=(state['clouds']-clouds0)/simulated)
                results.append(result)
                print(json.dumps(result), flush=True)
                args.output.write_text(json.dumps(results, indent=2)+'\n')
            finally:
                for process in reversed(processes):
                    stop(process)
                for stream in logs:
                    stream.close()
                # Drain samples from the world that just stopped.
                end = time.monotonic()+.5
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
