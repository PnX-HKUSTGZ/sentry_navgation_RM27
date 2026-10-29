#!/usr/bin/env python3
"""Run isolated headless ground-truth navigation benchmark repetitions."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def write_json(path, value):
    tmp = path.with_suffix('.tmp')
    with tmp.open('w', encoding='utf-8') as out:
        json.dump(value, out, indent=2)
        out.write('\n')
    os.replace(tmp, path)


def stop_group(process):
    if process is None:
        return
    # Each child is a session leader; the group can outlive its leader.
    for sig, delay in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.1)
        else:
            continue
        break
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--params', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--domain', type=int, default=92)
    parser.add_argument('--world', default='RMUC2026')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--goals', required=True)
    parser.add_argument('--mode', choices=['gt', 'full_chain'], default='gt')
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--wall-timeout', type=float, default=240.0)
    args = parser.parse_args()
    if args.repeats < 1 or not 0 <= args.domain <= 232 or args.timeout <= 0 or args.wall_timeout <= 0:
        parser.error('repeats and timeouts must be positive; domain must be between 0 and 232')
    try:
        goals = json.loads(args.goals)
        if not isinstance(goals, list) or not goals or any(not isinstance(g, list) or len(g) != 2 for g in goals):
            raise ValueError('expected nonempty list of [x, y] pairs')
    except ValueError as exc:
        parser.error(f'invalid goals: {exc}')
    params = Path(args.params).expanduser().resolve()
    if not params.is_file():
        parser.error(f'parameter file does not exist: {params}')
    output = Path(args.output).expanduser().resolve()
    repeat_dirs = [output / f'repeat_{i:02d}' for i in range(1, args.repeats + 1)]
    if any(path.exists() for path in repeat_dirs):
        parser.error('refusing to overwrite an existing repeat directory')
    output.mkdir(parents=True, exist_ok=True)
    script_dir = Path(__file__).resolve().parent
    try:
        git_head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=script_dir, text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        git_head = None
    repo = script_dir.parent.parent
    tracked = ['install/minco_controller/lib/libminco_controller.so', 'install/minco_planner/lib/libminco_planner.so',
               'install/rog_map/lib/librog_map.a', 'src/navigation/minco_controller/src/minco_mpc_controller.cpp',
               'src/navigation/minco_controller/src/mpc_solver.cpp', 'src/navigation/minco_planner/src/minco_core/minco_fsm.cpp',
               'src/navigation/minco_planner/src/minco_core/minco_planner.cpp', 'src/navigation/minco_planner/src/traj_opt/minco_optimizer.cpp',
               'src/pb2025_nav_bringup/config/simulation/minco_params.yaml']
    file_hashes = {name: hashlib.sha256((repo / name).read_bytes()).hexdigest() if (repo / name).is_file() else None for name in tracked}
    try:
        diff = subprocess.check_output(['git', 'diff', '--binary', 'HEAD'], cwd=repo)
        diff_sha256 = hashlib.sha256(diff).hexdigest()
    except (OSError, subprocess.CalledProcessError):
        diff_sha256 = None
    config_bytes = params.read_bytes()
    sha256 = hashlib.sha256(config_bytes).hexdigest()
    results = []
    interrupted = False
    for index, directory in enumerate(repeat_dirs, 1):
        # mkdir without exist_ok also guards against concurrent runs.
        directory.mkdir()
        snapshot = directory / 'params.yaml'
        snapshot.write_bytes(config_bytes)
        partition = f'rm27_bench_{os.getpid()}_{index}_{uuid.uuid4().hex[:10]}'
        env = os.environ.copy()
        env.update({'ROS_DOMAIN_ID': str(args.domain), 'GZ_PARTITION': partition})
        launch = ['ros2', 'launch', 'rm_27_stimulation', 'sim_with_nav.launch.py',
                  f'world:={args.world}', 'nav_world:=auto', 'navigation_mode:=minco',
                  f'use_ground_truth_odom:={str(args.mode == "gt").lower()}', 'enable_legacy_terrain:=auto',
                  'gui:=false', 'use_rviz:=false', 'gimbal_panel:=false',
                  f'params_file:={snapshot}', f'extra_gazebo_args:=--seed {args.seed}']
        observer = [sys.executable, str(script_dir / 'observe.py'), '--output', str(directory),
                    '--goals', args.goals, '--mode', args.mode, '--timeout', str(args.timeout),
                    '--wall-timeout', str(args.wall_timeout)]
        manifest = {'repeat': index, 'git_head': git_head, 'params_source': str(params),
                    'params_snapshot': str(snapshot), 'params_sha256': sha256, 'seed': args.seed,
                    'file_sha256': file_hashes, 'git_diff_sha256': diff_sha256,
                    'environment': {'ROS_DOMAIN_ID': str(args.domain), 'GZ_PARTITION': partition},
                    'launch_command': launch, 'observer_command': observer, 'status': 'running'}
        write_json(directory / 'manifest.json', manifest)
        launch_process = observer_process = None
        print(f'repeat {index}/{args.repeats}: {directory}', flush=True)
        try:
            with (directory / 'launch.log').open('w', encoding='utf-8') as launch_log, (directory / 'observer.log').open('w', encoding='utf-8') as observer_log:
                launch_process = subprocess.Popen(launch, stdout=launch_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                observer_process = subprocess.Popen(observer, stdout=observer_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                while observer_process.poll() is None:
                    if launch_process.poll() is not None:
                        print(f'launch exited early: {launch_process.returncode}', flush=True)
                        stop_group(observer_process)
                        break
                    time.sleep(0.2)
                observer_code = observer_process.poll()
                launch_code = launch_process.poll()
                manifest.update({'observer_exit_code': observer_code, 'launch_exit_code_before_cleanup': launch_code,
                                 'status': 'succeeded' if observer_code == 0 and launch_code is None else 'failed'})
        except KeyboardInterrupt:
            interrupted = True
            manifest.update({'status': 'interrupted', 'observer_exit_code': None,
                             'launch_exit_code_before_cleanup': launch_process.poll() if launch_process else None})
        except OSError as exc:
            manifest.update({'status': 'failed', 'error': str(exc), 'observer_exit_code': None})
        finally:
            stop_group(observer_process)
            stop_group(launch_process)
            manifest['observer_exit_code_after_cleanup'] = observer_process.poll() if observer_process else None
            manifest['launch_exit_code_after_cleanup'] = launch_process.poll() if launch_process else None
            write_json(directory / 'manifest.json', manifest)
            results.append({'repeat': index, 'directory': str(directory), 'status': manifest['status'],
                            'observer_exit_code': manifest.get('observer_exit_code')})
            write_json(output / 'summary.json', {'repeats': results})
        print(f'repeat {index}: {manifest["status"]}', flush=True)
        if interrupted:
            break
    return 130 if interrupted else (0 if all(r['status'] == 'succeeded' for r in results) else 1)


if __name__ == '__main__':
    sys.exit(main())
