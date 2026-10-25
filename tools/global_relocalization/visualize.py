#!/usr/bin/env python3
"""Start or restore Gazebo, both RViz views, and the final competition demo."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time
import uuid

import psutil
import yaml

from runtime import (
    ROOT,
    INSTALL,
    MANIFEST as BUILD_MANIFEST,
    boot_id,
    digest,
    owned_process,
    start_process,
    start_ticks,
    stop_process,
)

SESSION = ROOT / "tools/global_relocalization/.build/visualization"
STATE = SESSION / "session.json"
ORDER = ("demo", "algorithm", "display_helper", "rviz_map", "rviz_reference", "gazebo")


def save(state):
    temporary = STATE.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(STATE)


def stop_session():
    if not STATE.exists():
        print("No visualization session to stop.")
        return
    state = json.loads(STATE.read_text())
    controller = owned_process(state.get("controller"))
    if controller is not None and controller.pid != os.getpid():
        controller.send_signal(signal.SIGTERM)
        try:
            controller.wait(timeout=15)
        except psutil.TimeoutExpired:
            raise RuntimeError(
                "The visualization launcher has not stopped yet; retry --stop"
            )
        state = json.loads(STATE.read_text())
    for name in ORDER:
        stop_process(state.get("processes", {}).get(name))
    print("Stopped this session's Gazebo, localization server, and RViz windows.")


def interrupted(signum, frame):
    raise KeyboardInterrupt


def session_settings(args):
    settings = {
        key: str(getattr(args, key).resolve()) for key in ("map", "manifest", "params")
    }
    settings["render_mode"] = args.render_mode
    for key in ("map", "manifest", "params"):
        if not Path(settings[key]).is_file():
            raise ValueError(f"Missing {key}: {settings[key]}")
    settings["input_sha256"] = {
        key: digest(settings[key]) for key in ("map", "manifest", "params")
    }
    settings["build_sha256"] = digest(BUILD_MANIFEST)
    return settings


def run(args):
    from rm27_rendering import select_rendering

    settings = session_settings(args)
    reference = json.loads(args.manifest.read_text())
    if reference["map_sha256"] != digest(args.map):
        raise ValueError("Map hash differs from its manifest")
    state = json.loads(STATE.read_text()) if STATE.exists() else {}
    if state and state.get("settings") != settings:
        stop_session()
        state = {}
    if not state:
        state = {
            "settings": settings,
            "domain": int(os.environ.get("ROS_DOMAIN_ID", secrets.randbelow(100) + 80)),
            "partition": "global_relocalization_" + uuid.uuid4().hex,
            "processes": {},
        }
    state["controller"] = {
        "pid": os.getpid(),
        "boot_id": boot_id(),
        "start_ticks": start_ticks(os.getpid()),
    }
    save(state)
    environment = dict(
        os.environ, ROS_DOMAIN_ID=str(state["domain"]), GZ_PARTITION=state["partition"]
    )
    # This file contains session selectors only; no credentials or inherited environment.
    (SESSION / "environment.bash").write_text(
        f'source "{ROOT}/tools/global_relocalization/environment.bash"\n'
        f'export ROS_DOMAIN_ID={state["domain"]}\nexport GZ_PARTITION={state["partition"]}\n'
    )
    rendering = select_rendering(
        args.render_mode, gui=True, purpose="rviz", environ=environment
    )
    processes = state["processes"]
    started = []
    try:
        if owned_process(processes.get("gazebo")) is None:
            for name in ("algorithm", "display_helper"):
                stop_process(processes.get(name))
            subprocess.run(
                [
                    "bash",
                    str(ROOT / "tools/global_relocalization/prepare_simulation.bash"),
                ],
                env=environment,
                check=True,
            )
            spawn = yaml.safe_load(
                (ROOT / "src/rm_27_stimulation/config/spawn_poses.yaml").read_text()
            )
            spawn["spawn_poses"]["RMUC2026"].update(
                x=-9.575, y=-1.285, z=0.089, yaw=-1.5707963267948966
            )
            (SESSION / "spawn_poses.yaml").write_text(yaml.safe_dump(spawn))
        library = INSTALL / "global_relocalization/lib/global_relocalization"
        share = INSTALL / "global_relocalization/share/global_relocalization"
        commands = {
            "gazebo": [
                "ros2",
                "launch",
                str(ROOT / "src/global_relocalization/launch/simulation.launch.py"),
                "gui:=true",
                "render_mode:=" + args.render_mode,
                "robot_description_file:="
                + str(ROOT / "tools/global_relocalization/.build/robot_velocity.urdf"),
                "spawn_poses_config:=" + str(SESSION / "spawn_poses.yaml"),
            ],
            "algorithm": [
                str(library / "global_relocalization_node"),
                "--ros-args",
                "--params-file",
                settings["params"],
                "-p",
                "map_path:=" + settings["map"],
                "-p",
                "manifest_path:=" + settings["manifest"],
                "-p",
                "visualization_enabled:=true",
                "-p",
                "visualization_max_points:=12000",
                "-p",
                "visualization_cloud_hz:=3.0",
            ],
            "display_helper": ["/usr/bin/python3", str(library / "display_helper.py")],
            "rviz_map": [
                "ros2",
                "run",
                "rviz2",
                "rviz2",
                "-d",
                str(share / "rviz/observer.rviz"),
                "--ros-args",
                "-r",
                "__node:=relocalization_map_view",
                "-p",
                "use_sim_time:=true",
            ],
            "rviz_reference": [
                "ros2",
                "run",
                "rviz2",
                "rviz2",
                "-d",
                str(share / "rviz/observer_reference.rviz"),
                "--ros-args",
                "-r",
                "__node:=relocalization_scan_view",
                "-p",
                "use_sim_time:=true",
            ],
        }
        for name, command in commands.items():
            if owned_process(processes.get(name)) is None:
                if name.startswith("rviz"):
                    command = [*rendering.prefix, *command]
                processes[name] = start_process(
                    command,
                    SESSION / f"{name}.log",
                    rendering.environment if name.startswith("rviz") else environment,
                )
                started.append(name)
                save(state)
                print("Started:", name, flush=True)
        for attempt in range(4):
            with (SESSION / "startup_audit.log").open("a") as log:
                audit = subprocess.run(
                    [
                        "/usr/bin/python3",
                        str(ROOT / "tools/global_relocalization/audit_simulation.py"),
                        "--output",
                        str(SESSION / "startup_audit.json"),
                        "--duration",
                        "4",
                    ],
                    env=environment,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=15,
                )
            if audit.returncode == 0:
                break
            print("Waiting for Gazebo LiDAR and IMU...", flush=True)
        else:
            raise RuntimeError(f"Simulation is not ready; see {SESSION / 'gazebo.log'}")
        missing = [
            name for name in commands if owned_process(processes.get(name)) is None
        ]
        if missing:
            raise RuntimeError(
                f"Processes exited during startup: {missing}; see {SESSION}"
            )
        print("Gazebo and both RViz windows are running. Session:", SESSION, flush=True)
        if not args.no_demo:
            result = SESSION / ("demo_" + time.strftime("%Y%m%d_%H%M%S") + ".json")
            processes["demo"] = start_process(
                [
                    "/usr/bin/python3",
                    str(ROOT / "tools/global_relocalization/demo.py"),
                    "--output",
                    str(result),
                ],
                SESSION / "demo.log",
                environment,
            )
            save(state)
            demo = owned_process(processes["demo"])
            if demo is None:
                raise RuntimeError(f"Demo client exited; see {SESSION / 'demo.log'}")
            try:
                code = demo.wait(timeout=155)
            finally:
                stop_process(processes["demo"])
            if code != 0:
                raise RuntimeError(f"Demo did not succeed; see {SESSION / 'demo.log'}")
            report = json.loads(result.read_text())
            print(
                "Localization:",
                report["success"],
                "start-to-pose seconds:",
                report["terminal_status"]["competition"]["trigger_to_pose_sec"],
                flush=True,
            )
    except BaseException:
        for name in reversed(started):
            stop_process(processes.get(name))
        raise
    finally:
        state.pop("controller", None)
        save(state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    assets = ROOT / "tools/global_relocalization/assets/RMUC2026"
    parser.add_argument("--map", type=Path, default=assets / "map.pcd")
    parser.add_argument("--manifest", type=Path, default=assets / "manifest.json")
    parser.add_argument(
        "--params",
        type=Path,
        default=ROOT / "src/global_relocalization/config/competition.yaml",
    )
    parser.add_argument("--render-mode", choices=("auto", "gpu", "cpu"), default="auto")
    parser.add_argument(
        "--no-demo",
        action="store_true",
        help="Open or restore windows without starting a goal",
    )
    parser.add_argument(
        "--stop", action="store_true", help="Stop only this launcher's owned processes"
    )
    args = parser.parse_args()
    SESSION.mkdir(parents=True, exist_ok=True)
    if args.stop:
        stop_session()
        return
    signal.signal(signal.SIGTERM, interrupted)
    with (SESSION / "launch.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(
                "A visualization startup or demonstration is already running"
            )
        run(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
