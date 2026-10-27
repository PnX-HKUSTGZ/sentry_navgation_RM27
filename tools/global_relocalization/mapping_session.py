#!/usr/bin/env python3
"""Own an isolated full UC mapping test and leave its Gazebo/RViz result visible."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import secrets
import shlex
import signal
import subprocess
import time
import uuid

from runtime import ROOT, owned_process, start_process, stop_process

HERE = Path(__file__).resolve().parent
STATE = HERE / ".build/full_uc_visualization/session.json"


def write_state(state):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    temporary = STATE.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(STATE)


def stop_session():
    if not STATE.exists():
        print("No saved full UC session.")
        return
    state = json.loads(STATE.read_text())
    # The mapping process handles SIGINT by stopping commands and retaining
    # evidence. Keep the simulator alive until its cleanup has finished.
    for name in ("mapping", "observer", "rviz", "gui", "simulation"):
        info = state.get("processes", {}).get(name)
        process = owned_process(info)
        if process is None:
            continue
        if name == "mapping":
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=45)
            except __import__("psutil").TimeoutExpired:
                stop_process(info)
        else:
            stop_process(info)
    state["stopped"] = True
    write_state(state)
    print("Stopped this full UC session; saved evidence retained.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--view", type=Path, help="Display a saved capture without driving or rerunning SLAM")
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-mode", choices=("auto", "gpu", "cpu"), default="auto")
    parser.add_argument("--speed", type=float, default=.3)
    parser.add_argument("--rotation-speed", type=float, default=0.)
    parser.add_argument("--imu-input", action="store_true", help="Opt in to the documented simulation-only Point-LIO IMU override")
    parser.add_argument("--reference", type=Path, default=HERE / "assets/RMUC2026/map.pcd")
    args = parser.parse_args()
    if args.stop:
        stop_session()
        return
    if STATE.exists():
        old = json.loads(STATE.read_text())
        if any(owned_process(info) for info in old.get("processes", {}).values()):
            raise SystemExit("A full UC session is already open. Stop it with --stop before starting another.")
    if args.view and not (args.view / "mapping_errors.json").exists():
        parser.error("--view requires an existing mapping_errors.json")
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run = (args.view or args.output or HERE / "data" / ("full_uc_" + stamp) / "mapping").resolve()
    if not args.view and run.exists():
        parser.error("output must not already exist")
    logs = run.parent / (run.name + "_session" + ("_" + uuid.uuid4().hex[:6] if args.view else ""))
    logs.mkdir(parents=True, exist_ok=False)
    domain = int(os.environ.get("ROS_DOMAIN_ID", secrets.randbelow(100) + 80))
    partition = "rm27_full_uc_" + uuid.uuid4().hex
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), GZ_PARTITION=partition)
    state = {"result": str(run), "logs": str(logs), "domain": domain, "partition": partition, "processes": {}}
    (logs / "environment.bash").write_text(
        "source " + shlex.quote(str(HERE / "environment.bash")) + "\n"
        + f"export ROS_DOMAIN_ID={domain}\nexport GZ_PARTITION={partition}\n")
    write_state(state)
    def start(name, command, env=environment):
        state["processes"][name] = start_process(command, logs / f"{name}.log", env)
        write_state(state)
    mapper = HERE / ".build/mapping_ros/install/point_lio/lib/point_lio/pointlio_mapping"
    try:
        start("simulation", ["bash", str(HERE / "start_simulation.bash"), "gui:=false", "render_mode:=" + args.render_mode])
        if not args.headless:
            from rm27_rendering import select_rendering
            rendering = select_rendering(args.render_mode, gui=True, purpose="rviz", environ=environment)
            start("gui", [*rendering.prefix, "gz", "sim", "-g", "--gui-config", str(HERE / "config/mapping_gazebo.config")], rendering.environment)
            start("rviz", [*rendering.prefix, "ros2", "run", "rviz2", "rviz2", "-d",
                           str(HERE / "config/mapping_evaluation.rviz"), "--ros-args", "-r", "__node:=full_uc_mapping_view"], rendering.environment)
        start("observer", ["/usr/bin/python3", str(HERE / "mapping_observer.py"), "--result", str(run), "--gazebo"])
        if args.view:
            print(f"Saved result visible in Gazebo/RViz: {run}\nSession: {STATE}")
            return
        command = ["/usr/bin/python3", str(HERE / "full_uc_mapping.py"), "--output", str(run),
                   "--mapper", str(mapper), "--speed", str(args.speed), "--rotation-speed", str(args.rotation_speed)]
        if args.imu_input:
            command.extend(["--pointlio-overrides", str(HERE / "config/pointlio_simulation_imu_input.yaml")])
        start("mapping", command)
        print(f"Full UC mapping started: {run}\nEnvironment: {logs / 'environment.bash'}", flush=True)
        while owned_process(state["processes"]["mapping"]) is not None:
            time.sleep(.5)
        if not (run / "mapping_errors.json").exists():
            raise RuntimeError("No scored mapping data; inspect session mapping.log")
        from report_full_uc_mapping import report
        report(run, args.reference if args.reference.exists() else None,
               ROOT / "src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml")
        state["report"] = str(run / "report.html")
        write_state(state)
        print(f"Report: {run / 'report.html'}\nGazebo and RViz remain open. Stop with --stop.", flush=True)
    except (KeyboardInterrupt, Exception):
        stop_session()
        raise


if __name__ == "__main__":
    main()
