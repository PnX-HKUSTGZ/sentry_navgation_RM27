#!/usr/bin/env python3
"""Real Bullet regression: drop, leave a platform, turn both ways, watchdog stop.

Run after sourcing the workspace and building rm_27_stimulation:
  python3 src/rm_27_stimulation/test/check_drive_physics.py --output /tmp/drive-test
Uses the production robot inertia/collisions/actuator, without rendering sensors.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[3]
    os.environ["GZ_PARTITION"] = "rm27_drive_test_" + uuid.uuid4().hex
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
        str(root / "install/rm_27_stimulation/lib")
        + os.pathsep
        + os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    )
    from gz.transport13 import Node
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.twist_pb2 import Twist

    urdf = args.output / "robot.urdf"
    urdf.write_bytes(
        subprocess.check_output(
            [
                "xacro",
                str(root / "src/rm_27_stimulation/urdf/simulation_waking_robot.xacro"),
            ]
        )
    )
    model = ET.fromstring(subprocess.check_output(["gz", "sdf", "-p", str(urdf)])).find(
        "model"
    )
    for link in model.findall("link"):
        for item in list(link):
            if item.tag in ("sensor", "visual"):
                link.remove(item)
    ET.SubElement(model, "pose").text = "0 0 1 0 0 0"
    sdf = ET.fromstring(
        """<sdf version="1.9"><world name="drive_test">
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <physics name="default" type="bullet">
        <max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor>
      </physics><gravity>0 0 -9.81</gravity>
      <model name="floor"><static>true</static><pose>0 0 -0.5 0 0 0</pose>
        <link name="floor"><collision name="floor"><geometry>
          <box><size>100 100 1</size></box>
        </geometry></collision></link></model>
      <model name="platform"><static>true</static><pose>0 0 0.15 0 0 0</pose>
        <link name="platform"><collision name="platform"><geometry>
          <box><size>2 4 0.3</size></box>
        </geometry></collision></link></model>
    </world></sdf>"""
    )
    sdf.find("world").append(model)
    world = args.output / "world.sdf"
    ET.ElementTree(sdf).write(world)
    node = Node()
    samples = []

    def observe(msg):
        p, q, v, w = (
            msg.pose.position,
            msg.pose.orientation,
            msg.twist.linear,
            msg.twist.angular,
        )
        samples.append(
            {
                "time": msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                "position": [p.x, p.y, p.z],
                "yaw": math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
                "linear": [v.x, v.y, v.z],
                "angular": [w.x, w.y, w.z],
            }
        )

    assert node.subscribe(Odometry, "/ground_truth/odometry", observe)
    publisher = node.advertise("/cmd_vel", Twist)
    report = {"phases": [], "passed": False}
    with (args.output / "gazebo.log").open("w") as log:
        process = subprocess.Popen(
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--physics-engine",
                "gz-physics-bullet-featherstone-plugin",
                str(world),
            ],
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 20
            while not samples:
                assert process.poll() is None, "Gazebo exited; inspect gazebo.log"
                assert time.monotonic() < deadline, "No physics odometry"
                publisher.publish(Twist())
                time.sleep(0.02)

            def phase(name, duration, vx=0.0, wz=0.0, publish=True):
                start = samples[-1]["time"]
                timeout = time.monotonic() + duration * 5 + 10
                while samples[-1]["time"] < start + duration:
                    assert time.monotonic() < timeout, "Physics stopped advancing"
                    if publish:
                        msg = Twist()
                        msg.linear.x, msg.angular.z = vx, wz
                        publisher.publish(msg)
                    time.sleep(0.02)
                result = dict(samples[-1], phase=name)
                report["phases"].append(result)
                print(json.dumps(result), flush=True)
                return result

            dropped = phase("drop_to_platform", 2)
            assert (
                abs(dropped["position"][2] - 0.38) < 0.015
            ), "Gravity did not settle on platform"
            driven = phase("drive_off_platform", 6, vx=0.5)
            assert driven["position"][0] > 2, "No forward motion"
            assert (
                abs(driven["position"][2] - 0.08) < 0.015
            ), "Robot floats after leaving platform"
            phase("stop", 2)
            for name, speed in [("turn_left", 1.0), ("turn_right", -1.0)]:
                turned = phase(name, 4, wz=speed)
                assert (
                    abs(turned["angular"][2] - speed) < 0.12
                ), "Incorrect yaw response"
                assert abs(turned["position"][2] - 0.08) < 0.015, "Turning lifts robot"
            phase("drive_before_timeout", 2, vx=0.5)
            stopped = phase("command_timeout", 3, publish=False)
            assert (
                math.hypot(*stopped["linear"][:2]) < 0.02
            ), "Watchdog did not stop translation"
            assert abs(stopped["angular"][2]) < 0.02, "Watchdog did not stop turning"
            report["passed"] = True
        except Exception as exc:
            report["error"] = str(exc)
            raise
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            (args.output / "result.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            (args.output / "samples.json").write_text(json.dumps(samples) + "\n")


if __name__ == "__main__":
    main()
