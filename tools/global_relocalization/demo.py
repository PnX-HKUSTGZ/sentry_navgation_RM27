#!/usr/bin/env python3
"""Run one visual competition demonstration against the already running server."""
import argparse, json, time, sys
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_srvs.srv import Trigger
from global_relocalization_interfaces.action import Relocalize

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node("visual_relocalization_demo")
action = ActionClient(node, Relocalize, "/global_relocalization/run")
trigger = node.create_client(Trigger, "/global_relocalization/start_match")
states = []


def status(m):
    data = json.loads(m.data)
    if not states or data["stage"] != states[-1]["stage"]:
        print("Stage:", data["stage"], data.get("reason", ""), flush=True)
    states.append(data)


node.create_subscription(String, "/global_relocalization/status", status, 100)
handle = None
complete = False
output = {"visualization_only": True}


def wait(future, timeout):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    if not future.done():
        raise TimeoutError("ROS request timed out")
    return future.result()


try:
    if not action.wait_for_server(timeout_sec=10) or not trigger.wait_for_service(
        timeout_sec=5
    ):
        raise RuntimeError("server unavailable")
    handle = wait(action.send_goal_async(Relocalize.Goal()), 5)
    if not handle.accepted:
        raise RuntimeError("another run is active")
    result = handle.get_result_async()
    armed = None
    triggered = False
    deadline = time.monotonic() + 140
    while not result.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if armed is None and states and states[-1]["stage"] == "ARMED":
            armed = time.monotonic()
            print(
                "Ready: rotating for five seconds so the prepared state is visible.",
                flush=True,
            )
        if armed is not None and not triggered and time.monotonic() - armed >= 5:
            triggered = True
            response = wait(trigger.call_async(Trigger.Request()), 5)
            print("Match start:", response.success, response.message, flush=True)
    response = wait(result, 3)
    complete = True
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    r = response.result
    pose = r.pose.pose
    output.update(
        success=r.success,
        reason=r.reason,
        elapsed_sec=r.elapsed_sec,
        fitness=r.fitness,
        inlier_ratio=r.inlier_ratio,
        pose={
            "frame": r.pose.header.frame_id,
            "position": [pose.position.x, pose.position.y, pose.position.z],
            "quaternion_xyzw": [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        },
        terminal_status=states[-1] if states else None,
    )
    print(
        json.dumps(
            {k: v for k, v in output.items() if k != "terminal_status"}, indent=2
        ),
        flush=True,
    )
except Exception as e:
    output["error"] = repr(e)
    print("Demo error:", repr(e), flush=True)
finally:
    if handle is not None and not complete:
        try:
            wait(handle.cancel_goal_async(), 5)
        except Exception:
            pass
    output["status_events"] = states
    args.output.write_text(json.dumps(output, indent=2) + "\n")
    node.destroy_node()
    rclpy.shutdown()

raise SystemExit(0 if output.get("success") else 1)
