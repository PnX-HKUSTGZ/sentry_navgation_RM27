"""Exercise the opt-in competition start service on the real ROS server."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from global_relocalization_interfaces.action import Relocalize
from test_action_contract import controlled_ros_clock, _test_node, _wait_for_clock


def test_early_start_duplicate_cancel_and_retry_never_publish_pose_or_navigate():
    binary = Path(os.environ['GLOBAL_RELOCALIZATION_NODE'])
    with tempfile.TemporaryDirectory() as temporary:
        path = Path(temporary) / 'map.pcd'
        points = [(x*.3, y*.3, z*.3) for x in range(5) for y in range(4) for z in range(3)]
        path.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n'
                        + ''.join(f'{x} {y} {z}\n' for x, y, z in points))
        process = subprocess.Popen([str(binary), '--ros-args', '-p', 'use_sim_time:=true',
            '-p', f'map_path:={path}', '-p', 'input_mode:=raw_imu', '-p', 'competition_mode:=true',
            '-p', 'prestart_timeout_sec:=8.0'], env=os.environ.copy(), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        context = rclpy.context.Context()
        rclpy.init(context=context)
        node = _test_node('competition_start_contract', context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        action = ActionClient(node, Relocalize, '/global_relocalization/run')
        trigger = node.create_client(Trigger, '/global_relocalization/start_match')
        commands, poses, states = [], [], []
        node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
        node.create_subscription(PoseStamped, '/global_relocalization/pose', poses.append, 10)
        node.create_subscription(String, '/global_relocalization/status', lambda m: states.append(json.loads(m.data)), 10)

        def wait(future):
            deadline = time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.005)
            assert future.done(), states[-2:]
            return future.result()

        def wait_state(stage, after=0, run_id=None, generation=None):
            # Action results and status use independent DDS channels. Arrival
            # of one never proves that the matching status callback has run.
            deadline = time.monotonic()+5
            while time.monotonic()<deadline:
                matching = [s for s in states[after:] if s['stage']==stage
                            and (run_id is None or s['run_id']==run_id)
                            and (generation is None or s['competition']['generation']==generation)]
                if matching:
                    return matching[-1]
                executor.spin_once(timeout_sec=.005)
            raise AssertionError({'expected_stage':stage, 'run_id':run_id,
                                  'generation':generation, 'states':states[-3:]})

        try:
            assert action.wait_for_server(timeout_sec=5)
            assert trigger.wait_for_service(timeout_sec=5)
            _wait_for_clock(node, executor)
            inactive = wait(trigger.call_async(Trigger.Request()))
            assert not inactive.success and 'no active' in inactive.message
            first_index = len(states)
            first = wait(action.send_goal_async(Relocalize.Goal()))
            assert first.accepted
            first_status = wait_state('SETTLING', after=first_index)
            first_run = first_status['run_id']
            first_generation = first_status['competition']['generation']
            early = trigger.call_async(Trigger.Request())
            repeated = trigger.call_async(Trigger.Request())
            assert not wait(early).success
            repeat = wait(repeated)
            assert not repeat.success
            assert 'already latched' in repeat.message or 'no active' in repeat.message
            failed = wait(first.get_result_async())
            assert not failed.result.success and 'before preparation' in failed.result.reason
            terminal = wait_state('FAILED', run_id=first_run, generation=first_generation)
            assert terminal['competition']['start_latched']
            assert not terminal['competition']['prepared']
            assert not terminal['competition']['release_control']
            previous_generation = terminal['competition']['generation']
            second_index = len(states)
            second = wait(action.send_goal_async(Relocalize.Goal()))
            assert second.accepted
            second_status = wait_state('SETTLING', after=second_index)
            second_run = second_status['run_id']
            second_generation = second_status['competition']['generation']
            assert wait(second.cancel_goal_async()).goals_canceling
            canceled = wait(second.get_result_async())
            assert canceled.status == 5 and not canceled.result.success
            latest = wait_state('CANCELED', run_id=second_run, generation=second_generation)
            assert latest['competition']['generation'] > previous_generation
            assert not latest['competition']['start_latched']
            assert not latest['competition']['prepared']
            assert not latest['competition']['release_control']
            third_index = len(states)
            third = wait(action.send_goal_async(Relocalize.Goal()))
            assert third.accepted
            third_status = wait_state('SETTLING', after=third_index)
            assert not wait(trigger.call_async(Trigger.Request())).success
            assert not wait(third.get_result_async()).result.success
            wait_state('FAILED', run_id=third_status['run_id'],
                       generation=third_status['competition']['generation'])
            assert commands and all(c.angular.z==0 and c.linear.x==0 for c in commands)
            assert not poses
            assert not node.get_publishers_info_by_topic('/initialpose')
            assert not node.get_publishers_info_by_topic('/goal_pose')
            assert not node.get_publishers_info_by_topic('/navigate_to_pose')
        finally:
            action.destroy()
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            context.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            output = process.communicate(timeout=5)[0]
            assert process.returncode==0, output
