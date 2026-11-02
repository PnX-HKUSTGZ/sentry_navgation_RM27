import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.events.process import ProcessExited
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument) and action.default_value is not None:
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    context.launch_configurations.update(overrides)
    return context


class SimulationStartup(unittest.TestCase):
    def test_world_view_and_explicit_override(self):
        module = load('rm_simulation')
        context = context_for(module, sim_world='RMUL2027')
        self.assertTrue(module._resolve_gui_config(context).endswith('/config/rmul2027_gui.config'))
        context.launch_configurations['sim_world'] = 'RMUC2026'
        self.assertTrue(module._resolve_gui_config(context).endswith('/config/gazebo_gui.config'))
        context.launch_configurations['gui_config'] = '/tmp/custom_view.config'
        self.assertEqual(module._resolve_gui_config(context), '/tmp/custom_view.config')

    def test_robot_and_navigation_require_successful_world_clock(self):
        module = load('sim_with_nav')
        context = context_for(module, world='RMUL2027')
        actions = module._launch_sim_stack(context)
        # Only the world and clock waiter start before the readiness event.
        self.assertEqual(sum(isinstance(a, IncludeLaunchDescription) for a in actions), 1)
        self.assertFalse(any(isinstance(a, TimerAction) for a in actions))
        clock = next(a for a in actions if isinstance(a, Node))
        handler = next(a.event_handler for a in actions if isinstance(a, RegisterEventHandler))
        def event(code):
            return ProcessExited(action=clock, name='clock', cmd=[], cwd=None,
                                 env=None, pid=1, returncode=code)
        self.assertTrue(handler.matches(event(0)))
        ready = handler.handle(event(0), context)
        timer = next(a for a in ready if isinstance(a, TimerAction))
        self.assertTrue(any(isinstance(a, IncludeLaunchDescription) for a in timer.actions))
        self.assertTrue(any(isinstance(a, TimerAction) for a in timer.actions))
        failed = handler.handle(event(1), context)
        self.assertEqual(len(failed), 1)
        self.assertIsInstance(failed[0], EmitEvent)

    def test_existing_world_keeps_its_startup_policy(self):
        module = load('sim_with_nav')
        actions = module._launch_sim_stack(context_for(module, world='RMUC2026'))
        self.assertEqual(sum(isinstance(a, IncludeLaunchDescription) for a in actions), 2)
        self.assertTrue(any(isinstance(a, TimerAction) for a in actions))


if __name__ == '__main__':
    unittest.main()
