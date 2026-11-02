"""Global initialization retains one map-to-odom owner in every mode."""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext

BRINGUP_DIR = Path(__file__).resolve().parents[1]


def load_launch(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("truth,global_match,owner", [
    ("false", "true", "relocalization_manager_node"),
    ("false", "false", "relocalization_manager_node"),
    ("true", "true", None),
    ("true", "false", None),
    ("1", "true", None),
])
def test_global_handoff_has_one_map_odom_owner(truth, global_match, owner):
    module = load_launch(BRINGUP_DIR / "launch/localization_launch.py", "rm27_localization")
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(use_ground_truth_odom=truth,
                                         global_relocalization=global_match,
                                         use_composition="false")
    nodes = []
    for entity in description.entities:
        if hasattr(entity, "node_package"):
            nodes.append(entity)
        elif hasattr(entity, "get_sub_entities"):
            nodes.extend(e for e in entity.get_sub_entities() if hasattr(e, "node_package"))
    active = {n.node_executable for n in nodes
              if n.node_executable in {"global_pose_handoff.py", "relocalization_manager_node"}
              and n.condition.evaluate(context)}
    assert active == ({owner} if owner else set())
    pointlio = next(n for n in nodes if n.node_executable == "pointlio_mapping")
    assert pointlio.condition.evaluate(context) == (truth == "false")
