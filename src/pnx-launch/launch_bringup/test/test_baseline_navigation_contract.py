"""Freeze the user-requested 774bc1a normal navigation/physics contract.

Only the three initialization/rendering metadata blocks may extend the profiles.
The golden digests come from that commit, not from the implementation under test.
"""
import hashlib
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = Path(__file__).resolve().parents[1]
BASELINE = json.loads((PACKAGE / 'test/data/baseline_774bc1a.json').read_text())


@pytest.mark.parametrize('profile,expected', BASELINE['profiles'].items())
def test_normal_navigation_parameters_match_774bc1a(profile, expected):
    actual = yaml.safe_load((PACKAGE / 'config' / profile).read_text())
    extra = set(actual) - set(expected)
    assert extra <= {'global_relocalization', 'relocalize_then_navigate', 'simulation_rendering'}
    for node, digest in expected.items():
        encoded = json.dumps(actual[node], sort_keys=True, separators=(',', ':')).encode()
        assert hashlib.sha256(encoded).hexdigest() == digest, f'{profile}: {node} differs from 774bc1a'


@pytest.mark.parametrize('relative,digest', BASELINE['sources'].items())
def test_normal_navigation_and_physics_sources_match_774bc1a(relative, digest):
    assert hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() == digest, relative


def test_normal_localization_methods_match_774bc1a():
    source = (ROOT / 'src/pnx-localization/relocalization_manager/src/relocalization_manager.cpp').read_text()
    regions = {
        'before_constructor': source.split('RelocalizationManagerNode::RelocalizationManagerNode(')[0],
        'after_initialization': source[source.index('void RelocalizationManagerNode::publishPriorPcdMap('):],
    }
    for name, digest in BASELINE['localization_methods'].items():
        assert hashlib.sha256(regions[name].encode()).hexdigest() == digest, name


def test_normal_robot_expansion_matches_774bc1a():
    import xml.etree.ElementTree as ET
    import xacro
    from ament_index_python.packages import get_package_share_directory
    source = ROOT / 'src/pnx-stimulation/rm_27_stimulation/urdf/simulation_waking_robot.xacro'
    tree = ET.fromstring(xacro.process_file(str(source)).toxml())
    canonical = ET.tostring(tree, encoding='unicode').replace(
        get_package_share_directory('rm_27_stimulation'), '<simulation_share>')
    assert hashlib.sha256(canonical.encode()).hexdigest() == BASELINE['normal_robot_urdf_sha256']
