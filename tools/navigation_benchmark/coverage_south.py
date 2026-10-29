#!/usr/bin/env python3
"""Check candidate south crossing endpoints against the actual collision meshes."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/global_relocalization'))
from actual_collision_support import CollisionSupportScene, robot_collision_envelope
try:
    from .geometry_audit import WORLD
except ImportError:
    from geometry_audit import WORLD


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-urdf', required=True, type=Path,
                        help='expanded current simulation URDF, with fixed collision joints')
    args = parser.parse_args()
    scene = CollisionSupportScene.from_world(WORLD)
    envelope = robot_collision_envelope(args.robot_urdf)
    checks = []
    for xy in [(-2., -4.4), (2., -4.2)]:
        accepted, height, reason = scene.check_position(xy, envelope)
        checks.append({'xy': xy, 'accepted': accepted, 'support_height_m': height if accepted else None, 'reason': reason})
    print(json.dumps({'goals': [[-2., -4.4], [2., -4.2], [-2., -4.4]], 'endpoint_checks': checks,
        'robot_envelope': envelope, 'scene_source_sha256': scene.source_sha256,
        'limitations': ['Endpoint screening assumes level stationary poses and conservative full-turn envelope.',
                        'Intermediate slope and actual crossing are not certified. Coverage requires observed trajectory.']}, indent=2))
    if not all(c['accepted'] for c in checks):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
