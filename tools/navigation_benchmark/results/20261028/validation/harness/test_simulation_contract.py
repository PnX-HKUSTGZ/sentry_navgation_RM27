"""Cross-check simulation safety parameters against the physical URDF."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


class SimulationContractTests(unittest.TestCase):
    def test_height_and_filter_frames_match_physical_robot(self):
        root = Path(__file__).resolve().parents[2]
        model = ET.parse(root / 'src/rm_27_stimulation/urdf/simulation_waking_robot.xacro').getroot()
        properties = {x.attrib['name']: x.attrib['value'] for x in model
                      if x.tag.endswith('}property')}
        links = {x.attrib['name'] for x in model.findall('link')}
        config = yaml.safe_load((root / 'src/pb2025_nav_bringup/config/simulation/minco_params.yaml').read_text())
        rog = config['planner_server']['ros__parameters']['MincoPlanner']['rog_map']
        self.assertGreaterEqual(rog['projection']['vehicle_height'],
                                float(properties['folded_vehicle_height']))
        # An absent filter frame makes the fail-closed filter reject every scan.
        crop = rog['cloud_filter']
        self.assertTrue(set(crop['box_frames']).issubset(links))
        self.assertIn(rog['projection']['robot_pose_frame'], links)


if __name__ == '__main__':
    unittest.main()
