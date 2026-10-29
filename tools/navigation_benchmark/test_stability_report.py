import math
import unittest

from stability_report import lateral_motion


class LateralMotionTest(unittest.TestCase):
    def samples(self):
        return [dict(t=i * 0.05, vx=1., vy=0., cmd_vx=1., cmd_vy=0.,
                     active=True, goal_distance=2., yaw=0.) for i in range(20)]

    def test_straight_acceleration_is_not_lateral(self):
        samples = self.samples()
        for i, s in enumerate(samples):
            s['vx'] += i * 0.03
            s['cmd_vx'] = s['vx']
        self.assertEqual(lateral_motion(samples)['lateral_accel_rms'], 0.)

    def test_rotating_all_vectors_preserves_measure(self):
        samples = self.samples()
        for i, s in enumerate(samples):
            s['vy'] = s['cmd_vy'] = (-1)**i * 0.04
        original = lateral_motion(samples)
        self.assertGreater(original['lateral_accel_rms'], 1.)
        for s in samples:
            for x, y in [('vx', 'vy'), ('cmd_vx', 'cmd_vy')]:
                s[x], s[y] = (s[x]-s[y])/math.sqrt(2), (s[x]+s[y])/math.sqrt(2)
        self.assertAlmostEqual(original['lateral_accel_rms'], lateral_motion(samples)['lateral_accel_rms'])

    def test_gap_and_goal_approach_are_excluded(self):
        samples = self.samples()[:2]
        samples[1]['t'] = 1.
        self.assertIsNone(lateral_motion(samples)['lateral_accel_rms'])
        samples[1]['t'] = 0.05
        samples[1]['goal_distance'] = 0.1
        self.assertIsNone(lateral_motion(samples)['lateral_accel_rms'])

    def test_body_commands_rotate_to_world(self):
        samples = self.samples()
        for s in samples:
            s['yaw'] = s['t']
            s['cmd_vx'], s['cmd_vy'] = math.cos(s['yaw']), -math.sin(s['yaw'])
        self.assertAlmostEqual(lateral_motion(samples)['command_lateral_rate_rms'], 0.)

    def test_level_ground_filter_does_not_bridge_tilt_or_missing_pose(self):
        samples = self.samples()[:3]
        for s in samples:
            s.update(pitch=0., roll=0.)
        self.assertAlmostEqual(lateral_motion(samples, 0.06)['duration_s'], 0.1)
        samples[1]['roll'] = 0.1
        self.assertEqual(lateral_motion(samples, 0.06)['duration_s'], 0.)
        self.assertAlmostEqual(lateral_motion(samples)['duration_s'], 0.1)
        samples[1]['roll'] = 0.
        del samples[1]['pitch']
        self.assertEqual(lateral_motion(samples, 0.06)['duration_s'], 0.)
