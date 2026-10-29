import unittest

from metrics import analyze_run


def sample(t, **kwargs):
    result = dict(t=t, x=0, y=0, vx=0, vy=0, active=True, goal_distance=2)
    result.update(kwargs)
    return result


class MetricsTests(unittest.TestCase):
    def test_episodes_and_safety_evidence(self):
        samples = [sample(i / 10, region='ramp', safety_reason='obstacle' if i == 25 else '') for i in range(61)]
        result = analyze_run({'samples': samples})
        self.assertEqual(result['total_episodes'], 1)
        self.assertEqual(result['ramp_count'], 1)
        self.assertEqual(result['excluded_safety_count'], 0)
        self.assertLess(result['episodes'][-1]['safety_coverage'], 0.8)
        self.assertEqual(result['episodes'][-1]['duration'], 4)

    def test_gap_goal_and_missing_data_break_stops(self):
        samples = [sample(t) for t in (2, 2.25, 2.5, 2.75, 3, 4, 4.25, 4.5, 4.75, 5)]
        samples += [sample(5.25, goal_distance=0.1), sample(5.5, active=None)]
        result = analyze_run({'samples': samples}, {'start_grace': 0})
        self.assertEqual(result['total_episodes'], 2)
        self.assertEqual(result['unexpected_candidate_count'], 2)
        self.assertEqual(result['coverage']['missing_active'], 1)

    def test_position_fallback_and_region(self):
        samples = [sample(i / 4, vx=None, vy=None) for i in range(9)]
        result = analyze_run({'samples': samples}, {'start_grace': 0},
                             {'regions': [{'name': 'tunnel', 'xmin': -1, 'xmax': 1, 'ymin': -1, 'ymax': 1}]})
        self.assertEqual(result['tunnel_count'], 1)
        self.assertEqual(result['coverage']['missing_speed'], 1)
        self.assertEqual(result['episodes'][0]['safety_status'], 'unknown')

    def test_active_grace_exclusions_and_command(self):
        samples = [sample(i / 4, active=False) for i in range(8)]
        samples += [sample(i / 4, cmd_vx=0.3, cmd_vy=0, region='upper_ramp') for i in range(8, 25)]
        samples += [sample(i / 4, goal_distance=0.1) for i in range(25, 30)]
        result = analyze_run({'samples': samples})
        self.assertEqual(result['excluded_start_count'], 1)
        self.assertEqual(result['excluded_goal_count'], 1)
        self.assertEqual(result['total_episodes'], 1)
        self.assertEqual(result['ramp_count'], 1)
        middle = next(e for e in result['episodes'] if e['unexpected_candidate'])
        self.assertEqual(middle['max_cmd_speed'], 0.3)
        self.assertEqual(middle['start_position']['x'], 0)

    def test_safety_requires_duration_coverage(self):
        samples = [sample(i / 4, safety_reason='obstacle' if i < 9 else '') for i in range(11)]
        result = analyze_run({'samples': samples}, {'start_grace': 0})
        self.assertEqual(result['excluded_safety_count'], 1)
        self.assertAlmostEqual(result['episodes'][0]['safety_coverage'], 0.9)

    def test_roll_ramp_and_tunnel_overlap(self):
        samples = [sample(i / 4, roll=0.2, pitch=0) for i in range(9)]
        result = analyze_run({'samples': samples}, {'start_grace': 0},
                             {'ramp_pitch_threshold': 0.1, 'regions': [
                                 {'name': 'tunnel_1', 'xmin': -1, 'xmax': 1, 'ymin': -1, 'ymax': 1}]})
        self.assertEqual(result['ramp_count'], 1)
        self.assertEqual(result['tunnel_count'], 1)

    def test_order_is_validated(self):
        with self.assertRaises(ValueError):
            analyze_run({'samples': [sample(1), sample(1)]})


if __name__ == '__main__':
    unittest.main()
