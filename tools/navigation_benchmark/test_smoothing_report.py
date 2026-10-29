import math
import unittest
from smoothing_report import geometry, summarize


class SmoothingReportTest(unittest.TestCase):
    def test_straight_line_and_duplicates(self):
        result = geometry([[0, 0], [1, 0], [1, 0], [2, 0]])
        self.assertEqual(result['length_m'], 2)
        self.assertEqual(result['bending_per_m'], 0)

    def test_right_angle(self):
        result = geometry([[0, 0], [1, 0], [1, 1]])
        self.assertAlmostEqual(result['max_turn_deg'], 90)
        self.assertAlmostEqual(result['bending_per_m'], math.pi**2 / 8)

    def test_failure_and_missing_evidence_are_preserved(self):
        result = summarize({'status': 'timeout_or_clock_reset', 'goal': [1, 1],
                            'samples': [], 'start_sim_time': 0, 'end_sim_time': 90})
        self.assertEqual(result['status'], 'timeout_or_clock_reset')
        self.assertIsNone(result['first_global_path'])
        self.assertIsNone(result['speed_rate_rms_m_s2'])
        self.assertIsNone(result['reference_jerk_peak_m_s3'])
