import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from reset_pose import ResetPoseError, reset_pose, reset_summary

POSE = {'x': -1., 'y': 2., 'z': .08, 'yaw': 0.}


def result(stdout='data: true\n', code=0, stderr=''):
    return subprocess.CompletedProcess([], code, stdout, stderr)


class ResetTests(unittest.TestCase):
    def test_fixed_matrix_preserves_frozen_cells_and_denominator(self):
        root = Path(__file__).resolve().parent
        frozen = json.loads((root/'config/trials_left_physical_120.json').read_text())
        pilot = json.loads((root/'config/pilot_raw_frontend_fix_15.json').read_text())
        original = {row['trial_id']: row for row in frozen['trials']}
        self.assertEqual(len(pilot['trials']), 15)
        self.assertEqual(len({row['trial_id'] for row in pilot['trials']}), 15)
        for row in pilot['trials']:
            self.assertEqual(row, original[row['trial_id']])
        self.assertFalse(pilot['selection_manifest']['selection_by_new_localization_success'])

    @patch('reset_pose.subprocess.run')
    def test_cli_timeout_retries_identical_pose_and_preserves_attempt(self, run):
        run.side_effect = [result('Service call timed out\n', 1), result()]
        attempts = []
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / 'attempts.json'
            reset_pose(POSE, attempts=attempts, log_path=log)
            self.assertIn('cli_timeout', log.read_text())
        self.assertEqual(run.call_args_list[0], run.call_args_list[1])
        self.assertEqual([x['classification'] for x in attempts], ['cli_timeout', 'success'])
        summary = reset_summary([{'reset_attempts': attempts}])
        self.assertEqual(summary['first_attempt_success_count'], 0)
        self.assertEqual(summary['recovered_on_retry_count'], 1)
        self.assertTrue(all(x['started_utc'] and x['ended_utc'] and x['wall_sec'] >= 0 for x in attempts))

    @patch('reset_pose.subprocess.run')
    def test_outer_timeout_is_distinct_and_original_error_survives(self, run):
        run.side_effect = [subprocess.TimeoutExpired([], 15, output=b'original output', stderr=b'original stderr'),
                           result('second failure', 2)]
        with self.assertRaises(ResetPoseError) as raised:
            reset_pose(POSE)
        self.assertIn('original output', str(raised.exception))
        self.assertIn('original stderr', str(raised.exception))
        self.assertEqual(raised.exception.attempts[0]['classification'], 'outer_timeout')
        self.assertIsNone(raised.exception.attempts[0]['returncode'])
        self.assertEqual(run.call_count, 2)

    @patch('reset_pose.subprocess.run')
    def test_false_does_not_retry_even_if_timeout_mentioned(self, run):
        run.return_value = result('data: false\n', stderr='timeout text')
        with self.assertRaises(ResetPoseError) as raised:
            reset_pose(POSE)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(raised.exception.attempts[0]['classification'], 'boolean_false')

    @patch('reset_pose.subprocess.run')
    def test_two_timeouts_stop_at_two(self, run):
        run.side_effect = [result('Service call timed out', 1)] * 2
        with self.assertRaises(ResetPoseError) as raised:
            reset_pose(POSE)
        self.assertEqual(len(raised.exception.attempts), 2)
        self.assertEqual(run.call_count, 2)

    @patch('reset_pose.subprocess.run')
    def test_log_failure_retains_original_service_evidence(self, run):
        run.return_value = result('first failure', 3, 'stderr detail')
        attempts = []
        with patch.object(Path, 'write_text', side_effect=OSError('disk full')):
            with self.assertRaises(ResetPoseError) as raised:
                reset_pose(POSE, attempts=attempts, log_path='unused')
        self.assertIn('first failure', str(raised.exception))
        self.assertIn('stderr detail', str(raised.exception))
        self.assertIn('disk full', str(raised.exception))
        self.assertEqual(attempts[0]['returncode'], 3)
        self.assertEqual(run.call_count, 1)


if __name__ == '__main__':
    unittest.main()
