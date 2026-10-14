import unittest
from receipt_metrics import ReceiptMetrics


class ReceiptTests(unittest.TestCase):
    def test_run_gaps_and_counts_do_not_leak_across_trials(self):
        metrics = ReceiptMetrics()
        metrics.status('1')
        metrics.receive(1000000000, 1.)
        metrics.receive(2000000000, 2.)
        self.assertEqual(metrics.status('1')['source_receipt']['max_stamp_gap_sec'], 1.)
        self.assertEqual(metrics.status('2')['source_receipt']['lidar_count'], 0)
        metrics.receive(9000000000, 9.)
        metrics.receive(9100000000, 9.1)
        row = metrics.status('2')
        self.assertEqual(row['source_receipt']['lidar_count'], 2)
        self.assertAlmostEqual(row['source_receipt']['max_stamp_gap_sec'], .1)
        self.assertAlmostEqual(row['source_receipt']['max_callback_gap_sec'], .1)
        self.assertEqual(row['session_source_receipt']['lidar_count'], 4)
        self.assertEqual(row['session_source_receipt']['max_callback_gap_sec'], 7.)
        metrics.receive(9200000000, 9.2)
        self.assertEqual(row['source_receipt']['lidar_count'], 2)


if __name__ == '__main__':
    unittest.main()
