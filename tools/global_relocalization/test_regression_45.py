#!/usr/bin/env python3
import copy
import unittest
import json
from pathlib import Path
import tempfile
from regression_45 import round_passes, round_cells, changed_paths, digest

class RegressionAcceptance(unittest.TestCase):
    def setUp(self):
        self.ids = [f'cell_{i}' for i in range(15)]
        self.report = {'summary': {'batch': {'observed_trials': 15, 'success_count': 15}},
            'trials': [{'trial_id': cell, 'run_id': 'frozen', 'repetition': 2,
                'success': True, 'position_3d_error_m': .1,
                'orientation_geodesic_error_deg': 2.} for cell in self.ids]}

    def valid(self, report):
        return round_passes(report, 'frozen', 2, self.ids)

    def test_exact_thresholds_pass(self):
        self.assertTrue(self.valid(self.report))

    def test_duplicate_missing_and_old_scores_fail(self):
        for key, value in [('trial_id', 'cell_0'), ('run_id', 'old'), ('repetition', 1)]:
            report = copy.deepcopy(self.report)
            report['trials'][1][key] = value
            self.assertFalse(self.valid(report))

    def test_null_nonfinite_failed_and_outside_3d_so3_fail(self):
        for key, value in [('position_3d_error_m', None), ('position_3d_error_m', float('nan')),
                ('position_3d_error_m', .100001), ('orientation_geodesic_error_deg', 2.001), ('success', False)]:
            report = copy.deepcopy(self.report)
            report['trials'][4][key] = value
            self.assertFalse(self.valid(report))

    def test_missing_summary_keeps_executed_raw_and_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            (folder/'trials.jsonl').write_text(json.dumps({'trial_id':'cell_0','action':{'success':False}})+'\n'+json.dumps({'trial_id':'cell_1','action':{'success':True}})+'\n')
            (folder/'cell_2').mkdir()
            cells=round_cells(folder,{},'frozen',2,self.ids)
            self.assertEqual([c['state'] for c in cells[:4]], ['failed','executed_unscored','executed_unscored','unattempted'])
            self.assertEqual(sum(c['executed'] for c in cells),3)

    def test_frozen_helper_change_and_deletion_are_detected(self):
        with tempfile.TemporaryDirectory() as tmp:
            helper=Path(tmp)/'reset_pose.py';helper.write_text('original')
            frozen={str(helper):digest(helper)}
            self.assertEqual(changed_paths(frozen),[])
            helper.write_text('changed')
            self.assertEqual(changed_paths(frozen),[str(helper)])
            helper.unlink()
            self.assertEqual(changed_paths(frozen),[str(helper)])

    def test_wrong_identity_cannot_inflate_passed_cells(self):
        with tempfile.TemporaryDirectory() as tmp:
            report=copy.deepcopy(self.report)
            report['trials'][4]['run_id']='old'
            cells=round_cells(Path(tmp),report,'frozen',2,self.ids)
            self.assertEqual(sum(c['state']=='passed' for c in cells),14)
            self.assertEqual(cells[4]['state'],'failed')

if __name__ == '__main__':
    unittest.main()
