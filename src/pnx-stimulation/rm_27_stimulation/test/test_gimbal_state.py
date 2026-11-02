import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gimbal_state import GimbalState, angle_error


class GimbalStateTest(unittest.TestCase):
    def test_rotation_rejected_while_folded(self):
        state = GimbalState()
        state.update(0, 0, 0, 0, .02)
        self.assertFalse(state.set_spin(True)[0])

    def test_fold_requires_actual_home_and_stopped_yaw(self):
        state = GimbalState()
        state.want_upright = True
        state.update(1.2, math.pi/2, 0, 0, .02)
        state.set_upright(False)
        for _ in range(100):
            _, roll = state.update(1.2, math.pi/2, 0, 0, .02)
            self.assertAlmostEqual(roll, math.pi/2)
        _, roll = state.update(0, math.pi/2, .2, 0, .02)
        self.assertAlmostEqual(roll, math.pi/2)
        _, roll = state.update(0, math.pi/2, 0, 0, .02)
        self.assertLess(roll, math.pi/2)

    def test_stop_returns_to_nearest_full_turn(self):
        state = GimbalState()
        state.want_upright = True
        yaw = 8*math.pi+.4
        state.update(yaw, math.pi/2, 0, 0, .02)
        self.assertTrue(state.set_spin(True)[0])
        state.set_spin(False)
        start = yaw
        for _ in range(500):
            yaw, roll = state.update(yaw, math.pi/2, 0, 0, .02)
        self.assertAlmostEqual(angle_error(yaw), 0, places=5)
        self.assertLess(abs(yaw-start), .5)
        self.assertAlmostEqual(roll, math.pi/2)

    def test_raise_rotate_fold_sequence(self):
        state = GimbalState()
        yaw = roll = yv = rv = 0.0
        state.set_upright(True)
        for _ in range(300):
            yn, rn = state.update(yaw, roll, yv, rv, .02)
            yv, rv = (yn-yaw)/.02, (rn-roll)/.02
            yaw, roll = yn, rn
        self.assertTrue(state.set_spin(True)[0])
        for _ in range(150):
            yaw, roll = state.update(yaw, roll, .3, 0, .02)
        self.assertGreater(yaw, .5)
        state.set_upright(False)
        previous_roll = roll
        for _ in range(700):
            yn, rn = state.update(yaw, roll, yv, rv, .02)
            if rn < previous_roll-1e-9:
                self.assertLess(abs(angle_error(yaw)), .025)
            yv, rv = (yn-yaw)/.02, (rn-roll)/.02
            yaw, roll, previous_roll = yn, rn, rn
        self.assertAlmostEqual(roll, 0)
        self.assertAlmostEqual(angle_error(yaw), 0)
        self.assertFalse(state.spinning)

    def test_stale_feedback_holds_without_losing_operator_intent(self):
        state = GimbalState()
        state.spinning = state.want_upright = True
        state.hold(.4, 1.2)
        self.assertEqual((state.yaw_ref, state.roll_ref), (.4, 1.2))
        self.assertTrue(state.spinning)
        state.hold(.4, 1.2, reset=True)
        self.assertFalse(state.spinning)
        self.assertFalse(state.want_upright)

    def test_invalid_parameters_and_feedback(self):
        with self.assertRaises(ValueError):
            GimbalState(spin_speed=float('nan'))
        with self.assertRaises(ValueError):
            GimbalState().update(float('nan'), 0, 0, 0, .02)


if __name__ == '__main__':
    unittest.main()
