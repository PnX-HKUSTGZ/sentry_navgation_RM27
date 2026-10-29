import unittest

from batch import normalize_goals


class GoalInputTests(unittest.TestCase):
    def test_integer_coordinates_become_ros_floats(self):
        goals = normalize_goals('[[-2,-4.4],[2,-4.2]]')
        self.assertEqual(goals, [[-2.0, -4.4], [2.0, -4.2]])
        self.assertTrue(all(type(v) is float for g in goals for v in g))

    def test_invalid_coordinates_fail_before_launch(self):
        for raw in ('[]', '[1,2]', '[[1]]', '[[true,2]]', '[["1",2]]',
                    '[[NaN,2]]', '[[Infinity,2]]', '{}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                normalize_goals(raw)


if __name__ == '__main__':
    unittest.main()
