import unittest

from audit_coverage import crossings


class CrossingTests(unittest.TestCase):
    region = {'xmin': 0, 'xmax': 1, 'ymin': -1, 'ymax': 1}

    def samples(self, points):
        return [dict(t=i * .1, x=x, y=y) for i, (x, y) in enumerate(points)]

    def test_observed_crossing_in_both_directions(self):
        samples = self.samples([(-1, 0), (.5, 0), (2, 0), (.5, 0), (-1, 0)])
        self.assertEqual([r['direction'] for r in crossings(samples, self.region)],
                         ['west_to_east', 'east_to_west'])

    def test_touching_region_then_detouring_is_not_a_crossing(self):
        samples = self.samples([(-1, 0), (.5, 0), (.5, 2), (2, 2), (2, 0)])
        self.assertEqual(crossings(samples, self.region), [])

    def test_gap_does_not_certify_a_crossing(self):
        samples = self.samples([(-1, 0), (.5, 0), (2, 0)])
        samples[-1]['t'] = 1.0
        self.assertEqual(crossings(samples, self.region), [])


if __name__ == '__main__':
    unittest.main()
