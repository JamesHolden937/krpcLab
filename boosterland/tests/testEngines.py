"""Throttle allocation: center first, then balanced pairs."""

import math
import unittest

from boosterland.engines import allocate, group_engines


def ring(n, radius, start=0, thrust=1.0, phase=0.0):
    return [(start + i, (radius * math.cos(phase + 2 * math.pi * i / n),
                         radius * math.sin(phase + 2 * math.pi * i / n)),
             thrust) for i in range(n)]


class TestGrouping(unittest.TestCase):

    def test_five_engine_plate_is_center_then_two_pairs(self):
        engines = [(0, (0.0, 0.0), 1.0)] + ring(4, 1.2, start=1)
        groups = group_engines(engines)
        self.assertEqual(groups[0], [0])
        self.assertEqual(len(groups), 3)
        for g in groups[1:]:
            self.assertEqual(len(g), 2)
            a, b = (engines[k][1] for k in g)
            self.assertAlmostEqual(a[0] + b[0], 0.0, places=6)
            self.assertAlmostEqual(a[1] + b[1], 0.0, places=6)

    def test_a_single_engine_is_one_group(self):
        self.assertEqual(group_engines([(7, (0.0, 0.0), 1.0)]), [[7]])

    def test_an_odd_ring_lights_together(self):
        groups = group_engines(ring(3, 1.0))
        self.assertEqual([sorted(g) for g in groups], [[0, 1, 2]])

    def test_offsets_are_about_the_cluster_not_the_vessel(self):
        shifted = [(k, (p[0] + 5.0, p[1] - 2.0), t)
                   for k, p, t in [(0, (0.0, 0.0), 1.0)] + ring(4, 1.2, 1)]
        self.assertEqual(group_engines(shifted)[0], [0])

    def test_inner_ring_before_outer(self):
        engines = ring(2, 0.8, start=0) + ring(2, 2.0, start=2, phase=0.3)
        groups = group_engines(engines)
        self.assertEqual(sorted(groups[0]), [0, 1])
        self.assertEqual(sorted(groups[1]), [2, 3])

    def test_no_center_engine_means_no_center_group(self):
        groups = group_engines(ring(4, 1.0))
        self.assertEqual(len(groups), 2)


class TestAllocation(unittest.TestCase):

    def setUp(self):
        self.groups = [[0], [1, 3], [2, 4]]
        self.thrust = {k: 1.0 for k in range(5)}

    def test_low_demand_is_the_center_engine_alone(self):
        limits = allocate(0.1, self.groups, self.thrust)
        self.assertAlmostEqual(limits[0], 0.5)
        self.assertEqual([limits[k] for k in (1, 2, 3, 4)], [0.0] * 4)

    def test_center_full_before_a_pair_lights(self):
        limits = allocate(0.4, self.groups, self.thrust)
        self.assertAlmostEqual(limits[0], 1.0)
        self.assertAlmostEqual(limits[1], 0.5)
        self.assertAlmostEqual(limits[3], 0.5)
        self.assertEqual(limits[2], 0.0)

    def test_the_total_is_the_demand(self):
        for demand in (0.0, 0.13, 0.5, 0.77, 1.0):
            limits = allocate(demand, self.groups, self.thrust)
            self.assertAlmostEqual(sum(limits.values()) / 5.0, demand)

    def test_full_demand_is_everything(self):
        limits = allocate(1.0, self.groups, self.thrust)
        self.assertEqual(set(limits.values()), {1.0})


if __name__ == "__main__":
    unittest.main()
