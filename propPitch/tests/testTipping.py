"""The tip-over limit and the reaction-torque counter (propPitch/tipping.py)."""

import unittest

from propPitch import tipping
from propPitch.bladeModel import Rotor

# kRPC's vessel frame: x right, y nose, z down.  CoM at the origin.
UP = (0.0, 0.0, -1.0)
MAINS = (0.0, -1.0, 2.0)            # 1 m behind the CoM, 2 m below
NOSE = (0.0, 4.0, 2.0)
HIGH_HUB = (0.0, 2.0, -2.0)         # props 2 m above the CoM


def bounds(hub, thrust, margin, accel=None, contacts=(MAINS, NOSE)):
    """A 1 t plane on its wheels, braked only by its props (the measured
    deceleration is the thrust's)."""
    a = accel if accel is not None else (0.0, thrust / 1000.0, 0.0)
    return tipping.thrust_bounds(list(contacts), UP, 10.0, 1000.0, a,
                                 (0.0, thrust, 0.0), hub, thrust, margin)


class TestTipping(unittest.TestCase):
    def test_tail_over_by_hand(self):
        """Props 4 m above the ground, CoM 2 m: reverse thrust T lifts the
        nose about the mains by T (4 - 2) against gravity's 10 kN x 1 m, so
        it goes over at 5 kN; half the margin leaves 2.5 kN."""
        floor, ceiling = bounds(HIGH_HUB, -1000.0, 0.0)
        self.assertAlmostEqual(floor, -5000.0, delta=1.0)
        floor, _ = bounds(HIGH_HUB, -1000.0, 0.5)
        self.assertAlmostEqual(floor, -2500.0, delta=1.0)

    def test_same_bound_from_any_thrust_now(self):
        for t in (-4000.0, -100.0, 0.0):
            self.assertAlmostEqual(bounds(HIGH_HUB, t, 0.5)[0], -2500.0,
                                   delta=1.0)

    def test_nose_over_forward(self):
        """Forward thrust from the same props rolls the plane onto its nose:
        2 T against 10 kN x 4 m, so 20 kN."""
        _, ceiling = bounds(HIGH_HUB, 1000.0, 0.0)
        self.assertAlmostEqual(ceiling, 20000.0, delta=1.0)

    def test_low_props_nose_over(self):
        """Props below the CoM (1 m above the ground): reverse thrust pitches
        the nose down about the nose wheel by T (2 - 1) against 10 kN x 4
        m."""
        floor, _ = bounds((0.0, 2.0, 1.0), -1000.0, 0.0)
        self.assertAlmostEqual(floor, -40000.0, delta=1.0)

    def test_wheel_brakes_count(self):
        """Wheel brakes add deceleration at the CoM (2 m up): 0.5 g of it
        leaves the props more reverse thrust before the tail goes down."""
        a = (0.0, -1000.0 / 1000.0 - 5.0, 0.0)
        floor, _ = bounds(HIGH_HUB, -1000.0, 0.0, accel=a)
        # Inertia 5 kN x 2 m = 10 kN m more nose-down: 10 kN more thrust.
        self.assertAlmostEqual(floor, -10000.0, delta=1.0)

    def test_one_wheel_cannot_say(self):
        self.assertIsNone(bounds(HIGH_HUB, -1000.0, 0.5, contacts=(MAINS,)))
        self.assertIsNone(bounds(HIGH_HUB, -1000.0, 0.5,
                                 contacts=(MAINS, (1.0, -1.0, 2.0))))

    def test_contact_point(self):
        p = tipping.contact_point((-0.5, 1.0, 1.0), (0.5, 2.0, 2.0), UP)
        self.assertEqual(p, (0.0, 1.5, 2.0))


class TestCounter(unittest.TestCase):
    def test_pair_cancels(self):
        r = Rotor((0.0, 1.0, 0.0), [])
        t = tipping.reaction_torque([r, r], [5.0, 5.0], [1.0, -1.0])
        self.assertEqual(t, (0.0, 0.0, 0.0))

    def test_single_rotor(self):
        """Blades resisting a spin along +y turn the airframe along -y;
        the roll input that answers it makes +y torque (KSP applies -u)."""
        r = Rotor((0.0, 1.0, 0.0), [])
        t = tipping.reaction_torque([r], [5.0], [1.0])
        self.assertEqual(t, (0.0, -5.0, 0.0))
        u = tipping.counter_trim(t, ((10.0, 20.0, 10.0), (-10.0, -20.0, -10.0)))
        self.assertEqual(u, (0.0, -0.25, 0.0))
        self.assertEqual(tipping.reaction_torque([r], [5.0], [None]),
                         (0.0, 0.0, 0.0))

    def test_no_authority(self):
        u = tipping.counter_trim((0.0, -5.0, 0.0), ((0, 0, 0), (0, 0, 0)))
        self.assertEqual(u, (0.0, 0.0, 0.0))
        u = tipping.counter_trim((0.0, -50.0, 0.0), ((0, 1, 0), (0, -1, 0)))
        self.assertEqual(u, (0.0, -1.0, 0.0))


if __name__ == "__main__":
    unittest.main()
