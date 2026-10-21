"""``common.idle``: gimbals locked with no thrust, surfaces held neutral
with no air, each written only on a change."""

import unittest
from types import SimpleNamespace

from common.idle import IdleGimbals, VacuumSurfaces


class Engine(SimpleNamespace):
    def __setattr__(self, name, value):
        if name == "gimbal_locked":
            self.__dict__.setdefault("writes", []).append(value)
        super().__setattr__(name, value)


def vessel(engines=(), surfaces=()):
    return SimpleNamespace(parts=SimpleNamespace(engines=list(engines),
                                                 control_surfaces=list(
                                                     surfaces)))


class TestIdleGimbals(unittest.TestCase):
    def test_locks_idle_frees_firing_and_writes_only_on_change(self):
        e = Engine(gimballed=True)
        fixed = Engine(gimballed=False)
        g = IdleGimbals(vessel([e, fixed]))
        g.update(False)
        g.update(False)
        g.update(True)
        self.assertEqual(e.writes, [True, False])
        self.assertFalse(hasattr(fixed, "writes"))


class TestVacuumSurfaces(unittest.TestCase):
    def surfaces(self):
        return [SimpleNamespace(deflection=0.4, deflection_override=False)
                for _ in range(2)]

    def test_held_in_vacuum_released_in_air_with_hysteresis(self):
        cs = self.surfaces()
        h = VacuumSurfaces(vessel(surfaces=cs), 10.0, 20.0)
        h.update(0.0)
        self.assertTrue(all(c.deflection_override for c in cs))
        self.assertTrue(all(c.deflection == VacuumSurfaces.NEUTRAL
                            for c in cs))
        h.update(15.0)                      # inside the band: still held
        self.assertTrue(h.held)
        h.update(25.0)
        self.assertFalse(any(c.deflection_override for c in cs))

    def test_suspended_while_the_pilot_uses_them(self):
        cs = self.surfaces()
        h = VacuumSurfaces(vessel(surfaces=cs), 10.0, 20.0)
        h.update(0.0)
        h.suspend("measuring")
        self.assertFalse(any(c.deflection_override for c in cs))
        h.update(0.0)
        self.assertFalse(h.held)
        h.resume()
        h.update(0.0)
        self.assertTrue(h.held)


if __name__ == "__main__":
    unittest.main()
