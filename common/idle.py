"""Actuators held still while they can do nothing: engine gimbals with no
thrust, control surfaces with no air.

Neither changes a flight -- a gimbal makes no torque without thrust and a
surface none without dynamic pressure -- but a vehicle whose fins flap and
whose nozzles swing in vacuum looks broken, and an inactive engine's gimbal
also throws inside kRPC's torque total (boosterland failure 7).  Both write
only on a change, so a held state costs nothing per tick.
"""


class IdleGimbals:
    """Lock every gimballed engine while nothing is firing; free them the
    tick something is.  ``update(firing)`` with the pilot's own idea of
    "firing" -- the throttle it commanded, or the thrust it reads when the
    throttle is not its own (MechJeb's ascent)."""

    def __init__(self, vessel, log=None):
        self.vessel = vessel
        self.log = log            # callable(text) or None
        self.engines = None
        self.locked = None

    def rescan(self):
        """Forget the engine list (after staging)."""
        self.engines = None
        self.locked = None

    def update(self, firing):
        want = not firing
        if want == self.locked:
            return
        self.engines = None       # re-listed on every flip: staging moves it
        if self.engines is None:
            try:
                self.engines = [e for e in self.vessel.parts.engines
                                if e.gimballed]
            except Exception:                           # noqa: BLE001
                self.engines = []
        done = 0
        for engine in self.engines:
            try:
                engine.gimbal_locked = want
                done += 1
            except Exception:                           # noqa: BLE001
                continue
        self.locked = want
        if self.log is not None and self.engines:
            self.log("gimbals %s (%d of %d engine(s))"
                     % ("locked: no thrust" if want else "free: firing",
                        done, len(self.engines)))


class VacuumSurfaces:
    """Hold every control surface at neutral below ``on_q`` Pa of dynamic
    pressure and hand them back above ``off_q`` (hysteresis), unless the
    pilot has suspended it -- while it measures or flies the surfaces
    itself.  ``update(q)`` each tick.

    Neutral is a hair off zero: a deflection of exactly 0 freezes the
    surface under AtmosphereAutopilot (``spaceplane`` ``canard_trim``)."""

    NEUTRAL = 0.003

    def __init__(self, vessel, on_q, off_q, log=None):
        self.vessel = vessel
        self.on_q = float(on_q)
        self.off_q = float(off_q)
        self.log = log
        self.held = False
        self.suspended = False

    def _set(self, hold):
        try:
            for cs in self.vessel.parts.control_surfaces:
                cs.deflection_override = hold
                if hold:
                    cs.deflection = self.NEUTRAL
        except Exception:                               # noqa: BLE001
            return False
        return True

    def update(self, q):
        if self.suspended:
            return
        if not self.held and q < self.on_q:
            if self._set(True):
                self.held = True
                if self.log is not None:
                    self.log("control surfaces held neutral: no air "
                             "(q %.1f Pa)" % q)
        elif self.held and q > self.off_q:
            self.release("air (q %.1f Pa)" % q)

    def release(self, why="released"):
        if self.held and self._set(False) and self.log is not None:
            self.log("control surfaces handed back: %s" % why)
        self.held = False

    def suspend(self, why):
        """Hand the surfaces back and keep them back until ``resume``."""
        self.release(why)
        self.suspended = True

    def resume(self):
        self.suspended = False
