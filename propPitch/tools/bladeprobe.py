#!/usr/bin/env python3
"""The pilot with its command replaced by a sweep, to check the blade model
against the game: the Deploy Angle steps through ``ANGLES`` (one step per
``STEP_S`` game seconds, the optimum between every two), and each log line
carries the measured ``J`` beside the model's thrust at the angle flown.
Across angles at one speed, ``J - model`` should not move (it is the
airframe's drag).  Fly it from ``propfly.py --probe``.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from common import paths                                # noqa: E402
paths.use_venv()
from propPitch import autopilot                         # noqa: E402

ANGLES = (40.0, 60.0, 70.0, 75.0, 80.0, 84.0, 87.0, 89.0)
STEP_S = 0.5


class ProbePilot(autopilot.PropPilot):
    def command(self, ut, opt, v_air, rho, sound):
        if opt is None:
            return None
        k = int((ut - self.start_ut) / STEP_S)
        if k % 2 == 0:
            return opt.deploy
        return ANGLES[(k // 2) % len(ANGLES)]


if __name__ == "__main__":
    sys.exit(autopilot.main(pilot_class=ProbePilot))
