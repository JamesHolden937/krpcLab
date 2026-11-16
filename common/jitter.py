"""How much a re-computed quantity jitters from one tick to the next.

A prediction re-propagated every tick moves by its own noise (a drag
estimate settling, a tick's worth of state) even when nothing real changed.
A deadband smaller than that acts on noise; one larger is a dead zone the
law cannot see into.  So deadbands are made of this: ``sd`` is the
tick-to-tick scatter, ``sqrt(mean(|d|^2) / 2)`` over the last ``SAMPLES``
differences (an exponential average), which is the sample's own noise when
the differences are independent.  None until ``SAMPLES`` differences are in:
an unknown noise is not a small one.
"""

import math

from common import vec

SAMPLES = 20            # differences the average spans


class Jitter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last = None
        self.mean_sq = None
        self.count = 0

    def update(self, value):
        """``value`` is a vector (tuple); returns :attr:`sd`."""
        if value is None:
            return self.sd
        value = tuple(value)
        if self.last is not None:
            d2 = vec.dot(vec.sub(value, self.last), vec.sub(value, self.last))
            w = 1.0 / min(SAMPLES, self.count + 1)
            self.mean_sq = d2 if self.mean_sq is None else (
                (1.0 - w) * self.mean_sq + w * d2)
            self.count += 1
        self.last = value
        return self.sd

    @property
    def sd(self):
        if self.count < SAMPLES or self.mean_sq is None:
            return None
        return math.sqrt(0.5 * self.mean_sq)

    def band(self, sigmas):
        """``sigmas`` standard deviations, or None while unmeasured."""
        sd = self.sd
        return None if sd is None else sigmas * sd
