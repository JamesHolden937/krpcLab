"""The planner: the cheapest transfer that finishes within a time budget.

The guidance's effectivity cutoff trades time for dv: near 1 it burns only
in short arcs at the apsides (the cheapest, and the slowest); lower, it
burns over more of each orbit.  The planner flies the transfer in the
stand-in over a grid of cutoffs and takes the highest (cheapest) whose
flight meets the deadline.  A scan rather than a bisection, so a time that
is not monotonic in the cutoff cannot mislead it.

The propagation *is* the guidance flown forward -- the same
``guidance.command`` the vessel will fly -- so its dv and time predict a
law somebody actually flies.
"""

import math
from collections import namedtuple

from lowThrust import sim

Plan = namedtuple("Plan", "feasible eta result reason table")


def plan(body, vessel, r, v, target, cfg, within_s=None):
    """The cheapest transfer arriving within ``within_s`` (None: no deadline).

    ``table`` is every cutoff flown, as (eta, done, time, dv), so a deadline
    that cannot be met can say what is possible.
    """
    def fly(eta, record=False):
        return sim.fly(body, vessel, r, v, target, eta, cfg, record=record)

    if within_s is None:
        res = fly(cfg.ETA_DEFAULT, record=True)
        return Plan(res.done, cfg.ETA_DEFAULT, res, res.reason, [])
    # spaced evenly in arc half-width (acos eta), where the cost lives: the
    # useful range is crowded near eta = 1 (0.99 is +-8 deg, 0.95 is +-18)
    n = cfg.PLAN_GRID
    w_lo, w_hi = math.acos(cfg.ETA_MAX), math.acos(cfg.ETA_MIN)
    table = []
    for k in range(n):
        eta = math.cos(w_lo + (w_hi - w_lo) * k / (n - 1))
        res = fly(eta)
        table.append((eta, res.done, res.time, res.dv))
    ok = [row for row in table if row[1] and row[2] <= within_s]
    if ok:
        eta = max(ok)[0]
        return Plan(True, eta, fly(eta, record=True), "arrives in time", table)
    done = [row for row in table if row[1]]
    if not done:
        return Plan(False, table[0][0], fly(table[0][0], record=True),
                    "no cutoff arrives", table)
    eta = min(done, key=lambda row: row[2])[0]
    return Plan(False, eta, fly(eta, record=True),
                "late: the deadline is shorter than the fastest transfer",
                table)
