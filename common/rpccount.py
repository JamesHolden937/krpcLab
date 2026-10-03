"""How many kRPC round trips a tick costs, and which procedures they are.

The farm's time scale is ``interval / busy`` and ``busy`` is now mostly round
trips, not maths (PyPy made the propagation cheap).  A round trip costs at
least one game frame of the server's attention, more when the farm is loaded,
so the first thing to know about a slow tick is how many calls it makes and
which -- a read that could be a stream, or a write that did not change.

``RpcCounter.install(conn)`` wraps the client's single synchronous entry point
(``Client._invoke``; streams do not go through it) and attributes every call
to whatever ``phase`` the loop last set.  ``report()`` is one log line.
"""

import time


class RpcCounter:
    def __init__(self, top=6):
        self.phase = None
        self.top = top
        self.calls = {}         # phase -> {procedure: [count, wall]}
        self.ticks = {}         # phase -> ticks
        self.span = {}          # phase -> [wall s, game s] from tick to tick
        self._last = None       # (phase, monotonic, ut) of the last tick

    def install(self, conn):
        inner = conn._invoke

        def invoke(service, procedure, *rest):
            started = time.monotonic()
            try:
                return inner(service, procedure, *rest)
            finally:
                row = self.calls.setdefault(self.phase, {}).setdefault(
                    procedure, [0, 0.0])
                row[0] += 1
                row[1] += time.monotonic() - started
        conn._invoke = invoke
        conn._rpc_counter = self
        return self

    def batched(self, calls):
        """One round trip carrying ``calls`` (``common.krpcbatch``)."""
        row = self.calls.setdefault(self.phase, {}).setdefault(
            "batch%d" % len(calls), [0, 0.0])
        row[0] += 1

    def tick(self, phase, ut=None):
        """The loop is starting a tick of ``phase`` (``ut``: the last known)."""
        now = time.monotonic()
        if self._last is not None:
            was, then, then_ut = self._last
            row = self.span.setdefault(was, [0.0, 0.0])
            row[0] += now - then
            if ut is not None and then_ut is not None and ut >= then_ut:
                row[1] += ut - then_ut
        self._last = (phase, now, ut)
        self.phase = phase
        self.ticks[phase] = self.ticks.get(phase, 0) + 1

    def wall_report(self):
        """Where the flight's wall-clock went: the farm's real cost per phase."""
        total = sum(r[0] for r in self.span.values())
        return "wall per phase (wall s / game s = speed): total %.0fs " % total + " ".join(
            "%s %.0f/%.0f=%.1fx" % (p, r[0], r[1], r[1] / r[0] if r[0] else 0.0)
            for p, r in self.span.items())

    def report(self):
        bits = []
        for phase, procs in self.calls.items():
            n = max(1, self.ticks.get(phase, 0))
            count = sum(r[0] for r in procs.values())
            wall = sum(r[1] for r in procs.values())
            worst = sorted(procs.items(), key=lambda kv: -kv[1][0])[:self.top]
            bits.append("%s %.1f/tick %.1fms [%s]" % (
                phase, count / n, 1000.0 * wall / n,
                " ".join("%s:%.1f" % (p, r[0] / n) for p, r in worst)))
        return "rpc per tick (calls, wall in rpc): " + " | ".join(bits)
