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
        return self

    def tick(self, phase):
        """The loop is starting a tick of ``phase``."""
        self.phase = phase
        self.ticks[phase] = self.ticks.get(phase, 0) + 1

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
