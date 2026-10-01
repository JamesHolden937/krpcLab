#!/usr/bin/env python3
"""How well does the simulator predict a recorded game flight?

    ./kspSim/tools/fidelity.py logs/kspsim/ft/g_qs_cone_pitch.jsonl [--model qs_cone]

Replays a ``flighttest.py`` recording (or any recording with the same row
format) through ``kspSim.world`` in-process, two ways:

**Forces at the game's state** (one-step).  At every recorded frame the
simulator's vessel is put in the game's exact state and asked for its
aerodynamic force, which is compared with the force the game reported that
frame (``Flight.aerodynamic_force``), and for its total torque, compared
with the torque the game's angular acceleration implies (``I dw/dt + w x Iw``
from the recorded rates).  No integration, no drift: a wrong table shows up
here and nowhere else looks cleaner.

**Prediction from restarts** (multi-step).  From a restart every
``--every`` seconds, the simulator starts in the game's state and flies the
game's own recorded control inputs for each horizon; the error against the
game's state at the same UT is reported per horizon -- position, velocity,
attitude, rate.  This is what an autopilot flying the simulator would
actually feel.

The actuators have state the recording does not show (surface deflection,
gear), so a shadow of them is run along the recorded inputs from the start
of the recording and handed to each restart.
"""
import argparse
import json
import math
import os
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from kspSim import quat, vec  # noqa: E402
from kspSim import world as W  # noqa: E402

AXES = ("pitch", "yaw", "roll", "throttle", "forward", "up", "right")


def read(path):
    rows = [json.loads(line) for line in open(path)]
    return rows[0], rows[1:]


class Replay:
    def __init__(self, model_name, header, rows):
        path = os.path.join(W.MODELS, model_name + ".json")
        self.model = json.load(open(path))
        self.header = header
        self.rows = rows
        self.world = W.World()
        self.world.t = self.model["ut"]
        self.world.body = W.Body(self.model)
        self.world.vessel = W.Vessel(self.model, self.world)
        self.world.loaded = True
        self.world.save = model_name
        self.v = self.world.vessel
        self.amounts0 = [[r.amount for r in p.resources] for p in self.v.parts]
        self._discrete()
        self._shadow()

    # -- the inputs -------------------------------------------------------

    def _discrete(self):
        """gear / brakes / other switches over time, from the script."""
        self.switches = []
        for t, ev in self.header.get("events", []):
            sw = {k: val for k, val in ev.items() if k not in AXES}
            if sw:
                self.switches.append((t, sw))

    def switch_state(self, t):
        st = self.model["state"]
        h = self.header
        s = {"gear": bool(h.get("gear", st.get("gear"))),
             "brakes": bool(h.get("brakes", st.get("brakes"))),
             "rcs": bool(h.get("rcs", st.get("rcs")))}
        for te, sw in self.switches:
            if te <= t:
                s.update(sw)
        return s

    def actuators(self):
        """The vessel's actuator state: what the recording cannot show."""
        v = self.v
        return (list(v.eff), list(v.deflections), v.gear_deploy,
                [list(x) for x in v.wheel_input], [list(e.actuation) for e in v.engines],
                {k: (b["deploy"], list(b["ignore"]), b["angle"]) for k, b in v.airbrakes.items()})

    def set_actuators(self, st):
        v = self.v
        eff, defl, gear, wheels, gimbals, brakes = st
        for k, (dep, ign, ang) in brakes.items():
            b = v.airbrakes[k]
            b["deploy"], b["ignore"], b["angle"] = dep, list(ign), ang
        v.eff = list(eff)
        v.deflections = list(defl)
        v.gear_deploy = gear
        v.wheel_input = [list(x) for x in wheels]
        for e, a in zip(v.engines, gimbals):
            e.actuation = list(a)

    def _shadow(self):
        """One pass along the recording, the vessel put in the game's state
        at every frame and stepped once with the frame's input: the
        actuators evolve by the simulator's own laws under the recorded
        inputs, and each frame's (sim wrench, game wrench) comes out of the
        same pass."""
        v = self.v
        self.shadow = []
        self.frames = []
        body = self.world.body
        rows = self.rows
        st = self.actuators()
        groups = [False] * 10
        brakes = False
        for i, row in enumerate(rows):
            self._kinematics(i)
            self.set_actuators(st)
            # Recorded switches: fire what changed (a group already on at the
            # first frame was fired before the recording began).
            for g, on in enumerate(row.get("ag") or []):
                if on != groups[g]:
                    groups[g] = on
                    v.fire_group(g, on)
            if "brakes" in row and bool(row["brakes"]) != brakes:
                brakes = bool(row["brakes"])
                v.set_brakes(brakes)
            st = self.actuators()
            self.shadow.append(st)
            self.apply_inputs(i)
            t_next = rows[i + 1]["ut"] if i + 1 < len(rows) else row["ut"] + W.DT
            n = max(1, int(round((t_next - row["ut"]) / W.DT)))
            f_tot, t_tot = v.wrench(W.DT)
            env = v.env
            f_aero = v.last_aero_body
            for _ in range(n - 1):
                v.wrench(W.DT)
            st = self.actuators()
            if i + 1 >= len(rows) or n != 1:
                continue
            a, b = row, rows[i + 1]
            dt = b["ut"] - a["ut"]
            qa = quat.norm(tuple(a["q"]))
            qb = quat.norm(tuple(b["q"]))
            g = vec.scale(tuple(a["r"]), -body.mu / vec.norm(tuple(a["r"])) ** 3)
            acc = vec.sub(vec.scale(vec.sub(tuple(b["v"]), tuple(a["v"])), 1.0 / dt), g)
            f_game = vec.scale(quat.rotate(quat.conj(qa), acc), a["m"])
            wa = quat.rotate(quat.conj(qa), tuple(a["w"]))
            wb = quat.rotate(quat.conj(qb), tuple(b["w"]))
            dw = vec.scale(vec.sub(wb, wa), 1.0 / dt)
            I = v.inertia
            t_game = vec.add(vec.mat_vec(I, dw), vec.cross(wa, vec.mat_vec(I, wa)))
            self.frames.append({"i": i, "t": a["t"], "mach": env["mach"], "q": env["q"],
                                "alpha": env["alpha"], "beta": env["beta"], "alt": env["alt"],
                                "fa_game": tuple(b["fa"]), "fa_sim": f_aero,
                                "f_game": f_game, "f_sim": f_tot,
                                "t_game": t_game, "t_sim": t_tot, "c": a["c"]})

    # -- state ------------------------------------------------------------

    def _kinematics(self, i):
        row = self.rows[i]
        v = self.v
        w = self.world
        w.t = row["ut"]
        w.events = []
        v.r = tuple(row["r"])
        v.v = tuple(row["v"])
        v.q = quat.norm(tuple(row["q"]))
        v.w = tuple(row["w"])
        v.destroyed = False
        v.contact = False
        v.drain_active = False
        # Mass: scale the propellant to the recorded mass.
        for p, amts in zip(v.parts, self.amounts0):
            for r, a in zip(p.resources, amts):
                r.amount = a
        v._mass_props()
        dm = v.mass - row["m"]
        if abs(dm) > 1e-4:
            pool = [r for p in v.parts for r in p.resources if r.density > 0 and r.amount > 0]
            have = sum(r.amount * r.density for r in pool)
            if have > 0:
                f = max(0.0, 1.0 - dm / have)
                for r in pool:
                    r.amount *= f
            v._mass_props()
        if "root" in row:
            # The recorded CoM (the vessel frame's origin) placed by the root
            # part: the mass alone cannot say which tanks emptied.
            root = self.model["vessel"].get("root") or 0
            p = v.parts[root].position
            v.com = tuple(p[k] - row["root"][k] for k in range(3))
        if "ec" in row:
            have = v.resource_amount("ElectricCharge")
            if have < row["ec"]:
                v.add_resource("ElectricCharge", row["ec"] - have)
            elif have > row["ec"]:
                v.draw("ElectricCharge", have - row["ec"])
        v._authority_age = 99

    def set_state(self, i):
        self._kinematics(i)
        self.set_actuators(self.shadow[i])
        self.apply_inputs(i)

    def apply_inputs(self, i):
        row = self.rows[i]
        c = self.v.control
        for name, value in zip(AXES, row["c"]):
            setattr(c, name, value)
        sw = self.switch_state(row["t"])
        c.rcs = bool(row.get("rcs", sw["rcs"]))
        c.gear = bool(row.get("gear", sw["gear"]))
        c.brakes = 1.0 if row.get("brakes", sw["brakes"]) else 0.0

    # -- one-step: forces at the game's state -------------------------------

    def forces(self, stride=1):
        """Per frame i: what the sim's step from the game's state i applies,
        against what the game applied between frames i and i+1.

        The game reports ``aerodynamic_force`` a frame late -- the force in
        row i+1 is the one computed from row i's state (measured: aligning
        them so takes the normal-force rms from 3.4 kN to 0.7 kN on qs_cone)
        -- and the step's total force and torque are read off the change in
        velocity and rate from row i to i+1."""
        return self.frames[::stride]

    # -- multi-step: restarts ---------------------------------------------

    def predict(self, horizons, every):
        rows = self.rows
        ts = [r["t"] for r in rows]
        hmax = max(horizons)
        out = []
        k = 0
        while k < len(rows):
            t0 = ts[k]
            if t0 + hmax > ts[-1]:
                break
            self.set_state(k)
            res = {"t0": t0}
            j = k
            for h in sorted(horizons):
                target = t0 + h
                while True:
                    # Advance one physics step with the input recorded for
                    # the frame the sim is in.
                    while j + 1 < len(rows) and rows[j + 1]["ut"] <= self.world.t + 1e-6:
                        j += 1
                    if self.world.t >= rows[k]["ut"] + h - 1e-6:
                        break
                    self.apply_inputs(j)
                    self.world.step()
                # The recorded frame at the sim's time.
                g = rows[j]
                if abs(g["ut"] - self.world.t) > 0.011:
                    res[h] = None
                    continue
                res[h] = self.compare(g)
            out.append(res)
            nxt = t0 + every
            while k < len(rows) and ts[k] < nxt:
                k += 1
        return out

    def compare(self, g):
        v = self.v
        dr = vec.sub(v.r, tuple(g["r"]))
        dv = vec.sub(v.v, tuple(g["v"]))
        qg = quat.norm(tuple(g["q"]))
        rel = quat.mul(quat.conj(qg), v.q)
        ang = math.degrees(2.0 * math.acos(min(1.0, abs(rel[3]))))
        dw = vec.norm(vec.sub(v.w, tuple(g["w"])))
        # Along-track / vertical split of the position error.
        vg = tuple(g["v"])
        along = vec.dot(dr, vec.unit(vg))
        up = vec.dot(dr, vec.unit(tuple(g["r"])))
        return {"dr": vec.norm(dr), "along": along, "up": up, "dv": vec.norm(dv),
                "datt": ang, "dw": math.degrees(dw), "destroyed": v.destroyed}


def _rows(fs, key_g, key_s, names, unit):
    lines = []
    for k, name in enumerate(names):
        g = [f[key_g][k] for f in fs]
        s = [f[key_s][k] for f in fs]
        err = [y - x for x, y in zip(g, s)]
        mag = statistics.median(abs(x) for x in g)
        rms = math.sqrt(sum(e * e for e in err) / len(err))
        lines.append("  %-10s game |med| %9.0f %-2s  sim-game median %+8.0f  rms %8.0f  "
                     "(%5.1f%% of |med|)  corr %.3f"
                     % (name, mag, unit, statistics.median(err), rms,
                        100.0 * rms / max(mag, 1e-9), _corr(g, s)))
    return lines


def summarise_forces(fs):
    if not fs:
        return "no frames"
    lines = []
    aero = [f for f in fs if f["q"] > 1.0]
    if aero:
        lines += _rows(aero, "fa_game", "fa_sim", ("aero Fx", "aero Fy", "aero Fz"), "N")
    lines += _rows(fs, "f_game", "f_sim", ("total Fx", "total Fy", "total Fz"), "N")
    lines += _rows(fs, "t_game", "t_sim", ("T pitch x", "T roll y", "T yaw z"), "Nm")
    return "\n".join(lines)


def _corr(a, b):
    n = len(a)
    if n < 3:
        return float("nan")
    ma, mb = sum(a) / n, sum(b) / n
    sa = math.sqrt(sum((x - ma) ** 2 for x in a))
    sb = math.sqrt(sum((x - mb) ** 2 for x in b))
    if sa == 0 or sb == 0:
        return float("nan")
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb)


def summarise_predict(ps, horizons):
    lines = ["  horizon   n   |dr| m med/max    along m med   up m med   |dv| m/s med/max   "
             "att deg med/max   rate deg/s med/max"]
    for h in sorted(horizons):
        rs = [p[h] for p in ps if p.get(h)]
        if not rs:
            continue

        def mm(key):
            xs = [r[key] for r in rs]
            return statistics.median(xs), max(xs)
        dr, dv, da, dw = mm("dr"), mm("dv"), mm("datt"), mm("dw")
        lines.append("  %5.1fs %4d   %8.2f %8.2f   %+9.2f   %+8.2f   %7.3f %7.3f     %6.2f %6.2f    "
                     "%6.2f %6.2f" % (h, len(rs), dr[0], dr[1],
                                      statistics.median(r["along"] for r in rs),
                                      statistics.median(r["up"] for r in rs),
                                      dv[0], dv[1], da[0], da[1], dw[0], dw[1]))
    return "\n".join(lines)


def summarise_segments(fs):
    """Mean game vs sim wrench over each stretch of constant input."""
    segs = []
    for f in fs:
        key = tuple(round(x, 2) for x in f["c"])
        while len(key) > 4 and key[-1] == 0:
            key = key[:-1]
        if segs and segs[-1][0] == key:
            segs[-1][1].append(f)
        else:
            segs.append((key, [f]))
    lines = ["  input (p y r thr)          n    force game -> sim (N, body x y z)"
             "                        torque game -> sim (Nm, pitch roll yaw)"]
    for key, group in segs:
        if len(group) < 5:
            continue
        g = group[2:]          # skip the actuator transient

        def mean(k, j):
            return sum(x[k][j] for x in g) / len(g)
        lines.append("  %-24s %4d  %s  %s" % (
            " ".join("%+.2f" % x for x in key), len(g),
            " ".join("%+8.0f>%+8.0f" % (mean("f_game", j), mean("f_sim", j)) for j in range(3)),
            " ".join("%+8.0f>%+8.0f" % (mean("t_game", j), mean("t_sim", j)) for j in range(3))))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording", nargs="+")
    ap.add_argument("--model", default=None, help="model name (default: the recording's save)")
    ap.add_argument("--horizons", default="0.5,2,5,10")
    ap.add_argument("--every", type=float, default=1.0)
    ap.add_argument("--stride", type=int, default=1, help="one-step: every Nth frame")
    ap.add_argument("--segments", action="store_true",
                    help="mean wrench over each stretch of constant input")
    ap.add_argument("--json", default=None, help="write the per-frame results here")
    args = ap.parse_args()
    horizons = [float(x) for x in args.horizons.split(",")]
    for path in args.recording:
        header, rows = read(path)
        model = args.model or header["save"]
        rp = Replay(model, header, rows)
        span = rows[-1]["t"] - rows[0]["t"]
        hs = [h for h in horizons if h < span]
        print("%s  (%s, %s, %d frames, %.1f s)" % (os.path.basename(path), model,
                                                  header.get("script"), len(rows), span))
        fs = rp.forces(args.stride)
        print(" one-step, sim vs game at the game's state:")
        print(summarise_forces(fs))
        if args.segments:
            print(summarise_segments(fs))
        ps = rp.predict(hs, args.every) if hs else []
        print(" restarts every %.1f s, flying the game's inputs:" % args.every)
        print(summarise_predict(ps, hs))
        if args.json:
            with open(args.json, "w") as fh:
                json.dump({"forces": fs, "predict": [{str(k): v for k, v in p.items()}
                                                     for p in ps]}, fh)


if __name__ == "__main__":
    main()
