"""Fly the real guidance against the measured tables, offline.

The truth model and the predictor share their physics here, so this cannot
say anything about whether the *model* is right -- that is what the game is
for, and the sibling project has a whole section on sim results that did not
survive contact with it.  What it can say is whether the guidance converges:
whether the solve's signs are right, whether it saturates, whether the phases
hand over sensibly, and whether the vehicle arrives at the gate on speed.
Those are the failures worth catching before spending a three-minute flight.

    python3 -m tests.glidesim --dv 60
    python3 -m tests.glidesim --sweep
"""
import argparse
import math
import sys

from boosterland import vec
from spaceplane import guidance, trajectory
from spaceplane.config import Config, apply_overrides
from spaceplane.trajectory import Steer
from tests.fakeplane import FakeEnv, circular_state

MASS = 6715.0


def advance(env, cfg, r, v, mass, steer, duration, dt_max=0.5):
    """Step the truth forward with the commanded controls held.

    ``reversing`` is forced off here, and that is the difference between a
    truth model and a prediction.  The *predictor* models the mean of a
    reversing entry -- vertical lift reduced by cos(bank), lateral lift
    averaging to nothing -- because that is the manoeuvre the vehicle flies
    over minutes.  The *vehicle* at any instant has a real bank and really
    turns.  Left at the default this simulated a vehicle that could never
    change its heading at all, which made the cross-track behaviour
    meaningless and looked like a guidance failure.
    """
    truth = Steer(alpha=steer.alpha, bank=steer.bank, cfg=steer.cfg,
                  mass=steer.mass, reversing=False)
    t = 0.0
    while t < duration:
        step = min(dt_max, duration - t)
        r, v = trajectory._step(env, r, v, mass, step, cfg, truth)
        t += step
    return r, v


def start_longitude(cfg, dv, alpha=None):
    """Where to be in orbit so that burning ``dv`` now reaches the runway.

    One trial propagation: fly the entry from an arbitrary longitude, measure
    the ground track it covers, and step back that far from the gate.  The
    propagation is already in the rotating frame, so the planet's rotation
    during the entry is inside the answer and does not need adding.
    """
    env = FakeEnv(cfg)
    r, v = circular_state(env, 80000.0, 0.0)
    speed = vec.norm(v)
    v = vec.add(v, vec.scale(v, -dv / speed))
    end = env.runway.ends["09"]
    gate = env.runway.gate(end)
    steer = Steer(alpha=alpha or cfg.ENTRY_ALPHA_DEG,
                  bank=cfg.SOLVE_BANK_MIN_DEG, cfg=cfg, mass=MASS)
    p = trajectory.predict(env, r, v, MASS, cfg, steer=steer,
                           target_radius=vec.norm(gate))
    arc = trajectory.surface_distance(env, r, p.position)
    degrees = math.degrees(arc / env.equatorial_radius)
    gate_lon = math.degrees(math.atan2(gate[2], gate[0]))
    return gate_lon - degrees


def fly(cfg, dv, longitude=0.0, verbose=True, jitter=0.0):
    env = FakeEnv(cfg)
    r, v = circular_state(env, 80000.0, longitude)
    speed = vec.norm(v)
    retro = vec.scale(v, -1.0 / speed)
    v = vec.add(v, vec.scale(retro, dv))
    if jitter:
        # A cross-track kick, to see whether bank actually takes it out.
        up = vec.unit(r)
        side = vec.unit(vec.cross(up, v))
        v = vec.add(v, vec.scale(side, jitter))

    end = env.runway.choose(r, v)
    gate = env.runway.gate(end)
    steer = Steer(alpha=cfg.ENTRY_ALPHA_DEG, bank=cfg.SOLVE_BANK_MIN_DEG,
                  cfg=cfg, mass=MASS)
    t = 0.0
    rows = []
    tick = cfg.GLIDE_TICK_S
    solved = 0
    while t < 4000.0:
        altitude = vec.norm(r) - env.equatorial_radius
        height = vec.norm(r) - env.target_radius
        distance = trajectory.surface_distance(env, r, gate)
        if cfg.HAC_ON:
            if height <= cfg.HAC_ALT_M or distance <= cfg.HAC_ENTRY_DIST_M:
                break
        else:
            if height <= cfg.GATE_ALT_M:
                break
            if height <= 1.15 * cfg.GATE_ALT_M \
                    and distance <= cfg.GATE_CAPTURE_M:
                break
        if height <= cfg.ENTRY_INTERFACE_M:
            new, prediction = guidance.solve_glide(env, r, v, MASS, cfg, end,
                                                   steer.alpha, steer.bank)
            solved += 1
            alpha = vec.clamp(new.alpha, steer.alpha - cfg.ALPHA_RATE_DEG_S * tick,
                              steer.alpha + cfg.ALPHA_RATE_DEG_S * tick)
            bank = vec.clamp(new.bank, steer.bank - cfg.BANK_RATE_DEG_S * tick,
                             steer.bank + cfg.BANK_RATE_DEG_S * tick)
            steer = Steer(alpha=alpha, bank=bank, cfg=cfg, mass=MASS)
            miss = ((prediction.long, prediction.cross)
                    if prediction is not None and prediction.reached
                    else (float("nan"), float("nan")))
            rows.append((t, height, vec.norm(v), alpha, bank, miss, distance))
        r, v = advance(env, cfg, r, v, MASS, steer, tick)
        t += tick

    # -- and now fly it down.  The approach and the flare are geometric laws
    # with no prediction in them at all, so this is the only place they get
    # exercised before a runway is involved.
    gravity = 9.81
    # -- the cone, when it is flying.  Same law the autopilot runs, same
    # exit test, so a sign error here is a sign error there.
    hac_rows = []
    if cfg.HAC_ON:
        end, side = guidance.hac_choose(env, cfg, env.runway, r, v)
        while t < 5000.0:
            height = vec.norm(r) - env.target_radius
            command = guidance.hac(env, cfg, end, r, v, MASS, gravity,
                                   height, side)
            if command is None:
                break
            hac_rows.append((t, height, vec.norm(v), command.alpha,
                             command.bank, command.turn_deg,
                             command.distance, command.radius, command.laps))
            if ((command.turn_deg <= cfg.HAC_EXIT_TURN_DEG
                 and command.gate_range <= cfg.HAC_ROLLOUT_M
                 and command.laps == 0
                 and height <= command.needed_height + cfg.HAC_EXIT_SURPLUS_M)
                    or height <= cfg.GATE_ALT_M):
                break
            steer = Steer(alpha=command.alpha, bank=command.bank, cfg=None,
                          mass=MASS)
            r, v = advance(env, cfg, r, v, MASS, steer, 1.0, dt_max=0.25)
            t += 1.0

    approach_rows = []
    flare_rows = []
    flare_start = None
    touchdown = None
    while t < 5000.0:
        height = vec.norm(r) - env.target_radius
        if height <= 0.0:
            touchdown = (vec.norm(v), -vec.dot(v, vec.unit(r)))
            break
        sink_now = -vec.dot(v, vec.unit(r))
        if height <= cfg.FLARE_ALT_M + cfg.FLARE_LEAD_S * max(0.0, sink_now):
            if flare_start is None:
                flare_start = t
            alpha, sink, needed = guidance.flare(
                env, cfg, r, v, MASS, gravity, height, t - flare_start)
            bank = 0.0
            flare_rows.append((t, height, vec.norm(v), alpha, sink, needed))
        else:
            command = guidance.approach(env, cfg, end, r, v, MASS, gravity,
                                        height)
            alpha, bank, sink = command.alpha, command.bank, command.sink
            approach_rows.append((t, height, vec.norm(v), alpha, bank,
                                  command.cross, command.distance))
        # ``cfg=None``, so no arrival-speed cap.  That cap is a *glide* law:
        # it gives up angle of attack to arrive able to land.  Applied to the
        # flare it clamps a 30 degree command down to 9, which is what made
        # the first offline landings arrive at 18 m/s of sink with the log
        # showing a perfectly sensible flare command.  The vehicle in game
        # never had the cap on this path either -- ``Autoland.aim`` sends the
        # angle straight to the autopilot -- so the sim was modelling a
        # restriction the real thing does not have.
        steer = Steer(alpha=alpha, bank=bank, cfg=None, mass=MASS)
        r, v = advance(env, cfg, r, v, MASS, steer, 0.2, dt_max=0.05)
        t += 0.2

    height = vec.norm(r) - env.target_radius
    offset = (vec.norm(r) - env.target_radius - cfg.GATE_ALT_M,
              trajectory.surface_distance(env, r, gate))
    threshold_offset = trajectory.miss_components(env, end, r,
                                                  end["threshold"])
    result = {
        "end": end["name"], "time": t, "height": height,
        "speed": vec.norm(v), "long": threshold_offset[0],
        "cross": threshold_offset[1],
        "distance": math.hypot(*threshold_offset),
        "touchdown_speed": touchdown[0] if touchdown else None,
        "sink": touchdown[1] if touchdown else None,
        "solves": solved, "rows": rows, "approach": approach_rows,
        "hac": hac_rows,
    }
    if verbose:
        print("dv=%.0f  runway %s  %d solves" % (dv, end["name"], solved))
        print("   t     h      v    alpha  bank      long     cross   d_gate")
        step = max(1, len(rows) // 22)
        for row in rows[::step] + (rows[-1:] if rows else []):
            t0, h, sp, a, b, m, d = row
            print("  %5.0f %6.0f %6.1f  %5.1f %+6.1f  %+8.0f %+8.0f %8.0f"
                  % (t0, h, sp, a, b, m[0], m[1], d))
        if hac_rows:
            print()
            print("  HAC  (turning %s)" % ("left" if hac_rows[0][4] >= 0
                                           else "right"))
            print("     t      h      v   alpha   bank    turn"
                  "       d       R  laps")
            hstep = max(1, len(hac_rows) // 16)
            for row in hac_rows[::hstep] + hac_rows[-1:]:
                print("  %5.0f %6.0f %6.1f  %5.1f %+6.1f %7.1f %7.0f %7.0f"
                      "   %d" % row)
        print()
        print("  APPROACH and FLARE")
        print("     t      h      v   alpha   bank      xt    to aim")
        step = max(1, len(approach_rows) // 12)
        for row in approach_rows[::step] + approach_rows[-1:]:
            print("  %5.0f %6.1f %6.1f  %5.1f %+6.1f %+7.0f %9.0f" % row)
        if flare_rows:
            print("  flare:   t      h      v   alpha   sink  needed")
            fstep = max(1, len(flare_rows) // 10)
            for row in flare_rows[::fstep] + flare_rows[-1:]:
                print("        %5.0f %6.1f %6.1f  %5.1f %6.2f %6.2f" % row)
        if touchdown:
            print("  TOUCHDOWN at %.1f m/s, sink %.2f m/s" % touchdown)
        else:
            print("  NO TOUCHDOWN: ran out of time at h=%.0f" % height)
        print("  from the threshold: along %+.0f  across %+.0f  (%.0f m)"
              % (threshold_offset[0], threshold_offset[1],
                 result["distance"]))
    return result


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--dv", type=float, default=60.0)
    p.add_argument("--longitude", type=float, default=None)
    p.add_argument("--jitter", type=float, default=0.0)
    p.add_argument("--sweep", action="store_true")
    p.add_argument("--set", action="append", default=[])
    args = p.parse_args(argv or sys.argv[1:])
    cfg = apply_overrides(Config(), args.set)
    if args.longitude is None:
        args.longitude = start_longitude(cfg, args.dv)
        print("# starting at longitude %.1f (gate is at %.1f)"
              % (args.longitude, args.longitude))
    if not args.sweep:
        fly(cfg, args.dv, args.longitude, jitter=args.jitter)
        return 0
    print("  dv  jitter   runway     |d to gate|    v      h    solves")
    for dv in (40.0, 60.0, 80.0, 120.0):
        # Each dv reaches the runway from a different place, which is what the
        # deorbit solve works out in flight; offline it has to be handed over.
        longitude = start_longitude(cfg, dv)
        for jitter in (0.0, 60.0, -60.0):
            r = fly(cfg, dv, longitude, verbose=False, jitter=jitter)
            print("  %3.0f  %+6.0f   %-6s   %9.0f %7.1f %6.0f %6d"
                  % (dv, jitter, r["end"], r["distance"], r["speed"],
                     r["height"], r["solves"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
