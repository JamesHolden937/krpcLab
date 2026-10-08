#!/usr/bin/env python3
"""Passive kRPC auto-pilot observer for one farm instance (scratch diagnostic).

Below 15 km: every ~0.25 s wall, log kRPC's own view (commanded vs effective
target, its attitude error, oscillation detector/latch, PID gains, pitch input).
When the nose has sat >3 deg off kRPC's commanded target for 8 game-seconds,
switch kRPC's DiagnosticLogging on (one minute, 50 Hz) and save the CSV.
Writes nothing to the vessel except DiagnosticLogging."""
import math, os, sys, time
ROOT = "/home/holden/krpcLab"
sys.path.insert(0, ROOT)
import krpc

inst = sys.argv[1]
out = sys.argv[2]
deadline = time.time() + float(sys.argv[3] if len(sys.argv) > 3 else 1500)
d = os.path.join(ROOT, "testInstances", "ksp" + inst)
rp, sp = int(open(d + "/.rpc_port").read()), int(open(d + "/.stream_port").read())
log = open(out + ".txt", "w")


def ang(a, b):
    na = math.sqrt(sum(x * x for x in a)); nb = math.sqrt(sum(x * x for x in b))
    if na == 0 or nb == 0:
        return float("nan")
    c = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


conn = None
since = None
triggers = 0
waiting_dump = False
started = False
while time.time() < deadline:
    try:
        if conn is None:
            conn = krpc.connect(name="apwatch", rpc_port=rp, stream_port=sp)
            sc = conn.space_center
            v = sc.active_vessel
            ap = v.auto_pilot
            F = v.orbit.body.reference_frame
            fl = v.flight(F)
            vid = v.id if hasattr(v, "id") else None
        alt = fl.mean_altitude
        if alt > 15000 or v.situation.name not in ("flying",):
            if started and v.situation.name in ("landed", "splashed"):
                break
            time.sleep(2.0)
            continue
        started = True
        ut = sc.ut
        tgt = ap.target_direction
        cur = ap.current_target_direction
        nose = v.direction(F)
        err = ap.current_attitude_error
        lvl = ap.oscillation_level
        latched = ap.pitch_yaw_oscillation_latched
        rlat = ap.roll_oscillation_latched
        cosc = ap.pitch_yaw_control_oscillation
        gains = ap.pitch_pid_gains
        ctl = v.control
        pin = ctl.pitch
        e_nose = ang(nose, tgt)
        log.write("%.2f alt=%.0f spd=%.1f aoa=%.1f nose-cmd=%.2f eff-cmd=%.2f krpcerr=(%.2f,%.2f,%.2f) osc=(%.2f,%.2f,%.2f) latch=%d/%d cosc=%.2f gains=(%.4f,%.4f,%.4f) pin=%.3f\n" % (
            ut, alt, fl.speed, fl.angle_of_attack, e_nose, ang(cur, tgt), err[0], err[1], err[2],
            lvl[0], lvl[1], lvl[2], int(latched), int(rlat), cosc, gains[0], gains[1], gains[2], pin))
        log.flush()
        if waiting_dump:
            if not ap.diagnostic_logging:
                open("%s.diag%d.csv" % (out, triggers), "w").write(ap.diagnostic_log)
                log.write("# dumped diag %d\n" % triggers)
                waiting_dump = False
        elif triggers < 3:
            if e_nose > 3.0:
                since = since if since is not None else ut
                if ut - since > 8.0:
                    ap.diagnostic_logging = True
                    triggers += 1
                    waiting_dump = True
                    since = None
                    log.write("# diag on %d at %.2f\n" % (triggers, ut))
            else:
                since = None
        time.sleep(0.25)
    except Exception as e:  # load/scene change: reconnect
        log.write("# exc %s\n" % (str(e)[:120],))
        try:
            conn.close()
        except Exception:
            pass
        conn = None
        time.sleep(3.0)
if waiting_dump:
    try:
        open("%s.diag%d.csv" % (out, triggers), "w").write(ap.diagnostic_log)
    except Exception:
        pass
log.close()
