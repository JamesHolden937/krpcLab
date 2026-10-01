"""kRPC 0.6.0's auto-pilot, ported: ``AttitudeController.Update`` and its PIDs.

The source is ``service/SpaceCenter/src/AutoPilot/AttitudeController.cs`` at
tag v0.6.0 (the installed ``KRPC.SpaceCenter.dll`` carries its symbols).  The
cascade is the same: a velocity profile turns the attitude error into a
target angular velocity in the roll-invariant frame, autotuned PI loops track
it, an analytic acceleration feed-forward and a gyroscopic term are added,
and a soft start fades the command in.  What is left out is the
flexible-craft machinery (chatter detector, notch and low-pass rate filters,
bandwidth floor, output smoothing, the antipodal plane latch): on a rigid
craft the detector never latches and every one of those passes the signal
through unchanged.

Frames: ``rotation`` is the vessel's rotation in the auto-pilot's reference
frame (vessel axes x right, y nose, z belly), ``omega_rel`` the vessel's
angular velocity relative to that frame expressed in it, Unity's sense.
Outputs are the (pitch, roll, yaw) control fractions KSP applies.
"""
import math

from kspSim import quat

TORQUE_SMOOTH_TC = 0.5
FF_SMOOTH_TC = 0.05
DEADBAND_LOW_FRACTION = 0.5
PROFILE_CONE_FRACTION = 0.5
MIN_THETA_JOINT = 1e-10
SOFT_START_TIME = 0.5
UP = (0.0, 1.0, 0.0)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def clamp_angle_180(a):
    a = a % 360.0
    return a - 360.0 if a > 180.0 else a


def to_angle_axis(q):
    q = quat.norm(q)
    w = clamp(q[3], -1.0, 1.0)
    s = math.sqrt(1.0 - w * w)
    if s < 1e-10:
        return 0.0, UP
    return math.degrees(2.0 * math.acos(w)), (q[0] / s, q[1] / s, q[2] / s)


def from_to(a, b):
    na = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    nb = math.sqrt(b[0] ** 2 + b[1] ** 2 + b[2] ** 2)
    a = (a[0] / na, a[1] / na, a[2] / na)
    b = (b[0] / nb, b[1] / nb, b[2] / nb)
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    if d >= 1.0 - 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    if d <= -1.0 + 1e-10:
        if abs(a[0]) < 0.9:
            p = (0.0, a[2], -a[1])            # a x right
        else:
            p = (-a[2], 0.0, a[0])            # a x up
        n = math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
        return (p[0] / n, p[1] / n, p[2] / n, 0.0)
    c = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
    return quat.norm((c[0], c[1], c[2], 1.0 + d))


def canonical(q):
    if q[3] < 0 or (q[3] == 0 and _largest_negative(q)):
        return (-q[0], -q[1], -q[2], -q[3])
    return q


def _largest_negative(q):
    ax, ay, az = abs(q[0]), abs(q[1]), abs(q[2])
    if ax >= ay and ax >= az:
        return q[0] < 0
    if ay >= az:
        return q[1] < 0
    return q[2] < 0


def angle_axis(angle_deg, axis):
    return quat.from_axis_angle(axis, math.radians(angle_deg))


def signed_angle(f, t, axis):
    n = math.sqrt(sum(x * x for x in axis))
    axis = tuple(x / n for x in axis)
    fd = sum(a * b for a, b in zip(f, axis))
    td = sum(a * b for a, b in zip(t, axis))
    f = tuple(a - fd * b for a, b in zip(f, axis))
    t = tuple(a - td * b for a, b in zip(t, axis))
    fn = math.sqrt(sum(x * x for x in f))
    tn = math.sqrt(sum(x * x for x in t))
    if fn < 1e-10 or tn < 1e-10:
        return 0.0
    f = tuple(x / fn for x in f)
    t = tuple(x / tn for x in t)
    u = math.degrees(math.acos(clamp(sum(a * b for a, b in zip(f, t)), -1.0, 1.0)))
    c = (f[1] * t[2] - f[2] * t[1], f[2] * t[0] - f[0] * t[2], f[0] * t[1] - f[1] * t[0])
    return -u if sum(a * b for a, b in zip(axis, c)) < 0 else u


def vangle(a, b):
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(x * x for x in b))
    if na == 0 or nb == 0:
        return 0.0
    return math.degrees(math.acos(clamp(sum(x * y for x, y in zip(a, b)) / (na * nb), -1.0, 1.0)))


def rotation_from_direction_up_roll(direction, up, roll):
    n = math.sqrt(sum(x * x for x in direction))
    d = tuple(x / n for x in direction)
    ud = sum(a * b for a, b in zip(up, d))
    roof = tuple(a - ud * b for a, b in zip(up, d))
    q0 = from_to(UP, d)
    rn = math.sqrt(sum(x * x for x in roof))
    if rn < 1e-6:
        return q0
    roof0 = quat.rotate(q0, (0.0, 0.0, -1.0))
    align = signed_angle(roof0, tuple(x / rn for x in roof), d)
    return quat.norm(quat.mul(angle_axis(align - roll, d), q0))


def roll_relative_to(rotation, up):
    nose = quat.rotate(rotation, UP)
    reference = rotation_from_direction_up_roll(nose, up, 0.0)
    residual = quat.mul(rotation, quat.conj(reference))
    angle, axis = to_angle_axis(residual)
    angle = clamp_angle_180(angle)
    n = math.sqrt(sum(x * x for x in nose))
    return -angle * sum(a * b / n for a, b in zip(axis, nose))


class PID:
    def __init__(self):
        self.kp, self.ki, self.kd = 1.0, 0.0, 0.0
        self.lo, self.hi = -1.0, 1.0
        self.integral = 0.0
        self.last = 0.0

    def reset_state(self):
        self.integral = 0.0
        self.last = 0.0

    def set(self, kp, ki, kd, lo=-1.0, hi=1.0):
        self.kp, self.ki, self.kd, self.lo, self.hi = kp, ki, kd, lo, hi
        self.integral = clamp(self.integral, lo, hi)

    def update(self, setpoint, value, dt):
        error = setpoint - value
        di = self.ki * error * dt
        if not ((self.integral >= self.hi and di > 0) or (self.integral <= self.lo and di < 0)):
            self.integral += di
        self.integral = clamp(self.integral, self.lo, self.hi)
        deriv = (value - self.last) / dt
        out = clamp(self.kp * error + self.integral - self.kd * deriv, self.lo, self.hi)
        self.last = value
        return out


class Sample:
    __slots__ = ("s_pitch", "s_yaw", "theta_pitch", "theta_yaw", "speed", "alpha",
                 "bandwidth", "cap", "cone", "cone_slope", "linear", "deadband",
                 "theta", "estop", "valid")

    def __init__(self):
        self.s_pitch = self.s_yaw = self.theta_pitch = self.theta_yaw = 0.0
        self.speed = self.alpha = self.bandwidth = self.cone_slope = 0.0
        self.deadband = self.theta = self.estop = 0.0
        self.cap = self.cone = self.linear = self.valid = False


class AttitudeController:
    def __init__(self):
        self.pids = [PID(), PID(), PID()]     # pitch, roll, yaw
        self.engaged = False
        self.reference_frame = None
        self.max_angular_velocity = (1.0, 1.0, 1.0)
        self.roll_attenuation = 1.0
        self.pitch_yaw_attenuation = 1.0
        self.roll_start = 20.0
        self.roll_engage = 15.0
        self.auto_tune = True
        self.overshoot = (0.01, 0.01, 0.01)
        self.time_to_peak = (1.0, 1.0, 1.0)
        self.target_smoothing = 0.0
        self.up_reference = (1.0, 0.0, 0.0)
        self.target_rotation = (0.0, 0.0, 0.0, 1.0)
        self.effective_rotation = self.target_rotation
        self.roll_controlled = False
        self.slew_pending = False
        self.slew_speed = 0.0
        self.smoothed_torque = (0.0, 0.0, 0.0)
        self.smoothed_ff = [0.0, 0.0, 0.0]
        self.unfloored_kp = [0.0, 0.0, 0.0]
        self.twice_zeta_omega = [0.0, 0.0, 0.0]
        self.omega_sq = [0.0, 0.0, 0.0]
        self.engage_time = 0.0
        self.point_valid = False
        self.point_rotation = None
        self.prev_point = None
        self.log_angles = (0.0, 0.0, 0.0)
        self._update_gains()
        self.set_target(0.0, 0.0, float("nan"))

    # -- configuration --------------------------------------------------

    def _update_gains(self):
        for i in range(3):
            lo = math.log(self.overshoot[i])
            zeta = math.sqrt(lo * lo / (math.pi * math.pi + lo * lo))
            w = math.pi / (self.time_to_peak[i] * math.sqrt(1.0 - zeta * zeta))
            self.twice_zeta_omega[i] = 2.0 * zeta * w
            self.omega_sq[i] = w * w

    def set_time_to_peak(self, v):
        self.time_to_peak = tuple(v)
        self._update_gains()

    def set_overshoot(self, v):
        self.overshoot = tuple(v)
        self._update_gains()

    def set_target(self, pitch, heading, roll):
        # Pitch/heading targets are not used by the autopilots here; keep
        # the rotation that the defaults describe (pointing at the horizon).
        self.roll_controlled = not math.isnan(roll)
        self.target_rotation = (0.0, 0.0, 0.0, 1.0)
        self.slew_pending = True

    def set_target_direction(self, direction):
        self.target_rotation = from_to(UP, direction)
        self.roll_controlled = False
        self.slew_pending = True

    def set_target_rotation(self, rotation):
        self.target_rotation = tuple(rotation)
        self.roll_controlled = True
        self.slew_pending = True

    def set_direction_and_up(self, direction, up, roll):
        self.up_reference = tuple(up)
        self.target_rotation = rotation_from_direction_up_roll(direction, up, roll)
        self.roll_controlled = True
        self.slew_pending = True

    def set_target_roll(self, value):
        if math.isnan(value):
            self.roll_controlled = False
            self.slew_pending = True
            return
        self.target_rotation = rotation_from_direction_up_roll(
            self.target_direction(), self.up_reference, value)
        self.roll_controlled = True
        self.slew_pending = True

    def target_direction(self):
        return quat.rotate(self.target_rotation, UP)

    def target_roll(self):
        if not self.roll_controlled:
            return float("nan")
        return roll_relative_to(self.target_rotation, self.up_reference)

    def engage(self, now, torque, moi):
        self.engaged = True
        self.start(now, torque, moi)

    def disengage(self):
        self.engaged = False

    def start(self, now, torque, moi):
        self.engage_time = now
        self.effective_rotation = self.target_rotation
        self.slew_speed = 0.0
        self.slew_pending = False
        for p in self.pids:
            p.reset_state()
        self.point_valid = False
        self.smoothed_ff = [0.0, 0.0, 0.0]
        if self.auto_tune:
            self._auto_tune(torque, moi)

    # -- the loop ---------------------------------------------------------

    def update(self, now, dt, rotation, omega_rel, torque, moi):
        """One physics tick.  Returns (pitch, roll, yaw)."""
        if self.slew_pending:
            self.slew_speed = (quat.angle_between(self.effective_rotation, self.target_rotation)
                               / self.target_smoothing if self.target_smoothing > 0 else 0.0)
            self.slew_pending = False
        if self.target_smoothing > 0:
            ang = quat.angle_between(self.effective_rotation, self.target_rotation)
            step = self.slew_speed * dt
            if ang <= step or ang < 1e-9:
                self.effective_rotation = self.target_rotation
            else:
                self.effective_rotation = _slerp(self.effective_rotation, self.target_rotation,
                                                 step / ang)
        else:
            self.effective_rotation = self.target_rotation

        lin = clamp((now - self.engage_time) / SOFT_START_TIME, 0.0, 1.0) if SOFT_START_TIME > 0 else 1.0
        soft = lin * lin * (3.0 - 2.0 * lin)

        current_dir = quat.rotate(rotation, UP)
        # Roll-invariant frame, carried continuously.
        if self.point_valid:
            delta = from_to(self.prev_point, current_dir)
            qp = quat.norm(quat.mul(delta, self.point_rotation))
        else:
            qp = from_to(UP, current_dir)
        self.point_rotation = qp
        self.prev_point = current_dir
        self.point_valid = True
        bx = quat.rotate(quat.mul(quat.conj(qp), rotation), (1.0, 0.0, 0.0))
        phi = math.atan2(-bx[2], bx[0])
        cphi, sphi = math.cos(phi), math.sin(phi)

        inv = quat.conj(rotation)
        wb = quat.rotate(inv, omega_rel)
        current = (-wb[0], -wb[1], -wb[2])

        current_ri = _to_ri(current, cphi, sphi)
        target, py_sample, roll_sample = self._target_velocity(
            torque, moi, current, current_dir, cphi, sphi, rotation, inv)
        target = list(target)
        if not self.roll_controlled:
            target[1] = 0.0
        else:
            err = vangle(current_dir, quat.rotate(self.effective_rotation, UP))
            if self._roll_weight(err) == 0.0:
                self.pids[1].integral = 0.0

        ff = self._feedforward(py_sample, roll_sample, current_ri, torque, moi)
        if not self.roll_controlled:
            ff[1] = 0.0
        beta = 1.0 - math.exp(-dt / FF_SMOOTH_TC)
        for i in range(3):
            self.smoothed_ff[i] += beta * (ff[i] - self.smoothed_ff[i])

        decay = math.exp(-dt / TORQUE_SMOOTH_TC)
        self.smoothed_torque = tuple(max(torque[i], self.smoothed_torque[i] * decay)
                                     for i in range(3))
        if self.auto_tune:
            self._auto_tune(self.smoothed_torque, moi)
        if soft < 1.0:
            for p in self.pids:
                p.integral = 0.0

        out = []
        for i in range(3):
            if torque[i] > 0:
                out.append(self.pids[i].update(target[i], current_ri[i], dt))
            else:
                self.pids[i].integral = 0.0
                out.append(0.0)
        vp = clamp(out[0] + self.smoothed_ff[0], -1.0, 1.0)
        vr = clamp(out[1] + self.smoothed_ff[1], -1.0, 1.0)
        vy = clamp(out[2] + self.smoothed_ff[2], -1.0, 1.0)
        body = _from_ri((vp, 0.0, vy), cphi, sphi)
        # Gyroscopic feed-forward.
        h = (moi[0] * current[0], moi[1] * current[1], moi[2] * current[2])
        g = (current[1] * h[2] - current[2] * h[1],
             current[2] * h[0] - current[0] * h[2],
             current[0] * h[1] - current[1] * h[0])
        gyro = tuple(-g[i] / torque[i] if torque[i] > 0 else 0.0 for i in range(3))
        u_pitch = clamp(body[0] + gyro[0], -1.0, 1.0)
        u_roll = clamp(vr + gyro[1], -1.0, 1.0)
        u_yaw = clamp(body[2] + gyro[2], -1.0, 1.0)
        return soft * u_pitch, soft * u_roll, soft * u_yaw

    def _roll_weight(self, err):
        return clamp((self.roll_start - err) / (self.roll_start - self.roll_engage), 0.0, 1.0)

    def _profile_kp(self, i):
        return self.unfloored_kp[i] if self.auto_tune else self.pids[i].kp

    def _target_velocity(self, torque, moi, omega, current_dir, cphi, sphi, rotation, inv):
        target_dir = quat.rotate(self.effective_rotation, UP)
        dir_rot = from_to(current_dir, target_dir)
        angle, axis = to_angle_axis(dir_rot)
        angle = clamp_angle_180(angle)
        ab = quat.rotate(inv, (axis[0] * angle, axis[1] * angle, axis[2] * angle))
        ari = list(_to_ri(ab, cphi, sphi))
        ari[1] = 0.0
        if self.roll_controlled:
            err = vangle(current_dir, target_dir)
            w = self._roll_weight(err)
            if w > 0:
                res = quat.mul(quat.mul(self.effective_rotation, inv), quat.conj(dir_rot))
                ra, rax = to_angle_axis(canonical(res))
                ra = clamp_angle_180(ra)
                rb = quat.rotate(inv, rax)
                ari[1] = ra * rb[1] * w
        self.log_angles = tuple(ari)
        omega_ri = _to_ri(omega, cphi, sphi)
        roll_bw = self._profile_kp(1) * torque[1] / moi[1] if moi[1] > 0 else 0.0
        roll_v, roll_sample = self._axis_velocity(
            ari[1], torque[1], moi[1], omega_ri[1], self.max_angular_velocity[1],
            self.roll_attenuation, roll_bw)
        pv, yv, py_sample = self._pitch_yaw_velocity(ari, omega_ri, torque, moi)
        return (pv, roll_v, yv), py_sample, roll_sample

    def _pitch_yaw_velocity(self, ari, omega_ri, torque, moi):
        s = Sample()
        tp = math.radians(ari[0])
        ty = math.radians(ari[2])
        ap = torque[0] / moi[0] if moi[0] > 0 else 0.0
        ay = torque[2] / moi[2] if moi[2] > 0 else 0.0
        op, oy = omega_ri[0], omega_ri[2]
        om = math.sqrt(op * op + oy * oy)
        bw0 = self._profile_kp(0) * torque[0] / moi[0] if moi[0] > 0 else 0.0
        bw2 = self._profile_kp(2) * torque[2] / moi[2] if moi[2] > 0 else 0.0
        coeff = 0.0
        linear = False
        lin_bw = 0.0
        if om > 0:
            u, v = op / om, oy / om
            aw = u * u * ap + v * v * ay
            if aw > 0:
                cq = om / (2.0 * aw)
                coeff = cq
                bwo = u * u * bw0 + v * v * bw2
                if bwo > 0:
                    coeff = max(cq, 1.0 / bwo)
                    linear = 1.0 / bwo > cq
                    lin_bw = bwo
        ep = tp + coeff * op
        ey = ty + coeff * oy
        em = math.sqrt(ep * ep + ey * ey)
        if em <= MIN_THETA_JOINT:
            return 0.0, 0.0, s
        sp, sy = ep / em, ey / em
        a2 = sp * sp * ap + sy * sy * ay
        mvp, mvy = self.max_angular_velocity[0], self.max_angular_velocity[2]
        if mvp > 0 and mvy > 0:
            mv = math.sqrt(1.0 / (sp * sp / (mvp * mvp) + sy * sy / (mvy * mvy)))
        else:
            mv = min(mvp, mvy)
        bws = sp * sp * bw0 + sy * sy * bw2
        slope = PROFILE_CONE_FRACTION * bws
        speed = 0.0
        unclamped = 0.0
        cone = False
        if a2 > 0:
            unclamped = math.sqrt(2.0 * em * a2)
            speed = min(mv, unclamped)
            if slope > 0 and slope * em < speed:
                speed = slope * em
                cone = True
        tm = math.sqrt(tp * tp + ty * ty)
        db = _deadband(tm, self.pitch_yaw_attenuation)
        s.s_pitch, s.s_yaw, s.theta_pitch, s.theta_yaw = sp, sy, tp, ty
        s.speed, s.alpha = speed, a2
        s.bandwidth = lin_bw if linear else 0.0
        s.cap = a2 > 0 and mv < unclamped and not cone
        s.cone, s.cone_slope, s.linear, s.deadband = cone, slope, linear, db
        s.theta, s.estop, s.valid = tm, em, a2 > 0
        return -sp * speed * db, -sy * speed * db, s

    def _axis_velocity(self, angle_deg, torque, moi, omega, maxv, dbhigh, pid_bw):
        s = Sample()
        theta = math.radians(angle_deg)
        pure = theta
        acc = torque / moi if moi > 0 else 0.0
        linear = False
        if acc > 0:
            cq = 0.5 * omega * abs(omega) / acc
            corr = cq
            if pid_bw > 0:
                cl = omega / pid_bw
                linear = abs(cl) > abs(cq)
                corr = cq if abs(cq) >= abs(cl) else cl
            theta += corr
        unclamped = math.sqrt(2.0 * abs(theta) * acc) if acc > 0 else 0.0
        speed = min(maxv, unclamped)
        slope = PROFILE_CONE_FRACTION * pid_bw
        cone = False
        if acc > 0 and slope > 0 and slope * abs(theta) < speed:
            speed = slope * abs(theta)
            cone = True
        db = _deadband(abs(pure), dbhigh)
        sign = (theta > 0) - (theta < 0)
        s.s_pitch = float(sign)
        s.theta_pitch = pure
        s.speed, s.alpha = speed, acc
        s.bandwidth = pid_bw if linear else 0.0
        s.cap = acc > 0 and maxv < unclamped and not cone
        s.cone, s.cone_slope, s.linear, s.deadband = cone, slope, linear, db
        s.theta, s.estop, s.valid = abs(pure), abs(theta), acc > 0
        return -sign * speed * db, s

    def _magnitude_rate(self, s, theta_dot, tracking, dbhigh):
        rate = 0.0
        if s.cone and s.speed > 1e-9:
            if s.linear:
                e = -s.speed * s.bandwidth / (s.bandwidth + s.cone_slope)
            else:
                e = -s.speed * s.alpha / (s.alpha + s.cone_slope * s.speed)
            rate = s.cone_slope * (e * tracking)
        elif not s.cap and s.speed > 1e-9:
            if s.linear:
                e = -(s.bandwidth * s.speed * s.speed) / (s.bandwidth * s.speed + s.alpha)
            else:
                e = -0.5 * s.speed
            rate = (s.alpha / s.speed) * (e * tracking)
        db_rate = 0.0
        if 0.0 < s.deadband < 1.0:
            high = math.radians(dbhigh)
            low = DEADBAND_LOW_FRACTION * high
            if high > low:
                db_rate = s.speed * (1.0 / (high - low)) * theta_dot * tracking
        return rate * s.deadband + db_rate

    def _feedforward(self, py, roll, omega_ri, torque, moi):
        ap = torque[0] / moi[0] if moi[0] > 0 else 0.0
        ar = torque[1] / moi[1] if moi[1] > 0 else 0.0
        ay = torque[2] / moi[2] if moi[2] > 0 else 0.0
        ffp = ffy = ffr = 0.0
        if py.valid:
            td = 0.0
            if py.theta > 1e-12:
                td = (py.theta_pitch * omega_ri[0] + py.theta_yaw * omega_ri[2]) / py.theta
            along = -(py.s_pitch * omega_ri[0] + py.s_yaw * omega_ri[2])
            tr = clamp(along / py.speed, 0.0, 1.0) if py.speed > 1e-9 else 0.0
            gd = self._magnitude_rate(py, td, tr, self.pitch_yaw_attenuation)
            over = min(1.0, py.speed / max(py.speed, along)) if py.speed > 1e-9 else 1.0
            if ap > 0:
                ffp = -py.s_pitch * gd / ap * over
            if ay > 0:
                ffy = -py.s_yaw * gd / ay * over
        if roll.valid and ar > 0:
            td = 0.0
            if roll.theta > 1e-12:
                td = ((roll.theta_pitch > 0) - (roll.theta_pitch < 0)) * omega_ri[1]
            along = -(roll.s_pitch * omega_ri[1])
            tr = clamp(along / roll.speed, 0.0, 1.0) if roll.speed > 1e-9 else 0.0
            gd = self._magnitude_rate(roll, td, tr, self.roll_attenuation)
            over = min(1.0, roll.speed / max(roll.speed, along)) if roll.speed > 1e-9 else 1.0
            ffr = -roll.s_pitch * gd / ar * over
        return [ffp, ffr, ffy]

    def _auto_tune(self, torque, moi):
        for i in range(3):
            if torque[i] <= 0:
                continue
            inv = moi[i] / torque[i]
            self.unfloored_kp[i] = self.twice_zeta_omega[i] * inv
            self.pids[i].set(self.twice_zeta_omega[i] * inv, self.omega_sq[i] * inv, 0.0)


def _deadband(err, high_deg):
    high = math.radians(high_deg)
    low = DEADBAND_LOW_FRACTION * high
    if high <= low:
        return 1.0 if err > low else 0.0
    return clamp((err - low) / (high - low), 0.0, 1.0)


def _to_ri(v, c, s):
    return (v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c)


def _from_ri(v, c, s):
    return (v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c)


def _slerp(a, b, t):
    d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
    if d < 0:
        b = (-b[0], -b[1], -b[2], -b[3])
        d = -d
    if d > 0.9995:
        return quat.norm(tuple(a[i] + t * (b[i] - a[i]) for i in range(4)))
    th = math.acos(d)
    s = math.sin(th)
    wa = math.sin((1 - t) * th) / s
    wb = math.sin(t * th) / s
    return quat.norm(tuple(wa * a[i] + wb * b[i] for i in range(4)))
