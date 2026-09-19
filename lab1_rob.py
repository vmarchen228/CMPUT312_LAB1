#!/usr/bin/env python3
"""
CMPUT 312 Lab 1 -- shape driving on the EV3.

Section 3.4: lemniscate of Bernoulli with a = 0.5 m, driven from the
parametric curve using time-varying curvature.

Notes on conventions used throughout:
  * All lengths are in centimetres, all angles in radians unless stated.
  * The body frame has +x forward and +y to the LEFT, so a POSITIVE angular
    velocity w is a counter-clockwise (left) turn.  The EV3 gyro reports
    clockwise-positive degrees, hence GYRO_SIGN below.
  * OUTPUT_A is the LEFT wheel, OUTPUT_B is the RIGHT wheel.
"""

import time
import math

from ev3dev2.motor import (MoveTank, LargeMotor, OUTPUT_A, OUTPUT_B,
                           SpeedPercent, SpeedDPS)
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1


# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------

gyro = GyroSensor(INPUT_1)
gyro.reset()

robot_drive = MoveTank(OUTPUT_A, OUTPUT_B)
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)

DEFAULT_RAMP = 2000          # ms, used by the on_for_rotations() shapes
left_motor.ramp_up_sp = DEFAULT_RAMP
left_motor.ramp_down_sp = DEFAULT_RAMP
right_motor.ramp_up_sp = DEFAULT_RAMP
right_motor.ramp_down_sp = DEFAULT_RAMP


# ---------------------------------------------------------------------------
# Robot geometry  ---  MEASURE THESE, do not trust the nominal values.
# Run menu options 'w' and 'b' to calibrate, then paste the results here.
# ---------------------------------------------------------------------------

WHEEL_DIAMETER = 5.6         # cm, loaded rolling diameter (nominal 5.6)
BASE_WIDTH = 15.0            # cm, EFFECTIVE track width (contact patches)
OUTER_SPEED = 30             # percent, used by the simple shapes
TARGET_ANGLE = 85            # deg, gyro target for turn_90_dumb()

WHEEL_RADIUS = WHEEL_DIAMETER / 2.0
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

GYRO_SIGN = -1.0             # multiply gyro.angle by this to get CCW-positive


# ---------------------------------------------------------------------------
# Lemniscate parameters
# ---------------------------------------------------------------------------

LEMNI_A = 50.0               # cm, a = 0.5 m as required
LEMNI_DT = 0.05              # s, nominal control period (20 Hz)
LEMNI_KP = 1.5               # rad/s of correction per rad of heading error
LEMNI_SPEED_FRAC = 0.30      # fraction of max motor speed for the OUTER wheel


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _dps(v_cm_s):
    """Linear wheel speed [cm/s] -> motor angular speed [deg/s]."""
    return v_cm_s / WHEEL_RADIUS * 180.0 / math.pi


def _max_dps():
    return min(left_motor.max_speed, right_motor.max_speed)


def _wrap(angle):
    """Wrap an angle in radians to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _heading():
    """Current heading in radians, CCW-positive, relative to last gyro reset."""
    return GYRO_SIGN * math.radians(gyro.angle)


def _set_fast_response(on):
    """
    Disable (or restore) the motor ramps.

    The 2000 ms ramp is fine for on_for_rotations() moves, but it low-passes
    the 20 Hz speed commands of the lemniscate loop into mush -- the motors
    would never reach any commanded speed before the next command arrives.
    """
    v = 0 if on else DEFAULT_RAMP
    left_motor.ramp_up_sp = v
    left_motor.ramp_down_sp = v
    right_motor.ramp_up_sp = v
    right_motor.ramp_down_sp = v


class Odometry(object):
    """Dead-reckoned pose from the wheel encoders. Used for the report only."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self._pl = left_motor.position
        self._pr = right_motor.position

    def update(self):
        pl = left_motor.position
        pr = right_motor.position
        dl = math.radians(pl - self._pl) * WHEEL_RADIUS
        dr = math.radians(pr - self._pr) * WHEEL_RADIUS
        self._pl = pl
        self._pr = pr

        ds = 0.5 * (dl + dr)
        dth = (dr - dl) / BASE_WIDTH
        # midpoint (second-order) pose update
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th += dth

    def distance_from_origin(self):
        return math.hypot(self.x, self.y)


# ---------------------------------------------------------------------------
# The lemniscate
# ---------------------------------------------------------------------------
#
#   (x^2 + y^2)^2 = a^2 (x^2 - y^2)
#
# parametrised as
#
#   x(t) = a cos t / (1 + sin^2 t)
#   y(t) = a sin t cos t / (1 + sin^2 t)
#
# With s = sin t, c = cos t, D = 1 + s^2:
#
#   x'  = -a s (3 - s^2) / D^2
#   y'  =  a (1 - 3 s^2) / D^2
#   x'' = -a c (3 - 12 s^2 + s^4) / D^3
#   y'' = -2 a s c (5 - 3 s^2) / D^3
#
# Curvature and the differential-drive mapping:
#
#   kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^{3/2}
#   w     = kappa * v
#   vL    = v - w B/2 ,   vR = v + w B/2
#
# kappa = 0 at the crossing node (t = pi/2) and |kappa|max = 3/a at the two
# tips, so Rmin = a/3 = 16.7 cm >> B/2 = 7.5 cm: the inner wheel never has to
# reverse.  Starting at the node means the robot begins straight, with no
# step change in wheel speed.
# ---------------------------------------------------------------------------

def _lemni_deriv(t, a):
    """Return (x', y', x'', y'') of the lemniscate at parameter t."""
    s = math.sin(t)
    c = math.cos(t)
    ss = s * s
    D = 1.0 + ss
    D2 = D * D
    D3 = D2 * D
    xp = -a * s * (3.0 - ss) / D2
    yp = a * (1.0 - 3.0 * ss) / D2
    xpp = -a * c * (3.0 - 12.0 * ss + ss * ss) / D3
    ypp = -2.0 * a * s * c * (5.0 - 3.0 * ss) / D3
    return xp, yp, xpp, ypp


def _lemni_speed(t, a):
    """|dr/dt| -- the parameter speed, in cm per unit t."""
    xp, yp, _, _ = _lemni_deriv(t, a)
    return math.hypot(xp, yp)


def lemniscate(a=LEMNI_A, dt=LEMNI_DT, laps=1, use_gyro=True,
               kp=LEMNI_KP, verbose=True):
    """
    Drive one (or `laps`) full lemniscate.

    use_gyro=False gives the pure feedforward result -- use that one to
    measure and report the open-loop closure error.
    use_gyro=True adds proportional heading correction, which regulates away
    the accumulated calibration error and actually closes the figure.
    """
    half_base = BASE_WIDTH / 2.0

    # Speed budget.  The outer wheel runs at v * (1 + |kappa|max * B/2) and
    # |kappa|max = 3/a, so pick v so the outer wheel never saturates.  Leave
    # headroom for the gyro correction term on top of that.
    v_wheel_max = LEMNI_SPEED_FRAC * _max_dps() * math.pi / 180.0 * WHEEL_RADIUS
    v_path = v_wheel_max / (1.0 + 3.0 * half_base / a)
    if use_gyro:
        v_path *= 0.85                      # headroom for the correction term

    dps_limit = _max_dps()

    t0 = math.pi / 2.0                      # the crossing node: kappa = 0
    t = t0
    t_end = t0 + laps * 2.0 * math.pi
    theta_ref = 0.0                         # integrated reference heading

    odo = Odometry()
    periods = []
    saturations = 0

    _set_fast_response(True)
    if use_gyro:
        gyro.reset()
        time.sleep(0.05)
    odo.reset()

    next_tick = time.time()
    prev = next_tick

    try:
        while t < t_end:
            xp, yp, xpp, ypp = _lemni_deriv(t, a)
            sp = math.hypot(xp, yp)
            kappa = (xp * ypp - yp * xpp) / (sp ** 3)

            v = v_path
            w = kappa * v                   # feedforward angular velocity

            if use_gyro:
                err = _wrap(theta_ref - _heading())
                w += kp * err               # proportional heading correction

            vl = v - w * half_base
            vr = v + w * half_base

            dl = _dps(vl)
            dr = _dps(vr)
            if abs(dl) > dps_limit or abs(dr) > dps_limit:
                saturations += 1
                scale = dps_limit / max(abs(dl), abs(dr))
                dl *= scale
                dr *= scale

            robot_drive.on(SpeedDPS(dl), SpeedDPS(dr))

            # --- wait out the control period -------------------------------
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.time()))

            # --- advance the model using the time that ACTUALLY elapsed -----
            # Assuming a perfect dt here is the single biggest source of drift
            # on the EV3: one iteration writes four sysfs attributes and does
            # the trig in Python, which regularly overruns 50 ms.
            now = time.time()
            dt_real = now - prev
            prev = now
            periods.append(dt_real)

            # Midpoint (RK2) step in the parameter, so that the robot covers a
            # constant arc length v*dt per step.  Plain forward Euler here
            # accumulates an O(dt) error in total path length, i.e. the figure
            # closes short or long.
            ds = v * dt_real
            t_mid = t + 0.5 * ds / sp
            t += ds / _lemni_speed(t_mid, a)

            theta_ref += w * dt_real
            odo.update()

    finally:
        robot_drive.off(brake=True)
        _set_fast_response(False)

    odo.update()

    if verbose and periods:
        mean_dt = sum(periods) / len(periods)
        print("LEMNISCATE complete  (a = %.1f cm, v = %.1f cm/s, %s)"
              % (a, v_path, "gyro" if use_gyro else "open loop"))
        print("  steps            : %d" % len(periods))
        print("  control period   : mean %.1f ms, max %.1f ms (nominal %.0f)"
              % (1000.0 * mean_dt, 1000.0 * max(periods), 1000.0 * dt))
        print("  speed saturations: %d" % saturations)
        print("  odometry close   : dx %.1f  dy %.1f  |d| %.1f cm"
              % (odo.x, odo.y, odo.distance_from_origin()))
        print("  odometry heading : %.1f deg (ideal %.0f)"
              % (math.degrees(odo.th), 360.0 * laps))
        if use_gyro:
            print("  gyro heading     : %.1f deg" % math.degrees(_heading()))

    return odo


def lemniscate_open_loop(a=LEMNI_A):
    """Feedforward only -- the number to quote as the open-loop error."""
    return lemniscate(a=a, use_gyro=False)


def lemniscate_two_arc(a=LEMNI_A):
    """
    Minimum implementation: two tangent circles of radius a/2 traversed in
    opposite senses.  Kept as a fallback and as a baseline for the report.
    """
    R = a / 2.0
    r_out = R + BASE_WIDTH / 2.0
    r_in = R - BASE_WIDTH / 2.0
    rot = r_out / WHEEL_RADIUS
    inner = OUTER_SPEED * (r_in / r_out)

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED),
                                 SpeedPercent(inner), rotations=rot)
    robot_drive.on_for_rotations(SpeedPercent(inner),
                                 SpeedPercent(OUTER_SPEED), rotations=rot)
    print("LEMNISCATE (two-arc) complete")


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def calibrate_wheel(distance=200.0):
    """
    Drive a commanded straight distance; you measure the real one with a tape.

    The nominal 5.6 cm is the moulded diameter; under load the rolling radius
    is typically 1-2 % smaller, which shows up as a pure scale error.
    """
    print("Driving a commanded %.0f cm in a straight line..." % distance)
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED),
                                 SpeedPercent(OUTER_SPEED),
                                 rotations=distance / CIRCUMFERENCE)
    robot_drive.off(brake=True)
    try:
        measured = float(input("Measured distance [cm]: ").strip())
    except ValueError:
        print("  not a number, skipping")
        return
    print("  WHEEL_DIAMETER = %.3f   (currently %.3f)"
          % (WHEEL_DIAMETER * measured / distance, WHEEL_DIAMETER))


def calibrate_base(turns=5):
    """
    Spin in place N full turns and compare the gyro against the commanded
    angle to recover the EFFECTIVE track width.

    BASE_WIDTH enters w = (vR - vL)/B linearly, and one lemniscate lobe turns
    a net 360 deg, so a 3 % error in B is ~11 deg of heading error at the
    node -- easily 8 cm of miss.  Use several turns so the per-turn error is
    well above gyro noise.
    """
    rot = turns * BASE_WIDTH / WHEEL_DIAMETER
    gyro.reset()
    time.sleep(0.1)
    robot_drive.on_for_rotations(SpeedPercent(-15), SpeedPercent(15),
                                 rotations=rot, brake=True)
    time.sleep(0.5)
    measured = abs(gyro.angle)
    commanded = 360.0 * turns
    print("  commanded %.0f deg, gyro measured %.0f deg" % (commanded, measured))
    if measured > 1.0:
        print("  BASE_WIDTH = %.3f   (currently %.3f)"
              % (BASE_WIDTH * measured / commanded, BASE_WIDTH))


def check_loop_rate(n=200):
    """
    Time the control loop without driving, to size dt honestly for the report.
    """
    _set_fast_response(True)
    periods = []
    prev = time.time()
    try:
        for i in range(n):
            t = math.pi / 2.0 + i * 0.01
            xp, yp, xpp, ypp = _lemni_deriv(t, LEMNI_A)
            sp = math.hypot(xp, yp)
            kappa = (xp * ypp - yp * xpp) / (sp ** 3)
            w = kappa * 10.0
            robot_drive.on(SpeedDPS(_dps(10.0 - w * BASE_WIDTH / 2.0)),
                           SpeedDPS(_dps(10.0 + w * BASE_WIDTH / 2.0)))
            now = time.time()
            periods.append(now - prev)
            prev = now
    finally:
        robot_drive.off(brake=True)
        _set_fast_response(False)

    periods = periods[1:]
    mean = sum(periods) / len(periods)
    print("  bare loop: mean %.1f ms, min %.1f, max %.1f"
          % (1000 * mean, 1000 * min(periods), 1000 * max(periods)))
    print("  -> choose LEMNI_DT >= %.2f s" % (math.ceil(mean * 1000 * 1.5) / 1000.0))


# ---------------------------------------------------------------------------
# The other shapes (unchanged from the rest of the lab)
# ---------------------------------------------------------------------------

def straight_line():
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED),
                                 SpeedPercent(OUTER_SPEED),
                                 rotations=40 / CIRCUMFERENCE)
    print("STRAIGHT LINE complete")


def circle(radius=50.0):
    r_outer = radius + BASE_WIDTH / 2.0
    r_inner = radius - BASE_WIDTH / 2.0
    inner_speed = OUTER_SPEED * (r_inner / r_outer)
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED),
                                 SpeedPercent(inner_speed),
                                 rotations=r_outer / WHEEL_RADIUS)
    print("CIRCLE complete")


def turn_90_dumb():
    turn_rotations = BASE_WIDTH / (4.0 * WHEEL_DIAMETER)
    start_angle = gyro.angle

    robot_drive.on_for_rotations(SpeedPercent(-OUTER_SPEED / 3),
                                 SpeedPercent(OUTER_SPEED / 3),
                                 rotations=turn_rotations, brake=True)

    while abs(gyro.angle - start_angle) < TARGET_ANGLE:
        robot_drive.on(SpeedPercent(-OUTER_SPEED / 6),
                       SpeedPercent(OUTER_SPEED / 6))
        time.sleep(0.01)

    robot_drive.off(brake=True)
    print("Turned:", abs(gyro.angle - start_angle))
    time.sleep(0.2)


def rectangle():
    length_rotations = 100 / CIRCUMFERENCE
    width_rotations = 50 / CIRCUMFERENCE
    for rot in (length_rotations, width_rotations,
                length_rotations, width_rotations):
        robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED),
                                     SpeedPercent(OUTER_SPEED), rotations=rot)
        turn_90_dumb()
    print("RECTANGLE complete")


# ---------------------------------------------------------------------------
# Menu
# ---------------------------------------------------------------------------

def main():
    while True:
        print("")
        print("Select an action:")
        print("  1: Straight line")
        print("  2: Circle")
        print("  3: Rectangle")
        print("  4: Lemniscate")
        print("  5: Turn 90")
        print("  q: Quit")

        choice = input("Enter choice: ").strip().lower()

        if choice == '1':
            straight_line()
        elif choice == '2':
            circle()
        elif choice == '3':
            rectangle()
        elif choice == '4':
            lemniscate()
        elif choice == '5':
            turn_90_dumb()
        elif choice == 'q':
            print("Exiting...")
            break
        else:
            print("Invalid selection. Please choose 1-5 or q.")


if __name__ == '__main__':
    try:
        main()
    finally:
        robot_drive.off(brake=True)
        _set_fast_response(False)