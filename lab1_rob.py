#!/usr/bin/env python3
import time, math
from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1

gyro = GyroSensor(INPUT_1)
gyro.reset()

robot_drive = MoveTank(OUTPUT_A, OUTPUT_B)

left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_B)

left_motor.ramp_up_sp = 2000
left_motor.ramp_down_sp = 2000

right_motor.ramp_up_sp = 2000
right_motor.ramp_down_sp = 2000

WHEEL_DIAMETER = 5.6
BASE_WIDTH = 15.0
OUTER_SPEED = 30
TARGET_ANGLE = 85

CIRCUMFERENCE = 2 * (WHEEL_DIAMETER / 2.0) * math.pi

# ---- lemniscate settings --------------------------------------------------
LEMNI_A = 50.0       # cm, a = 0.5 m
LEMNI_V = 14.0       # cm/s along the path.  Lower = shakier (motor stiction),
                     # higher = more slip on the tips.  12-18 works well.
LEMNI_DT = 0.10      # s between speed updates (10 Hz)


def straight_line():
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=40 / CIRCUMFERENCE)

    print("STRAIGHT LINE complete")


def circle():
    r_outer = 50 + (BASE_WIDTH / 2.0)
    r_inner = 50 - (BASE_WIDTH / 2.0)

    speed_ratio = r_inner / r_outer
    inner_speed = OUTER_SPEED * speed_ratio

    wheel_radius = WHEEL_DIAMETER / 2.0
    required_rotations = r_outer / wheel_radius

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(inner_speed), rotations=required_rotations)

    print("CIRCLE complete")


def rectangle():
    length_rotations = 100 / CIRCUMFERENCE
    width_rotations = 50 / CIRCUMFERENCE

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=length_rotations)
    turn_90_dumb()

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=width_rotations)
    turn_90_dumb()

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=length_rotations)
    turn_90_dumb()

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=width_rotations)
    turn_90_dumb()

    print("RECTANGLE complete")


# ---------------------------------------------------------------------------
# Lemniscate of Bernoulli   (x^2+y^2)^2 = a^2 (x^2-y^2)
#
#   x(t) = a cos t / (1 + sin^2 t)
#   y(t) = a sin t cos t / (1 + sin^2 t)
#
# with s = sin t, D = 1 + s^2:
#   x'  = -a s (3 - s^2) / D^2
#   y'  =  a (1 - 3 s^2) / D^2
#   x'' = -a cos t (3 - 12 s^2 + s^4) / D^3
#   y'' = -2 a s cos t (5 - 3 s^2) / D^3
#
#   kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
#   w     = kappa * v
#   vL    = v - w * B/2        vR = v + w * B/2
#
# kappa = 0 at the crossing node (t = pi/2), |kappa|max = 3/a at the tips,
# so Rmin = a/3 = 16.7 cm, well clear of B/2 = 7.5 cm: the inner wheel never
# reverses and the robot starts and ends going straight.
# ---------------------------------------------------------------------------

def _lemni_curvature_and_speed(t, a):
    """Return (kappa, |dr/dt|) at parameter t."""
    s = math.sin(t)
    c = math.cos(t)
    ss = s * s
    D = 1.0 + ss

    xp = -a * s * (3.0 - ss) / (D * D)
    yp = a * (1.0 - 3.0 * ss) / (D * D)
    xpp = -a * c * (3.0 - 12.0 * ss + ss * ss) / (D * D * D)
    ypp = -2.0 * a * s * c * (5.0 - 3.0 * ss) / (D * D * D)

    sp = math.hypot(xp, yp)
    kappa = (xp * ypp - yp * xpp) / (sp * sp * sp)
    return kappa, sp


def lemniscate(a=LEMNI_A, v=LEMNI_V, dt=LEMNI_DT):
    wheel_radius = WHEEL_DIAMETER / 2.0
    half_base = BASE_WIDTH / 2.0
    max_sp = min(left_motor.max_speed, right_motor.max_speed)

    def dps(v_cm_s):
        return int(round(v_cm_s / wheel_radius * 180.0 / math.pi))

    def clamp(x):
        return max(-max_sp, min(max_sp, x))

    # Short ramp: long enough to smooth the 10 Hz steps, short enough that the
    # motors still track them.  The 2000 ms default would swallow them whole.
    left_motor.ramp_up_sp = left_motor.ramp_down_sp = 100
    right_motor.ramp_up_sp = right_motor.ramp_down_sp = 100

    t = math.pi / 2.0                     # start at the node, kappa = 0
    t_end = t + 2.0 * math.pi

    # Start both motors ONCE with run_forever.  After this we only write
    # speed_sp -- one sysfs attribute per wheel per update.  Re-issuing the
    # run-forever command every cycle is what makes the robot judder.
    left_motor.speed_sp = dps(v)
    right_motor.speed_sp = dps(v)
    left_motor.run_forever()
    right_motor.run_forever()
    time.sleep(0.3)                       # let it come up to speed first

    prev = time.time()
    try:
        while t < t_end:
            kappa, sp = _lemni_curvature_and_speed(t, a)
            w = kappa * v                 # rad/s, + = counter-clockwise

            left_motor.speed_sp = clamp(dps(v - w * half_base))
            right_motor.speed_sp = clamp(dps(v + w * half_base))

            time.sleep(dt)

            # advance using the time that ACTUALLY passed, not the nominal dt
            now = time.time()
            dt_real = now - prev
            prev = now

            # midpoint step so the robot covers a constant arc length v*dt
            ds = v * dt_real
            _, sp_mid = _lemni_curvature_and_speed(t + 0.5 * ds / sp, a)
            t += ds / sp_mid
    finally:
        left_motor.stop(stop_action='brake')
        right_motor.stop(stop_action='brake')
        left_motor.ramp_up_sp = left_motor.ramp_down_sp = 2000
        right_motor.ramp_up_sp = right_motor.ramp_down_sp = 2000

    print("LEMNISCATE complete")


def turn_90_dumb():
    turn_rotations = BASE_WIDTH / (4.0 * WHEEL_DIAMETER)

    start_angle = gyro.angle

    robot_drive.on_for_rotations(
        SpeedPercent(-OUTER_SPEED / 3),
        SpeedPercent(OUTER_SPEED / 3),
        rotations=turn_rotations,
        brake=True
    )

    while abs(gyro.angle - start_angle) < TARGET_ANGLE:
        robot_drive.on(
            SpeedPercent(-OUTER_SPEED / 6),
            SpeedPercent(OUTER_SPEED / 6)
        )
        time.sleep(0.01)

    robot_drive.off(brake=True)

    print("Turned:", abs(gyro.angle - start_angle))

    time.sleep(0.2)


def main():
    while True:
        print("\nSelect an action:")
        print("1: Straight Line")
        print("2: Circle")
        print("3: Rectangle")
        print("4: Lemniscate")
        print("5: Turn 90")
        print("q: Quit")

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
    main()