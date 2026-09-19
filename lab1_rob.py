#!/usr/bin/env python3
import time, math
from ev3dev2.motor import MoveTank, LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, SpeedDPS
from ev3dev2.motor import SpeedPercent
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

required_rotations = 0.0

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

LEMNI_A = 50.0      # cm  -> a = 0.5 m
LEMNI_DT = 0.05     # s   control period (20 Hz)

def _dps(v_cm_s):
    """linear wheel speed [cm/s] -> motor speed [deg/s]"""
    return v_cm_s / (WHEEL_DIAMETER / 2.0) * 180.0 / math.pi

def lemniscate(a=LEMNI_A, dt=LEMNI_DT, laps=1, use_gyro=False):
    """
    Bernoulli lemniscate (x^2+y^2)^2 = a^2 (x^2-y^2), driven open-loop from the
    parametric curve with time-varying curvature. Starts at the crossing node.
    """
    wheel_radius = WHEEL_DIAMETER / 2.0
    half_base = BASE_WIDTH / 2.0

    # speed budget: outer wheel runs v*(1 + kappa_max*B/2), kappa_max = 3/a
    max_dps = min(left_motor.max_speed, right_motor.max_speed)
    v_wheel_max = (OUTER_SPEED / 100.0) * max_dps * math.pi / 180.0 * wheel_radius
    v_path = v_wheel_max / (1.0 + 3.0 * half_base / a)

    def deriv(t):
        s, c = math.sin(t), math.cos(t)
        D = 1.0 + s * s
        xp  = -a * s * (3.0 - s * s) / D**2
        yp  =  a * (1.0 - 3.0 * s * s) / D**2
        xpp = -a * c * (3.0 - 12.0 * s * s + s**4) / D**3
        ypp = -2.0 * a * s * c * (5.0 - 3.0 * s * s) / D**3
        return xp, yp, xpp, ypp

    # a 2 s ramp would completely swamp a 20 Hz control loop
    saved = (left_motor.ramp_up_sp, left_motor.ramp_down_sp,
             right_motor.ramp_up_sp, right_motor.ramp_down_sp)
    left_motor.ramp_up_sp = left_motor.ramp_down_sp = 0
    right_motor.ramp_up_sp = right_motor.ramp_down_sp = 0

    t0 = math.pi / 2.0                  # node: kappa = 0, smooth start
    t = t0
    t_end = t0 + laps * 2.0 * math.pi
    theta_ref = 0.0                     # integrated reference heading [rad]
    if use_gyro:
        gyro.reset()

    next_tick = time.time()
    try:
        while t < t_end:
            xp, yp, xpp, ypp = deriv(t)
            sp = math.hypot(xp, yp)                       # |dr/dt|, cm per unit t
            kappa = (xp * ypp - yp * xpp) / sp**3

            v = v_path
            w = kappa * v                                 # rad/s, +ve = CCW

            if use_gyro:
                # EV3 gyro is +ve clockwise -> negate to match the math frame
                theta_meas = -math.radians(gyro.angle)
                err = math.atan2(math.sin(theta_ref - theta_meas),
                                 math.cos(theta_ref - theta_meas))
                w += 1.5 * err                            # P gain, rad/s per rad

            vl = v - w * half_base
            vr = v + w * half_base
            robot_drive.on(SpeedDPS(_dps(vl)), SpeedDPS(_dps(vr)))

            # advance by arc length so |v| stays constant along the curve
            t += (v * dt) / sp
            theta_ref += w * dt

            next_tick += dt
            time.sleep(max(0.0, next_tick - time.time()))
    finally:
        robot_drive.off(brake=True)
        (left_motor.ramp_up_sp, left_motor.ramp_down_sp,
         right_motor.ramp_up_sp, right_motor.ramp_down_sp) = saved

    print("LEMNISCATE complete (a = %.1f cm, v = %.1f cm/s)" % (a, v_path))


def lemniscate_two_arc(a=LEMNI_A):
    """Fallback / minimum implementation: two tangent circles of radius a/2."""
    R = a / 2.0
    r_out, r_in = R + BASE_WIDTH / 2.0, R - BASE_WIDTH / 2.0
    rot = r_out / (WHEEL_DIAMETER / 2.0)
    inner = OUTER_SPEED * (r_in / r_out)
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(inner), rotations=rot)
    robot_drive.on_for_rotations(SpeedPercent(inner), SpeedPercent(OUTER_SPEED), rotations=rot)
    print("LEMNISCATE (two-arc) complete")

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
        print("q: Quit")
        
        choice = input("Enter choice: ").strip()

        if choice == '1':
            straight_line()
        elif choice == '2':
            circle()
        elif choice == '3':
            rectangle()
        elif choice == '4':
            lemniscate()
        elif choice.lower() == '5':
            turn_90_dumb()
        elif choice.lower() == 'q':
            print("Exiting...")
            break
        
        else:
            print("Invalid selection. Please choose 1, 2, 3, or q.")

if __name__ == '__main__':
    main()