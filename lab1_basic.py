#!/usr/bin/env python3
import time, math
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_B
from ev3dev2.motor import SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1

gyro = GyroSensor(INPUT_1)
gyro.reset()

robot_drive = MoveTank(OUTPUT_A, OUTPUT_B)

WHEEL_DIAMETER = 5.6
BASE_WIDTH = 15.0  
OUTER_SPEED = 30

CIRCUMFERENCE = 2 * (WHEEL_DIAMETER / 2.0) * 3.1416

required_rotations = 0.0

def straight_line():
    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=100 / CIRCUMFERENCE) 

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
    quarter_circumference = (BASE_WIDTH / 2 * math.pi)

    def turn_90(target=75):
        robot_drive.off(brake=True)
        time.sleep(0.2)                      # let the robot settle
        start = gyro.angle                   # remember where we are
        robot_drive.on(SpeedPercent(OUTER_SPEED), SpeedPercent(-OUTER_SPEED))
        while abs(gyro.angle - start) < target:   # stop a bit early for overshoot
            time.sleep(0.005)
        robot_drive.off(brake=True)
        print("Turned:", abs(gyro.angle - start), "degrees")

    length_rotations = 50 / CIRCUMFERENCE
    width_rotations = 25 / CIRCUMFERENCE

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=length_rotations)
    # time.sleep(0.1)
    turn_90()

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=width_rotations)
    # time.sleep(0.1)
    turn_90()

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=length_rotations)
    # time.sleep(0.1)
    turn_90()

    robot_drive.on_for_rotations(SpeedPercent(OUTER_SPEED), SpeedPercent(OUTER_SPEED), rotations=width_rotations)
    # time.sleep(0.1)
    turn_90()


    print("RECTANGLE complete")

def lemniscate():
    pass

def main():
    while True:
        print("\nSelect an action:")
        print("1: Straight Line")
        print("2: Circle")
        print("3: Rectangle")
        print("q: Quit")
        
        choice = input("Enter choice: ").strip()

        if choice == '1':
            straight_line()
        elif choice == '2':
            circle()
        elif choice == '3':
            rectangle()
        elif choice.lower() == '5':
            turn_90()
        elif choice.lower() == 'q':
            print("Exiting...")
            break
        
        else:
            print("Invalid selection. Please choose 1, 2, 3, or q.")

if __name__ == '__main__':
    main()
