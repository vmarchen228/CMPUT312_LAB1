#!/usr/bin/env python3
import time, math
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_B
from ev3dev2.motor import SpeedPercent
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor import INPUT_1

# Constants & Setup
WHEEL_DIAMETER = 5.6
BASE_WIDTH = 15.0  
OUTER_SPEED = 50 / 2
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

robot_drive = MoveTank(OUTPUT_A, OUTPUT_B)
gyro = GyroSensor(INPUT_1)
gyro.reset()

def turn_90():
    gyro.reset()
    robot_drive.on(SpeedPercent(OUTER_SPEED), SpeedPercent(-OUTER_SPEED))
    while abs(gyro.angle) < 80:
        time.sleep(0.01)
    robot_drive.off(brake=True)

def straight_line():
    robot_drive.on_for_rotations(
        SpeedPercent(OUTER_SPEED), 
        SpeedPercent(OUTER_SPEED), 
        rotations=100 / CIRCUMFERENCE
    )
    print("STRAIGHT LINE complete")

def circle(): 
    r_outer = 50 + (BASE_WIDTH / 2.0)
    r_inner = 50 - (BASE_WIDTH / 2.0)

    speed_ratio = r_inner / r_outer
    inner_speed = OUTER_SPEED * speed_ratio

    wheel_radius = WHEEL_DIAMETER / 2.0
    required_rotations = r_outer / wheel_radius

    robot_drive.on_for_rotations(
        SpeedPercent(OUTER_SPEED), 
        SpeedPercent(inner_speed), 
        rotations=required_rotations
    )
    print("CIRCLE complete")

def rectangle():
    length_rotations = 100 / CIRCUMFERENCE
    width_rotations = 50 / CIRCUMFERENCE

    for _ in range(2):
        robot_drive.on_for_rotations(
            SpeedPercent(OUTER_SPEED), 
            SpeedPercent(OUTER_SPEED), 
            rotations=length_rotations
        )
        turn_90()
        
        robot_drive.on_for_rotations(
            SpeedPercent(OUTER_SPEED), 
            SpeedPercent(OUTER_SPEED), 
            rotations=width_rotations
        )
        turn_90()

    print("RECTANGLE complete")

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
        elif choice.lower() == 'q':
            print("Exiting...")
            break
        else:
            print("Invalid selection. Please choose 1, 2, 3, or q.")

if __name__ == '__main__':
    main()