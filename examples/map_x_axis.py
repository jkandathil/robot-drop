#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    print("Initializing robot for X-axis (Shoulder) mapping...")
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 30
    robot.linear.moving_speed = 50

    # Wait for LED controller to initialize
    time.sleep(0.1)

    print("Moving linear track to safe height and centering arm...")
    # Safe height prevents crashing into pipette racks. 1500 is roughly center.
    robot.move_arm_servos(shoulder=1500, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    time.sleep(1)

    # Moving the shoulder joint physically sweeps the arm left and right
    # Values between 1000 and 2000 are the standard working bounds for the table
    print("Sweeping shoulder (pseudo X-axis) from 1000 to 2000...")
    for shoulder_pos in range(1000, 2001, 100):
        print(f"Moving to shoulder position: {shoulder_pos}")
        robot.move_arm_servos(shoulder=shoulder_pos, linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)

    print("Sweeping back...")
    for shoulder_pos in range(2000, 999, -100):
        print(f"Moving to shoulder position: {shoulder_pos}")
        robot.move_arm_servos(shoulder=shoulder_pos, linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)

    print("Homing robot to a safe folded position...")
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    print("Mapping complete!")

if __name__ == '__main__':
    main()
