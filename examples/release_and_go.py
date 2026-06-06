#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    print("Initializing robot...")
    # Initialize the Robot with your correct ports
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 30
    robot.linear.moving_speed = 50
    
    # Wait for the LED controller to initialize
    time.sleep(0.1)

    print("Opening gripper to release whatever it is holding...")
    robot.open_gripper()
    
    # Optionally also release the thumb just in case
    robot.thumb_neutral()
    
    # Wait a moment to ensure it fully unlatches
    time.sleep(1)

    print("Moving arm away safely (Go!)...")
    # Home the robot to a safe, folded up position
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    
    print("Release and go sequence finished!")

if __name__ == '__main__':
    main()
