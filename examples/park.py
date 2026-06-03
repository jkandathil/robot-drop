#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    print("Connecting to robot for shutdown sequence...")
    try:
        robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
        robot.max_speed = 20 # Move a bit slower for parking
        robot.enable_torque()

        # Fold the arm first so it aligns safely over the base
        print("Folding arm inward...")
        robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        time.sleep(0.5)
        
        # Lower the arm gradually to the very bottom
        print("Lowering arm to the bottom rest position...")
        # 3125 is the physical bottom rest limit of the linear track
        robot.move_arm_servos(linear=3125)
        
        # Give it an extra second to settle
        time.sleep(1)
        
        # Turn off the motors in software so it goes limp gently 
        # before you hit the hard power switch
        print("Disabling motor torque...")
        robot.disable_torque()
        
        print("\nThe robot is now safely parked.")
        print("You may now turn off the physical power switch without the arm dropping!")
        
    except Exception as e:
        print(f"Error during parking: {e}")

if __name__ == '__main__':
    main()