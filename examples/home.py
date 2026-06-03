#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    print("Connecting to robot...")
    try:
        robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
        robot.max_speed = 30
        
        print("Positions before:", robot.get_servo_positions())

        # FIX: Ensure torque is fully enabled first before attempting to stage coordinate moves
        robot.enable_torque()

        # Define home coordinates
        HOME_SHOULDER = 1100
        HOME_ELBOW = 1500
        HOME_WRIST = 1000

        # Check if the arm is already near the home position
        positions = robot.get_servo_positions()
        if (abs(positions[0] - HOME_SHOULDER) < 20 and 
            abs(positions[1] - HOME_ELBOW) < 20 and 
            abs(positions[2] - HOME_WRIST) < 20):
            print("Arm is already at home. Stretching out to the right first...")
            # Stretch straight out
            robot.move_arm_servos(shoulder=2048, elbow=2048, wrist=2048)
            time.sleep(0.5)
            
            print("Going all the way up...")
            # 500 is a very high Z-axis coordinate (lower value = physically higher)
            robot.move_arm_servos(linear=500)
            time.sleep(1)

            print("Going all the way down...")
            # 2500 is much lower than the standard GRAB_HEIGHT (2035). 
            # Larger numbers = further down. 
            robot.move_arm_servos(linear=2500)
            time.sleep(1)

        print("Homing robot to a safe, folded standby position...")
        # First pull linear up to avoid collisions
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        
        # Then fold the shoulder, elbow, and wrist into a compact resting pose
        # (Using safe coordinates derived from typical slot positions to keep the arm tucked in)
        robot.move_arm_servos(shoulder=HOME_SHOULDER, elbow=HOME_ELBOW, wrist=HOME_WRIST)
        
        # Release the gripper and reset the thumb to neutral as a baseline reset
        robot.open_gripper()
        robot.thumb_neutral()
        
        print("Homing complete!")
    except Exception as e:
        print(f"\nCaught an error: {e}")
        print("\n*** IMPORTANT: HARDWARE OVERLOAD DETECTED ***")
        print("If the robot threw an 'Overload error!', it means the gripper motor squeezed too hard and shut itself down for safety.")
        print("To clear this hardware lock, you MUST switch the robot's physical power OFF, wait 5 seconds, and switch it back ON.")
        print("Once the robot is powered back on, please run this script again.")

if __name__ == '__main__':
    main()