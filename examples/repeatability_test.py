import sys
import os
import time

# Ensure we can import andrew_robot
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from andrew_robot.robot import AndrewRobot

def main():
    print("Initializing robot for Repeatability Test...")
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 40
    robot.enable_torque()
    
    # Check torque was verified
    print("Torque enabled successfully.")
    
    # 1. Mandatory Homing Sequence
    print("1. Running pre-run homing sequence...")
    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
    time.sleep(1)
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
    time.sleep(2)
    
    # Pick a random safe position inside the boundary mapping
    # Using inverse_kinematics values equivalent (approx):
    target_s = 1500
    target_e = 1500
    target_w = 1000
    target_z = 2500
    
    print(f"\n2. Starting repeatability test for 50 cycles to [S:{target_s}, E:{target_e}, Z:{target_z}]...")
    
    logged_data = []
    
    for cycle in range(1, 51):
        # Move away to a randomish/neutral corner
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        time.sleep(1.0)
        
        # Move back using Backlash Compensation (X-, Y-, Z+ Approach)
        robot.move_arm_servos(shoulder=target_s - 25, elbow=target_e - 25, wrist=target_w)
        time.sleep(0.5)
        robot.move_arm_servos(shoulder=target_s, elbow=target_e, wrist=target_w)
        time.sleep(1.0)
        
        # Approach from above securely
        robot.move_arm_servos(linear=target_z + 100)
        time.sleep(0.3)
        robot.move_arm_servos(linear=target_z)
        time.sleep(1.0)
        
        # Record Encoder Values
        actual_pos = robot.get_servo_positions()
        
        # If camera tracking was here, we would add the deviation delta:
        camera_delta = "(Pending Camera Verification)"
        
        print(f"Cycle {cycle}/50 -> Actual Ticks: [S:{actual_pos[0]}, E:{actual_pos[1]}, W:{actual_pos[2]}, Z:{actual_pos[3]}] | Camera Delta: {camera_delta}")
        logged_data.append(actual_pos)
        
    print("\nTest Complete! Homing...")
    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
    time.sleep(1)
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
    time.sleep(2)

if __name__ == '__main__':
    main()