#!/usr/bin/env python
from context import andrew_robot
import time
import math
from andrew_robot import AndrewRobot

# --- ANDREW ROBOT KINEMATIC CONSTANTS ---
# Taken from D:\Resources\andrew.xml
L1 = 152.13  # Length from Shoulder to Elbow (mm)
L2 = 151.67  # Length from Elbow to Wrist (mm)

TICKS_PER_RADIAN = 4096 / (2 * math.pi)
CENTER_TICK = 2048

# Approximate factory calibration offsets based on typical dynamixel alignments
# (Adjust these if the orientation is physically skewed)
OFFSET_THETA1 = 0
OFFSET_THETA2 = 0

def forward_kinematics(ticks_shoulder, ticks_elbow):
    """ Converts joint ticks back into physical X, Y millimeters """
    theta1 = (ticks_shoulder - CENTER_TICK) / TICKS_PER_RADIAN + OFFSET_THETA1
    theta2 = (ticks_elbow - CENTER_TICK) / TICKS_PER_RADIAN + OFFSET_THETA2
    
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y

def inverse_kinematics(x, y, elbow_up=True):
    """ Converts physical X, Y millimeters into Dynamixel servo ticks """
    distance_sq = x**2 + y**2
    
    # Check if the requested coordinate is physically reachable by the arm lengths
    if distance_sq > (L1 + L2)**2 or distance_sq < (L1 - L2)**2:
        raise ValueError(f"Target (X={x:.1f}, Y={y:.1f}) is out of physical reach!")
        
    # Standard 2-Bone Inverse Kinematics math (Law of Cosines)
    cos_theta2 = (distance_sq - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Clip to bounds [-1, 1] to prevent floating point inaccuracies crashing acos
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.acos(cos_theta2)
    
    if not elbow_up:
        theta2 = -theta2
        
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    # Translate math angles back to standard Dynamixel ticks
    ticks_shoulder = int((theta1 - OFFSET_THETA1) * TICKS_PER_RADIAN + CENTER_TICK)
    ticks_elbow = int((theta2 - OFFSET_THETA2) * TICKS_PER_RADIAN + CENTER_TICK)
    
    return ticks_shoulder, ticks_elbow

def main():
    print("Initializing robot with Inverse Kinematics solver...")
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 40
    robot.linear.moving_speed = 50
    time.sleep(0.1)

    print("Homing robot to a safe folded position...")
    robot.move_arm_servos(shoulder=1500, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    time.sleep(0.5)

    # Pick a starting coordinate (X, Y) roughly in front of the robot. 
    start_x = 150.0
    start_y = 180.0
    
    # Use IK to calculate where the servos need to move to hit that specific X/Y point
    start_shoulder, start_elbow = inverse_kinematics(start_x, start_y, elbow_up=True)
    print(f"Calculated starting ticks -> Shoulder: {start_shoulder}, Elbow: {start_elbow}")
    
    print(f"Moving to absolute coordinates (X={start_x}mm, Y={start_y}mm)...")
    robot.move_arm_servos(shoulder=start_shoulder, elbow=start_elbow, linear=robot.SAFE_HEIGHT)
    time.sleep(1.5)

    print("Drawing a perfectly straight 100mm line strictly along the X-axis!")
    # Keep Y perfectly constant, only increment X
    for step in range(0, 105, 5):
        current_x = start_x + step
        
        try:
            # Continuously solve for the changing joint angles as we move linearly
            s_ticks, e_ticks = inverse_kinematics(current_x, start_y, elbow_up=True)
            print(f"  Target [X={current_x:.1f}, Y={start_y:.1f}] -> Servos [{s_ticks}, {e_ticks}]")
            
            # Write without full wait/staging so it glides smoothly
            robot.shoulder.set_goal_position(s_ticks)
            robot.elbow.set_goal_position(e_ticks)
            time.sleep(0.05) # Small pacing delay for smooth continuous motion
            
        except ValueError as e:
            print(f"WARNING: {e}")
            break

    print("Linear sweep complete! Homing...")
    time.sleep(0.5)
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)

if __name__ == '__main__':
    main()
