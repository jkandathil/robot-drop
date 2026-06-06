#!/usr/bin/env python
from context import andrew_robot
import time
import math
from andrew_robot import AndrewRobot

# --- ANDREW ROBOT KINEMATIC CONSTANTS ---
L1 = 152.13  # Length from Shoulder to Elbow (mm)
L2 = 151.67  # Length from Elbow to Wrist (mm)
MAX_REACH = L1 + L2

TICKS_PER_RADIAN = 4096 / (2 * math.pi)
CENTER_TICK = 2048

def inverse_kinematics(x, y, elbow_up=True):
    """ Converts physical X, Y millimeters into Dynamixel servo ticks """
    distance_sq = x**2 + y**2
    if distance_sq > (MAX_REACH - 1)**2 or distance_sq < (L1 - L2)**2:
        raise ValueError(f"Target (X={x:.1f}, Y={y:.1f}) is out of physical reach!")
        
    cos_theta2 = (distance_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.acos(cos_theta2)
    
    if not elbow_up:
        theta2 = -theta2
        
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    # Translate to ticks
    ticks_shoulder = int(theta1 * TICKS_PER_RADIAN + CENTER_TICK)
    ticks_elbow = int(theta2 * TICKS_PER_RADIAN + CENTER_TICK)
    
    return ticks_shoulder, ticks_elbow

def main():
    print("Initializing robot...")
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 30
    robot.linear.moving_speed = 50
    time.sleep(0.1)

    print("Homing robot to a safe folded position...")
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    time.sleep(1)

    # Define a safe rectangular working area on the table.
    # X = Forward distance from the base. Y = Left/Right distance.
    X_NEAR = 130.0   # Closer to the robot base
    X_FAR = 240.0    # Reaching out towards the user
    Y_LEFT = 150.0   # Reaching to the left parameter
    Y_RIGHT = -150.0 # Reaching to the right parameter
    
    corners = [
        ("Near Left", X_NEAR, Y_LEFT),
        ("Far Left", X_FAR, Y_LEFT),
        ("Far Right", X_FAR, Y_RIGHT),
        ("Near Right", X_NEAR, Y_RIGHT),
        ("Center of Desk", (X_NEAR+X_FAR)/2, 0.0)
    ]

    print("\nStarting Working Area Perimeter Sweep:")
    for name, x, y in corners:
        s_ticks, e_ticks = inverse_kinematics(x, y, elbow_up=True)
        print(f" -> Moving to {name} corner (X={x}mm, Y={y}mm)... Ticks: [S:{s_ticks}, E:{e_ticks}]")
        
        # Use move_arm_servos so linear track stays safely high
        robot.move_arm_servos(shoulder=s_ticks, elbow=e_ticks, linear=robot.SAFE_HEIGHT)
        time.sleep(2) # Give it time to reach the destination

    print("\nMapping complete. Homing...")
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)

if __name__ == '__main__':
    main()
