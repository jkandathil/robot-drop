#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    # 1. Initialize the Robot
    # You will need to change the COM ports and config path to match your 1000R setup
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 30
    robot.linear.moving_speed = 50
    robot.thumb.moving_speed = 70
    
    # Wait for the LED controller to initialize
    time.sleep(0.1)

    # 2. "Home" the robot
    # The API uses servo ticks (0-4095). 2048 is typically the center position.
    print("Homing robot to a safe folded position...")
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    robot.open_gripper()

    # 3. Grab a pipette
    # Grabs a pipette from slot 1 (modify 1-5 based on which slot you use)
    print("Grabbing pipette from slot 1...")
    robot.grab_pipette(slot_index=1)
    
    # Secure the pipette
    print("Ensure the pipette tip is attached.")
    robot.close_gripper()

    # --- DEFINE XYZ LOCATIONS IN JOINT SPACE ---
    # Note: Because the API uses servo encoder ticks instead of Cartesian X, Y, Z, 
    # you must define your physical XYZ location by its corresponding joint angles
    
    # Format: (Shoulder, Elbow, Wrist, Linear/Z-height)
    LOCATION_A_SOURCE = (1330, 1278, 1283, 1289)
    LOCATION_B_DEST   = (1841, 1061, 1017, 1289)

    # 4. Aspirate Liquid
    print("Moving to aspiration location...")
    # First move XY above the vial, keeping linear Z safely high
    robot.move_arm_servos(
        shoulder=LOCATION_A_SOURCE[0], 
        elbow=LOCATION_A_SOURCE[1], 
        wrist=LOCATION_A_SOURCE[2], 
        linear=robot.SAFE_HEIGHT
    )
    
    # Lower the arm into the vial
    robot.move_arm_servos(linear=LOCATION_A_SOURCE[3])

    # Pre-depress the plunger BEFORE going into the liquid (if doing forward pipetting)
    # Or aspirate while in the liquid (reverse pipetting - shown here)
    print("Aspirating...")
    robot.thumb_depress_first_position()
    time.sleep(0.5)
    robot.thumb_neutral()
    time.sleep(1.0) # Wait for liquid to fully enter the tip

    # Extract upwards safely
    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)


    # 5. Dispense Liquid
    print("Moving to dispense location...")
    # Move XY above the destination vial
    robot.move_arm_servos(
        shoulder=LOCATION_B_DEST[0], 
        elbow=LOCATION_B_DEST[1], 
        wrist=LOCATION_B_DEST[2], 
        linear=robot.SAFE_HEIGHT
    )

    # Lower the arm into the destination vial
    robot.move_arm_servos(linear=LOCATION_B_DEST[3])

    print("Dispensing...")
    # Go to first stop to dispense
    robot.thumb_depress_first_position()
    time.sleep(0.5)
    
    # Go to second stop (blowout) to expel any remaining droplets
    robot.thumb_depress_second_position()
    time.sleep(0.5)

    # Move completely out of the vial before releasing the thumb to avoid sucking liquid back up
    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
    robot.thumb_neutral()

    print("Homing robot...")
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    
    print("Routine Complete!")

if __name__ == '__main__':
    main()