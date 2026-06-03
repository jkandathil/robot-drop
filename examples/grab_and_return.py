#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    # Initialize the Robot with your correct ports
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 30
    robot.linear.moving_speed = 50
    
    # Wait for the LED controller to initialize
    time.sleep(0.1)

    print("Homing robot to a safe folded position...")
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    robot.open_gripper()

    target_slot = 3
    print(f"Grabbing pipette from slot {target_slot}...")
    robot.grab_pipette(slot_index=target_slot)
    
    print("Pipette secured!")
    
    # Move away to arbitrary coordinates
    print("Moving the pipette away to present it to the user...")
    # These are arbitrary Joint values moving the arm outward safely
    robot.move_arm_servos(shoulder=1871, elbow=1905, wrist=1977, linear=robot.SAFE_HEIGHT)
    time.sleep(2)
    
    # --- RETURN PIPETTE LOGIC ---
    print(f"Returning pipette to slot {target_slot}...")
    # Get the slot coordinate data
    slot = robot.config.pipette_slots[f'slot{target_slot}']
    
    # Move to the slot's pre-entry "start" position, but stay high up
    robot.move_servos_proportional(*slot.start_position, linear=robot.SAFE_HEIGHT)
    
    # Drop down to grab height from the start position
    robot.move_servos_proportional(*slot.start_position, linear=robot.GRAB_HEIGHT)
    
    # Slide directly into the release position holder
    robot.move_servos_proportional(*slot.release_position)
    
    # Open the gripper to unlatch the pipette
    robot.open_gripper()
    
    # Slide backwards to the start position to release it completely
    robot.move_servos_proportional(*slot.start_position)
    
    # Pull the arm straight up to a safe height
    robot.move_servos_proportional(linear=robot.SAFE_HEIGHT)
    
    print("Pipette successfully returned!")

if __name__ == '__main__':
    main()
