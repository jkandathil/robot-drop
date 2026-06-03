#!/usr/bin/env python
from context import andrew_robot
import time
from andrew_robot import AndrewRobot

def main():
    # Initialize the Robot
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 30
    robot.linear.moving_speed = 50
    time.sleep(0.1)

    target_slot = 3
    slot = robot.config.pipette_slots[f'slot{target_slot}']

    print(f"Executing emergency return to slot {target_slot}...")
    
    # 1. Pull the arm all the way up to clear any obstacles FIRST
    print("Moving up to a safe height...")
    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)

    # 2. Swing the arm to be positioned directly in front of the slot 
    print("Swinging over to the slot's start position...")
    robot.move_arm_servos(shoulder=slot.start_position[0], 
                          elbow=slot.start_position[1], 
                          wrist=slot.start_position[2])

    # 3. Lower the arm to the correct height to slide the pipette in
    print("Lowering to grab height...")
    robot.move_arm_servos(linear=robot.GRAB_HEIGHT)

    # 4. Slide directly forward into the release latch
    print("Inserting pipette into the dock...")
    robot.move_arm_servos(shoulder=slot.release_position[0],
                          elbow=slot.release_position[1],
                          wrist=slot.release_position[2])

    # 5. Let go
    print("Opening gripper...")
    robot.open_gripper()

    # 6. Slide backwards to clear the newly placed pipette
    print("Sliding backwards...")
    robot.move_arm_servos(shoulder=slot.start_position[0], 
                          elbow=slot.start_position[1], 
                          wrist=slot.start_position[2])

    # 7. Move up to a safe height
    print("Moved to safe height. Pipette secured!")
    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
    
if __name__ == '__main__':
    main()