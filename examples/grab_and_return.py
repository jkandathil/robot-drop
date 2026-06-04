#!/usr/bin/env python
from context import andrew_robot
import time
import threading
import cv2
from andrew_robot import AndrewRobot

# Global flag to control the camera thread
camera_running = True

def robot_movement():
    global camera_running
    try:
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
        robot.move_arm_servos(shoulder=1871, elbow=1905, wrist=1977, linear=robot.SAFE_HEIGHT)
        time.sleep(2)
        
        # --- RETURN PIPETTE LOGIC ---
        print(f"Returning pipette to slot {target_slot}...")
        slot = robot.config.pipette_slots[f'slot{target_slot}']
        
        robot.move_servos_proportional(*slot.start_position, linear=robot.SAFE_HEIGHT)
        robot.move_servos_proportional(*slot.start_position, linear=robot.GRAB_HEIGHT)
        robot.move_servos_proportional(*slot.release_position)
        robot.open_gripper()
        robot.move_servos_proportional(*slot.start_position)
        robot.move_servos_proportional(linear=robot.SAFE_HEIGHT)
        
        print("Pipette successfully returned!")
    except Exception as e:
        print(f"Robot error: {e}")
    finally:
        # Tell the main thread (camera) that movement is finished
        camera_running = False

def main():
    global camera_running

    # Start the ROBOT movement in a background thread 
    # (OpenCV windows freeze if they are not in the main thread!)
    arm_thread = threading.Thread(target=robot_movement)
    arm_thread.start()

    # Run the CAMERA UI in the main thread
    print("Starting arm camera feed (with auto-reconnect)...")
    cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
    
    while camera_running:
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)
            time.sleep(0.5)
            continue
            
        ret, frame = cap.read()
        if ret:
            cv2.imshow('Arm Tip Camera - Live Feed', frame)
        else:
            # If camera drops frame, try to reconnect
            cap.release()
            
        # OpenCV GUI event processing must happen in the main thread
        if cv2.waitKey(1) == ord('q'):
            camera_running = False
            break
            
    if cap:
        cap.release()
    cv2.destroyAllWindows()
    
    # Wait for the arm thread to wrap up gracefully if we stopped early
    arm_thread.join(timeout=2.0)

if __name__ == '__main__':
    main()
