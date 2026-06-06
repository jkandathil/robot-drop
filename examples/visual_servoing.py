#!/usr/bin/env python
from context import andrew_robot
import time
import math
import cv2
import numpy as np
from andrew_robot import AndrewRobot

# --- KINEMATICS CONSTANTS ---
L1 = 152.13
L2 = 151.67
MAX_REACH = L1 + L2
TICKS_PER_RADIAN = 4096 / (2 * math.pi)
CENTER_TICK = 2048

def inverse_kinematics(x, y, elbow_up=True):
    distance_sq = x**2 + y**2
    if distance_sq > (MAX_REACH - 1)**2 or distance_sq < (L1 - L2)**2:
        raise ValueError("Target out of reach")
        
    cos_theta2 = (distance_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.acos(cos_theta2)
    
    if not elbow_up: theta2 = -theta2
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    return int(theta1 * TICKS_PER_RADIAN + CENTER_TICK), int(theta2 * TICKS_PER_RADIAN + CENTER_TICK)

def get_brightest_dot(frame):
    """ Finds the center of the brightest white dot in the image """
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply a slight blur to remove tiny noisy specks
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Lower the threshold to 160 to catch off-white objects under normal lighting
    _, thresh = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)
    
    # Find shapes/contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, thresh
        
    # Find the largest white shape
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Still ignore tiny flecks of dust
    if cv2.contourArea(largest_contour) < 50:
        return None, thresh
        
    # Calculate geometric center of the shape
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY), thresh
        
    return None, thresh

def main():
    print("Initializing robot for Visual Tracking...")
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 25
    robot.linear.moving_speed = 50
    time.sleep(0.1)

    print("Homing robot...")
    # Step 1: Proper safe homing sequence first
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)
    robot.led_arm(255) 
    time.sleep(1)

    # Start the arm roaming in the middle of our workspace rectangle
    current_x = 185.0
    current_y = 0.0
    
    s_ticks, e_ticks = inverse_kinematics(current_x, current_y)
    
    print("Moving above tracking position...")
    robot.move_arm_servos(shoulder=s_ticks, elbow=e_ticks, linear=robot.SAFE_HEIGHT)
    time.sleep(1)
    
    print("Lowering arm to desk focal height...")
    # FIXED: Lifted the tracking height slightly to 1800 so it hovers ABOVE the desk
    # instead of overloading the motor crashing into it.
    TRACKING_HEIGHT = 1800 
    robot.move_arm_servos(linear=TRACKING_HEIGHT)
    time.sleep(1)

    print("Starting Camera...")
    print("-> Place a white speck/dot under the camera.")
    print("-> Press 'q' to quit.")
    
    # Intelligently find the right camera index, since index 2 seems to be failing/disconnecting
    cap = None
    for cam_idx in [2, 1, 0, 3]:
        cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            print(f"Connected to camera index {cam_idx}!")
            break
            
    # Proportional Gain (How fast the robot chases the pixel error)
    # Higher = faster but jittery. Lower = smooth but slow.
    Kp_x = 0.03
    Kp_y = 0.03

    while True:
        if not cap or not cap.isOpened():
            if cap: cap.release()
            for cam_idx in [2, 1, 0, 3]:
                cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
                if cap.isOpened():
                    break
            time.sleep(0.5)
            continue
            
        ret, frame = cap.read()
        if not ret: continue

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        
        # Draw red crosshair target at center of camera
        cv2.drawMarker(frame, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        # Process OpenCV
        dot_pos, thresh_img = get_brightest_dot(frame)

        if dot_pos:
            cX, cY = dot_pos
            cv2.circle(frame, (cX, cY), 8, (0, 255, 0), -1) # Draw green dot
            cv2.line(frame, (center_x, center_y), (cX, cY), (255, 0, 0), 2) # Draw tracking line

            # Calculate error from crosshair
            err_x = cX - center_x
            err_y = cY - center_y
            
            # Deadband radius (if it's within 15 pixels of center, consider it parked)
            distance = math.hypot(err_x, err_y)
            if distance < 15:
                cv2.putText(frame, "PARKED ON TARGET!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                # Convert the image pixel error to Physical robot movements
                # FIXED: It turns out the camera is rotated 90 degrees right in the housing!
                # X pixel error maps to physical Y, and Y pixel error maps to physical X.
                dx = -err_y * Kp_x  
                dy = err_x * Kp_y  

                # Clip the maximum movement per frame to prevent violent jumps
                dx = max(min(dx, 3.0), -3.0)
                dy = max(min(dy, 3.0), -3.0)

                new_x = current_x + dx
                new_y = current_y + dy
                
                try:
                    s_ticks, e_ticks = inverse_kinematics(new_x, new_y)
                    # Tell servos to move continuously without blocking
                    robot.shoulder.set_goal_position(s_ticks)
                    robot.elbow.set_goal_position(e_ticks)
                    
                    # Store current successful location
                    current_x = new_x
                    current_y = new_y
                except ValueError:
                    cv2.putText(frame, "LIMIT REACHED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    print("Hit edge of workspace! The routing axis is likely mismatched.")

        else:
             cv2.putText(frame, "No spot detected...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

        # Show windows
        cv2.imshow("Robot Arm View", frame)
        cv2.imshow("What the threshold sees", thresh_img)

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    robot.led_arm(0)
    robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000, linear=robot.SAFE_HEIGHT)

if __name__ == '__main__':
    main()