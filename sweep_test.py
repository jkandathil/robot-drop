import sys
import time
import math
sys.path.append('c:\\Users\\jayan.kandathil\\Documents\\andrew-robot_test')
from andrew_robot.robot import AndrewRobot

# Copied from server.py to run standalone
CENTER_TICK = 2048
TICKS_PER_RADIAN = 4096 / (2 * math.pi)
L1 = 152.13  
L2 = 151.67  
MAX_REACH = L1 + L2

def inverse_kinematics(x, y, elbow_up=True):
    distance_sq = x**2 + y**2
    if distance_sq > MAX_REACH**2:
        raise ValueError("Target out of absolute reach")
    cos_theta2 = (distance_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    ticks_shoulder = int(theta1 * TICKS_PER_RADIAN + CENTER_TICK)
    ticks_elbow = int(theta2 * TICKS_PER_RADIAN + CENTER_TICK)
    return ticks_shoulder, ticks_elbow

def test_sweep():
    try:
        print("Connecting to robot...")
        robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
        robot.max_speed = 30
        robot.enable_torque()
        
        print("Homing...")
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(1)
        robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        time.sleep(2)
        
        print("Moving to Y=250, X=0 (Center)")
        s, e = inverse_kinematics(0, 250)
        robot.move_arm_servos(shoulder=s, elbow=e)
        time.sleep(2)
        
        print("\n--- SWEEPING X OUTWARDS (Y=250) ---")
        for x in range(0, 171, 10):
            try:
                s, e = inverse_kinematics(x, 250)
                # Ensure it doesn't violate hard limits
                if not (1000 <= s <= 3100 and 1000 <= e <= 3100):
                    print(f"X={x} -> hits hard limit! s={s}, e={e}")
                    break
                robot.move_arm_servos(shoulder=s, elbow=e)
                print(f"X={x} -> s={s}, e={e} (Success)")
                time.sleep(0.5)
            except Exception as ex:
                print(f"Failed at X={x}: {ex}")
                break
                
        time.sleep(1)
        print("\nReturning to center...")
        s, e = inverse_kinematics(0, 250)
        robot.move_arm_servos(shoulder=s, elbow=e)
        time.sleep(2)
        
        print("\n--- SWEEPING Y OUTWARDS (X=0) ---")
        for y in range(220, 305, 10):
            try:
                s, e = inverse_kinematics(0, y)
                if not (1000 <= s <= 3100 and 1000 <= e <= 3100):
                    print(f"Y={y} -> hits hard limit! s={s}, e={e}")
                    break
                robot.move_arm_servos(shoulder=s, elbow=e)
                print(f"Y={y} -> s={s}, e={e} (Success)")
                time.sleep(0.5)
            except Exception as ex:
                print(f"Failed at Y={y}: {ex}")
                break
                
        time.sleep(1)
        print("\n--- SWEEPING Y INWARDS (X=0) ---")
        for y in range(250, 200, -10):
            try:
                s, e = inverse_kinematics(0, y)
                if not (1000 <= s <= 3100 and 1000 <= e <= 3100):
                    print(f"Y={y} -> hits inner limit! s={s}, e={e} (Elbow won't fold more)")
                    break
                robot.move_arm_servos(shoulder=s, elbow=e)
                print(f"Y={y} -> s={s}, e={e} (Success)")
                time.sleep(0.5)
            except Exception as ex:
                print(f"Failed at Y={y}: {ex}")
                break
                
        print("\nTest complete! Disabling torque.")
        robot.disable_torque()
        
    except Exception as e:
        print("Fatal Error:", e)

if __name__ == "__main__":
    test_sweep()
