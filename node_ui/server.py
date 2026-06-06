import sys
import os
import time
import math
from flask import Flask, request, jsonify
from flask_cors import CORS

# Add parent directory to access andrew_robot
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
try:
    from andrew_robot import AndrewRobot
except ImportError:
    AndrewRobot = None

app = Flask(__name__)
CORS(app)

robot = None

# Kinematics constants
L1 = 152.13  
L2 = 151.67  
MAX_REACH = L1 + L2
MIN_REACH = abs(L1 - L2)
TICKS_PER_RADIAN = 4096 / (2 * math.pi)
CENTER_TICK = 2048

def inverse_kinematics(x, y, elbow_up=True):
    distance_sq = x**2 + y**2
    if distance_sq > MAX_REACH**2 or distance_sq < MIN_REACH**2:
        raise ValueError(f"Target is out of reach!")
    cos_theta2 = (distance_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.acos(cos_theta2)
    if not elbow_up: theta2 = -theta2
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    ticks_shoulder = int(theta1 * TICKS_PER_RADIAN + CENTER_TICK)
    ticks_elbow = int(theta2 * TICKS_PER_RADIAN + CENTER_TICK)
    
    if ticks_shoulder > 3100 or ticks_shoulder < 1000:
        raise ValueError("Outside safe zone!")
    return ticks_shoulder, ticks_elbow

@app.route('/init', methods=['POST'])
def init():
    global robot
    try:
        if not AndrewRobot:
            return jsonify({"status": "error", "message": "dynamixel_sdk not found (mock mode only)"}), 500
        robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
        robot.max_speed = 40
        robot.enable_torque()
        return jsonify({"status": "success", "message": "Robot Initialized"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/go', methods=['POST'])
def go():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        data = request.json
        s_ticks, e_ticks = inverse_kinematics(data['x'], data['y'])
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        robot.move_arm_servos(shoulder=s_ticks, elbow=e_ticks, wrist=1000)
        return jsonify({"status": "success", "message": "Moved to target"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/home', methods=['POST'])
def home():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        return jsonify({"status": "success", "message": "Homed"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/park', methods=['POST'])
def park():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        time.sleep(1.0)
        robot.move_arm_servos(linear=3125)
        time.sleep(2.0)
        robot.disable_torque()
        return jsonify({"status": "success", "message": "Parked"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(port=5000)