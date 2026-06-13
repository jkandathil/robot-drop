import sys
import os
import time
import math
import threading
import json
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

from contextlib import contextmanager

robot = None
robot_lock = threading.Lock()
sequence_running = False
sequence_progress = {"current": 0, "total": 0, "status": "Idle"}

# Always anchored next to this script - a relative path would make the settings
# silently load/save in whatever directory the app happened to be launched from,
# losing the saved aspirate/dispense/grab positions between runs.
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'settings.json')
USER_Z_LIMIT = 3125 # Default absolute safe limit
THUMB_LIMITS = {} # slot -> aspirate thumb position limit
THUMB_DISPENSE_LIMITS = {} # slot -> dispense thumb position limit
# The system has two cameras: the arm-mounted one (used for calibration) and the
# aspiration-volume one. Default to index 2 (the arm camera in the prototype);
# override with "arm_camera_index" in settings.json so we never grab the other.
ARM_CAMERA_INDEX = 2
# Dynamixel moving_speed applied to the arm joints (shoulder/elbow/wrist/linear).
# The old code hard-capped this at 40, which made all mapping moves and Z-jogging
# crawl. 100 is roughly 2.5x faster while still settling cleanly; tune live via the
# Calibrate-page speed slider. 0 would mean "unlimited" on a Dynamixel, so we never
# allow that.
ARM_SPEED = 100
# Per-area SAFE APPROACH Z (ticks; higher tick = physically lower). A map click /
# "Go" only ever descends to this height - NEVER to the taught surface plane - so
# a click can't crash the tip into the substrate or an ink vial. From there the
# user jogs Z down manually and captures the real working height.
# Substrate default raised 3000 -> 2700 (~25 mm higher): 3000 sat below the well
# plate top and fast descent crashed the tip. Adjustable per zone via
# /area_travel_z/set (the "Set to current Z" button on the Calibrate page).
AREA_TRAVEL_Z = {'substrate': 2700, 'reagent': 2500}
# Per-slot taught grab pose overriding the factory andrew.xml grabPosition. The
# factory positions were taught for Gilson Pipetman bodies; other pipettes (e.g.
# Rainin LTS) need the gripper deeper/at a different height, so the user can
# hand-teach a pose per slot. slot(str) -> {"grab": [shoulder, elbow, wrist], "z": linear}
GRAB_OVERRIDES = {}

def load_settings():
    global USER_Z_LIMIT, THUMB_LIMITS, THUMB_DISPENSE_LIMITS, ARM_CAMERA_INDEX, ARM_SPEED, AREA_TRAVEL_Z, GRAB_OVERRIDES
    if os.path.exists(SETTINGS_FILE):
        try:
            with open(SETTINGS_FILE, 'r') as f:
                data = json.load(f)
                USER_Z_LIMIT = data.get('safe_z_limit', 3125)
                THUMB_LIMITS = data.get('thumb_limits', {})
                THUMB_DISPENSE_LIMITS = data.get('thumb_dispense_limits', {})
                ARM_CAMERA_INDEX = data.get('arm_camera_index', 2)
                ARM_SPEED = data.get('arm_speed', 100)
                saved = data.get('area_travel_z', {})
                AREA_TRAVEL_Z = {'substrate': int(saved.get('substrate', 2700)),
                                 'reagent': int(saved.get('reagent', 2500))}
                # Migration: 3000 was the old hardcoded substrate default, which sat
                # below the well plate and crashed the tip on fast descent. No UI
                # could set this value before, so a stored 3000 can only BE that old
                # default - lift it to the safer height.
                if AREA_TRAVEL_Z['substrate'] >= 3000:
                    AREA_TRAVEL_Z['substrate'] = 2700
                GRAB_OVERRIDES = data.get('grab_overrides', {})
        except Exception:
            pass

def save_settings():
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump({'safe_z_limit': USER_Z_LIMIT, 'thumb_limits': THUMB_LIMITS,
                       'thumb_dispense_limits': THUMB_DISPENSE_LIMITS,
                       'arm_camera_index': ARM_CAMERA_INDEX,
                       'arm_speed': ARM_SPEED,
                       'area_travel_z': AREA_TRAVEL_Z,
                       'grab_overrides': GRAB_OVERRIDES}, f)
    except Exception:
        pass

load_settings()

current_pipette_slot = 1

@contextmanager
def use_robot():
    with robot_lock:
        if robot and hasattr(robot, 'port_handler') and hasattr(robot.port_handler, 'is_using'):
            robot.port_handler.is_using = False
        yield

# Kinematics constants
L1 = 152.13  
L2 = 151.67  
MAX_REACH = L1 + L2
MIN_REACH = abs(L1 - L2)
TICKS_PER_RADIAN = 4096 / (2 * math.pi)
CENTER_TICK = 2048

def inverse_kinematics(x, y, elbow_up=True, block_id=None):
    # Apply mapped correction offsets
    cx, cy = get_correction_offset(x, y, block_id)
    x += cx
    y += cy
    
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
    
    if ticks_shoulder > 4095 or ticks_shoulder < 0:
        raise ValueError("Outside safe zone!")
    return ticks_shoulder, ticks_elbow

def compute_kinematics(x, y, elbow_up=True):
    # Raw variant without correction offset if needed
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
    return ticks_shoulder, ticks_elbow

CORRECTION_MAP = {}
def load_correction_map():
    global CORRECTION_MAP
    import os, json
    if os.path.exists("correction_map.json"):
        try:
            with open("correction_map.json", "r") as f:
                CORRECTION_MAP = json.load(f)
        except:
            CORRECTION_MAP = {}

def get_correction_offset(x, y, block_id=None):
    if block_id and robot and hasattr(robot, 'config') and hasattr(robot.config, 'tip_geometry_offset'):
        offset = robot.config.tip_geometry_offset.get(block_id)
        if offset:
            return (offset['x'], offset['y'])
            
    # Bilinear Interpolation from 4 corners
    if 'math' in CORRECTION_MAP and 'phys' in CORRECTION_MAP:
        try:
            math_pts = CORRECTION_MAP['math']
            phys_pts = CORRECTION_MAP['phys']
            
            bl_m = math_pts['bl']; br_m = math_pts['br']; tr_m = math_pts['tr']; tl_m = math_pts['tl']
            bl_p = phys_pts['bl']; br_p = phys_pts['br']; tr_p = phys_pts['tr']; tl_p = phys_pts['tl']
            
            w = br_m['x'] - bl_m['x']
            h = tl_m['y'] - bl_m['y']
            if w == 0 or h == 0: return (0.0, 0.0)
            
            u = (x - bl_m['x']) / w
            v = (y - bl_m['y']) / h
            
            # Allow slight extrapolation if outside bounds
            phys_x = (1-u)*(1-v)*bl_p['x'] + u*(1-v)*br_p['x'] + u*v*tr_p['x'] + (1-u)*v*tl_p['x']
            phys_y = (1-u)*(1-v)*bl_p['y'] + u*(1-v)*br_p['y'] + u*v*tr_p['y'] + (1-u)*v*tl_p['y']
            
            return (phys_x - x, phys_y - y)
        except Exception as e:
            print("Interpolation error:", e)
            
    return (0.0, 0.0)

load_correction_map()

def move_with_backlash_comp(robot, x, y, final_z=None, wrist_tick=1000, block_id=None):
    # Direct XY approach (removed broken backlash offset that caused overshooting)
    fsx, fex = inverse_kinematics(x, y, block_id=block_id)
    robot.move_arm_servos(shoulder=fsx, elbow=fex, wrist=wrist_tick)
    time.sleep(0.5)

    if final_z is not None:
        # Apply factory Z arm deflection if available
        z_comp_ticks = 0
        if block_id and hasattr(robot, 'config') and hasattr(robot.config, 'arm_deflection'):
            block_def = robot.config.arm_deflection.get(block_id)
            if block_def:
                # Average mm deflection for the block
                # Deflection is how much the arm SAGS (drops). 
                # To compensate, we must pull the arm UP.
                # Since higher Z ticks = lower physical height, we SUBTRACT ticks to go up!
                z_comp_mm = (block_def['LT'] + block_def['RB'] + block_def['LB']) / 3.0
                z_comp_ticks = round(z_comp_mm / 0.08174)

        adjusted_final_z = final_z - z_comp_ticks
        adjusted_final_z = min(adjusted_final_z, USER_Z_LIMIT)
        
        # Move to 20 ticks above first to eliminate momentum overshoot
        robot.move_arm_servos(linear=adjusted_final_z - 20)
        time.sleep(0.3)
        # Drop gently into final position
        robot.move_arm_servos(linear=adjusted_final_z)
        time.sleep(0.5)
        
    return fsx, fex

@app.route('/init', methods=['POST'])
def init():
    global robot
    try:
        if not AndrewRobot:
            return jsonify({"status": "error", "message": "dynamixel_sdk not found (mock mode only)"}), 500
        robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
        # The max_speed setter only CAPS speeds (never raises them), so also set the
        # arm joints' moving_speed directly to actually apply the configured speed.
        robot.max_speed = ARM_SPEED
        for s in (robot.shoulder, robot.elbow, robot.wrist, robot.linear):
            try:
                s.moving_speed = ARM_SPEED
            except Exception:
                pass
        robot.enable_torque()
        # Mandatory homing
        with use_robot():
            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
            time.sleep(0.5)
            robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
            time.sleep(1.5)
            
        return jsonify({"status": "success", "message": "Robot Initialized and Homed"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/go', methods=['POST'])
def go():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        data = request.json
        block_id = data.get('block_id')
        with use_robot():
            robot.enable_torque()
            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
            time.sleep(0.5)
            # Use approach compensation with 1-2 counts precision
            fsx, fex = move_with_backlash_comp(robot, data['x'], data['y'], block_id=block_id)
            # Verify position convergence
            time.sleep(0.5)
            actual_pos = robot.get_servo_positions()
        return jsonify({"status": "success", "message": "Moved to target (with backlash comp)", "final_ticks": [fsx, fex], "actual": actual_pos})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/home', methods=['POST'])
def home():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
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
        with use_robot():
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

@app.route('/gripper/open', methods=['POST'])
def gripper_open():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
            robot.open_gripper()
        return jsonify({"status": "success", "message": "Gripper opened"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/gripper/close', methods=['POST'])
def gripper_close():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
            robot.close_gripper()
        return jsonify({"status": "success", "message": "Gripper closed"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/grab_pipette', methods=['POST'])
def grab_pipette_endpoint():
    global current_pipette_slot
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        data = request.json
        slot = int(data.get('slot', 1))
        current_pipette_slot = slot
        ov = GRAB_OVERRIDES.get(str(slot))
        with use_robot():
            robot.enable_torque()
            if ov:
                robot.grab_pipette(slot, grab_position=tuple(ov['grab']), grab_height=int(ov['z']))
            else:
                robot.grab_pipette(slot)
        taught = " (taught pose)" if ov else " (factory pose)"
        return jsonify({"status": "success", "message": f"Grabbed pipette from slot {slot}{taught}"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

def _hold_arm_at_present():
    """
    Re-engage the XY joints WITHOUT any snap-back: write goal=present position
    while torque is still off, THEN enable torque. Enabling first would make the
    servo drive to its stale pre-release goal and yank the arm out of the
    hand-taught pose.
    """
    for s in (robot.shoulder, robot.elbow, robot.wrist):
        s.set_goal_position(int(s.position))
        s.enable_torque()

@app.route('/grab_cal/teach', methods=['POST'])
def grab_cal_teach():
    """
    Start hand-teaching a grab pose for a slot: drive the open gripper to the
    slot's factory approach point at grab height, then release shoulder/elbow/
    wrist (Z, thumb, gripper and twister stay held) so the user can push the
    gripper onto the pipette body by hand. Same release-and-verify dance as
    /torque/free_xy - a single torque-off write sometimes doesn't take.
    """
    global current_pipette_slot
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    try:
        slot = int(data.get('slot', current_pipette_slot))
        current_pipette_slot = slot
        cfg_slot = robot.config.pipette_slots[f'slot{slot}']
        ov = GRAB_OVERRIDES.get(str(slot))
        teach_z = int(ov['z']) if ov else robot.GRAB_HEIGHT
        free = (robot.shoulder, robot.elbow, robot.wrist)
        stuck = []
        with use_robot():
            robot.enable_torque()
            robot.open_gripper()
            robot.move_servos_proportional(*cfg_slot.start_position, linear=teach_z)
            for s in robot.servos:
                if s not in free:
                    s.enable_torque()           # Z / thumb / gripper / twister hold
            for s in free:
                released = False
                for _ in range(4):
                    s.disable_torque()
                    try:
                        if s.read_bytes(s.control_table.addr_torque_enable, 1) == 0:
                            released = True
                            break
                    except Exception:
                        released = True
                        break
                    time.sleep(0.03)
                if not released:
                    stuck.append(s.id)
        if stuck:
            return jsonify({"status": "error",
                            "message": f"Could not release joint(s) {stuck}. Re-issue Teach Grab."}), 500
        return jsonify({"status": "success",
                        "message": f"Slot {slot}: gripper open at approach pose, arm released. "
                                   f"Hand-guide the open gripper onto the pipette (use Z nudge for height), then Save Grab Pose."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/grab_cal/goto', methods=['POST'])
def grab_cal_goto():
    """
    Jog-based teaching, step 1: drive the OPEN gripper to the slot's factory
    approach point at grab height with all joints held (torque ON). From here the
    user jogs In/Out along the approach line and nudges Z, then saves - no
    hand-guiding needed.
    """
    global current_pipette_slot
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    try:
        slot = int(data.get('slot', current_pipette_slot))
        current_pipette_slot = slot
        cfg_slot = robot.config.pipette_slots[f'slot{slot}']
        ov = GRAB_OVERRIDES.get(str(slot))
        teach_z = int(ov['z']) if ov else robot.GRAB_HEIGHT
        with use_robot():
            robot.enable_torque()
            robot.open_gripper()
            robot.move_servos_proportional(*cfg_slot.start_position, linear=teach_z)
        return jsonify({"status": "success",
                        "message": f"Slot {slot}: gripper open at approach pose. "
                                   f"Jog In toward the pipette and nudge Z, then Save Grab Pose."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

approach_jogging = False

@app.route('/grab_cal/approach_jog/start', methods=['POST'])
def approach_jog_start():
    """
    Hold-to-jog the gripper along the slot's approach line: the joint-space
    segment from the factory startPosition (t=0) to grabPosition (t=1),
    extrapolated past t=1 so the gripper can go DEEPER than the factory pose
    (what a Rainin LTS body needs). 'in' increases t, 'out' decreases it.
    Joints are re-held in place first, so this also works right after a
    free-arm hand move (the goal snaps to the nearest point ON the line).
    """
    global approach_jogging
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    # Guard against double-start: a second jog thread would fight the first over the same servos
    if approach_jogging: return jsonify({"status": "success", "message": "Already jogging"})

    data = request.json or {}
    direction = data.get('direction', 'in')
    try:
        slot = int(data.get('slot', current_pipette_slot))
        cfg_slot = robot.config.pipette_slots[f'slot{slot}']
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    start = cfg_slot.start_position
    grab = cfg_slot.grab_position
    vec = [g - s for s, g in zip(start, grab)]
    vec_len_sq = sum(v * v for v in vec)
    biggest = max(abs(v) for v in vec)
    if biggest == 0:
        return jsonify({"status": "error",
                        "message": f"Slot {slot}: factory startPosition and grabPosition are identical in "
                                   f"andrew.xml - no approach direction to jog along."}), 400

    approach_jogging = True
    def jog_loop():
        global approach_jogging
        try:
            with use_robot():
                _hold_arm_at_present()
                pos = robot.get_servo_positions()
            # Where along the approach line are we now? (projection of the current
            # pose onto the start->grab segment, in joint space)
            t = sum((p - s) * v for p, s, v in zip(pos[:3], start, vec)) / vec_len_sq
        except Exception:
            approach_jogging = False
            return

        # Per 50ms iteration the fastest-moving joint advances ~4 ticks (~80 ticks/s,
        # same feel as the medium Z nudge). t is clamped so Out can't go far behind
        # the approach point and In can't run more than 2.5x the factory depth.
        dt = 4.0 / biggest
        step = dt if direction == 'in' else -dt
        while approach_jogging:
            t = max(-0.25, min(2.5, t + step))
            target = [int(round(s + t * v)) for s, v in zip(start, vec)]
            try:
                with use_robot():
                    # Write goal positions directly to avoid blocking and stuttering
                    robot.shoulder.set_goal_position(target[0])
                    robot.elbow.set_goal_position(target[1])
                    robot.wrist.set_goal_position(target[2])
            except Exception:
                pass
            time.sleep(0.05)

    threading.Thread(target=jog_loop, daemon=True).start()
    return jsonify({"status": "success", "message": f"Approach jogging {direction}"})

@app.route('/grab_cal/approach_jog/stop', methods=['POST'])
def approach_jog_stop():
    global approach_jogging
    approach_jogging = False
    return jsonify({"status": "success", "message": "Approach jogging stopped"})

@app.route('/grab_cal/save', methods=['POST'])
def grab_cal_save():
    """
    Capture the current hand-placed pose as the slot's grab override and re-hold
    the arm there. Reads PRESENT positions (not goals - goals are stale after a
    torque-off hand move, see /save_target note).
    """
    global GRAB_OVERRIDES
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    try:
        slot = int(data.get('slot', current_pipette_slot))
        with use_robot():
            pos = [s.position for s in robot.servos]
            _hold_arm_at_present()
        GRAB_OVERRIDES[str(slot)] = {"grab": [int(pos[0]), int(pos[1]), int(pos[2])],
                                     "z": int(pos[3])}
        save_settings()
        return jsonify({"status": "success",
                        "message": f"Saved grab pose for slot {slot}: shoulder/elbow/wrist "
                                   f"{pos[0]}/{pos[1]}/{pos[2]} at Z {pos[3]}. Use Test Grab to verify.",
                        "override": GRAB_OVERRIDES[str(slot)]})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/grab_cal/retract', methods=['POST'])
def grab_cal_retract():
    """
    Safely leave the teaching pose WITHOUT grabbing: re-hold the arm, back the
    open gripper straight out of the holder at the CURRENT height first, and only
    then lift to safe height. Lifting first would catch the open gripper on the
    pipette's hanging collar / holder hook.
    """
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    try:
        slot = int(data.get('slot', current_pipette_slot))
        cfg_slot = robot.config.pipette_slots[f'slot{slot}']
        with use_robot():
            _hold_arm_at_present()
            robot.move_servos_proportional(*cfg_slot.start_position)
            robot.move_servos_proportional(*cfg_slot.start_position, linear=robot.SAFE_HEIGHT)
        return jsonify({"status": "success", "message": f"Backed out of slot {slot} and lifted to safe height."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/grab_cal/clear', methods=['POST'])
def grab_cal_clear():
    global GRAB_OVERRIDES
    data = request.json or {}
    try:
        slot = str(int(data.get('slot', current_pipette_slot)))
        if slot in GRAB_OVERRIDES:
            del GRAB_OVERRIDES[slot]
            save_settings()
            return jsonify({"status": "success", "message": f"Cleared taught grab pose for slot {slot} - factory pose active."})
        return jsonify({"status": "success", "message": f"Slot {slot} has no taught grab pose (factory pose already active)."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/grab_cal/get', methods=['GET'])
def grab_cal_get():
    return jsonify({"status": "success", "overrides": GRAB_OVERRIDES})

z_jogging = False

@app.route('/z_jog/start', methods=['POST'])
def z_jog_start():
    global z_jogging
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    # Guard against double-start: a second jog thread would fight the first over the same servo
    if z_jogging: return jsonify({"status": "success", "message": "Already jogging"})

    data = request.json
    direction = data.get('direction', 'down')
    speed = data.get('speed', 'fast')
    # Per-area FAST-descent floor: fast jog stops at the zone's safe approach
    # height (substrate 3000 / reagent 2500 by default) so holding the button
    # can't crash the tip. SLOW (fine) descent is intentionally unrestricted -
    # apart from the global Safe-Z limit - for the final manual approach when
    # setting ink / dispense heights.
    area = data.get('area')
    fast_floor = AREA_TRAVEL_Z.get(area, 3000)

    z_jogging = True
    def jog_loop():
        try:
            with use_robot():
                pos = robot.get_servo_positions()
            current_z = pos[3]
        except:
            return
            
        # Increased step sizes for faster jogging. Fine is max physical resolution (1 tick);
        # 'medium' (~8 mm/s) is for visible-but-controlled moves like grab teaching.
        step_val = 50 if speed == 'fast' else (5 if speed == 'medium' else 1)
        step = step_val if direction == "down" else -step_val
        while z_jogging:
            next_z = current_z + step
            
            # FAST descent stops at the zone's safe approach height (or, when no
            # area was given, slightly above the global Safe-Z to kill momentum).
            # If the tip is ALREADY below that floor (after a slow approach), fast
            # down simply holds position - it must never jump back up.
            fast_jog_limit = min(fast_floor, USER_Z_LIMIT - 50)
            if speed == 'fast' and direction == 'down' and next_z > fast_jog_limit:
                next_z = max(fast_jog_limit, current_z)
                
            if next_z < 200: next_z = 200
            if next_z > USER_Z_LIMIT: next_z = USER_Z_LIMIT
            
            if next_z != current_z:
                current_z = next_z
                try:
                    with use_robot():
                        # Write goal position directly to avoid blocking and stuttering!
                        robot.linear.set_goal_position(int(current_z))
                except Exception:
                    pass
            time.sleep(0.05) # Increased frequency for smoother movement
            
    threading.Thread(target=jog_loop, daemon=True).start()
    return jsonify({"status": "success", "message": f"Jogging {direction}"})

@app.route('/set_safe_z', methods=['POST'])
def set_safe_z():
    global USER_Z_LIMIT
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
            # Get actual current position of linear axis
            pos = robot.get_servo_positions()
            current_z = pos[3]
        USER_Z_LIMIT = current_z
        save_settings()
        return jsonify({"status": "success", "message": f"Safe Z limit successfully set to {USER_Z_LIMIT} ticks."})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/get_safe_z', methods=['GET'])
def get_safe_z():
    return jsonify({"safe_z_limit": USER_Z_LIMIT})

@app.route('/area_travel_z/get', methods=['GET'])
def area_travel_z_get():
    return jsonify({"status": "success", "substrate": AREA_TRAVEL_Z['substrate'],
                    "reagent": AREA_TRAVEL_Z['reagent']})

@app.route('/area_travel_z/set', methods=['POST'])
def area_travel_z_set():
    """
    Set a zone's fast-descent stop height ("travel Z"): jog the tip to just above
    the tallest labware in the zone and capture the current Z, or pass an explicit
    tick value. Clamped to stay above the global Safe-Z (with the same 50-tick
    momentum margin the fast jog uses), so this can only ever make fast descent
    stop HIGHER, never let it pass the Safe-Z limit.
    """
    data = request.get_json(silent=True) or {}
    area = data.get('area')
    if area not in AREA_TRAVEL_Z:
        return jsonify({"status": "error", "message": f"Unknown area '{area}' - use 'substrate' or 'reagent'"}), 400
    try:
        z = data.get('z')
        if z is None:
            if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
            with use_robot():
                z = robot.get_servo_positions()[3]
        z = int(z)
        z = max(200, min(z, USER_Z_LIMIT - 50))
        AREA_TRAVEL_Z[area] = z
        save_settings()
        zone = 'wells' if area == 'substrate' else 'inks'
        return jsonify({"status": "success", "z": z,
                        "message": f"Fast descent in the {zone} zone now stops at {z} ticks "
                                   f"({ticks_to_depth_mm(z)} mm)"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/set_speed', methods=['POST'])
def set_speed():
    """Set the arm joints' moving speed live (used to speed up zone mapping)."""
    global ARM_SPEED
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    try:
        speed = int(data.get('speed', ARM_SPEED))
        # Clamp: never 0 (=unlimited on Dynamixel) and never a runaway value.
        speed = max(20, min(speed, 250))
        with use_robot():
            for s in (robot.shoulder, robot.elbow, robot.wrist, robot.linear):
                s.moving_speed = speed
        robot._max_speed = speed
        ARM_SPEED = speed
        save_settings()
        return jsonify({"status": "success", "message": f"Arm speed set to {speed}"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/get_speed', methods=['GET'])
def get_speed():
    return jsonify({"status": "success", "speed": ARM_SPEED})

@app.route('/z_jog/stop', methods=['POST'])
def z_jog_stop():
    global z_jogging
    z_jogging = False
    return jsonify({"status": "success", "message": "Jogging stopped"})

thumb_jogging = False

@app.route('/thumb_jog/start', methods=['POST'])
def thumb_jog_start():
    global thumb_jogging
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    # Guard against double-start: a second jog thread would fight the first over the same servo
    if thumb_jogging: return jsonify({"status": "success", "message": "Already jogging"})

    data = request.json
    direction = data.get('direction', 'down')

    thumb_jogging = True
    def jog_loop():
        try:
            with use_robot():
                pos = robot.get_servo_positions()
                current_thumb = pos[4]
                # Hold at the present position BEFORE torque-on (no snap-back),
                # then enable: after an e-stop the thumb is limp and goal writes
                # alone would move nothing.
                robot.thumb.set_goal_position(int(current_thumb))
                robot.thumb.enable_torque()
        except:
            return

        THUMB_JOG_MIN = 1400
        THUMB_JOG_MAX = 3200
        # Two-speed jog: free travel is coarse, but past 60% of the range the
        # thumb is on the plunger, where the aspirate/dispense stops live - there
        # it drops to 1/5 speed (4 ticks/50ms) for granular positioning.
        fine_zone_start = THUMB_JOG_MIN + int(0.6 * (THUMB_JOG_MAX - THUMB_JOG_MIN))  # 2480
        while thumb_jogging:
            step_val = 20 if current_thumb < fine_zone_start else 4
            step = step_val if direction == "down" else -step_val
            next_thumb = current_thumb + step

            if next_thumb < THUMB_JOG_MIN: next_thumb = THUMB_JOG_MIN
            if next_thumb > THUMB_JOG_MAX: next_thumb = THUMB_JOG_MAX
            # Never coarse-step PAST the zone boundary going down - land on it,
            # then continue at fine speed
            if direction == "down" and current_thumb < fine_zone_start < next_thumb:
                next_thumb = fine_zone_start

            if next_thumb != current_thumb:
                current_thumb = next_thumb
                try:
                    with use_robot():
                        robot.thumb.set_goal_position(int(current_thumb))
                except Exception:
                    pass
            time.sleep(0.05)

    threading.Thread(target=jog_loop, daemon=True).start()
    return jsonify({"status": "success", "message": f"Thumb Jogging {direction}"})

@app.route('/thumb_jog/stop', methods=['POST'])
def thumb_jog_stop():
    global thumb_jogging
    thumb_jogging = False
    return jsonify({"status": "success", "message": "Thumb Jogging stopped"})

def _requested_slot():
    """
    The pipette slot a save/test applies to. The UI sends the dropdown selection
    with each request; trusting only the server-side current_pipette_slot (set by
    the last grab) made saves land on slot 1 after every server restart - the
    'aspirate/dispense positions not saving' bug.
    """
    global current_pipette_slot
    data = request.get_json(silent=True) or {}
    slot = int(data.get('slot', current_pipette_slot))
    current_pipette_slot = slot
    return slot

@app.route('/save_thumb_pos', methods=['POST'])
def save_thumb_pos():
    global THUMB_LIMITS
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        slot = _requested_slot()
        with use_robot():
            pos = robot.get_servo_positions()
            current_thumb = int(pos[4])
        THUMB_LIMITS[str(slot)] = current_thumb
        save_settings()
        # Read back from disk so "saved" in the message is a verified fact
        saved = json.load(open(SETTINGS_FILE)).get('thumb_limits', {}).get(str(slot))
        if saved != current_thumb:
            return jsonify({"status": "error", "message": f"Write to {SETTINGS_FILE} failed - check file permissions"}), 500
        return jsonify({"status": "success", "message": f"Saved aspirate pos {current_thumb} for Pipette Slot {slot}"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/save_thumb_dispense_pos', methods=['POST'])
def save_thumb_dispense_pos():
    global THUMB_DISPENSE_LIMITS
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        slot = _requested_slot()
        with use_robot():
            pos = robot.get_servo_positions()
            current_thumb = int(pos[4])
        THUMB_DISPENSE_LIMITS[str(slot)] = current_thumb
        save_settings()
        saved = json.load(open(SETTINGS_FILE)).get('thumb_dispense_limits', {}).get(str(slot))
        if saved != current_thumb:
            return jsonify({"status": "error", "message": f"Write to {SETTINGS_FILE} failed - check file permissions"}), 500
        return jsonify({"status": "success", "message": f"Saved dispense pos {current_thumb} for Pipette Slot {slot}"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

def _wait_servo(servo, target, timeout_s, margin=10, should_continue=None):
    """
    Wait until a servo PHYSICALLY arrives at target. move_servos() caps its own
    wait at 2.5s, but e.g. the thumb at speed 60 needs ~2.7s+ for a full plunge
    (more under plunger-spring load) and the linear axis at low arm speeds can
    need 6s+ for a full lift - relying on move_servos meant later steps started
    while the joint was still travelling. Takes/releases the robot lock per read
    so the e-stop stays responsive; should_continue() aborts the wait early
    (used by the sequence runner's stop flag).
    """
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if should_continue is not None and not should_continue():
            return False
        try:
            with use_robot():
                if abs(servo.position - target) <= margin:
                    return True
        except Exception:
            pass
        time.sleep(0.05)
    return False

def _wait_thumb(target, timeout_s, margin=10):
    return _wait_servo(robot.thumb, target, timeout_s, margin)

def _thumb_set(target):
    """
    Command the thumb to `target`, tolerating a latched overload alarm. The plunger
    bottoming out (full press) can trip the servo's overload shutdown, which latches
    the error byte so the NEXT thumb write would raise "Overload error!". If the
    command fails, clear the alarm (restore torque) and retry once - so one hard
    press never crashes the run. Caller still uses _wait_servo to wait for arrival.
    """
    target = int(target)
    try:
        with use_robot():
            robot.thumb.enable_torque()
            robot.thumb.set_goal_position(target)
        return
    except Exception:
        pass
    with use_robot():
        robot.recover_overload(robot.thumb)
    try:
        with use_robot():
            robot.thumb.enable_torque()
            robot.thumb.set_goal_position(target)
    except Exception as e:
        print(f"[thumb] command failed even after overload recovery: {e}")

def _test_plunge(target_thumb, hold_s, action_name):
    """
    Shared test cycle for the aspirate/dispense buttons: plunge the thumb to the
    saved position, WAIT until it really gets there, DWELL for the aspirate/
    dispense time, then ALWAYS return to neutral (and wait for that too). Sleeps
    happen OUTSIDE the robot lock so other endpoints (e.g. e-stop) aren't blocked.
    """
    target_thumb = int(target_thumb)
    with use_robot():
        robot.enable_torque()
        robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        time.sleep(1.0)
        robot.thumb.enable_torque()
        robot.thumb.set_goal_position(target_thumb)
    reached = _wait_thumb(target_thumb, 8.0)
    with use_robot():
        actual = robot.thumb.position
    time.sleep(hold_s)
    with use_robot():
        robot.thumb.set_goal_position(robot.THUMB_NEUTRAL_POSITION)
    _wait_thumb(robot.THUMB_NEUTRAL_POSITION, 8.0)
    if not reached:
        return (f"WARNING: {action_name} test only reached thumb {actual} of target {target_thumb} "
                f"after 8s - plunger may be blocked or the thumb torque-limited. Returned to neutral.")
    return f"{action_name} test at Home: plunged to {target_thumb}, held {hold_s:g}s, returned to neutral"

@app.route('/thumb_limits/get', methods=['GET'])
def thumb_limits_get():
    """Saved aspirate/dispense thumb positions per slot, straight from disk so the
    UI shows what will actually survive a restart - not just in-memory state."""
    on_disk = {}
    try:
        with open(SETTINGS_FILE, 'r') as f:
            on_disk = json.load(f)
    except Exception:
        pass
    return jsonify({"status": "success",
                    "aspirate": on_disk.get('thumb_limits', THUMB_LIMITS),
                    "dispense": on_disk.get('thumb_dispense_limits', THUMB_DISPENSE_LIMITS)})

@app.route('/test_aspirate', methods=['POST'])
def test_aspirate():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        slot = _requested_slot()
        data = request.get_json(silent=True) or {}
        hold_s = max(0.0, min(10.0, float(data.get('hold_s', 2.0))))
        target_thumb = THUMB_LIMITS.get(str(slot), 2888)
        msg = _test_plunge(target_thumb, hold_s, f"Slot {slot} aspirate")
        return jsonify({"status": "success", "message": msg})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/test_dispense', methods=['POST'])
def test_dispense():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        slot = _requested_slot()
        data = request.get_json(silent=True) or {}
        hold_s = max(0.0, min(10.0, float(data.get('hold_s', 2.0))))
        target_thumb = THUMB_DISPENSE_LIMITS.get(str(slot), 3050)
        msg = _test_plunge(target_thumb, hold_s, f"Slot {slot} dispense")
        return jsonify({"status": "success", "message": msg})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/thumb_neutral', methods=['POST'])
def thumb_neutral():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
            robot.thumb_neutral()
        return jsonify({"status": "success", "message": "Thumb returned to neutral"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/eject_tip', methods=['POST'])
def eject_tip():
    """
    Press the pipette's tip ejector with the thumb, hold briefly so the tip
    actually drops, then return the thumb to neutral. Ejects IN PLACE - position
    the arm over a waste bin first; this deliberately does not move the arm.
    """
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
            robot.thumb.enable_torque()
            robot.thumb_eject()
            time.sleep(0.3)
            robot.thumb_neutral()
        return jsonify({"status": "success", "message": "Tip ejected - thumb returned to neutral"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

ink_pos = None
well_pos = None

@app.route('/save_target/<target_type>', methods=['POST'])
def save_target(target_type):
    global ink_pos, well_pos
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        # NOTE: Manually taught positions might have torque disabled. It is safer to only read them if Torque=ON, 
        # Read actual physical position using s.position instead of get_goal_position!
        # If torque is OFF and you move the robot, get_goal_position is wrong!
        with use_robot():
            pos = [s.position for s in robot.servos]
        if target_type == "ink":
            ink_pos = pos
        elif target_type == "well":
            well_pos = pos
        return jsonify({"status": "success", "message": f"Saved to {target_type} (ensure torque is verified)", "pos": pos})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

def forward_kinematics(ticks_shoulder, ticks_elbow, elbow_up=True):
    # Convert ticks back to radians
    theta1 = (ticks_shoulder - CENTER_TICK) / TICKS_PER_RADIAN
    theta2 = (ticks_elbow - CENTER_TICK) / TICKS_PER_RADIAN
    
    # Calculate XY from angles
    # Note: Andrew Robot's physical Y is left/right, physical X is forward.
    # We output physical coordinates, and let the frontend map them.
    phy_x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    phy_y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    
    return phy_x, phy_y

@app.route('/generate_grid', methods=['POST'])
def generate_grid():
    data = request.json
    base_pos = data.get('base_pos')
    delta_x = float(data.get('delta_x', 0))
    delta_y = float(data.get('delta_y', 0))
    rep_x = int(data.get('rep_x', 1))
    rep_y = int(data.get('rep_y', 1))

    if not base_pos or len(base_pos) < 4:
        return jsonify({"status": "error", "message": "Invalid base position"}), 400

    base_shoulder = base_pos[0]
    base_elbow = base_pos[1]
    wrist = base_pos[2]
    linear = base_pos[3]

    try:
        base_phy_x, base_phy_y = forward_kinematics(base_shoulder, base_elbow)
        # Convert physical arm pos to tip pos to avoid double-offsetting
        cx, cy = get_correction_offset(base_phy_x, base_phy_y)
        base_tip_x = base_phy_x - cx
        base_tip_y = base_phy_y - cy
        
        locations = []
        for col in range(rep_x):
            for row in range(rep_y):
                # Map UI Canvas coordinates to Python Physical Coordinates
                # UI Delta X (col) = Left/Right Canvas movement = -Python Y
                # UI Delta Y (row) = Forward/Back Canvas movement = -Python X
                target_tip_x = base_tip_x - (row * delta_y)
                target_tip_y = base_tip_y - (col * delta_x)
                
                # Inverse kinematics adds the correction offset back automatically
                fsx, fex = inverse_kinematics(target_tip_x, target_tip_y)
                
                # CRITICAL TIP-LINEARITY FIX:
                # If we keep the wrist ticks constant, the wrist is locked to the elbow.
                # When the elbow rotates, the 165mm pipette swings in a giant arc!
                # To make the tip move in a perfect straight line, we must lock the wrist's angle in the WORLD frame.
                # Since world_angle = shoulder + elbow + wrist, we compensate the wrist backwards:
                new_wrist = base_shoulder + base_elbow + wrist - fsx - fex
                # Clamp to the valid 0..4095 servo range. An out-of-range value would
                # wrap when packed to 2 bytes and fling the wrist to a wrong angle.
                if new_wrist < 0 or new_wrist > 4095:
                    raise ValueError(
                        f"Grid cell {col+1}-{row+1} needs wrist tick {new_wrist}, "
                        f"outside the servo range 0..4095. Reduce the array size or move the base position.")

                name = f"Grid {col+1}-{row+1}"
                pos = [fsx, fex, new_wrist, linear]
                locations.append({"name": name, "type": "GRID", "pos": pos})
                
        return jsonify({"status": "success", "locations": locations})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/current_pos', methods=['GET'])
def get_current_pos():
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        # Get actual hardware ticks
        with use_robot():
            pos = [s.position for s in robot.servos]
            try:
                # Prevent gravity droop error by using goal_position for Z if torque was on
                z_goal = robot.linear.get_goal_position()
                if abs(pos[3] - z_goal) < 100:
                    pos[3] = z_goal
            except:
                pass
        
        # Calculate real physical location in mm using forward kinematics and apply correction
        raw_x, raw_y = forward_kinematics(pos[0], pos[1])
        cx, cy = get_correction_offset(raw_x, raw_y)
        phy_x, phy_y = raw_x - cx, raw_y - cy
        return jsonify({"status": "success", "pos": pos, "phy_x": phy_x, "phy_y": phy_y})
    except Exception as e:
        import traceback; traceback.print_exc()
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/torque/<action>', methods=['POST'])
def torque_control(action):
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    try:
        with use_robot():
            if action == 'enable': robot.enable_torque()
            elif action == 'disable': robot.disable_torque()
            else: return jsonify({"status": "error", "message": "Invalid action"}), 400
        return jsonify({"status": "success"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/torque/free_xy', methods=['POST'])
def torque_free_xy():
    """
    Release ALL the arm-positioning joints (shoulder, elbow, wrist) so the user
    can freely move the tip in X/Y by hand, while keeping the linear (Z height),
    thumb, gripper and twister HELD - otherwise the arm drops and the pipette grip
    lets go. Used by 'Free X/Y' on the calibrate page.
    """
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    try:
        free = (robot.shoulder, robot.elbow, robot.wrist)   # the XY-positioning joints
        z_target = data.get('z')
        # The UI sends {lower: true} to drop the arm to the configured Safe-Z (the
        # lowest safe point near the deck) for boundary mapping, without having to
        # know the tick value itself.
        if z_target is None and data.get('lower'):
            z_target = USER_Z_LIMIT
        z_msg = ""
        with use_robot():
            # 1) Optionally drop the arm to a teaching height FIRST (with the linear
            #    held), so the operator drags the tip near the deck while mapping the
            #    boundary. Clamp to the configured Safe-Z so we never crash the deck.
            if z_target is not None:
                z_target = int(round(float(z_target)))
                z_target = max(robot.SAFE_HEIGHT, min(z_target, USER_Z_LIMIT))
                robot.linear.enable_torque()
                cur_z = robot.linear.position
                if z_target > cur_z + 2:          # higher tick = lower height -> descending
                    # Two-stage descent to avoid momentum overshoot past the target.
                    robot.linear.set_goal_position(max(cur_z, z_target - 20))
                    time.sleep(0.3)
                robot.linear.set_goal_position(z_target)
                time.sleep(0.4)
                z_msg = f" Arm lowered to teach height {z_target} ticks ({ticks_to_depth_mm(z_target)} mm)."

            # 2) Hold everything that must stay put, then release the three XY joints.
            for s in robot.servos:
                if s not in free:
                    s.enable_torque()           # Z / thumb / gripper / twister hold
            # 3) Release shoulder, elbow AND wrist - verifying each actually dropped
            #    torque and retrying, because a single geared-servo write sometimes
            #    doesn't "take" and leaves that joint stiff (the reported bug).
            stuck = []
            for s in free:
                released = False
                for _ in range(4):
                    s.disable_torque()
                    try:
                        if s.read_bytes(s.control_table.addr_torque_enable, 1) == 0:
                            released = True
                            break
                    except Exception:
                        released = True            # can't read back; assume the write took
                        break
                    time.sleep(0.03)
                if not released:
                    stuck.append(s.id)
        if stuck:
            return jsonify({"status": "error",
                            "message": f"Could not release joint(s) {stuck} - they are still holding torque. "
                                       f"Re-issue Free X/Y or check the servo wiring."}), 500
        return jsonify({"status": "success",
                        "message": "Shoulder, elbow & wrist released for hand positioning - Z height & pipette grip held." + z_msg})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

SERVO_NAMES = {1: 'shoulder', 2: 'elbow', 3: 'wrist', 4: 'linear',
               5: 'thumb', 6: 'gripper', 7: 'twister'}

@app.route('/servo/diagnose', methods=['GET'])
def servo_diagnose():
    """
    Read each joint's live state so a misbehaving joint (e.g. the shoulder only
    moving one way) can be diagnosed: is its torque still ON (so it springs back),
    or off (then it's mechanical/wiring), and is it sitting against an angle limit?
    All reads are non-destructive.
    """
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    out = []
    try:
        with use_robot():
            for s in robot.servos:
                row = {"id": s.id, "name": SERVO_NAMES.get(s.id, str(s.id))}
                def _safe(fn):
                    try: return fn()
                    except Exception: return None
                te = _safe(lambda: s.read_bytes(s.control_table.addr_torque_enable, 1))
                limits = _safe(lambda: s.get_angle_limits())
                row["torque_enabled"] = None if te is None else bool(te)
                row["position"] = _safe(lambda: s.position)
                row["angle_limit_cw"] = limits[0] if limits else None
                row["angle_limit_ccw"] = limits[1] if limits else None
                row["wheel_mode"] = (limits == (0, 0)) if limits else None
                row["present_load"] = _safe(lambda: s.present_load)
                out.append(row)
        return jsonify({"status": "success", "servos": out})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/save_calibration', methods=['POST'])
def save_calibration():
    data = request.json
    try:
        import json
        with open("correction_map.json", "w") as f:
            json.dump(data, f, indent=2)
        load_correction_map()
        return jsonify({"status": "success"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/calibration_map', methods=['GET'])
def get_calibration_map():
    return jsonify({"status": "success", "map": CORRECTION_MAP})


@app.route('/run_sequence', methods=['POST'])
def run_sequence():
    global sequence_running, sequence_progress
    if not robot: return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running: return jsonify({"status": "error", "message": "A sequence is already running!"}), 400
    
    data = request.json
    sequence = data.get('sequence', [])
    repeat_count = int(data.get('repeat_count', 1))
    # How long the tip dwells at the dispense position AFTER the plunger press has
    # fully completed, before the arm lifts - the drop needs this time to form and
    # detach. The plunger stays depressed during the lift (prevents suck-back) and
    # is only released once the arm is fully up.
    try:
        dispense_dwell = float(data.get('dispense_dwell', 1.5))
    except (TypeError, ValueError):
        dispense_dwell = 1.5
    dispense_dwell = max(0.2, min(dispense_dwell, 10.0))

    sequence_running = True
    sequence_progress = {
        "current": 0, 
        "total": repeat_count, 
        "status": "Running",
        "start_time": time.time(),
        "elapsed": 0,
        "remaining": 0
    }
    
    def seq_loop():
        global sequence_running, sequence_progress
        try:
            with use_robot():
                robot.enable_torque()
                robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
            time.sleep(0.4)
            
            for cycle in range(repeat_count):
                if not sequence_running: break
                sequence_progress["current"] = cycle + 1
                
                for step in sequence:
                    if not sequence_running: break
                    action = step.get('action')
                    
                    if action == 'home':
                        with use_robot():
                            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
                        time.sleep(0.3)
                        if not sequence_running: break
                        with use_robot():
                            robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
                        time.sleep(0.7)
                        continue
                        
                    pos = step.get('pos')
                    block_id = step.get('block_id')
                    if not pos or len(pos) < 4: continue
                    
                    # Move XY with the same backlash-compensated approach used
                    # everywhere else, for repeatable landing during dispensing.
                    with use_robot():
                        approach_xy(pos[0], pos[1], pos[2],
                                    should_continue=lambda: sequence_running)
                    time.sleep(0.3)
                    if not sequence_running: break
                    
                    # Record actual position for verification
                    with use_robot():
                        actual_pos = robot.get_servo_positions()
                        if abs(actual_pos[0] - pos[0]) > 2 or abs(actual_pos[1] - pos[1]) > 2:
                            print(f"Warning: Position drift detected! Target {pos[:2]}, Actual {actual_pos[:2]}")
                    
                    target_z = min(pos[3], USER_Z_LIMIT)
                    
                    # NOTE on thumb/linear waits below: move_servos() caps its
                    # internal wait at 2.5s while a full thumb press takes ~3s
                    # (and a full lift at low arm speed 6s+). Without an explicit
                    # arrival wait, the press finished DURING the lift - the
                    # reported "dispensing while the arm moves up" bug.
                    seq_alive = lambda: sequence_running

                    if action == 'aspirate':
                        target_thumb = int(THUMB_LIMITS.get(str(current_pipette_slot), 2888))
                        # Depress the plunger FULLY before entering the liquid,
                        # waiting for the real arrival (overload-tolerant command).
                        _thumb_set(target_thumb)
                        _wait_servo(robot.thumb, target_thumb, 8.0, should_continue=seq_alive)
                        if not sequence_running: break

                        # Two-stage Z approach to prevent momentum overshoot
                        with use_robot():
                            robot.move_arm_servos(linear=target_z - 20)
                        time.sleep(0.15)
                        if not sequence_running: break
                        with use_robot():
                            robot.move_arm_servos(linear=target_z)
                        time.sleep(0.4)
                        if not sequence_running: break

                        # Release the plunger AT DEPTH - this is what draws the
                        # liquid - wait until fully released, then let it finish
                        # flowing before the lift.
                        _thumb_set(robot.THUMB_NEUTRAL_POSITION)
                        _wait_servo(robot.thumb, robot.THUMB_NEUTRAL_POSITION, 8.0, should_continue=seq_alive)
                        time.sleep(1.5)
                        if not sequence_running: break
                        with use_robot():
                            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
                        time.sleep(0.4)
                    elif action == 'dispense':
                        # 1) Arm DOWN to the set Z point (two-stage to prevent
                        #    momentum overshoot)
                        with use_robot():
                            robot.move_arm_servos(linear=target_z - 20)
                        time.sleep(0.15)
                        if not sequence_running: break
                        with use_robot():
                            robot.move_arm_servos(linear=target_z)
                        time.sleep(0.4)
                        if not sequence_running: break

                        # 2) Dispense AT the set Z: FULLY press the plunger and wait
                        #    until the press has physically COMPLETED there.
                        #    A dispense must push the plunger well PAST neutral to
                        #    expel. A saved limit at/below neutral (e.g. a mis-captured
                        #    1699 vs neutral 1700) would press nowhere and dispense
                        #    nothing - guard against it and fall back to a real full
                        #    press (the aspirate limit, else 3050).
                        neutral = robot.THUMB_NEUTRAL_POSITION
                        saved = THUMB_DISPENSE_LIMITS.get(str(current_pipette_slot))
                        asp = THUMB_LIMITS.get(str(current_pipette_slot))
                        if saved is not None and int(saved) > neutral + 50:
                            target_thumb = int(saved)
                        elif asp is not None and int(asp) > neutral + 50:
                            target_thumb = int(asp)
                            print(f"[dispense] slot {current_pipette_slot}: saved dispense press "
                                  f"{saved} is not past neutral ({neutral}); using aspirate full press {target_thumb}")
                        else:
                            target_thumb = 3050
                            print(f"[dispense] slot {current_pipette_slot}: no valid thumb press "
                                  f"(saved={saved}); using default full press {target_thumb}")
                        _thumb_set(target_thumb)
                        _wait_servo(robot.thumb, target_thumb, 8.0, should_continue=seq_alive)
                        if not sequence_running: break

                        # 3) Drop-settle dwell: stay AT the well, plunger pressed,
                        #    until the drop has fully formed and detached
                        time.sleep(dispense_dwell)
                        if not sequence_running: break

                        # 4) Lift FULLY, plunger still pressed (prevents suck-back)
                        with use_robot():
                            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
                        _wait_servo(robot.linear, robot.SAFE_HEIGHT, 10.0, should_continue=seq_alive)
                        if not sequence_running: break

                        # 5) Only now, fully up, release the thumb
                        _thumb_set(robot.THUMB_NEUTRAL_POSITION)
                        time.sleep(0.5)

        except Exception as e:
            print("Sequence Error:", e)
        finally:
            sequence_running = False
            sequence_progress["status"] = "Idle"
            
    threading.Thread(target=seq_loop, daemon=True).start()
    return jsonify({"status": "success", "message": "Sequence Started"})

@app.route('/stop_sequence', methods=['POST'])
def stop_sequence():
    global sequence_running, sequence_progress
    if not sequence_running:
        return jsonify({"status": "success", "message": "No sequence is currently running."})
        
    sequence_running = False
    sequence_progress["status"] = "Stopped"
    try:
        with use_robot():
            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        time.sleep(0.5)
        with use_robot():
            robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
        return jsonify({"status": "success", "message": "Sequence Stopped and Homed!"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/sequence_status', methods=['GET'])
def sequence_status():
    if sequence_running and "start_time" in sequence_progress:
        elapsed = time.time() - sequence_progress["start_time"]
        sequence_progress["elapsed"] = elapsed
        if sequence_progress["current"] > 0:
            time_per_cycle = elapsed / sequence_progress["current"]
            cycles_left = sequence_progress["total"] - sequence_progress["current"]
            sequence_progress["remaining"] = cycles_left * time_per_cycle
        else:
            sequence_progress["remaining"] = 0

    return jsonify({
        "status": "success",
        "running": sequence_running,
        "progress": sequence_progress
    })

@app.route('/save_sequence', methods=['POST'])
def save_sequence():
    try:
        data = request.json
        name = data.get('name', 'routine')
        locations = data.get('locations', [])
        sequence = data.get('sequence', [])
        
        # Clean the name to ensure it's a valid filename
        filename = "".join(c for c in name if c.isalnum() or c in (' ', '-', '_')).strip()
        if not filename:
            filename = "routine"
        filename = f"{filename}.json"
        
        with open(filename, "w") as f:
            json.dump({"locations": locations, "sequence": sequence}, f, indent=4)
        
        return jsonify({"status": "success", "message": f"Saved as {filename}"})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/list_sequences', methods=['GET'])
def list_sequences():
    try:
        files = [f for f in os.listdir('.') if f.endswith('.json') and f not in ('package.json', 'package-lock.json', 'config.json')]
        return jsonify({"status": "success", "files": files})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/load_sequence', methods=['POST'])
def load_sequence():
    try:
        data = request.json
        name = data.get('name')
        fit = bool(data.get('fit'))
        try:
            fit_scale = min(1.0, max(0.05, float(data.get('scale', 0.9))))
        except (TypeError, ValueError):
            fit_scale = 0.9
        if not name or not os.path.exists(name):
            return jsonify({"status": "error", "message": "File not found"}), 404
        with open(name, "r") as f:
            content = json.load(f)

        loaded_locs = content.get("locations", [])
        sequence = content.get("sequence", [])
        well_locs = [l for l in loaded_locs if l.get("type") == "WELL" and "pos" in l]

        fit_msg = None
        if fit and well_locs:
            if not _area_ready('substrate'):
                return jsonify({"status": "error",
                                "message": "Teach the 4 dispensing-wells corners before fitting a pattern"}), 400
            healthy, why = _area_quad_health('substrate')
            if not healthy:
                return jsonify({"status": "error",
                                "message": f"Can't fit the pattern: {why}"}), 400
            remapped = _fit_wells_to_area(well_locs, fit_scale)
            # Legacy files reference steps by raw pose - move those along too.
            for s in sequence:
                if isinstance(s.get('pos'), list) and tuple(s['pos']) in remapped:
                    s['pos'] = list(remapped[tuple(s['pos'])])
            fit_msg = f"fitted to {int(round(fit_scale * 100))}% of the wells area"

        wells = []
        for loc in well_locs:
            pose = loc["pos"]
            raw_x, raw_y = forward_kinematics(pose[0], pose[1])
            cx, cy = get_correction_offset(raw_x, raw_y)
            x, y = raw_x - cx, raw_y - cy
            try:
                u, v = _uv_from_xy('substrate', x, y)
            except Exception:
                u, v = 0.5, 0.5
            wells.append({
                "name": loc.get("name", "Well"),
                "pose": pose,
                "u": u,
                "v": v
            })
        if wells:
            CALIBRATION['locations']['wells'] = wells
            save_calibration()

        return jsonify({"status": "success", "locations": loaded_locs, "sequence": sequence, "fit": fit_msg})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/config', methods=['GET', 'POST'])
def config_ops():
    global ink_pos, well_pos
    if request.method == 'GET':
        try:
            with open("config.json", "r") as f:
                data = json.load(f)
                ink_pos = data.get("ink_pos")
                well_pos = data.get("well_pos")
            return jsonify({"status": "success", "message": "Loaded config", "ink": ink_pos, "well": well_pos})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        try:
            with open("config.json", "w") as f:
                json.dump({"ink_pos": ink_pos, "well_pos": well_pos}, f)
            return jsonify({"status": "success", "message": "Saved config"})
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500

# =============================================================================
#  CALIBRATION & AUTOMATED DISPENSING
#
#  A working area (the substrate, and optionally a separate reagent/ink area) is
#  taught ONCE per installation by jogging the tip to its four physical corners
#  and capturing the full encoder pose [shoulder, elbow, wrist, linear] at each.
#
#  Any point inside the area is addressed by normalized coordinates (u, v) where
#  u runs 0..1 from the left edge (TL->TR) and v runs 0..1 from the top edge
#  (TL->BL). The robot pose for (u, v) is the BILINEAR interpolation of the four
#  taught corner poses, done per-channel. Because we interpolate the real
#  measured encoder values - including wrist and Z - the mapping automatically
#  absorbs board tilt and arm sag without any kinematic model or correction map.
#
#  Individual reagents/wells/targets are taught the same way (jog + capture) and
#  stored persistently in calibration.json so they survive restarts.
# =============================================================================

CALIB_FILE = 'calibration.json'
CORNERS = ('TL', 'TR', 'BR', 'BL')
TICK_MIN, TICK_MAX = 0, 4095
# Keep auto-mapped joints this far from their hard travel stops. A servo commanded
# to ~0 or ~4095 jams against its end-stop and buzzes/stalls (overload) instead of
# holding position - the usual cause of the "weird sounds" while auto-mapping. The
# wrist is the most exposed here because its angle is locked across all 4 corners.
WRIST_STALL_MARGIN = 80
# A taught point is considered repeatable if encoder spread stays within this
# many ticks across cycles (1 tick ~= 0.088 deg / ~0.08 mm on the linear axis).
REPEATABILITY_TICK_TOLERANCE = 3
# Test Run flags a grid point if any arm channel lands this far from its target.
# ~8 ticks (~0.7 deg) is about the steady-state precision of these servos; the UI
# can override it per run.
TEST_RUN_TICK_TOLERANCE = 8
# A single move_arm_servos() call gives up after a 5 s internal timeout, which is
# not enough for a long full-range sweep. We re-issue the move until every
# commanded channel is within this many ticks (or a few attempts run out).
MOVE_CONVERGE_TOL = 6
# With a realistic POSITION_ERROR_MARGIN, a single move now lands within tolerance,
# so we rarely re-issue. 2 attempts is an ample safety net (was 4, which could stack
# multiple full move timeouts).
MOVE_CONVERGE_ATTEMPTS = 2

# --- Z height units --------------------------------------------------------
# The robot's linear axis is in ticks where a HIGHER tick = LOWER physical
# height. The UI works in millimetres of DEPTH below the safe travel height:
#   0 mm  = the safe travel height (SAFE_HEIGHT_REF ticks)
#   +N mm = N millimetres lower (closer to the deck)
# This gives a single, unambiguous zero without a separate surface-touch datum.
MM_PER_TICK = 0.08174            # linear-axis scale, from the deflection code
SAFE_HEIGHT_REF = 1600           # mirrors AndrewRobot.SAFE_HEIGHT (ticks)


def depth_mm_to_ticks(mm):
    return SAFE_HEIGHT_REF + int(round(float(mm) / MM_PER_TICK))


def ticks_to_depth_mm(tick):
    return round((tick - SAFE_HEIGHT_REF) * MM_PER_TICK, 2)


def classify_z_ticks(tick):
    """Return ('ok'|'warn'|'error', message) for a proposed linear Z tick."""
    if tick < TICK_MIN or tick > TICK_MAX:
        return 'error', f"Z={tick} ticks is outside the servo range {TICK_MIN}..{TICK_MAX}."
    if tick > USER_Z_LIMIT:
        return 'error', (f"Z={tick} ticks ({ticks_to_depth_mm(tick)} mm) is BELOW the configured "
                         f"Safe-Z limit ({USER_Z_LIMIT} ticks) - it would crash into the deck.")
    if tick < SAFE_HEIGHT_REF:
        return 'warn', (f"Z={ticks_to_depth_mm(tick)} mm is ABOVE the safe travel height - "
                        f"the tip may not reach the substrate/well.")
    return 'ok', f"Z set to {ticks_to_depth_mm(tick)} mm ({tick} ticks)."


def _new_area():
    return {"corners": {c: None for c in CORNERS}, "deposition_z": None,
            "width_mm": None, "height_mm": None}


# Two physically separate calibrated areas:
#   substrate = the dispensing-WELLS area  -> locations['wells']
#   reagent   = the INKS area              -> locations['inks']
# Each area's deposition_z is its height (well dispense height / ink aspirate height).
AREA_LIST = {'substrate': 'wells', 'reagent': 'inks'}

CALIBRATION = {
    "substrate": _new_area(),
    "reagent":   _new_area(),
    "locations": {"inks": [], "wells": []},
    "maps": {},   # name -> saved {corners, deposition_z, width_mm, height_mm}
}


def _targets_for(area):
    """The location list that belongs to an area: wells for substrate, inks for reagent."""
    return CALIBRATION['locations'][AREA_LIST.get(area, 'wells')]


def load_calibration():
    global CALIBRATION
    if not os.path.exists(CALIB_FILE):
        return
    try:
        with open(CALIB_FILE, 'r') as f:
            data = json.load(f)
        # Merge defensively so a partial/old file can't drop required keys.
        for area in ('substrate', 'reagent'):
            if isinstance(data.get(area), dict):
                CALIBRATION[area].update(data[area])
                corners = data[area].get('corners', {})
                CALIBRATION[area]['corners'] = {c: corners.get(c) for c in CORNERS}
                CALIBRATION[area].setdefault('deposition_z', None)
        locs = data.get('locations', {})
        for kind in ('inks', 'wells'):
            if isinstance(locs.get(kind), list):
                CALIBRATION['locations'][kind] = locs[kind]
        # Migrate the old flat 'targets' list into wells (dispensing points).
        if isinstance(locs.get('targets'), list) and locs['targets']:
            CALIBRATION['locations']['wells'].extend(locs['targets'])
        if isinstance(data.get('maps'), dict):
            CALIBRATION['maps'] = data['maps']
    except Exception as e:
        print("Failed to load calibration:", e)


def save_calibration():
    try:
        with open(CALIB_FILE, 'w') as f:
            json.dump(CALIBRATION, f, indent=2)
        return True
    except Exception as e:
        print("Failed to save calibration:", e)
        return False


load_calibration()


def _area_ready(area):
    if area not in ('substrate', 'reagent'):
        return False
    c = CALIBRATION[area]['corners']
    return all(c.get(k) is not None for k in CORNERS)


def _validate_pose(pose):
    """Raise if any channel is outside the servo range; returns a clean int list."""
    if not pose or len(pose) < 4:
        raise ValueError("Pose must have 4 values [shoulder, elbow, wrist, linear]")
    out = []
    for i, t in enumerate(pose[:4]):
        t = int(round(t))
        if t < TICK_MIN or t > TICK_MAX:
            raise ValueError(f"Pose channel {i} = {t} is outside the servo range {TICK_MIN}..{TICK_MAX}")
        out.append(t)
    return out


def bilinear_pose(area, u, v):
    """Interpolate a pose at normalized (u, v) inside an area.

    POSITION (shoulder, elbow) is interpolated in PHYSICAL XY space and then
    solved with inverse kinematics - NOT by averaging servo ticks. The arm is a
    2-link mechanism, so blending corner *angles* linearly warps the map: a click
    at the centre of the canvas could land 7-12 mm off the true physical centre
    (worse on larger areas). Interpolating the tip's XY keeps (u, v) linear in
    real space, matching the canvas and the XY-space inverse used elsewhere
    (_uv_from_xy), so a centre click now goes to the centre.

    WRIST (world angle = shoulder+elbow+wrist) and LINEAR (the taught surface-Z
    plane) still interpolate linearly in tick space - that is correct for those
    channels and keeps the pipette's world orientation and the Z plane smooth.
    At the four corners (u,v in {0,1}) this reproduces the taught ticks exactly.
    """
    c = CALIBRATION[area]['corners']
    TL, TR, BR, BL = c['TL'], c['TR'], c['BR'], c['BL']
    corners = (TL, TR, BR, BL)

    def blend(vals):
        top = vals[0] + (vals[1] - vals[0]) * u   # TL -> TR (top edge)
        bot = vals[3] + (vals[2] - vals[3]) * u   # BL -> BR (bottom edge)
        return top + (bot - top) * v

    xys = [forward_kinematics(cc[0], cc[1]) for cc in corners]
    x = blend([p[0] for p in xys])
    y = blend([p[1] for p in xys])
    world = blend([cc[0] + cc[1] + cc[2] for cc in corners])
    linear = blend([cc[3] for cc in corners])

    # Match the elbow configuration the corners were taught in (a valid rectangle
    # shares one config; the average decides cleanly even with a stray corner).
    elbow_up = (TL[1] + TR[1] + BR[1] + BL[1]) / 4.0 >= CENTER_TICK
    try:
        s, e = compute_kinematics(x, y, elbow_up=elbow_up)
    except ValueError:
        # Out of reach / degenerate corners: fall back to the old tick blend so we
        # never raise here. _area_quad_health() rejects bad rectangles upstream.
        return [int(round(blend([cc[0] for cc in corners]))),
                int(round(blend([cc[1] for cc in corners]))),
                int(round(blend([cc[2] for cc in corners]))),
                int(round(linear))]
    return [int(s), int(e), int(round(world - s - e)), int(round(linear))]


def _capture_pose(samples=6):
    """
    Read the current arm pose [shoulder, elbow, wrist, linear], averaged over a
    few samples to filter encoder read noise so taught corners/locations are as
    stable as the hardware allows. Caller holds the lock.
    """
    reads = []
    for _ in range(max(1, samples)):
        p = robot.get_servo_positions()
        reads.append(p[:4])
        time.sleep(0.02)
    return [int(round(sum(r[i] for r in reads) / len(reads))) for i in range(4)]


def reproject_targets():
    """
    Re-derive every well/ink pose from its stored (u, v) using its area's CURRENT
    corners (wells from substrate, inks from reagent). Points that carry (u, v)
    therefore always follow the active calibration - re-teaching a corner or
    loading a map no longer leaves stale, mis-placed points behind. Returns how
    many changed.
    """
    changed = 0
    for area, key in AREA_LIST.items():
        if not _area_ready(area):
            continue
        for t in CALIBRATION['locations'][key]:
            if t.get('u') is None or t.get('v') is None:
                continue
            try:
                new_pose = _validate_pose(bilinear_pose(area, t['u'], t['v']))
            except ValueError:
                continue
            if t.get('pose') != new_pose:
                t['pose'] = new_pose
                changed += 1
    return changed


# On startup, reconcile any saved targets with the saved corners (in case the
# file was written with a different corner set at some point).
if reproject_targets():
    save_calibration()


_CH_IDX = {'shoulder': 0, 'elbow': 1, 'wrist': 2, 'linear': 3}

# Backlash compensation: always make the FINAL approach to a target from the same
# direction (here, from a lower tick value upward) so the gear lash is taken up
# identically every time. This converts random +/- backlash into a consistent
# offset, which the corner calibration then cancels - the key repeatability win
# on a geared servo arm. ~35 ticks comfortably exceeds the lash (~1-2 ticks).
BACKLASH_APPROACH = 35


def settled_move(tol=MOVE_CONVERGE_TOL, attempts=MOVE_CONVERGE_ATTEMPTS,
                 should_continue=None, **goals):
    """
    Issue move_arm_servos(**goals), re-issuing it until every commanded channel is
    within `tol` ticks or `attempts` run out. A single move_arm_servos() call bails
    at its internal 5 s timeout; re-issuing lets a long sweep finish instead of
    landing hundreds of ticks short. Returns the final [s,e,w,linear] positions, or
    None if should_continue() became False (used to abort a Test Run mid-move).
    Caller holds the robot lock.
    """
    pos = None
    for _ in range(max(1, attempts)):
        robot.move_arm_servos(**goals)
        if should_continue is not None and not should_continue():
            return None
        pos = robot.get_servo_positions()
        worst = max((abs(pos[_CH_IDX[k]] - v) for k, v in goals.items() if v is not None),
                    default=0)
        if worst <= tol:
            break
        time.sleep(0.15)
    return pos


def approach_xy(s, e, w, tol=MOVE_CONVERGE_TOL, should_continue=None):
    """
    Backlash-compensated move of the shoulder/elbow/wrist to (s, e, w): first
    pre-position a fixed amount BELOW each target, then drive UP into it, so the
    final approach direction (and therefore the gear engagement) is identical for
    every point. Returns the final positions, or None if aborted.
    """
    pre = settled_move(shoulder=max(TICK_MIN, s - BACKLASH_APPROACH),
                       elbow=max(TICK_MIN, e - BACKLASH_APPROACH),
                       wrist=max(TICK_MIN, w - BACKLASH_APPROACH),
                       should_continue=should_continue)
    if should_continue is not None and pre is None:
        return None
    time.sleep(0.15)
    return settled_move(shoulder=s, elbow=e, wrist=w, tol=tol, should_continue=should_continue)


def move_to_pose(pose, two_stage=True, settle=0.5):
    """
    Safely drive the arm to a 4-channel pose. The CALLER must hold use_robot()
    so the whole multi-step move is serialized against jogs/other moves.
    """
    s, e, w, z = _validate_pose(pose)
    z = min(z, USER_Z_LIMIT)
    # 1) Lift to a safe travelling height so we never drag across the deck.
    settled_move(linear=robot.SAFE_HEIGHT)
    time.sleep(0.1)
    # 2) Swing to the XY/wrist target with a consistent (backlash-comp) approach.
    approach_xy(s, e, w)
    time.sleep(0.1)
    # 3) Two-stage descent: always reaches Z from a lower tick (above), so the
    #    linear gear lash is taken up the same way every time too.
    if two_stage and z - 20 > robot.SAFE_HEIGHT:
        settled_move(linear=z - 20)
        time.sleep(0.1)
    settled_move(linear=z)
    time.sleep(settle)


def _corners_xy(area):
    """Forward-kinematics XY (mm) of each taught corner, for drawing the area."""
    c = CALIBRATION[area]['corners']
    out = {}
    for k in CORNERS:
        if c.get(k) is not None:
            raw_x, raw_y = forward_kinematics(c[k][0], c[k][1])
            cx, cy = get_correction_offset(raw_x, raw_y)
            x = raw_x - cx
            y = raw_y - cy
            out[k] = {"x": round(x, 2), "y": round(y, 2)}
    return out


# ---- Area overlap detection (warn when wells & inks zones collide) ----

def _signed_area(poly):
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x1 * y2 - x2 * y1
    return s / 2.0


def _seg_intersect(p, q, a, b):
    x1, y1, x2, y2 = p[0], p[1], q[0], q[1]
    x3, y3, x4, y4 = a[0], a[1], b[0], b[1]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        return q
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def _clip_polygon(subject, clip):
    """Sutherland-Hodgman intersection of two convex polygons (CCW winding)."""
    out = list(subject)
    n = len(clip)
    for i in range(n):
        if not out:
            break
        a, b = clip[i], clip[(i + 1) % n]
        inp, out = out, []
        s = inp[-1]
        ss = (b[0] - a[0]) * (s[1] - a[1]) - (b[1] - a[1]) * (s[0] - a[0])
        for p in inp:
            sp = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
            if sp >= 0:
                if ss < 0:
                    out.append(_seg_intersect(s, p, a, b))
                out.append(p)
            elif ss >= 0:
                out.append(_seg_intersect(s, p, a, b))
            s, ss = p, sp
    return out


def _area_poly_xy(area):
    xy = _corners_xy(area)
    if not all(k in xy for k in CORNERS):
        return None
    poly = [(xy[k]['x'], xy[k]['y']) for k in ('TL', 'TR', 'BR', 'BL')]
    if _signed_area(poly) < 0:
        poly.reverse()
    return poly


def _areas_overlap_warning():
    """Detailed warning when the wells and inks areas overlap in physical XY."""
    if not (_area_ready('substrate') and _area_ready('reagent')):
        return None
    pa, pb = _area_poly_xy('substrate'), _area_poly_xy('reagent')
    if not pa or not pb:
        return None
    inter = _clip_polygon(pa, pb)
    if len(inter) < 3:
        return None
    ov = abs(_signed_area(inter))
    if ov < 4.0:           # ignore < ~2x2 mm grazing contact
        return None
    xs = [p[0] for p in inter]
    ys = [p[1] for p in inter]
    return (f"OVERLAP: the dispensing-wells and inks areas overlap by ~{ov:.0f} mm2 "
            f"(region ~{max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} mm at "
            f"X {min(xs):.1f}..{max(xs):.1f}, Y {min(ys):.1f}..{max(ys):.1f} mm). "
            f"Re-position one of the areas so the zones do not collide.")


def _uv_from_xy(area, px, py):
    """
    Locate a physical XY point as normalized (u, v) inside an area by inverting
    the bilinear map of the corners' forward-kinematics XY (coarse-to-fine search).
    Used to remember WHERE a height was captured so it can be re-applied as an
    offset from the interpolated surface plane at any other point.
    """
    xy = _corners_xy(area)
    if not all(k in xy for k in CORNERS):
        return None
    TL, TR, BR, BL = (xy[k] for k in ('TL', 'TR', 'BR', 'BL'))

    def P(u, v):
        tx = TL['x'] + (TR['x'] - TL['x']) * u
        ty = TL['y'] + (TR['y'] - TL['y']) * u
        bx = BL['x'] + (BR['x'] - BL['x']) * u
        by = BL['y'] + (BR['y'] - BL['y']) * u
        return tx + (bx - tx) * v, ty + (by - ty) * v

    u0, v0, span = 0.5, 0.5, 0.5
    for _ in range(7):
        best = None
        for du in (-span, -span / 2, 0, span / 2, span):
            for dv in (-span, -span / 2, 0, span / 2, span):
                u = min(1.2, max(-0.2, u0 + du))
                v = min(1.2, max(-0.2, v0 + dv))
                x, y = P(u, v)
                d = (x - px) ** 2 + (y - py) ** 2
                if best is None or d < best[0]:
                    best = (d, u, v)
        _, u0, v0 = best
        span /= 2
    return (round(u0, 4), round(v0, 4))


def _area_quad_health(area):
    """
    Sanity-check that the taught corners form a plausible rectangle in physical
    XY. Corners taught with the arm in mixed elbow configurations (or in the
    wrong order) produce a self-intersecting quad - every u,v <-> pose mapping
    is then meaningless. Returns (ok, message).
    """
    xy = _corners_xy(area)
    if not all(k in xy for k in CORNERS):
        return False, "corners not fully taught"
    def dist(a, b):
        return math.hypot(xy[a]['x'] - xy[b]['x'], xy[a]['y'] - xy[b]['y'])
    top, bottom = dist('TL', 'TR'), dist('BL', 'BR')
    left, right = dist('TL', 'BL'), dist('TR', 'BR')
    d1, d2 = dist('TL', 'BR'), dist('TR', 'BL')
    edge_ratio = max(top, bottom) / max(1e-6, min(top, bottom))
    side_ratio = max(left, right) / max(1e-6, min(left, right))
    diag_ratio = min(d1, d2) / max(1e-6, max(d1, d2))
    if edge_ratio > 1.5 or side_ratio > 1.5 or diag_ratio < 0.6:
        return False, (f"corners don't form a rectangle (top {top:.0f}mm vs bottom {bottom:.0f}mm, "
                       f"left {left:.0f}mm vs right {right:.0f}mm, diagonals {d1:.0f}/{d2:.0f}mm). "
                       f"Re-teach all 4 corners without flipping the elbow, or use "
                       f"'Auto-map rectangle from current corner'")
    return True, "ok"


def _fit_wells_to_area(well_locs, scale):
    """
    Uniformly scale + center the physical XY footprint of the given WELL
    locations into the substrate area, preserving the pattern's aspect ratio.
    Each pose is then re-derived from the area's CURRENT corners, so a pattern
    authored under an old corner calibration can be reused after re-teaching.
    Mutates each loc's 'pos' in place; returns {old_pose_tuple: new_pose_list}
    so the caller can remap pose-based sequence steps too.
    """
    xy = _corners_xy('substrate')
    TL, TR, BR, BL = (xy[k] for k in ('TL', 'TR', 'BR', 'BL'))
    # Area frame: U along the top/bottom edges, V along the sides (mm).
    ux = (TR['x'] - TL['x'] + BR['x'] - BL['x']) / 2
    uy = (TR['y'] - TL['y'] + BR['y'] - BL['y']) / 2
    vx = (BL['x'] - TL['x'] + BR['x'] - TR['x']) / 2
    vy = (BL['y'] - TL['y'] + BR['y'] - TR['y']) / 2
    w, h = math.hypot(ux, uy), math.hypot(vx, vy)
    ux, uy, vx, vy = ux / w, uy / w, vx / h, vy / h
    acx = (TL['x'] + TR['x'] + BR['x'] + BL['x']) / 4
    acy = (TL['y'] + TR['y'] + BR['y'] + BL['y']) / 4

    # Pattern footprint in the area frame, about its own bounding-box center.
    pts = []
    for l in well_locs:
        raw_x, raw_y = forward_kinematics(l['pos'][0], l['pos'][1])
        cx, cy = get_correction_offset(raw_x, raw_y)
        pts.append((raw_x - cx, raw_y - cy))
    
    pcx = sum(p[0] for p in pts) / len(pts)
    pcy = sum(p[1] for p in pts) / len(pts)
    ab = [((p[0] - pcx) * ux + (p[1] - pcy) * uy,
           (p[0] - pcx) * vx + (p[1] - pcy) * vy) for p in pts]
    amin, amax = min(a for a, _ in ab), max(a for a, _ in ab)
    bmin, bmax = min(b for _, b in ab), max(b for _, b in ab)
    ac_mid, bc_mid = (amin + amax) / 2, (bmin + bmax) / 2
    f = scale * min(w / max(amax - amin, 1e-6), h / max(bmax - bmin, 1e-6))

    mapping = {}
    for loc, (a, b) in zip(well_locs, ab):
        nx = acx + (a - ac_mid) * f * ux + (b - bc_mid) * f * vx
        ny = acy + (a - ac_mid) * f * uy + (b - bc_mid) * f * vy
        u, v = _uv_from_xy('substrate', nx, ny)
        u, v = min(1.0, max(0.0, u)), min(1.0, max(0.0, v))
        new_pose = bilinear_pose('substrate', u, v)
        mapping[tuple(loc['pos'])] = new_pose
        loc['pos'] = new_pose
    return mapping


def _surface_z(area, u, v):
    """The taught surface plane's Z (ticks) at (u, v) - bilinear of the corner Zs."""
    return bilinear_pose(area, u, v)[3]


def _deposition_info(area):
    z = CALIBRATION[area].get('deposition_z')
    if z is None:
        return {"ticks": None, "mm": None}
    return {"ticks": z, "mm": ticks_to_depth_mm(z)}


@app.route('/cal/state', methods=['GET'])
def cal_state():
    """Full calibration snapshot plus per-area ready flags, deposition Z and corner XY."""
    return jsonify({
        "status": "success",
        "calibration": CALIBRATION,
        "ready": {
            "substrate": _area_ready('substrate'),
            "reagent": _area_ready('reagent'),
        },
        "deposition": {
            "substrate": _deposition_info('substrate'),
            "reagent": _deposition_info('reagent'),
        },
        "corners_xy": {
            "substrate": _corners_xy('substrate'),
            "reagent": _corners_xy('reagent'),
        },
        # Corner-derived size estimate (mm) so the UI can pre-fill / sanity-check
        # the Area size field instead of trusting a possibly-wrong saved value.
        "est_size_mm": {
            "substrate": dict(zip(("width", "height"), _substrate_mm_per_uv('substrate'))),
            "reagent": dict(zip(("width", "height"), _substrate_mm_per_uv('reagent'))),
        },
        "z_limit_ticks": USER_Z_LIMIT,
        "z_limit_mm": ticks_to_depth_mm(USER_Z_LIMIT),
    })


@app.route('/cal/teach_corner', methods=['POST'])
def cal_teach_corner():
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    area = data.get('area', 'substrate')
    corner = data.get('corner')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    if corner not in CORNERS:
        return jsonify({"status": "error", "message": "Corner must be one of TL, TR, BR, BL"}), 400
    try:
        with use_robot():
            pose = _capture_pose()
        CALIBRATION[area]['corners'][corner] = pose
        # Optional physical dimensions, used only for grid-in-mm spacing.
        if data.get('width_mm') is not None:
            CALIBRATION[area]['width_mm'] = float(data['width_mm'])
        if data.get('height_mm') is not None:
            CALIBRATION[area]['height_mm'] = float(data['height_mm'])
        # Keep existing points aligned to the new corner (both areas reproject).
        moved = reproject_targets()
        save_calibration()
        msg = f"Taught {area} corner {corner}"
        if moved:
            msg += f" (re-aligned {moved} target(s) to the new corners)"
        return jsonify({"status": "success", "message": msg,
                        "pose": pose, "ready": _area_ready(area),
                        "warning": _areas_overlap_warning()})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/cal/teach_rect', methods=['POST'])
def cal_teach_rect():
    """
    Map an area as a TRUE RECTANGLE from ONE taught corner: the operator jogs the
    tip to the TOP-LEFT corner of the zone, supplies the rectangle's width and
    height in mm, and the robot derives the other three corners kinematically
    (locking the wrist's world angle and the elbow configuration), VISITS each
    derived corner at the zone's safe approach height for visual verification,
    then saves all four corners + the exact size. Guarantees a geometric
    width x height rectangle and warns if the two zones overlap.
    """
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running:
        return jsonify({"status": "error", "message": "Busy: a routine is running"}), 400
    data = request.json or {}
    area = data.get('area', 'substrate')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    try:
        width = float(data['width_mm'])
        height = float(data['height_mm'])
        if width <= 0 or height <= 0:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return jsonify({"status": "error",
                        "message": "Enter the rectangle's width and height in mm first"}), 400
    visit = bool(data.get('visit', True))
    corner_at = (data.get('corner') or 'TL').upper()
    if corner_at not in CORNERS:
        corner_at = 'TL'
    try:
        with use_robot():
            taught = _capture_pose()
        raw_px, raw_py = forward_kinematics(taught[0], taught[1])
        # Convert raw physical coordinates to true physical coordinates
        cx0, cy0 = get_correction_offset(raw_px, raw_py)
        px0 = raw_px - cx0
        py0 = raw_py - cy0
        
        # Auto-mapped dimensions generated by inverse_kinematics MUST include the
        # correction map offsets, otherwise the returned ticks will point into the
        # wrong physical space and distort the requested dimensions when printed.
        elbow_up = taught[1] >= CENTER_TICK     # keep the present elbow configuration
        w_world = taught[0] + taught[1] + taught[2]   # lock the wrist's world angle
        
        x_tl = px0 + (height if corner_at in ('BL', 'BR') else 0.0)
        y_tl = py0 + (width if corner_at in ('TR', 'BR') else 0.0)
        phys = {'TL': (x_tl, y_tl),
                'TR': (x_tl, y_tl - width),
                'BR': (x_tl - height, y_tl - width),
                'BL': (x_tl - height, y_tl)}
        corners = {}
        for name, (px, py) in phys.items():
            if name == corner_at:
                corners[name] = [int(v) for v in taught]   # keep the exact taught pose
                continue
            try:
                # Use standard inverse_kinematics which includes the correction offset
                fsx, fex = inverse_kinematics(px, py, elbow_up=elbow_up)
            except ValueError:
                return jsonify({"status": "error",
                                "message": f"Corner {name} of a {width:g}x{height:g} mm rectangle is OUT OF "
                                           f"REACH (arm max {MAX_REACH:.0f} mm from the base). Reduce the size, "
                                           f"move the area closer to the base, or set 'I'm at corner' to the "
                                           f"corner you actually jogged to (you taught {corner_at})."}), 400
            wr = w_world - fsx - fex
            if not (TICK_MIN <= fsx <= TICK_MAX and TICK_MIN <= fex <= TICK_MAX):
                return jsonify({"status": "error",
                                "message": f"Corner {name} needs shoulder/elbow values outside the servo "
                                           f"range. Reduce the rectangle or teach from a different corner."}), 400
            # Keep the (locked) wrist clear of its hard travel stops, not just inside
            # the raw 0..4095 range - a wrist jammed at an end-stop stalls and buzzes.
            if not (TICK_MIN + WRIST_STALL_MARGIN <= wr <= TICK_MAX - WRIST_STALL_MARGIN):
                return jsonify({"status": "error",
                                "message": f"Corner {name} would drive the wrist to {int(wr)} ticks, into its "
                                           f"travel stop (safe band {TICK_MIN + WRIST_STALL_MARGIN}.."
                                           f"{TICK_MAX - WRIST_STALL_MARGIN}). The wrist angle is locked across "
                                           f"all 4 corners, so re-teach with the tip rotated toward the middle "
                                           f"of its range, reduce the rectangle, or teach from a different "
                                           f"corner."}), 400
            corners[name] = [int(fsx), int(fex), int(round(wr)), int(taught[3])]
        # Flag corners sitting near the arm's reach limit: the shoulder/elbow are
        # weakest at full extension, so the arm can STALL (overload) reaching them -
        # the usual cause of the grinding/buzzing while auto-mapping or running there.
        # Computed BEFORE the visit so we never physically drive into a stall.
        safe_reach = MAX_REACH * 0.95
        near = sorted(n for n, (px, py) in phys.items() if math.hypot(px, py) > safe_reach)
        reach_warn = None
        if near:
            far = max(math.hypot(*phys[n]) for n in near)
            reach_warn = (f"Corner(s) {', '.join(near)} are near the arm's reach limit "
                          f"({far:.0f} of {MAX_REACH:.0f} mm) — the arm is weak there and may stall "
                          f"(overload). Move the plate closer to the base or use a smaller area.")
        # Visit each corner at the SAFE APPROACH height (crash guard) so the operator
        # can watch the rectangle being traced before it is saved - but ONLY if every
        # corner is comfortably in reach. Driving into a stall-prone corner is exactly
        # what makes the weird sounds, so when any corner is near the limit we skip the
        # motion and just save + warn instead of grinding the servos.
        visited = False
        if visit and not near:
            cap = AREA_TRAVEL_Z.get(area, 3000)
            with use_robot():
                robot.enable_torque()
                for name in ('TL', 'TR', 'BR', 'BL'):
                    p = corners[name]
                    move_to_pose([p[0], p[1], p[2], min(p[3], cap, USER_Z_LIMIT)],
                                 two_stage=False, settle=0.2)
            visited = True
        CALIBRATION[area]['corners'] = {k: corners[k] for k in CORNERS}
        CALIBRATION[area]['width_mm'] = width
        CALIBRATION[area]['height_mm'] = height
        moved = reproject_targets()
        save_calibration()
        warn = " | ".join(w for w in (_areas_overlap_warning(), reach_warn) if w) or None
        msg = f"Mapped {area} as a {width} x {height} mm rectangle from the taught {corner_at} corner"
        if visited:
            msg += " (all 4 corners visited at safe height)"
        elif visit and near:
            msg += " (corners saved but NOT visited — too close to the reach limit to move safely)"
        if moved:
            msg += f"; re-aligned {moved} point(s)"
        return jsonify({"status": "success", "message": msg, "warning": warn,
                        "corners_xy": _corners_xy(area), "ready": _area_ready(area)})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/cal/complete_rect', methods=['POST'])
def cal_complete_rect():
    """
    TILT-AWARE rectangle mapping from TWO taught ADJACENT corners. /cal/teach_rect
    derives 3 corners from 1 and ASSUMES the plate is square to the robot's base, so
    a rotated plate gets phantom corners flung off-axis (often "out of reach"). This
    instead reads two adjacent corners the operator already taught, measures the TRUE
    edge angle between them, and builds a perfect rectangle ALONG that angle - so a
    plate at any rotation maps correctly. Workflow: teach two adjacent corners
    (e.g. TL then TR) with the normal corner buttons, enter the perpendicular size,
    then call this.
    """
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running:
        return jsonify({"status": "error", "message": "Busy: a routine is running"}), 400
    data = request.json or {}
    area = data.get('area', 'substrate')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    try:
        width = float(data['width_mm'])
        height = float(data['height_mm'])
        if width <= 0 or height <= 0:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        return jsonify({"status": "error",
                        "message": "Enter the rectangle's width and height in mm first"}), 400
    visit = bool(data.get('visit', True))
    cyc = CORNERS  # ('TL','TR','BR','BL') in cyclic order
    try:
        src = CALIBRATION[area]['corners']
        taught = [k for k in cyc if src.get(k) is not None]
        if len(taught) != 2:
            return jsonify({"status": "error",
                            "message": f"Teach exactly TWO adjacent corners first (you have "
                                       f"{len(taught)}). Jog to one corner and teach it, jog to the "
                                       f"NEXT corner along an edge and teach it, then click this."}), 400
        A, B = taught
        if (cyc.index(A) - cyc.index(B)) % 4 not in (1, 3):
            return jsonify({"status": "error",
                            "message": f"{A} and {B} are DIAGONAL, not adjacent. Teach two corners that "
                                       f"share an EDGE (e.g. TL+TR, or TL+BL)."}), 400

        def true_xy(pose):
            rx, ry = forward_kinematics(pose[0], pose[1])
            cx, cy = get_correction_offset(rx, ry)
            return (rx - cx, ry - cy)
        A_xy, B_xy = true_xy(src[A]), true_xy(src[B])
        ex, ey = B_xy[0] - A_xy[0], B_xy[1] - A_xy[1]
        edge_len = math.hypot(ex, ey)
        if edge_len < 1.0:
            return jsonify({"status": "error",
                            "message": "The two taught corners are at the same spot — re-teach them at "
                                       "opposite ends of one edge."}), 400

        # Taught edge is the width edge for {TL,TR}/{BL,BR}, else the height edge.
        is_horiz = frozenset((A, B)) in ({frozenset(('TL', 'TR')), frozenset(('BL', 'BR'))})
        edge_dim = width if is_horiz else height     # what the taught edge should measure
        perp_dim = height if is_horiz else width     # how far the rectangle extends perpendicular

        def other_neighbour(c, notc):
            i = cyc.index(c)
            n1, n2 = cyc[(i - 1) % 4], cyc[(i + 1) % 4]
            return n1 if n2 == notc else n2
        A2, B2 = other_neighbour(A, B), other_neighbour(B, A)

        # Perpendicular unit vector has two choices; pick the one that makes the full
        # TL->TR->BR->BL outline wind CLOCKWISE (negative signed area) - the same
        # canonical orientation teach_rect builds - so the rectangle lands on the
        # correct side no matter which edge was taught or how the plate is rotated.
        ux, uy = ex / edge_len, ey / edge_len

        def shoelace(pts):
            return sum(pts[i][0] * pts[(i + 1) % 4][1] - pts[(i + 1) % 4][0] * pts[i][1]
                       for i in range(4))
        chosen = None
        for sign in (1.0, -1.0):
            px, py = -uy * sign, ux * sign
            pos = {A: A_xy, B: B_xy,
                   A2: (A_xy[0] + px * perp_dim, A_xy[1] + py * perp_dim),
                   B2: (B_xy[0] + px * perp_dim, B_xy[1] + py * perp_dim)}
            if shoelace([pos[c] for c in cyc]) < 0:
                chosen = pos
                break
        if chosen is None:
            chosen = pos

        # Taught corners keep their EXACT pose; derived corners use IK with the elbow
        # config + locked wrist world-angle taken from taught corner A.
        aP = src[A]
        elbow_up = aP[1] >= CENTER_TICK
        w_world = aP[0] + aP[1] + aP[2]
        z_lin = aP[3]
        corners = {}
        for name in cyc:
            if name in (A, B):
                corners[name] = [int(v) for v in src[name]]
                continue
            cx_, cy_ = chosen[name]
            try:
                fsx, fex = inverse_kinematics(cx_, cy_, elbow_up=elbow_up)
            except ValueError:
                return jsonify({"status": "error",
                                "message": f"Derived corner {name} is out of reach. Check that width/height "
                                           f"match the real plate (taught edge measured {edge_len:.0f} mm)."}), 400
            wr = w_world - fsx - fex
            if not (TICK_MIN <= fsx <= TICK_MAX and TICK_MIN <= fex <= TICK_MAX):
                return jsonify({"status": "error",
                                "message": f"Derived corner {name} needs shoulder/elbow outside the servo range."}), 400
            if not (TICK_MIN + WRIST_STALL_MARGIN <= wr <= TICK_MAX - WRIST_STALL_MARGIN):
                return jsonify({"status": "error",
                                "message": f"Derived corner {name} would jam the wrist ({int(wr)} ticks). Keep "
                                           f"the tip's rotation mid-range while teaching the two corners."}), 400
            corners[name] = [int(fsx), int(fex), int(round(wr)), int(z_lin)]

        phys = {k: chosen[k] for k in cyc}
        safe_reach = MAX_REACH * 0.95
        near = sorted(n for n in cyc if math.hypot(*phys[n]) > safe_reach)
        reach_warn = None
        if near:
            far = max(math.hypot(*phys[n]) for n in near)
            reach_warn = (f"Corner(s) {', '.join(near)} sit near the arm's reach limit "
                          f"({far:.0f} of {MAX_REACH:.0f} mm) and may stall; saved without visiting.")
        size_warn = None
        if abs(edge_len - edge_dim) > max(5.0, 0.1 * edge_dim):
            size_warn = (f"Heads up: the taught edge measures {edge_len:.0f} mm but you entered "
                         f"{edge_dim:.0f} mm for that side — the rectangle uses your entered size.")

        visited = False
        if visit and not near:
            cap = AREA_TRAVEL_Z.get(area, 3000)
            with use_robot():
                robot.enable_torque()
                for name in cyc:
                    p = corners[name]
                    move_to_pose([p[0], p[1], p[2], min(p[3], cap, USER_Z_LIMIT)],
                                 two_stage=False, settle=0.2)
            visited = True

        CALIBRATION[area]['corners'] = {k: corners[k] for k in CORNERS}
        CALIBRATION[area]['width_mm'] = width
        CALIBRATION[area]['height_mm'] = height
        moved = reproject_targets()
        save_calibration()
        warn = " | ".join(w for w in (_areas_overlap_warning(), reach_warn, size_warn) if w) or None
        msg = (f"Mapped {area} as a tilt-corrected {width:g} x {height:g} mm rectangle from taught "
               f"{A}+{B} (real edge angle {math.degrees(math.atan2(ey, ex)):.1f}°)")
        if visited:
            msg += " — all 4 corners visited"
        elif visit and near:
            msg += " — saved without visiting (near reach limit)"
        if moved:
            msg += f"; re-aligned {moved} point(s)"
        return jsonify({"status": "success", "message": msg, "warning": warn,
                        "corners_xy": _corners_xy(area), "ready": _area_ready(area)})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/cal/clear_area', methods=['POST'])
def cal_clear_area():
    data = request.json or {}
    area = data.get('area', 'substrate')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    CALIBRATION[area] = _new_area()
    save_calibration()
    return jsonify({"status": "success", "message": f"Cleared {area} calibration"})


@app.route('/cal/clear_corner', methods=['POST'])
def cal_clear_corner():
    """Reset a SINGLE taught corner so it can be re-mapped (jog + re-teach)."""
    data = request.json or {}
    area = data.get('area', 'substrate')
    corner = data.get('corner')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    if corner not in CORNERS:
        return jsonify({"status": "error", "message": "Corner must be one of TL, TR, BR, BL"}), 400
    CALIBRATION[area]['corners'][corner] = None
    save_calibration()
    return jsonify({"status": "success",
                    "message": f"Reset {area} corner {corner} - jog the arm and teach it again to re-map",
                    "ready": _area_ready(area)})


@app.route('/cal/preview', methods=['POST'])
def cal_preview():
    """Return the interpolated pose for (u, v) WITHOUT moving (for validation)."""
    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error", "message": f"{area} is not fully calibrated"}), 400
    try:
        u = float(data['u'])
        v = float(data['v'])
    except (KeyError, TypeError, ValueError):
        return jsonify({"status": "error", "message": "u and v are required numbers"}), 400
    pose = bilinear_pose(area, u, v)
    extrapolating = not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0)
    in_range = all(TICK_MIN <= t <= TICK_MAX for t in pose)
    return jsonify({"status": "success", "pose": pose,
                    "extrapolating": extrapolating, "in_range": in_range})


@app.route('/cal/goto_uv', methods=['POST'])
def cal_goto_uv():
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running:
        return jsonify({"status": "error", "message": "Busy: a routine is running"}), 400
    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error", "message": f"{area} is not fully calibrated"}), 400
    try:
        u = float(data['u'])
        v = float(data['v'])
        if not (-0.05 <= u <= 1.05 and -0.05 <= v <= 1.05):
            return jsonify({"status": "error", "message": "u,v must be within the calibrated area"}), 400
        pose = bilinear_pose(area, u, v)
        # CRASH GUARD: the interpolated pose Z is the taught SURFACE plane, so a
        # plain goto would drive the tip into the substrate / ink vial. Clamp the
        # descent to the area's safe approach height (substrate 3000 / reagent
        # 2500 by default); the user jogs Z down manually from there and captures
        # the real working height.
        cap = AREA_TRAVEL_Z.get(area)
        clamped = False
        if cap is not None and pose[3] > cap:
            pose[3] = int(cap)
            clamped = True
        pose[3] = min(pose[3], USER_Z_LIMIT)
        descend = bool(data.get('descend', True))
        with use_robot():
            robot.enable_torque()
            move_to_pose(pose, two_stage=descend)
        msg = f"Moved to ({u:.3f}, {v:.3f})"
        if clamped:
            msg += f" — stopped at safe approach height ({cap} ticks); jog Z down to reach the surface"
        return jsonify({"status": "success", "message": msg, "pose": pose,
                        "z_clamped": clamped})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/cal/repeatability', methods=['POST'])
def cal_repeatability():
    """
    Drive to the same (u, v) target 'cycles' times, each time approaching from
    home, and report the encoder spread per channel. This is the objective
    repeatability check the user asked for.
    """
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running:
        return jsonify({"status": "error", "message": "Busy: a routine is running"}), 400
    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error", "message": f"{area} is not fully calibrated"}), 400
    try:
        u = float(data.get('u', 0.5))
        v = float(data.get('v', 0.5))
        cycles = max(2, min(int(data.get('cycles', 10)), 50))
        target = bilinear_pose(area, u, v)
        # CRASH GUARD: measure repeatability at the safe approach height, not at
        # the taught surface - encoder spread is identical, without touch risk.
        cap = AREA_TRAVEL_Z.get(area)
        if cap is not None and target[3] > cap:
            target[3] = int(cap)
        _validate_pose(target)
        samples = []
        with use_robot():
            robot.enable_torque()
            for _ in range(cycles):
                # Retreat to home so every approach starts from the same place.
                robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
                time.sleep(0.3)
                robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
                time.sleep(0.6)
                move_to_pose(target, two_stage=True, settle=0.5)
                samples.append(robot.get_servo_positions()[:4])
            robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
        labels = ['shoulder', 'elbow', 'wrist', 'linear']
        stats = []
        for ch in range(4):
            vals = [s[ch] for s in samples]
            mean = sum(vals) / len(vals)
            spread = max(vals) - min(vals)
            std = (sum((x - mean) ** 2 for x in vals) / len(vals)) ** 0.5
            stats.append({"channel": labels[ch], "min": min(vals), "max": max(vals),
                          "mean": round(mean, 2), "spread": spread, "std": round(std, 3)})
        max_spread = max(st["spread"] for st in stats)
        return jsonify({"status": "success", "target": target, "cycles": cycles,
                        "samples": samples, "stats": stats, "max_spread": max_spread,
                        "passed": max_spread <= REPEATABILITY_TICK_TOLERANCE,
                        "tolerance": REPEATABILITY_TICK_TOLERANCE})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


def _loc_list(kind):
    if kind not in ('inks', 'wells'):
        return None
    return CALIBRATION['locations'][kind]


@app.route('/loc/save', methods=['POST'])
def loc_save():
    """Capture the current arm pose as a named ink / well / target location."""
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    data = request.json or {}
    kind = data.get('kind')
    items = _loc_list(kind)
    if items is None:
        return jsonify({"status": "error", "message": "kind must be inks or wells"}), 400
    name = (data.get('name') or '').strip() or f"{kind[:-1]} {len(items) + 1}"
    try:
        with use_robot():
            pose = _capture_pose()
        entry = {"name": name, "pose": pose}
        # Remember the (u, v) if supplied so the point follows re-calibration.
        if data.get('u') is not None and data.get('v') is not None:
            entry["u"] = float(data['u'])
            entry["v"] = float(data['v'])
        items.append(entry)
        save_calibration()
        return jsonify({"status": "success", "message": f"Saved {kind[:-1]} '{name}'",
                        "entry": entry, "index": len(items) - 1})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/loc/add_uv', methods=['POST'])
def loc_add_uv():
    """Add a point to an area (well for substrate, ink for reagent) from (u, v)."""
    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error", "message": f"{area} is not fully calibrated"}), 400
    try:
        u = float(data['u'])
        v = float(data['v'])
        items = _targets_for(area)
        label = AREA_LIST.get(area, 'wells')[:-1]   # 'well' / 'ink'
        name = (data.get('name') or '').strip() or f"{label} {len(items) + 1}"
        pose = _validate_pose(bilinear_pose(area, u, v))
        entry = {"name": name, "pose": pose, "u": u, "v": v}
        items.append(entry)
        save_calibration()
        return jsonify({"status": "success", "message": f"Added {label} '{name}'", "entry": entry})
    except (KeyError, TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/loc/list', methods=['GET'])
def loc_list_all():
    return jsonify({"status": "success", "locations": CALIBRATION['locations']})


@app.route('/loc/goto', methods=['POST'])
def loc_goto():
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running:
        return jsonify({"status": "error", "message": "Busy: a routine is running"}), 400
    data = request.json or {}
    kind = data.get('kind')
    items = _loc_list(kind)
    if items is None:
        return jsonify({"status": "error", "message": "Bad kind"}), 400
    try:
        idx = int(data.get('index'))
        if idx < 0 or idx >= len(items):
            return jsonify({"status": "error", "message": "Index out of range"}), 400
        pose = list(items[idx]['pose'])
        # CRASH GUARD: saved wells/inks carry surface-level Z. The "Go" button is a
        # positioning aid, so descend only to the area's safe approach height; the
        # full depth is used only inside an actual dispensing run.
        cap = AREA_TRAVEL_Z.get('substrate' if kind == 'wells' else 'reagent')
        clamped = False
        if cap is not None and pose[3] > cap:
            pose[3] = int(cap)
            clamped = True
        descend = bool(data.get('descend', True))
        with use_robot():
            robot.enable_torque()
            move_to_pose(pose, two_stage=descend)
        msg = f"Moved to {items[idx]['name']}"
        if clamped:
            msg += f" (stopped at safe approach height {cap} ticks)"
        return jsonify({"status": "success", "message": msg, "pose": pose})
    except (TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": str(e)}), 400
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/loc/delete', methods=['POST'])
def loc_delete():
    data = request.json or {}
    kind = data.get('kind')
    items = _loc_list(kind)
    if items is None:
        return jsonify({"status": "error", "message": "Bad kind"}), 400
    try:
        idx = int(data.get('index'))
        if idx < 0 or idx >= len(items):
            return jsonify({"status": "error", "message": "Index out of range"}), 400
        removed = items.pop(idx)
        save_calibration()
        return jsonify({"status": "success", "message": f"Deleted {removed['name']}"})
    except (TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route('/loc/clear_targets', methods=['POST'])
def loc_clear_targets():
    """Clear all points for an area (wells for substrate, inks for reagent)."""
    data = request.json or {}
    area = data.get('area', 'substrate')
    key = AREA_LIST.get(area, 'wells')
    n = len(CALIBRATION['locations'][key])
    CALIBRATION['locations'][key] = []
    save_calibration()
    return jsonify({"status": "success", "message": f"Cleared {n} {key}"})


@app.route('/cal/generate_targets', methods=['POST'])
def cal_generate_targets():
    """
    Build a regular grid of targets across the calibrated substrate. rows/cols
    are evenly spaced; 'margin' (0..0.4) insets the grid from the edges so the
    outermost points aren't right on the taught corners. Nothing moves.
    """
    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error", "message": f"{area} is not fully calibrated"}), 400
    try:
        rows = max(1, int(data.get('rows', 1)))
        cols = max(1, int(data.get('cols', 1)))
        margin = min(max(float(data.get('margin', 0.0)), 0.0), 0.4)
        replace = bool(data.get('replace', False))
    except (TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": str(e)}), 400

    def axis(n, i):
        if n == 1:
            return 0.5
        lo, hi = margin, 1.0 - margin
        return lo + (hi - lo) * i / (n - 1)

    key = AREA_LIST.get(area, 'wells')
    targets = [] if replace else list(CALIBRATION['locations'][key])
    generated = []
    try:
        for r in range(rows):
            for c in range(cols):
                u, v = axis(cols, c), axis(rows, r)
                pose = _validate_pose(bilinear_pose(area, u, v))
                entry = {"name": f"R{r + 1}C{c + 1}", "pose": pose, "u": u, "v": v}
                generated.append(entry)
        targets.extend(generated)
        CALIBRATION['locations'][key] = targets
        save_calibration()
        return jsonify({"status": "success", "message": f"Generated {len(generated)} {key}",
                        "generated": generated, "targets": targets})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


def _substrate_mm_per_uv(area):
    """Physical mm spanned by the full u-axis (TL->TR) and v-axis (TL->BL)."""
    xy = _corners_xy(area)
    if not all(k in xy for k in ('TL', 'TR', 'BL')):
        return None, None
    tl, tr, bl = xy['TL'], xy['TR'], xy['BL']
    width = math.hypot(tr['x'] - tl['x'], tr['y'] - tl['y'])    # mm across u
    height = math.hypot(bl['x'] - tl['x'], bl['y'] - tl['y'])   # mm down v
    return (width or None), (height or None)


@app.route('/cal/generate_array', methods=['POST'])
def cal_generate_array():
    """
    Build a dispensing array anchored at a reference point (u_ref, v_ref) using
    PHYSICAL spacing: delta_x_mm along the substrate's u-axis and delta_y_mm along
    its v-axis, rep_x by rep_y points. The reference Z (ticks) is applied to every
    point so the whole array dispenses at the height you set. Nothing moves.
    """
    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error", "message": f"{area} is not fully calibrated"}), 400
    try:
        u_ref = float(data['u'])
        v_ref = float(data['v'])
        dx_mm = float(data.get('delta_x_mm', 0))
        dy_mm = float(data.get('delta_y_mm', 0))
        rep_x = max(1, int(data.get('rep_x', 1)))
        rep_y = max(1, int(data.get('rep_y', 1)))
        replace = bool(data.get('replace', False))
        z = data.get('z')
        z = int(z) if z is not None else None
    except (KeyError, TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": f"Bad parameters: {e}"}), 400

    # Pitch conversion needs the area's PHYSICAL size - and that is FULLY DEFINED
    # by the taught corners, so the user never has to type it. We derive mm across
    # the u-axis (TL->TR) and v-axis (TL->BL) straight from the corner geometry;
    # the array then needs only a pitch (mm) and a row/col count.
    #
    # If an accurate measured size was saved by Auto-map (and still agrees with the
    # corners), we keep it for best precision; a stale/mistyped value that disagrees
    # with the corners by >3x is ignored rather than allowed to blow up the pitch.
    est_w, est_h = _substrate_mm_per_uv(area)
    if not est_w or not est_h:
        return jsonify({"status": "error",
                        "message": "Could not measure the substrate size from its corners - re-teach them"}), 400
    sw = CALIBRATION[area].get('width_mm')
    sh = CALIBRATION[area].get('height_mm')
    width_mm = sw if (sw and 1/3 < sw / est_w < 3) else est_w
    height_mm = sh if (sh and 1/3 < sh / est_h < 3) else est_h
    note = ""

    # mm spacing -> normalized (u, v) spacing.
    du = dx_mm / width_mm
    dv = dy_mm / height_mm

    key = AREA_LIST.get(area, 'wells')
    prefix = 'Ink' if key == 'inks' else 'W'
    targets = [] if replace else list(CALIBRATION['locations'][key])
    # Surface-following Z: the captured Z was taken AT the reference point, so keep
    # it as an offset from the taught surface plane there and re-apply that offset
    # at every point. A constant tick across a tilted/sagging area made the tip
    # collide with the substrate partway through the array.
    z_offset = None
    if z is not None:
        z_offset = z - _surface_z(area, u_ref, v_ref)
    generated, skipped = [], []
    for j in range(rep_y):
        for i in range(rep_x):
            u = u_ref + i * du
            v = v_ref + j * dv
            try:
                pose = bilinear_pose(area, u, v)
                if z_offset is not None:
                    # pose[3] is already the surface plane at (u, v)
                    pose[3] = int(round(pose[3] + z_offset))
                pose = _validate_pose(pose)
            except ValueError:
                skipped.append(f"R{j+1}C{i+1}")
                continue
            generated.append({"name": f"{prefix}{j+1}-{i+1}", "pose": pose, "u": u, "v": v})
    if not generated:
        return jsonify({"status": "error",
                        "message": "No array points are reachable - check spacing/reference"}), 400
    targets.extend(generated)
    CALIBRATION['locations'][key] = targets
    save_calibration()
    msg = f"Generated {len(generated)} array points ({rep_y} rows x {rep_x} cols, dx={dx_mm} x dy={dy_mm} mm pitch)"
    if skipped:
        msg += f"; skipped {len(skipped)} out-of-reach"
    msg += note
    return jsonify({"status": "success", "message": msg,
                    "generated": generated, "skipped": skipped, "targets": targets})


@app.route('/dispense/plan', methods=['POST'])
def dispense_plan():
    """
    Assemble an automated dispense routine in the SAME step format the existing
    /run_sequence executor consumes, so we reuse the tested aspirate/dispense
    motion instead of re-implementing it.

    Body: { ink_index, target_indices?: [..], repeat?, home_between?: bool }
    Builds "aspirate(ink) -> dispense(each well)" for one selected ink across all
    wells. Returns { locations, sequence } ready to POST to /run_sequence.
    """
    data = request.json or {}
    inks = CALIBRATION['locations']['inks']
    wells = CALIBRATION['locations']['wells']
    try:
        ink_index = int(data.get('ink_index', 0))
    except (TypeError, ValueError):
        return jsonify({"status": "error", "message": "ink_index must be an integer"}), 400
    if not inks:
        return jsonify({"status": "error", "message": "No inks defined"}), 400
    if ink_index < 0 or ink_index >= len(inks):
        return jsonify({"status": "error", "message": "ink_index out of range"}), 400
    if not wells:
        return jsonify({"status": "error", "message": "No wells defined"}), 400

    tgt_idx = data.get('target_indices')
    if tgt_idx is None:
        tgt_idx = list(range(len(wells)))
    chosen = []
    for i in tgt_idx:
        i = int(i)
        if 0 <= i < len(wells):
            chosen.append(wells[i])
    if not chosen:
        return jsonify({"status": "error", "message": "No valid wells selected"}), 400

    home_between = bool(data.get('home_between', True))
    # Each step drops to its area's configured height: wells -> substrate Deposition
    # Z, ink aspirate -> reagent Deposition Z. Falls back to the point's taught Z.
    #
    # PRECISION FIX: a single tick value is NOT a constant physical height across
    # the area (board tilt + extension-dependent arm sag), so a height captured at
    # one well dug into the substrate at others. When we know WHERE the height was
    # captured (deposition_uv), we convert it into an OFFSET from the taught
    # surface plane and re-apply that offset per point, so the tip follows the
    # substrate's real plane at every well/ink.
    def _z_for(area, z_const, uv_cap, t):
        if z_const is None:
            return None                      # keep the point's own taught Z
        if uv_cap and t.get('u') is not None and t.get('v') is not None and _area_ready(area):
            offset = z_const - _surface_z(area, uv_cap[0], uv_cap[1])
            z = int(round(_surface_z(area, t['u'], t['v']) + offset))
        else:
            z = z_const                       # legacy: constant tick
        return max(TICK_MIN, min(z, USER_Z_LIMIT))

    well_z = CALIBRATION['substrate'].get('deposition_z')
    well_uv = CALIBRATION['substrate'].get('deposition_uv')
    ink_z = CALIBRATION['reagent'].get('deposition_z')
    ink_uv = CALIBRATION['reagent'].get('deposition_uv')

    # SAFETY: without a captured working height, the fallback is each point's taught
    # Z - which is the SURFACE/safe-limit plane (~3124 = deepest). The run executor
    # drives straight to that with no clamp, so the tip crashes into the ink vial /
    # substrate. Refuse to build the plan until both heights are captured, naming
    # exactly which one is missing, instead of producing a crashing routine.
    missing = []
    if well_z is None:
        missing.append("WELL dispense height (Calibrate ▸ Dispensing wells ▸ 'Well dispense height' ▸ Capture)")
    if ink_z is None:
        missing.append("INK aspirate height (Calibrate ▸ Inks ▸ 'Ink aspirate height' ▸ Capture)")
    if missing:
        return jsonify({"status": "error",
                        "message": "Capture the working height first, or the tip will crash: "
                                   + "; ".join(missing)}), 400

    ink = inks[ink_index]
    ink_pose = list(ink['pose'])
    zi = _z_for('reagent', ink_z, ink_uv, ink)
    if zi is not None:
        ink_pose[3] = zi
    locations = [{"name": ink['name'], "type": "INK", "pos": ink_pose}]
    sequence = []
    for t in chosen:
        pose = list(t['pose'])
        zw = _z_for('substrate', well_z, well_uv, t)
        if zw is not None:
            pose[3] = zw
        sequence.append({"action": "aspirate", "pos": ink_pose})
        sequence.append({"action": "dispense", "pos": pose})
        if home_between:
            sequence.append({"action": "home"})
        locations.append({"name": t['name'], "type": "WELL", "pos": pose})
    return jsonify({"status": "success", "locations": locations, "sequence": sequence,
                    "message": f"Planned {len(chosen)} × (aspirate {ink['name']} → dispense well)"})


# -------------------------- Z HEIGHT CONFIGURATION --------------------------

def _resolve_z(data):
    """
    Resolve a target Z tick from the request: either an explicit 'value_mm'
    (depth below safe height) or the robot's CURRENT linear position. Returns
    (tick, source) or raises ValueError.
    """
    if data.get('value_mm') is not None:
        return depth_mm_to_ticks(data['value_mm']), 'entered'
    if not robot:
        raise ValueError("Robot not initialized - cannot capture current Z")
    with use_robot():
        tick = robot.get_servo_positions()[3]
    return int(tick), 'captured'


@app.route('/cal/set_deposition_z', methods=['POST'])
def set_deposition_z():
    """Set the substrate Deposition Z, from a typed mm value or the live position."""
    data = request.json or {}
    area = data.get('area', 'substrate')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    try:
        tick, source = _resolve_z(data)
    except ValueError as e:
        return jsonify({"status": "error", "message": str(e)}), 400
    level, message = classify_z_ticks(tick)
    if level == 'error':
        return jsonify({"status": "error", "message": message}), 400
    CALIBRATION[area]['deposition_z'] = tick
    # Remember WHERE a live capture happened (as u,v) so the height can be applied
    # as an offset from the taught surface plane at every other point. A constant
    # tick is NOT a constant physical height across a tilted/sagging area - that
    # caused the tip to dig into the substrate partway through arrays.
    uv = None
    if source == 'captured' and robot and _area_ready(area):
        try:
            with use_robot():
                p = robot.get_servo_positions()
            raw_px, raw_py = forward_kinematics(p[0], p[1])
            cx, cy = get_correction_offset(raw_px, raw_py)
            uv = _uv_from_xy(area, raw_px - cx, raw_py - cy)
        except Exception:
            uv = None
    CALIBRATION[area]['deposition_uv'] = list(uv) if uv else None
    if uv:
        message += " (surface-following: height will track the substrate plane)"
    save_calibration()
    return jsonify({"status": "success", "message": message, "source": source,
                    "ticks": tick, "mm": ticks_to_depth_mm(tick),
                    "warning": message if level == 'warn' else None})


@app.route('/loc/set_z', methods=['POST'])
def loc_set_z():
    """Set the Z (aspiration/dispense depth) of one ink/well, from mm or live position."""
    data = request.json or {}
    kind = data.get('kind')
    items = _loc_list(kind)
    if items is None:
        return jsonify({"status": "error", "message": "Bad kind"}), 400
    try:
        idx = int(data.get('index'))
        if idx < 0 or idx >= len(items):
            return jsonify({"status": "error", "message": "Index out of range"}), 400
        tick, source = _resolve_z(data)
    except (TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": str(e)}), 400
    level, message = classify_z_ticks(tick)
    if level == 'error':
        return jsonify({"status": "error", "message": message}), 400
    items[idx]['pose'][3] = tick
    save_calibration()
    return jsonify({"status": "success", "message": f"{items[idx]['name']}: {message}",
                    "ticks": tick, "mm": ticks_to_depth_mm(tick),
                    "warning": message if level == 'warn' else None})


# ----------------------------- NAMED MAP LIBRARY ----------------------------

@app.route('/map/save', methods=['POST'])
def map_save():
    """Snapshot the active substrate area (corners + deposition Z) under a name."""
    data = request.json or {}
    name = (data.get('name') or '').strip()
    if not name:
        return jsonify({"status": "error", "message": "A map name is required"}), 400
    if not _area_ready('substrate'):
        return jsonify({"status": "error", "message": "Teach all four substrate corners first"}), 400
    sub = CALIBRATION['substrate']
    CALIBRATION['maps'][name] = {
        "corners": {c: list(sub['corners'][c]) for c in CORNERS},
        "deposition_z": sub.get('deposition_z'),
        "width_mm": sub.get('width_mm'),
        "height_mm": sub.get('height_mm'),
    }
    save_calibration()
    return jsonify({"status": "success", "message": f"Saved map '{name}'", "name": name})


@app.route('/map/list', methods=['GET'])
def map_list():
    out = []
    for name, m in CALIBRATION['maps'].items():
        dz = m.get('deposition_z')
        out.append({"name": name,
                    "deposition_mm": ticks_to_depth_mm(dz) if dz is not None else None})
    return jsonify({"status": "success", "maps": out})


@app.route('/map/load', methods=['POST'])
def map_load():
    """Make a saved map the active substrate area so XY operations run on it."""
    data = request.json or {}
    name = data.get('name')
    m = CALIBRATION['maps'].get(name)
    if not m:
        return jsonify({"status": "error", "message": "Map not found"}), 404
    CALIBRATION['substrate']['corners'] = {c: (list(m['corners'][c]) if m['corners'].get(c) else None)
                                           for c in CORNERS}
    CALIBRATION['substrate']['deposition_z'] = m.get('deposition_z')
    CALIBRATION['substrate']['width_mm'] = m.get('width_mm')
    CALIBRATION['substrate']['height_mm'] = m.get('height_mm')
    reproject_targets()   # re-align any targets to this map's corners
    save_calibration()
    return jsonify({"status": "success", "message": f"Loaded map '{name}'",
                    "ready": _area_ready('substrate')})


@app.route('/map/delete', methods=['POST'])
def map_delete():
    data = request.json or {}
    name = data.get('name')
    if name in CALIBRATION['maps']:
        del CALIBRATION['maps'][name]
        save_calibration()
        return jsonify({"status": "success", "message": f"Deleted map '{name}'"})
    return jsonify({"status": "error", "message": "Map not found"}), 404


# ------------------------------- TEST RUN -----------------------------------
# Visits every grid target so the user can confirm reachability and physical
# alignment. Points that fall outside the servo range, below the Safe-Z limit,
# or that fail to converge are flagged instead of stopping the whole run.

test_running = False
test_thread = None
test_report = {"running": False, "current": 0, "total": 0, "results": [], "status": "Idle"}


@app.route('/test_run/start', methods=['POST'])
def test_run_start():
    global test_running, test_thread, test_report
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized - click Init first"}), 400
    # Self-heal a stale flag: if a previous run crashed without clearing the flag,
    # the thread is dead and we should allow a new run rather than block forever.
    if test_running and (test_thread is None or not test_thread.is_alive()):
        print("Test Run: clearing stale test_running flag from a previous run")
        test_running = False
    if test_running:
        return jsonify({"status": "error", "message": "Busy: a Test Run is already running"}), 400
    if sequence_running:
        return jsonify({"status": "error", "message": "Busy: a dispense sequence is running"}), 400

    data = request.json or {}
    area = data.get('area', 'substrate')
    if not _area_ready(area):
        return jsonify({"status": "error",
                        "message": f"The {area} area is not fully calibrated - teach all 4 corners first"}), 400
    targets = list(_targets_for(area))
    if not targets:
        return jsonify({"status": "error",
                        "message": f"No points in the {area} area - generate them first"}), 400

    dep_z = CALIBRATION[area].get('deposition_z')
    try:
        tol = max(1, int(data.get('tolerance', TEST_RUN_TICK_TOLERANCE)))
    except (TypeError, ValueError):
        tol = TEST_RUN_TICK_TOLERANCE
    print(f"Test Run: starting over {len(targets)} targets, area={area}, "
          f"deposition_z={dep_z}, tolerance={tol}")
    test_running = True
    test_report = {"running": True, "current": 0, "total": len(targets),
                   "results": [], "status": "Running", "tolerance": tol}

    def staged_move(target):
        """
        Safe descent to one point, holding the lock for THIS point only and
        bailing out between sub-moves the moment Stop is pressed. Each leg is
        re-issued until it converges, so a long sweep (e.g. the first point from
        home) finishes instead of timing out hundreds of ticks short. Returns the
        actual pose, or None if the run was aborted mid-move.
        """
        s, e, w, z = target
        z = min(z, USER_Z_LIMIT)
        alive = lambda: test_running
        with use_robot():
            robot.enable_torque()
            if settled_move(linear=robot.SAFE_HEIGHT, should_continue=alive) is None:
                return None
            # Backlash-compensated XY approach for repeatable landing.
            if approach_xy(s, e, w, tol=tol, should_continue=alive) is None:
                return None
            if z - 20 > robot.SAFE_HEIGHT:
                if settled_move(linear=z - 20, should_continue=alive) is None:
                    return None
            if settled_move(linear=z, tol=tol, should_continue=alive) is None:
                return None
            time.sleep(0.3)
            return robot.get_servo_positions()[:4]

    def run_loop():
        global test_running, test_report
        try:
            # NOTE: the lock is acquired PER POINT (inside staged_move), not for
            # the whole run, so the stop/status/poll endpoints stay responsive.
            for i, t in enumerate(targets):
                if not test_running:
                    break
                test_report["current"] = i + 1
                target = list(t['pose'])
                if dep_z is not None:
                    target[3] = dep_z
                # CRASH GUARD: a Test Run verifies XY landing accuracy without
                # dispensing, so never descend past the safe approach height.
                cap = AREA_TRAVEL_Z.get(area)
                if cap is not None and target[3] > cap:
                    target[3] = int(cap)
                result = {"name": t.get('name', f"#{i+1}"),
                          "u": t.get('u'), "v": t.get('v')}
                # 1) Range / safety check - never move to an invalid pose.
                labels = ('shoulder', 'elbow', 'wrist', 'linear')
                bad = [f"{labels[j]}={v}" for j, v in enumerate(target)
                       if v < TICK_MIN or v > TICK_MAX]
                if bad:
                    reason = f"Out of servo range (0..4095): {', '.join(bad)}"
                elif target[3] > USER_Z_LIMIT:
                    reason = (f"Z={target[3]} is below the Safe-Z limit ({USER_Z_LIMIT}) - "
                              f"raise the Safe-Z limit or lower the Deposition Z")
                else:
                    reason = None
                if reason:
                    print(f"Test Run: {result['name']} FLAGGED - {reason}")
                    result.update({"ok": False, "reachable": False,
                                   "reason": reason, "target": target})
                    test_report["results"].append(result)
                    continue
                # 2) Move and measure convergence.
                try:
                    print(f"Test Run: moving to {result['name']} -> {target}")
                    actual = staged_move(target)
                    if actual is None:      # Stop pressed mid-move
                        break
                    dev = max(abs(actual[j] - target[j]) for j in range(4))
                    ok = dev <= tol
                    result.update({"ok": ok, "reachable": True, "deviation": dev,
                                   "reason": "" if ok else f"Off target by {dev} ticks",
                                   "target": target, "actual": actual})
                except Exception as e:
                    result.update({"ok": False, "reachable": False,
                                   "reason": str(e), "target": target})
                test_report["results"].append(result)
            # Retreat to a safe height when finished or stopped.
            try:
                with use_robot():
                    robot.move_arm_servos(linear=robot.SAFE_HEIGHT)
            except Exception:
                pass
        except Exception as e:
            test_report["status"] = f"Error: {e}"
        finally:
            test_running = False
            test_report["running"] = False
            failed = [r for r in test_report["results"] if not r.get("ok")]
            done = len(test_report["results"])
            if done < test_report["total"]:
                test_report["status"] = (f"Stopped at {done}/{test_report['total']} "
                                         f"({len(failed)} flagged)")
            else:
                test_report["status"] = (f"Done: {len(failed)} of {test_report['total']} flagged"
                                         if failed else f"Done: all {test_report['total']} points OK")

    test_thread = threading.Thread(target=run_loop, daemon=True)
    test_thread.start()
    return jsonify({"status": "success", "message": f"Test Run started over {len(targets)} points"})


@app.route('/test_run/status', methods=['GET'])
def test_run_status():
    return jsonify({"status": "success", "report": test_report})


@app.route('/test_run/stop', methods=['POST'])
def test_run_stop():
    global test_running
    test_running = False
    return jsonify({"status": "success", "message": "Stopping Test Run"})


# =============================================================================
#  CAMERA-BASED CORRECTION  (arm-mounted camera, hand-eye visual servoing)
#
#  Reuses the proven control from examples/visual_servoing.py: the arm-tip camera
#  looks down, we detect the brightest dot (a fiducial placed at the point being
#  refined), and nudge the shoulder/elbow until that dot sits under the image
#  centre. We then CAPTURE the joint pose - so the taught corner/location becomes
#  camera-accurate, and every interpolated grid point inherits that accuracy.
#
#  Control constants are taken from the working prototype (camera mounted rotated
#  90 deg, hence the err_y->X / err_x->Y mapping).
# =============================================================================

VISION_KP = 0.03            # mm of arm move per pixel of error (from prototype)
VISION_MAX_STEP_MM = 2.0    # clip per-frame move so it can't lunge
VISION_DEADBAND_PX = 6      # "centred" once the dot is within this many pixels
VISION_TIMEOUT_S = 30
VISION_MIN_AREA = 50        # ignore dust specks smaller than this

vision_running = False


def get_brightest_dot(frame):
    """Centre (cx, cy) of the largest bright blob, plus the threshold image."""
    import cv2
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, thresh
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < VISION_MIN_AREA:
        return None, thresh
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None, thresh
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])), thresh


def _open_arm_camera():
    """
    Open ONLY the configured arm-camera index. We deliberately do NOT scan other
    indices - on a two-camera rig that is exactly how the wrong (volume) camera
    gets grabbed. If the configured index is wrong, use the camera picker to find
    and save the right one.
    """
    import cv2
    cap = cv2.VideoCapture(ARM_CAMERA_INDEX, cv2.CAP_DSHOW)
    if cap.isOpened():
        return cap, ARM_CAMERA_INDEX
    cap.release()
    return None, None


def camera_center(show=True, deadband=VISION_DEADBAND_PX, timeout=VISION_TIMEOUT_S):
    """
    Servo the shoulder/elbow until the brightest dot is centred under the camera.
    Caller holds the robot lock. Returns a dict with success / residual_px / pose
    / captured tick pose. Reuses the prototype's proportional control verbatim.
    """
    import cv2
    global vision_running
    cap, idx = _open_arm_camera()
    if cap is None:
        raise RuntimeError("Arm camera not found - set 'arm_camera_index' in settings.json")

    # Start from the arm's current XY (via forward kinematics) and nudge from there.
    pos = robot.get_servo_positions()
    cur_x, cur_y = forward_kinematics(pos[0], pos[1])
    start = time.time()
    residual = None
    win = "Arm Camera - Calibration"
    can_show = show
    try:
        while time.time() - start < timeout:
            if not vision_running:
                break
            ret, frame = cap.read()
            if not ret:
                continue
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2
            dot, thresh = get_brightest_dot(frame)
            if can_show:
                try:
                    cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                    if dot:
                        cv2.circle(frame, dot, 8, (0, 255, 0), -1)
                        cv2.line(frame, (cx, cy), dot, (255, 0, 0), 2)
                    cv2.imshow(win, frame)
                    cv2.waitKey(1)
                except Exception:
                    can_show = False   # headless fallback if no display surface
            if not dot:
                continue
            err_x, err_y = dot[0] - cx, dot[1] - cy
            if math.hypot(err_x, err_y) <= deadband:
                residual = round(math.hypot(err_x, err_y), 1)
                break
            # Camera is mounted rotated 90 deg: x-pixel->physical Y, y-pixel->physical X.
            dx = max(min(-err_y * VISION_KP, VISION_MAX_STEP_MM), -VISION_MAX_STEP_MM)
            dy = max(min(err_x * VISION_KP, VISION_MAX_STEP_MM), -VISION_MAX_STEP_MM)
            try:
                st, et = compute_kinematics(cur_x + dx, cur_y + dy)  # raw IK for a local nudge
                robot.shoulder.set_goal_position(int(st))
                robot.elbow.set_goal_position(int(et))
                cur_x, cur_y = cur_x + dx, cur_y + dy
            except ValueError:
                pass
            time.sleep(0.05)
        time.sleep(0.3)                       # let the last nudge settle
        pose = _capture_pose()
        return {"success": residual is not None, "residual_px": residual,
                "camera_index": idx, "pose": pose}
    finally:
        cap.release()
        if show:
            try:
                cv2.destroyWindow(win)
            except Exception:
                pass


@app.route('/vision/center', methods=['POST'])
def vision_center_endpoint():
    """Centre the arm on the fiducial and report the residual + captured pose."""
    global vision_running
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running or test_running or vision_running:
        return jsonify({"status": "error", "message": "Busy: another routine is running"}), 400
    data = request.json or {}
    show = bool(data.get('show', True))
    vision_running = True
    try:
        with use_robot():
            robot.enable_torque()
            result = camera_center(show=show)
        msg = (f"Centred (residual {result['residual_px']} px)" if result["success"]
               else "Could not centre within timeout - is the dot visible?")
        return jsonify({"status": "success", "message": msg, **result})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    finally:
        vision_running = False


@app.route('/vision/refine_corner', methods=['POST'])
def vision_refine_corner():
    """Camera-centre on a corner's fiducial, then store that camera-accurate pose."""
    global vision_running
    if not robot:
        return jsonify({"status": "error", "message": "Robot not initialized"}), 400
    if sequence_running or test_running or vision_running:
        return jsonify({"status": "error", "message": "Busy: another routine is running"}), 400
    data = request.json or {}
    area = data.get('area', 'substrate')
    corner = data.get('corner')
    if area not in ('substrate', 'reagent'):
        return jsonify({"status": "error", "message": "Unknown area"}), 400
    if corner not in CORNERS:
        return jsonify({"status": "error", "message": "Corner must be TL, TR, BR or BL"}), 400
    vision_running = True
    try:
        with use_robot():
            robot.enable_torque()
            result = camera_center(show=bool(data.get('show', True)))
        if not result["success"]:
            return jsonify({"status": "error",
                            "message": "Could not centre on the fiducial - check lighting/dot"}), 400
        CALIBRATION[area]['corners'][corner] = result["pose"]
        moved = reproject_targets()
        save_calibration()
        msg = f"Refined {area} corner {corner} by camera (residual {result['residual_px']} px)"
        if moved:
            msg += f", re-aligned {moved} target(s)"
        return jsonify({"status": "success", "message": msg,
                        "pose": result["pose"], "ready": _area_ready(area)})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500
    finally:
        vision_running = False


@app.route('/vision/stop', methods=['POST'])
def vision_stop():
    global vision_running
    vision_running = False
    return jsonify({"status": "success", "message": "Stopping camera correction"})


@app.route('/vision/cameras', methods=['GET'])
def vision_cameras():
    """Probe indices 0..5 and report which open, so the user can find the arm cam."""
    try:
        import cv2
    except Exception:
        return jsonify({"status": "error", "message": "OpenCV (cv2) not available"}), 500
    found = []
    for i in range(6):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            found.append(i)
        cap.release()
    return jsonify({"status": "success", "cameras": found, "current": ARM_CAMERA_INDEX})


@app.route('/vision/snapshot', methods=['POST'])
def vision_snapshot():
    """
    Grab ONE frame from a given camera index and return it as a base64 JPEG, so
    the user can identify which physical camera each index is - reliably, without
    depending on an OpenCV window opening in a worker thread.
    """
    try:
        import cv2, base64
    except Exception:
        return jsonify({"status": "error", "message": "OpenCV (cv2) not available"}), 500
    data = request.json or {}
    try:
        index = int(data.get('index', ARM_CAMERA_INDEX))
    except (TypeError, ValueError):
        return jsonify({"status": "error", "message": "index must be an integer"}), 400
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        return jsonify({"status": "error", "message": f"Camera index {index} did not open"}), 400
    frame = None
    for _ in range(8):                 # let the sensor warm up
        ok, f = cap.read()
        if ok:
            frame = f
            break
        time.sleep(0.05)
    cap.release()
    if frame is None:
        return jsonify({"status": "error", "message": f"Index {index} opened but gave no frame"}), 500
    ok, buf = cv2.imencode('.jpg', frame)
    if not ok:
        return jsonify({"status": "error", "message": "Could not encode frame"}), 500
    b64 = base64.b64encode(buf.tobytes()).decode('ascii')
    return jsonify({"status": "success", "index": index,
                    "image": "data:image/jpeg;base64," + b64})


@app.route('/vision/set_camera', methods=['POST'])
def vision_set_camera():
    """Persist the chosen arm-camera index to settings.json."""
    global ARM_CAMERA_INDEX
    data = request.json or {}
    try:
        ARM_CAMERA_INDEX = int(data['index'])
    except (KeyError, TypeError, ValueError):
        return jsonify({"status": "error", "message": "index must be an integer"}), 400
    save_settings()
    return jsonify({"status": "success",
                    "message": f"Arm camera set to index {ARM_CAMERA_INDEX} (saved)"})


if __name__ == '__main__':
    # threaded=True so control endpoints (e.g. /test_run/stop, /stop_sequence)
    # are still served while another request is blocked waiting on the robot lock.
    # Without this the single-threaded dev server makes Stop appear to do nothing.
    app.run(port=5000, threaded=True)