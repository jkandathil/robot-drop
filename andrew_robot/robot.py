from dynamixel_sdk import PortHandler, PacketHandler
from .servo import Servo, ServoPacketException
from .led import LedController
from .config import AndrewConfig
import time

class AndrewRobot:
    DXL_PROTOCOL_VERSION = 1.0
    DXL_ALL_ID = 254
    # "Move finished" convergence margin, in encoder ticks. The geared servos only
    # settle to ~3-8 ticks of their goal, so a margin of 1 could NEVER be met - every
    # move then ran the full timeout below (seconds) before the code moved on, and
    # since one pose move stacks ~5 of these, that was the source of the big delays.
    # 4 ticks is within real precision. This does NOT lower final accuracy: the servo
    # keeps driving to its exact goal regardless; the margin only decides how long the
    # code WAITS before issuing the next instruction.
    POSITION_ERROR_MARGIN = 4
    # TODO Most of these should be read from config files on the robot rather than being hardcoded
    # They may function incorrectly on the wrong model of robot
    SAFE_HEIGHT = 1600
    GRAB_HEIGHT = 2035
    GRIPPER_CLOSED_LOAD = 250
    GRIPPER_CLOSED_POSITION = 2100
    GRIPPER_OPEN_POSITION = 2531
    THUMB_NEUTRAL_POSITION = 1700
    THUMB_DEPRESS_FIRST_POSITION = 2970
    THUMB_DEPRESS_SECOND_POSITION = 3050
    THUMB_EJECT_POSITION = 1498
    ARM_LED_ID = 1
    BODY_LED_ID = 2

    # Relatively slow default speed
    _max_speed = 60

    def __init__(self, config_path: str, servo_com: str, servo_baud: int, led_com: str) -> None:
        self.config = AndrewConfig(config_path)
        self.port_handler = PortHandler(servo_com)
        # The baudrate passed to setupPort is ignored
        # So we have to set it separately
        self.port_handler.baudrate = servo_baud
        self.port_handler.setupPort(servo_baud)
        self.packet_handler = PacketHandler(self.DXL_PROTOCOL_VERSION)

        self._init_servos()
        self.led = LedController(led_com)

    def _init_servos(self):
        # I increased the Ki for the linear and gripper because they had trouble at low speeds

        # Values taken from AndrewOS logs, tuned for higher repeatability
        self.shoulder = Servo(1, self.port_handler, self.packet_handler)
        self.shoulder.torque_limit = 750
        self.shoulder.set_joint_mode(0, 4095)
        self.shoulder.set_pid(40, 5, 15)
        try: self.shoulder.goal_acceleration = 20
        except ValueError: pass
        self.shoulder.temperature_limit = 75

        self.elbow = Servo(2, self.port_handler, self.packet_handler)
        self.elbow.torque_limit = 750
        self.elbow.set_joint_mode(0, 4095)
        self.elbow.set_pid(40, 5, 15)
        try: self.elbow.goal_acceleration = 20
        except ValueError: pass
        self.elbow.temperature_limit = 75

        self.wrist = Servo(3, self.port_handler, self.packet_handler)
        self.wrist.torque_limit = 750
        self.wrist.set_joint_mode(0, 4095)
        self.wrist.set_pid(20, 0, 0)
        # The wrist swings the long pipette tip; without an acceleration ramp it
        # snapped to full speed instantly - the main visible jerk of the arm.
        try: self.wrist.goal_acceleration = 20
        except ValueError: pass
        self.wrist.temperature_limit = 75

        self.linear = Servo(4, self.port_handler, self.packet_handler)
        self.linear.torque_limit = 1023
        self.linear.set_joint_mode(0, 4095)
        self.linear.set_pid(50, 10, 15)
        try: self.linear.goal_acceleration = 20
        except ValueError: pass
        self.linear.temperature_limit = 90

        self.thumb = Servo(5, self.port_handler, self.packet_handler)
        self.thumb.torque_limit = 1023
        self.thumb.set_joint_mode(0, 4095)
        self.thumb.set_pid(50, 0, 0)
        self.thumb.temperature_limit = 75

        self.gripper = Servo(6, self.port_handler, self.packet_handler)
        self.gripper.torque_limit = 1023
        self.gripper.set_joint_mode(0, 4095)
        self.gripper.set_pid(50, 5, 0)
        self.gripper.temperature_limit = 75

        # Twister seems to follow different rules
        self.twister = Servo(7, self.port_handler, self.packet_handler)
        self.twister.torque_limit = 1023
        self.twister.set_wheel_mode()
        self.twister.set_pid(50, 0, 0)
        self.twister.temperature_limit = 75

        self.servos = [
            self.shoulder,
            self.elbow,
            self.wrist,
            self.linear,
            self.thumb,
            self.gripper,
            self.twister]

        # Reply with no return delay so every read/write round-trip is as fast as the
        # bus allows (the factory default adds ~0.5 ms of latency per transaction).
        for s in self.servos:
            try:
                s.return_delay_time = 0
            except Exception:
                pass

        # Remember each joint's configured torque limit. An overload Alarm Shutdown
        # zeroes the RAM torque limit on Protocol 1.0; we re-apply this value to
        # bring the joint back (see recover_overload / _move_servos_unsafe).
        _torque_limits = {1: 750, 2: 750, 3: 750, 4: 1023, 5: 1023, 6: 1023, 7: 1023}
        for s in self.servos:
            s.configured_torque_limit = _torque_limits.get(s.id, 1023)

        # Make sure the setter gets called
        self.max_speed = self._max_speed
        
    @property
    def max_speed(self):
        return self._max_speed
    
    @max_speed.setter
    def max_speed(self, value):
        self._max_speed = value
        for s in self.servos:
            if s.is_wheel_mode():
                continue
            curr_speed = s.moving_speed
            if curr_speed == 0 or curr_speed > value:
                s.moving_speed = value

    def _bulk_read_positions(self):
        """
        Read every joint's present position in ONE bus transaction via GroupBulkRead
        instead of one round-trip per servo. Returns the list, or None if bulk read
        isn't supported / fails - the caller then falls back to per-servo reads. Once
        it fails it's disabled so we never pay a repeated timeout.
        """
        if getattr(self, '_bulk_disabled', False):
            return None
        reader = getattr(self, '_pos_reader', None)
        try:
            from dynamixel_sdk import GroupBulkRead, COMM_SUCCESS
            if reader is None:
                reader = GroupBulkRead(self.port_handler, self.packet_handler)
                self._pos_params = []
                for s in self.servos:
                    addr = s.control_table.addr_present_position
                    if not reader.addParam(s.id, addr, 2):
                        raise RuntimeError("addParam failed")
                    self._pos_params.append((s.id, addr))
                self._pos_reader = reader
            if reader.txRxPacket() != COMM_SUCCESS:
                raise RuntimeError("bulk txRx failed")
            out = []
            for sid, addr in self._pos_params:
                if not reader.isAvailable(sid, addr, 2):
                    raise RuntimeError("bulk data unavailable")
                out.append(reader.getData(sid, addr, 2))
            return out
        except Exception:
            # Disable and fall back; per-servo reads always work on Protocol 1.0.
            self._bulk_disabled = True
            self._pos_reader = None
            return None

    def get_servo_positions(self):
        bulk = self._bulk_read_positions()
        if bulk is not None:
            return bulk
        return [s.position for s in self.servos]

    def execute_staged_writes(self):
        self.packet_handler.action(self.port_handler, self.DXL_ALL_ID)

    def close_gripper(self):
        self.gripper.set_goal_position(self.GRIPPER_CLOSED_POSITION)
        self.gripper.enable_torque()

        while abs(self.gripper.position - self.GRIPPER_CLOSED_POSITION) > self.POSITION_ERROR_MARGIN:
            if self.gripper.present_load > self.GRIPPER_CLOSED_LOAD:
                break

    def open_gripper(self):
        self.move_servos(gripper=self.GRIPPER_OPEN_POSITION)

    def thumb_depress_first_position(self):
        self.move_servos(thumb=self.THUMB_DEPRESS_FIRST_POSITION)

    def thumb_depress_second_position(self):
        self.move_servos(thumb=self.THUMB_DEPRESS_SECOND_POSITION)

    def thumb_neutral(self):
        self.move_servos(thumb=self.THUMB_NEUTRAL_POSITION)

    def thumb_eject(self):
        self.move_servos(thumb=self.THUMB_EJECT_POSITION)

    def grab_pipette(self, slot_index: int, grab_position=None, grab_height=None):
        """
        Grabs a pipette from the specified slot. Slots are numbered 1-5, with 1 being closest to the robot.

        The factory positions in andrew.xml were taught for Gilson Pipetman bodies.
        grab_position (shoulder, elbow, wrist) and grab_height override them for
        pipettes with a different body geometry (e.g. Rainin LTS), which need the
        gripper to reach a different depth/height inside the holder.
        """
        slot = self.config.pipette_slots[f'slot{slot_index}']
        if grab_position is None:
            grab_position = slot.grab_position
        if grab_height is None:
            grab_height = self.GRAB_HEIGHT
        self.open_gripper()
        self.move_servos_proportional(*slot.start_position, linear=grab_height)
        self.move_servos_proportional(*grab_position)
        self.close_gripper()
        # Pull the pipette out so that the user doesn't need to worry about bumping into the holder
        self.move_servos_proportional(*slot.start_position, linear=self.SAFE_HEIGHT)

    def move_arm_servos(self,
                        shoulder: int=None,
                        elbow: int=None,
                        wrist: int=None,
                        linear: int=None):
        """
        Moves exclusively the servos related to arm movement. Same as move_servos, but clarifies the intent better.

        When two or more XY joints are commanded, their speeds are scaled so they
        all ARRIVE TOGETHER (proportional move). With fixed per-joint speeds the
        nearest joint finished first while the rest kept going, so the tip traced
        a kinked, jerky path instead of a smooth sweep.
        """
        xy_goals = sum(p is not None for p in (shoulder, elbow, wrist))
        if xy_goals >= 2:
            self.move_servos_proportional(shoulder=shoulder, elbow=elbow,
                                          wrist=wrist, linear=linear)
        else:
            self.move_servos(shoulder=shoulder, elbow=elbow, wrist=wrist, linear=linear)
    
    def move_servos_proportional(self,
                                 shoulder: int=None,
                                 elbow: int=None,
                                 wrist: int=None,
                                 linear: int=None,
                                 thumb: int=None,
                                 gripper: int=None):
        """
        Moves servos at speeds proportional to the distance they need to travel, so
        all servos reach their goal position at the same time.
        """
        # Exclude linear, as it needs to move up and down separately to reach a safe height
        goals = [shoulder, elbow, wrist, None, thumb, gripper, None]

        # Speeds to restore after this move
        old_speeds = []

        # Find the servo that will take the longest to move at its set speed
        distances = []
        max_time = 0
        max_servo = None
        for s, p in zip(self.servos, goals):
            if p is None:
                old_speeds.append(None)
                distances.append(None)
                continue

            dist = abs(s.position - p)
            distances.append(dist)
            speed = s.moving_speed
            old_speeds.append(speed)
            if speed == 0 and dist > self.POSITION_ERROR_MARGIN:
                raise Exception(f"Servo {s.id} cannot reach its destination with a speed of 0")

            time = dist / speed
            if time > max_time:
                max_time = time
                max_servo = s
        
        # No movement needed, so just return
        if max_servo is None:
            return
        
        # Calculate the speed for each servo
        for s, dist in zip(self.servos, distances):
            if dist is None:
                continue
            calc_speed = int(dist / max_time)
            # Never 0: on Dynamixel, moving_speed 0 means UNLIMITED speed - even for
            # an already-arrived joint that would let the goal write snap at full power.
            s.moving_speed = max(1, calc_speed)

        self.move_servos(shoulder=shoulder,
                         elbow=elbow,
                         wrist=wrist,
                         linear=linear,
                         thumb=thumb,
                         gripper=gripper)
        
        # Restore the old speeds
        for s, speed in zip(self.servos, old_speeds):
            if speed is None:
                continue

            s.moving_speed = speed

    # Move such that it won't bump into the pipette holder (unless that's the specified goal)
    def move_servos(self,
                    shoulder: int=None,
                    elbow: int=None,
                    wrist: int=None,
                    linear: int=None,
                    thumb: int=None,
                    gripper: int=None):
        """
        Moves the servos to the specified positions. Attempts to avoid hitting the pipette holder by moving to a safe height before going sideways.
        """
        moving_xy = shoulder is not None or elbow is not None or wrist is not None
        # If we're moving the linear and there's a chance it could hit the pipette holder, move it up first
        if linear is not None and moving_xy and self.linear.position > self.SAFE_HEIGHT:
            # move to the higher of the two (lower servo position)
            self.move_servos_unsafe(linear=min(self.SAFE_HEIGHT, linear))

        # We should now be at a height where we can do whatever without hitting the pipette holder
        self.move_servos_unsafe(shoulder, elbow, wrist, thumb=thumb, gripper=gripper)

        self.move_servos_unsafe(linear=linear)

    def move_servos_unsafe(self,
                                shoulder: int=None,
                                elbow: int=None,
                                wrist: int=None,
                                linear: int=None,
                                thumb: int=None,
                                gripper: int=None):
        self._move_servos_unsafe([shoulder, elbow, wrist, linear, thumb, gripper, None])

    def _move_servos_unsafe(self, positions: [int]):
        zipped = list(zip(self.servos, positions))
        
        # Enable torque first so the servos don't ignore the goal position logic
        for s, p in zipped:
            if p is not None:
                s.enable_torque(stage=True)
        self.execute_staged_writes()

        # Now send the target coordinates
        for s, p in zipped:
            if p is not None:
                s.set_goal_position(p, stage=True)
        self.execute_staged_writes()

        done = False
        timeout_start = time.time()
        while not done:
            done = True
            for s, p in zip(self.servos, positions):
                if p is None:
                    continue
                try:
                    arrived = abs(s.position - p) <= self.POSITION_ERROR_MARGIN
                except ServoPacketException as e:
                    # A joint stalled hard enough to trip its overload alarm (the
                    # plunger bottoming out, or the tip pressing the surface). Clear
                    # it and HOLD this joint where it stalled - don't crash the whole
                    # move, and don't keep driving into the stop (which re-overloads).
                    if getattr(e, 'error', 0) & 0x20:
                        s.recover_alarm(getattr(s, 'configured_torque_limit', 1023))
                        try:
                            s.set_goal_position(s.position)
                        except Exception:
                            pass
                        print(f"Warning: servo {s.id} overload - cleared and holding position")
                        continue
                    raise
                # if not there yet, we need to continue
                if not arrived:
                    done = False
                    break
            
            # Safety net for a genuinely blocked/stuck servo. With a realistic margin
            # above this is rarely hit; 2.5 s is ample for any single arm move and
            # avoids a long stall when something does jam.
            if not done and time.time() - timeout_start > 2.5:
                print("Warning: Movement timeout! A servo didn't reach its exact target position.")
                break

            if not done:
                time.sleep(0.05) # Prevent serial buffer flooding!

    def enable_torque(self):
        for s in self.servos:
            s.enable_torque()

    def disable_torque(self):
        for s in self.servos:
            s.disable_torque()

    def recover_overload(self, servo=None):
        """Clear a latched overload shutdown and restore torque so motion can resume.
        Pass a single servo, or none to sweep all. Returns the recovered joint ids."""
        targets = [servo] if servo is not None else list(self.servos)
        recovered = []
        for s in targets:
            try:
                s.recover_alarm(getattr(s, 'configured_torque_limit', 1023))
                recovered.append(s.id)
            except Exception:
                pass
        return recovered

    def led_arm(self, power=255):
        self.led.set_power(self.ARM_LED_ID, power)

    def led_body(self, power=255):
        self.led.set_power(self.BODY_LED_ID, power)