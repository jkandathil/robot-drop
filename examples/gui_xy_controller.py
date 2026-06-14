#!/usr/bin/env python
from context import andrew_robot
import tkinter as tk
import time
import math
import threading
import json
from tkinter import messagebox, filedialog
from andrew_robot import AndrewRobot

# --- ANDREW ROBOT KINEMATIC CONSTANTS ---
L1 = 152.13  # Length from Shoulder to Elbow (mm)
L2 = 151.67  # Length from Elbow to Wrist (mm)
MAX_REACH = L1 + L2
MIN_REACH = abs(L1 - L2)

TICKS_PER_RADIAN = 4096 / (2 * math.pi)
CENTER_TICK = 2048
OFFSET_THETA1 = 0
OFFSET_THETA2 = 0

def inverse_kinematics(x, y, elbow_up=True):
    distance_sq = x**2 + y**2
    if distance_sq > MAX_REACH**2 or distance_sq < MIN_REACH**2:
        raise ValueError(f"Target (X={x:.1f}, Y={y:.1f}) is out of physical reach!")
    cos_theta2 = (distance_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
    theta2 = math.acos(cos_theta2)
    
    if not elbow_up:
        theta2 = -theta2
        
    theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))
    
    ticks_shoulder = int((theta1 - OFFSET_THETA1) * TICKS_PER_RADIAN + CENTER_TICK)
    ticks_elbow = int((theta2 - OFFSET_THETA2) * TICKS_PER_RADIAN + CENTER_TICK)
    
    # SAFETY CLAMP: Prevent the base from rotating backwards and getting stuck in the cables!
    # Forward is 2048. Safe front-facing zone is typically between 1000 and 3100 ticks (approx +/- 90 degrees).
    if ticks_shoulder > 3100 or ticks_shoulder < 1000:
        raise ValueError(f"Shoulder rotation ({ticks_shoulder}) is outside the safe front-facing zone!")
        
    return ticks_shoulder, ticks_elbow

class RobotGUI:
    def __init__(self, master):
        self.master = master
        master.title("Andrew Robot XY Map Controller")
        master.geometry("550x850")
        master.configure(bg="#f0f2f5")
        
        self.show_login()

    def show_login(self):
        self.login_frame = tk.Frame(self.master, bg="#ffffff", padx=40, pady=40, highlightthickness=1, highlightbackground="#cccccc")
        self.login_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        
        tk.Label(self.login_frame, text="Login to Andrew Robot", font=("Segoe UI", 16, "bold"), bg="#ffffff").pack(pady=(0, 20))
        
        # Username
        user_frame = tk.Frame(self.login_frame, bg="#ffffff")
        user_frame.pack(fill=tk.X, pady=5)
        tk.Label(user_frame, text="Username:", font=("Segoe UI", 10), bg="#ffffff").pack(side=tk.LEFT)
        self.user_entry = tk.Entry(user_frame, font=("Segoe UI", 10), relief="flat", highlightthickness=1, highlightbackground="#cccccc")
        self.user_entry.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=(10, 0))
        
        # Password
        pass_frame = tk.Frame(self.login_frame, bg="#ffffff")
        pass_frame.pack(fill=tk.X, pady=5)
        tk.Label(pass_frame, text="Password:", font=("Segoe UI", 10), bg="#ffffff").pack(side=tk.LEFT)
        self.pass_entry = tk.Entry(pass_frame, show="*", font=("Segoe UI", 10), relief="flat", highlightthickness=1, highlightbackground="#cccccc")
        self.pass_entry.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=(10, 0))
        
        # Remember Me
        self.remember_var = tk.BooleanVar()
        self.remember_chk = tk.Checkbutton(self.login_frame, text="Remember Password", variable=self.remember_var, bg="#ffffff", font=("Segoe UI", 9))
        self.remember_chk.pack(anchor=tk.W, pady=5)
        
        # Login button
        self.login_btn = tk.Button(self.login_frame, text="Login", command=self.do_login, font=("Segoe UI", 10, "bold"), bg="#0d6efd", fg="white", relief="flat", pady=5)
        self.login_btn.pack(fill=tk.X, pady=(10, 0))

        self.login_err_lbl = tk.Label(self.login_frame, text="", font=("Segoe UI", 9), fg="red", bg="#ffffff")
        self.login_err_lbl.pack(pady=(5, 0))

        # Try to load saved credentials
        self.load_credentials()
        
    def do_login(self):
        username = self.user_entry.get()
        password = self.pass_entry.get()
        
        if username == "admin" and password == "admin":
            if self.remember_var.get():
                self.save_credentials(username, password)
            else:
                self.clear_credentials()
            
            self.login_frame.destroy()
            self.build_main_ui()
        else:
            self.login_err_lbl.config(text="Invalid credentials (try admin/admin)")

    def save_credentials(self, username, password):
        try:
            with open("login.json", "w") as f:
                json.dump({"username": username, "password": password}, f)
        except Exception:
            pass

    def load_credentials(self):
        try:
            with open("login.json", "r") as f:
                data = json.load(f)
                if "username" in data and "password" in data:
                    self.user_entry.insert(0, data["username"])
                    self.pass_entry.insert(0, data["password"])
                    self.remember_var.set(True)
        except Exception:
            pass
            
    def clear_credentials(self):
        try:
            import os
            if os.path.exists("login.json"):
                os.remove("login.json")
        except Exception:
            pass

    def build_main_ui(self):
        self.canvas_size = 400
        # Set robot base somewhat near the center
        self.cx = self.canvas_size / 2
        self.cy = self.canvas_size / 2
        
        # Scale to map mm to pixels: MAX_REACH is ~304mm.
        # Let's map 350mm to 180 pixels half size.
        self.scale = 180.0 / 350.0 
        
        # Start the target somewhere physically reachable in front of the robot
        self.target_x = 0.0
        self.target_y = 200.0

        # Robot reference
        self.robot = None
        self.busy = False
        
        # UI Elements
        self.info_label = tk.Label(self.master, text="Initializing Robot... Please Wait", font=("Segoe UI", 12, "bold"), bg="#f0f2f5")
        self.info_label.pack(pady=(15, 5))
        
        self.coord_label = tk.Label(self.master, text=f"Target Location: X={self.target_x:.1f}, Y={self.target_y:.1f}", font=("Segoe UI", 14), bg="#f0f2f5")
        self.coord_label.pack(pady=5)
        
        self.canvas = tk.Canvas(self.master, width=self.canvas_size, height=self.canvas_size, bg="#ffffff", highlightthickness=1, highlightbackground="#cccccc")
        self.canvas.pack(pady=10)
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        
        self.draw_workspace()
        
        btn_frame = tk.Frame(self.master, bg="#ffffff", padx=10, pady=10)
        btn_frame.pack(pady=5, fill=tk.X, padx=20)
        
        self.go_btn = tk.Button(btn_frame, text="GO to target", command=self.cmd_go, width=15, font=("Segoe UI", 10, "bold"), bg="#0d6efd", fg="white", relief="flat", pady=5)
        self.go_btn.grid(row=0, column=0, padx=5)
        
        self.home_btn = tk.Button(btn_frame, text="HOME", command=self.cmd_home, width=15, font=("Segoe UI", 10, "bold"), bg="#6c757d", fg="white", relief="flat", pady=5)
        self.home_btn.grid(row=0, column=1, padx=5)
        
        self.park_btn = tk.Button(btn_frame, text="PARK", command=self.cmd_park, width=15, font=("Segoe UI", 10, "bold"), bg="#dc3545", fg="white", relief="flat", pady=5)
        self.park_btn.grid(row=0, column=2, padx=5)
        
        btn_frame.grid_columnconfigure(0, weight=1)
        btn_frame.grid_columnconfigure(1, weight=1)
        btn_frame.grid_columnconfigure(2, weight=1)
        
        # --- NEW: Saved Targeting ---
        save_frame = tk.Frame(self.master, bg="#ffffff", padx=10, pady=10)
        save_frame.pack(pady=5, fill=tk.X, padx=20)
        
        self.ink_pos = None
        self.well_pos = None
        
        self.save_ink_btn = tk.Button(save_frame, text="Save Robot Pos to INK", command=lambda: self.cmd_save_target("ink"), bg="#d1e7dd", font=("Segoe UI", 9), relief="flat", pady=3)
        self.save_ink_btn.grid(row=0, column=0, padx=5, sticky="ew")
        
        self.ink_lbl = tk.Label(save_frame, text="Ink: Not Set", font=("Segoe UI", 9), bg="#ffffff")
        self.ink_lbl.grid(row=1, column=0, pady=(2, 0))
        
        self.save_well_btn = tk.Button(save_frame, text="Save Robot Pos to WELL", command=lambda: self.cmd_save_target("well"), bg="#fff3cd", font=("Segoe UI", 9), relief="flat", pady=3)
        self.save_well_btn.grid(row=0, column=1, padx=5, sticky="ew")
        
        self.well_lbl = tk.Label(save_frame, text="Well: Not Set", font=("Segoe UI", 9), bg="#ffffff")
        self.well_lbl.grid(row=1, column=1, pady=(2, 0))

        self.print_run_btn = tk.Button(save_frame, text="RUN PRINT CYCLE", command=self.cmd_print_cycle, font=("Segoe UI", 10, "bold"), bg="#198754", fg="white", relief="flat", pady=5)
        self.print_run_btn.grid(row=0, column=2, rowspan=2, padx=10, sticky="nsew")
        
        save_frame.grid_columnconfigure(0, weight=1)
        save_frame.grid_columnconfigure(1, weight=1)
        save_frame.grid_columnconfigure(2, weight=1)
        
        # --- NEW: Save/Load Configs ---
        file_frame = tk.Frame(self.master, bg="#ffffff", padx=10, pady=10)
        file_frame.pack(pady=5, fill=tk.X, padx=20)
        
        self.save_cfg_btn = tk.Button(file_frame, text="Save Config (.json)", command=self.cmd_save_cfg, bg="#e2e3e5", font=("Segoe UI", 9), relief="flat", pady=3)
        self.save_cfg_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        self.load_cfg_btn = tk.Button(file_frame, text="Load Config (.json)", command=self.cmd_load_cfg, bg="#e2e3e5", font=("Segoe UI", 9), relief="flat", pady=3)
        self.load_cfg_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # Z-Axis (Linear) Controls
        z_frame = tk.Frame(self.master, bg="#ffffff", padx=10, pady=10)
        z_frame.pack(pady=5, fill=tk.X, padx=20)
        tk.Label(z_frame, text="Z-Axis (Vertical):", font=("Segoe UI", 10, "bold"), bg="#ffffff").pack(side=tk.LEFT, padx=5)
        
        self.z_up_btn = tk.Button(z_frame, text="UP ↑", width=10, bg="#f8f9fa", font=("Segoe UI", 9), relief="flat", pady=3)
        self.z_up_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        self.z_up_btn.bind("<ButtonPress-1>", lambda e: self.z_jog_start("up"))
        self.z_up_btn.bind("<ButtonRelease-1>", lambda e: self.z_jog_stop())
        
        self.z_down_btn = tk.Button(z_frame, text="DOWN ↓", width=10, bg="#f8f9fa", font=("Segoe UI", 9), relief="flat", pady=3)
        self.z_down_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        self.z_down_btn.bind("<ButtonPress-1>", lambda e: self.z_jog_start("down"))
        self.z_down_btn.bind("<ButtonRelease-1>", lambda e: self.z_jog_stop())
        
        self.z_jogging = False
        self.z_jog_thread = None

        # Gripper Controls
        grip_frame = tk.Frame(self.master, bg="#ffffff", padx=10, pady=10)
        grip_frame.pack(pady=5, fill=tk.X, padx=20)
        
        self.open_grip_btn = tk.Button(grip_frame, text="Open Gripper (Insert Pipette)", command=self.cmd_open_gripper, bg="#e2e3e5", font=("Segoe UI", 9), relief="flat", pady=3)
        self.open_grip_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        self.close_grip_btn = tk.Button(grip_frame, text="Close Gripper (Lock Pipette)", command=self.cmd_close_gripper, bg="#e2e3e5", font=("Segoe UI", 9), relief="flat", pady=3)
        self.close_grip_btn.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)
        
        # Start connection in background to prevent freezing UI
        threading.Thread(target=self.init_robot, daemon=True).start()

    def draw_workspace(self):
        self.canvas.delete("all")
        
        # Draw grid
        for i in range(0, self.canvas_size, 50):
            self.canvas.create_line(i, 0, i, self.canvas_size, fill="#f0f0f0")
            self.canvas.create_line(0, i, self.canvas_size, i, fill="#f0f0f0")
            
        # Draw axes
        self.canvas.create_line(self.cx, 0, self.cx, self.canvas_size, fill="#aaa", width=1)
        self.canvas.create_line(0, self.cy, self.canvas_size, self.cy, fill="#aaa", width=1)
        
        # Draw reachable zone (Inner/Outer radius)
        r_out = MAX_REACH * self.scale
        r_in = MIN_REACH * self.scale
        self.canvas.create_oval(self.cx - r_out, self.cy - r_out, self.cx + r_out, self.cy + r_out, outline="#007bff", dash=(4, 4), width=2)
        self.canvas.create_oval(self.cx - r_in, self.cy - r_in, self.cx + r_in, self.cy + r_in, outline="#dc3545", dash=(4, 4), width=1)
        
        # Robot base marker
        self.canvas.create_oval(self.cx - 6, self.cy - 6, self.cx + 6, self.cy + 6, fill="black")
        self.canvas.create_text(self.cx - 10, self.cy - 10, text="Base", anchor="sw")
        
        # Current Target marker
        tx_pix = self.cx + (self.target_x * self.scale)
        ty_pix = self.cy - (self.target_y * self.scale)
        self.canvas.create_oval(tx_pix - 5, ty_pix - 5, tx_pix + 5, ty_pix + 5, fill="#28a745", tags="target")
        self.canvas.create_text(tx_pix + 10, ty_pix, text="Target", anchor="w", tags="target_text", fill="#28a745", font=("Arial", 10, "bold"))

    def init_robot(self):
        try:
            self.robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
            self.robot.max_speed = 40
            self.robot.enable_torque()
            self.info_label.config(text="Robot Ready - Click canvas to move target", fg="green")
        except Exception as e:
            self.info_label.config(text=f"Robot Connection Error: {e}", fg="red")

    def on_canvas_click(self, event):
        if self.busy:
            return
            
        # Convert pixels back to math physical coordinates (mm)
        # Math Y is UP so invert pixel Y
        new_target_x = (event.x - self.cx) / self.scale
        new_target_y = (self.cy - event.y) / self.scale
        
        # Test if the new target is physically valid before accepting it
        robot_x = new_target_y
        robot_y = -new_target_x
        try:
            inverse_kinematics(robot_x, robot_y, elbow_up=True)
            self.target_x = new_target_x
            self.target_y = new_target_y
            self.coord_label.config(text=f"Target Location: X={self.target_x:.1f}, Y={self.target_y:.1f}")
            self.info_label.config(text="Target selected.", fg="green")
            self.draw_workspace()
        except ValueError as ve:
            self.info_label.config(text=f"Cannot select: {ve}", fg="red")

    def run_in_thread(self, func):
        if self.busy:
            return
        if not self.robot:
            messagebox.showwarning("Warning", "Robot is not connected or initialized yet!")
            return
            
        self.busy = True
        self.info_label.config(text="Executing Movement...", fg="#ff8c00")
        self.master.update()
        
        def ui_update(text, fg_color, err_title=None, err_body=None):
            self.info_label.config(text=text, fg=fg_color)
            if err_title:
                # Set specific font for this error locally, wait to show messagebox so label updates first
                self.info_label.config(font=("Arial", 10, "bold"))
                self.master.update()
                messagebox.showerror(err_title, err_body)
            else:
                self.info_label.config(font=("Arial", 12, "bold"))
            self.busy = False

        def wrapper():
            try:
                func()
                self.master.after(0, lambda: ui_update("Movement Complete. Ready for next command.", "green"))
            except ValueError as ve:
                self.master.after(0, lambda: ui_update(f"Invalid Target: {ve}", "red"))
            except Exception as e:
                err_str = str(e)
                if "Overload" in err_str or "overload" in err_str.lower():
                    self.master.after(0, lambda: ui_update(
                        "Hardware Error: OVERLOAD. \nYou MUST physically turn the robot off and back on!", 
                        "red",
                        "Hardware Overload Lock", 
                        "The robot threw an 'Overload' hardware lock! This is a physical firmware safety shutdown.\n\nTo clear this, you MUST switch the robot's physical power switch OFF, wait 5 seconds, and switch it back ON."
                    ))
                else:
                    self.master.after(0, lambda: ui_update(f"Error: {e}", "red"))
                    
        threading.Thread(target=wrapper, daemon=True).start()

    def cmd_go(self):
        def _go():
            # Fix mapping: Visual X corresponds to Robot's Y (Left/Right) 
            # But wait, visually X>0 is right of screen. On robot, Y>0 is Left. So Visual X -> -Robot Y
            # Visually Y>0 is Up. On robot, X>0 is Forward. So Visual Y -> Robot X
            robot_x = self.target_y
            robot_y = -self.target_x
            
            # Calculate IK to verify target is reachable before moving
            s_ticks, e_ticks = inverse_kinematics(robot_x, robot_y, elbow_up=True)
            
            # Ensure safe height on Z-axis first
            self.robot.move_arm_servos(linear=self.robot.SAFE_HEIGHT)
            time.sleep(0.5)
            
            # Move to new X/Y location, keeping the wrist neutral (1000) so it doesn't drag
            self.robot.move_arm_servos(shoulder=s_ticks, elbow=e_ticks, wrist=1000)
            time.sleep(1.0)
            
        self.run_in_thread(_go)

    def cmd_home(self):
        def _home():
            self.robot.move_arm_servos(linear=self.robot.SAFE_HEIGHT)
            time.sleep(0.5)
            self.robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
            time.sleep(1.0)
        self.run_in_thread(_home)

    def cmd_park(self):
        def _park():
            self.robot.move_arm_servos(linear=self.robot.SAFE_HEIGHT)
            time.sleep(0.5)
            # Fold inward
            self.robot.move_arm_servos(shoulder=1100, elbow=1500, wrist=1000)
            time.sleep(1.0)
            # Lower arm to bottom limit
            self.robot.move_arm_servos(linear=3125)
            time.sleep(2.0)
            # Disable torque for safe shutdown
            self.robot.disable_torque()
            self.master.after(0, lambda: self.info_label.config(text="Robot is parked and powered down.", fg="blue"))
        self.run_in_thread(_park)

    def cmd_open_gripper(self):
        def _open():
            self.robot.open_gripper()
        self.run_in_thread(_open)

    def cmd_close_gripper(self):
        def _close():
            self.robot.close_gripper()
        self.run_in_thread(_close)

    def cmd_save_target(self, target_type):
        if not self.robot:
            messagebox.showwarning("Error", "Robot not connected.")
            return
            
        # Read the commanded goal positions rather than present physical positions 
        # to prevent mechanical drift/repeatability issues during saves.
        pos = [s.get_goal_position() for s in self.robot.servos]
        # pos indices: 0=Shoulder, 1=Elbow, 2=Wrist, 3=Linear Z
        
        if target_type == "ink":
            self.ink_pos = pos
            self.ink_lbl.config(text=f"Ink: S={pos[0]}, E={pos[1]}, Z={pos[3]}")
        else:
            self.well_pos = pos
            self.well_lbl.config(text=f"Well: S={pos[0]}, E={pos[1]}, Z={pos[3]}")
            
    def cmd_save_cfg(self):
        if not self.ink_pos and not self.well_pos:
            messagebox.showwarning("Warning", "No endpoints to save! Map Ink and Well first.")
            return
            
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, "w") as f:
                data = {
                    "ink_pos": self.ink_pos,
                    "well_pos": self.well_pos
                }
                json.dump(data, f, indent=4)
            messagebox.showinfo("Success", "Positions successfully saved!")

    def cmd_load_cfg(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if file_path:
            try:
                with open(file_path, "r") as f:
                    data = json.load(f)
                    
                self.ink_pos = data.get("ink_pos")
                self.well_pos = data.get("well_pos")
                
                if self.ink_pos:
                    self.ink_lbl.config(text=f"Ink: S={self.ink_pos[0]}, E={self.ink_pos[1]}, Z={self.ink_pos[3]}")
                if self.well_pos:
                    self.well_lbl.config(text=f"Well: S={self.well_pos[0]}, E={self.well_pos[1]}, Z={self.well_pos[3]}")
                messagebox.showinfo("Loaded", "Ink and Well positions successfully loaded!")
            except Exception as e:
                messagebox.showerror("Load Error", f"Failed to load configuration:\n{e}")

    def cmd_print_cycle(self):
        if not self.ink_pos or not self.well_pos:
            messagebox.showerror("Error", "You must save both INK and WELL locations first!")
            return
            
        def _run_print():
            # 1. Start safely up high
            self.robot.move_arm_servos(linear=self.robot.SAFE_HEIGHT)
            time.sleep(1.0)
            
            # --- ASPIRATE FROM INK ---
            self.master.after(0, lambda: self.info_label.config(text="Moving to INK...", fg="blue"))
            # Move XY over ink
            self.robot.move_arm_servos(shoulder=self.ink_pos[0], elbow=self.ink_pos[1], wrist=self.ink_pos[2])
            time.sleep(1.5)
            
            # Prep pipette (eject air while above the liquid). Custom deep plunge for higher aspiration volume.
            # THUMB_NEUTRAL is 1700, THUMB_DEPRESS_SECOND (max safe bottom) is 3050. Total stroke = 1350 units.
            # 88% of max stroke = ~1188. So 1700 + 1188 = 2888 length.
            self.robot.move_servos(thumb=2888)
            time.sleep(0.5)
            
            # Lower into ink
            self.robot.move_arm_servos(linear=self.ink_pos[3])
            time.sleep(1.0)
            
            # Aspirate (pull liquid in)
            self.robot.thumb_neutral()
            time.sleep(1.5) # Wait for thick ink to fill
            
            # Raise back to safe height
            self.robot.move_arm_servos(linear=self.robot.SAFE_HEIGHT)
            time.sleep(1.0)
            
            # --- DISPENSE IN WELL ---  
            self.master.after(0, lambda: self.info_label.config(text="Moving to WELL...", fg="purple"))
            # Move XY over well
            self.robot.move_arm_servos(shoulder=self.well_pos[0], elbow=self.well_pos[1], wrist=self.well_pos[2])
            time.sleep(1.5)
            
            # Lower into well
            self.robot.move_arm_servos(linear=self.well_pos[3])
            time.sleep(1.0)
            
            # Dispense (push liquid out using the second position to blow out all remaining liquid)
            # DO NOT use thumb_eject() as 1498 is actually higher up than neutral. Max depth is second_position!
            self.robot.thumb_depress_second_position()
            time.sleep(1.5)
            
            # Keep thumb held down while raising to avoid sucking liquid back in!
            self.robot.move_arm_servos(linear=self.robot.SAFE_HEIGHT)
            time.sleep(1.0)
            
            # Neutral thumb now that we are safe in the air
            self.robot.thumb_neutral()
            time.sleep(0.5)
            
        self.run_in_thread(_run_print)

    def z_jog_start(self, direction):
        if not self.robot or self.busy:
            return
        self.z_jogging = True
        
        def jog_loop():
            # Get current position
            pos = self.robot.get_servo_positions()
            current_z = pos[3] # Index 3 is linear z
            # The linear z-axis physical limit is roughly between 200 (Highest) and 3125 (Lowest).
            # Going below 200 may cause the carriage to jam hard against the top motor mount!
            step = 10 if direction == "down" else -10 # positive is down
            
            while self.z_jogging:
                current_z += step
                # Clamp boundaries to prevent jamming at the top
                if current_z < 200:
                    current_z = 200
                if current_z > 3125:
                    current_z = 3125
                    
                try:
                    # Crucially, actually block the background thread dynamically to wait for the servo response
                    # This prevents flooding the serial buffer which leaves the port permanently stuck!
                    self.robot.move_arm_servos(linear=int(current_z))
                except Exception:
                    pass
                # Keep loop relatively slow so serial buffer can fully cycle!
                time.sleep(0.1)
                
        self.z_jog_thread = threading.Thread(target=jog_loop, daemon=True)
        self.z_jog_thread.start()

    def z_jog_stop(self):
        self.z_jogging = False
        # Do not send any more robot commands here from the main thread!
        # The background thread was likely colliding with this main thread 
        # trying to read/write to the serial port simultaneously, crashing it.

if __name__ == '__main__':
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()
