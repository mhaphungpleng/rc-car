#!/usr/bin/env python3
import sys
import threading
import time
import math
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Bool, String, Float32

# พยายาม import Service message (ถ้าไม่มีให้แก้เป็น comment หรือใช้ dummy)
from eva_interfaces.srv import SetLidarTune, SetCCTune, SetPidTune


class RobotDashboard(Node):
    def __init__(self):
        super().__init__('robot_dashboard')

        # ---- State Variables ----
        self.scan_data = None
        self.target_data = {
            'found': False,
            'dist': 0.0,
            'bearing': 0.0,
            'width': 0.0
        }
        self.manual_esc = 1000
        self.manual_s1 = 90
        self.mode = "MANUAL"
        self.emergency_latched = False

        # ---- ROS Pubs ----
        self.pub_mode = self.create_publisher(String, '/control_mode', 10)
        self.pub_emg = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_emg_rst = self.create_publisher(Bool, '/emergency_reset', 10)

        # ---- ROS Subs ----
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/target_found', self.cb_found, 10)
        self.create_subscription(Float32, '/target_distance', self.cb_dist, 10)
        self.create_subscription(Float32, '/target_bearing', self.cb_bearing, 10)
        self.create_subscription(Float32, '/target_width', self.cb_width, 10)
        self.create_subscription(Bool, '/emergency_latched', self.cb_emg_latched, 10)
        self.create_subscription(Int16, '/cmd_esc', self.cb_cmd_esc, 10)
        self.create_subscription(Int16, '/cmd_s1',  self.cb_cmd_s1,  10)


        # ---- Service Client (Lidar Tune) ----
        self.cli_lidar = self.create_client(SetLidarTune, '/apply_lidar_tune')
        self.cli_cc  = self.create_client(SetCCTune, '/apply_cc_tune')
        self.cli_pid = self.create_client(SetPidTune, '/apply_pid_tune')

        # ---- GUI Setup ----
        self.root = tk.Tk()
        self.root.title("EVA Robot Dashboard")
        self.root.geometry("1100x700")
        self._cmd_lock = threading.Lock()
        self.last_cmd_esc = None
        self.last_cmd_s1  = None
        
        # Style
        style = ttk.Style()
        style.theme_use('clam')

        # Layout: Left=Visual, Right=Controls
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_panel = ttk.Frame(main_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right_panel = ttk.Frame(main_frame, width=350)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))

        # --- 1. Visualization (Canvas) ---
        self.canvas_size = 500
        self.scale_zoom = 80.0 # pixels per meter
        self.canvas = tk.Canvas(left_panel, bg="black", width=self.canvas_size, height=self.canvas_size)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Labels for visual status
        self.lbl_status = ttk.Label(left_panel, text="Waiting for scan...", font=("Arial", 12))
        self.lbl_status.pack(pady=5)

        # --- 2. Manual Control ---
        ctrl_frame = ttk.LabelFrame(right_panel, text="Manual Control")
        ctrl_frame.pack(fill=tk.X, pady=5)

        # Mode Buttons
        mode_btn_frame = ttk.Frame(ctrl_frame)
        mode_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(mode_btn_frame, text="MANUAL", command=lambda: self.set_mode("MANUAL")).pack(side=tk.LEFT, expand=True, padx=2)
        ttk.Button(mode_btn_frame, text="AUTO", command=lambda: self.set_mode("AUTO")).pack(side=tk.LEFT, expand=True, padx=2)
        ttk.Button(mode_btn_frame, text="AUTO+", command=lambda: self.set_mode("AUTO+")).pack(side=tk.LEFT, expand=True, padx=2)

        # Emergency
        emg_frame = ttk.Frame(ctrl_frame)
        emg_frame.pack(fill=tk.X, pady=10)
        self.btn_emg = tk.Button(emg_frame, text="EMERGENCY STOP", bg="red", fg="white", font=("Arial", 10, "bold"), command=self.send_emg)
        self.btn_emg.pack(fill=tk.X, padx=5)
        ttk.Button(emg_frame, text="Reset Emergency", command=self.reset_emg).pack(fill=tk.X, padx=5, pady=2)

        # Sliders
        ttk.Label(ctrl_frame, text="Speed (ESC) [1000-2000]").pack(pady=(10,0))
        self.slider_esc = tk.Scale(ctrl_frame, from_=1000, to=2000, orient=tk.HORIZONTAL)
        self.slider_esc.set(1000)
        self.slider_esc.pack(fill=tk.X, padx=5)

        ttk.Label(ctrl_frame, text="Steering (S1) [0-180]").pack(pady=(10,0))
        self.slider_s1  = tk.Scale(ctrl_frame, from_=180, to=0, orient=tk.HORIZONTAL) # Left(180)..Right(0) typically
        self.slider_s1.set(90)
        self.slider_s1.pack(fill=tk.X, padx=5)

        # --- 3. Tuning Lidar ---
        tune_frame = ttk.LabelFrame(right_panel, text="Tuning")
        tune_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        canvas_tune = tk.Canvas(tune_frame)
        scrollbar = ttk.Scrollbar(tune_frame, orient="vertical", command=canvas_tune.yview)
        scroll_frame = ttk.Frame(canvas_tune)

        scroll_frame.bind("<Configure>", lambda e: canvas_tune.configure(scrollregion=canvas_tune.bbox("all")))
        canvas_tune.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas_tune.configure(yscrollcommand=scrollbar.set)

        canvas_tune.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")        
        # Create input fields based on lidar.py params
        self.entries = {}

        r = 0
        ttk.Label(scroll_frame, text="--- Lidar Tuning ---", font=("Arial", 10, "bold")).grid(row=r, column=0, pady=10, sticky="w")
        r += 1
        
        params = [
            ("min_target_width_m", "0.10"),
            ("max_target_width_m", "0.40"),
            ("min_dist_m", "0.50"),
            ("max_dist_m", "2.50"),
            ("cluster_gap_m", "0.18"),
            ("min_cluster_points", "3"),
            ("lost_hold_s", "0.45"),
            ("fov_min_deg", "-35.0"),
            ("fov_max_deg", "35.0"),
            ("ema_alpha", "0.25"),
            ("y_penalty", "0.7"),
            ("assoc_wx", "0.8"),
            ("assoc_wy", "1.2"),
            ("use_intensity", "0"), # 0=False, 1=True
            ("min_intensity", "800.0"),
            ("assoc_enable", "1") # 0=False, 1=True
        ]

        # Scrollable frame for tuning

        for name, default in params:
            ttk.Label(scroll_frame, text=name).grid(row=r, column=0, sticky="w", padx=5)
            ent = ttk.Entry(scroll_frame, width=10)
            ent.insert(0, default)
            ent.grid(row=r, column=1, sticky="e", padx=5)
            self.entries[name] = ent
            r += 1

        ttk.Button(scroll_frame, text="APPLY Lidar", command=self.apply_tuning).grid(row=r, column=0, columnspan=2, pady=5)
        r += 1
        # --- 4. Tuning CC (Cruise Control) ---
        ttk.Label(scroll_frame, text="--- CC Tuning ---", font=("Arial", 10, "bold")).grid(row=r, column=0, pady=10)
        r += 1
        cc_params = [
            ("desired_distance_m", "1.0"),
            ("cc_kp", "5.0"),
            ("cc_kd", "0.2"),
            ("min_cmd", "1000"),
            ("max_cmd", "1100"),
            ("base_cmd_found", "1055"),
            ("fallback_cmd", "1070"),
            ("ramp_up_per_s", "10.0"),
            ("ramp_down_per_s", "80.0")
        ]
        for name, default in cc_params:
            ttk.Label(scroll_frame, text=name).grid(row=r, column=0, sticky="w", padx=5)
            ent = ttk.Entry(scroll_frame, width=10)
            ent.insert(0, default)
            ent.grid(row=r, column=1, sticky="e", padx=5)
            self.entries[name] = ent
            r += 1
        ttk.Button(scroll_frame, text="APPLY CC", command=self.apply_cc_tuning).grid(row=r, column=0, columnspan=2, pady=5)
        r += 1

        # --- 5. Tuning PID (Yaw Hold) ---
        ttk.Label(scroll_frame, text="--- PID Tuning ---", font=("Arial", 10, "bold")).grid(row=r, column=0, pady=10)
        r += 1
        pid_params = [
            ("pid_kp", "0.7"),
            ("pid_kd", "0.15"),
            ("deadband_deg", "2.0"),
            ("s1_center_param", "90"),
            ("s1_min", "72"),
            ("s1_max", "108"),
            ("turn_sign", "-1.0"),
            ("d_lpf_alpha", "0.25"),
            ("yaw_lpf_alpha", "0.2"),
            ("bearing_mode", "servo") # "servo" or "deg"
        ]
        for name, default in pid_params:
            ttk.Label(scroll_frame, text=name).grid(row=r, column=0, sticky="w", padx=5)
            ent = ttk.Entry(scroll_frame, width=10)
            ent.insert(0, default)
            ent.grid(row=r, column=1, sticky="e", padx=5)
            self.entries[name] = ent
            r += 1
        ttk.Button(scroll_frame, text="APPLY PID", command=self.apply_pid_tuning).grid(row=r, column=0, columnspan=2, pady=5)
        r += 1

        # ---- Timer Loop for GUI ----
        self.root.after(50, self.update_gui_loop)

        # Start ROS thread
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

    # ---------- ROS Callbacks ----------
    def ros_spin(self):
        rclpy.spin(self)

    def cb_scan(self, msg):
        self.scan_data = msg

    def cb_found(self, msg):
        self.target_data['found'] = msg.data

    def cb_dist(self, msg):
        self.target_data['dist'] = msg.data

    def cb_bearing(self, msg):
        self.target_data['bearing'] = msg.data

    def cb_width(self, msg):
        self.target_data['width'] = msg.data
    
    def cb_emg_latched(self, msg):
        self.emergency_latched = msg.data
        if self.emergency_latched:
            self.btn_emg.config(bg="darkred", text="STOPPED (LATCHED)")
        else:
            self.btn_emg.config(bg="red", text="EMERGENCY STOP")

    def cb_cmd_esc(self, msg: Int16):
        with self._cmd_lock:
            self.last_cmd_esc = int(msg.data)

    def cb_cmd_s1(self, msg: Int16):
        with self._cmd_lock:
            self.last_cmd_s1 = int(msg.data)


    # ---------- GUI Functions ----------
    def set_mode(self, mode):
        self.mode = mode
        self.pub_mode.publish(String(data=mode))
        self.get_logger().info(f"Set Mode: {mode}")

    def send_emg(self):
        self.pub_emg.publish(Bool(data=True))

    def reset_emg(self):
        self.pub_emg_rst.publish(Bool(data=True))

    def update_gui_loop(self):
        with self._cmd_lock:
            esc = self.last_cmd_esc
            s1  = self.last_cmd_s1

        # อัปเดตเฉพาะถ้ามีค่ามาแล้ว
        if esc is not None:
            # clamp กันหลุดช่วงสไลด์
            esc = max(min(esc, int(self.slider_esc.cget("to"))), int(self.slider_esc.cget("from")))
            self.slider_esc.set(esc)

        if s1 is not None:
            # สำหรับสไลด์ที่ from_=180 to=0 ต้อง clamp ด้วย min/max แบบนี้
            lo = min(int(self.slider_s1.cget("from")), int(self.slider_s1.cget("to")))
            hi = max(int(self.slider_s1.cget("from")), int(self.slider_s1.cget("to")))
            s1 = max(min(s1, hi), lo)
            self.slider_s1.set(s1)

        self.draw_lidar()
        
        # Update text status
        found = self.target_data['found']
        dist = self.target_data['dist']
        bearing_rad = self.target_data['bearing']
        width = self.target_data['width']
        
        status_txt = f"Mode: {self.mode} | Found: {found}"
        if found and not math.isnan(dist):
            status_txt += f" | Dist: {dist:.2f}m | Bear: {math.degrees(bearing_rad):.1f}° | W: {width:.2f}m"
        
        self.lbl_status.config(text=status_txt, foreground="green" if found else "red")

        self.root.after(50, self.update_gui_loop)

    def draw_lidar(self):
        if self.scan_data is None:
            return

        self.canvas.delete("all")
        
        # Robot center
        cx, cy = self.canvas_size // 2, self.canvas_size - 50
        
        # Draw robot
        self.canvas.create_polygon(cx, cy-10, cx-10, cy+10, cx+10, cy+10, fill="gray", outline="white")
        
        # Draw FOV lines (approx from config -35 to 35)
        fov_len = 300
        for deg in [-35, 35]:
            rad = math.radians(deg - 90) # -90 because 0 is right, -90 is up in tkinter? no wait.
            # standard math: 0 right, 90 up. Tkinter: 0 right, y goes down.
            # Robot facing up (y decreases). So 'Front' is -90 deg in Tkinter sense.
            # If lidar 0 is front:
            angle = math.radians(deg - 90)
            x = cx + fov_len * math.cos(angle)
            y = cy + fov_len * math.sin(angle)
            self.canvas.create_line(cx, cy, x, y, dash=(2,2), fill="yellow")

        # Draw Scan Points
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment
        
        # Optimization: Don't draw every single point if too many
        step = 2 
        ranges = self.scan_data.ranges

        for i in range(0, len(ranges), step):
            r = ranges[i]
            if math.isinf(r) or math.isnan(r) or r > 10.0:
                continue
            
            # Lidar Angle (0 is front usually? Check lidar.py)
            # Standard ROS: x front, y left.
            # Lidar frame: angle 0 is usually front.
            theta = angle_min + i * angle_inc
            
            # Convert to Canvas Coordinates
            # Canvas: x right, y down. Robot: x up, y left.
            # screen_x = cx - y_robot * scale
            # screen_y = cy - x_robot * scale
            
            x_robot = r * math.cos(theta)
            y_robot = r * math.sin(theta)
            
            screen_x = cx - (y_robot * self.scale_zoom) # flip y for left/right
            screen_y = cy - (x_robot * self.scale_zoom) # flip x for up
            
            self.canvas.create_oval(screen_x-1, screen_y-1, screen_x+1, screen_y+1, outline="red", fill="red")

        # Draw Target if found
        if self.target_data['found']:
            d = self.target_data['dist']
            b = self.target_data['bearing']
            
            if not math.isnan(d):
                tx_robot = d * math.cos(b)
                ty_robot = d * math.sin(b)
                
                sx = cx - (ty_robot * self.scale_zoom)
                sy = cy - (tx_robot * self.scale_zoom)
                
                # Draw Line to target
                self.canvas.create_line(cx, cy, sx, sy, fill="lime", width=2)
                # Draw Target Box
                self.canvas.create_oval(sx-5, sy-5, sx+5, sy+5, outline="lime", width=2)

    def apply_tuning(self):
        if not self.cli_lidar.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Service /apply_lidar_tune not available")
            return

        req = SetLidarTune.Request()
        try:
            req.min_target_width_m = float(self.entries["min_target_width_m"].get())
            req.max_target_width_m = float(self.entries["max_target_width_m"].get())
            req.min_dist_m = float(self.entries["min_dist_m"].get())
            req.max_dist_m = float(self.entries["max_dist_m"].get())
            req.cluster_gap_m = float(self.entries["cluster_gap_m"].get())
            req.min_cluster_points = int(self.entries["min_cluster_points"].get())
            req.lost_hold_s = float(self.entries["lost_hold_s"].get())
            req.fov_min_deg = float(self.entries["fov_min_deg"].get())
            req.fov_max_deg = float(self.entries["fov_max_deg"].get())
            req.ema_alpha = float(self.entries["ema_alpha"].get())
            req.y_penalty = float(self.entries["y_penalty"].get())
            req.assoc_wx = float(self.entries["assoc_wx"].get())
            req.assoc_wy = float(self.entries["assoc_wy"].get())
            req.use_intensity = bool(int(self.entries["use_intensity"].get()))
            req.min_intensity = float(self.entries["min_intensity"].get())
            req.assoc_enable = bool(int(self.entries["assoc_enable"].get()))

            future = self.cli_lidar.call_async(req)
            future.add_done_callback(self.tune_response)
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")
            return
    def apply_cc_tuning(self):
        if not self.cli_cc.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Service /apply_cc_tune not available")
            return
        req = SetCCTune.Request()
        try:
            req.desired_distance_m = float(self.entries["desired_distance_m"].get())
            req.kp = float(self.entries["cc_kp"].get())
            req.kd = float(self.entries["cc_kd"].get())
            req.min_cmd = int(self.entries["min_cmd"].get())
            req.max_cmd = int(self.entries["max_cmd"].get())
            req.base_cmd_found = int(self.entries["base_cmd_found"].get())
            req.fallback_cmd = int(self.entries["fallback_cmd"].get())
            req.ramp_up_per_s = float(self.entries["ramp_up_per_s"].get())
            req.ramp_down_per_s = float(self.entries["ramp_down_per_s"].get())
            req.found_timeout_s = 0.3 # ค่า default หรือเพิ่ม entry
            req.dist_timeout_s = 0.3

            self.cli_cc.call_async(req).add_done_callback(self.tune_response)
        except ValueError:
            messagebox.showerror("Error", "Invalid CC values")
            return

    def apply_pid_tuning(self):
        if not self.cli_pid.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Service /apply_pid_tune not available")
            return
        req = SetPidTune.Request()
        try:
            req.kp = float(self.entries["pid_kp"].get())
            req.kd = float(self.entries["pid_kd"].get())
            req.deadband_deg = float(self.entries["deadband_deg"].get())
            req.s1_center = int(self.entries["s1_center_param"].get())
            req.s1_min = int(self.entries["s1_min"].get())
            req.s1_max = int(self.entries["s1_max"].get())
            req.timeout_s = 0.4
            req.turn_sign = float(self.entries["turn_sign"].get())
            req.d_lpf_alpha = float(self.entries["d_lpf_alpha"].get())
            req.yaw_lpf_alpha = float(self.entries["yaw_lpf_alpha"].get())
            req.bearing_mode = self.entries["bearing_mode"].get()
            req.bearing_deg_max = 30.0
            req.bearing_timeout_s = 0.25

            self.cli_pid.call_async(req).add_done_callback(self.tune_response)
        except ValueError:
            messagebox.showerror("Error", "Invalid PID values")
            return

    def tune_response(self, future):
        try:
            res = future.result()
            if res.success:
                print(f"Tune Applied: {res.message}")
            else:
                print("Tune Failed")
        except Exception as e:
            print(f"Service Call Failed: {e}")

    def run(self):
        self.root.mainloop()
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    gui = RobotDashboard()
    gui.run()

if __name__ == '__main__':
    main()

