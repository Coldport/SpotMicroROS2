import tkinter as tk
from tkinter import ttk
import json
import os
import time
import numpy as np  # NEW: needed for IK math
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# =========================
# Hardware Setup
# =========================
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50

# =========================
# Model / State
# =========================
LEGS = ["Front Right", "Front Left", "Back Right", "Back Left"]
JOINTS = ["Hip", "Thigh", "Knee"]
LEG_POSITIONS_2x2 = {
    "Front Right": (0, 0),
    "Front Left": (0, 1),
    "Back Right": (1, 0),
    "Back Left": (1, 1),
}

# Mapping to your IK leg names and from_180 rule
IK_LEG_NAME = {
    "Front Right": "FR",
    "Front Left":  "FL",
    "Back Right":  "RR",
    "Back Left":   "RL",
}
IK_RIGHT_LEG = {"FR", "RR"}  # your from_180=True

CONFIG_FILE = "servo_calibration.txt"

# Cache of servo objects by (channel, actuation_range)
servo_objects = {}

def create_servo(channel, actuation_range):
    key = (channel, actuation_range)
    if key not in servo_objects:
        servo_objects[key] = servo.Servo(
            pca.channels[channel],
            min_pulse=500,
            max_pulse=2500,
            actuation_range=actuation_range
        )
    return servo_objects[key]

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"Failed to load config: {e}")
    return {}

def save_config(data):
    with open(CONFIG_FILE, "w") as f:
        json.dump(data, f, indent=4)

# =========================
# Your IK (exact math you posted)
# =========================
def ik_rotational_preprocess_xyz(leg_name, X, Y, Z, roll, pitch, yaw, angles_in_degrees=True):
    """
    Your original body -> hip pre-processor (kept as-is).
    Uses simple offsets tied to roll/pitch/yaw and leg side.
    """
    if angles_in_degrees:
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)
    else:
        roll_rad, pitch_rad, yaw_rad = roll, pitch, yaw

    # Your constants (as in your Leg.HIP_OFFSETS usage): 93 and 40 and 43.5
    z_off = 93 * np.sin(pitch_rad)
    if leg_name in ["FL", "FR"]:
        new_Z1 = Z + z_off
    else:
        new_Z1 = Z - z_off

    y_off = new_Z1 * np.sin(roll_rad)
    if leg_name in ["FL", "RL"]:
        new_Y2 = Y + y_off
    else:
        new_Y2 = Y - y_off

    Xoff = 43.5 * np.sin(yaw_rad)
    if leg_name in ["FL", "FR"]:
        new_X = X + Xoff
    else:
        new_X = X - Xoff

    return float(new_X), float(new_Y2), float(new_Z1)

def ik_inverse_kinematics(leg_name, X, Y, Z):
    """
    Your original IK (kept as-is), returns (hip, lower, upper) in radians.
    """
    # Adjust Y by adding the 40mm hip offset
    Y_adjusted = Y + 40.0

    # Intermediate distances
    D = np.sqrt(Y_adjusted**2 + Z**2 - 1600.0)  # 40^2 = 1600
    G = np.sqrt(D**2 + X**2)

    # Joint angles
    hip_angle = np.arctan2(Z, Y_adjusted) + np.arctan2(40.0, D)
    lower_angle = np.arccos((G**2 - 28800.0) / (-28800.0))  # 2 * 120^2 = 28800
    upper_angle = np.arctan2(D, X) - np.arcsin((120.0 * np.sin(lower_angle)) / G)

    # Apply angle conversion for right legs
    if leg_name in IK_RIGHT_LEG:
        lower_angle = np.pi - lower_angle
        upper_angle = np.pi - upper_angle

    return hip_angle, lower_angle, upper_angle

def ik_create_points(start, end, steps):
    """
    Your linear 6D pose interpolation (kept as-is).
    start/end: [x,y,z,roll,pitch,yaw] (degrees for angles)
    """
    if steps < 1:
        raise ValueError("steps must be >= 1")
    if len(start) != 6 or len(end) != 6:
        raise ValueError("start/end must be [x,y,z,roll,pitch,yaw]")

    x1, y1, z1, r1, p1, ya1 = start
    x2, y2, z2, r2, p2, ya2 = end
    pts = []
    for i in range(steps + 1):
        t = i / steps
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        z = z1 + t * (z2 - z1)
        r = r1 + t * (r2 - r1)
        p = p1 + t * (p2 - p1)
        ya = ya1 + t * (ya2 - ya1)
        pts.append([x, y, z, r, p, ya])
    return pts

# =========================
# GUI App
# =========================
class ServoCalApp:
    def __init__(self, root):
        self.root = root
        root.title("Spot Micro Servo Calibration + IK")
        root.geometry("1150x780")

        self.status_var = tk.StringVar(value="Ready")
        self.config_data = load_config()

        # Data structure:
        self.manual_vars = {leg: {} for leg in LEGS}
        self.cycle_vars = {leg: {} for leg in LEGS}

        # Per-joint runtime cycle state
        self.cycle_state = {leg: {} for leg in LEGS}

        # Global cycle clock and universal speed (deg/s)
        self.last_cycle_ts = time.time()
        self.universal_speed = tk.DoubleVar(value=float(
            self._get_any("speed", default=30.0)
        ))

        # IK state
        self.tab_ik_vars = {leg: {} for leg in LEGS}
        self.ik_running = {leg: False for leg in LEGS}
        self.ik_waypoints = {leg: [] for leg in LEGS}
        self.ik_step_index = {leg: 0 for leg in LEGS}
        self.ik_direction = {leg: +1 for leg in LEGS}
        # Current logical angles (deg) tracked for speed limiting
        self.ik_current_deg = {
            leg: {"Hip": 90.0, "Thigh": 90.0, "Knee": 90.0} for leg in LEGS
        }
        # Targets (deg) for the current waypoint
        self.ik_target_deg = {
            leg: {"Hip": 90.0, "Thigh": 90.0, "Knee": 90.0} for leg in LEGS
        }

        # Tabs
        self.notebook = ttk.Notebook(root)
        self.tab_manual = ttk.Frame(self.notebook)
        self.tab_cycle = ttk.Frame(self.notebook)
        self.tab_ik = ttk.Frame(self.notebook)  # NEW
        self.notebook.add(self.tab_manual, text="Manual")
        self.notebook.add(self.tab_cycle, text="Cycle")
        self.notebook.add(self.tab_ik, text="IK Cycle")  # NEW
        self.notebook.pack(fill="both", expand=True)

        self._build_manual_tab()
        self._build_cycle_tab()
        self._build_ik_tab()  # NEW

        # Bottom bar
        bottom = ttk.Frame(root)
        bottom.pack(fill="x", side="bottom")
        ttk.Button(bottom, text="Save Config", command=self.on_save).pack(side="right", padx=8, pady=6)
        ttk.Label(bottom, textvariable=self.status_var, relief="sunken", anchor="w").pack(side="left", fill="x", expand=True)

        # Schedule updates (frame-synced)
        self._schedule_tick()

        # Close handler
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- helpers ----------
    def _get_any(self, key, default=None):
        for leg in LEGS:
            for joint in JOINTS:
                v = self.config_data.get(leg, {}).get(joint, {}).get(key, None)
                if v is not None:
                    try:
                        return float(v)
                    except:
                        return v
        return default

    # ---------- Manual Tab ----------
    def _build_manual_tab(self):
        outer = ttk.Frame(self.tab_manual)
        outer.pack(fill="both", expand=True)

        top = ttk.Frame(outer)
        top.pack(fill="x", side="top", padx=6, pady=6)
        ttk.Button(top, text="Set→Start (All)", command=self.record_all_to_cycle_start).pack(side="left", padx=4)
        ttk.Button(top, text="Set→End (All)", command=self.record_all_to_cycle_end).pack(side="left", padx=4)

        container = ttk.Frame(outer)
        container.pack(fill="both", expand=True)
        for r in range(2):
            container.grid_rowconfigure(r, weight=1)
        for c in range(2):
            container.grid_columnconfigure(c, weight=1)

        for leg, (row, col) in LEG_POSITIONS_2x2.items():
            frame = ttk.LabelFrame(container, text=leg, padding=10)
            frame.grid(row=row, column=col, sticky="nsew", padx=6, pady=6)

            hdr = ttk.Frame(frame)
            hdr.pack(fill="x")
            ttk.Label(hdr, text="Joint", width=10).pack(side="left")
            ttk.Label(hdr, text="Channel").pack(side="left", padx=6)
            ttk.Label(hdr, text="Range").pack(side="left", padx=6)
            ttk.Label(hdr, text="Offset (°)").pack(side="left", padx=6)
            ttk.Label(hdr, text="Angle (°)").pack(side="left", padx=6)
            ttk.Label(hdr, text="Slider / Actions").pack(side="left", padx=6)

            for joint in JOINTS:
                rowf = ttk.Frame(frame)
                rowf.pack(fill="x", pady=4)

                ttk.Label(rowf, text=joint, width=10).pack(side="left")

                prev = self.config_data.get(leg, {}).get(joint, {})
                ch_val = str(prev.get("channel", "0"))
                range_val = str(prev.get("range", "270"))
                angle_val = float(prev.get("angle", 90.0))
                offset_val = float(prev.get("offset", 0.0))

                ch_var = tk.StringVar(value=ch_val)
                range_var = tk.StringVar(value=range_val)
                offset_var = tk.StringVar(value=str(offset_val))
                angle_var = tk.DoubleVar(value=angle_val)

                ttk.Entry(rowf, textvariable=ch_var, width=4).pack(side="left")
                ttk.Combobox(rowf, textvariable=range_var, values=["180", "270"], width=5).pack(side="left", padx=6)
                ttk.Entry(rowf, textvariable=offset_var, width=6).pack(side="left", padx=6)

                slider = ttk.Scale(
                    rowf, from_=0.0, to=270.0, orient="horizontal", length=150,
                    command=lambda _=None, l=leg, j=joint: self._on_slider(l, j)
                )
                slider.pack(side="left", padx=4)

                angle_entry = ttk.Entry(rowf, width=7)
                angle_entry.pack(side="left", padx=4)

                btns = ttk.Frame(rowf)
                btns.pack(side="left", padx=4)
                ttk.Button(btns, text="Move",
                           command=lambda l=leg, j=joint: self._do_manual_move(l, j)
                           ).pack(fill="x", pady=1)
                ttk.Button(btns, text="Set→Start",
                           command=lambda l=leg, j=joint: self._record_to_cycle_from_manual(l, j, target="start")
                           ).pack(fill="x", pady=1)
                ttk.Button(btns, text="Set→End",
                           command=lambda l=leg, j=joint: self._record_to_cycle_from_manual(l, j, target="end")
                           ).pack(fill="x", pady=1)

                def entry_commit(event, l=leg, j=joint):
                    try:
                        a = float(self.manual_vars[l][j]["angle_var"].get())
                        self._set_slider_from_angle(l, j, a)
                        self._do_manual_move(l, j)
                    except Exception as e:
                        self.status_var.set(f"Error: {e}")

                angle_entry.bind("<Return>", entry_commit)

                def on_range_change(*_ignored, l=leg, j=joint):
                    try:
                        r = int(self.manual_vars[l][j]["range_var"].get())
                        self.manual_vars[l][j]["slider"].configure(to=float(r))
                        a = float(self.manual_vars[l][j]["angle_var"].get())
                        if a > r:
                            self.manual_vars[l][j]["angle_var"].set(float(r))
                            self._set_slider_from_angle(l, j, float(r))
                    except:
                        pass

                self.manual_vars[leg][joint] = {
                    "ch_var": ch_var,
                    "range_var": range_var,
                    "offset_var": offset_var,
                    "angle_var": angle_var,
                    "slider": slider,
                    "angle_entry": angle_entry,
                }

                angle_entry.insert(0, f"{angle_val:.2f}")
                slider.set(angle_val)
                range_var.trace_add("write", on_range_change)
                on_range_change()

    def _on_slider(self, leg, joint):
        s = self.manual_vars[leg][joint]["slider"].get()
        self.manual_vars[leg][joint]["angle_var"].set(float(s))
        self.manual_vars[leg][joint]["angle_entry"].delete(0, tk.END)
        self.manual_vars[leg][joint]["angle_entry"].insert(0, f"{s:.2f}")
        self._do_manual_move(leg, joint, from_slider=True)

    def _set_slider_from_angle(self, leg, joint, angle):
        slider = self.manual_vars[leg][joint]["slider"]
        slider.set(float(angle))
        self.manual_vars[leg][joint]["angle_entry"].delete(0, tk.END)
        self.manual_vars[leg][joint]["angle_entry"].insert(0, f"{float(angle):.2f}")

    def _do_manual_move(self, leg, joint, from_slider=False):
        try:
            ch = int(self.manual_vars[leg][joint]["ch_var"].get())
            act_range = int(self.manual_vars[leg][joint]["range_var"].get())
            offset = float(self.manual_vars[leg][joint]["offset_var"].get())
            angle = float(self.manual_vars[leg][joint]["angle_var"].get())
            actual = clamp(angle + offset, 0.0, float(act_range))
            s = create_servo(ch, act_range)
            s.angle = actual
            src = "slider" if from_slider else "manual"
            self.status_var.set(f"[{src}] {leg}-{joint} → {angle:.2f}+{offset:.2f}={actual:.2f}° (ch {ch}, range {act_range})")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

    def _record_to_cycle_from_manual(self, leg, joint, target="start"):
        try:
            angle = float(self.manual_vars[leg][joint]["angle_var"].get())
            if target == "start":
                self.cycle_vars[leg][joint]["start_var"].set(angle)
                self.status_var.set(f"Recorded Manual {angle:.2f}° → Cycle Start for {leg}-{joint}")
            else:
                self.cycle_vars[leg][joint]["end_var"].set(angle)
                self.status_var.set(f"Recorded Manual {angle:.2f}° → Cycle End for {leg}-{joint}")
        except Exception as e:
            self.status_var.set(f"Record error {leg}-{joint}: {e}")

    def record_all_to_cycle_start(self):
        try:
            for leg in LEGS:
                for joint in JOINTS:
                    angle = float(self.manual_vars[leg][joint]["angle_var"].get())
                    self.cycle_vars[leg][joint]["start_var"].set(angle)
            self.status_var.set("Recorded Manual angles → Cycle Start for ALL joints")
        except Exception as e:
            self.status_var.set(f"Record-all Start error: {e}")

    def record_all_to_cycle_end(self):
        try:
            for leg in LEGS:
                for joint in JOINTS:
                    angle = float(self.manual_vars[leg][joint]["angle_var"].get())
                    self.cycle_vars[leg][joint]["end_var"].set(angle)
            self.status_var.set("Recorded Manual angles → Cycle End for ALL joints")
        except Exception as e:
            self.status_var.set(f"Record-all End error: {e}")

    # ---------- Cycle Tab ----------
    def _build_cycle_tab(self):
        container = ttk.Frame(self.tab_cycle)
        container.pack(fill="both", expand=True)
        for r in range(2):
            container.grid_rowconfigure(r, weight=1)
        for c in range(2):
            container.grid_columnconfigure(c, weight=1)

        topbar = ttk.Frame(self.tab_cycle)
        topbar.pack(fill="x", side="top", padx=6, pady=6)
        ttk.Label(topbar, text="Universal Speed (°/s):").pack(side="left", padx=4)
        spd_entry = ttk.Entry(topbar, textvariable=self.universal_speed, width=8)
        spd_entry.pack(side="left", padx=4)
        ttk.Button(topbar, text="Apply All (Start)", command=self.apply_all_start).pack(side="left", padx=6)
        ttk.Button(topbar, text="Stop All", command=self.stop_all).pack(side="left", padx=6)

        for leg, (row, col) in LEG_POSITIONS_2x2.items():
            frame = ttk.LabelFrame(container, text=f"{leg} — Cycle", padding=10)
            frame.grid(row=row, column=col, sticky="nsew", padx=6, pady=6)

            hdr = ttk.Frame(frame)
            hdr.pack(fill="x")
            ttk.Label(hdr, text="Joint", width=10).pack(side="left")
            ttk.Label(hdr, text="Ch").pack(side="left", padx=6)
            ttk.Label(hdr, text="Range").pack(side="left", padx=6)
            ttk.Label(hdr, text="Offset").pack(side="left", padx=6)
            ttk.Label(hdr, text="Start°").pack(side="left", padx=6)
            ttk.Label(hdr, text="End°").pack(side="left", padx=6)
            ttk.Label(hdr, text="Start/Stop").pack(side="left", padx=6)

            for joint in JOINTS:
                rowf = ttk.Frame(frame)
                rowf.pack(fill="x", pady=4)

                ttk.Label(rowf, text=joint, width=10).pack(side="left")

                prev = self.config_data.get(leg, {}).get(joint, {})
                ch_val = str(prev.get("channel", "0"))
                range_val = str(prev.get("range", "270"))
                offset_val = float(prev.get("offset", 0.0))

                ch_var = tk.StringVar(value=ch_val)
                range_var = tk.StringVar(value=range_val)
                offset_var = tk.StringVar(value=str(offset_val))
                start_var = tk.DoubleVar(value=float(prev.get("start", 30.0)))
                end_var = tk.DoubleVar(value=float(prev.get("end", 120.0)))

                ttk.Entry(rowf, textvariable=ch_var, width=4).pack(side="left")
                ttk.Combobox(rowf, textvariable=range_var, values=["180", "270"], width=5).pack(side="left", padx=6)
                ttk.Entry(rowf, textvariable=offset_var, width=6).pack(side="left", padx=6)
                ttk.Entry(rowf, textvariable=start_var, width=7).pack(side="left", padx=6)
                ttk.Entry(rowf, textvariable=end_var, width=7).pack(side="left", padx=6)

                ttk.Button(rowf, text="Start",
                           command=lambda l=leg, j=joint: self.start_cycle(l, j)
                           ).pack(side="left", padx=4)
                ttk.Button(rowf, text="Stop",
                           command=lambda l=leg, j=joint: self.stop_cycle(l, j)
                           ).pack(side="left", padx=4)

                self.cycle_vars[leg][joint] = {
                    "ch_var": ch_var,
                    "range_var": range_var,
                    "offset_var": offset_var,
                    "start_var": start_var,
                    "end_var": end_var,
                    "current_angle": float(prev.get("angle", 90.0)),
                }

                self.cycle_state[leg][joint] = {
                    "running": False,
                    "direction": +1,
                }

    # Cycle control
    def start_cycle(self, leg, joint):
        try:
            cyc = self.cycle_vars[leg][joint]
            start_a = float(cyc["start_var"].get())
            end_a = float(cyc["end_var"].get())
            lo, hi = (start_a, end_a) if start_a <= end_a else (end_a, start_a)
            cyc["current_angle"] = clamp(cyc["current_angle"], lo, hi)

            self.cycle_state[leg][joint]["running"] = True
            self.cycle_state[leg][joint]["direction"] = +1 if cyc["current_angle"] <= (lo + hi) / 2 else -1
            self.status_var.set(f"Cycle started: {leg}-{joint}")
        except Exception as e:
            self.status_var.set(f"Error starting cycle {leg}-{joint}: {e}")

    def stop_cycle(self, leg, joint):
        self.cycle_state[leg][joint]["running"] = False
        self.status_var.set(f"Cycle stopped: {leg}-{joint}")

    def apply_all_start(self):
        for leg in LEGS:
            for joint in JOINTS:
                self.start_cycle(leg, joint)

    def stop_all(self):
        for leg in LEGS:
            for joint in JOINTS:
                self.stop_cycle(leg, joint)
        for leg in LEGS:
            self.ik_running[leg] = False

    # -------- IK Tab (NEW) --------
    def _build_ik_tab(self):
        outer = ttk.Frame(self.tab_ik)
        outer.pack(fill="both", expand=True)

        # Top controls: universal speed + run/stop all
        top = ttk.Frame(outer)
        top.pack(fill="x", side="top", padx=6, pady=6)
        ttk.Label(top, text="Universal Speed (°/s):").pack(side="left", padx=4)
        ttk.Entry(top, textvariable=self.universal_speed, width=8).pack(side="left", padx=4)
        ttk.Button(top, text="Run All", command=self.ik_run_all).pack(side="left", padx=6)
        ttk.Button(top, text="Stop All", command=self.stop_all).pack(side="left", padx=6)

        container = ttk.Frame(outer)
        container.pack(fill="both", expand=True)
        for r in range(2):
            container.grid_rowconfigure(r, weight=1)
        for c in range(2):
            container.grid_columnconfigure(c, weight=1)

        for leg, (row, col) in LEG_POSITIONS_2x2.items():
            frame = ttk.LabelFrame(container, text=f"{leg} — IK Cycle", padding=10)
            frame.grid(row=row, column=col, sticky="nsew", padx=6, pady=6)

            # Header
            hdr = ttk.Frame(frame)
            hdr.pack(fill="x")
            ttk.Label(hdr, text="Start [x y z r p y]").pack(side="left", padx=6)
            ttk.Label(hdr, text="End [x y z r p y]").pack(side="left", padx=20)
            ttk.Label(hdr, text="Steps").pack(side="left", padx=20)

            row1 = ttk.Frame(frame)
            row1.pack(fill="x", pady=4)

            # Start pose entries
            sv = [tk.DoubleVar(value=v) for v in [0, 0, 120, 0, 0, 0]]
            for i in range(6):
                ttk.Entry(row1, textvariable=sv[i], width=6).pack(side="left", padx=2)

            # End pose entries
            ev = [tk.DoubleVar(value=v) for v in [0, 0, 90, 0, 0, 0]]
            for i in range(6):
                ttk.Entry(row1, textvariable=ev[i], width=6).pack(side="left", padx=2)

            steps_var = tk.IntVar(value=10)
            ttk.Entry(row1, textvariable=steps_var, width=5).pack(side="left", padx=8)

            row2 = ttk.Frame(frame)
            row2.pack(fill="x", pady=4)
            ttk.Button(row2, text="Start", command=lambda l=leg: self.ik_start_leg(l)).pack(side="left", padx=4)
            ttk.Button(row2, text="Stop", command=lambda l=leg: self.ik_stop_leg(l)).pack(side="left", padx=4)

            self.tab_ik_vars[leg] = {
                "start_vars": sv,   # list of 6 DoubleVar
                "end_vars": ev,     # list of 6 DoubleVar
                "steps_var": steps_var
            }

    def ik_run_all(self):
        for leg in LEGS:
            self.ik_start_leg(leg)

    def ik_start_leg(self, leg):
        try:
            v = self.tab_ik_vars[leg]
            start = [var.get() for var in v["start_vars"]]
            end   = [var.get() for var in v["end_vars"]]
            steps = int(v["steps_var"].get())
            pts = ik_create_points(start, end, steps)

            self.ik_waypoints[leg] = pts
            self.ik_step_index[leg] = 0
            self.ik_direction[leg] = +1
            self.ik_running[leg] = True

            # Initialize current and target from first point
            self._ik_update_targets_from_pose(leg, pts[0])
            self.status_var.set(f"IK started for {leg} with {steps} steps")
        except Exception as e:
            self.status_var.set(f"IK start error for {leg}: {e}")

    def ik_stop_leg(self, leg):
        self.ik_running[leg] = False
        self.status_var.set(f"IK stopped for {leg}")

    # ---- IK tick helpers ----
    def _ik_update_targets_from_pose(self, leg, pose6):
        """Compute IK targets (deg) for this leg from a 6D pose."""
        x, y, z, r, p, ya = pose6
        ikname = IK_LEG_NAME[leg]
        xh, yh, zh = ik_rotational_preprocess_xyz(ikname, x, y, z, r, p, ya, angles_in_degrees=True)
        hip_r, low_r, up_r = ik_inverse_kinematics(ikname, xh, yh, zh)

        # Convert to degrees for output layer
        hip_d = float(np.rad2deg(hip_r))
        thigh_d = float(np.rad2deg(low_r))   # your "lower" -> thigh
        knee_d = float(np.rad2deg(up_r))     # your "upper" -> knee

        self.ik_target_deg[leg]["Hip"] = hip_d
        self.ik_target_deg[leg]["Thigh"] = thigh_d
        self.ik_target_deg[leg]["Knee"] = knee_d

    def _ik_advance_waypoint_if_reached(self, leg, eps=0.5):
        """Advance to next waypoint if all joints are within eps degrees."""
        for j in JOINTS:
            if abs(self.ik_current_deg[leg][j] - self.ik_target_deg[leg][j]) > eps:
                return  # not yet
        # reached: move to next
        idx = self.ik_step_index[leg] + self.ik_direction[leg]
        pts = self.ik_waypoints[leg]
        if idx >= len(pts):
            idx = len(pts) - 2
            self.ik_direction[leg] = -1
        elif idx < 0:
            idx = 1
            self.ik_direction[leg] = +1
        self.ik_step_index[leg] = idx
        self._ik_update_targets_from_pose(leg, pts[idx])

    def _ik_tick_compute_all(self, dt):
        """Move current angles toward IK targets with speed limit (deg/s)."""
        spd = abs(float(self.universal_speed.get()))
        step_deg = spd * dt
        for leg in LEGS:
            if not self.ik_running[leg]:
                continue
            # Update logical angles toward targets
            for j in JOINTS:
                cur = self.ik_current_deg[leg][j]
                tgt = self.ik_target_deg[leg][j]
                if abs(tgt - cur) <= step_deg:
                    self.ik_current_deg[leg][j] = tgt
                else:
                    self.ik_current_deg[leg][j] = cur + step_deg * np.sign(tgt - cur)
            # Waypoint management
            self._ik_advance_waypoint_if_reached(leg)

    def _ik_tick_command_all(self):
        """
        Send servo commands for IK **in three passes**:
        1) all Hips, 2) all Thighs, 3) all Knees.
        Uses the per-leg channel/range/offset from the Manual/Cycle forms.
        """
        # Helper to send a single joint
        def send_joint(leg, joint):
            try:
                # Source config: prefer Manual tab if present, else Cycle tab
                if leg in self.manual_vars and joint in self.manual_vars[leg]:
                    ch = int(self.manual_vars[leg][joint]["ch_var"].get())
                    act_range = int(self.manual_vars[leg][joint]["range_var"].get())
                    offset = float(self.manual_vars[leg][joint]["offset_var"].get())
                else:
                    ch = int(self.cycle_vars[leg][joint]["ch_var"].get())
                    act_range = int(self.cycle_vars[leg][joint]["range_var"].get())
                    offset = float(self.cycle_vars[leg][joint]["offset_var"].get())

                logical = float(self.ik_current_deg[leg][joint])  # deg
                actual = clamp(logical + offset, 0.0, float(act_range))
                s = create_servo(ch, act_range)
                s.angle = actual

                # keep manual slider in sync
                if leg in self.manual_vars and joint in self.manual_vars[leg]:
                    self.manual_vars[leg][joint]["angle_var"].set(logical)
                    self._set_slider_from_angle(leg, joint, logical)
            except Exception as e:
                self.status_var.set(f"IK cmd error {leg}-{joint}: {e}")

        # Pass 1: Hips
        for leg in LEGS:
            if self.ik_running[leg]:
                send_joint(leg, "Hip")
        # Pass 2: Thighs
        for leg in LEGS:
            if self.ik_running[leg]:
                send_joint(leg, "Thigh")
        # Pass 3: Knees
        for leg in LEGS:
            if self.ik_running[leg]:
                send_joint(leg, "Knee")

    # -------- Frame-synced update loop (both Cycle + IK) --------
    def _schedule_tick(self):
        now = time.time()
        dt = max(0.0, now - self.last_cycle_ts)
        self.last_cycle_ts = now

        # Legacy cycle: compute then command (together per frame)
        self._cycle_tick_compute_all(dt)
        self._cycle_tick_command_all()

        # IK: compute toward targets, then command in 3 passes
        self._ik_tick_compute_all(dt)
        self._ik_tick_command_all()

        self.root.after(30, self._schedule_tick)

    def _cycle_tick_compute_all(self, dt):
        spd = abs(float(self.universal_speed.get()))
        for leg in LEGS:
            for joint in JOINTS:
                if not self.cycle_state[leg][joint]["running"]:
                    continue
                vars_j = self.cycle_vars[leg][joint]
                start_a = float(vars_j["start_var"].get())
                end_a = float(vars_j["end_var"].get())
                lo, hi = (start_a, end_a) if start_a <= end_a else (end_a, start_a)
                cur = float(vars_j["current_angle"])
                dirn = self.cycle_state[leg][joint]["direction"]
                step = spd * dt * dirn
                nxt = cur + step
                if nxt > hi:
                    nxt = hi
                    dirn = -1
                elif nxt < lo:
                    nxt = lo
                    dirn = +1
                self.cycle_state[leg][joint]["direction"] = dirn
                vars_j["current_angle"] = nxt

    def _cycle_tick_command_all(self):
        for leg in LEGS:
            for joint in JOINTS:
                if not self.cycle_state[leg][joint]["running"]:
                    continue
                vars_j = self.cycle_vars[leg][joint]
                try:
                    ch = int(vars_j["ch_var"].get())
                    act_range = int(vars_j["range_var"].get())
                    offset = float(vars_j["offset_var"].get())
                    logical = float(vars_j["current_angle"])
                    actual = clamp(logical + offset, 0.0, float(act_range))
                    s = create_servo(ch, act_range)
                    s.angle = actual

                    if leg in self.manual_vars and joint in self.manual_vars[leg]:
                        self.manual_vars[leg][joint]["angle_var"].set(logical)
                        self._set_slider_from_angle(leg, joint, logical)
                except Exception as e:
                    self.status_var.set(f"Cycle command error {leg}-{joint}: {e}")
                    self.cycle_state[leg][joint]["running"] = False

    # ---------- Save / Close ----------
    def on_save(self):
        data = {}
        for leg in LEGS:
            data[leg] = {}
            for joint in JOINTS:
                if leg in self.manual_vars and joint in self.manual_vars[leg]:
                    mv = self.manual_vars[leg][joint]
                    ch = mv["ch_var"].get()
                    angle = mv["angle_var"].get()
                    rng = mv["range_var"].get()
                    off = mv["offset_var"].get()
                else:
                    mv = None
                    ch = self.cycle_vars[leg][joint]["ch_var"].get()
                    angle = f'{self.cycle_vars[leg][joint]["current_angle"]:.2f}'
                    rng = self.cycle_vars[leg][joint]["range_var"].get()
                    off = self.cycle_vars[leg][joint]["offset_var"].get()

                cv = self.cycle_vars[leg][joint]
                start_a = cv["start_var"].get()
                end_a = cv["end_var"].get()

                data[leg][joint] = {
                    "channel": ch,
                    "angle": angle,
                    "range": rng,
                    "offset": off,
                    "start": start_a,
                    "end": end_a,
                    "speed": f"{self.universal_speed.get():.2f}",
                }

        save_config(data)
        self.status_var.set(f"Configuration saved to {CONFIG_FILE}")

    def on_close(self):
        self.stop_all()
        try:
            pca.deinit()
        except Exception:
            pass
        self.root.destroy()

# =========================
# Main Entry
# =========================
if __name__ == "__main__":
    root = tk.Tk()
    app = ServoCalApp(root)
    root.mainloop()
