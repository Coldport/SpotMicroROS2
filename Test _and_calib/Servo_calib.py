import tkinter as tk
from tkinter import ttk
import json
import os
import time
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
# GUI App
# =========================
class ServoCalApp:
    def __init__(self, root):
        self.root = root
        root.title("Spot Micro Servo Calibration")
        root.geometry("1150x780")

        self.status_var = tk.StringVar(value="Ready")
        self.config_data = load_config()

        # Data structure:
        self.manual_vars = {leg: {} for leg in LEGS}
        self.cycle_vars = {leg: {} for leg in LEGS}

        # Per-joint runtime cycle state
        # running, direction (+1/-1), current_angle
        self.cycle_state = {leg: {} for leg in LEGS}

        # Global cycle clock and universal speed
        self.last_cycle_ts = time.time()
        self.universal_speed = tk.DoubleVar(value=float(
            self._get_any("speed", default=30.0)
        ))

        # Tabs
        self.notebook = ttk.Notebook(root)
        self.tab_manual = ttk.Frame(self.notebook)
        self.tab_cycle = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_manual, text="Manual")
        self.notebook.add(self.tab_cycle, text="Cycle")
        self.notebook.pack(fill="both", expand=True)

        self._build_manual_tab()
        self._build_cycle_tab()

        # Bottom bar
        bottom = ttk.Frame(root)
        bottom.pack(fill="x", side="bottom")
        ttk.Button(bottom, text="Save Config", command=self.on_save).pack(side="right", padx=8, pady=6)
        ttk.Label(bottom, textvariable=self.status_var, relief="sunken", anchor="w").pack(side="left", fill="x", expand=True)

        # Schedule cycle updates (frame-synced)
        self._schedule_cycle_tick()

        # Close handler
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- helpers ----------
    def _get_any(self, key, default=None):
        # try pull a value from any joint to seed a global default (e.g., speed)
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

        # Top controls row: global record buttons
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

            # Header row
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

                # Live slider (shorter)
                slider = ttk.Scale(
                    rowf, from_=0.0, to=270.0, orient="horizontal", length=150,
                    command=lambda _=None, l=leg, j=joint: self._on_slider(l, j)
                )
                slider.pack(side="left", padx=4)

                # Angle entry (float)
                angle_entry = ttk.Entry(rowf, width=7)
                angle_entry.pack(side="left", padx=4)

                # Action buttons stacked
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

                # Bind angle entry ↔ slider sync
                def entry_commit(event, l=leg, j=joint):
                    try:
                        a = float(self.manual_vars[l][j]["angle_var"].get())
                        self._set_slider_from_angle(l, j, a)
                        self._do_manual_move(l, j)
                    except Exception as e:
                        self.status_var.set(f"Error: {e}")

                angle_entry.bind("<Return>", entry_commit)

                # Adjust slider max when range changes
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

                # Store refs
                self.manual_vars[leg][joint] = {
                    "ch_var": ch_var,
                    "range_var": range_var,
                    "offset_var": offset_var,
                    "angle_var": angle_var,
                    "slider": slider,
                    "angle_entry": angle_entry,
                }

                # Initialize widgets
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

        # Controls row (top): universal speed + apply/stop
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

            # Header row
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
                    "direction": +1,  # +1 towards end, -1 towards start
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

    # -------- Frame-synced cycle tick --------
    def _cycle_tick_compute_all(self, dt):
        """Compute next logical angles for all running joints (no hardware I/O here)."""
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

                # bounce at bounds
                if nxt > hi:
                    nxt = hi
                    dirn = -1
                elif nxt < lo:
                    nxt = lo
                    dirn = +1

                # store updates only (no commands yet)
                self.cycle_state[leg][joint]["direction"] = dirn
                vars_j["current_angle"] = nxt

    def _cycle_tick_command_all(self):
        """Send all servo commands together for this frame."""
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

                    # Keep manual tab in sync visually
                    if leg in self.manual_vars and joint in self.manual_vars[leg]:
                        self.manual_vars[leg][joint]["angle_var"].set(logical)
                        self._set_slider_from_angle(leg, joint, logical)
                except Exception as e:
                    self.status_var.set(f"Cycle command error {leg}-{joint}: {e}")
                    self.cycle_state[leg][joint]["running"] = False

    def _schedule_cycle_tick(self):
        now = time.time()
        dt = max(0.0, now - self.last_cycle_ts)
        self.last_cycle_ts = now

        # First pass: compute all updates
        self._cycle_tick_compute_all(dt)
        # Second pass: command all at once
        self._cycle_tick_command_all()

        self.root.after(30, self._schedule_cycle_tick)

    # ---------- Save / Close ----------
    def on_save(self):
        data = {}
        for leg in LEGS:
            data[leg] = {}
            for joint in JOINTS:
                # manual fields
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

                # cycle params
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
                    "speed": f"{self.universal_speed.get():.2f}",  # persisted once, read as universal
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
