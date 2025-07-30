import tkinter as tk
from tkinter import ttk
import json
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# =========================
# Hardware Setup
# =========================
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50

# Dictionary to hold servo objects with key = (channel, range)
servo_objects = {}

def create_servo(channel, actuation_range):
    """Create or reuse servo object for given channel & range"""
    key = (channel, actuation_range)
    if key not in servo_objects:
        servo_objects[key] = servo.Servo(
            pca.channels[channel],
            min_pulse=500,
            max_pulse=2500,
            actuation_range=actuation_range
        )
    return servo_objects[key]

def update_servo_angle(leg, servo_name, channel_var, angle_var, range_var):
    try:
        channel = int(channel_var.get())
        angle = int(angle_var.get())
        actuation_range = int(range_var.get())
        s = create_servo(channel, actuation_range)
        # Clamp angle within allowed range
        angle = max(0, min(angle, actuation_range))
        s.angle = angle
        status_var.set(f"{leg}-{servo_name} → {angle}° (ch {channel}, range {actuation_range})")
    except Exception as e:
        status_var.set(f"Error: {e}")

def save_config():
    data = {}
    for leg in servo_vars:
        data[leg] = {}
        for servo_name in servo_vars[leg]:
            data[leg][servo_name] = {
                "channel": servo_vars[leg][servo_name]["channel"].get(),
                "angle": servo_vars[leg][servo_name]["angle"].get(),
                "range": servo_vars[leg][servo_name]["range"].get()
            }
    with open("servo_calibration.txt", "w") as f:
        json.dump(data, f, indent=4)
    status_var.set("Configuration saved to servo_calibration.txt")

# =========================
# GUI Setup
# =========================
root = tk.Tk()
root.title("Spot Micro Servo Calibration")
root.geometry("800x600")

status_var = tk.StringVar(value="Ready")

servo_vars = {
    "Front Right": {},
    "Front Left": {},
    "Back Right": {},
    "Back Left": {}
}

leg_positions = {
    "Front Right": (0, 0),
    "Front Left": (0, 1),
    "Back Right": (1, 0),
    "Back Left": (1, 1)
}

main_frame = ttk.Frame(root)
main_frame.pack(fill="both", expand=True)
main_frame.grid_rowconfigure(0, weight=1)
main_frame.grid_rowconfigure(1, weight=1)
main_frame.grid_columnconfigure(0, weight=1)
main_frame.grid_columnconfigure(1, weight=1)

for leg, (row, col) in leg_positions.items():
    frame = ttk.LabelFrame(main_frame, text=leg, padding=10)
    frame.grid(row=row, column=col, sticky="nsew", padx=5, pady=5)

    for joint in ["Hip", "Thigh", "Knee"]:
        row_frame = ttk.Frame(frame)
        row_frame.pack(fill="x", pady=5)

        ttk.Label(row_frame, text=joint).pack(side="left")

        ch_var = tk.StringVar(value="0")
        angle_var = tk.StringVar(value="90")
        range_var = tk.StringVar(value="270")

        ttk.Label(row_frame, text="Ch:").pack(side="left")
        ttk.Entry(row_frame, textvariable=ch_var, width=3).pack(side="left")

        ttk.Label(row_frame, text="Angle:").pack(side="left")
        ttk.Entry(row_frame, textvariable=angle_var, width=5).pack(side="left")

        ttk.Label(row_frame, text="Range:").pack(side="left")
        ttk.Combobox(row_frame, textvariable=range_var, values=["180", "270"], width=4).pack(side="left")

        ttk.Button(row_frame, text="Move",
                   command=lambda l=leg, j=joint, c=ch_var, a=angle_var, r=range_var:
                   update_servo_angle(l, j, c, a, r)).pack(side="left")

        servo_vars[leg][joint] = {"channel": ch_var, "angle": angle_var, "range": range_var}

# Save button & Status bar
ttk.Button(root, text="Save Config", command=save_config).pack(side="bottom", pady=10)
ttk.Label(root, textvariable=status_var, relief="sunken", anchor="w").pack(side="bottom", fill="x")

root.mainloop()
pca.deinit()
