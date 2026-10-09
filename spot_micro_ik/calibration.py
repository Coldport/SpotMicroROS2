"""
Loads config/servo_calibration.yaml into typed, in-memory calibration data.

This is the YAML counterpart of the older servo_calibration.txt (JSON) that
Servo_calib.py, the manual jogging GUI, still owns. The two files are NOT kept
in sync automatically — see the comment at the top of the .yaml.
"""

from dataclasses import dataclass
from pathlib import Path

import yaml

# IK leg name ("FL"/"FR"/"RL"/"RR") -> calibration YAML top-level key.
# "Rear" in the IK/leg-naming code and "Back" in the calibration file both
# refer to the same two legs; this is the one place that mapping is spelled out.
LEG_TO_YAML_KEY = {
    "FL": "Front Left",
    "FR": "Front Right",
    "RL": "Back Left",
    "RR": "Back Right",
}


@dataclass(frozen=True)
class JointCal:
    channel: int
    angle: float     # recorded neutral angle (deg) — informational, not used by IK
    range: float      # servo actuation range (deg): 180 or 270
    offset: float     # deg added to the commanded angle before clamping to [0, range]


@dataclass(frozen=True)
class LegCal:
    hip: JointCal
    thigh: JointCal
    knee: JointCal


def _joint_cal(d):
    return JointCal(
        channel=int(d["channel"]),
        angle=float(d["angle"]),
        range=float(d["range"]),
        offset=float(d["offset"]),
    )


def load_calibration(path="config/servo_calibration.yaml"):
    """
    Load servo_calibration.yaml -> dict[leg_name] = LegCal, keyed by IK leg
    name ("FL", "FR", "RL", "RR").
    """
    path = Path(path)
    with path.open("r") as f:
        raw = yaml.safe_load(f)

    calibration = {}
    for leg_name, yaml_key in LEG_TO_YAML_KEY.items():
        if yaml_key not in raw:
            raise ValueError(f"{path}: missing leg '{yaml_key}' (for {leg_name})")
        leg_raw = raw[yaml_key]
        calibration[leg_name] = LegCal(
            hip=_joint_cal(leg_raw["Hip"]),
            thigh=_joint_cal(leg_raw["Thigh"]),
            knee=_joint_cal(leg_raw["Knee"]),
        )
    return calibration
