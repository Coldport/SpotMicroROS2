"""
Loads config/servo_calibration.yaml into typed, in-memory calibration data.

This is the YAML counterpart of the older servo_calibration.txt (JSON) that
Servo_calib.py, the manual jogging GUI, still owns. The two files are NOT kept
in sync automatically — see the comment at the top of the .yaml.
"""

import os
import re
import tempfile
from dataclasses import dataclass, replace
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


JOINTS = ("hip", "thigh", "knee")
_YAML_JOINT = {"hip": "Hip", "thigh": "Thigh", "knee": "Knee"}


def with_offset(leg_cal, joint, offset):
    """Copy of `leg_cal` with one joint's ("hip"/"thigh"/"knee") offset changed."""
    return replace(leg_cal, **{joint: replace(getattr(leg_cal, joint), offset=float(offset))})


def _format_offset(value):
    value = round(float(value), 2)
    return str(int(value)) if value == int(value) else repr(value)


def update_offsets_text(text, calibration):
    """Return the calibration YAML `text` with every joint's `offset:` set from
    `calibration` (dict[leg_name] -> LegCal). Line-based, so comments and layout
    survive; raises ValueError if a leg/joint/offset line can't be found."""
    want = {}
    for leg_name, yaml_key in LEG_TO_YAML_KEY.items():
        for joint in JOINTS:
            want[(yaml_key, _YAML_JOINT[joint])] = getattr(calibration[leg_name], joint).offset
    leg = joint = None
    lines = text.split("\n")
    for i, line in enumerate(lines):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        m = re.match(r"^(\s*)([^:#]+):\s*(.*?)(\s+#.*)?$", line)
        if not m:
            continue
        key = m.group(2).strip()
        if indent == 0:
            leg, joint = key, None
        elif m.group(3) == "":
            joint = key
        elif key == "offset" and (leg, joint) in want:
            lines[i] = f"{m.group(1)}offset: {_format_offset(want.pop((leg, joint)))}{m.group(4) or ''}"
    if want:
        missing = ", ".join(f"{l}/{j}" for l, j in want)
        raise ValueError(f"offset line not found for {missing}")
    return "\n".join(lines)


def save_offsets(path, calibration):
    """Write the joint offsets in `calibration` back into the YAML at `path`
    (atomic replace; symlinks followed). Only `offset:` values change."""
    path = os.path.realpath(path)
    with open(path) as f:
        text = f.read()
    new_text = update_offsets_text(text, calibration)
    if new_text == text:
        return False
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix="." + os.path.basename(path))
    try:
        with os.fdopen(fd, "w") as f:
            f.write(new_text)
        os.chmod(tmp, os.stat(path).st_mode & 0o7777)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return True
