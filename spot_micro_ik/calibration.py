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


# The scale term pivots here, so `offset` keeps meaning "correction at 90 deg"
# (the middle of the IK range) whatever the scale is.
SCALE_PIVOT_DEG = 90.0
# Two-point calibration results outside these are treated as a bad measurement.
SCALE_LIMITS = (0.75, 1.33)
MIN_POINT_SPREAD_DEG = 15.0


@dataclass(frozen=True)
class JointCal:
    channel: int
    angle: float     # recorded neutral angle (deg) — informational, not used by IK
    range: float      # servo actuation range (deg): 180 or 270
    offset: float     # deg added to the commanded angle before clamping to [0, range]
    # Gain about SCALE_PIVOT_DEG (two-point calibration, see fit_joint); 1.0 = none.
    scale: float = 1.0

    def command(self, angle_deg):
        """Servo angle before calibration (IK frame, right-leg mirror applied)
        -> the angle written to the servo, before clamping to [0, range]."""
        return SCALE_PIVOT_DEG + self.scale * (angle_deg - SCALE_PIVOT_DEG) + self.offset


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
        scale=float(d.get("scale", 1.0)),
    )


def load_calibration(path="config/servo_calibration.yaml"):
    """
    Load servo_calibration.yaml -> dict[leg_name] = LegCal, keyed by IK leg
    name ("FL", "FR", "RL", "RR").
    """
    path = Path(path)
    with path.open("r", encoding='utf-8') as f:
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


def with_cal(leg_cal, joint, offset, scale):
    """Copy of `leg_cal` with one joint's offset and scale changed."""
    return replace(leg_cal, **{joint: replace(getattr(leg_cal, joint), offset=float(offset), scale=float(scale))})


def fit_joint(points, scale=1.0):
    """Two-point (or one-point) servo calibration.

    points: [(servo, actual), ...] — `servo` is the angle that was written to
    the servo, `actual` the angle the joint really sat at, in the same IK frame
    as Leg.joint_angles (right-leg mirror applied). One point fits the offset
    only (keeping `scale`); two fit offset and scale exactly.
    Returns (offset, scale); raises ValueError for unusable points.
    """
    if not points:
        raise ValueError("no points recorded")
    if len(points) == 1:
        (c1, a1), = points
    else:
        (c1, a1), (c2, a2) = points[-2:]
        if abs(a2 - a1) < MIN_POINT_SPREAD_DEG:
            raise ValueError(f"points must be at least {MIN_POINT_SPREAD_DEG:.0f} deg apart")
        scale = (c2 - c1) / (a2 - a1)
        lo, hi = SCALE_LIMITS
        if not lo <= scale <= hi:
            raise ValueError(f"scale {scale:.2f} is outside {lo}-{hi}: re-check the angles you entered")
    offset = c1 - SCALE_PIVOT_DEG - scale * (a1 - SCALE_PIVOT_DEG)
    return round(offset, 2), round(scale, 4)


def _format_offset(value):
    value = round(float(value), 2)
    return str(int(value)) if value == int(value) else repr(value)


def update_offsets_text(text, calibration):
    """Return the calibration YAML `text` with every joint's `offset:` (and
    `scale:`, added under the offset where it isn't 1 and has no line yet) set
    from `calibration` (dict[leg_name] -> LegCal). Line-based, so comments and
    layout survive; raises ValueError if a leg/joint/offset line can't be found."""
    want = {}
    for leg_name, yaml_key in LEG_TO_YAML_KEY.items():
        for joint in JOINTS:
            jc = getattr(calibration[leg_name], joint)
            want[(yaml_key, _YAML_JOINT[joint])] = (jc.offset, jc.scale)
    lines = text.split("\n")
    # First pass: which joints already have a scale line.
    has_scale = set()
    leg = joint = None
    parsed = []
    for line in lines:
        m = None
        if line.strip() and not line.lstrip().startswith("#"):
            m = re.match(r"^(\s*)([^:#]+):\s*(.*?)(\s+#.*)?$", line)
        if m:
            key = m.group(2).strip()
            if not m.group(1):
                leg, joint = key, None
            elif m.group(3) == "":
                joint = key
            elif key == "scale":
                has_scale.add((leg, joint))
        parsed.append((m, leg, joint))
    out = []
    missing = set(want)
    for line, (m, leg, joint) in zip(lines, parsed):
        if m is None or (leg, joint) not in want:
            out.append(line)
            continue
        key = m.group(2).strip()
        offset, scale = want[(leg, joint)]
        if key == "offset":
            missing.discard((leg, joint))
            out.append(f"{m.group(1)}offset: {_format_offset(offset)}{m.group(4) or ''}")
            if scale != 1.0 and (leg, joint) not in has_scale:
                out.append(f"{m.group(1)}scale: {_format_scale(scale)}")
        elif key == "scale":
            out.append(f"{m.group(1)}scale: {_format_scale(scale)}{m.group(4) or ''}")
        else:
            out.append(line)
    if missing:
        raise ValueError(f"offset line not found for {', '.join(f'{l}/{j}' for l, j in sorted(missing))}")
    return "\n".join(out)


def _format_scale(value):
    value = round(float(value), 4)
    return str(int(value)) if value == int(value) else repr(value)


def save_offsets(path, calibration):
    """Write the joint offsets (and scales) in `calibration` back into the YAML
    at `path` (atomic replace; symlinks followed). Only `offset:` / `scale:`
    values change."""
    path = os.path.realpath(path)
    with open(path, encoding='utf-8') as f:
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
