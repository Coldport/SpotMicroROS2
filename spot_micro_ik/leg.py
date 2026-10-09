"""
Leg — ties the pure IK math (kinematics.py) to one leg's calibration, and
produces final per-channel servo angles (degrees, clamped, offset-applied).

Channel assignment and the right-leg 180-degree mirror reproduce
Test _and_calib/test_IK.py's Move_To_L exactly (validated on the real robot):
  hip_angle          -> calibration's Hip channel
  lower_angle (2nd)  -> calibration's Knee channel   (test_IK.py calls this "Th_ang")
  upper_angle (3rd)  -> calibration's Thigh channel  (test_IK.py calls this "An_ang")
That lower/upper <-> Knee/Thigh pairing looks swapped against the calibration
joint *names*, but it's how the servos are actually wired on this robot — kept
as-is rather than "fixed".

Right legs (FR, RR) mirror the IK output: servo_angle = 180 - angle.

One behavior change from test_IK.py: test_IK.py never applied the calibration
"offset" at all. This module does (offset is added before clamping, same as
Servo_calib.py's manual jogging tool) — so re-check the standing pose before
enabling motion after switching from test_IK.py.
"""

import math

from .kinematics import inverse_kinematics, rotational_preprocess_xyz

LEG_NAMES = ("FL", "FR", "RL", "RR")
RIGHT_LEGS = {"FR", "RR"}   # mirrored: servo_angle = 180 - angle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class Leg:
    """One leg's IK + calibration. Pure computation — does not touch hardware."""

    def __init__(self, leg_name, leg_cal):
        if leg_name not in LEG_NAMES:
            raise ValueError(f"leg_name must be one of {LEG_NAMES}, got '{leg_name}'")
        self.leg_name = leg_name
        self.cal = leg_cal
        self.mirrored = leg_name in RIGHT_LEGS

    def solve(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Args:
            x, y, z: foot target in the BODY frame (mm).
            roll, pitch, yaw: body orientation (deg).

        Returns:
            dict[int channel] -> float angle_deg, ready to write to that
            servo channel. A channel is omitted if the IK target was
            unreachable (non-finite result) for that joint.
        """
        xh, yh, zh = rotational_preprocess_xyz(self.leg_name, x, y, z, roll, pitch, yaw)
        hip_rad, lower_rad, upper_rad = inverse_kinematics(self.leg_name, xh, yh, zh)

        out = {}
        self._add(out, self.cal.hip, math.degrees(hip_rad))
        self._add(out, self.cal.knee, math.degrees(lower_rad))
        self._add(out, self.cal.thigh, math.degrees(upper_rad))
        return out

    def _add(self, out, joint_cal, angle_deg):
        if not math.isfinite(angle_deg):
            return
        if self.mirrored:
            angle_deg = 180.0 - angle_deg
        commanded = _clamp(angle_deg + joint_cal.offset, 0.0, joint_cal.range)
        out[joint_cal.channel] = commanded
