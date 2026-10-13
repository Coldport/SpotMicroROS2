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

    def solve(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, joint_angles=None):
        """
        Args:
            x, y, z: foot target in the BODY frame (mm).
            roll, pitch, yaw: body orientation (deg).
            joint_angles: optional {"hip"/"thigh"/"knee": servo angle (deg)}
                overriding IK for those joints. Same frame as joint_angles()
                returns (right-leg mirror applied, offset NOT applied): the
                offset is still added, which is what calibrating by angle needs.

        Returns:
            dict[int channel] -> float angle_deg, ready to write to that
            servo channel. A channel is omitted if the IK target was
            unreachable (non-finite result) for that joint.
        """
        angles = self.joint_angles(x, y, z, roll, pitch, yaw)
        angles.update(joint_angles or {})
        out = {}
        for joint, angle_deg in angles.items():
            if angle_deg is not None:
                joint_cal = getattr(self.cal, joint)
                out[joint_cal.channel] = _clamp(angle_deg + joint_cal.offset, 0.0, joint_cal.range)
        return out

    def joint_angles(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """IK for a foot target -> {"hip"/"thigh"/"knee": servo angle (deg) before
        the calibration offset, right-leg mirror applied}, or None for a joint
        whose IK result is non-finite (unreachable)."""
        xh, yh, zh = rotational_preprocess_xyz(self.leg_name, x, y, z, roll, pitch, yaw)
        hip_rad, lower_rad, upper_rad = inverse_kinematics(self.leg_name, xh, yh, zh)
        out = {}
        for joint, rad in (("hip", hip_rad), ("knee", lower_rad), ("thigh", upper_rad)):
            deg = math.degrees(rad)
            if not math.isfinite(deg):
                out[joint] = None
                continue
            out[joint] = 180.0 - deg if self.mirrored else deg
        return out

    def servo_command(self, joint, angle_deg):
        """Servo angle (deg, pre-offset) for one joint -> the value actually
        written to its channel (offset added, clamped to the servo's range)."""
        joint_cal = getattr(self.cal, joint)
        return _clamp(angle_deg + joint_cal.offset, 0.0, joint_cal.range)
