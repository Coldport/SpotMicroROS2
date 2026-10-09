"""
Pure leg IK math — no hardware, no calibration, no ROS. Ported unchanged from
Test _and_calib/test_IK.py's Leg.inverse_kinematics / rotational_preprocess_xyz
(the version validated against the real robot), so it can be unit-tested and
reused without a servo driver.

Coordinate convention (matches test_IK.py):
  Inputs to inverse_kinematics() are in the HIP frame, X = fore/aft,
  Y = lateral (adjusted by the 40mm hip offset internally), Z = vertical
  (leg extension). rotational_preprocess_xyz() converts a BODY-frame foot
  target + body roll/pitch/yaw into that hip frame for one named leg.
"""

import numpy as np

VALID_LEG_NAMES = {"FL", "FR", "RL", "RR"}


def inverse_kinematics(leg_name, X, Y, Z):
    """
    Args:
        leg_name: "FL", "FR", "RL", or "RR" (the link geometry is identical
            for all four legs; left/right differences are handled by the
            caller's servo/channel mapping, not here).
        X, Y, Z (float): target foot position in the HIP frame (mm).

    Returns:
        (hip_angle, lower_angle, upper_angle) in radians.
    """
    if leg_name not in VALID_LEG_NAMES:
        raise NotImplementedError(f"IK not implemented for leg '{leg_name}'")

    # Adjust Y by adding the 40mm hip offset
    Y_adjusted = Y + 40

    # Clamp to 0 instead of going negative: a target inside the 40mm dead-zone
    # around the hip axis would otherwise produce NaN (sqrt of a negative
    # number) instead of just being slightly unreachable.
    D_sq = np.maximum(Y_adjusted**2 + Z**2 - 1600, 0.0)  # 40^2 = 1600
    D = np.sqrt(D_sq)
    G = np.sqrt(D**2 + X**2)

    hip_angle = np.arctan2(Z, Y_adjusted) + np.arctan2(40, D)
    lower_angle = np.arccos(np.clip((G**2 - 28800) / (-28800), -1.0, 1.0))  # 2 * 120^2 = 28800
    upper_angle = np.arctan2(D, X) - np.arcsin((120 * np.sin(lower_angle)) / G)
    upper_angle = np.pi / 2 - upper_angle

    return hip_angle, lower_angle, upper_angle


def rotational_preprocess_xyz(leg_name, X, Y, Z, roll, pitch, yaw, angles_in_degrees=True):
    """
    Convert a BODY-frame foot target (x,y,z) into the HIP frame of leg_name,
    after applying a body orientation (roll, pitch, yaw).

    Returns:
        (x_hip, y_hip, z_hip)
    """
    if leg_name not in VALID_LEG_NAMES:
        raise NotImplementedError(f"IK not implemented for leg '{leg_name}'")

    if angles_in_degrees:
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)
    else:
        roll_rad, pitch_rad, yaw_rad = roll, pitch, yaw

    z_off = 93 * np.sin(pitch_rad)

    # pitch
    if leg_name in ["FL", "FR"]:
        new_Z1 = Z + z_off
    else:
        new_Z1 = Z - z_off

    # roll
    y_off = new_Z1 * np.sin(roll_rad)
    if leg_name in ["FL", "RL"]:
        new_Y2 = Y + y_off
    else:
        new_Y2 = Y - y_off

    # yaw
    Xoff = 43.5 * np.sin(yaw_rad)
    if leg_name in ["FL", "FR"]:
        new_X = X + Xoff
    else:
        new_X = X - Xoff

    return float(new_X), float(new_Y2), float(new_Z1)
