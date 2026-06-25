"""Forward and inverse kinematic transformations for a quadruped robot.

Source: mike4192/spot_micro_kinematics_python (MIT License)
https://github.com/mike4192/spot_micro_kinematics_python

Equations from:
    Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017).
    Inverse Kinematic Analysis Of A Quadruped Robot.
    International Journal of Scientific & Technology Research. 6.
"""

from . import transformations
from math import pi, cos, sin, atan2, sqrt
import numpy as np


def t_rightback(t_m, l, w):
    tmp = np.block([[transformations.roty(pi/2), np.array([[-l/2], [0], [w/2]])],
                    [np.array([0, 0, 0, 1])]])
    return np.matmul(t_m, tmp)


def t_rightfront(t_m, l, w):
    tmp = np.block([[transformations.roty(pi/2), np.array([[l/2], [0], [w/2]])],
                    [np.array([0, 0, 0, 1])]])
    return np.matmul(t_m, tmp)


def t_leftfront(t_m, l, w):
    tmp = np.block([[transformations.roty(-pi/2), np.array([[l/2], [0], [-w/2]])],
                    [np.array([0, 0, 0, 1])]])
    return np.matmul(t_m, tmp)


def t_leftback(t_m, l, w):
    tmp = np.block([[transformations.roty(-pi/2), np.array([[-l/2], [0], [-w/2]])],
                    [np.array([0, 0, 0, 1])]])
    return np.matmul(t_m, tmp)


def t_0_to_1(theta1, l1):
    return np.block([[transformations.rotz(theta1),
                      np.array([[-l1*cos(theta1)], [-l1*sin(theta1)], [0]])],
                     [np.array([0, 0, 0, 1])]])


def t_1_to_2():
    return np.array([[ 0,  0, -1,  0],
                     [-1,  0,  0,  0],
                     [ 0,  1,  0,  0],
                     [ 0,  0,  0,  1]], dtype=float)


def t_2_to_3(theta2, l2):
    return np.block([[transformations.rotz(theta2),
                      np.array([[l2*cos(theta2)], [l2*sin(theta2)], [0]])],
                     [np.array([0, 0, 0, 1])]])


def t_3_to_4(theta3, l3):
    return np.block([[transformations.rotz(theta3),
                      np.array([[l3*cos(theta3)], [l3*sin(theta3)], [0]])],
                     [np.array([0, 0, 0, 1])]])


def t_0_to_4(theta1, theta2, theta3, l1, l2, l3):
    return np.matmul(np.matmul(np.matmul(
        t_0_to_1(theta1, l1), t_1_to_2()),
        t_2_to_3(theta2, l2)),
        t_3_to_4(theta3, l3))


def ikine(x4, y4, z4, l1, l2, l3, legs12=True):
    """Analytical IK for one leg.

    Args:
        x4, y4, z4: Foot position in the leg's local coordinate frame (metres).
        l1: Hip link length (metres).
        l2: Upper leg length (metres).
        l3: Lower leg length (metres).
        legs12: True for right legs (1 & 2), False for left legs (3 & 4).

    Returns:
        (q1, q2, q3) joint angles in radians — (hip, upper_leg, knee).
    """
    D = (x4**2 + y4**2 + z4**2 - l1**2 - l2**2 - l3**2) / (2 * l2 * l3)
    D = max(-1.0, min(1.0, D))  # clamp for numerical safety

    q3 = atan2( sqrt(1 - D**2), D) if legs12 else atan2(-sqrt(1 - D**2), D)
    q2 = atan2(z4, sqrt(max(0.0, x4**2 + y4**2 - l1**2))) \
         - atan2(l3 * sin(q3), l2 + l3 * cos(q3))
    q1 = atan2(y4, x4) + atan2(sqrt(max(0.0, x4**2 + y4**2 - l1**2)), -l1)

    return (q1, q2, q3)
