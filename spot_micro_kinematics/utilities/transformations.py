#!/usr/bin/env python
# Source: mike4192/spot_micro_kinematics_python (MIT License)
# https://github.com/mike4192/spot_micro_kinematics_python

"""Functions for computing 3D rotation, transformation, and homogeneous transformation matrices."""

import numpy as np
from math import cos, sin


def rotx(ang):
    return np.array([[1,        0,         0],
                     [0, cos(ang), -sin(ang)],
                     [0, sin(ang),  cos(ang)]])


def roty(ang):
    return np.array([[ cos(ang), 0, sin(ang)],
                     [        0, 1,        0],
                     [-sin(ang), 0, cos(ang)]])


def rotz(ang):
    return np.array([[cos(ang), -sin(ang), 0],
                     [sin(ang),  cos(ang), 0],
                     [       0,         0, 1]])


def rotxyz(x_ang, y_ang, z_ang):
    return np.matmul(np.matmul(rotx(x_ang), roty(y_ang)), rotz(z_ang))


def homog_rotxyz(x_ang, y_ang, z_ang):
    return np.block([[rotxyz(x_ang, y_ang, z_ang), np.zeros((3, 1))],
                     [np.array([0, 0, 0, 1])]])


def homog_transxyz(x, y, z):
    return np.block([[np.eye(3), np.array([[x], [y], [z]])],
                     [np.array([0, 0, 0, 1])]])


def homog_transform(x_ang, y_ang, z_ang, x_t, y_t, z_t):
    return np.matmul(homog_rotxyz(x_ang, y_ang, z_ang), homog_transxyz(x_t, y_t, z_t))


def ht_inverse(ht):
    temp_rot    = ht[0:3, 0:3].transpose()
    temp_vec    = -ht[0:3, 3]
    temp_rot_ht = np.block([[temp_rot,        np.zeros((3, 1))],
                             [np.zeros((1, 3)), np.eye(1)]])
    temp_vec_ht      = np.eye(4)
    temp_vec_ht[0:3, 3] = temp_vec
    return np.matmul(temp_rot_ht, temp_vec_ht)
