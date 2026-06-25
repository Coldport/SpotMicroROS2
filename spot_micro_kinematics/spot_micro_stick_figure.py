# Source: mike4192/spot_micro_kinematics_python (MIT License)
# https://github.com/mike4192/spot_micro_kinematics_python

from math import pi
import numpy as np
from .utilities import spot_micro_kinematics as smk
from .utilities import transformations

d2r = pi / 180
r2d = 180 / pi


class SpotMicroLeg:
    """One leg of SpotMicro: 3 links, 3 joints (hip, upper leg, knee)."""

    def __init__(self, q1, q2, q3, l1, l2, l3, ht_leg_start, leg12):
        self._q1 = q1
        self._q2 = q2
        self._q3 = q3
        self._l1 = l1
        self._l2 = l2
        self._l3 = l3
        self._ht_leg_start = ht_leg_start
        self._leg12 = leg12
        self._t01 = smk.t_0_to_1(q1, l1)
        self._t12 = smk.t_1_to_2()
        self._t23 = smk.t_2_to_3(q2, l2)
        self._t34 = smk.t_3_to_4(q3, l3)

    def set_angles(self, q1, q2, q3):
        self._q1, self._q2, self._q3 = q1, q2, q3
        self._t01 = smk.t_0_to_1(q1, self._l1)
        self._t23 = smk.t_2_to_3(q2, self._l2)
        self._t34 = smk.t_3_to_4(q3, self._l3)

    def set_homog_transf(self, ht):
        self._ht_leg_start = ht

    def get_homog_transf(self):
        return self._ht_leg_start

    def set_foot_position_in_local_coords(self, x4, y4, z4):
        q = smk.ikine(x4, y4, z4, self._l1, self._l2, self._l3, self._leg12)
        self.set_angles(*q)

    def set_foot_position_in_global_coords(self, x4, y4, z4):
        ht_inv = transformations.ht_inverse(self._ht_leg_start)
        p4_leg = ht_inv.dot(np.array([x4, y4, z4, 1]))
        self.set_foot_position_in_local_coords(p4_leg[0], p4_leg[1], p4_leg[2])

    def get_leg_points(self):
        """Return (p1, p2, p3, p4) joint positions in global frame (metres)."""
        p1 = self._ht_leg_start[0:3, 3]
        ht = np.matmul(np.matmul(self._ht_leg_start, self._t01), self._t12)
        p2 = ht[0:3, 3]
        ht = np.matmul(ht, self._t23)
        p3 = ht[0:3, 3]
        ht = np.matmul(ht, self._t34)
        p4 = ht[0:3, 3]
        return (p1, p2, p3, p4)

    def get_foot_position_in_global_coords(self):
        ht = np.matmul(np.matmul(np.matmul(np.matmul(
            self._ht_leg_start, self._t01), self._t12), self._t23), self._t34)
        return ht[0:3, 3]

    def get_leg_angles(self):
        return (self._q1, self._q2, self._q3)


class SpotMicroStickFigure:
    """Full 12-DOF SpotMicro kinematic model with analytical IK.

    Coordinate system (this class):
        x = forward,  y = up,  z = right
        Units: metres,  angles: radians

    Default dimensions match a standard SpotMicro build.
    """

    # Physical dimensions (metres) — standard SpotMicro
    HIP_LENGTH       = 0.055
    UPPER_LEG_LENGTH = 0.1075
    LOWER_LEG_LENGTH = 0.130
    BODY_WIDTH       = 0.078
    BODY_LENGTH      = 0.186

    def __init__(self, x=0.0, y=0.18, z=0.0, phi=0.0, theta=0.0, psi=0.0):
        self.x, self.y, self.z         = x, y, z
        self.phi, self.theta, self.psi = phi, theta, psi

        self.hip_length        = self.HIP_LENGTH
        self.upper_leg_length  = self.UPPER_LEG_LENGTH
        self.lower_leg_length  = self.LOWER_LEG_LENGTH
        self.body_width        = self.BODY_WIDTH
        self.body_length       = self.BODY_LENGTH

        self.ht_body = np.matmul(
            transformations.homog_transxyz(x, y, z),
            transformations.homog_rotxyz(phi, psi, theta))

        rb = [0, -30*d2r,  60*d2r]
        rf = [0, -30*d2r,  60*d2r]
        lf = [0,  30*d2r, -60*d2r]
        lb = [0,  30*d2r, -60*d2r]

        L, W = self.body_length, self.body_width
        l1, l2, l3 = self.hip_length, self.upper_leg_length, self.lower_leg_length

        self.legs = {
            'leg_rightback':  SpotMicroLeg(*rb, l1, l2, l3, smk.t_rightback(self.ht_body,  L, W), leg12=True),
            'leg_rightfront': SpotMicroLeg(*rf, l1, l2, l3, smk.t_rightfront(self.ht_body, L, W), leg12=True),
            'leg_leftfront':  SpotMicroLeg(*lf, l1, l2, l3, smk.t_leftfront(self.ht_body,  L, W), leg12=False),
            'leg_leftback':   SpotMicroLeg(*lb, l1, l2, l3, smk.t_leftback(self.ht_body,   L, W), leg12=False),
        }

    # ── pose control ─────────────────────────────────────────────────────────

    def set_body_angles(self, phi=0.0, theta=0.0, psi=0.0):
        """Rotate body in place (feet stay fixed in global frame)."""
        r_xyz = transformations.rotxyz(phi, psi, theta)
        ht = self.ht_body.copy()
        ht[0:3, 0:3] = r_xyz
        self.set_absolute_body_pose(ht)

    def set_absolute_body_pose(self, ht_body):
        """Move/rotate body while keeping feet in their current global positions."""
        feet = {name: leg.get_foot_position_in_global_coords()
                for name, leg in self.legs.items()}
        self.ht_body = ht_body
        L, W = self.body_length, self.body_width
        self.legs['leg_rightback'].set_homog_transf(smk.t_rightback(ht_body,  L, W))
        self.legs['leg_rightfront'].set_homog_transf(smk.t_rightfront(ht_body, L, W))
        self.legs['leg_leftfront'].set_homog_transf(smk.t_leftfront(ht_body,  L, W))
        self.legs['leg_leftback'].set_homog_transf(smk.t_leftback(ht_body,   L, W))
        foot_mat = np.array([feet['leg_rightback'],
                             feet['leg_rightfront'],
                             feet['leg_leftfront'],
                             feet['leg_leftback']])
        self.set_absolute_foot_coordinates(foot_mat)

    def set_absolute_foot_coordinates(self, foot_coords):
        """Set all four feet from a (4, 3) array in global frame (metres).

        Row order: right_back, right_front, left_front, left_back.
        """
        order = ['leg_rightback', 'leg_rightfront', 'leg_leftfront', 'leg_leftback']
        for i, name in enumerate(order):
            self.legs[name].set_foot_position_in_global_coords(*foot_coords[i])

    def set_leg_angles(self, leg_angs):
        """Set all 12 angles directly.

        leg_angs: ((rb_q1,rb_q2,rb_q3), (rf_...), (lf_...), (lb_...))
        """
        order = ['leg_rightback', 'leg_rightfront', 'leg_leftfront', 'leg_leftback']
        for i, name in enumerate(order):
            self.legs[name].set_angles(*leg_angs[i])

    # ── read-back ─────────────────────────────────────────────────────────────

    def get_leg_angles(self):
        """Return ((rb_q1,rb_q2,rb_q3), (rf_...), (lf_...), (lb_...)) in radians."""
        return (self.legs['leg_rightback'].get_leg_angles(),
                self.legs['leg_rightfront'].get_leg_angles(),
                self.legs['leg_leftfront'].get_leg_angles(),
                self.legs['leg_leftback'].get_leg_angles())

    def get_leg_coordinates(self):
        """Return joint positions for all legs as tuple of 4 × (p1,p2,p3,p4)."""
        return (self.legs['leg_rightback'].get_leg_points(),
                self.legs['leg_rightfront'].get_leg_points(),
                self.legs['leg_leftfront'].get_leg_points(),
                self.legs['leg_leftback'].get_leg_points())
