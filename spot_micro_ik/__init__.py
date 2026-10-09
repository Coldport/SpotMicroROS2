"""
spot_micro_ik — standalone SpotMicro leg IK + servo calibration library.

Extracted from Test _and_calib/test_IK.py (the hand-tuned IK validated against
the real robot) so it can be reused by the spot_micro_move ROS2 package without
depending on ROS or on hardware libraries (adafruit_motor/adafruit_pca9685/board
are only imported by spot_micro_ik.hardware, which nothing here imports at
module load time).

    from spot_micro_ik import load_calibration, Dog

    cal = load_calibration("config/servo_calibration.yaml")
    dog = Dog(cal)                 # dry_run=True by default — computes, doesn't move
    dog.move_all(x=0, y=0, z=190, roll=0, pitch=0, yaw=0)
"""

from .calibration import JointCal, LegCal, load_calibration
from .kinematics import inverse_kinematics, rotational_preprocess_xyz
from .leg import Leg, LEG_NAMES, RIGHT_LEGS
from .dog import Dog

__all__ = [
    "JointCal", "LegCal", "load_calibration",
    "inverse_kinematics", "rotational_preprocess_xyz",
    "Leg", "LEG_NAMES", "RIGHT_LEGS",
    "Dog",
]
