"""
move.launch.py — gait_node, standalone.

Not part of spot_micro_bringup on purpose: bringup stays sensing-only.
gait_node opens the real I2C/PCA9685 bus at startup (no dry-run mode) but
starts DISARMED — nothing is commanded until the Deck holds l3+r3 for 3s.

    ros2 launch spot_micro_move move.launch.py
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The installed launch file sits at:
#   <ws>/install/spot_micro_move/share/spot_micro_move/launch/
# Five levels up is the workspace root, where spot_micro_ik/ and config/ live.
# abspath, NOT realpath: with --symlink-install this file is a symlink to the
# src/ copy, and realpath() would resolve it back there — 2 levels below the
# workspace root instead of 4, silently pointing PYTHONPATH at /home instead
# of the workspace (this broke gait_node with ModuleNotFoundError until found
# 2026-09-24 by testing the actual `ros2 launch` path, not just direct-Python
# test scripts that set PYTHONPATH themselves).
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_LAUNCH_DIR, '..', '..', '..', '..', '..'))

# gait_node runs under the system ROS interpreter (ros2 launch's console_scripts shebang),
# not .venv — but board/adafruit_motor/adafruit_pca9685 (spot_micro_ik.hardware's real
# ServoDriver) are only installed in .venv, since nothing else in this workspace touches
# real GPIO/I2C hardware libs from a ROS node. Put .venv's site-packages on PYTHONPATH too
# so the same interpreter can still import them (.venv is --system-site-packages and the
# same Python 3.12, so this is ABI-safe, not a real venv "activation").
_VENV_SITE_PACKAGES = glob.glob(os.path.join(_WS_ROOT, '.venv', 'lib', 'python3.*', 'site-packages'))

_PYTHONPATH = ':'.join([_WS_ROOT, *_VENV_SITE_PACKAGES, os.environ.get('PYTHONPATH', '')])
_DEFAULT_CALIBRATION = os.path.join(_WS_ROOT, 'config', 'servo_calibration.yaml')


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spot_micro_move'), 'config', 'move_params.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'calibration_path', default_value=_DEFAULT_CALIBRATION,
            description='Path to servo_calibration.yaml'),

        Node(
            package='spot_micro_move',
            executable='gait_node',
            name='gait_node',
            output='screen',
            additional_env={'PYTHONPATH': _PYTHONPATH},
            parameters=[default_params, {
                'calibration_path': LaunchConfiguration('calibration_path'),
            }],
        ),
    ])
