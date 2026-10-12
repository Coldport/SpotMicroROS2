#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Installed at <ws>/install/spot_micro_imu/share/spot_micro_imu/launch/, so the
# workspace root is five levels up (abspath, not realpath — see move.launch.py).
_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        '..', '..', '..', '..', '..'))


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spot_micro_imu'), 'config', 'imu_params.yaml')
    # The node saves its mount correction (IMU zero / axis calibration) into the
    # installed params file AND its src/ original, so a rebuild keeps it.
    params_files = ':'.join([
        default_params,
        os.path.join(_WS_ROOT, 'src', 'spot_micro_imu', 'config', 'imu_params.yaml')])

    frame_id = LaunchConfiguration('frame_id', default='imu_link')

    return LaunchDescription([

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='TF frame_id to stamp published IMU/mag/temperature messages with'),

        Node(
            package='spot_micro_imu',
            executable='bno055_node',
            name='spotmicro_imu',
            namespace='spotmicro_imu',
            parameters=[default_params, {
                'frame_id': frame_id,
                'params_files': params_files,
            }],
            output='screen'),
    ])
