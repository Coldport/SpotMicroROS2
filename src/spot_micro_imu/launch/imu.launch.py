#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spot_micro_imu'), 'config', 'imu_params.yaml')

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
            }],
            output='screen'),
    ])
