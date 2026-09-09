#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spot_micro_camera'), 'config', 'camera_params.yaml')

    video_device = LaunchConfiguration('video_device', default='/dev/spotmicro_cam')
    frame_id = LaunchConfiguration('frame_id', default='camera')

    return LaunchDescription([

        DeclareLaunchArgument(
            'video_device',
            default_value=video_device,
            description='V4L2 device path for the camera (the udev rule keeps this stable at '
                        '/dev/spotmicro_cam regardless of /dev/videoN enumeration)'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='TF frame_id to stamp published images with'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='spotmicro_cam',
            namespace='spotmicro_cam',
            parameters=[default_params, {
                'video_device': video_device,
                'frame_id': frame_id,
            }],
            output='screen'),
    ])
