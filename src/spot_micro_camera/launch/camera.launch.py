#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spot_micro_camera'), 'config', 'camera_params.yaml')

    serial_no = LaunchConfiguration('serial_no')
    pointcloud = LaunchConfiguration('pointcloud')
    colorize_depth = LaunchConfiguration('colorize_depth')

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_no',
            default_value='',
            description='Serial number of the RealSense to open (see rs-enumerate-devices -s). '
                        'Empty picks the first RealSense found, which is fine with one camera.'),

        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='Publish a colored point cloud at /spotmicro_cam/d405/depth/color/points'),

        DeclareLaunchArgument(
            'colorize_depth',
            default_value='false',
            description='For viewing only: replace the raw 16-bit depth on '
                        '/spotmicro_cam/d405/depth/image_rect_raw with a color-mapped rgb8 '
                        'image, which is easier to look at in a viewer like Foxglove than raw 16-bit '
                        'depth. Colorizing throws away the metric millimeter values, so leave '
                        'this off for anything that consumes depth.'),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d405',
            namespace='spotmicro_cam',
            parameters=[default_params, {
                # ParameterValue forces the type: without it a numeric serial
                # like 335122273035 gets parsed as an int, which the driver
                # rejects because serial_no is a string parameter.
                'serial_no': ParameterValue(serial_no, value_type=str),
                # On ARM builds (the Pi) the driver names this filter
                # "pointcloud__neon_" instead of "pointcloud", and silently
                # ignores overrides for parameters it hasn't declared, so set
                # both names to work on either architecture.
                'pointcloud__neon_.enable': ParameterValue(pointcloud, value_type=bool),
                'pointcloud.enable': ParameterValue(pointcloud, value_type=bool),
                'colorizer.enable': ParameterValue(colorize_depth, value_type=bool),
            }],
            output='screen',
            emulate_tty=True),
    ])
