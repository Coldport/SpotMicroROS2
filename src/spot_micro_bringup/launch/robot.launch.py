#!/usr/bin/env python3
"""
robot.launch.py — real-robot sensing stack for the SpotMicro (no motion).

Starts:
  robot_state_publisher  — URDF: base_link -> laser / imu_link / d405_link
  sllidar_node           — RPLIDAR A2M12 (/scan)
  bno055_node            — BNO055 IMU (/spotmicro_imu/imu/data)
  realsense2_camera      — D405 color + depth (/spotmicro_cam/d405/...)
  static odom -> base_link — identity, stands in for real odometry
  slam_toolbox           — 2D lidar SLAM, publishes /map and map -> odom

It does not start gait_controller or anything that moves the robot. When the
gait controller is running it publishes the real odom -> base_link, so launch
this with static_odom:=false to avoid two publishers of the same transform.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items())


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('spot_micro_description'), 'urdf', 'spot_micro.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    slam_params = os.path.join(
        get_package_share_directory('spot_micro_bringup'), 'config', 'slam_toolbox.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port', default_value='/dev/rplidar',
            description='Serial port of the RPLIDAR (stable symlink from the udev rule)'),
        DeclareLaunchArgument(
            'lidar', default_value='true', description='Start the RPLIDAR driver'),
        DeclareLaunchArgument(
            'imu', default_value='true', description='Start the BNO055 IMU driver'),
        DeclareLaunchArgument(
            'camera', default_value='true', description='Start the RealSense D405 driver'),
        DeclareLaunchArgument(
            'slam', default_value='true', description='Start slam_toolbox'),
        DeclareLaunchArgument(
            'static_odom', default_value='true',
            description='Publish a static identity odom -> base_link. Set false when '
                        'gait_controller (or any real odometry) publishes it.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
            condition=IfCondition(LaunchConfiguration('static_odom'))),

        _include('sllidar_ros2', 'sllidar_a2m12_launch.py',
                 condition=IfCondition(LaunchConfiguration('lidar')),
                 launch_arguments={'serial_port': LaunchConfiguration('serial_port'),
                                   'frame_id': 'laser'}),

        _include('spot_micro_imu', 'imu.launch.py',
                 condition=IfCondition(LaunchConfiguration('imu'))),

        _include('spot_micro_camera', 'camera.launch.py',
                 condition=IfCondition(LaunchConfiguration('camera'))),

        # slam_toolbox's own launch file defaults use_sim_time to TRUE, which on
        # real hardware makes it wait forever for a simulated /clock.
        _include('slam_toolbox', 'online_async_launch.py',
                 condition=IfCondition(LaunchConfiguration('slam')),
                 launch_arguments={'slam_params_file': slam_params,
                                   'use_sim_time': 'false'}),
    ])
