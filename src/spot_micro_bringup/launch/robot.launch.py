#!/usr/bin/env python3
"""
robot.launch.py — real-robot sensing stack for the SpotMicro (no motion).

Starts:
  robot_state_publisher  — URDF: base_link -> laser / imu_link / d405_link
  sllidar_node           — RPLIDAR A2M12 (/scan)
  bno055_node            — BNO055 IMU (/spotmicro_imu/imu/data)
  camera                 — whichever is plugged in (spot_micro_camera auto-detects):
                           RealSense D405 color + depth (/spotmicro_cam/d405/...) or a
                           USB webcam, color only (/spotmicro_cam/usb/...)
  static odom -> base_link — identity, stands in for real odometry
  scan_gate              — /scan -> /scan_gated, drops scans taken while a foot is up
                           (passes everything when gait_node isn't running); also stops
                           the lidar while gait_node is in sentry mode
  slam_toolbox           — 2D lidar SLAM on /scan_gated, publishes /map and map -> odom
  deck_bridge            — MQTT bridge to the Steam Deck app (streams + /spotmicro_deck/joy)

It does not start gait_controller or anything that moves the robot. When the
gait controller is running it publishes the real odom -> base_link, so launch
this with static_odom:=false to avoid two publishers of the same transform.
"""

import os
import subprocess

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


def _bridge_already_running():
    """True if a deck_bridge is up (e.g. the spotmicro-deck boot service).

    The bridge uses a fixed MQTT client id, so a second one would keep kicking the
    first off the broker.
    """
    try:
        return subprocess.run(['pgrep', '-f', 'spot_micro_deck/deck_bridge'],
                              capture_output=True, check=False, timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


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
            'camera', default_value='true', description='Start the camera driver'),
        DeclareLaunchArgument(
            'camera_type', default_value='auto',
            description='auto (detect: RealSense D405 if plugged in, else a USB webcam), '
                        'realsense or usb'),
        DeclareLaunchArgument(
            'slam', default_value='true', description='Start slam_toolbox'),
        DeclareLaunchArgument(
            'deck', default_value='false' if _bridge_already_running() else 'true',
            description='Start the Steam Deck bridge (needs the Mosquitto broker running). '
                        'Defaults to false when a bridge is already running, e.g. the '
                        'spotmicro-deck boot service.'),
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
                 condition=IfCondition(LaunchConfiguration('camera')),
                 launch_arguments={'camera_type': LaunchConfiguration('camera_type')}),

        _include('spot_micro_deck', 'deck.launch.py',
                 condition=IfCondition(LaunchConfiguration('deck'))),

        # Relay that keeps SLAM fed even with no gait_node running. Runs with the
        # lidar (not just SLAM) because it also stops the lidar in sentry mode.
        Node(
            package='spot_micro_move',
            executable='scan_gate',
            name='scan_gate',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lidar'))),

        # slam_toolbox's own launch file defaults use_sim_time to TRUE, which on
        # real hardware makes it wait forever for a simulated /clock.
        _include('slam_toolbox', 'online_async_launch.py',
                 condition=IfCondition(LaunchConfiguration('slam')),
                 launch_arguments={'slam_params_file': slam_params,
                                   'use_sim_time': 'false'}),
    ])
