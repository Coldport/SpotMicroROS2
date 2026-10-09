#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('spot_micro_deck'), 'config', 'deck_params.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'mqtt_host', default_value='localhost',
            description='MQTT broker the Steam Deck app also connects to'),
        DeclareLaunchArgument(
            'stack_launcher', default_value='true',
            description='Also run stack_launcher: hold L1+R1 on the Deck for 3 s to start, or '
                        'stop, the sensing stack + gait node (full_stack.launch.py). The gait '
                        'node still comes up disarmed until L3+R3 is held for 3 s.'),

        Node(
            package='spot_micro_deck',
            executable='deck_bridge',
            name='deck_bridge',
            namespace='spotmicro_deck',
            parameters=[default_params, {'mqtt_host': LaunchConfiguration('mqtt_host')}],
            output='screen'),

        Node(
            package='spot_micro_deck',
            executable='stack_launcher',
            name='stack_launcher',
            namespace='spotmicro_deck',
            condition=IfCondition(LaunchConfiguration('stack_launcher')),
            output='screen'),
    ])
