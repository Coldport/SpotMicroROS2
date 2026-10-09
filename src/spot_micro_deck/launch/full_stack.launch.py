"""
full_stack.launch.py — sensing stack + gait node together.

This is what stack_launcher runs by default when the Deck holds L1+R1 for 3s:
robot.launch.py (sensing) plus spot_micro_move's gait_node. The gait node
opens the real I2C/PCA9685 bus as soon as it starts, but stays DISARMED
(nothing commanded) until the Deck holds L3+R3 for 3s — see
spot_micro_move/gait_node.py. Holding L1+R1 again stops everything started
here, gait node included.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'deck', default_value='false',
            description='Forwarded to robot.launch.py. False by default: the deck_bridge '
                        'that reads this chord is already running (e.g. the spotmicro-deck '
                        'boot service), so a second one would fight it for the MQTT client id.'),

        _include('spot_micro_bringup', 'robot.launch.py',
                 launch_arguments={'deck': LaunchConfiguration('deck')}),

        _include('spot_micro_move', 'move.launch.py'),
    ])
