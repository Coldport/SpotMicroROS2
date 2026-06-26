"""
sim.launch.py — full SpotMicro simulation stack

Starts:
  robot_state_publisher  — URDF + TF tree
  gait_controller        — trot gait, publishes /joint_states and /odom
  point_nav              — drives to /goal_pose via /cmd_vel
  rviz2                  — visualisation
"""

import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# The installed launch file sits at:
#   <ws>/install/spot_micro_control/share/spot_micro_control/launch/
# Five levels up is the workspace root, where spot_micro_kinematics/ lives.
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_WS_ROOT = os.path.abspath(os.path.join(_LAUNCH_DIR, '..', '..', '..', '..', '..'))
_PYTHONPATH = _WS_ROOT + ':' + os.environ.get('PYTHONPATH', '')


def generate_launch_description():
    desc_pkg = FindPackageShare('spot_micro_description')

    xacro_file  = PathJoinSubstitution([desc_pkg, 'urdf', 'spot_micro.urdf.xacro'])
    rviz_config = PathJoinSubstitution([desc_pkg, 'rviz', 'spot_micro.rviz'])

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='spot_micro_control',
            executable='gait_controller',
            name='gait_controller',
            output='screen',
            additional_env={'PYTHONPATH': _PYTHONPATH},
        ),

        Node(
            package='spot_micro_control',
            executable='point_nav',
            name='point_nav',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
