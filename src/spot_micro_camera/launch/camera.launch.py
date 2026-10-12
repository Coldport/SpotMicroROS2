#!/usr/bin/env python3
"""
camera.launch.py — starts the driver for whichever camera is plugged in.

  RealSense (D405)  realsense2_camera -> /spotmicro_cam/d405/color/..., .../depth/...
  USB webcam        usb_cam           -> /spotmicro_cam/usb/image_raw (+ /compressed)

camera_type:=auto (default) asks spot_micro_camera.detect, which reads sysfs;
with both plugged in the RealSense wins. Force one with camera_type:=realsense
or camera_type:=usb. If auto finds nothing, no driver is started (re-launch
after plugging one in). deck_bridge works out which one is running on its own.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from spot_micro_camera.detect import REALSENSE, USB, Camera, detect_camera, find_usb_webcam


def _realsense(config):
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d405',
        namespace='spotmicro_cam',
        parameters=[os.path.join(config, 'camera_params.yaml'), {
            # ParameterValue forces the type: without it a numeric serial
            # like 335122273035 gets parsed as an int, which the driver
            # rejects because serial_no is a string parameter.
            'serial_no': ParameterValue(LaunchConfiguration('serial_no'), value_type=str),
            # On ARM builds (the Pi) the driver names this filter
            # "pointcloud__neon_" instead of "pointcloud", and silently
            # ignores overrides for parameters it hasn't declared, so set
            # both names to work on either architecture.
            'pointcloud__neon_.enable': ParameterValue(LaunchConfiguration('pointcloud'), value_type=bool),
            'pointcloud.enable': ParameterValue(LaunchConfiguration('pointcloud'), value_type=bool),
            'colorizer.enable': ParameterValue(LaunchConfiguration('colorize_depth'), value_type=bool),
        }],
        output='screen',
        emulate_tty=True)


def _usb(config, device):
    return Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb',
        namespace='spotmicro_cam',
        parameters=[os.path.join(config, 'usb_cam_params.yaml'), {'video_device': device}],
        # usb_cam publishes image_raw/camera_info in the namespace, not under its
        # node name; keep them beside the node so the topics say which camera it is.
        remappings=[('camera_info', 'usb/camera_info')] + [
            (f'image_raw{t}', f'usb/image_raw{t}')
            for t in ('', '/compressed', '/compressedDepth', '/theora', '/zstd')],
        output='screen')


def _choose(context):
    config = os.path.join(get_package_share_directory('spot_micro_camera'), 'config')
    wanted = LaunchConfiguration('camera_type').perform(context).strip().lower()
    device = LaunchConfiguration('video_device').perform(context).strip()

    if wanted == 'auto':
        cam = detect_camera()
        if cam.kind is None:
            return [LogInfo(msg='[camera] No RealSense or USB webcam found; not starting a camera '
                                'driver. Plug one in and relaunch.')]
    elif wanted == REALSENSE:
        cam = detect_camera()
        if cam.kind != REALSENSE:
            cam = Camera(REALSENSE, 'RealSense (not detected yet)')
    elif wanted == USB:
        cam = find_usb_webcam() or Camera(USB, 'USB camera', '/dev/video0')
    else:
        raise ValueError(f"camera_type must be auto, realsense or usb, not '{wanted}'")

    if cam.kind == REALSENSE:
        # wait_for_device_timeout -1 in camera_params.yaml: a forced realsense
        # that isn't plugged in yet waits for it rather than failing.
        return [LogInfo(msg=f'[camera] Using {cam.model} (depth + color).'), _realsense(config)]
    # usb_cam can't follow the relative by-id symlink (it opens "/dev/../../videoN"),
    # so hand it the node that link points at right now.
    device = os.path.realpath(device or cam.device)
    return [LogInfo(msg=f'[camera] Using USB webcam {cam.model} at {device} (color only, no depth).'),
            _usb(config, device)]


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'camera_type', default_value='auto',
            description='auto (detect what is plugged in; RealSense preferred), realsense or usb'),

        DeclareLaunchArgument(
            'video_device', default_value='',
            description='USB webcam only: V4L2 device to open. Empty uses the detected one '
                        '(its stable /dev/v4l/by-id/... path).'),

        DeclareLaunchArgument(
            'serial_no',
            default_value='',
            description='RealSense only: serial number to open (see rs-enumerate-devices -s). '
                        'Empty picks the first RealSense found, which is fine with one camera.'),

        DeclareLaunchArgument(
            'pointcloud',
            default_value='false',
            description='RealSense only: publish a colored point cloud at '
                        '/spotmicro_cam/d405/depth/color/points'),

        DeclareLaunchArgument(
            'colorize_depth',
            default_value='false',
            description='RealSense only, for viewing: replace the raw 16-bit depth on '
                        '/spotmicro_cam/d405/depth/image_rect_raw with a color-mapped rgb8 '
                        'image, which is easier to look at in a viewer like Foxglove than raw 16-bit '
                        'depth. Colorizing throws away the metric millimeter values, so leave '
                        'this off for anything that consumes depth.'),

        OpaqueFunction(function=_choose),
    ])
