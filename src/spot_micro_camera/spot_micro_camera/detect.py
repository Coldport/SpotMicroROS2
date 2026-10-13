"""
Which camera is plugged into the robot: the Intel RealSense depth camera (D405)
or a plain USB (UVC) webcam.

Reads sysfs only — no ROS, no driver, no opening of the device — so it is cheap
enough to call from a launch file and from deck_bridge every few seconds, and
it never fights a running driver for the camera.

    from spot_micro_camera.detect import detect_camera
    cam = detect_camera()
    cam.kind      # 'realsense' | 'usb' | None
    cam.model     # e.g. 'RealSense D405', 'Anker PowerConf C200'
    cam.device    # usb: /dev/v4l/by-id/... (or /dev/videoN); realsense: None
    cam.serial    # USB serial number if the device reports one
    cam.has_depth

A RealSense also enumerates as several /dev/videoN nodes, so it is never
reported as a USB webcam. With both plugged in, the RealSense wins (depth).
`python3 -m spot_micro_camera.detect` prints what it finds.
"""

import glob
import os
from dataclasses import dataclass
from typing import Optional

REALSENSE = 'realsense'
USB = 'usb'

INTEL_VENDOR = '8086'
# Product ids of RealSense depth cameras (librealsense's rs_usb list). Anything
# else from Intel's vendor id is not treated as a camera.
REALSENSE_MODELS = {
    '0ad1': 'D400', '0ad2': 'D410', '0ad3': 'D415', '0ad4': 'D430', '0ad5': 'D430',
    '0ad6': 'D400', '0af6': 'D420', '0afe': 'D420', '0aff': 'D410', '0b00': 'D400',
    '0b01': 'D430', '0b03': 'D460', '0b07': 'D435', '0b0c': 'D405U', '0b3a': 'D435i',
    '0b3d': 'D435i', '0b48': 'D430', '0b49': 'D416', '0b4b': 'D430', '0b4d': 'D465',
    '0b52': 'D416', '0b5b': 'D405', '0b5c': 'D455', '0b64': 'L515', '0b68': 'L535',
    '0b6a': 'D456', '0b6b': 'D401', '0abe': 'D457', '0b4f': 'D456', '0b5d': 'D401',
}

_SYS_USB = '/sys/bus/usb/devices'
_SYS_V4L = '/sys/class/video4linux'
_BY_ID = '/dev/v4l/by-id'


@dataclass(frozen=True)
class Camera:
    kind: Optional[str] = None
    model: str = ''
    device: Optional[str] = None
    serial: str = ''

    @property
    def has_depth(self):
        return self.kind == REALSENSE

    def as_dict(self):
        return {'type': self.kind, 'model': self.model, 'device': self.device,
                'serial': self.serial, 'depth': self.has_depth}


def _read(path):
    try:
        with open(path, encoding='utf-8') as f:
            return f.read().strip()
    except OSError:
        return ''


def _usb_parent(path):
    """Nearest ancestor sysfs directory that is a USB device (has idVendor)."""
    path = os.path.realpath(path)
    while path not in ('/', ''):
        if os.path.isfile(os.path.join(path, 'idVendor')):
            return path
        path = os.path.dirname(path)
    return None


def _usb_name(dev_dir):
    product = _read(os.path.join(dev_dir, 'product'))
    maker = _read(os.path.join(dev_dir, 'manufacturer'))
    if product and maker and not product.lower().startswith(maker.lower()):
        return f'{maker} {product}'
    return product or maker


def find_realsense(sys_usb=_SYS_USB):
    for dev in sorted(glob.glob(os.path.join(sys_usb, '*'))):
        if _read(os.path.join(dev, 'idVendor')).lower() != INTEL_VENDOR:
            continue
        pid = _read(os.path.join(dev, 'idProduct')).lower()
        if pid in REALSENSE_MODELS:
            return Camera(REALSENSE, f'RealSense {REALSENSE_MODELS[pid]}', None,
                          _read(os.path.join(dev, 'serial')))
    return None


def _by_id_link(video_node, by_id=_BY_ID):
    """Stable /dev/v4l/by-id/... symlink for /dev/videoN, if udev made one."""
    target = os.path.realpath(video_node)
    for link in sorted(glob.glob(os.path.join(by_id, '*'))):
        if os.path.realpath(link) == target:
            return link
    return None


def find_usb_webcam(sys_v4l=_SYS_V4L, dev_dir='/dev', by_id=_BY_ID):
    """First USB video-capture node that isn't part of a RealSense.

    A UVC camera exposes two nodes; index 0 is the capture one, the other only
    carries metadata. Non-USB nodes (the Pi's ISP and codec) are skipped.
    """
    nodes = glob.glob(os.path.join(sys_v4l, 'video*'))
    for node in sorted(nodes, key=lambda n: int(os.path.basename(n)[5:] or 0)):
        if _read(os.path.join(node, 'index')) not in ('', '0'):
            continue
        usb = _usb_parent(os.path.join(node, 'device'))
        if usb is None or _read(os.path.join(usb, 'idVendor')).lower() == INTEL_VENDOR:
            continue
        dev = os.path.join(dev_dir, os.path.basename(node))
        name = _usb_name(usb) or _read(os.path.join(node, 'name')).split(':')[0]
        return Camera(USB, name or 'USB camera', _by_id_link(dev, by_id) or dev,
                      _read(os.path.join(usb, 'serial')))
    return None


def detect_camera():
    """-> Camera; kind is None when no camera is plugged in."""
    return find_realsense() or find_usb_webcam() or Camera()


def main():
    cam = detect_camera()
    if cam.kind is None:
        print('No camera found (no RealSense, no USB webcam).')
    else:
        print(f'{cam.kind}: {cam.model}' + (f' at {cam.device}' if cam.device else '')
              + (f' (serial {cam.serial})' if cam.serial else ''))


if __name__ == '__main__':
    main()
