import os

from spot_micro_camera import detect


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text + '\n')


def _usb_device(root, name, vendor, pid, **attrs):
    dev = os.path.join(root, 'devices', 'usb1', name)
    _write(os.path.join(dev, 'idVendor'), vendor)
    _write(os.path.join(dev, 'idProduct'), pid)
    for key, value in attrs.items():
        _write(os.path.join(dev, key), value)
    return dev


def _video_node(root, n, parent, index, name='cam'):
    """sys/class/video4linux/videoN with a device -> link into the USB tree."""
    real = os.path.join(parent, f'{os.path.basename(parent)}:1.0', 'video4linux', f'video{n}')
    _write(os.path.join(real, 'index'), str(index))
    _write(os.path.join(real, 'name'), name)
    cls = os.path.join(root, 'class', 'video4linux')
    os.makedirs(cls, exist_ok=True)
    os.symlink(real, os.path.join(cls, f'video{n}'))
    os.symlink(os.path.dirname(os.path.dirname(real)), os.path.join(real, 'device'))


def test_realsense_found_by_product_id(tmp_path):
    root = str(tmp_path)
    usb = os.path.join(root, 'bus')
    os.makedirs(usb)
    dev = _usb_device(root, '2-1', '8086', '0b5b', serial='335122273035')
    os.symlink(dev, os.path.join(usb, '2-1'))
    cam = detect.find_realsense(usb)
    assert cam.kind == detect.REALSENSE and cam.model == 'RealSense D405'
    assert cam.serial == '335122273035' and cam.has_depth


def test_other_intel_devices_are_not_cameras(tmp_path):
    usb = os.path.join(str(tmp_path), 'bus')
    os.makedirs(usb)
    os.symlink(_usb_device(str(tmp_path), '1-1', '8086', '1234'), os.path.join(usb, '1-1'))
    assert detect.find_realsense(usb) is None


def test_usb_webcam_capture_node_and_by_id_link(tmp_path):
    root = str(tmp_path)
    dev = _usb_device(root, '2-1', '291a', '3369', product='Anker PowerConf C200',
                      manufacturer='Anker', serial='ACNV')
    _video_node(root, 0, dev, 0)
    _video_node(root, 1, dev, 1)            # metadata node, must be skipped
    devdir = os.path.join(root, 'dev')
    by_id = os.path.join(devdir, 'v4l', 'by-id')
    os.makedirs(by_id)
    _write(os.path.join(devdir, 'video0'), '')
    os.symlink(os.path.join(devdir, 'video0'), os.path.join(by_id, 'usb-Anker-video-index0'))
    cam = detect.find_usb_webcam(os.path.join(root, 'class', 'video4linux'), devdir, by_id)
    assert cam.kind == detect.USB and not cam.has_depth
    assert cam.model == 'Anker PowerConf C200'
    assert cam.device == os.path.join(by_id, 'usb-Anker-video-index0')


def test_realsense_video_nodes_are_not_a_webcam(tmp_path):
    root = str(tmp_path)
    dev = _usb_device(root, '2-1', '8086', '0b5b')
    _video_node(root, 0, dev, 0)
    assert detect.find_usb_webcam(os.path.join(root, 'class', 'video4linux'),
                                  os.path.join(root, 'dev')) is None


def test_non_usb_video_nodes_are_skipped(tmp_path):
    root = str(tmp_path)
    real = os.path.join(root, 'devices', 'platform', 'isp', 'video4linux', 'video20')
    _write(os.path.join(real, 'index'), '0')
    os.symlink(os.path.dirname(real), os.path.join(real, 'device'))
    cls = os.path.join(root, 'class', 'video4linux')
    os.makedirs(cls)
    os.symlink(real, os.path.join(cls, 'video20'))
    assert detect.find_usb_webcam(cls, os.path.join(root, 'dev')) is None
