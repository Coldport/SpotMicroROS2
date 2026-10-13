import cv2
import numpy as np
import pytest

from spot_micro_deck import static_map


def write_map(tmp_path, pixels, **extra):
    cv2.imwrite(str(tmp_path / 'm.pgm'), pixels)
    meta = {'image': 'm.pgm', 'mode': 'trinary', 'resolution': 0.05, 'origin': [-1.0, 2.0, 0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196, **extra}
    (tmp_path / 'm.yaml').write_text('\n'.join(f'{k}: {v}' for k, v in meta.items()))
    return str(tmp_path / 'm.yaml')


def test_map_saver_values_and_row_order(tmp_path):
    # map_saver trinary: 0 = wall, 254 = free, 205 = unknown; image row 0 = top (highest y)
    px = np.array([[0, 254, 205],
                   [254, 254, 254]], dtype=np.uint8)
    w, h, res, ox, oy, cells = static_map.load(write_map(tmp_path, px))
    assert (w, h, res, ox, oy) == (3, 2, 0.05, -1.0, 2.0)
    assert cells.tolist() == [[0, 0, 0], [100, 0, -1]]


def test_missing_files_raise(tmp_path):
    with pytest.raises(static_map.MapLoadError):
        static_map.load(str(tmp_path / 'none.yaml'))
    (tmp_path / 'x.yaml').write_text('image: gone.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    with pytest.raises(static_map.MapLoadError):
        static_map.load(str(tmp_path / 'x.yaml'))
