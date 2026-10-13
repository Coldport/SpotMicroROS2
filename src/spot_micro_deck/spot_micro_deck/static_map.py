"""
Reads a map saved by nav2's map_saver (what slam_toolbox's save_map writes): a
YAML file next to a .pgm image. No ROS imports, so it can be unit-tested.

The image is turned back into OccupancyGrid values the way nav2's map_server
does in its default "trinary" mode: occupancy p = (255 - pixel) / 255 (or
pixel / 255 if negate), p > occupied_thresh -> 100, p < free_thresh -> 0,
anything between -> -1 (unknown). Rows are flipped, so row 0 is the lowest y.
"""

import os

import cv2
import numpy as np
import yaml


class MapLoadError(Exception):
    pass


def load(yaml_path):
    """-> (width, height, resolution, origin_x, origin_y, cells int8 array, row 0 = lowest y)"""
    try:
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image = meta['image']
        resolution = float(meta['resolution'])
        origin = [float(v) for v in meta['origin']]
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as exc:
        raise MapLoadError(f'{yaml_path}: {exc}') from exc
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    pixels = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    if pixels is None:
        raise MapLoadError(f'cannot read map image {image}')
    p = pixels.astype(np.float32) / 255.0
    if not int(meta.get('negate', 0)):
        p = 1.0 - p
    cells = np.full(p.shape, -1, dtype=np.int8)
    cells[p > float(meta.get('occupied_thresh', 0.65))] = 100
    cells[p < float(meta.get('free_thresh', 0.25))] = 0
    cells = np.ascontiguousarray(cells[::-1])
    h, w = cells.shape
    return w, h, resolution, origin[0], origin[1], cells
