"""Image-space virtual depth sensor helpers. No object IDs or object poses."""
import io
import math

import numpy as np
from PIL import Image, ImageDraw

from .geometry import pixel_to_depth


def depth_preview(depth, display_range=(.35, .8)):
    """Human/AI-visible colour image; exact metre values remain in the depth array."""
    near, far = display_range
    if not 0 < near < far:
        raise ValueError('Invalid depth display range')
    valid = np.isfinite(depth) & (depth > 0)
    t = np.clip((np.where(valid, depth, far)-near)/(far-near), 0, 1)
    # Near = yellow, far = dark blue. Never encode identity or segmentation.
    start, end = np.array([255, 235, 65]), np.array([15, 40, 130])
    rgb = np.asarray(start+(end-start)*t[..., None], dtype=np.uint8)
    rgb[~valid] = 0
    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)
    draw.rectangle([0, 0, image.width, 34], fill='#102033')
    draw.text((12, 10), 'VIRTUAL DEPTH | aligned to RGB | camera-forward distance, metres', fill='white')
    draw.rectangle([0, image.height-44, image.width, image.height], fill='#102033')
    draw.text((12, image.height-34), f'Yellow: {near:.2f} m or nearer | Blue: {far:.2f} m or farther | Black: invalid', fill='white')
    draw.text((12, image.height-18), 'For exact XYZ/height, call sample_depth with observation_id and original RGB pixel.', fill='white')
    stream = io.BytesIO()
    image.save(stream, format='PNG')
    return stream.getvalue()


def sample_depth(depth, calibration, pixel_x, pixel_y):
    if (isinstance(pixel_x, bool) or isinstance(pixel_y, bool)
            or not all(math.isfinite(p) and int(p) == p for p in (pixel_x, pixel_y))):
        raise ValueError('Use integer pixel indices in the original RGB image')
    u, v = int(pixel_x), int(pixel_y)
    h, w = depth.shape
    if (w, h) != (calibration['width'], calibration['height']):
        raise ValueError('Depth and RGB calibration dimensions differ')
    if not (0 <= u < w and 0 <= v < h):
        raise ValueError('Pixel is outside the original observation image')
    d = float(depth[v, u])
    point = pixel_to_depth(calibration, u, v, d)
    origin = np.asarray(calibration['camera_position_m'])
    patch = depth[max(0, v-1):min(h, v+2), max(0, u-1):min(w, u+2)]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    return {'pixel_xy': [u, v], 'depth_m': d,
            'depth_convention': 'positive camera-forward Z, not Euclidean range',
            'range_m': float(np.linalg.norm(np.asarray(point)-origin)),
            'work_position_m': point, 'work_frame': calibration['work_frame'],
            'height_above_table_m': point[2], 'table_plane_z_m': 0.,
            'neighborhood_3x3_depth_range_m': [float(valid.min()), float(valid.max())],
            'source': 'aligned rendered depth pixels + camera calibration',
            'limitation': 'Visible surface only. Height is above the calibrated table, not object size or object identity. Ideal simulation depth has no hardware noise.'}
