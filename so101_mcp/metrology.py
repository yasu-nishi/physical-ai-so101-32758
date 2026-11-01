"""Single RGB view: visible vertical edge + calibrated support plane.

No object dimensions, object state, segmentation or depth buffer are inputs.
The host must identify corresponding upper/lower endpoints of the SAME edge.
"""
import itertools
import math

import numpy as np

from .geometry import camera_ray, pixel_to_plane


def _edge(calibration, bottom_px, top_px, support_z):
    bottom = np.asarray(pixel_to_plane(calibration, *bottom_px, support_z))
    origin, ray = camera_ray(calibration, *top_px)
    horizontal = float(ray[:2] @ ray[:2])
    if horizontal < .04:
        raise ValueError('View is too nearly vertical for single-view height metrology')
    depth = float((bottom[:2]-origin[:2]) @ ray[:2] / horizontal)
    if depth <= 0:
        raise ValueError('Selected edge is behind the camera')
    top = origin+depth*ray
    residual = float(np.linalg.norm(top[:2]-bottom[:2]))
    return bottom, top, residual


def measure_vertical(calibration, bottom_pixel, top_pixel, center_pixel, support_z_m=0., pixel_error_px=2.):
    points = np.asarray([bottom_pixel, top_pixel], dtype=float)
    if points.shape != (2, 2) or not np.isfinite(points).all():
        raise ValueError('Provide bottom and top as finite [u,v] pairs')
    center_px = None if center_pixel is None else np.asarray(center_pixel, dtype=float)
    if center_px is not None and (center_px.shape != (2,) or not np.isfinite(center_px).all()):
        raise ValueError('Top-face centre must be a finite [u,v] pair or null for height only')
    if not math.isfinite(support_z_m) or not 0 <= support_z_m <= .15:
        raise ValueError('Support plane must be calibrated or previously measured, in [0, .15] m')
    if not 0 < pixel_error_px <= 4:
        raise ValueError('Pixel error allowance must be in (0,4] px')
    if np.linalg.norm(points[1]-points[0]) < 6:
        raise ValueError('Visible edge is too short for a reliable height estimate')
    bottom, top, residual = _edge(calibration, *points[:2], support_z_m)
    if residual > .004:
        raise ValueError('Pixels do not agree with one vertical edge within 4 mm')
    height = float(top[2]-support_z_m)
    if not .008 <= height <= .12:
        raise ValueError('Estimated edge height is outside the supported 8..120 mm range')
    heights=[]
    for offsets in itertools.product((-pixel_error_px,pixel_error_px), repeat=4):
        _, t, _ = _edge(calibration, points[0]+offsets[:2], points[1]+offsets[2:], support_z_m)
        heights.append(float(t[2]-support_z_m))
    if max(heights)-min(heights) > .016:
        raise ValueError('Height uncertainty exceeds 16 mm; improve the view or select clearer endpoints')
    center=None if center_px is None else pixel_to_plane(calibration,*center_px,float(top[2]))
    return {'bottom_pixel':points[0].tolist(),'top_pixel':points[1].tolist(),
            'center_pixel':None if center_px is None else center_px.tolist(),
            'support_z_m':support_z_m,'estimated_height_m':height,'estimated_top_z_m':float(top[2]),
            'estimated_top_center_m':center,'height_interval_m':[min(heights),max(heights)],
            'vertical_edge_residual_m':residual,'pixel_error_allowance_px':pixel_error_px,
            'height_source':'RGB vertical-edge geometry; object height was not supplied',
            'depth_sensor_used':False,'assumptions':['upright vertical edge', 'visible corresponding endpoints',
              'calibrated camera and known support plane']+(
                  ['selected centre lies on horizontal top face'] if center_px is not None else []),
            'uncertainty_note':'Pixel allowance only; calibration error, support-plane uncertainty and wrong edge correspondence are not covered'}
