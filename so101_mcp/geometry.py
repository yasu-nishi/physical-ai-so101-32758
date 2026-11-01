"""Calibrated image-ray / known horizontal plane intersection; no object state.

MuJoCo camera coordinates: +X right, +Y up, looking along -Z.
Pixel coordinates: origin at top left; u right, v down. Work frame is metres.
"""
import math

import numpy as np


def camera_ray(calibration, u, v):
    """Unnormalised ray: scale by forward camera-Z depth, not Euclidean range."""
    values = np.asarray([u, v], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('Pixel values must be finite')
    width, height = calibration['width'], calibration['height']
    if not (0 <= u < width and 0 <= v < height):
        raise ValueError('Pixel is outside the original observation image')
    if 'valid_pixel_roi_xywh' in calibration:
        x, y, w, h = calibration['valid_pixel_roi_xywh']
        if not (x <= u < x+w and y <= v < y+h):
            raise ValueError('Pixel is outside the valid rectified RGB region')
    focal = np.asarray(calibration.get('focal_px_xy', [calibration.get('focal_px')]*2), dtype=float)
    rotation = np.asarray(calibration['camera_to_work_rotation'], dtype=float)
    origin = np.asarray(calibration['camera_position_m'], dtype=float)
    if (focal.shape != (2,) or not np.all(np.isfinite(focal)) or np.any(focal <= 0) or rotation.shape != (3, 3)
            or origin.shape != (3,) or not np.all(np.isfinite(rotation))
            or not np.all(np.isfinite(origin))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6)):
        raise ValueError('Invalid camera calibration')
    center = np.asarray(calibration.get('principal_point_px', [width/2, height/2]), dtype=float)
    if center.shape != (2,) or not np.all(np.isfinite(center)):
        raise ValueError('Invalid camera calibration principal point')
    ray = rotation @ np.array([(u-center[0])/focal[0], -(v-center[1])/focal[1], -1.])
    return origin, ray


def pixel_to_plane(calibration, u, v, plane_z_m):
    if not math.isfinite(plane_z_m):
        raise ValueError('Plane value must be finite')
    origin, ray = camera_ray(calibration, u, v)
    if abs(ray[2]) < 1e-8:
        raise ValueError('Camera ray is parallel to the work plane')
    distance = (plane_z_m-origin[2])/ray[2]
    if distance <= 0:
        raise ValueError('Work plane is behind the camera')
    return (origin+distance*ray).tolist()


def pixel_to_depth(calibration, u, v, depth_m):
    """Deproject an aligned metric camera-forward depth pixel into work XYZ."""
    if not math.isfinite(depth_m) or depth_m <= 0:
        raise ValueError('Depth must be a finite positive distance in metres')
    origin, ray = camera_ray(calibration, u, v)
    return (origin+depth_m*ray).tolist()


def locate_top(calibration, pixel_x, pixel_y, settings):
    """User/host-selected TOP FACE CENTER, not the silhouette or table contact."""
    z = settings['object_top_z_m']
    point = pixel_to_plane(calibration, pixel_x, pixel_y, z)
    error_px = settings['pixel_tolerance_px']
    envelope = [pixel_to_plane(calibration, pixel_x+du, pixel_y+dv, z)
                for du in (-error_px, error_px) for dv in (-error_px, error_px)]
    lo = np.min(envelope, axis=0)
    hi = np.max(envelope, axis=0)
    for axis, limits in enumerate(settings['pick_xy_bounds_m']):
        if not limits[0] <= lo[axis] <= hi[axis] <= limits[1]:
            raise ValueError('Selected point / pixel tolerance exceeds the configured pick workspace')
    return {'pixel_xy': [pixel_x, pixel_y], 'work_frame': 'robot_base_table_z0',
            'estimated_top_center_m': point, 'pick_xy_m': point[:2],
            'assumed_top_z_m': z, 'pixel_tolerance_px': error_px,
            'xy_bounds_from_pixel_tolerance_m': [lo[:2].tolist(), hi[:2].tolist()],
            'method': 'host visual pixel selection + calibrated ray / known-height plane',
            'limitation': 'Tolerance is assumed, not a measured vision confidence. Known upright 30 mm cube only.'}
