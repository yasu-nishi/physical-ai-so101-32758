"""Ordinary RGB calibration usable on hardware. No simulator imports or depth."""
import hashlib
import io
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def board_points(columns, rows, square_m):
    if columns < 3 or rows < 3 or not np.isfinite(square_m) or square_m <= 0:
        raise ValueError('Use at least 3 x 3 inner corners and a measured positive square size')
    points = np.zeros((columns*rows, 3), np.float32)
    points[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2)*square_m
    return points


def find_board(rgb, columns, rows):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    found, points = cv2.findChessboardCornersSB(gray, (columns, rows),
                                              cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not found:
        raise ValueError('Full checkerboard not found; keep all inner corners visible')
    return points.reshape(-1, 2)


def calibrate_intrinsics(object_points, image_points, image_size):
    if len(image_points) < 10 or len(object_points) != len(image_points):
        raise ValueError('At least 10 distinct board views are required')
    centers = np.array([np.mean(p, axis=0) for p in image_points])
    if np.any(np.ptp(centers, axis=0)/image_size < .15):
        raise ValueError('Move the board across both image axes, with varied tilt and distance')
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        [np.asarray(p, np.float32) for p in object_points],
        [np.asarray(p, np.float32).reshape(-1, 1, 2) for p in image_points], image_size, None, None)
    errors = []
    for xyz, uv, r, t in zip(object_points, image_points, rotations, translations, strict=True):
        projected, _ = cv2.projectPoints(xyz, r, t, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-uv)**2, axis=1)))))
    if not np.isfinite(rms) or rms > .7 or max(errors) > 1.5:
        raise ValueError('Calibration reprojection error too large; capture sharper, varied views')
    result = {'image_width': int(image_size[0]), 'image_height': int(image_size[1]),
              'camera_matrix': matrix.tolist(), 'distortion_coefficients': distortion.ravel().tolist(),
              'rms_px': float(rms), 'per_view_rms_px': errors, 'views': len(image_points),
              'source': 'OpenCV calibration from photographed checkerboard'}
    validate_intrinsics(result)
    return result


def validate_intrinsics(data):
    k = np.asarray(data['camera_matrix'], dtype=float)
    d = np.asarray(data['distortion_coefficients'], dtype=float)
    w, h = data['image_width'], data['image_height']
    if (k.shape != (3, 3) or d.ndim != 1 or len(d) not in (4, 5, 8, 12, 14)
            or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or w <= 0 or h <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1]) > 1e-9
            or min(k[0, 0], k[1, 1]) <= 0 or not .5 < k[0, 0]/k[1, 1] < 2
            or not (0 < k[0, 2] < w and 0 < k[1, 2] < h)):
        raise ValueError('Invalid RGB camera intrinsics')
    return k, d


def estimate_camera_pose(intrinsics, world_points, image_points, *, work_frame='robot_base_table_z0'):
    if work_frame not in ('robot_base_table_z0', 'checkerboard'):
        raise ValueError('Specify robot-base or checkerboard coordinates explicitly')
    k, d = validate_intrinsics(intrinsics)
    xyz, uv = np.asarray(world_points, np.float64), np.asarray(image_points, np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or uv.shape != (len(xyz), 2) or len(xyz) < 6:
        raise ValueError('At least six corresponding work points and RGB pixels are required')
    if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(uv)):
        raise ValueError('Calibration correspondences must be finite')
    okay, rvec, tvec = cv2.solvePnP(xyz, uv, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not okay:
        raise ValueError('Camera pose estimation failed')
    rotation_cv, _ = cv2.Rodrigues(rvec)
    if np.any((xyz @ rotation_cv.T+tvec.ravel())[:, 2] <= 0):
        raise ValueError('Calibration points are behind the camera')
    predicted, _ = cv2.projectPoints(xyz, rvec, tvec, k, d)
    rms = float(np.sqrt(np.mean(np.sum((predicted.reshape(-1, 2)-uv)**2, axis=1))))
    if not np.isfinite(rms) or rms > .7:
        raise ValueError('Camera pose reprojection error exceeds 0.7 px')
    return {'camera_position_m': (-rotation_cv.T @ tvec.ravel()).tolist(),
            'camera_to_work_rotation': (rotation_cv.T @ np.diag([1., -1., -1.])).tolist(),
            'pose_rms_px': rms, 'work_frame': work_frame,
            'source': 'measured checkerboard coordinates + RGB solvePnP; no depth measurement'}


def load_calibration(path, require_verified=True):
    data = json.loads(Path(path).read_text())
    if data.get('backend') != 'real' or data.get('mount') != 'fixed' or (require_verified and data.get('verified') is not True):
        raise ValueError('Real RGB calibration requires a locally verified fixed camera')
    validate_intrinsics(data)
    if data.get('work_frame') != 'robot_base_table_z0':
        raise ValueError('Calibrate relative to the robot base and table Z=0')
    for key in ('rms_px', 'pose_rms_px'):
        if not np.isfinite(data[key]) or not 0 <= data[key] <= .7:
            raise ValueError('RGB calibration reprojection quality is insufficient')
    rotation = np.asarray(data['camera_to_work_rotation'])
    position = np.asarray(data['camera_position_m'])
    if (rotation.shape != (3, 3) or position.shape != (3,) or not np.all(np.isfinite(position))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError('Invalid RGB camera work-frame transform')
    data['calibration_sha256'] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    return data


def load_verified_calibration(path):
    return load_calibration(path, require_verified=True)


def rectify_png(png, calibration):
    """Return a rectified RGB image and geometry matching those exact pixels."""
    raw = np.asarray(Image.open(io.BytesIO(png)).convert('RGB'))
    h, w = raw.shape[:2]
    if (w, h) != (calibration['image_width'], calibration['image_height']):
        raise ValueError('Actual camera resolution differs from RGB calibration; recalibrate')
    k, d = validate_intrinsics(calibration)
    rectified_k, roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 0, (w, h))
    rectified = cv2.undistort(raw, k, d, None, rectified_k)
    stream = io.BytesIO()
    Image.fromarray(rectified).save(stream, format='PNG')
    meta = {'width': w, 'height': h, 'pixel_origin': 'top_left_u_right_v_down',
            'pixel_coordinates': 'integer indices denote pixel centres',
            'focal_px_xy': [float(rectified_k[0, 0]), float(rectified_k[1, 1])],
            'principal_point_px': [float(rectified_k[0, 2]), float(rectified_k[1, 2])],
            'camera_position_m': calibration['camera_position_m'],
            'camera_to_work_rotation': calibration['camera_to_work_rotation'],
            'work_frame': calibration['work_frame'], 'distortion': 'rectified RGB',
            'valid_pixel_roi_xywh': [int(v) for v in roi],
            'source': 'real RGB checkerboard calibration; no depth sensor or object pose',
            'calibration_sha256': calibration['calibration_sha256']}
    return stream.getvalue(), meta
