"""Geometry tests with independent projected data; real camera remains unverified."""
import io
import json

import cv2
import numpy as np
from PIL import Image
import pytest

from so101_mcp.config import load_config
from so101_mcp.controller import Controller
from so101_mcp.geometry import pixel_to_plane
from so101_mcp.joints import ARM, Pose
from so101_mcp.real import RealBackend
from so101_mcp.rgb_calibration import (board_points, calibrate_intrinsics, estimate_camera_pose,
                                      load_verified_calibration, rectify_png)


def intrinsics():
    return {'image_width': 640, 'image_height': 480,
            'camera_matrix': [[600., 0, 319.5], [0, 610., 239.5], [0, 0, 1]],
            'distortion_coefficients': [.08, -.03, .001, -.002, .01], 'rms_px': .1}


def projected_board():
    data = intrinsics()
    xyz = board_points(9, 6, .02).astype(float)+[.1, -.1, 0]
    rotation = np.diag([1., -1., -1.])
    rvec, _ = cv2.Rodrigues(rotation)
    origin = np.array([.2, -.05, .6])
    tvec = -rotation @ origin
    uv, _ = cv2.projectPoints(xyz, rvec, tvec, np.asarray(data['camera_matrix']),
                             np.asarray(data['distortion_coefficients']))
    return data, xyz, uv.reshape(-1,2), origin


def test_rgb_calibration_recovers_camera_pose_and_rectified_plane_point():
    data, xyz, uv, expected_origin = projected_board()
    pose = estimate_camera_pose(data, xyz, uv)
    assert pose['camera_position_m'] == pytest.approx(expected_origin, abs=1e-6)
    assert np.asarray(pose['camera_to_work_rotation']) == pytest.approx(np.eye(3), abs=1e-6)
    raw = io.BytesIO(); Image.new('RGB', (640,480)).save(raw, format='PNG')
    png, cal = rectify_png(raw.getvalue(), {**data, **pose, 'calibration_sha256': 'test'})
    assert Image.open(io.BytesIO(png)).size == (640,480)
    # Project a known 30 mm plane point into the REAL pinhole model, undistort
    # with the same returned image matrix, then reconstruct from only that pixel.
    point = np.array([[.24, -.04, .03]])
    rvec, _ = cv2.Rodrigues(np.diag([1., -1., -1.]))
    uv, _ = cv2.projectPoints(point, rvec, -np.diag([1.,-1.,-1.]) @ expected_origin,
                             np.asarray(data['camera_matrix']), np.asarray(data['distortion_coefficients']))
    fx, fy = cal['focal_px_xy']; cx, cy = cal['principal_point_px']
    k_new = np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    rect_uv = cv2.undistortPoints(uv, np.asarray(data['camera_matrix']),
                                np.asarray(data['distortion_coefficients']), P=k_new).reshape(2)
    assert pixel_to_plane(cal, *rect_uv, .03) == pytest.approx(point[0], abs=1e-6)


def test_intrinsics_need_varied_views_and_recover_independent_parameters():
    data = intrinsics()
    k, d = np.asarray(data['camera_matrix']), np.asarray(data['distortion_coefficients'])
    xyz = board_points(9,6,.02)
    objects, images = [], []
    rng = np.random.default_rng(18)
    for _ in range(40):
        rv = rng.uniform(-.35,.35,3)
        tv = [rng.uniform(-.16,.02),rng.uniform(-.12,.04),rng.uniform(.45,.75)]
        uv,_ = cv2.projectPoints(xyz,rv,np.array(tv),k,d)
        uv=uv.reshape(-1,2)
        if np.all(uv > 5) and np.all(uv < [635,475]):
            objects.append(xyz); images.append(uv)
    result = calibrate_intrinsics(objects,images,(640,480))
    assert result['rms_px'] < .001
    assert np.asarray(result['camera_matrix']) == pytest.approx(k,abs=.02)
    with pytest.raises(ValueError,match='both image axes'):
        calibrate_intrinsics([xyz]*10,[images[0]]*10,(640,480))
    with pytest.raises(ValueError,match='10 distinct'):
        calibrate_intrinsics(objects[:3],images[:3],(640,480))


def test_bad_size_unverified_calibration_and_invalid_pixels_rejected(tmp_path):
    data,xyz,uv,_ = projected_board()
    complete={**data,**estimate_camera_pose(data,xyz,uv),'backend':'real','mount':'fixed','verified':False}
    path=tmp_path/'camera.local.json'; path.write_text(json.dumps(complete))
    with pytest.raises(ValueError,match='verified'):
        load_verified_calibration(path)
    complete['verified']=True; path.write_text(json.dumps(complete))
    cal=load_verified_calibration(path)
    raw=io.BytesIO(); Image.new('RGB',(320,240)).save(raw,format='PNG')
    with pytest.raises(ValueError,match='resolution'):
        rectify_png(raw.getvalue(),cal)
    raw=io.BytesIO(); Image.new('RGB',(640,480)).save(raw,format='PNG')
    _,meta=rectify_png(raw.getvalue(),cal)
    meta['valid_pixel_roi_xywh']=[10,10,600,440]
    with pytest.raises(ValueError,match='rectified'):
        pixel_to_plane(meta,5,5,.03)


def test_board_reference_cannot_be_used_as_robot_registration(tmp_path):
    data, xyz, uv, origin = projected_board()
    pose = estimate_camera_pose(data, xyz, uv, work_frame='checkerboard')
    assert pose['camera_position_m'] == pytest.approx(origin, abs=1e-6)
    assert pose['work_frame'] == 'checkerboard'
    path = tmp_path/'reference.local.json'
    path.write_text(json.dumps({**data, **pose, 'backend': 'real', 'mount': 'fixed', 'verified': True}))
    with pytest.raises(ValueError, match='robot base'):
        load_verified_calibration(path)
    with pytest.raises(ValueError, match='explicitly'):
        estimate_camera_pose(data, xyz, uv, work_frame='unregistered')


def test_real_rgb_observation_and_common_localization_use_no_depth_or_motor_write(tmp_path):
    data,xyz,uv,_ = projected_board()
    config=load_config('real')
    real=RealBackend(config,tmp_path)
    real.rgb_calibration={**data,**estimate_camera_pose(data,xyz,uv),'calibration_sha256':'software-fixture'}
    raw=io.BytesIO(); Image.new('RGB',(640,480),(160,160,160)).save(raw,format='PNG')
    real.capture=lambda: raw.getvalue()
    real.connect=lambda: None
    real.state=lambda: Pose(dict.fromkeys(ARM,0.),.5)
    # RealBackend.capture_bundle and Controller execute unchanged; no sim class.
    control=Controller(real,config,tmp_path)
    try:
        meta,images=control.observe_bundle()
        assert len(images)==1
        assert not meta['capabilities']['depth_sampling']
        assert meta['capabilities']['known_plane_localization']
        assert not meta['sensors']['end_effector']['available']
        result=control.locate_on_plane(meta['observation_id'],350,250,.03)
        assert result['height_measured'] is False
        assert result['assumed_plane_z_m']==.03
        assert result['work_position_m'][2]==pytest.approx(.03)
        assert real.writes==0
        with pytest.raises(ValueError,match='unavailable'):
            control.sample_depth(meta['observation_id'],350,250)
    finally:
        control.close()
