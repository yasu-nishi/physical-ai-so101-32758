"""Viewpoint geometry and offline draft checks, not real-camera accuracy claims."""
import io
import json
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image
import pytest

from so101_mcp.calibration_review import inspect_height
from so101_mcp.camera_views import look_at_rotation
from so101_mcp.config import ROOT, load_config
from so101_mcp.geometry import pixel_to_plane
from so101_mcp.metrology import measure_vertical
from so101_mcp.rgb_calibration import load_calibration, load_verified_calibration, rectify_png
from so101_mcp.sim import SimBackend


def projected_fixture(eye):
    k = np.array([[720., 0, 479.5], [0, 730., 359.5], [0, 0, 1.]])
    d = np.array([.04, -.01, .001, -.001, 0.])
    rotation = look_at_rotation(eye, [.23, -.04, .025])
    cv_rotation = np.diag([1., -1., -1.]) @ rotation.T
    rvec, _ = cv2.Rodrigues(cv_rotation)
    data = {'backend': 'real', 'mount': 'fixed', 'verified': False,
            'image_width': 960, 'image_height': 720, 'camera_matrix': k.tolist(),
            'distortion_coefficients': d.tolist(), 'camera_position_m': eye,
            'camera_to_work_rotation': rotation.tolist(), 'work_frame': 'robot_base_table_z0',
            'rms_px': .1, 'pose_rms_px': .1, 'calibration_sha256': 'synthetic-camera-fixture'}
    # Reference points belong only to this software test, never host perception.
    xyz = np.array([[.25, -.06, 0.], [.25, -.06, .05], [.235, -.045, .05]])
    uv, _ = cv2.projectPoints(xyz, rvec, -cv_rotation @ eye, k, d)
    png = io.BytesIO(); Image.new('RGB', (960, 720), '#aaa').save(png, format='PNG')
    _, meta = rectify_png(png.getvalue(), data)
    fx, fy = meta['focal_px_xy']; cx, cy = meta['principal_point_px']
    rect_uv = cv2.undistortPoints(uv, k, d, P=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])).reshape(-1,2)
    return data, png.getvalue(), rect_uv, xyz


@pytest.mark.parametrize('eye', [[.62,-.12,.43], [.48,.47,.43], [.62,-.40,.30]])
def test_different_fixed_cameras_recover_same_independent_reference(eye):
    data, png, uv, xyz = projected_fixture(eye)
    result, preview = inspect_height(png, data, *uv, reference_height_mm=50.)
    assert result['measurement']['estimated_height_m'] == pytest.approx(.05, abs=1e-6)
    assert result['measurement']['estimated_top_center_m'] == pytest.approx(xyz[2], abs=1e-6)
    assert result['independent_comparison']['within_tolerance']
    assert not result['calibration_verified'] and not result['motion_enabled']
    assert result['motor_connections'] == result['motor_goal_writes'] == 0
    assert preview.startswith(b'\x89PNG')
    wrong, _ = inspect_height(png, data, *uv, reference_height_mm=70.)
    assert wrong['measurement'] == result['measurement']
    assert not wrong['independent_comparison']['within_tolerance']


def test_draft_preview_does_not_approve_calibration_and_cli_persists_failed_check(tmp_path):
    data, png, uv, _ = projected_fixture([.62,-.12,.43])
    cal = tmp_path/'draft.json'; cal.write_text(json.dumps(data))
    image = tmp_path/'raw.png'; image.write_bytes(png)
    before = cal.read_bytes()
    assert load_calibration(cal, require_verified=False)['verified'] is False
    with pytest.raises(ValueError, match='verified'):
        load_verified_calibration(cal)
    out = tmp_path/'result.json'
    args = [sys.executable, str(ROOT/'scripts/calibrate_rgb.py'), 'height', '--calibration', str(cal),
            '--image', str(image), '--bottom', *map(str, uv[0]), '--top', *map(str, uv[1]),
            '--center', *map(str, uv[2]), '--reference-height-mm', '70', '--output', str(out)]
    process = subprocess.run(args, capture_output=True, text=True)
    assert process.returncode == 2, process.stderr
    result = json.loads(out.read_text())
    assert result['measurement_status'] == 'estimated'
    assert not result['independent_comparison']['within_tolerance']
    assert out.with_suffix('.preview.png').exists()
    assert cal.read_bytes() == before


def test_sim_override_updates_render_and_calibration_without_moving_scene(tmp_path):
    reference = load_config('sim', ROOT/'config/physical_ai/sim_staged.json')
    changed = load_config('sim', ROOT/'config/physical_ai/camera_views/side.json')
    a, b = (SimBackend(c, tmp_path, seed=402, record=False) for c in (reference, changed))
    try:
        a.connect(); b.connect()
        # Physics identical; only rendering and camera metadata change.
        assert a.data.qpos == pytest.approx(b.data.qpos, abs=1e-12)
        assert b.camera_calibration()['camera_position_m'] == pytest.approx([.48,.47,.43])
        assert a.capture() != b.capture()
        c = b.camera_calibration()
        xyz = [.24,-.08,.03]
        # Independent OpenCV projection, not SimBackend._project.
        rotation = np.diag([1.,-1.,-1.]) @ np.asarray(c['camera_to_work_rotation']).T
        rvec,_ = cv2.Rodrigues(rotation)
        f=c['focal_px']; u,v=c['principal_point_px']
        uv,_ = cv2.projectPoints(np.array([xyz]),rvec,-rotation@np.array(c['camera_position_m']),
                                np.array([[f,0,u],[0,f,v],[0,0,1.]]),None)
        assert pixel_to_plane(c,*uv.reshape(2),.03) == pytest.approx(xyz,abs=1e-8)
    finally:
        a.close(); b.close()


def test_invalid_camera_geometry_and_unobservable_edge_fail():
    with pytest.raises(ValueError, match='parallel'):
        look_at_rotation([0,0,.6],[0,0,0])
    with pytest.raises(ValueError, match='finite'):
        look_at_rotation([float('nan'),0,0],[1,0,0])
    data,png,uv,_ = projected_fixture([.62,-.12,.43])
    result,_ = inspect_height(png,data,uv[0],uv[0]+[1,1])
    assert result['measurement_status']=='rejected' and 'short' in result['reason']

