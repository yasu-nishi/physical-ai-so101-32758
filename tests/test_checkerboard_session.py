"""Independent camera projections and capture provenance; no hardware access."""
import hashlib
import json

import cv2
import numpy as np
import pytest

from so101_mcp.checkerboard_session import fit_session, resume_capture, save_capture
from so101_mcp.rgb_calibration import board_points, find_board


def projected_views(tmp_path, *, flat=False):
    rng = np.random.default_rng(42)
    k = np.array([[600., 0, 319.5], [0, 610., 239.5], [0, 0, 1.]])
    d = np.array([.03, -.01, .0005, -.0007, 0.])
    xyz = board_points(9, 6, .02)
    for attempt in range(150):
        rv = np.zeros(3) if flat else rng.uniform(-.5, .5, 3)
        tv = np.array([rng.uniform(-.17, .04), rng.uniform(-.13, .07), rng.uniform(.45, .75)])
        uv, _ = cv2.projectPoints(xyz, rv, tv, k, d)
        uv = uv.reshape(-1, 2) + rng.normal(0, .06, (54, 2))
        if not np.all(uv > 8) or not np.all(uv < [632, 472]): continue
        # Colour encodes the fixture index so each evidence hash is distinct.
        rgb = np.full((480, 640, 3), attempt, np.uint8)
        try: save_capture(tmp_path, rgb, uv, 20., measured=True)
        except ValueError: continue
        if len(list(tmp_path.glob('view_*.json'))) == 25: break
    assert len(list(tmp_path.glob('view_*.json'))) == 25
    return k


def test_checkerboard_is_detected_in_image_but_not_in_blank():
    rgb = np.full((480, 640, 3), 255, np.uint8)
    for y in range(7):
        for x in range(10):
            if (x+y) % 2 == 0: rgb[120+y*25:145+y*25, 180+x*25:205+x*25] = 0
    corners = find_board(rgb, 9, 6)
    assert corners.shape == (54, 2)
    assert np.allclose(np.linalg.norm(np.diff(corners.reshape(6, 9, 2), axis=1), axis=2), 25, atol=.4)
    with pytest.raises(ValueError): find_board(np.full_like(rgb, 255), 9, 6)


def test_capture_requires_measurement_preserves_png_and_rejects_reversed_duplicate(tmp_path):
    uv = board_points(9, 6, 20.)[:, :2] + [200, 150]
    rgb = np.full((480, 640, 3), 128, np.uint8)
    with pytest.raises(ValueError, match='定規'):
        save_capture(tmp_path, rgb, uv, 20.)
    record = save_capture(tmp_path, rgb, uv, 20., measured=True)
    assert hashlib.sha256((tmp_path/record['image']).read_bytes()).hexdigest() == record['image_sha256']
    with pytest.raises(ValueError, match='似た姿勢'):
        save_capture(tmp_path, rgb+1, uv[::-1], 20., measured=True)
    with pytest.raises(ValueError, match='寸法・解像度'):
        save_capture(tmp_path, rgb+2, uv+[50, 20], 19., measured=True)
    with pytest.raises(ValueError, match='余白'):
        save_capture(tmp_path, rgb+3, uv+[500, 0], 20., measured=True)
    assert len(list(tmp_path.glob('*.png'))) == 1


def test_fit_recovers_noisy_independent_camera_with_held_out_views(tmp_path):
    expected = projected_views(tmp_path)
    result = fit_session(tmp_path)
    fitted = np.asarray(result['camera_matrix'])
    assert fitted[:2, :2] == pytest.approx(expected[:2, :2], abs=3)
    assert fitted[:2, 2] == pytest.approx(expected[:2, 2], abs=3)
    assert set(result['training_indices']).isdisjoint(result['holdout_indices'])
    assert len(result['holdout_indices']) == 5
    assert max(result['holdout_rms_px']) < .2
    assert result['verified'] is False
    assert min(result['tilt_spread_deg']) >= 15
    assert result['board']['square_m'] == .02


def test_too_few_flat_only_and_modified_evidence_are_rejected(tmp_path):
    with pytest.raises(ValueError, match='最低15枚'): fit_session(tmp_path)
    flat = tmp_path/'flat'; flat.mkdir()
    projected_views(flat, flat=True)
    with pytest.raises(ValueError): fit_session(flat)
    varied = tmp_path/'varied'; varied.mkdir()
    projected_views(varied)
    (varied/'view_001.png').write_bytes(b'changed')
    with pytest.raises(ValueError, match='ハッシュ'): fit_session(varied)


def test_heldout_errors_cannot_be_hidden_by_fitting_training_views(tmp_path):
    projected_views(tmp_path)
    path = tmp_path/'view_005.json'
    data = json.loads(path.read_text())
    corners = np.asarray(data['corners_px']); corners[20] += [25, -30]
    data['corners_px'] = corners.tolist(); path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='計算に使わなかった画像'): fit_session(tmp_path)


def test_resume_preserves_evidence_and_rejects_changed_camera_or_files(tmp_path):
    directory = tmp_path/'views'; directory.mkdir()
    projected_views(directory)
    dimensions = {'width': 640, 'height': 480, 'fps': 30}
    session = {'camera_index': 2, 'camera_dimensions': dimensions, 'square_mm': 20.,
               'views': 25, 'robot_connections': 0, 'motor_writes': 0, 'verified': False}
    (tmp_path/'session.json').write_text(json.dumps(session))
    before = {p.name: p.read_bytes() for p in directory.iterdir()}
    assert resume_capture(tmp_path, 2, dimensions) == session
    assert before == {p.name: p.read_bytes() for p in directory.iterdir()}
    with pytest.raises(ValueError, match='同じカメラ'): resume_capture(tmp_path, 1, dimensions)
    (directory/'view_001.png').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='ハッシュ'): resume_capture(tmp_path, 2, dimensions)


def test_explicit_quality_review_is_hash_bound_and_preserves_all_images(tmp_path):
    projected_views(tmp_path)
    path = tmp_path/'view_005.json'
    data = json.loads(path.read_text())
    corners = np.asarray(data['corners_px']); corners[20] += [25, -30]
    data['corners_px'] = corners.tolist(); path.write_text(json.dumps(data))
    with pytest.raises(ValueError): fit_session(tmp_path)
    review = {'exclusions': [{'image': data['image'], 'image_sha256': data['image_sha256'],
                              'reason': 'Synthetic nonplanar correspondence fixture; original image retained.'}]}
    review_path = tmp_path/'capture_review.json'; review_path.write_text(json.dumps(review))
    result = fit_session(tmp_path, review_file=review_path)
    assert result['total_captures'] == 25 and result['selected_captures'] == 24
    assert result['excluded_captures'] == review['exclusions']
    assert data['image'] not in result['capture_files']
    assert (tmp_path/data['image']).exists() and len(list(tmp_path.glob('*.png'))) == 25
    review['exclusions'][0]['image_sha256'] = 'incorrect'
    review_path.write_text(json.dumps(review))
    with pytest.raises(ValueError, match='除外記録'): fit_session(tmp_path, review_file=review_path)
