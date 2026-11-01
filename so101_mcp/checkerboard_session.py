"""Camera-only checkerboard capture and draft fitting; never enables hardware."""
import hashlib
import io
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from .rgb_calibration import board_points, calibrate_intrinsics, validate_intrinsics


def resume_capture(directory, camera_index, dimensions):
    """Reopen our private session without replacing any image or calibration."""
    directory = Path(directory)
    session = json.loads((directory/'session.json').read_text())
    if (session.get('camera_index') != camera_index or session.get('camera_dimensions') != dimensions
            or session.get('robot_connections') != 0 or session.get('motor_writes') != 0):
        raise ValueError('撮影時と同じカメラ・解像度のセッションを指定してください。')
    records = [json.loads(p.read_text()) for p in sorted((directory/'views').glob('view_*.json'))]
    for i, record in enumerate(records, 1):
        name = f'view_{i:03}.png'
        if (record.get('image') != name or record.get('measured') is not True
                or record.get('square_mm') != session.get('square_mm')
                or record.get('image_size') != [dimensions['width'], dimensions['height']]
                or hashlib.sha256((directory/'views'/name).read_bytes()).hexdigest() != record['image_sha256']):
            raise ValueError('撮影データの連番・寸法・画像ハッシュを確認してください。')
    if session.get('views') != len(records):
        raise ValueError('保存枚数がセッション記録と一致しません。')
    return session


def corner_distance(a, b):
    """Ignore the detector's ambiguous 180 degree board ordering."""
    a, b = np.asarray(a), np.asarray(b)
    return min(float(np.sqrt(np.mean(np.sum((a - p)**2, axis=1)))) for p in (b, b[::-1]))


def save_capture(directory, rgb, corners, square_mm, *, measured=False):
    if measured is not True:
        raise ValueError('定規で印刷寸法を確認してからチェックを入れてください。')
    if not np.isfinite(square_mm) or not 5 <= square_mm <= 50:
        raise ValueError('1マスの実測寸法を5〜50mmの範囲で入力してください。')
    uv = np.asarray(corners, dtype=float)
    h, w = rgb.shape[:2]
    if uv.shape != (54, 2) or not np.all(np.isfinite(uv)):
        raise ValueError('9×6の全交点が見えるようにしてください。')
    if np.any(uv < 5) or np.any(uv > [w-6, h-6]):
        raise ValueError('模様全体と周囲の白い余白が画面に入るようにしてください。')
    if cv2.contourArea(cv2.convexHull(uv.astype(np.float32))) < w*h*.025:
        raise ValueError('54点は検出できましたが、画面内の格子が保存条件を満たす大きさに届いていません。カメラは固定したまま、シートを少し近づけてください。')
    directory = Path(directory)
    records = [json.loads(p.read_text()) for p in sorted(directory.glob('view_*.json'))]
    buffer = io.BytesIO(); Image.fromarray(rgb).save(buffer, format='PNG')
    png = buffer.getvalue(); digest = hashlib.sha256(png).hexdigest()
    for record in records:
        if record['square_mm'] != square_mm or record['image_size'] != [w, h]:
            raise ValueError('寸法・解像度が途中で変わりました。新しい撮影セッションが必要です。')
        if record['image_sha256'] == digest or corner_distance(uv, record['corners_px']) < 7:
            raise ValueError('似た姿勢は保存済みです。位置・傾き・距離を変えてください。')
    number = len(records)+1
    directory.mkdir(parents=True, exist_ok=True)
    stem = f'view_{number:03}'
    result = {'image': stem+'.png', 'image_sha256': digest, 'image_size': [w, h],
              'corners_px': uv.tolist(), 'square_mm': square_mm, 'measured': True}
    with (directory/(stem+'.png')).open('xb') as f: f.write(png)
    with (directory/(stem+'.json')).open('x') as f: json.dump(result, f, indent=2)
    return result


def fit_session(directory, *, review_file=None):
    directory = Path(directory)
    records = [json.loads(p.read_text()) for p in sorted(directory.glob('view_*.json'))]
    if len(records) < 15:
        raise ValueError('異なる姿勢で最低15枚（目安20〜25枚）撮影してください。')
    for record in records:
        filename = record['image']
        if Path(filename).name != filename or not filename.endswith('.png'):
            raise ValueError('Invalid capture filename')
        if hashlib.sha256((directory/filename).read_bytes()).hexdigest() != record['image_sha256']:
            raise ValueError('保存した画像のハッシュが一致しません。')
        if (record['measured'] is not True or record['square_mm'] != records[0]['square_mm']
                or record['image_size'] != records[0]['image_size']):
            raise ValueError('同じ実測寸法・解像度の撮影だけを使用してください。')
    total_captures = len(records)
    exclusions = []
    if review_file is not None:
        exclusions = json.loads(Path(review_file).read_text())['exclusions']
        by_name = {r['image']: r for r in records}
        excluded_names = set()
        for entry in exclusions:
            name = entry['image']
            if (name not in by_name or name in excluded_names
                    or entry['image_sha256'] != by_name[name]['image_sha256']
                    or not isinstance(entry.get('reason'), str) or not entry['reason'].strip()):
                raise ValueError('除外記録には元画像の名前・ハッシュ・具体的な品質上の理由が必要です。')
            excluded_names.add(name)
        records = [r for r in records if r['image'] not in excluded_names]
        if len(records) < 15:
            raise ValueError('品質確認後の画像が最低15枚必要です。')
    points = [np.asarray(r['corners_px'], np.float32) for r in records]
    for i, uv in enumerate(points):
        if uv.shape != (54, 2) or not np.all(np.isfinite(uv)):
            raise ValueError('Invalid stored checkerboard corners')
        if any(corner_distance(uv, earlier) < 7 for earlier in points[:i]):
            raise ValueError('重複した姿勢があります。')
    xyz = board_points(9, 6, records[0]['square_mm']/1000)
    train = [i for i in range(len(records)) if i % 5 != 4]
    heldout = [i for i in range(len(records)) if i % 5 == 4]
    result = calibrate_intrinsics([xyz]*len(train), [points[i] for i in train], tuple(records[0]['image_size']))
    k, d = validate_intrinsics(result)
    normals, heldout_errors = [], []
    for i, uv in enumerate(points):
        ok, rvec, tvec = cv2.solvePnP(xyz, uv, k, d)
        if not ok: raise ValueError('校正板の姿勢を求められませんでした。')
        rotation, _ = cv2.Rodrigues(rvec)
        if np.any((xyz @ rotation.T + tvec.ravel())[:, 2] <= 0):
            raise ValueError('校正板の姿勢が不正です。')
        normal = rotation[:, 2]
        if normal[2] < 0: normal = -normal
        if i in train: normals.append(normal)
        if i in heldout:
            projected, _ = cv2.projectPoints(xyz, rvec, tvec, k, d)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-uv)**2, axis=1))))
            if not np.isfinite(error) or error > 1.5:
                raise ValueError('計算に使わなかった画像で誤差が大きいため、鮮明な画像を撮り直してください。')
            heldout_errors.append(error)
    normals = np.asarray(normals)
    tilt = np.degrees(np.arctan2(normals[:, :2], normals[:, 2:3]))
    spread = np.ptp(tilt, axis=0)
    if np.any(spread < 15):
        raise ValueError('上下・左右の傾きが不足しています。板を両方向に傾けて追加撮影してください。')
    result.update(board={'columns': 9, 'rows': 6, 'square_m': records[0]['square_mm']/1000},
                  verified=False, total_captures=total_captures, selected_captures=len(records), training_indices=train,
                  holdout_indices=heldout, holdout_rms_px=heldout_errors, tilt_spread_deg=spread.tolist(),
                  capture_files=[r['image'] for r in records], excluded_captures=exclusions,
                  capture_sha256=[r['image_sha256'] for r in records],
                  review_required='Draft intrinsics only. Check fixed focus, board flatness, measured scale, '
                                  'work-frame extrinsics and independent distances before robot use.')
    return result
