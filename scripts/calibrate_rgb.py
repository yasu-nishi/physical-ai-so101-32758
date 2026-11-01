"""Calibrate an ordinary fixed RGB camera. Capture never connects to robot motors."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_mcp.config import ROOT, load_config, real_camera_dimensions
from so101_mcp.rgb_calibration import board_points, calibrate_intrinsics, estimate_camera_pose, find_board, load_calibration, rectify_png


def save_json(path, data):
    if path.exists():
        raise ValueError('Use a new output file; existing calibration is never overwritten')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    board = sub.add_parser('board', help='Print at actual size; measure printed square size')
    board.add_argument('--output', type=Path, default=Path('output/pdf/rgb_checkerboard.pdf'))
    capture = sub.add_parser('capture', help='Save one ordinary RGB frame; no robot connection')
    capture.add_argument('--config', type=Path)
    capture.add_argument('--output', type=Path, required=True)
    intrinsic = sub.add_parser('intrinsics')
    intrinsic.add_argument('--images', type=Path, required=True)
    intrinsic.add_argument('--output', type=Path, required=True)
    reference = sub.add_parser('reference', help='Mark the board origin/axes without assuming a robot transform')
    pose = sub.add_parser('pose', help='Fixed camera + board lying on the measured work plane')
    for p in (reference, pose):
        p.add_argument('--intrinsics', type=Path, required=True)
        p.add_argument('--image', type=Path, required=True)
        p.add_argument('--flip-rows', action='store_true', help='Reverse board Y corner order after checking the overlay')
        p.add_argument('--flip-columns', action='store_true', help='Reverse board X corner order after checking the overlay')
        p.add_argument('--output', type=Path, required=True)
    pose.add_argument('--origin-m', type=float, nargs=3, required=True, metavar=('X','Y','Z'))
    pose.add_argument('--yaw-deg', type=float, required=True)
    rectify = sub.add_parser('rectify', help='Preview corrected RGB using a draft calibration; no robot')
    check = sub.add_parser('check', help='Compare a selected rectified pixel with an independently measured point')
    height = sub.add_parser('height', help='Offline unknown-height measurement from rectified RGB endpoints; no robot')
    for p in (rectify, check, height):
        p.add_argument('--calibration', type=Path, required=True)
        p.add_argument('--image', type=Path, required=True, help='Original unrectified RGB image')
        p.add_argument('--output', type=Path, required=True)
    check.add_argument('--pixel', type=float, nargs=2, required=True, metavar=('U','V'))
    check.add_argument('--known-xyz-m', type=float, nargs=3, required=True, metavar=('X','Y','Z'))
    height.add_argument('--bottom', type=float, nargs=2, required=True, metavar=('U','V'))
    height.add_argument('--top', type=float, nargs=2, required=True, metavar=('U','V'))
    height.add_argument('--center', type=float, nargs=2, metavar=('U','V'), help='Optional horizontal top-face centre in the rectified image')
    height.add_argument('--support-z-m', type=float, default=0., help='Independently known support plane, table=0')
    height.add_argument('--pixel-error-px', type=float, default=2.)
    height.add_argument('--reference-height-mm', type=float, help='Optional measured reference; used only AFTER estimating height')
    height.add_argument('--tolerance-mm', type=float, default=3.)
    for p in (board, intrinsic, reference, pose):
        p.add_argument('--columns', type=int, default=9, help='Number of INNER corners along board +X')
        p.add_argument('--rows', type=int, default=6, help='Number of INNER corners along board +Y')
        p.add_argument('--square-mm', type=float, default=20.)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; use a new output file')
    if args.command in ('rectify', 'check', 'height'):
        data = load_calibration(args.calibration, require_verified=False)
        if args.command == 'height':
            from so101_mcp.calibration_review import inspect_height
            preview = args.output.with_suffix('.preview.png')
            if preview.exists():
                parser.error('Preview exists; use a new output file')
            result, annotated = inspect_height(args.image.read_bytes(), data, args.bottom, args.top, args.center,
                args.support_z_m, args.pixel_error_px, args.reference_height_mm, args.tolerance_mm)
            save_json(args.output, result)
            preview.write_bytes(annotated)
            print(json.dumps(result, indent=2))
            if result['measurement_status'] != 'estimated' or result.get('independent_comparison', {}).get('within_tolerance') is False:
                raise SystemExit(2)
            return
        png, meta = rectify_png(args.image.read_bytes(), data)
        if args.command == 'rectify':
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_bytes(png)
        else:
            from so101_mcp.geometry import pixel_to_plane
            expected = np.asarray(args.known_xyz_m)
            if not np.all(np.isfinite(expected)):
                raise ValueError('Independent measured point must be finite')
            point = pixel_to_plane(meta, *args.pixel, expected[2])
            result = {'pixel_xy': args.pixel, 'known_xyz_m': expected.tolist(),
                      'estimated_xyz_m': point, 'xy_error_mm': float(np.linalg.norm(np.array(point[:2])-expected[:2])*1000),
                      'height_measured': False, 'calibration_sha256': data['calibration_sha256'],
                      'note': 'Independent XY check on a supplied known height plane; no robot motion or automatic verification flag'}
            save_json(args.output, result)
            print(json.dumps(result, indent=2))
    elif args.command == 'board':
        from reportlab.pdfgen import canvas
        from reportlab.lib.pagesizes import A4, landscape
        from reportlab.lib.units import mm
        board_points(args.columns, args.rows, args.square_mm/1000)
        w, h = landscape(A4)
        size = args.square_mm*mm
        bw, bh = (args.columns+1)*size, (args.rows+1)*size
        if bw > w-40*mm or bh > h-50*mm:
            parser.error('Board does not fit on A4 with labels')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        c = canvas.Canvas(str(args.output), pagesize=(w,h))
        c.setAuthor(''); c.setTitle('RGB camera checkerboard calibration')
        x0, y0 = (w-bw)/2, (h-bh)/2
        for y in range(args.rows+1):
            for x in range(args.columns+1):
                if (x+y)%2 == 0:
                    c.rect(x0+x*size, y0+y*size, size, size, fill=1, stroke=0)
        c.setFont('Helvetica', 12)
        c.drawString(20*mm, h-15*mm, f'RGB calibration: {args.columns} x {args.rows} inner corners; square {args.square_mm:g} mm')
        c.setFont('Helvetica', 9)
        c.drawString(20*mm, 15*mm, 'Print at 100% / actual size. Measure the printed square. Board coordinates: +X right, +Y up.')
        c.setFillColorRGB(.8,0,0)
        c.drawString(x0, y0-5*mm, 'Origin: first INNER corner above/right of lower-left outer square. Verify corner 0 overlay.')
        c.save()
    elif args.command == 'capture':
        from dotenv import dotenv_values
        import os
        from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
        config = load_config('real', args.config)
        env = {**dotenv_values(ROOT/'.env'), **os.environ}
        index = env.get(config['camera_index_env'])
        if index is None:
            parser.error('Configure the real workspace camera index locally')
        camera = OpenCVCamera(OpenCVCameraConfig(index_or_path=int(index), **real_camera_dimensions(config, env)))
        try:
            camera.connect()
            frame = camera.read()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(frame).save(args.output)
            print(json.dumps({'width': frame.shape[1], 'height': frame.shape[0], 'motor_connections': 0}))
        finally:
            if camera.is_connected:
                camera.disconnect()
    elif args.command == 'intrinsics':
        xyz = board_points(args.columns, args.rows, args.square_mm/1000)
        objects, pixels, hashes = [], [], set()
        shape = None
        for path in sorted(args.images.glob('*.png')):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest in hashes:
                raise ValueError('Duplicate calibration images; capture distinct board poses')
            hashes.add(digest)
            rgb = np.asarray(Image.open(path).convert('RGB'))
            current_shape = (rgb.shape[1], rgb.shape[0])
            if shape is not None and shape != current_shape:
                raise ValueError('All calibration frames must have the same resolution')
            shape = current_shape
            corners = find_board(rgb, args.columns, args.rows)
            objects.append(xyz); pixels.append(corners)
        result = calibrate_intrinsics(objects, pixels, shape)
        result['board'] = {'columns': args.columns, 'rows': args.rows, 'square_m': args.square_mm/1000}
        save_json(args.output, result)
        print(json.dumps({'views': result['views'], 'rms_px': result['rms_px']}))
    else:
        intrinsic_data = json.loads(args.intrinsics.read_text())
        expected = {'columns': args.columns, 'rows': args.rows, 'square_m': args.square_mm/1000}
        if intrinsic_data.get('board') != expected:
            raise ValueError('Use the same checkerboard dimensions as the intrinsic calibration')
        rgb = np.asarray(Image.open(args.image).convert('RGB'))
        if (rgb.shape[1],rgb.shape[0]) != (intrinsic_data['image_width'],intrinsic_data['image_height']):
            raise ValueError('Pose image resolution differs from intrinsic calibration')
        corners = find_board(rgb, args.columns, args.rows)
        grid = corners.reshape(args.rows, args.columns, 2)
        if args.flip_rows:
            grid = grid[::-1]
        if args.flip_columns:
            grid = grid[:, ::-1]
        corners = grid.reshape(-1, 2).copy()
        preview = rgb.copy()
        labels = [('A (origin)', 0), ('B (+X)', args.columns-1), ('C (+Y)', args.columns*(args.rows-1))]
        for label, index in labels:
            point = tuple(np.rint(corners[index]).astype(int))
            cv2.circle(preview, point, 5, (255,0,255), 2)
            cv2.putText(preview, label, (point[0]+7,point[1]-7), cv2.FONT_HERSHEY_SIMPLEX,.5,(255,0,255),1,cv2.LINE_AA)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        preview_path = args.output.with_suffix('.preview.png')
        if preview_path.exists():
            parser.error('Preview exists; use a new output file')
        Image.fromarray(preview).save(preview_path)
        xyz = board_points(args.columns, args.rows, args.square_mm/1000).astype(float)
        if args.command == 'reference':
            reference_pose = estimate_camera_pose(intrinsic_data, xyz, corners, work_frame='checkerboard')
            if reference_pose['camera_position_m'][2] <= 0:
                raise ValueError('Check board axis order: camera estimated below board; inspect preview')
            result = {**intrinsic_data, **reference_pose, 'backend': 'real', 'mount': 'fixed',
                      'verified': False, 'robot_registration_verified': False,
                      'image_sha256': hashlib.sha256(args.image.read_bytes()).hexdigest(),
                      'intrinsics_sha256': hashlib.sha256(args.intrinsics.read_bytes()).hexdigest(),
                      'flip_rows': args.flip_rows, 'flip_columns': args.flip_columns,
                      'reference_points': {label: {'pixel_xy': corners[i].tolist(),
                                                   'board_xyz_m': xyz[i].tolist()} for label, i in labels},
                      'review_required': 'Board coordinates only, NOT robot-base coordinates. Confirm printed axes, scale, planarity and independently measure robot registration. Never apply this reference as robot calibration.'}
            save_json(args.output, result)
            print(json.dumps({'pose_rms_px': result['pose_rms_px'], 'work_frame': result['work_frame'],
                              'robot_registration_verified': False}))
            return
        angle = np.deg2rad(args.yaw_deg)
        r = np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
        xyz = xyz @ r.T+args.origin_m
        pose_data = estimate_camera_pose(intrinsic_data, xyz, corners)
        if pose_data['camera_position_m'][2] <= 0:
            raise ValueError('Camera estimated below table: check board order / board coordinate orientation')
        result = {**intrinsic_data, **pose_data, 'backend': 'real', 'mount': 'fixed', 'verified': False,
                  'board_origin_work_m': args.origin_m, 'board_yaw_deg': args.yaw_deg,
                  'review_required': 'Check corner A (origin) and board axes against measured robot-base coordinates, then verify with independent points. Only then set verified=true locally.'}
        save_json(args.output, result)
        print(json.dumps({'pose_rms_px': result['pose_rms_px'], 'verified': False,
                          'review': result['review_required']}))


if __name__ == '__main__':
    main()
