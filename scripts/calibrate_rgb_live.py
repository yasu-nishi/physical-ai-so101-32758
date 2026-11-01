"""Local checkerboard capture UI. Opens only a camera; never a robot bus."""
import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
import numpy as np
from dotenv import dotenv_values

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_mcp.checkerboard_session import corner_distance, fit_session, resume_capture, save_capture
from so101_mcp.config import ROOT, load_config, real_camera_dimensions
from so101_mcp.rgb_calibration import find_board


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8770)
    parser.add_argument('--resume', action='store_true', help='Resume this private session with the same fixed camera')
    args = parser.parse_args()
    if not args.output.name.endswith('.local'):
        parser.error('Keep real images in a private .local directory')
    if args.output.exists() and any(args.output.iterdir()) and not args.resume:
        parser.error('Use a new/empty output directory; existing captures are never overwritten')
    cfg = load_config('real', args.config)
    env = {**dotenv_values(ROOT/'.env'), **os.environ}
    index = int(env[cfg['camera_index_env']])
    dimensions = real_camera_dimensions(cfg, env)
    if args.resume and not (args.output/'session.json').exists():
        parser.error('An existing session.json is required to resume')
    args.output.mkdir(parents=True, exist_ok=True)
    session = {'created_utc': datetime.now(timezone.utc).isoformat(), 'camera_index': index,
               'camera_dimensions': dimensions, 'robot_connections': 0, 'motor_writes': 0,
               'calibration_status': 'awaiting_printed_board', 'verified': False,
               'focus_lock_verified': False, 'views': 0}
    if args.resume:
        session = resume_capture(args.output, index, dimensions)
        session.setdefault('resumed_utc', []).append(datetime.now(timezone.utc).isoformat())
    reference_file = session.get('work_plane_reference_file')
    reference_preview = None
    if reference_file:
        if Path(reference_file).name != reference_file:
            parser.error('Reference must be a file inside the private session')
        reference_preview = (args.output/reference_file).with_suffix('.preview.png')
        if not reference_preview.is_file():
            parser.error('Saved reference preview is missing')
    lock = threading.Lock(); operations = threading.Lock()
    running = threading.Event(); running.set()
    state = {'rgb': None, 'corners': None, 'jpeg': None, 'time': 0., 'steady': False,
             'camera_error': None, 'auto': False, 'square_mm': session.get('square_mm'),
             'views': session['views'], 'capture_target': max(20, session['views']+5),
             'last_capture': 0., 'message': '', 'frame': 0}

    def write_session():
        temp = args.output/'session.tmp'
        temp.write_text(json.dumps(session, indent=2)+'\n')
        temp.replace(args.output/'session.json')

    write_session()

    def capture(data):
        # Caller holds operations; this binds the exact RGB to its detected corners.
        with lock:
            if state['rgb'] is None or time.monotonic()-state['time'] > 2:
                raise ValueError('新しいカメラ画像を取得できていません。')
            if state['corners'] is None:
                raise ValueError('全54交点が見えるように模様全体を映してください。')
            if not state['steady']:
                raise ValueError('紙を約2秒静止してください。')
            rgb, uv = state['rgb'].copy(), state['corners'].copy()
        square = float(data['square_mm'])
        save_capture(args.output/'views', rgb, uv, square, measured=data.get('measured'))
        with lock:
            state['views'] += 1; state['last_capture'] = time.monotonic()
            count = state['views']; state['message'] = f'{count}枚保存しました。次の位置・傾きに変えてください。'
            if count >= state['capture_target'] and state['auto']:
                state['auto'] = False; state['message'] = f'{count}枚保存しました。「校正値を計算」を押してください。'
        session.update(views=count, square_mm=square, printed_size_measured=True,
                       calibration_status='capturing_intrinsics')
        write_session()
        return {'views': count, 'message': f'{count}枚保存しました。'}

    def camera_loop():
        from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
        camera = OpenCVCamera(OpenCVCameraConfig(index_or_path=index, **dimensions))
        previous = None
        try:
            camera.connect()
            while running.is_set():
                camera.new_frame_event.clear()
                rgb = camera.async_read(timeout_ms=1000).copy()
                observed_at = time.monotonic()
                try: uv = find_board(rgb, 9, 6)
                except ValueError: uv = None
                steady = uv is not None and previous is not None and corner_distance(uv, previous) < 1.5
                previous = None if uv is None else uv.copy()
                annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if uv is not None:
                    cv2.drawChessboardCorners(annotated, (9, 6), uv.reshape(-1, 1, 2).astype(np.float32), True)
                ok, jpeg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok: raise RuntimeError('Camera frame encoding failed')
                with lock:
                    state.update(rgb=rgb, corners=uv, jpeg=jpeg.tobytes(), time=observed_at,
                                 steady=steady, frame=state['frame']+1)
                    auto = state['auto']; square = state['square_mm']
                    due = time.monotonic()-state['last_capture'] >= 2
                if auto and due:
                    with operations:
                        with lock: still_auto = state['auto']
                        if still_auto:
                            try: capture({'square_mm': square, 'measured': True})
                            except ValueError as error:
                                with lock: state['message'] = str(error)
                time.sleep(.15)
        except Exception as error:
            with lock: state.update(camera_error=str(error), auto=False)
            print(json.dumps({'camera_error': str(error)}), flush=True)
        finally:
            if camera.is_connected: camera.disconnect()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass

        def send_bytes(self, mime, body, status=200):
            self.send_response(status)
            self.send_header('Content-Type', mime); self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store'); self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers(); self.wfile.write(body)

        def send_json(self, value, status=200):
            self.send_bytes('application/json; charset=utf-8', json.dumps(value, ensure_ascii=False).encode(), status)

        def do_GET(self):
            if self.path == '/':
                self.send_bytes('text/html; charset=utf-8', (ROOT/'so101_mcp/web/checkerboard_calibration.html').read_bytes())
            elif self.path == '/checkerboard.pdf':
                self.send_bytes('application/pdf', (ROOT/'output/pdf/rgb_checkerboard.pdf').read_bytes())
            elif self.path == '/reference.png' and reference_preview is not None:
                self.send_bytes('image/png', reference_preview.read_bytes())
            elif self.path == '/state':
                with lock:
                    result = {k: state[k] for k in ('camera_error', 'views', 'auto', 'message', 'frame', 'square_mm', 'capture_target')}
                    result.update(fresh=state['rgb'] is not None and time.monotonic()-state['time'] < 2,
                                  detected=state['corners'] is not None, camera_index=index,
                                  calibration_status=session['calibration_status'],
                                  reference_available=reference_preview is not None,
                                  width=dimensions['width'], height=dimensions['height'],
                                  robot_connections=0, motor_writes=0)
                self.send_json(result)
            elif self.path == '/stream':
                self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store'); self.end_headers()
                sent = -1
                try:
                    while running.is_set():
                        with lock: jpeg, sequence = state['jpeg'], state['frame']
                        if jpeg is not None and sequence != sent:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                            self.wfile.flush(); sent = sequence
                        time.sleep(.1)
                except (BrokenPipeError, ConnectionResetError): pass
            else: self.send_error(404)

        def do_POST(self):
            origin = f'http://127.0.0.1:{args.port}'
            if self.headers.get('Origin') not in (None, origin) or self.headers.get('Host') != f'127.0.0.1:{args.port}':
                return self.send_json({'error': 'Local page origin required'}, 403)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 16384: raise ValueError('Request too large')
                data = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(data, dict): raise ValueError('JSON object required')
                with operations: result = self.operation(data)
                self.send_json(result)
            except (ValueError, KeyError, TypeError, cv2.error) as error:
                self.send_json({'error': str(error)}, 400)

        def operation(self, data):
            if self.path == '/stop':
                running.clear(); threading.Thread(target=server.shutdown, daemon=True).start()
                return {'stopped': True}
            if self.path == '/pause':
                with lock: state.update(auto=False, message='撮影を一時停止しました。')
                return {'auto': False}
            if self.path == '/capture': return capture(data)
            if self.path == '/auto':
                square = float(data['square_mm'])
                if data.get('measured') is not True or not np.isfinite(square) or not 5 <= square <= 50:
                    raise ValueError('定規で印刷寸法を確認し、1マスの長さを入力してください。')
                if session.get('square_mm', square) != square:
                    raise ValueError('途中で寸法を変えられません。新しいセッションが必要です。')
                with lock:
                    state.update(auto=True, square_mm=square, capture_target=max(20, state['views']+5),
                                 message='板を前後・左右に傾け、各姿勢で約2秒静止してください。')
                return {'auto': True}
            if self.path == '/fit':
                with lock: state['auto'] = False
                review = args.output/'capture_review.json'
                result = fit_session(args.output/'views', review_file=review if review.exists() else None)
                note = args.output/'measurement_note.json'
                if note.exists():
                    result['print_scale_observation'] = json.loads(note.read_text())
                result['focus_lock_verified'] = session['focus_lock_verified']
                n = 1
                while (args.output/f'intrinsics_draft_{n:03}.json').exists(): n += 1
                path = args.output/f'intrinsics_draft_{n:03}.json'
                with path.open('x') as f: json.dump(result, f, indent=2)
                session.update(calibration_status='intrinsics_draft', intrinsic_file=path.name)
                write_session()
                with lock: state['message'] = '内部校正の下書きを保存しました。次は実測した作業座標との位置合わせです。'
                return {'file': path.name, 'rms_px': result['rms_px'], 'verified': False,
                        'selected_captures': result['selected_captures'], 'excluded_captures': len(result['excluded_captures'])}
            raise ValueError('Unknown operation')

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    def shutdown(*_):
        running.clear(); threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    worker = threading.Thread(target=camera_loop, daemon=True); worker.start()
    print(f'Checkerboard capture: http://127.0.0.1:{args.port}; no robot connection', flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        running.clear(); server.server_close(); worker.join(timeout=4)


if __name__ == '__main__': main()
