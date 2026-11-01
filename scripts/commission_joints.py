"""Local, explicit, bounded commissioning jogs. Not an MCP task-motion bypass.

Default is read-only. --execute requires a saved operator authorization note.
STOP is latched; torque stays holding to avoid dropping the arm. No auto resume.
"""
import argparse
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import signal
import shutil
import sys
import time
import threading
import urllib.request

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dotenv import dotenv_values
from so101_mcp.config import ROOT, load_config
from so101_mcp.joints import ARM, JointMap
from so101_mcp.commissioning import JointCommissioner
from so101_mcp.servo_diagnostics import CheckedReads,timestamped


def start_stop_panel(run_dir, port, camera_port):
    """Loopback-only stop/status. It cannot enable torque or submit motion."""
    page = '''<!doctype html><html lang="ja"><meta charset="utf-8">
<title>SO-101 初期姿勢調整</title><style>body{font:20px sans-serif;max-width:900px;margin:24px auto}
button{font:700 30px sans-serif;background:#ad1717;color:white;padding:24px;border:0;cursor:pointer}
img{width:100%;max-width:640px}pre{font:14px monospace;white-space:pre-wrap}</style>
<h1>SO-101 初期姿勢調整</h1><p>腕の周囲から手を離してください。停止後は現在姿勢を保持します。</p>
<button id="stop">停止 — 位置保持</button><p id="message"></p>
<img src="http://127.0.0.1:CAMERA/stream" alt="作業台のライブ画像"><pre id="status"></pre>
<p>通信が切れた場合は電源スイッチで停止できます。電源を切ると保持力がなくなります。</p>
<script>document.querySelector('#stop').onclick=async()=>{try{
let r=await fetch('/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
if(!r.ok)throw new Error();document.querySelector('#message').textContent='停止要求を保存しました';
}catch(e){document.querySelector('#message').textContent='停止要求の送信に失敗しました';}};
setInterval(async()=>{try{let r=await fetch('/status');let s=await r.json();
document.querySelector('#status').textContent=JSON.stringify(s,null,2);}catch(e){}},300);</script></html>'''.replace('CAMERA',str(camera_port))
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def send(self, mime, value, status=200):
            self.send_response(status); self.send_header('Content-Type',mime)
            self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(value)
        def do_GET(self):
            if self.path == '/': self.send('text/html; charset=utf-8',page.encode())
            elif self.path == '/status':
                path=run_dir/'status.json'
                self.send('application/json',path.read_bytes() if path.exists() else b'{"phase":"starting"}')
            else: self.send_error(404)
        def do_POST(self):
            origin=f'http://127.0.0.1:{port}'
            if self.path != '/stop': return self.send_error(404)
            if self.headers.get('Host') != f'127.0.0.1:{port}' or self.headers.get('Origin') not in (None,origin) or self.headers.get('Content-Type') != 'application/json':
                return self.send_error(403)
            (run_dir/'STOP').touch()
            self.send('application/json',b'{"stop_requested":true}')
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    return server


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--authorization-note',type=Path)
    p.add_argument('--camera-port',type=int,default=8770)
    p.add_argument('--panel-port',type=int,default=8771)
    p.add_argument('--adopt-stopped-run',type=Path,
                   help='One explicit, recorded handover of an unchanged stopped hold; never automatic')
    a = p.parse_args()
    if not a.run_dir.name.endswith('.local'):
        p.error('Use a private .local run directory')
    if a.execute and (not a.authorization_note or not a.authorization_note.is_file()):
        p.error('Explicit operator authorization must be recorded before --execute')
    if (a.run_dir/'status.json').exists():
        p.error('Use a new commissioning run directory; stopped runs cannot resume')
    a.run_dir.mkdir(parents=True,exist_ok=True)
    (a.run_dir/'controller.pid').write_text(str(os.getpid()))
    source_dir=a.run_dir/'executed_source';source_dir.mkdir()
    for source in (Path(__file__).resolve(),ROOT/'so101_mcp/commissioning.py',ROOT/'so101_mcp/servo_diagnostics.py',a.config):
        shutil.copy2(source,source_dir/source.name)
    cfg=load_config('real',a.config)
    previous=None
    if a.adopt_stopped_run:
        old=a.adopt_stopped_run
        if not a.execute or not (old/'STOP').is_file() or not (old/'EXIT').is_file():
            p.error('Handover requires --execute and the previous run stopped and exited')
        old_pid=int((old/'controller.pid').read_text())
        try:os.kill(old_pid,0)
        except ProcessLookupError:pass
        else:p.error('The previous serial owner is still running')
        previous=json.loads((old/'status.json').read_text())
        if 'session_initial_joints_deg' not in previous:
            entries=[json.loads(line) for line in (old/'events.jsonl').read_text().splitlines()]
            previous['session_initial_joints_deg']=next(e['joints_deg'] for e in entries if e['event']=='preflight_passed')
    env={**dotenv_values(ROOT/'.env'),**os.environ}
    # Prevent two instances of this local commissioning controller sharing the bus.
    lock_path=ROOT/'artifacts/commissioning_bus.local.lock'; lock_path.parent.mkdir(exist_ok=True)
    lock=lock_path.open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from lerobot.robots.so_follower.so_follower import SO101Follower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    # Opening the serial bus alone does not configure motors or toggle torque.
    robot=SO101Follower(SOFollowerRobotConfig(port=env[cfg['port_env']],id=env[cfg['robot_id_env']],
                            use_degrees=True,cameras={},disable_torque_on_disconnect=False))
    def camera_fresh():
        with urllib.request.urlopen(f'http://127.0.0.1:{a.camera_port}/state',timeout=.3) as response:
            state=json.load(response)
        return state.get('fresh') is True and state.get('camera_error') is None
    def feedback_record(row):
        if row.get('validation_error') or row.get('result',0)!=0 or row.get('kind')=='read_failure_input_resync':
            with (a.run_dir/'feedback_faults.jsonl').open('a') as f:
                f.write(json.dumps(timestamped(row))+'\n')
    feedback=CheckedReads(robot.bus,feedback_record)
    controller=JointCommissioner(robot.bus,JointMap(cfg['mapping']),cfg['limits_deg'],a.run_dir,
                                camera_fresh,adopt_stopped=previous,read_feedback=feedback.read)
    def stop_signal(*_):
        (a.run_dir/'STOP').touch()
    signal.signal(signal.SIGINT,stop_signal)
    signal.signal(signal.SIGTERM,stop_signal)
    panel=start_stop_panel(a.run_dir,a.panel_port,a.camera_port) if a.execute else None
    try:
        robot.bus.connect();robot.bus.set_timeout(100)
        if not a.execute:
            controller.preflight()
            return
        note=json.loads(a.authorization_note.read_text())
        if not note.get('user_instruction'):
            raise ValueError('Operator authorization note is empty')
        controller.event('operator_authorization',instruction=note['user_instruction'],
                         scope='initial pose commissioning only; no task calibration flags changed')
        if previous:controller.event('explicit_stopped_hold_handover',previous_status=previous)
        controller.arm()
        pending=a.run_dir/'commands';pending.mkdir(exist_ok=True)
        processed=a.run_dir/'processed';processed.mkdir(exist_ok=True)
        while not (a.run_dir/'EXIT').exists():
            if controller.latched:
                controller.monitor_stopped();time.sleep(.5)
                continue
            controller.tick()
            for path in sorted(pending.glob('*.json')):
                command=json.loads(path.read_text())
                path.rename(processed/path.name)
                controller.step(command)
                break
            time.sleep(.05)
        controller.halt('Local EXIT: controller closing while position hold remains active')
    except Exception as error:
        controller.halt(type(error).__name__+': '+str(error))
        print(json.dumps(controller.last_result),flush=True)
        # Keep the serial owner available and STOP latched if any motor was enabled.
        while controller.armed and not (a.run_dir/'EXIT').exists():
            controller.monitor_stopped();time.sleep(.5)
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)
        feedback.restore()
        if panel: panel.shutdown(); panel.server_close()
        lock.close()


if __name__=='__main__':main()
