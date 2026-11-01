"""Record the existing local MJPEG camera stream, without reopening the camera.

Frames and timestamps stay in a private .local folder. A RECORD_STOP file ends
recording; at most ten minutes are recorded. This process never opens motors.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time
import urllib.request
import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--port',type=int,default=8770)
    a=p.parse_args()
    if not a.run_dir.name.endswith('.local'):p.error('Use a private .local directory')
    a.run_dir.mkdir(parents=True,exist_ok=True)
    if (a.run_dir/'camera.mp4').exists():p.error('Recording already exists')
    state={'image':None,'sequence':0,'time':0.,'error':None}
    lock=threading.Lock();done=threading.Event()
    def reader():
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{a.port}/stream',timeout=3) as r:
                data=b''
                while not done.is_set():
                    data+=r.read(4096)
                    while b'\xff\xd9' in data:
                        start=data.find(b'\xff\xd8');end=data.find(b'\xff\xd9')+2
                        chunk=data[start:end];data=data[end:]
                        if start<0:continue
                        img=cv2.imdecode(np.frombuffer(chunk,np.uint8),cv2.IMREAD_COLOR)
                        if img is not None:
                            with lock:
                                state.update(image=img,sequence=state['sequence']+1,time=time.monotonic())
        except Exception as e:
            with lock:state['error']=str(e)
    threading.Thread(target=reader,daemon=True).start()
    started=time.monotonic();writer=None;count=0;first=None
    with (a.run_dir/'camera_timestamps.jsonl').open('w') as log:
        try:
            while time.monotonic()-started<600 and not (a.run_dir/'RECORD_STOP').exists():
                with lock:s=dict(state)
                if s['error']:raise RuntimeError(s['error'])
                if s['image'] is None:
                    if time.monotonic()-started>5:raise RuntimeError('No fresh camera frame')
                    time.sleep(.05);continue
                if time.monotonic()-s['time']>2:raise RuntimeError('Camera became stale')
                if writer is None:
                    h,w=s['image'].shape[:2]
                    writer=cv2.VideoWriter(str(a.run_dir/'camera.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),5.,(w,h))
                    if not writer.isOpened():raise RuntimeError('Cannot create video')
                    first=time.monotonic()
                writer.write(s['image'])
                log.write(json.dumps({'frame':count,'time_utc':datetime.now(timezone.utc).isoformat(),
                    'source_sequence':s['sequence'],'source_age_s':time.monotonic()-s['time']})+'\n');log.flush()
                count+=1
                time.sleep(max(0.,first+count/5-time.monotonic()))
        finally:
            done.set()
            if writer:writer.release()
            (a.run_dir/'camera_recording.json').write_text(json.dumps({'frames':count,'fps':5,
                'video_duration_s':count/5,'elapsed_s':time.monotonic()-first if first else 0,
                'source':'existing local workspace camera stream','motor_connections':0,
                'public_export':False},indent=2)+'\n')


if __name__=='__main__':main()
