"""Diagnose an already stopped SO-101 hold using READ packets only.

No rearming, torque changes, calibration or motor-position writes are possible.
The previous serial owner must have exited. A bounded detailed capture is
followed by passive monitoring until EXIT is created in the new run directory.
"""
import argparse
from collections import deque
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dotenv import dotenv_values
from so101_mcp.config import ROOT,load_config
from so101_mcp.servo_diagnostics import AuditedReads,timestamped,feedback_phase
from commission_joints import start_stop_panel


def save(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--previous-run',type=Path,required=True)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--capture-seconds',type=float,default=90.)
    p.add_argument('--panel-port',type=int,default=8771)
    p.add_argument('--after-restart',action='store_true',
                   help='Read a fresh baseline after an operator-reported restart; never enable torque')
    p.add_argument('--fresh-baseline',action='store_true',
                   help='Read a fresh baseline after operator maintenance; no assumption that a restart occurred')
    p.add_argument('--unicast-only',action='store_true',
                   help='Use separate reads for each motor instead of grouped register reads')
    p.add_argument('--sample-hz',type=float,default=5.)
    a=p.parse_args()
    if not a.run_dir.name.endswith('.local') or not 5<=a.capture_seconds<=300:
        p.error('Private .local directory and a 5–300 second capture required')
    if not .5<=a.sample_hz<=5:p.error('Sample rate must be between 0.5 and 5 Hz')
    fresh_baseline=a.after_restart or a.fresh_baseline
    old=json.loads((a.previous_run/'status.json').read_text())
    if not old['stop_latched'] or not (a.previous_run/'EXIT').is_file():
        p.error('Previous controller must already be stopped and exiting')
    try:os.kill(int((a.previous_run/'controller.pid').read_text()),0)
    except ProcessLookupError:pass
    else:p.error('Previous serial owner still running')
    a.run_dir.mkdir(parents=True,exist_ok=True)
    if (a.run_dir/'status.json').exists():p.error('Use a new diagnostic run directory')
    (a.run_dir/'STOP').touch()
    (a.run_dir/'controller.pid').write_text(str(os.getpid()))
    source=a.run_dir/'executed_source';source.mkdir()
    for f in (Path(__file__),ROOT/'so101_mcp/servo_diagnostics.py',a.config):shutil.copy2(f,source/f.name)
    lock=(ROOT/'artifacts/commissioning_bus.local.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    cfg=load_config('real',a.config);env={**dotenv_values(ROOT/'.env'),**os.environ}
    from lerobot.robots.so_follower.so_follower import SO101Follower
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    robot=SO101Follower(SOFollowerRobotConfig(port=env[cfg['port_env']],id=env[cfg['robot_id_env']],
                         use_degrees=True,cameras={},disable_torque_on_disconnect=False))
    ring=deque(maxlen=256);capture=[True]
    wire=(a.run_dir/'wire_packets.jsonl').open('w')
    samples_file=(a.run_dir/'samples.jsonl').open('w')
    def record(row):
        row=timestamped(row);ring.append(row)
        if capture[0]:wire.write(json.dumps(row)+'\n');wire.flush()
    audit=AuditedReads(robot.bus,record)
    def read_all(register):
        if a.unicast_only:return {n:audit.read(register,n) for n in robot.bus.motors}
        return audit.read(register)
    panel=None;rows=[];summary_saved=False
    def exit_signal(*_):(a.run_dir/'EXIT').touch()
    signal.signal(signal.SIGINT,exit_signal);signal.signal(signal.SIGTERM,exit_signal)
    try:
        robot.bus.connect();robot.bus.set_timeout(100)
        baseline={key:read_all(key) for key in ('Present_Position','Goal_Position','Torque_Enable',
            'Goal_Velocity','Operating_Mode','Min_Voltage_Limit','Max_Voltage_Limit','Protection_Current')}
        if not fresh_baseline and baseline['Present_Position']!=old['raw_positions']:
            raise ValueError('Pose changed since the previous stopped controller')
        if not fresh_baseline and any(baseline['Torque_Enable'][n]!=int(n!='gripper') for n in robot.bus.motors):
            raise ValueError('Stopped holding torque state changed')
        calibration_matches=True
        for n,c in robot.bus.calibration.items():
            for register,attr in [('Min_Position_Limit','range_min'),('Max_Position_Limit','range_max'),('Homing_Offset','homing_offset')]:
                if audit.read(register,n)!=getattr(c,attr):calibration_matches=False
        if not calibration_matches:raise ValueError('Stored motor calibration mismatch')
        save(a.run_dir/'baseline.json',{'registers':baseline,'calibration_matches':True,'motor_register_writes':0,
            'after_operator_restart':a.after_restart,'fresh_baseline_mode':fresh_baseline,
            'unicast_only':a.unicast_only,'sample_hz':a.sample_hz,'previous_position':old.get('raw_positions')})
        panel=start_stop_panel(a.run_dir,a.panel_port,8770)
        began=time.monotonic()
        while not (a.run_dir/'EXIT').exists():
            start=time.monotonic();values={};errors=[]
            try:
                if not a.unicast_only:values['voltage_group_raw']=audit.read('Present_Voltage')
                # Separate unicast reads exercise another receive path and preserve error flags.
                values['voltage_single_raw']={n:audit.read('Present_Voltage',n) for n in robot.bus.motors}
                for register in ('Status','Present_Position','Goal_Position','Torque_Enable','Present_Temperature','Present_Load','Present_Current'):
                    values[register]=read_all(register)
                for key in ('voltage_group_raw','voltage_single_raw'):
                    for n,v in values.get(key,{}).items():
                        if not baseline['Min_Voltage_Limit'][n]+3<v<baseline['Max_Voltage_Limit'][n]-1:
                            errors.append({'type':'voltage_guard','motor':n,'read_path':key,'raw':v})
                for key in ('Present_Position','Goal_Position','Torque_Enable'):
                    if values[key]!=baseline[key]:errors.append({'type':'holding_register_changed','register':key})
                if any(values['Status'].values()):errors.append({'type':'servo_status','raw':values['Status']})
            except Exception as e:errors.append({'type':'read_error','message':type(e).__name__+': '+str(e)})
            row=timestamped({'values':values,'errors':errors,'sample_read_s':time.monotonic()-start})
            if capture[0]:rows.append(row);samples_file.write(json.dumps(row)+'\n');samples_file.flush()
            if errors:
                with (a.run_dir/'anomalies.jsonl').open('a') as f:f.write(json.dumps({'sample':row,'recent_wire_packets':list(ring)})+'\n')
            save(a.run_dir/'status.json',{'mode':'read_only_wire_diagnostics','phase':feedback_phase(values,errors),
                'stop_latched':True,'motor_register_writes_this_session':0,'motion_commands_supported':False,
                'time_utc':row['time_utc'],'raw_positions':values.get('Present_Position'),
                'torque_enabled':values.get('Torque_Enable'),'after_operator_restart':a.after_restart,
                'fresh_baseline_mode':fresh_baseline,'unicast_only':a.unicast_only,'sample_hz':a.sample_hz,
                'voltage_v':{n:v/10 for n,v in values.get('voltage_group_raw',values.get('voltage_single_raw',{})).items()},
                'servo_status':values.get('Status'),'latest_errors':errors,
                'detailed_samples':len(rows),'read_packets':audit.read_packets,'rx_packets':audit.rx_packets,
                'invalid_feedback_packets':audit.invalid_packets,'servo_error_packets':audit.servo_error_packets,
                'capture_finished':summary_saved})
            if time.monotonic()-began>=a.capture_seconds and not summary_saved:
                names=list(robot.bus.motors)
                ranges={method:{n:[min(r['values'][method][n] for r in rows if method in r['values'])/10,
                    max(r['values'][method][n] for r in rows if method in r['values'])/10] for n in names}
                    for method in ('voltage_group_raw','voltage_single_raw') if any(method in r['values'] for r in rows)}
                summary={'mode':'read_only_wire_diagnostics','sample_count':len(rows),'elapsed_s':time.monotonic()-began,
                    'motor_register_writes':0,'non_read_serial_packets_sent':0,'blocked_non_read_packets':audit.blocked_non_read_packets,
                    'rx_packets':audit.rx_packets,'invalid_feedback_packets':audit.invalid_packets,
                    'servo_error_packets':audit.servo_error_packets,'voltage_ranges_v':ranges,
                    'anomalies':[{'time_utc':r['time_utc'],'errors':r['errors']} for r in rows if r['errors']],
                    'servo_status_nonzero_samples':sum(any(r['values'].get('Status',{}).values()) for r in rows),
                    'holding_register_change_samples':sum(any(e['type']=='holding_register_changed' for e in r['errors']) for r in rows),
                    'cause_identified':False,'automatic_motion_restart':False,
                    'after_operator_restart':a.after_restart,'fresh_baseline_mode':fresh_baseline,
                    'unicast_only':a.unicast_only,'sample_hz':a.sample_hz}
                save(a.run_dir/'summary.json',summary)
                capture[0]=False;summary_saved=True
            time.sleep(max(0.,(1/a.sample_hz if capture[0] else max(1.,1/a.sample_hz))-(time.monotonic()-start)))
    except Exception as e:
        save(a.run_dir/'fatal_error.json',{'error':type(e).__name__+': '+str(e),'motor_register_writes':0})
        raise
    finally:
        if robot.bus.is_connected:robot.bus.disconnect(disable_torque=False)
        audit.restore();wire.close();samples_file.close()
        if panel:panel.shutdown();panel.server_close()
        lock.close()


if __name__=='__main__':main()
