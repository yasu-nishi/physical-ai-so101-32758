"""Publishable aggregate of private commissioning logs; never copy real images."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',action='append',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();runs=[]
    for folder in a.run_dir:
        events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
        status=json.loads((folder/'status.json').read_text())
        start=next(e for e in events if e['event']=='preflight_passed')
        attempts=Counter(e['register'] for e in events if e['event']=='write_attempt')
        ack=Counter(e['register'] for e in events if e['event']=='write_ack')
        stops=[e for e in events if e['event']=='stop_latched']
        reason=stops[-1]['reason'] if stops else ''
        category='position_tracking_limit' if 'tracking error' in reason else 'supply_guard' if 'supply outside' in reason else 'other_details_private'
        clip=json.loads((folder/'camera_recording.json').read_text()) if (folder/'camera_recording.json').exists() else None
        validation=json.loads((folder/'camera_validation.json').read_text()) if (folder/'camera_validation.json').exists() else None
        evidence={name:sha(folder/name) for name in ['events.jsonl','before.jpg','camera.mp4','camera_timestamps.jsonl','camera_validation.json','diagnosis.json','stopped_monitor_samples.jsonl'] if (folder/name).is_file()}
        jogs=[e for e in events if e['event']=='step_started']
        (folder/'report_status_snapshot.json').write_text(json.dumps(status,indent=2)+'\n')
        evidence['report_status_snapshot.json']=sha(folder/'report_status_snapshot.json')
        runs.append({'index':len(runs)+1,'started_utc':start['time_utc'],
            'last_recorded_stop_utc':stops[-1]['time_utc'] if stops else None,
            'startup_to_stop_s':(datetime.fromisoformat(stops[-1]['time_utc'])-datetime.fromisoformat(start['time_utc'])).total_seconds() if stops else None,
            'jog_start_to_stop_s':(datetime.fromisoformat(stops[-1]['time_utc'])-datetime.fromisoformat(jogs[-1]['time_utc'])).total_seconds() if jogs and stops else None,
            'jog_attempts':sum(e['event']=='step_started' for e in events),
            'jogs_completed':sum(e['event']=='step_completed' for e in events),
            'acknowledged_register_writes':dict(ack),'attempted_register_writes':dict(attempts),
            'servo_speed_register_values':sorted({e['value'] for e in events if e['event']=='write_ack' and e['register']=='Goal_Velocity'}),
            'failure_category':category,'stop_latched':status['stop_latched'],
            'trigger_health_snapshot_saved':'health_at_stop' in stops[-1] if stops else False,
            'final_encoder_matches_initial':status['raw_positions']==start['initial_raw'],
            'position_hold_enabled':status['armed'],
            'executed_source_snapshot_available':(folder/'executed_source').exists(),
            'clip':clip,'video_all_frames_decoded':bool(validation and validation['decoded_all_frames']),
            'private_evidence_sha256':evidence})
    totals=Counter()
    for run in runs:totals.update(run['acknowledged_register_writes'])
    report={'schema_version':1,'created_utc':datetime.now(timezone.utc).isoformat(),
        'scope':'Explicit local initial-pose commissioning after the earlier read-only MCP connection check',
        'outcome':'not_unfolded; stopped with position hold; no automatic restart',
        'initialization_runs':len(runs),'jog_attempts':sum(r['jog_attempts'] for r in runs),
        'jogs_completed':sum(r['jogs_completed'] for r in runs),'confirmed_unfold_successes':0,
        'motor_goal_writes':totals['Goal_Position'],'torque_writes':totals['Torque_Enable'],
        'speed_register_writes':totals['Goal_Velocity'],'calibration_writes':0,
        'physical_pick_place_trials':0,'physical_pick_place_successes':None,
        'task_joint_limits_changed':False,'tracking_stop_limit_relaxed':False,
        'task_verification_flags_changed':False,'mcp_task_motion_enabled':False,'sim_modified':False,
        'speed_configuration_correction':{
            'original_value':12,'corrected_value':50,'unit':'encoder steps/second with 50-step/second effective speed resolution',
            'source':'https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation',
            'causality':'Original setting below documented effective resolution. No completed jog after correction; root cause of original no-motion not proven.'},
        'limits':['Motor-command acknowledgement is not task success.',
            'Second run stopped during hold on a supply guard before any jog was submitted.',
            'Trigger voltage sample was not retained in these runs; later status polling must not be presented as the trigger value.',
            'Final encoders equal initial encoders; intermediate physical motion was not independently measured.',
            'Real camera/model/workspace verification and pick/place remain unverified.',
            'Real imagery and device/calibration/authorization records remain private; only sanitized totals and evidence hashes exported.'],
        'runs':runs}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('outcome','initialization_runs','jog_attempts','jogs_completed','motor_goal_writes','torque_writes','speed_register_writes')}))


if __name__=='__main__':main()
