"""Sanitized supply/feedback report from private, read-only wire captures."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',action='append',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();runs=[]
    names={1:'shoulder_pan',2:'shoulder_lift',3:'elbow_flex',4:'wrist_flex',5:'wrist_roll',6:'gripper'}
    for folder in a.run_dir:
        summary=json.loads((folder/'summary.json').read_text())
        samples=[json.loads(line) for line in (folder/'samples.jsonl').read_text().splitlines()]
        packets=[json.loads(line) for line in (folder/'wire_packets.jsonl').read_text().splitlines()]
        volts={n:[] for n in names.values()};valid=0;bad=0;flags=[];comm=0;nonread=0
        for sample in samples:
            for method in ('voltage_group_raw','voltage_single_raw'):
                for name,value in sample['values'].get(method,{}).items():volts[name].append(value/10)
        for item in packets:
            if item['kind']=='tx_read':
                nonread+=item['instruction'] not in (1,2,130)
                continue
            if item['kind']!='rx':continue
            if item['result']!=0:comm+=1;continue
            ctx=item.get('context');raw=item['packet']
            if not ctx:continue  # Connection handshake has no register-read context.
            structure=(len(raw)==ctx['bytes']+6 and raw[:2]==[255,255] and raw[3]==ctx['bytes']+2 and
                       raw[2] in ctx['motor_ids'] and (sum(raw[2:])&255)==255)
            if not structure:bad+=1;continue
            valid+=1
            # Do not treat every wire frame as the requested register. A group
            # reader can discard a delayed reply with an earlier expected ID.
            # Add voltage error payloads only from a matching unicast request;
            # ordinary values above come from completed reads in sample logs.
            if ctx['register']=='Present_Voltage' and raw[4] and ctx['motor_ids']==[raw[2]]:
                volts[names[raw[2]]].append(raw[5]/10)
            if raw[4]:
                flags.append({'time_utc':item['time_utc'],'joint':names[raw[2]],'register':ctx['register'],
                    'error_bits':raw[4],'voltage_error':bool(raw[4]&1),
                    'voltage_v':raw[5]/10 if ctx['register']=='Present_Voltage' else None,
                    'reply_length_id_and_checksum_valid':True,
                    'packet_sha256':hashlib.sha256(bytes(raw)).hexdigest()})
        if nonread:raise ValueError('Non-read wire instruction found; report requires manual review')
        evidence={str(f.relative_to(folder)):hashlib.sha256(f.read_bytes()).hexdigest()
                  for f in [folder/'summary.json',folder/'samples.jsonl',folder/'wire_packets.jsonl',
                            folder/'baseline.json',folder/'operator_context.json',
                            *sorted((folder/'executed_source').glob('*'))] if f.is_file()}
        baseline=json.loads((folder/'baseline.json').read_text()) if (folder/'baseline.json').exists() else {}
        context=json.loads((folder/'operator_context.json').read_text()) if (folder/'operator_context.json').exists() else {}
        initial=baseline.get('registers',{}).get('Present_Position',{})
        position_delta={n:max((abs(row['values']['Present_Position'][n]-v) for row in samples
                            if n in row['values'].get('Present_Position',{})),default=0) for n,v in initial.items()}
        runs.append({'capture_index':len(runs)+1,'capture_s':summary['elapsed_s'],'samples':summary['sample_count'],
            'after_operator_restart':summary.get('after_operator_restart',False),
            'operator_context':context,'fresh_baseline_mode':summary.get('fresh_baseline_mode',summary.get('after_operator_restart',False)),
            'unicast_only':summary.get('unicast_only',False),'sample_hz':summary.get('sample_hz',5.),
            'baseline_torque_enabled':baseline.get('registers',{}).get('Torque_Enable'),
            'motor_register_writes':0,'non_read_instructions':nonread,
            'valid_register_response_packets':valid,'structural_or_id_mismatches':bad,
            'communication_error_results':comm,'servo_error_response_packets':len(flags),
            'voltage_ranges_v_including_error_replies':{n:[min(v),max(v)] for n,v in volts.items() if v},
            'servo_errors':flags,'holding_register_change_samples':summary['holding_register_change_samples'],
            'maximum_observed_position_delta_ticks':position_delta,
            'private_evidence_sha256':evidence})
    result={'schema_version':1,'date':'2026-11-01','scope':'Read-only diagnosis of a stopped robot; per-run torque state distinguishes holding from power restart',
        'motor_register_writes':0,'automatic_motion_restart':False,'robot_unfolded':False,
        'servo_voltage_error_reproduced':any(f['voltage_error'] for r in runs for f in r['servo_errors']),
        'physical_fault_location':'undetermined: adapter, distribution wiring/connectors, or servo electronics/sensing',
        'source':'https://docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation',
        'interpretation':'Voltage is an internal servo sensor report, not an external multimeter measurement. Error bit 0 is a voltage alarm.',
        'range_method':'Completed group/unicast read values plus matching unicast voltage-error replies; not every frame seen on the wire.',
        'software_findings':['Installed feetech-servo-sdk 1.0.0 group read discards servo error flags.',
            'Payload length must be checked against each read request.',
            'An interrupted group read can leave older replies queued; serial.flush() does not empty host input.',
            'Diagnostic code now distinguishes valid servo error replies from malformed packets, and clears host input after a failed read.',
            'No motion, torque, calibration or guard-limit changes were made in these diagnostic captures.'],
        'limits':['This does not identify the cause of the earlier elbow tracking failure.',
            'The original commissioning voltage trigger was not retained; these are separate later observations.',
            'A supplied product name or ordinary power adapter description does not establish voltage at a downstream servo.'],
        'runs':runs}
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'runs':len(runs),'voltage_error_reproduced':result['servo_voltage_error_reproduced'],
        'capture_samples':sum(r['samples'] for r in runs),'servo_error_packets':sum(r['servo_error_response_packets'] for r in runs),
        'motor_writes':0},indent=2))


if __name__=='__main__':main()
