"""Compare grouped and individual servo feedback without motor writes.

Run only after the motion server has exited. All transmitted instructions are
filtered by AuditedReads. Error responses remain errors; no automatic restart.
Use --motor-ids 6 --read-path individual for an isolated gripper diagnostic.
Selecting an ID does not prove that the operator has isolated its wiring.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dotenv import dotenv_values
from so101_mcp.config import ROOT, load_config
from so101_mcp.servo_diagnostics import AuditedReads, timestamped


def make_diagnostic_bus(port, motor_ids):
    """Raw-register bus for explicitly selected SO-101 IDs; no calibration writes."""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    names = ('shoulder_pan', 'shoulder_lift', 'elbow_flex',
             'wrist_flex', 'wrist_roll', 'gripper')
    if (not motor_ids or len(set(motor_ids)) != len(motor_ids)
            or any(i not in range(1, 7) for i in motor_ids)):
        raise ValueError('Select unique motor IDs from 1 through 6')
    motors = {names[i-1]: Motor(i, 'sts3215',
              MotorNormMode.RANGE_0_100 if i == 6 else MotorNormMode.DEGREES)
              for i in motor_ids}
    return FeetechMotorsBus(port=port, motors=motors)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seconds-per-path', type=float, default=10.)
    p.add_argument('--motor-ids', type=int, nargs='+', default=list(range(1, 7)),
                   help='IDs to read; selecting one does not verify physical isolation')
    p.add_argument('--read-path', choices=('both', 'individual', 'grouped'), default='both')
    p.add_argument('--setup-label', default='unspecified',
                   help='Operator-reported wiring label, stored as unverified context')
    args = p.parse_args()
    if not args.output.name.endswith('.local') or args.output.exists():
        p.error('Use a new private .local output directory')
    if not 2 <= args.seconds_per_path <= 30:
        p.error('Each read path must run for 2–30 seconds')
    cfg = load_config('real', args.config)
    env = {**dotenv_values(ROOT/'.env'), **os.environ}
    port = env[cfg['port_env']]
    if subprocess.run(['/usr/sbin/lsof', '-t', port], capture_output=True, text=True).stdout.strip():
        raise RuntimeError('Serial device still has an owner; stop that process first')
    args.output.mkdir(parents=True)
    lock = (ROOT/'artifacts/commissioning_bus.local.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    bus = make_diagnostic_bus(port, args.motor_ids)
    rows, summaries, fault_reports = [], {}, []
    wire = (args.output/'wire.local.jsonl').open('w')
    def record(row):
        stamped = timestamped(row)
        wire.write(json.dumps(stamped)+'\n'); wire.flush()
        report = fault_register_report(stamped)
        if report is not None:
            fault_reports.append(report)
    audit = AuditedReads(bus, record)
    registers = ('Present_Voltage', 'Status', 'Present_Position', 'Goal_Position',
                 'Torque_Enable', 'Present_Load', 'Present_Current', 'Present_Temperature')
    try:
        bus.connect(); bus.set_timeout(100)
        paths = ('grouped', 'individual') if args.read_path == 'both' else (args.read_path,)
        for mode in paths:
            began = time.monotonic()
            samples = []
            errors_before, invalid_before = audit.servo_error_packets, audit.invalid_packets
            while time.monotonic()-began < args.seconds_per_path:
                started = time.monotonic()
                values, errors = {}, []
                for register in registers:
                    try:
                        values[register] = (audit.read(register) if mode == 'grouped' else
                            {name: audit.read(register, name) for name in bus.motors})
                    except Exception as error:
                        errors.append({'register': register, 'error': str(error)})
                row = timestamped({'path': mode, 'read_s': time.monotonic()-started,
                                   'values': values, 'errors': errors})
                rows.append(row); samples.append(row)
                time.sleep(max(0., .25-(time.monotonic()-started)))
            voltage = [v for row in samples for v in row['values'].get('Present_Voltage', {}).values()]
            summaries[mode] = {'samples': len(samples), 'elapsed_s': time.monotonic()-began,
                'mean_read_s': sum(r['read_s'] for r in samples)/len(samples),
                'error_samples': sum(bool(r['errors']) for r in samples),
                'servo_error_packets': audit.servo_error_packets-errors_before,
                'invalid_packets': audit.invalid_packets-invalid_before,
                'nonzero_status_samples': sum(any(r['values'].get('Status', {}).values()) for r in samples),
                'voltage_raw_range': [min(voltage), max(voltage)] if voltage else None}
    finally:
        if bus.is_connected: bus.disconnect(disable_torque=False)
        audit.restore(); wire.close(); lock.close()
        (args.output/'samples.local.json').write_text(json.dumps(rows, indent=2)+'\n')
        result = {'paths': summaries, 'motor_register_writes': 0,
            'selected_motor_ids': args.motor_ids, 'setup_label': args.setup_label,
            'physical_wiring_verified_by_software': False,
            'range_note': 'voltage_raw_range includes accepted reads only; fault packet values are separate diagnostic evidence',
            'fault_register_reports': fault_reports,
            'blocked_non_read_packets': audit.blocked_non_read_packets,
            'last': rows[-1] if rows else None, 'automatic_motion_restart': False}
        (args.output/'summary.local.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))


def fault_register_report(row):
    """Retain validated fault payloads for diagnosis, never for motion feedback.

    For example a 3.8 V response with its voltage error bit set is rejected
    by CheckedReads. Reporting only accepted 4.8–4.9 V reads would conceal
    this evidence from the diagnostic summary.
    """
    if row.get('error_category') != 'servo_reported_error':
        return None
    context = row.get('context') or {}
    if context.get('register') not in ('Present_Voltage', 'Status'):
        return None
    from so101_mcp.servo_diagnostics import validate_feedback_packet, ServoReportedError
    packet = row.get('packet', [])
    try:
        validate_feedback_packet(packet, 1, context.get('motor_ids', []))
    except ServoReportedError:
        return {'time_utc': row.get('time_utc'), 'motor_id': packet[2],
                'register': context['register'], 'error_flags': packet[4],
                'reported_raw': packet[5], 'accepted_control_feedback': False}
    except ValueError:
        pass
    return None


if __name__ == '__main__':
    main()
