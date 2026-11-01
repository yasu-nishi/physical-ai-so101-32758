"""Attended, read-only real connection / waypoint capture. No goal writes."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_mcp.config import ROOT, load_config
from so101_mcp.real import RealBackend

PHASES = ['approach', 'descend', 'close', 'lift', 'transfer', 'release', 'observe_pose']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path)
    p.add_argument('--output', type=Path, default=Path('artifacts/real.local'))
    p.add_argument('--joints-only', action='store_true', help='Read bus/calibration/joints without opening any camera; cannot teach or move')
    p.add_argument('--skill', choices=['observe_pose', 'pick_place_slot_a', 'pick_place_slot_b'])
    p.add_argument('--phase', choices=PHASES)
    p.add_argument('--duration', type=float, default=4.)
    args = p.parse_args()
    if args.joints_only and args.skill:
        p.error('Joints-only inspection cannot capture taught skills')
    if args.output.exists() and any(args.output.iterdir()):
        p.error('Use a new/empty output directory to preserve earlier inspection evidence')
    if args.skill and (not args.phase or args.duration <= 0):
        p.error('Waypoint capture needs --phase and positive --duration')
    if args.skill == 'observe_pose' and args.phase != 'observe_pose':
        p.error('observe_pose skill must use observe_pose phase')
    config = load_config('real', args.config)
    backend = RealBackend(config, args.output)
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        backend.connect(cameras=not args.joints_only)
        pose = backend.state()
        if not args.joints_only:
            (args.output/'camera.png').write_bytes(backend.capture())
        state = {'connection_verified': True, 'goal_writes': 0, 'real_motion': 'unverified',
                 'camera_frame_acquired': not args.joints_only, 'camera_calibration_verified': backend.rgb_calibration is not None,
                 'inspection_mode': 'joints_only' if args.joints_only else 'joints_and_camera', **pose.as_dict()}
        (args.output/'state.json').write_text(json.dumps(state, indent=2)+'\n')
        print(json.dumps(state, indent=2))
        if args.skill:
            target = ROOT/config['taught_skills_file']
            data = json.loads(target.read_text()) if target.exists() else {
                'backend': 'real', 'units': 'degrees_and_gripper_fraction', 'skills': {}}
            existing = data['skills'].get(args.skill, [])
            existing = [v for v in existing if v['phase'] != args.phase]
            existing.append({'phase': args.phase, 'duration_s': args.duration, 'pose': pose.as_dict()})
            data['skills'][args.skill] = sorted(existing, key=lambda v: PHASES.index(v['phase']))
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(data, indent=2)+'\n')
            print('Saved one measured waypoint. No robot goal was sent.')
    except Exception as error:
        failure = {'connection_verified': False, 'real_motion': 'unverified',
                   'goal_writes': backend.writes, 'inspection_mode': 'joints_only' if args.joints_only else 'joints_and_camera',
                   'error_type': type(error).__name__, 'error': str(error),
                   'note': 'Inspection did not complete; no calibration flags changed'}
        (args.output/'inspection_error.json').write_text(json.dumps(failure, indent=2)+'\n')
        raise SystemExit(json.dumps(failure, indent=2)) from None
    finally:
        backend.close()


if __name__ == '__main__': main()
