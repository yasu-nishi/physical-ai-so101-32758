import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import sys

from .config import ROOT, load_config


def parser():
    p = argparse.ArgumentParser(description='SO-101 image-guided MCP skills')
    sub = p.add_subparsers(dest='command', required=True)
    serve = sub.add_parser('serve')
    serve.add_argument('--backend', choices=['sim', 'real'], required=True)
    serve.add_argument('--config', type=Path)
    serve.add_argument('--transport', choices=['stdio', 'streamable-http'], default='stdio')
    serve.add_argument('--port', type=int, default=8765)
    serve.add_argument('--run-dir', type=Path)
    serve.add_argument('--seed', type=int, default=0, help='Simulator initial arrangement; never an MCP observation')
    serve.add_argument('--no-record', action='store_true')
    serve.add_argument('--realtime', action='store_true')
    serve.add_argument('--enable-motion', action='store_true', help='Real only; requires local terminal ARM confirmation')
    stop = sub.add_parser('stop')
    stop.add_argument('--run-dir', type=Path, required=True)
    doctor = sub.add_parser('doctor')
    doctor.add_argument('--output', type=Path)
    return p


def doctor():
    import importlib.metadata
    import os
    import platform
    from serial.tools.list_ports import comports
    from dotenv import dotenv_values
    env = {**dotenv_values(ROOT / '.env'), **os.environ}
    versions = {}
    for name in ['lerobot', 'mujoco', 'mcp', 'numpy', 'scipy', 'opencv-python-headless', 'pillow', 'imageio', 'reportlab']:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = 'not installed'
    port = env.get('SO101_FOLLOWER_PORT', '')
    rid = env.get('SO101_FOLLOWER_ID', '')
    calibration = Path.home()/'.cache/huggingface/lerobot/calibration/robots/so_follower'/f'{rid}.json'
    data = json.loads(calibration.read_text()) if rid and calibration.is_file() else {}
    return {'os': platform.system(), 'os_version': platform.mac_ver()[0], 'architecture': platform.machine(),
            'python': platform.python_version(), 'packages': versions,
            'usb_serial_count': sum(p.vid is not None or any(s in p.device.lower() for s in ('usb', 'wch', 'acm')) for p in comports()),
            'configured_follower_port_exists': bool(port and Path(port).exists()),
            'configured_calibration_exists': bool(data), 'calibrated_joints': sorted(data),
            'robot_camera_verified': False,
            'real_connection': 'unverified', 'real_motion': 'unverified',
            'note': 'Read-only inventory. No motor writes or camera capture; no personal identifiers exported.'}


def main():
    args = parser().parse_args()
    if args.command == 'stop':
        if not args.run_dir.is_dir():
            raise SystemExit('Run directory does not exist')
        (args.run_dir/'STOP').touch()
        print('Local stop requested; inspect status. Restart with a new run directory to resume.')
        return
    if args.command == 'doctor':
        result = json.dumps(doctor(), indent=2)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(result+'\n')
        print(result)
        return
    config = load_config(args.backend, args.config)
    if args.enable_motion:
        if args.backend != 'real' or not sys.stdin.isatty() or args.transport == 'stdio':
            raise SystemExit('Real motion requires an attended terminal and streamable-http transport')
        if input('Clear the workspace and keep the local stop/power switch accessible. Type ARM to enable: ') != 'ARM':
            raise SystemExit('Motion not enabled')
    run_dir = args.run_dir or ROOT/'artifacts/runs'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    run_dir = run_dir.resolve()
    # Never silently overwrite or append to a previous experiment.
    if run_dir.exists() and any(run_dir.iterdir()):
        raise SystemExit('Use an empty/new run directory')
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir/'run_manifest.json').write_text(json.dumps({
        'backend': args.backend, 'seed': args.seed if args.backend == 'sim' else None,
        'config': config, 'llm_calls_inside_server': 0,
        'agent': 'external MCP client; exact model, if available, is recorded by that client',
        'created_utc': datetime.now(timezone.utc).isoformat()}, indent=2)+'\n')
    if args.backend == 'sim':
        from .sim import SimBackend
        backend = SimBackend(config, run_dir, seed=args.seed, record=not args.no_record)
    else:
        from .real import RealBackend
        backend = RealBackend(config, run_dir, enable_motion=args.enable_motion)
    from .controller import Controller
    from .server import make_server
    controller = Controller(backend, config, run_dir, realtime=args.realtime)
    def shutdown(signum, frame):
        controller.stop()
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown)
    print(f'SO-101 {args.backend}, mode={backend.mode}; local STOP file: {run_dir}/STOP', file=sys.stderr)
    try:
        make_server(controller, args.port).run(transport=args.transport)
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()


if __name__ == '__main__':
    main()
