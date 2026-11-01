"""Allowlist export: never publish the working repository or its history wholesale."""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT/'submission/public_repo'
SCRIPTS = ['mcp_call.py', 'real_check.py', 'calibrate_rgb.py', 'check_kinematics.py', 'summarize_experiments.py', 'summarize_continuous.py', 'summarize_sensors.py', 'summarize_rgb_portable.py', 'summarize_incremental.py', 'scenario_suite.py', 'staged_session.py', 'summarize_staged.py', 'build_staged_report.py', 'summarize_scenarios.py', 'build_scenario_report.py', 'build_deliverables.py', 'export_submission.py']
SCRIPTS.append('finalize_staged_submission.py')
SCRIPTS.append('calibrate_rgb_live.py')
SCRIPTS += ['camera_view_session.py', 'summarize_camera_views.py', 'build_camera_review_video.py', 'validate_camera_video.py']
SCRIPTS += ['commission_joints.py', 'record_local_preview.py', 'summarize_real_commissioning.py']
SCRIPTS += ['diagnose_real_hold.py', 'summarize_servo_diagnostics.py']
SCRIPTS += ['compare_servo_reads.py', 'start_real_staged.command']
SCRIPTS += ['verify_api_client.py']
SCRIPTS += ['summarize_api_runs.py']
SCRIPTS += ['build_api_video.py']


def check_private_text(path, relative):
    if path.suffix in ('.png', '.mp4', '.stl', '.pdf', '.zip'):
        return
    try: text = path.read_text()
    except UnicodeDecodeError: return
    # User-owned identity is derived locally; never bake it into public code.
    disallowed = [str(Path.home()), '/Users/'+Path.home().name, '/home/'+Path.home().name]
    if any(value in text for value in disallowed):
        raise ValueError(f'Local personal path detected: {relative}')
    if re.search(r'(?<![A-Za-z0-9_])(?:hf_[A-Za-z0-9]{25,}|sk-[A-Za-z0-9_-]{25,}|ghp_[A-Za-z0-9]{30,})', text):
        raise ValueError(f'Potential credential detected: {relative}')
    if re.search(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----', text):
        raise ValueError(f'Private key detected: {relative}')


def main():
    (ROOT/'submission').mkdir(exist_ok=True)
    if DEST.exists() and not (DEST/'.submission_generated').exists():
        raise SystemExit('Destination is not a generated submission; refusing to overwrite')
    stage = Path(tempfile.mkdtemp(prefix='public_repo_', dir=ROOT/'submission'))
    def copy(relative):
        src, dst = ROOT/relative, stage/relative
        if src.is_dir():
            shutil.copytree(src, dst, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.DS_Store', '*.local*'))
        else:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dst)
    try:
        for name in ['README.md', 'pyproject.toml', 'uv.lock', '.python-version', 'LICENSE', 'THIRD_PARTY_NOTICES.md',
                     'so101_mcp', 'tests', 'docs/physical_ai', 'assets/so101_tabletop', 'assets/fonts', 'config/scene.xml',
                     'config/physical_ai/sim.json', 'config/physical_ai/sim_continuous.json', 'config/physical_ai/real.json', 'config/physical_ai/mcp.example.toml',
                     'config/physical_ai/suite',
                     'config/physical_ai/sim_staged.json', 'config/physical_ai/sim_staged_stack.json',
                     'config/physical_ai/api_agent.json', 'artifacts/api_client_verification.json',
                     'artifacts/api_live_verification.json', 'artifacts/api_evidence',
                     'artifacts/api_video_validation.json',
                     'config/physical_ai/real_staged.json', 'config/physical_ai/staged_cases.json',
                     'config/physical_ai/camera_views',
                     'artifacts/environment.json', 'artifacts/experiments.json', 'artifacts/experiments.csv',
                     'artifacts/real_connection_verification.json',
                     'artifacts/real_commissioning_verification.json',
                     'artifacts/real_supply_verification.json',
                     'artifacts/real_standby_recovery_verification.json',
                     'artifacts/real_speed_configuration_verification.json',
                     'artifacts/real_connector_press_verification.json',
                     'artifacts/real_camera_calibration_verification.json',
                     'artifacts/continuous_experiments.json', 'artifacts/continuous_experiments.csv',
                     'artifacts/sensor_verification.json', 'artifacts/rgb_portable_experiments.json', 'artifacts/incremental_experiments.json',
                     'artifacts/scenario_experiments.json', 'artifacts/scenario_experiments.csv',
                     'artifacts/staged_experiments.json', 'artifacts/staged_experiments.csv', 'artifacts/staged_video_edit.json',
                     'artifacts/staged_video_validation.json',
                     'artifacts/staged_pdf_validation.json',
                     'artifacts/camera_view_experiments.json', 'artifacts/camera_view_experiments.csv',
                     'artifacts/camera_view_review.mp4', 'artifacts/camera_video_edit.json', 'artifacts/camera_video_validation.json',
                     'output/pdf/staged_unknown_height_review.pdf',
                     'artifacts/test_results.json', 'output/pdf/rgb_checkerboard.pdf']:
            copy(name)
        for name in SCRIPTS: copy('scripts/'+name)
        runs = [run for pattern in ('vision_*', 'continuous_*', 'sensors_*', 'rgb_portable_*', 'incremental_*', 'suite_*', 'staged_*', 'camera_*')
                for run in (ROOT/'artifacts/runs').glob(pattern)]
        for run in sorted(runs):
            for f in run.iterdir():
                if f.name in ('events.jsonl', 'evaluation_private.json', 'evaluation_history_private.jsonl',
                              'checkpoint_stack_private.json', 'checkpoints.json', 'simulation.mp4',
                              'run_manifest.json', 'case_result.json', 'stop_probe.json') or (
                                  f.name.startswith(('observe_','measurement_')) and f.suffix in ('.png', '.npz')):
                    copy(str(f.relative_to(ROOT)))
            if (run/'programs').is_dir():
                copy(str((run/'programs').relative_to(ROOT)))
            if run.name.startswith(('suite_','staged_','camera_')) and (run/'client').is_dir():
                copy(str((run/'client').relative_to(ROOT)))
        (stage/'.env.example').write_text(
            '# Local hardware settings only. Never commit .env or calibration data.\n'
            'SO101_FOLLOWER_PORT=/dev/tty.usbmodemEXAMPLE\n'
            'SO101_FOLLOWER_ID=so101_follower\n'
            'SO101_CAMERA_INDEX=0\nSO101_CAMERA_WIDTH=640\nSO101_CAMERA_HEIGHT=480\nSO101_CAMERA_FPS=30\n')
        with (stage/'.env.example').open('a') as stream:
            stream.write('\n# Separate API agent only; keep the real key local.\nOPENAI_API_KEY=\nOPENAI_MODEL=gpt-6-astra\n')
        (stage/'.gitignore').write_text('.env\n.venv/\n__pycache__/\n*.py[cod]\n.pytest_cache/\n*.local*\n.DS_Store\n'
                                     'MUJOCO_LOG.TXT\nsubmission/\ntmp/\n.codex/\n')
        (stage/'.submission_generated').write_text('SO-101 Physical AI allowlist export\n')
        files=[]
        for f in sorted(stage.rglob('*')):
            if f.is_file():
                rel=str(f.relative_to(stage))
                check_private_text(f, rel)
                files.append({'path': rel, 'bytes': f.stat().st_size, 'sha256': hashlib.sha256(f.read_bytes()).hexdigest()})
        for pdf in (stage/'output/pdf').glob('*.pdf'):
            # Strip all date metadata from submitted PDFs (including the
            # calibration worksheet). Older reports remain local history only.
            from pypdf import PdfReader, PdfWriter
            reader=PdfReader(pdf)
            clean_metadata={str(k):str(v) for k,v in (reader.metadata or {}).items()
                            if 'Date' not in str(k)}
            if any('Date' in str(k) for k in (reader.metadata or {})) or '/Metadata' in reader.trailer['/Root']:
                writer=PdfWriter();writer.append(reader);writer.add_metadata(clean_metadata)
                writer.root_object.pop('/Metadata',None)
                temporary=pdf.with_suffix('.clean.pdf');writer.write(temporary);temporary.replace(pdf)
                rel=str(pdf.relative_to(stage))
                entry=next(x for x in files if x['path']==rel)
                entry.update(bytes=pdf.stat().st_size,sha256=hashlib.sha256(pdf.read_bytes()).hexdigest())
            extracted = subprocess.check_output(['pdftotext',str(pdf),'-'],text=True)
            if str(Path.home()) in extracted or re.search(r'[\w.+-]+@[\w.-]+\.[A-Za-z]{2,}', extracted):
                raise ValueError('Personal path or email in PDF text')
            if re.search(r'\b20\d{2}[-/]\d{1,2}[-/]\d{1,2}\b|\b\d{1,2}:\d{2}(?::\d{2})?\b|(?:artifacts|docs|config|output)/',extracted):
                raise ValueError('Calendar timestamp or local file location in submission PDF')
        if DEST.exists(): shutil.rmtree(DEST)
        stage.rename(DEST)
        manifest = {'status': 'review_only_not_published', 'file_count': len(files),
                    'scan': {'local_home_paths': 'none', 'credential_patterns': 'none',
                             'private_keys': 'none', 'pdf_email_patterns': 'none',
                             'pdf_calendar_timestamps': 'none', 'pdf_local_file_locations': 'none',
                             'pdf_date_metadata': 'removed',
                             'scope': 'allowlisted text + extracted PDF; all video frames are simulation renders'},
                    'files': files}
        (ROOT/'submission/review_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        archive=ROOT/'submission/public_repo.zip'
        with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as out:
            for f in sorted(DEST.rglob('*')):
                if f.is_file():out.write(f,'public_repo/'+str(f.relative_to(DEST)))
        print(json.dumps({'files':len(files),'code_archive_bytes':archive.stat().st_size,
                          'scan':manifest['scan']},indent=2))
    finally:
        if stage.exists() and stage != DEST: shutil.rmtree(stage)


if __name__=='__main__': main()
