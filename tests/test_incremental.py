"""Fresh-image gates and bounded one-step generation, including real dry-run."""
import ast
import io
import time
from types import SimpleNamespace

import pytest
from PIL import Image

from so101_mcp.config import load_config
from so101_mcp.controller import Controller
from so101_mcp.incremental import step_target
from so101_mcp.joints import ARM, Pose
from so101_mcp.real import RealBackend
from so101_mcp.sim import SimBackend


def prepare(c, oid, delta, grip=0.):
    return c.prepare_step(oid, delta, grip, 'software test, not AI perception',
                          'synthetic test evidence', 'inspect a new image before any further step')[0]


def execute(c, program, request_id):
    return c.execute_skill('prepared_step', {'program_id': program['program_id']},
                           program['observation_id'], program['instruction'], request_id)


def completed(c):
    deadline = time.monotonic()+3
    while c.get_status()['state'] == 'running' and time.monotonic() < deadline:
        time.sleep(.01)
    status = c.get_status()
    assert status['state'] == 'completed', status
    return status


@pytest.mark.parametrize('delta,grip', [({'shoulder_pan': 2.01}, 0.),
    ({'shoulder_pan': 2., 'shoulder_lift': 2., 'elbow_flex': 1.}, 0.),
    ({}, .101), ({'wrist_roll': .1}, .01), ({'z_m': .001}, 0.),
    ({'elbow_flex': float('nan')}, 0.), ({'elbow_flex': True}, 0.), ({}, 0.)])
def test_invalid_increment_rejected(delta, grip):
    current = Pose(dict.fromkeys(ARM, 0.), .5)
    with pytest.raises(ValueError):
        step_target(current, delta, grip, load_config('sim')['incremental_steps'])


def test_sim_one_step_generation_reobservation_and_stop(tmp_path):
    cfg = load_config('sim')
    backend = SimBackend(cfg, tmp_path, record=False)
    c = Controller(backend, cfg, tmp_path)
    try:
        before = c.observe()[0]
        p = prepare(c, before['observation_id'], {'shoulder_pan': 2.})
        other = prepare(c, before['observation_id'], {'shoulder_pan': -2.})
        assert backend.elapsed == 0.  # preparation must not actuate
        assert len(p['trajectory']) == 1 and not p['automatic_next_step']
        assert p['height_measured'] is False
        source = (tmp_path/p['files_relative_to_run_dir']['script']).read_text()
        ast.parse(source)
        assert "'name': 'prepared_step'" in source and "call('observe')" in source
        with pytest.raises(ValueError, match='skill type'):
            c.execute_skill('prepared_pick', {'program_id': p['program_id']},
                             p['observation_id'], p['instruction'], 'wrong-skill')
        a = execute(c, p, 'first')
        completed(c)
        elapsed = backend.elapsed
        assert execute(c, p, 'first')['action_id'] == a['action_id']
        assert backend.elapsed == elapsed  # transport retry cannot repeat a step
        with pytest.raises(ValueError, match='moved'):
            execute(c, other, 'pre-generated-next')
        with pytest.raises(ValueError, match='moved'):
            prepare(c, before['observation_id'], {'shoulder_pan': 1.})
        after = c.observe()[0]
        assert after['joints_deg']['shoulder_pan'] > before['joints_deg']['shoulder_pan']+1.
        assert after['status']['task_success'] == 'unverified'
        q = prepare(c, after['observation_id'], {'shoulder_pan': -1.})
        assert q['trajectory'][0]['start']['joints_deg'] == pytest.approx(after['joints_deg'])
        c.stop()
        with pytest.raises(RuntimeError, match='latched'):
            execute(c, q, 'after-stop')
    finally:
        c.close()


class MockCameraReal(RealBackend):
    """Use the actual adapter state/command path with no serial device or camera."""
    def connect(self):
        self.robot = SimpleNamespace(
            bus=SimpleNamespace(is_connected=True,
                sync_read=lambda _: {**dict.fromkeys(ARM, 0.), 'gripper': 50.}),
            send_action=lambda _: (_ for _ in ()).throw(AssertionError('motor write in dry run')))

    def capture(self):
        output = io.BytesIO()
        Image.new('RGB', (640, 480), '#808080').save(output, format='PNG')
        return output.getvalue()

    def close(self): pass


def test_real_step_dry_run_keeps_measured_pose_and_zero_writes(tmp_path):
    cfg = load_config('real')
    backend = MockCameraReal(cfg, tmp_path)
    c = Controller(backend, cfg, tmp_path)
    try:
        before = c.observe()[0]
        assert before['capabilities']['incremental_joint_step']
        p = prepare(c, before['observation_id'], {'wrist_roll': 1.})
        execute(c, p, 'dry')
        status = completed(c)
        assert status['last_action']['command_status'] == 'dry_run'
        assert backend.writes == 0
        after = c.observe()[0]
        assert after['joints_deg'] == before['joints_deg']
        assert after['sensors']['depth']['available'] is False
        assert 'camera_calibration' not in after  # joint-relative control needs no pretend depth/IK
        backend.mode = 'motion_enabled'
        with pytest.raises(ValueError, match='verified and enabled locally'):
            prepare(c, after['observation_id'], {'wrist_roll': 1.})
    finally:
        c.close()


def test_incremental_target_still_obeys_absolute_limits(tmp_path):
    cfg = load_config('real')
    cfg['limits_deg']['shoulder_pan'] = [-1., 1.]
    backend = MockCameraReal(cfg, tmp_path)
    c = Controller(backend, cfg, tmp_path)
    try:
        oid = c.observe()[0]['observation_id']
        with pytest.raises(ValueError, match='outside'):
            prepare(c, oid, {'shoulder_pan': 2.})
        assert backend.last_command is None
        assert not (tmp_path/'programs').exists()
    finally:
        c.close()
