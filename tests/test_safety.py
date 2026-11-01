import copy
import json
import time
from types import SimpleNamespace

import pytest

from so101_mcp.config import load_config
from so101_mcp.controller import Controller
from so101_mcp.joints import ARM, JointMap, Pose, interpolate, safe_duration, validate_limits
from so101_mcp.real import RealBackend


def pose(angle=0, grip=.5):
    return Pose(dict.fromkeys(ARM, angle), grip)


def wait_for(controller, predicate, timeout=2):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        status = controller.get_status()
        if predicate(status):
            return status
        time.sleep(.005)
    raise AssertionError(controller.get_status())


class FakeBackend:
    mode = 'dry_run'

    def __init__(self):
        self.current = pose()
        self.commands = []
        self.halts = 0
        self.fail = False
        self.target = pose(2)

    def connect(self): pass
    def close(self): pass
    def state(self): return self.current
    def capture(self): return b'png'
    def plan(self, name, args): return [('test', self.target, .1)]
    def command(self, target):
        if self.fail:
            raise OSError('injected link failure')
        self.commands.append(target)
    def advance(self, dt): pass
    def halt(self): self.halts += 1


@pytest.fixture
def local(tmp_path):
    config = load_config('real')
    config['speed_deg_s'] = 30
    backend = FakeBackend()
    controller = Controller(backend, config, tmp_path, realtime=True)
    yield controller, backend
    controller.close()


def start(controller, rid='one', oid=None):
    oid = oid or controller.observe()[0]['observation_id']
    return controller.execute_skill('pick_place', {'slot': 'slot_a', 'destination': 'box'}, oid, 'red block', rid)


def test_joint_map_roundtrip_sign_offsets_and_gripper():
    config = load_config('real')['mapping']
    config['sign']['elbow_flex'] = -1
    config['offset_deg']['wrist_flex'] = 17
    mapper = JointMap(config)
    original = pose(23, .25)
    out = mapper.to_lerobot(original)
    assert out['elbow_flex.pos'] == -23
    assert out['wrist_flex.pos'] == 40
    assert out['gripper.pos'] == 25
    assert mapper.from_lerobot(out) == original


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_rejected(bad):
    with pytest.raises(ValueError): pose(bad)
    with pytest.raises(ValueError): pose(0, bad)


@pytest.mark.parametrize('bad', [-.01, 1.01])
def test_gripper_range_rejected(bad):
    with pytest.raises(ValueError): pose(0, bad)


def test_interpolation_obeys_peak_velocity():
    a, b = pose(-50, 0), pose(50, 1)
    duration = safe_duration(a, b, .1, 10, .25)
    dt = .01
    prev = a
    for step in range(1, round(duration/dt)+1):
        current = interpolate(a, b, step*dt/duration)
        assert max(abs(current.joints_deg[n]-prev.joints_deg[n])/dt for n in ARM) <= 10.0001
        assert abs(current.gripper_open-prev.gripper_open)/dt <= .25001
        prev = current


def test_limit_rejects_before_first_command(local):
    controller, backend = local
    backend.target = pose(999)
    with pytest.raises(ValueError, match='outside'): start(controller)
    assert backend.commands == []


def test_dry_run_and_task_success_are_distinct(local):
    controller, backend = local
    action = start(controller)
    s = wait_for(controller, lambda s: s['state'] == 'completed')
    assert s['last_action']['command_status'] == 'dry_run'
    assert s['last_action']['task_success'] == 'unverified'
    assert controller.observe()[0]['joints_deg'] == pose().joints_deg


def test_idempotent_retries_and_stale_images(local):
    controller, backend = local
    oid = controller.observe()[0]['observation_id']
    a = start(controller, oid=oid)
    b = start(controller, oid=oid)
    assert a['action_id'] == b['action_id'] and b['idempotent_replay']
    wait_for(controller, lambda s: s['state'] == 'completed')
    with pytest.raises(ValueError, match='moved'): start(controller, 'two', oid)


def test_stop_is_local_latched_and_prevents_commands(local):
    controller, backend = local
    backend.target = pose(50)
    start(controller)
    began = time.monotonic()
    controller.stop()
    wait_for(controller, lambda s: s['state'] == 'stopped')
    assert time.monotonic()-began < .2
    count = len(backend.commands)
    time.sleep(.08)
    assert len(backend.commands) == count
    assert backend.halts >= 1
    with pytest.raises(RuntimeError, match='latched'): start(controller, 'again')


def test_stop_file_does_not_depend_on_mcp(local):
    controller, backend = local
    backend.target = pose(50)
    start(controller)
    controller.stop_path.touch()
    s = wait_for(controller, lambda s: s['state'] == 'stopped')
    assert s['stop_latched']
    assert s['last_action']['command_status'] == 'stopped'


def test_link_failure_fails_and_latches_stop(local):
    controller, backend = local
    backend.fail = True
    start(controller)
    s = wait_for(controller, lambda s: s['stop_latched'])
    assert s['last_action']['command_status'] == 'failed'
    assert 'OSError' in s['last_action']['error']


def test_pre_action_image_is_not_a_success_verdict(local):
    controller, _ = local
    oid = controller.observe()[0]['observation_id']
    action = start(controller, oid=oid)
    wait_for(controller, lambda s: s['state'] == 'completed')
    with pytest.raises(ValueError, match='post-action'):
        controller.report_result(action['action_id'], oid, 'success', 'unsupported')
    post = controller.observe()[0]['observation_id']
    assert controller.report_result(action['action_id'], post, 'uncertain', 'occluded')['ai_visual_verdict'] == 'uncertain'


def test_real_dry_run_has_zero_goal_writes(tmp_path):
    config = load_config('real')
    real = RealBackend(config, tmp_path)
    calls = []
    real.robot = SimpleNamespace(send_action=lambda target: calls.append(target),
                                 bus=SimpleNamespace(is_connected=True))
    real.command(pose())
    real.halt()
    assert calls == [] and real.writes == 0


def test_initialization_failure_has_stage_and_private_diagnostic(tmp_path):
    backend = FakeBackend()
    backend.connection_stage = 'camera'
    def fail_connect():
        raise ConnectionError('private diagnostic device/path')
    backend.connect = fail_connect
    controller = Controller(backend, load_config('real'), tmp_path)
    try:
        status = controller.get_status()
        assert status['state'] == 'failed'
        assert status['initialization_stage'] == 'camera'
        assert 'private diagnostic' not in json.dumps(status)
        path = tmp_path/'initialization_error.local.json'
        assert json.loads(path.read_text())['error'] == 'private diagnostic device/path'
        assert path.stat().st_mode & 0o777 == 0o600
        assert 'private diagnostic' not in (tmp_path/'events.jsonl').read_text()
    finally:
        controller.close()


def test_motionless_one_degree_action_fails_after_bounded_local_settling(tmp_path):
    backend=FakeBackend();backend.mode='motion_enabled';backend.target=pose(1.)
    c=Controller(backend,load_config('real'),tmp_path)
    try:
        start(c)
        status=wait_for(c,lambda s:s['stop_latched'],timeout=4.)
        assert status['last_action']['command_status']=='failed'
        assert 'limit 0.50' in status['last_action']['error']
        assert status['last_action']['trajectory_s']<3.
        assert backend.halts>=1
    finally:c.close()
