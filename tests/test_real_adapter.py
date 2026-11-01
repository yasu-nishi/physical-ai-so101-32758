from types import SimpleNamespace
import pytest

from so101_mcp.config import load_config
from so101_mcp.joints import ARM, Pose
from so101_mcp.real import RealBackend


def test_real_observation_connection_never_configures_or_enables_motors(tmp_path, monkeypatch):
    import lerobot.robots.so_follower.so_follower as module
    calls = []
    class Bus:
        is_connected = False
        is_calibrated = True
        def connect(self): self.is_connected = True; calls.append('connect')
        def set_timeout(self, value): calls.append('host_timeout')
        def sync_read(self, key):
            calls.append('read')
            return {**dict.fromkeys(ARM, 1.), 'gripper': 20.}
        def disconnect(self, disable_torque):
            assert disable_torque is False
            self.is_connected = False
            calls.append('disconnect')
        def enable_torque(self): raise AssertionError('unexpected torque write')
        def disable_torque(self): raise AssertionError('unexpected torque write')
    class Camera:
        is_connected = False
        def connect(self): self.is_connected = True
        def disconnect(self): self.is_connected = False
    class Robot:
        def __init__(self, config):
            assert config.use_degrees
            assert config.cameras['workspace'].width == 800
            assert config.cameras['workspace'].height == 600
            assert config.cameras['workspace'].fps == 25
            self.bus = Bus()
            self.cameras = {'workspace': Camera()}
            self.calibration = {n: SimpleNamespace(range_min=0, range_max=4095) for n in (*ARM, 'gripper')}
        def configure(self): raise AssertionError('unexpected motor configure')
        def send_action(self, action): raise AssertionError('unexpected goal write')
    port = tmp_path/'serial'
    port.touch()
    monkeypatch.setenv('SO101_FOLLOWER_PORT', str(port))
    monkeypatch.setenv('SO101_FOLLOWER_ID', 'test_arm')
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    monkeypatch.setenv('SO101_CAMERA_WIDTH', '800')
    monkeypatch.setenv('SO101_CAMERA_HEIGHT', '600')
    monkeypatch.setenv('SO101_CAMERA_FPS', '25')
    monkeypatch.setattr(module, 'SO101Follower', Robot)
    backend = RealBackend(load_config('real'), tmp_path)
    backend.connect()
    assert backend.state().gripper_open == .2
    backend.command(backend.state())
    backend.halt()
    backend.close()
    assert calls == ['connect', 'host_timeout', 'read', 'read', 'disconnect']


def test_joints_only_never_opens_camera_or_writes_and_cannot_enable_motion(tmp_path, monkeypatch):
    import pytest
    import lerobot.robots.so_follower.so_follower as module
    calls=[]
    class Bus:
        is_connected=False
        is_calibrated=True
        def connect(self): self.is_connected=True; calls.append('connect')
        def set_timeout(self, value): pass
        def sync_read(self, key): return {**dict.fromkeys(ARM,2.),'gripper':25.}
        def disconnect(self, disable_torque):
            assert not disable_torque
            self.is_connected=False; calls.append('disconnect')
        def enable_torque(self): raise AssertionError('motor write')
        def disable_torque(self): raise AssertionError('motor write')
    class Robot:
        def __init__(self, config):
            assert not config.cameras
            self.cameras={}; self.bus=Bus()
            self.calibration={n:SimpleNamespace(range_min=0,range_max=4095) for n in (*ARM,'gripper')}
        def configure(self): raise AssertionError('motor write')
        def send_action(self, action): raise AssertionError('motor write')
    port=tmp_path/'serial';port.touch()
    monkeypatch.setenv('SO101_FOLLOWER_PORT',str(port))
    monkeypatch.setenv('SO101_FOLLOWER_ID','test_arm')
    monkeypatch.setenv('SO101_CAMERA_INDEX','not-a-camera')
    monkeypatch.setattr(module,'SO101Follower',Robot)
    config=load_config('real')
    draft=tmp_path/'unfinished.json';draft.write_text('{}')
    config['rgb_calibration_file']=str(draft)
    backend=RealBackend(config,tmp_path)
    try:
        backend.connect(cameras=False)
        assert backend.state().gripper_open==.25
        with pytest.raises(RuntimeError,match='joints-only'): backend.capture()
        backend.halt()
        assert backend.writes==0
    finally: backend.close()
    assert calls==['connect','disconnect']
    with pytest.raises(ValueError,match='cannot enable'):
        RealBackend(config,tmp_path,enable_motion=True).connect(cameras=False)
    assert calls==['connect','disconnect']


def _motion_backend(tmp_path, monkeypatch, torque):
    import lerobot.robots.so_follower.so_follower as module
    log = []
    class Bus:
        is_connected = False
        is_calibrated = True
        present = {**dict.fromkeys(ARM, 1.), 'gripper': 20.}
        goals = {**dict.fromkeys(ARM, 1.), 'elbow_flex': 2.8, 'gripper': 20.}
        def connect(self): self.is_connected = True
        def set_timeout(self, value): pass
        def sync_read(self, key, normalize=True):
            if key == 'Torque_Enable': return dict(torque)
            if key == 'Goal_Position': return dict(self.goals)
            return dict(self.present)
        def write(self, key, name, value, normalize=True):
            log.append((key, name, value))
            if key == 'Goal_Position': self.goals = {**self.goals, name: value}
        def sync_write(self, key, values):
            assert key == 'Goal_Position'
            self.robot.actions.append({f'{k}.pos':v for k,v in values.items()})
            self.goals = {**self.goals, **values}
            log.append('goal_write')
        def disconnect(self, disable_torque): assert disable_torque is False
        def enable_torque(self): log.append('enable_torque')
        def disable_torque(self): raise AssertionError('torque must never be dropped')
    class Robot:
        def __init__(self, config):
            self.bus = Bus(); self.cameras = {}
            self.config = config
            self.bus.robot = self
            self.actions = []
            self.calibration = {n: SimpleNamespace(range_min=0, range_max=4095) for n in (*ARM, 'gripper')}
        def configure(self): raise AssertionError('configure drops torque and resets gains')
        def send_action(self, action):
            log.append('seed')
            self.actions.append(dict(action))
            self.bus.goals = {**self.bus.goals, **{k.removesuffix('.pos'): v for k, v in action.items()}}
            return action
    port = tmp_path/'serial'; port.touch()
    monkeypatch.setenv('SO101_FOLLOWER_PORT', str(port))
    monkeypatch.setenv('SO101_FOLLOWER_ID', 'test_arm')
    monkeypatch.setattr(module, 'SO101Follower', Robot)
    config = load_config('real')
    config['servo_runtime'] = {'goal_velocity': {'elbow_flex': 0}, 'p_coefficient': {'elbow_flex': 32}}
    backend = RealBackend(config, tmp_path, enable_motion=True)
    backend._validate_motion_setup = lambda: None
    return backend, log



def test_motion_connect_on_energized_arm_keeps_torque_and_applies_gains(tmp_path, monkeypatch):
    backend, log = _motion_backend(tmp_path, monkeypatch, {**dict.fromkeys(ARM, 1), 'gripper': 0})
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0'); monkeypatch.setenv('SO101_MCP_CAMERA_INDEX', '0')
    backend.connect()
    assert log == [('Goal_Position', 'gripper', 20.), ('Goal_Velocity', 'elbow_flex', 0), ('P_Coefficient', 'elbow_flex', 32),
                   ('Torque_Enable', 'gripper', 1)]
    assert backend.robot.actions == []
    assert backend.robot.bus.goals['elbow_flex'] == 2.8


def test_motion_connect_on_unpowered_arm_seeds_before_enable(tmp_path, monkeypatch):
    backend, log = _motion_backend(tmp_path, monkeypatch, {**dict.fromkeys(ARM, 0), 'gripper': 0})
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0'); monkeypatch.setenv('SO101_MCP_CAMERA_INDEX', '0')
    backend.connect()
    assert log[0] == 'seed' and log[-1] == 'enable_torque'


def test_motion_connect_rejects_partially_energized_arm(tmp_path, monkeypatch):
    import pytest
    backend, log = _motion_backend(tmp_path, monkeypatch, {**dict.fromkeys(ARM, 0), 'elbow_flex': 1, 'gripper': 0})
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0'); monkeypatch.setenv('SO101_MCP_CAMERA_INDEX', '0')
    with pytest.raises(ValueError, match='Partially'):
        backend.connect()
    assert log == []


def test_energized_hold_survives_connect_first_tick_and_idle_close(tmp_path, monkeypatch):
    import pytest
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    backend.connect()
    assert backend.robot.actions == []
    start = backend.state()
    target = Pose({**start.joints_deg, 'elbow_flex': .5}, start.gripper_open)
    backend.begin_motion([('retreat', start, target, 1.)])
    backend.command(start)
    assert backend.robot.actions[-1]['elbow_flex.pos'] == pytest.approx(2.8)
    backend.command(Pose({**start.joints_deg, 'elbow_flex': .9}, .2))
    assert backend.robot.actions[-1]['elbow_flex.pos'] == pytest.approx(2.7)
    backend.end_motion()
    count = len(backend.robot.actions)
    backend.halt()
    backend.close()
    assert len(backend.robot.actions) == count


def test_distant_held_goal_is_rejected_without_writes(tmp_path, monkeypatch):
    import pytest
    backend, log = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    backend.connect()
    before = list(log)
    backend.robot.bus.goals['elbow_flex'] = 4.
    start = backend.state()
    with pytest.raises(ValueError, match='more than 2'):
        backend.begin_motion([('bad', start, start, 1.)])
    assert log == before


def test_hold_offset_cannot_push_target_beyond_limit(tmp_path, monkeypatch):
    import pytest
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    backend.connect()
    start = backend.state()
    target = Pose({**start.joints_deg, 'elbow_flex': backend.config['limits_deg']['elbow_flex'][1]}, .2)
    with pytest.raises(ValueError, match='outside'):
        backend.begin_motion([('bad', start, target, 1.)])
    assert not backend.robot.actions


def test_active_stop_keeps_bounded_load_offset_and_is_idempotent(tmp_path, monkeypatch):
    import pytest
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    backend.connect()
    start = backend.state()
    target = Pose({**start.joints_deg, 'elbow_flex': .5}, .2)
    backend.begin_motion([('retreat', start, target, 1.)])
    backend.robot.bus.present['elbow_flex'] = .8
    backend.halt()
    assert backend.robot.actions[-1]['elbow_flex.pos'] == pytest.approx(2.6)
    count = len(backend.robot.actions)
    backend.halt()
    assert len(backend.robot.actions) == count


def test_nominal_clamp_does_not_consume_bounded_hold_offset(tmp_path, monkeypatch):
    import pytest
    from so101_mcp.joints import interpolate
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    backend.connect()
    start = backend.state()
    target = Pose({**start.joints_deg, 'elbow_flex': 2.}, .2)
    backend.begin_motion([('small_step', start, target, .6)])
    previous = 2.8
    for tick in range(1, 13):
        sent = backend.command(interpolate(start, target, tick/12))
        assert abs(sent.joints_deg['elbow_flex']-1.-1.8) <= 2.
        assert abs(sent.joints_deg['elbow_flex']-1.) <= 3.8+1e-8
        assert abs(sent.joints_deg['elbow_flex']-previous) <= backend.config['speed_deg_s']/backend.config['control_hz']+1e-6
        previous = sent.joints_deg['elbow_flex']
    assert previous == pytest.approx(3.8)
    assert backend.state().joints_deg['elbow_flex'] == 1.  # Never fake measured movement.
    assert backend.robot.config.max_relative_target['elbow_flex'] == 2.
    assert (tmp_path/'servo_commands.local.jsonl').exists()


def test_negative_elbow_step_can_overcome_existing_hold_gap(tmp_path,monkeypatch):
    """Simple constant-load plant, not a claim about real actuator dynamics.

    A compensated goal clamped to +/-2 degrees cannot lift a joint whose
    equilibrium already requires a 23-tick negative hold error.
    """
    import pytest
    from so101_mcp.joints import interpolate
    backend,_=_motion_backend(tmp_path,monkeypatch,dict.fromkeys((*ARM,'gripper'),1))
    monkeypatch.setenv('SO101_CAMERA_INDEX','0');backend.connect()
    bus=backend.robot.bus
    gap=23*backend.ENCODER_TICK_DEG
    bus.present['elbow_flex']=81.45
    bus.goals['elbow_flex']=81.45-gap
    start=backend.state();target=Pose({**start.joints_deg,'elbow_flex':80.45},.2)
    original_write=bus.sync_write
    def loaded_write(key,values):
        original_write(key,values)
        equilibrium=values['elbow_flex']+gap
        bus.present['elbow_flex']+=.55*(equilibrium-bus.present['elbow_flex'])
    bus.sync_write=loaded_write
    context=backend.begin_motion([('elbow_lift',start,target,.8)])
    assert context['maximum_servo_error_deg']['elbow_flex']==pytest.approx(gap+2)
    old_goal=start.joints_deg['elbow_flex']-gap
    for tick in range(1,17):
        sent=backend.command(interpolate(start,target,tick/16))
        assert abs(sent.joints_deg['elbow_flex']-old_goal)<=backend.config['speed_deg_s']/backend.config['control_hz']+1e-8
        old_goal=sent.joints_deg['elbow_flex']
    assert backend.state().joints_deg['elbow_flex']==pytest.approx(80.45,abs=.06)
    assert min(a['elbow_flex.pos'] for a in backend.robot.actions)<start.joints_deg['elbow_flex']-2.5
    # Still fail before sending an unbounded hold offset or excessive rate.
    backend._hold_offset_deg['elbow_flex']=-3
    count=len(backend.robot.actions)
    with pytest.raises(ValueError,match='compensation'):
        backend.command(target)
    assert len(backend.robot.actions)==count


def test_held_goal_one_tick_roundoff_is_accepted_but_not_large_error(tmp_path,monkeypatch):
    import pytest
    backend,_=_motion_backend(tmp_path,monkeypatch,dict.fromkeys((*ARM,'gripper'),1))
    monkeypatch.setenv('SO101_CAMERA_INDEX','0');backend.connect()
    start=backend.state()
    backend.robot.bus.goals['elbow_flex']=1.-23*backend.ENCODER_TICK_DEG
    backend.begin_motion([('bounded',start,start,1.)])
    backend.robot.bus.goals['elbow_flex']=1.-24*backend.ENCODER_TICK_DEG
    with pytest.raises(ValueError,match='more than 2'):
        backend.begin_motion([('too_far',start,start,1.)])


@pytest.mark.parametrize('joint,gap_ticks,initial',[
    ('elbow_flex',23,81.45),('wrist_flex',1,34.20)])
def test_bounded_feedback_moves_stiction_fixture_and_releases_extra_effort(tmp_path,monkeypatch,joint,gap_ticks,initial):
    from so101_mcp.joints import interpolate
    backend,_=_motion_backend(tmp_path,monkeypatch,dict.fromkeys((*ARM,'gripper'),1))
    backend.feedback_gain[joint]=4.
    monkeypatch.setenv('SO101_CAMERA_INDEX','0');backend.connect()
    bus=backend.robot.bus;gap=gap_ticks*backend.ENCODER_TICK_DEG
    bus.present[joint]=initial;bus.goals[joint]=initial-gap
    moving=False
    original=bus.sync_write
    def friction_fixture(key,values):
        nonlocal moving
        original(key,values)
        drive=values[joint]+gap-bus.present[joint]
        if abs(drive)>1.1:moving=True
        if moving:
            amount=max(-50*backend.ENCODER_TICK_DEG/backend.config['control_hz'],
                       min(50*backend.ENCODER_TICK_DEG/backend.config['control_hz'],drive*.35))
            bus.present[joint]+=amount
            if abs(drive)<.06:moving=False
    bus.sync_write=friction_fixture
    start=backend.state();target=Pose({**start.joints_deg,joint:initial-1.},.2)
    backend.begin_motion([('one_joint',start,target,.8)])
    previous=start.joints_deg[joint]-gap
    for tick in range(1,25):
        sent=backend.command(interpolate(start,target,min(1.,tick/16)))
        assert abs(sent.joints_deg[joint]-previous)<=backend.config['speed_deg_s']/backend.config['control_hz']+1e-8
        assert initial-1.35<=bus.present[joint]<=initial+.05
        previous=sent.joints_deg[joint]
    assert backend.state().joints_deg[joint]==pytest.approx(initial-1.,abs=.35)
    backend.end_motion()
    assert bus.goals[joint]-bus.present[joint]==pytest.approx(-gap)
    again=backend.state()
    backend.begin_motion([('next_step',again,again,1.)])  # no adoption of temporary feedback effort


def test_invalid_feedback_gain_is_rejected_before_connection(tmp_path):
    import pytest
    for gain in (float('nan'),float('inf'),0.,4.1,True):
        cfg=load_config('real');cfg['servo_feedback']={'joint_error_gain':{'elbow_flex':gain}}
        with pytest.raises(ValueError,match='feedback gain'):RealBackend(cfg,tmp_path)


def test_actual_goal_rate_guard_still_rejects_excessive_change(tmp_path, monkeypatch):
    import pytest
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0')
    backend.connect()
    start = backend.state()
    backend.begin_motion([('step', start, start, 1.)])
    backend._last_goal = Pose({**start.joints_deg, 'elbow_flex': 1.}, .2)
    with pytest.raises(ValueError, match='servo-goal speed'):
        backend.command(start)
    assert backend.robot.actions == []


def test_repeated_goal_suppression_keeps_other_joints_and_verifies_registers(tmp_path, monkeypatch):
    import pytest
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0'); backend.connect()
    backend.suppress_repeated_goals = True
    backend.verify_servo_io = True
    start = backend.state()
    target = Pose({**start.joints_deg, 'elbow_flex': .5}, .2)
    backend.begin_motion([('step', start, target, 1.)])
    backend.command(start)
    assert backend.robot.actions == []  # Existing hold is not rewritten.
    nominal = Pose({**start.joints_deg, 'elbow_flex': .9}, .2)
    backend.command(nominal)
    assert backend.robot.actions == [{'elbow_flex.pos': pytest.approx(2.7)}]
    for _ in range(40): backend.command(nominal)
    assert len(backend.robot.actions) == 1
    # An unexpected register change is detected even on a skipped write.
    backend.robot.bus.goals['elbow_flex'] = 1.
    with pytest.raises(RuntimeError, match='readback'):
        backend.command(nominal)
    assert len(backend.robot.actions) == 1


def test_lost_broadcast_write_is_failure_not_command_success(tmp_path, monkeypatch):
    import pytest
    backend, _ = _motion_backend(tmp_path, monkeypatch, dict.fromkeys((*ARM, 'gripper'), 1))
    monkeypatch.setenv('SO101_CAMERA_INDEX', '0'); backend.connect()
    backend.verify_servo_io = True
    start = backend.state()
    backend.begin_motion([('step', start, start, 1.)])
    backend.robot.bus.sync_write = lambda *args, **kwargs: None
    with pytest.raises(RuntimeError, match='readback'):
        backend.command(Pose({**start.joints_deg, 'elbow_flex': .6}, .2))


@pytest.mark.parametrize('measured_jaw,accepted_jaw',[(.428,.4405),(.4,.35)])
def test_completed_jaw_retains_accepted_goal_for_next_arm_step(tmp_path,monkeypatch,measured_jaw,accepted_jaw):
    """Lag after opening and contact during closing must not replace the goal."""
    backend,_=_motion_backend(tmp_path,monkeypatch,dict.fromkeys((*ARM,'gripper'),1))
    monkeypatch.setenv('SO101_CAMERA_INDEX','0');backend.connect()
    backend.feedback_gain['wrist_flex']=4.
    bus=backend.robot.bus
    bus.present['gripper']=measured_jaw*100
    bus.goals['gripper']=accepted_jaw*100
    start=backend.state()
    target=Pose(start.joints_deg,accepted_jaw)
    backend.begin_motion([('jaw',start,target,.8)])
    backend.end_motion()
    assert backend.held_gripper_open==pytest.approx(accepted_jaw)
    assert bus.goals['gripper']==pytest.approx(accepted_jaw*100)
    # A subsequent arm-only trajectory must neither release the grasp nor
    # trip the jaw rate guard at its first tick.
    held=Pose(start.joints_deg,backend.held_gripper_open)
    backend.begin_motion([('arm',held,held,.8)])
    backend.command(held)
    assert bus.goals['gripper']==pytest.approx(accepted_jaw*100)
    backend.halt()


def test_interrupted_jaw_still_freezes_measured_opening(tmp_path,monkeypatch):
    backend,_=_motion_backend(tmp_path,monkeypatch,dict.fromkeys((*ARM,'gripper'),1))
    monkeypatch.setenv('SO101_CAMERA_INDEX','0');backend.connect()
    start=backend.state()
    backend.begin_motion([('jaw',start,Pose(start.joints_deg,.3),.8)])
    backend.robot.bus.present['gripper']=23.
    backend.halt()
    assert backend.robot.bus.goals['gripper']==pytest.approx(23.)
    assert backend.robot.bus.goals['elbow_flex']==pytest.approx(2.8)


def test_controller_remembers_capped_contact_goal_not_unsent_target(tmp_path,monkeypatch):
    import io
    import time
    from PIL import Image
    from so101_mcp.controller import Controller
    backend,_=_motion_backend(tmp_path,monkeypatch,dict.fromkeys((*ARM,'gripper'),1))
    monkeypatch.setenv('SO101_CAMERA_INDEX','0')
    backend.feedback_gain['wrist_flex']=4.
    backend.config['incremental_steps']['real_motion_verified']=True
    backend.config['rgb_calibration_file']=str(tmp_path/'no_camera_calibration.json')
    png=io.BytesIO();Image.new('RGB',(640,480),'gray').save(png,format='PNG')
    backend.capture=lambda:png.getvalue()
    c=Controller(backend,backend.config,tmp_path)
    try:
        obs=c.observe()[0]
        p=c.prepare_step(obs['observation_id'],{},-.1,'contact fixture','synthetic image','check held goal')[0]
        c.execute_skill('prepared_step',{'program_id':p['program_id']},p['observation_id'],p['instruction'],'close')
        deadline=time.monotonic()+3
        while c.get_status()['state']=='running' and time.monotonic()<deadline:time.sleep(.01)
        assert c.get_status()['state']=='completed',c.get_status()
        # Static fixture: actual jaw .20, requested .10, LeRobot relative cap .05.
        assert backend.state().gripper_open==pytest.approx(.2)
        assert c._gripper_command==pytest.approx(.15)
        assert backend.held_gripper_open==pytest.approx(.15)
        assert backend.robot.bus.goals['gripper']==pytest.approx(15.)
    finally:c.close()
