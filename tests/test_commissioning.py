"""Motor writes are simulated; these tests never open hardware."""
from types import SimpleNamespace
import pytest
from so101_mcp.commissioning import JointCommissioner, NAMES, validate_step
from so101_mcp.joints import ARM, JointMap


class FakeBus:
    is_calibrated = True
    def __init__(self):
        self.calibration = {n:SimpleNamespace(range_min=0,range_max=4095,drive_mode=0) for n in NAMES}
        defaults = {'Present_Position':2048,'Torque_Enable':0,'Operating_Mode':0,'Goal_Time':0,
                    'Velocity_Unit_factor':50,'Goal_Position':0,'Goal_Velocity':0,
                    'Min_Voltage_Limit':40,'Max_Voltage_Limit':80,'Protection_Current':500,
                    'Present_Voltage':49,'Present_Temperature':27,'Present_Current':0,'Present_Load':0,
                    'Phase':12,'Status':0}
        self.registers={k:dict.fromkeys(NAMES,v) for k,v in defaults.items()}
        self.writes=[]
        self.on_write=None
        self.follow=True
    def sync_read(self, key, normalize=False): return dict(self.registers[key])
    def write(self, key, n, value, normalize=False):
        self.writes.append((key,n,value))
        self.registers[key][n]=value
        if key=='Goal_Position' and self.registers['Torque_Enable'][n] and self.follow:
            self.registers['Present_Position'][n]=value
        if self.on_write: self.on_write(key,n,value)


@pytest.fixture
def setup(tmp_path,monkeypatch):
    clock=[100.]
    monkeypatch.setattr('so101_mcp.commissioning.time.monotonic',lambda:clock[0])
    def sleep(seconds): clock[0]+=max(0.,seconds)
    bus=FakeBus()
    mapping=JointMap({'sign':dict.fromkeys(ARM,1),'offset_deg':dict.fromkeys(ARM,0)})
    limits=dict.fromkeys(ARM,[-97.,97.])
    controller=JointCommissioner(bus,mapping,limits,tmp_path,lambda:True,sleep)
    return controller,bus


def test_read_only_no_writes(setup):
    c,b=setup;c.preflight()
    assert not b.writes and not c.armed


def test_checked_feedback_failure_before_arming_never_writes(setup):
    from so101_mcp.servo_diagnostics import ServoReportedError
    c,b=setup
    def checked(register):raise ServoReportedError('Servo error flags in response: 1')
    c.read_feedback=checked
    with pytest.raises(ServoReportedError):c.arm()
    assert not b.writes and not c.armed


def test_checked_feedback_failure_stops_interpolation_before_next_goal(setup):
    from so101_mcp.servo_diagnostics import ServoReportedError
    c,b=setup;c.arm();before=len(b.writes)
    def checked(register):raise ServoReportedError('Servo error flags in response: 1')
    c.read_feedback=checked
    with pytest.raises(ServoReportedError):c.step({'joint':'elbow_flex','delta_deg':-1.5})
    assert len(b.writes)==before
    c.halt('servo error test')
    assert c.latched and 'hold_error' in c.last_result and c.phase=='stopped_hold_unverified'
    assert len(b.writes)==before  # Unreadable pose cannot become a new hold goal.


def test_startup_seeds_every_goal_before_any_torque_and_never_touches_gripper(setup):
    c,b=setup;c.arm()
    first=next(i for i,w in enumerate(b.writes) if w[0]=='Torque_Enable')
    assert {n for key,n,v in b.writes[:first] if key=='Goal_Position' and v==2048}==set(ARM)
    assert all(n!='gripper' for _,n,_ in b.writes)
    assert b.registers['Torque_Enable']['gripper']==0
    assert c.torque_writes==5 and c.goal_writes==5


def test_seed_readback_failure_cannot_enable(setup):
    c,b=setup
    def corrupt(key,n,v):
        if key=='Goal_Position': b.registers[key][n]=0
    b.on_write=corrupt
    with pytest.raises(ValueError,match='readback'): c.arm()
    assert not c.armed and not any(key=='Torque_Enable' for key,_,_ in b.writes)


@pytest.mark.parametrize('register,value', [('Torque_Enable',1),('Operating_Mode',1),
    ('Goal_Time',10),('Velocity_Unit_factor',1),('Present_Voltage',40),
    ('Present_Current',300),('Present_Temperature',56)])
def test_preflight_guards_write_nothing(setup,register,value):
    c,b=setup;b.registers[register]['elbow_flex']=value
    with pytest.raises(ValueError): c.arm()
    assert not b.writes


def test_stop_during_seeding_prevents_next_write_and_enable(setup):
    c,b=setup;b.on_write=lambda *args:c.stop_path.touch()
    with pytest.raises(ValueError,match='STOP|Stopped'): c.arm()
    assert len(b.writes)==1 and not c.armed


def test_stale_camera_no_writes(setup):
    c,b=setup;c.camera_fresh=lambda:False
    with pytest.raises(ValueError,match='camera'): c.arm()
    assert not b.writes


@pytest.mark.parametrize('delta', [0,True,float('nan'),5.01,-6.])
def test_invalid_step_never_moves(setup,delta):
    c,b=setup;c.arm();count=len(b.writes)
    with pytest.raises(ValueError): c.step({'joint':'elbow_flex','delta_deg':delta})
    assert len(b.writes)==count


def test_folded_start_only_moves_toward_task_interval():
    limits={'elbow_flex':[-97,97]};current={'elbow_flex':97.2}
    assert validate_step(current,current,'elbow_flex',-1.5,limits)==95.7
    with pytest.raises(ValueError): validate_step(current,current,'elbow_flex',1.,limits)


def test_only_selected_joint_moves_with_one_bounded_goal(setup):
    c,b=setup;c.arm();before=len(b.writes)
    c.step({'joint':'elbow_flex','delta_deg':-1.5})
    writes=b.writes[before:]
    assert writes==[('Goal_Position','elbow_flex',2048-17)]
    assert c.steps==1 and c.last_result['status']=='completed'


def test_unmoving_joint_times_out_then_holds_measured_pose(setup):
    c,b=setup;c.arm();b.follow=False
    with pytest.raises(ValueError,match='settle'):c.step({'joint':'elbow_flex','delta_deg':-1.5})
    c.halt('tracking test')
    assert c.latched and all(b.registers['Goal_Position'][n]==2048 for n in ARM)
    assert all(b.registers['Torque_Enable'][n]==1 for n in ARM)
    with pytest.raises(ValueError,match='stopped'): c.step({'joint':'elbow_flex','delta_deg':-.5})


def test_moving_joint_outside_envelope_stops(setup):
    c,b=setup;c.arm()
    def overshoot(key,n,v):
        if key=='Goal_Position': b.registers['Present_Position'][n]=v-13
    b.on_write=overshoot
    with pytest.raises(ValueError,match='envelope'):c.step({'joint':'elbow_flex','delta_deg':-1.5})


def test_other_joint_drift_still_stops_during_step(setup):
    c,b=setup;c.arm()
    def sag(key,n,v):
        if key=='Goal_Position': b.registers['Present_Position']['shoulder_lift']+=9
    b.on_write=sag
    with pytest.raises(ValueError,match='tracking'):c.step({'joint':'elbow_flex','delta_deg':-1.5})


def test_stop_after_goal_write_latches_hold(setup):
    c,b=setup;c.arm();before=len(b.writes)
    b.on_write=lambda *args:c.stop_path.touch()
    with pytest.raises(ValueError,match='STOP'): c.step({'joint':'elbow_flex','delta_deg':-1.5})
    assert len(b.writes)==before+1
    c.halt('local stop')
    assert c.latched and c.phase=='stopped_holding'


def test_stalled_joint_gets_bounded_lead_then_settles(setup):
    from so101_mcp.commissioning import STEP_MAX_LEAD_TICKS
    c,b=setup;c.arm();b.follow=False;before=len(b.writes)
    def sticky(key,n,v):
        # Moves only once the goal leads the measured pose by 10 ticks, then stalls 6 short of goal.
        p=b.registers['Present_Position']
        if key=='Goal_Position' and n=='elbow_flex' and p[n]-v>=10: p[n]=v+8
    b.on_write=sticky
    c.step({'joint':'elbow_flex','delta_deg':-1.5})
    goals=[v for key,n,v in b.writes[before:]]
    assert all(key=='Goal_Position' and n=='elbow_flex' for key,n,v in b.writes[before:])
    assert min(goals)>=2031-STEP_MAX_LEAD_TICKS
    assert all(abs(v-u)<=1 for u,v in zip(goals[1:],goals[2:]) if v<u)
    assert abs(b.registers['Present_Position']['elbow_flex']-2031)<=6
    assert abs(c.last_result['hold_goal_raw']-2031)<=6


def test_lead_is_capped_when_joint_never_moves(setup):
    from so101_mcp.commissioning import STEP_MAX_LEAD_TICKS
    c,b=setup;c.arm();b.follow=False
    with pytest.raises(ValueError,match='settle'):c.step({'joint':'elbow_flex','delta_deg':-1.5})
    assert b.registers['Goal_Position']['elbow_flex']==2031-STEP_MAX_LEAD_TICKS


def test_isolated_voltage_flag_is_retried_and_logged(setup):
    from so101_mcp.servo_diagnostics import ServoReportedError
    c,b=setup;flags=[1]
    def checked(register):
        if flags[0]:flags[0]-=1;raise ServoReportedError('Servo error flags in response: 1')
        return b.sync_read(register)
    c.read_feedback=checked;c.arm()
    assert c.voltage_flags_tolerated==1 and c.armed
    assert 'voltage_flag_tolerated' in (c.run_dir/'events.jsonl').read_text()


@pytest.mark.parametrize('message',['Servo error flags in response: 1','Servo error flags in response: 32'])
def test_repeated_or_other_servo_flags_still_stop(setup,message):
    from so101_mcp.servo_diagnostics import ServoReportedError
    c,b=setup;c.arm();before=len(b.writes)
    def checked(register):raise ServoReportedError(message)
    c.read_feedback=checked
    with pytest.raises(ServoReportedError):c.step({'joint':'elbow_flex','delta_deg':-1.5})
    assert len(b.writes)==before


def test_voltage_flag_session_limit(setup):
    from so101_mcp.commissioning import VOLTAGE_FLAG_SESSION_LIMIT
    from so101_mcp.servo_diagnostics import ServoReportedError
    c,b=setup;c.arm();c.voltage_flags_tolerated=VOLTAGE_FLAG_SESSION_LIMIT;flags=[1]
    def checked(register):
        if flags[0]:flags[0]-=1;raise ServoReportedError('Servo error flags in response: 1')
        return b.sync_read(register)
    c.read_feedback=checked
    with pytest.raises(ServoReportedError):c.tick()


def test_hardware_encoder_range_never_bypassed(setup):
    c,b=setup;b.registers['Present_Position']['wrist_flex']=4096
    with pytest.raises(ValueError,match='physical encoder'): c.preflight()
    assert not b.writes


def test_guard_register_change_after_arming_is_detected(setup):
    c,b=setup;c.arm();b.registers['Goal_Velocity']['elbow_flex']=100
    with pytest.raises(ValueError,match='guard changed'): c.health()


def test_session_budget_and_pan_motion_rejected(setup):
    c,b=setup;c.arm();before=len(b.writes)
    with pytest.raises(ValueError):c.step({'joint':'wrist_roll','delta_deg':1.})
    c.total_travel_deg=150
    with pytest.raises(ValueError,match='budget'):c.step({'joint':'elbow_flex','delta_deg':1.})
    assert len(b.writes)==before


def test_explicit_handover_keeps_holding_without_torque_toggle(setup):
    import json
    c,b=setup;c.arm();c.halt('finished controller')
    previous=json.loads((c.run_dir/'status.json').read_text())
    c.stop_path.touch()
    new=c.run_dir/'second';new.mkdir()
    other=JointCommissioner(b,c.mapping,c.limits,new,lambda:True,c.sleep,adopt_stopped=previous)
    before=len(b.writes);other.arm()
    assert not any(key=='Torque_Enable' for key,n,v in b.writes[before:])
    assert other.initial==c.initial
    assert all(b.registers['Goal_Velocity'][n]==50 for n in ARM)


def test_handover_rejects_changed_pose_and_recursive_adoption(setup):
    import json
    c,b=setup;c.arm();c.halt('finished')
    previous=json.loads((c.run_dir/'status.json').read_text())
    other=JointCommissioner(b,c.mapping,c.limits,c.run_dir,lambda:True,c.sleep,adopt_stopped=previous)
    b.registers['Present_Position']['elbow_flex']+=24
    before=len(b.writes)
    with pytest.raises(ValueError,match='no longer matches'):other.arm()
    assert len(b.writes)==before
    b.registers['Present_Position']['elbow_flex']-=24
    from so101_mcp.commissioning import MAX_HANDOVERS
    previous['adoption_count']=MAX_HANDOVERS
    with pytest.raises(ValueError,match='handover limit'):other.arm()


def test_latched_monitor_never_writes_and_detects_hardware_error(setup):
    c,b=setup;c.arm();c.halt('test');before=len(b.writes)
    b.registers['Status']['elbow_flex']=32
    c.monitor_stopped()
    assert len(b.writes)==before and 'hardware error' in c.last_result['monitor_error']


def test_latched_monitor_does_not_claim_hold_when_goals_differ(setup):
    c,b=setup;c.arm();c.halt('test');before=len(b.writes)
    b.registers['Goal_Position']['elbow_flex']+=24
    c.monitor_stopped()
    assert c.phase=='stopped_hold_unverified' and 'goals do not match' in c.last_result['monitor_error']
    assert len(b.writes)==before


def test_voltage_fault_keeps_trigger_sample_after_monitor_recovers(setup):
    c,b=setup;c.arm();b.registers['Present_Voltage']['gripper']=41;c.health()
    with pytest.raises(ValueError,match='raw=41') as error:c.health()
    c.halt(str(error.value));b.registers['Present_Voltage']['gripper']=48
    c.monitor_stopped()
    assert c.last_result['health_at_stop']['voltage_raw']['gripper']==41
    assert c.health_snapshot['voltage_raw']['gripper']==48


def test_chained_handover_keeps_first_envelope_and_budget(setup):
    import json
    c,b=setup;c.arm();c.step({'joint':'elbow_flex','delta_deg':-1.5});c.halt('first')
    first=json.loads((c.run_dir/'status.json').read_text())
    runs=[c]
    for i in range(2):
        d=c.run_dir/f'r{i}';d.mkdir()
        prev=json.loads((runs[-1].run_dir/'status.json').read_text())
        nxt=JointCommissioner(b,c.mapping,c.limits,d,lambda:True,c.sleep,adopt_stopped=prev)
        nxt.arm();nxt.halt('chain');runs.append(nxt)
    last=json.loads((runs[-1].run_dir/'status.json').read_text())
    assert last['adoption_count']==2 and last['completed_steps']==1
    assert last['session_initial_joints_deg']==first['session_initial_joints_deg']
    assert last['total_travel_deg']==1.5


def test_voltage_guard_margin_allows_42_but_not_41(setup):
    c,b=setup;c.arm();b.registers['Present_Voltage']['wrist_roll']=42;c.health();c.health()
    b.registers['Present_Voltage']['wrist_roll']=41;c.health()
    with pytest.raises(ValueError,match='raw=41'):c.health()


def test_hold_compensation_is_bounded_and_rate_limited(setup):
    from so101_mcp.commissioning import HOLD_COMP_MAX_TICKS
    c,b=setup;c.arm();b.follow=False;before=len(b.writes)
    b.registers['Present_Position']['elbow_flex']=2048+7  # sagged, never recovers
    for _ in range(400):
        c.tick();c.sleep(.05)
    writes=b.writes[before:]
    assert writes and all(k=='Goal_Position' and n=='elbow_flex' for k,n,v in writes)
    assert b.registers['Goal_Position']['elbow_flex']==2048-HOLD_COMP_MAX_TICKS
    assert len(writes)==HOLD_COMP_MAX_TICKS


def test_hold_compensation_does_nothing_inside_deadband(setup):
    c,b=setup;c.arm();before=len(b.writes)
    b.registers['Present_Position']['elbow_flex']=2048+2
    for _ in range(50):
        c.tick();c.sleep(.05)
    assert len(b.writes)==before


def test_isolated_single_sample_voltage_dip_needs_confirmation(setup):
    c,b=setup;c.arm();b.registers['Present_Voltage']['wrist_roll']=41
    c.health()
    assert 'voltage_dip_pending' in (c.run_dir/'events.jsonl').read_text()
    with pytest.raises(ValueError,match='raw=41'):c.health()


def test_dip_on_two_servos_stops_at_once_single_at_limit_needs_repeat(setup):
    c,b=setup;c.arm()
    b.registers['Present_Voltage']['wrist_roll']=41;b.registers['Present_Voltage']['gripper']=41
    with pytest.raises(ValueError,match='raw=41'):c.health()
    b.registers['Present_Voltage']['gripper']=48;b.registers['Present_Voltage']['wrist_roll']=40
    c._voltage_dips=set();c.health()
    with pytest.raises(ValueError,match='raw=40'):c.health()


def test_handover_keeps_existing_hold_goal_lead(setup):
    import json
    c,b=setup;c.arm();c.halt('first')
    previous=json.loads((c.run_dir/'status.json').read_text())
    b.registers['Goal_Position']['elbow_flex']=2040  # energized lead above a sagging pose
    d=c.run_dir/'next';d.mkdir()
    other=JointCommissioner(b,c.mapping,c.limits,d,lambda:True,c.sleep,adopt_stopped=previous)
    before=len(b.writes);other.arm()
    assert ('Goal_Position','elbow_flex',2040) in b.writes[before:]
    assert other.expected['elbow_flex']==2048


def test_exit_while_holding_keeps_hold_goal_without_writes(setup):
    c,b=setup;c.arm();c.hold_goal['elbow_flex']=2040;b.registers['Goal_Position']['elbow_flex']=2040
    before=len(b.writes);c.halt('Local EXIT')
    assert len(b.writes)==before and c.phase=='stopped_holding'


def test_gripper_seeds_before_torque_then_moves(setup):
    c,b=setup;c.arm();before=len(b.writes)
    c.step({'gripper_open':.5})
    w=b.writes[before:]
    assert w[0]==('Goal_Velocity','gripper',400) and w[1]==('Goal_Position','gripper',2048)
    assert w[2]==('Torque_Enable','gripper',1)
    assert w[3]==('Goal_Position','gripper',round(4095*.5))
    assert c.gripper_enabled and c.last_result['gripper_result']=='reached'
    c.health()


def test_gripper_stall_is_reported_not_raised(setup):
    c,b=setup;c.arm()
    def stick(key,n,v):
        if key=='Goal_Position' and n=='gripper' and v<2000: b.registers['Present_Position']['gripper']=1500
    b.follow=True;b.on_write=stick
    c.step({'gripper_open':.5});b.follow=False
    c.step({'gripper_open':0})
    assert c.last_result['gripper_result']=='stalled'


@pytest.mark.parametrize('value',[-0.1,1.1,True,'x'])
def test_gripper_rejects_bad_values(setup,value):
    c,b=setup;c.arm();before=len(b.writes)
    with pytest.raises(ValueError):c.step({'gripper_open':value})
    assert len(b.writes)==before
