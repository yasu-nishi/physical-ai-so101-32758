"""Geometry independence, robot-only FK, bounded steps and continuous settling."""
import json
import time

import numpy as np
import pytest

from so101_mcp.config import ROOT,load_config
from so101_mcp.controller import Controller
from so101_mcp.geometry import pixel_to_plane
from so101_mcp.joints import Pose
from so101_mcp.kinematics import RobotKinematics,cartesian_step_plan
from so101_mcp.metrology import measure_vertical
from so101_mcp.sim import SimBackend


def projection(c,p):
    xyz=np.array(c['camera_to_work_rotation']).T@(np.array(p)-c['camera_position_m'])
    u,v=c['principal_point_px'];f=c['focal_px']
    return [u+f*xyz[0]/-xyz[2],v-f*xyz[1]/-xyz[2]]


@pytest.mark.parametrize('height',[.018,.027,.043,.061,.095])
def test_unknown_height_from_rgb_edges_not_object_size(height,tmp_path):
    cfg=load_config('sim',ROOT/'config/physical_ai/sim_staged.json')
    b=SimBackend(cfg,tmp_path,record=False);b.connect()
    try:
        cal=b.camera_calibration()
        base=[.25,-.08,0.];top=[.25,-.08,height];center=[.235,-.065,height]
        result=measure_vertical(cal,projection(cal,base),projection(cal,top),projection(cal,center))
        assert result['estimated_height_m']==pytest.approx(height,abs=1e-9)
        assert result['estimated_top_center_m']==pytest.approx(center,abs=1e-9)
        lo,hi=result['height_interval_m'];assert lo<height<hi
        assert not result['depth_sensor_used']
        wrong=pixel_to_plane(cal,*projection(cal,center),.03)
        if height!=.03:assert np.linalg.norm(np.array(wrong)-center)>.001
    finally:b.close()


def test_bad_correspondence_and_top_view_rejected(tmp_path):
    cfg=load_config('sim');b=SimBackend(cfg,tmp_path,record=False);b.connect()
    try:
        cal=b.camera_calibration();cal['camera_to_work_rotation']=np.eye(3).tolist();cal['camera_position_m']=[0,0,.6]
        u,v=cal['principal_point_px']
        with pytest.raises(ValueError,match='nearly vertical'):
            measure_vertical(cal,[u,v+15],[u,v],[u,v])
    finally:b.close()


def test_occluded_top_centre_can_measure_height_without_inventing_xy(tmp_path):
    cfg=load_config('sim',ROOT/'config/physical_ai/sim_staged.json')
    c=Controller(SimBackend(cfg,tmp_path,record=False),cfg,tmp_path)
    try:
        obs,_=c.observe();cal=obs['camera_calibration']
        result,png=c.measure_height(obs['observation_id'],projection(cal,[.25,-.08,0]),
                                   projection(cal,[.25,-.08,.03]),None,0.,'visible edge, top centre hidden')
        assert result['estimated_height_m']==pytest.approx(.03)
        assert result['estimated_top_center_m'] is None and result['center_pixel'] is None
        assert png.startswith(b'\x89PNG')
    finally:c.close()


def test_real_cartesian_motion_gate_requires_all_local_verifications(tmp_path):
    from so101_mcp.real import RealBackend
    cfg=load_config('real',ROOT/'config/physical_ai/real_staged.json')
    cfg['taught_skills_file']=str(tmp_path/'nonexistent.json')
    real=RealBackend(cfg,tmp_path,enable_motion=True)
    with pytest.raises(ValueError,match='mapping'):real._validate_motion_setup()
    cfg.update(mapping_verified=True,workspace_verified=True,camera_role='fixed_workspace')
    with pytest.raises(ValueError,match='locally verify'):real._validate_motion_setup()
    cfg['cartesian_steps']['real_kinematics_verified']=True
    with pytest.raises(ValueError,match='Calibrated real RGB'):real._validate_motion_setup()
    real.rgb_calibration={'synthetic_test_only':True}
    real._validate_motion_setup()
    assert real.writes==0
    cfg['camera_role']='wrist'
    with pytest.raises(ValueError,match='fixed workspace camera'):real._validate_motion_setup()


def test_robot_only_fk_matches_joint_driven_model_and_step_gates(tmp_path):
    cfg=load_config('sim',ROOT/'config/physical_ai/sim_staged.json')
    b=SimBackend(cfg,tmp_path,record=False);c=Controller(b,cfg,tmp_path)
    try:
        obs=c.observe()[0]
        pos=obs['sensors']['end_effector']['position_m']
        assert np.array(pos)==pytest.approx(b.data.site_xpos[b.site],abs=1e-9)
        assert obs['physics_during_deliberation'] and not obs['sensors']['depth']['available']
        before=b.elapsed;time.sleep(.12);assert b.elapsed>before+.05
        target=(np.array(pos)+[.01,0,0]).tolist()
        p=c.prepare_cartesian_step(obs['observation_id'],target,None,'test fixture','synthetic evidence','inspect next RGB')[0]
        assert len(p['trajectory'])==1 and not p['automatic_next_step']
        with pytest.raises(ValueError,match='translation limit'):
            c.prepare_cartesian_step(obs['observation_id'],[.28,.15,.08],None,'test fixture','synthetic evidence','new RGB')
        with pytest.raises(ValueError,match='exactly one'):
            c.prepare_cartesian_step(obs['observation_id'],target,.1,'test fixture','synthetic evidence','new RGB')
        c.execute_skill('prepared_cartesian_step',{'program_id':p['program_id']},p['observation_id'],p['instruction'],'one')
        deadline=time.monotonic()+4
        while c.get_status()['state']=='running' and time.monotonic()<deadline:time.sleep(.01)
        assert c.get_status()['state']=='completed'
        with pytest.raises(ValueError,match='moved'):
            c.prepare_cartesian_step(obs['observation_id'],target,None,'test fixture','synthetic evidence','new RGB')
        new=c.observe()[0]
        assert new['sensors']['end_effector']['position_m']==pytest.approx(target,abs=.002)
        c.stop()
        with pytest.raises(RuntimeError,match='latched'):
            c.prepare_cartesian_step(new['observation_id'],target,None,'test fixture','synthetic evidence','new RGB')
    finally:c.close()


def test_real_cartesian_requires_separate_local_kinematics_verification(tmp_path):
    from test_incremental import MockCameraReal
    cfg=load_config('real');cfg['cartesian_steps']={'enabled':True,'real_kinematics_verified':False}
    c=Controller(MockCameraReal(cfg,tmp_path),cfg,tmp_path)
    try:
        obs=c.observe()[0]
        assert not obs['capabilities']['cartesian_step']
        with pytest.raises(ValueError,match='not enabled/verified'):
            c.prepare_cartesian_step(obs['observation_id'],[.2,0,.1],None,'test','visible','new image')
    finally:c.close()


def test_arm_step_keeps_closing_command_under_contact(tmp_path):
    # A software fixture with a known cube, never counted as host AI perception.
    cfg=load_config('sim',ROOT/'config/physical_ai/sim_staged.json')
    cfg.pop('object_randomization');cfg['layout']='fixed';cfg['idle_physics']=False
    kin=RobotKinematics(cfg)
    cfg['home']=kin.solve_down([.24,-.08,.019],Pose.from_dict(cfg['home'])).as_dict()
    c=Controller(SimBackend(cfg,tmp_path,record=False),cfg,tmp_path)
    def run(xyz,grip,rid):
        obs=c.observe()[0]
        p=c.prepare_cartesian_step(obs['observation_id'],xyz,grip,'contact fixture','software test','new RGB')[0]
        c.execute_skill('prepared_cartesian_step',{'program_id':p['program_id']},p['observation_id'],p['instruction'],rid)
        deadline=time.monotonic()+3
        while c.get_status()['state']=='running' and time.monotonic()<deadline:time.sleep(.01)
        assert c.get_status()['state']=='completed'
        return p
    try:
        run(None,.04,'close')
        assert c.observe()[0]['gripper_open']>.10
        p=run([.24,-.08,.040],None,'lift')
        assert p['maintained_gripper_command']==pytest.approx(.04)
        assert p['trajectory'][0]['start']['gripper_open']==pytest.approx(.04)
        assert p['trajectory'][0]['target']['gripper_open']==pytest.approx(.04)
        # Independent physics evidence, appropriate only inside this test.
        evaluation=json.loads((tmp_path/'evaluation_private.json').read_text())
        assert any(v['position_m'][2]>.030 and v['gripper_contact_at_end'] for v in evaluation['blocks'].values())
    finally:c.close()


def test_measured_tolerance_cannot_emit_out_of_range_command():
    from types import SimpleNamespace
    cfg=load_config('sim',ROOT/'config/physical_ai/sim_staged.json')
    current=Pose.from_dict(cfg['home'])
    current.joints_deg['wrist_flex']=94.9002
    c=Controller.__new__(Controller);c.config=cfg;c.backend=SimpleNamespace(state=lambda:current)
    target=Pose({**current.joints_deg,'wrist_flex':94.},current.gripper_open)
    waypoints,_=c._validate_plan([('fixture',target,.8)])
    assert waypoints[0][1].joints_deg['wrist_flex']==94.9
    target.joints_deg['wrist_flex']=94.9002
    with pytest.raises(ValueError,match='outside'):c._validate_plan([('fixture',target,.8)])


@pytest.mark.parametrize('delta',[[0,0,-.01],[0,0,.01],[.005,0,0],[0,.005,0]])
def test_angled_cartesian_step_preserves_tilt_roll_and_existing_real_limits(delta):
    # A robot-only numeric fixture. No hardware connection, camera or object pose.
    cfg=load_config('real',ROOT/'config/physical_ai/real_staged.json')
    from so101_mcp.joints import ARM,validate_limits
    pose=Pose(dict(zip(ARM,[-38.,-81.,76.,34.,5.],strict=True)),.34)
    k=RobotKinematics(cfg)
    before=np.array(k.forward(pose)['position_m'])
    settings={**cfg['cartesian_steps'],'workspace_xyz_m':[[-1,1],[-1,1],[0,1]]}
    target=before+delta
    plan=cartesian_step_plan(k,pose,target,None,settings,'preserve_tilt')
    result=plan[0][1]
    validate_limits(result,cfg['limits_deg'])
    assert k.forward(result)['position_m']==pytest.approx(target,abs=.0001)
    assert k.forward(result)['local_axes_in_work'][2][0]==pytest.approx(
        k.forward(pose)['local_axes_in_work'][2][0],abs=.001)
    assert result.joints_deg['wrist_roll']==pose.joints_deg['wrist_roll']
    assert result.gripper_open==pose.gripper_open
    assert max(abs(result.joints_deg[n]-pose.joints_deg[n]) for n in ARM)<=5
    if delta==[0,0,-.01]:
        assert max(abs(result.joints_deg[n]-pose.joints_deg[n]) for n in ARM)>2
        with pytest.raises(ValueError,match='joint change|unreachable'):
            cartesian_step_plan(k,pose,target,None,settings)  # down cannot jump from this angle
    with pytest.raises(ValueError,match='joint change'):
        cartesian_step_plan(k,pose,target,None,{**settings,'max_joint_change_deg':.1},'preserve_tilt')
    with pytest.raises(ValueError,match='translation limit'):
        cartesian_step_plan(k,pose,before+[0,0,.03],None,settings,'preserve_tilt')
    with pytest.raises(ValueError,match='tool_orientation'):
        cartesian_step_plan(k,pose,target,None,settings,'unbounded')


@pytest.mark.parametrize('backend',['sim','real'])
def test_position_only_retreat_holds_elbow_and_roll_within_existing_limits(backend):
    from so101_mcp.joints import ARM,validate_limits
    cfg=load_config(backend,ROOT/f'config/physical_ai/{backend}_staged.json')
    pose=Pose(dict(zip(ARM,[-39.16,-82.37,78.81,34.20,4.79],strict=True)),.34)
    k=RobotKinematics(cfg)
    settings={**cfg['cartesian_steps'],'workspace_xyz_m':[[-1,1],[-1,1],[0,1]]}
    before=k.forward(pose)
    target=np.array(before['position_m'])+[0,0,.005]
    plan=cartesian_step_plan(k,pose,target,None,settings,'position_only',['elbow_flex'])
    after=plan[0][1]
    validate_limits(after,cfg['limits_deg'])
    assert k.forward(after)['position_m']==pytest.approx(target,abs=.0001)
    assert after.joints_deg['elbow_flex']==pose.joints_deg['elbow_flex']
    assert after.joints_deg['wrist_roll']==pose.joints_deg['wrist_roll']
    assert after.gripper_open==pose.gripper_open
    assert max(abs(after.joints_deg[n]-pose.joints_deg[n]) for n in ARM)<=5
    old_axis=np.array(before['local_axes_in_work'])[:,0]
    new_axis=np.array(k.forward(after)['local_axes_in_work'])[:,0]
    assert 1.<np.degrees(np.arccos(np.clip(old_axis@new_axis,-1,1)))<2.
    with pytest.raises(ValueError,match='tool-axis change'):
        cartesian_step_plan(k,pose,target,None,{**settings,'max_tool_axis_change_deg':.1},'position_only',['elbow_flex'])
    for bad in (['unknown'],['elbow_flex','shoulder_lift'],['wrist_roll'],'elbow_flex'):
        with pytest.raises(ValueError,match='locked_joints'):
            cartesian_step_plan(k,pose,target,None,settings,'position_only',bad)
    with pytest.raises(ValueError,match='requires a position_only'):
        cartesian_step_plan(k,pose,target,None,settings,'preserve_tilt',['elbow_flex'])
    with pytest.raises(ValueError,match='requires a position_only'):
        cartesian_step_plan(k,pose,None,.3,settings,'position_only',['elbow_flex'])
    with pytest.raises(ValueError,match='joint change'):
        cartesian_step_plan(k,pose,target,None,{**settings,'max_joint_change_deg':.1},'position_only',['elbow_flex'])
    for invalid_limit in (0,6,float('nan'),True):
        with pytest.raises(ValueError,match='tool-axis limit'):
            cartesian_step_plan(k,pose,target,None,{**settings,'max_tool_axis_change_deg':invalid_limit},'position_only',['elbow_flex'])


def test_stalled_elbow_cannot_finish_cartesian_upward_command(tmp_path):
    from test_safety import FakeBackend,wait_for
    from so101_mcp.joints import ARM
    cfg=load_config('real',ROOT/'config/physical_ai/real_staged.json')
    cfg['cartesian_steps']['real_kinematics_verified']=True
    start=Pose(dict(zip(ARM,[-38.,-83.,81.,30.,5.],strict=True)),.34)
    kin=RobotKinematics(cfg)
    target=kin.solve_preserve_tilt(np.array(kin.forward(start)['position_m'])+[0,0,.01],start)
    class StalledElbow(FakeBackend):
        mode='motion_enabled'
        def command(self,planned):
            self.commands.append(planned)
            self.current=Pose({**planned.joints_deg,'elbow_flex':start.joints_deg['elbow_flex']},planned.gripper_open)
    backend=StalledElbow();backend.current=start
    c=Controller(backend,cfg,tmp_path)
    try:
        # Feed a validated one-phase job to the normal worker, no serial device.
        with c._lock:
            c._job={'action_id':'stall-fixture','name':'prepared_cartesian_step','args':{},
                'plan':[('cartesian_step',start,target,1.5)],'index':0,'t':0.,'elapsed':0.,
                'started':time.monotonic(),'monitor_cartesian':True}
        status=wait_for(c,lambda s:s['stop_latched'])
        assert status['last_action']['command_status']=='failed'
        assert 'Cartesian' in status['last_action']['error']
        assert status['last_action']['trajectory_s']<1.5
        assert backend.halts>=1 and status['last_action']['task_success']=='unverified'
        count=len(backend.commands);time.sleep(.1);assert len(backend.commands)==count
    finally:c.close()


def test_cartesian_tracking_accepts_following_and_rejects_stationary_arm():
    from so101_mcp.joints import ARM,interpolate
    cfg=load_config('real',ROOT/'config/physical_ai/real_staged.json')
    c=Controller.__new__(Controller);c.config=cfg;c._kinematics=RobotKinematics(cfg)
    start=Pose(dict(zip(ARM,[-38.,-83.,81.,30.,5.],strict=True)),.34)
    target=c._kinematics.solve_preserve_tilt(np.array(c._kinematics.forward(start)['position_m'])+[0,0,.01],start)
    for alpha in np.linspace(0,1,31):
        expected=interpolate(start,target,float(alpha))
        c._check_cartesian_tracking(start,target,expected,expected)
    with pytest.raises(RuntimeError,match='tracking error'):
        c._check_cartesian_tracking(start,target,target,start)
