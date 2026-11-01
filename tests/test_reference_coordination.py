"""Asymmetric start-up lag reproduces upward commands going down.

The fixture is a bounded delayed actuator, not a validated real motor model.
"""
import time

import numpy as np
import pytest

from so101_mcp.config import ROOT, load_config
from so101_mcp.controller import Controller
from so101_mcp.joints import ARM, Pose
from so101_mcp.kinematics import RobotKinematics
from test_safety import FakeBackend, wait_for


@pytest.mark.parametrize('coordinated,delay_ticks,expected', [(False,13,'failed'),
    (True,13,'completed'), (True,1000,'failed')])
def test_vertical_retreat_waits_for_elbow_without_relaxing_stop_guards(tmp_path,coordinated,delay_ticks,expected):
    cfg=load_config('real',ROOT/'config/physical_ai/real_staged.json')
    cfg['cartesian_steps']['real_kinematics_verified']=True
    cfg['workspace_verified']=True
    cfg['cartesian_steps']['workspace_xyz_m']=[[-1,1],[-1,1],[0,1]]  # synthetic model frame
    if coordinated:
        cfg['cartesian_steps'].update(upward_progress_joint='elbow_flex',max_reference_lead_deg=.5)
    start=Pose(dict(zip(ARM,[-39.,-82.7,80.2,33.1,4.8],strict=True)),.34)
    kin=RobotKinematics(cfg)
    origin=np.array(kin.forward(start)['position_m'])
    target=kin.solve_preserve_tilt(origin+[0,0,.005],start)
    class DelayedElbow(FakeBackend):
        mode='motion_enabled'
        def command(self,reference):
            self.commands.append(reference)
            elbow=self.current.joints_deg['elbow_flex']
            if len(self.commands)>delay_ticks:
                elbow+=max(-.22,min(.22,reference.joints_deg['elbow_flex']-elbow))
            self.current=Pose({**reference.joints_deg,'elbow_flex':elbow},reference.gripper_open)
    backend=DelayedElbow();backend.current=start
    controller=Controller(backend,cfg,tmp_path)
    try:
        with controller._lock:
            controller._job={'action_id':'asymmetric-lag','name':'prepared_cartesian_step','args':{},
                'plan':[('cartesian_step',start,target,.8)],'index':0,'t':0.,'elapsed':0.,
                'started':time.monotonic(),'monitor_cartesian':True}
        status=wait_for(controller,lambda s:s['last_action'] is not None,timeout=4)
        assert status['last_action']['command_status']==expected
        if expected=='completed':
            assert np.linalg.norm(np.array(kin.forward(backend.current)['position_m'])-(origin+[0,0,.005]))<.003
            assert all(p.joints_deg['wrist_flex']==start.joints_deg['wrist_flex'] for p in backend.commands[:delay_ticks])
            assert all(p.joints_deg['wrist_roll']==start.joints_deg['wrist_roll'] for p in backend.commands)
        else:
            wait_for(controller,lambda s:s['stop_latched'])
            assert backend.halts>=1
            assert status['last_action']['trajectory_s']<3
        previous=start
        for reference in backend.commands:
            assert max(abs(reference.joints_deg[n]-previous.joints_deg[n]) for n in ARM)<=cfg['speed_deg_s']/cfg['control_hz']+1e-6
            previous=reference
        assert cfg['cartesian_steps']['tracking_tolerance_m']==.003
        assert cfg['cartesian_steps']['reverse_tolerance_m']==.001
    finally:controller.close()
