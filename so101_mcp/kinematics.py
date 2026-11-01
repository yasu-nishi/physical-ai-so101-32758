"""Robot-only SO-101 model shared by sim and calibrated real adapters.

This model contains no task objects. Input is measured common joint angles.
Real use requires independently checked joint zeros, tool and work transform.
"""
import math

import mujoco
import numpy as np
from scipy.optimize import least_squares

from .config import ROOT
from .joints import ARM, Pose


class RobotKinematics:
    def __init__(self, config):
        self.config=config
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/so101_tabletop/robotstudio_so101/so101.xml'))
        self.data=mujoco.MjData(self.model)
        self.qadr=[self.model.joint(n).qposadr[0] for n in ARM]
        self.site=self.model.site('gripperframe').id
        settings=config.get('cartesian_steps',{})
        self.offset=np.array([settings.get('model_offset_deg',{}).get(n,0.) for n in ARM])
        self.transform=np.array(settings.get('model_to_work',np.eye(4).tolist()),dtype=float)
        if (self.transform.shape!=(4,4) or not np.isfinite(self.transform).all()
            or not np.allclose(self.transform[3],[0,0,0,1])
            or not np.allclose(self.transform[:3,:3].T@self.transform[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(self.transform[:3,:3]),1.)):
            raise ValueError('Invalid robot model-to-work rigid transform')

    def _forward(self,degrees):
        self.data.qpos[self.qadr]=np.deg2rad(np.asarray(degrees)+self.offset)
        mujoco.mj_kinematics(self.model,self.data)
        r=self.transform[:3,:3]
        return (r@self.data.site_xpos[self.site]+self.transform[:3,3],
                r@self.data.site_xmat[self.site].reshape(3,3))

    def forward(self,pose):
        xyz,rotation=self._forward([pose.joints_deg[n] for n in ARM])
        return {'position_m':xyz.tolist(),'local_axes_in_work':rotation.tolist(),
                'source':'robot-only forward kinematics from measured joints; calibrated model required',
                'frame':'robot_base_table_z0','tool_frame':'gripperframe'}

    def solve_down(self,xyz,current):
        point=np.asarray(xyz,dtype=float)
        def residual(q):
            pos,rotation=self._forward(q)
            return np.r_[pos-point,.1*(rotation[:,0]-[0,0,-1]),.03*math.radians(q[4])]
        lo,hi=np.array([self.config['limits_deg'][n] for n in ARM]).T
        seed=np.clip([current.joints_deg[n] for n in ARM],lo+1e-6,hi-1e-6)
        result=least_squares(residual,seed,bounds=(lo,hi),ftol=1e-10,xtol=1e-10,gtol=1e-10)
        if np.linalg.norm(residual(result.x))>.001:
            raise ValueError('One-step target is unreachable with downward tool orientation')
        return Pose(dict(zip(ARM,result.x.tolist(),strict=True)),current.gripper_open)

    def solve_preserve_tilt(self,xyz,current):
        """Translate from an angled pose without forcing the tool vertical.

        Preserve the tool's inclination to the work table and the measured
        wrist-roll joint. Pan may turn with XY translation; this is not a
        fixed full orientation or obstacle-avoidance guarantee.
        """
        point=np.asarray(xyz,dtype=float)
        seed=np.array([current.joints_deg[n] for n in ARM])
        def tilt(rotation):
            axis=rotation[:,0]
            return math.atan2(axis[2],math.hypot(axis[0],axis[1]))
        desired_tilt=tilt(self._forward(seed)[1])
        def unpack(active):
            return np.r_[active,seed[4]]
        def residual(active):
            pos,rotation=self._forward(unpack(active))
            return np.r_[pos-point,.1*(tilt(rotation)-desired_tilt)]
        lo,hi=np.array([self.config['limits_deg'][n] for n in ARM[:4]]).T
        result=least_squares(residual,np.clip(seed[:4],lo+1e-6,hi-1e-6),
                             bounds=(lo,hi),ftol=1e-11,xtol=1e-11,gtol=1e-11)
        q=unpack(result.x)
        pos,rotation=self._forward(q)
        if np.linalg.norm(pos-point)>.0001 or abs(tilt(rotation)-desired_tilt)>math.radians(.1):
            raise ValueError('One-step target is unreachable while preserving tool tilt')
        return Pose(dict(zip(ARM,q.tolist(),strict=True)),current.gripper_open)

    def solve_position(self,xyz,current,locked_joints):
        """Position-only IK with measured roll and optional arm joint held.

        This does not disable a motor or relax tracking checks. A locked joint
        keeps its measured nominal angle; its actual angle is still monitored.
        Tool inclination may change and is checked separately by the planner.
        """
        point=np.asarray(xyz,dtype=float)
        seed=np.array([current.joints_deg[n] for n in ARM])
        active=[i for i,n in enumerate(ARM[:4]) if n not in locked_joints]
        def unpack(values):
            q=seed.copy();q[active]=values
            return q
        def residual(values):
            pos,_=self._forward(unpack(values))
            # With no extra lock there is one redundant DOF. Prefer a nearby
            # posture without sacrificing the separately checked XYZ accuracy.
            return np.r_[pos-point,1e-5*np.deg2rad(values-seed[active])]
        lo,hi=np.array([self.config['limits_deg'][ARM[i]] for i in active]).T
        result=least_squares(residual,np.clip(seed[active],lo+1e-6,hi-1e-6),
                             bounds=(lo,hi),ftol=1e-12,xtol=1e-12,gtol=1e-12)
        q=unpack(result.x)
        if np.linalg.norm(self._forward(q)[0]-point)>.0001:
            raise ValueError('Position-only target is unreachable with the requested joint held')
        return Pose(dict(zip(ARM,q.tolist(),strict=True)),current.gripper_open)


def cartesian_step_plan(kinematics,current,target_xyz,opening,settings,tool_orientation='down',locked_joints=None):
    if not settings.get('enabled'):
        raise ValueError('Cartesian steps are not enabled locally')
    if tool_orientation not in ('down','preserve_tilt','position_only'):
        raise ValueError('tool_orientation must be down, preserve_tilt or position_only')
    locked=[] if locked_joints is None else locked_joints
    if not isinstance(locked,list) or len(locked)>1 or any(n not in ARM[:4] for n in locked):
        raise ValueError('locked_joints may hold at most one named arm joint; wrist_roll is already held')
    if locked and (tool_orientation!='position_only' or target_xyz is None):
        raise ValueError('locked_joints requires a position_only XYZ step')
    if (target_xyz is None)==(opening is None):
        raise ValueError('Specify exactly one: target_xyz_m OR gripper_open, then observe again')
    if opening is not None:
        if isinstance(opening,bool) or not isinstance(opening,(float,int)) or not math.isfinite(opening):
            raise ValueError('Finite gripper opening fraction required')
        if abs(opening-current.gripper_open)>settings['max_gripper_change']:
            raise ValueError('Gripper change exceeds local limit')
        return [('gripper_step',Pose(current.joints_deg,opening),settings['minimum_duration_s'])]
    xyz=np.asarray(target_xyz,dtype=float)
    if xyz.shape!=(3,) or not np.isfinite(xyz).all():
        raise ValueError('Target must be finite work-frame [x,y,z] metres')
    bounds=np.array(settings['workspace_xyz_m'])
    if np.any(xyz<bounds[:,0]) or np.any(xyz>bounds[:,1]):
        raise ValueError('Cartesian target outside the configured workspace')
    before=np.array(kinematics.forward(current)['position_m'])
    distance=float(np.linalg.norm(xyz-before))
    if distance>settings['max_translation_m']+1e-8:
        raise ValueError('One-step translation limit exceeded; observe between shorter moves')
    if distance<.0001:
        raise ValueError('Translation is too small; use observe or a gripper-only step')
    if tool_orientation=='down':
        target=kinematics.solve_down(xyz,current)
    elif tool_orientation=='preserve_tilt':
        target=kinematics.solve_preserve_tilt(xyz,current)
    else:
        target=kinematics.solve_position(xyz,current,locked)
        maximum=settings.get('max_tool_axis_change_deg',5.)
        if isinstance(maximum,bool) or not isinstance(maximum,(int,float)) or not math.isfinite(maximum) or not 0<maximum<=5:
            raise ValueError('Position-only tool-axis limit must be in (0, 5] degrees')
        initial_axis=np.array(kinematics.forward(current)['local_axes_in_work'])[:,0]
        final_axis=np.array(kinematics.forward(target)['local_axes_in_work'])[:,0]
        change=math.degrees(math.acos(float(np.clip(initial_axis@final_axis,-1.,1.))))
        if change>maximum:
            raise ValueError('Position-only tool-axis change exceeds local limit')
    if max(abs(target.joints_deg[n]-current.joints_deg[n]) for n in ARM)>settings['max_joint_change_deg']:
        raise ValueError('One-step joint change exceeded; use a closer target or observation pose')
    duration=max(settings['minimum_duration_s'],1.5*distance/settings['speed_m_s'])
    return [('cartesian_step',target,duration)]
