"""Explicit units and mappings. Gripper opening is never an arm angle."""
from dataclasses import dataclass
import math

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


@dataclass(frozen=True)
class Pose:
    joints_deg: dict[str, float]
    gripper_open: float

    def __post_init__(self):
        if set(self.joints_deg) != set(ARM):
            raise ValueError('Exactly the five named arm joints are required')
        if not all(math.isfinite(v) for v in (*self.joints_deg.values(), self.gripper_open)):
            raise ValueError('Pose values must be finite')
        if not 0 <= self.gripper_open <= 1:
            raise ValueError('gripper_open must be in [0, 1]')

    def as_dict(self):
        return {'joints_deg': dict(self.joints_deg), 'gripper_open': self.gripper_open}

    @classmethod
    def from_dict(cls, value):
        return cls({k: float(v) for k, v in value['joints_deg'].items()}, float(value['gripper_open']))


class JointMap:
    def __init__(self, config):
        self.sign = config['sign']
        self.offset = config['offset_deg']
        if set(self.sign) != set(ARM) or set(self.offset) != set(ARM):
            raise ValueError('Mapping needs all five joint names')
        if any(v not in (-1, 1) for v in self.sign.values()):
            raise ValueError('Joint sign must be -1 or +1')
        if not all(math.isfinite(v) for v in self.offset.values()):
            raise ValueError('Offsets must be finite')
        self.closed = float(config.get('gripper_closed', 0))
        self.open = float(config.get('gripper_open', 100))
        if not all(math.isfinite(v) and 0 <= v <= 100 for v in (self.closed, self.open)) or self.open == self.closed:
            raise ValueError('Distinct gripper endpoints in LeRobot [0,100] required')

    def to_lerobot(self, pose):
        return {**{f'{n}.pos': self.sign[n]*pose.joints_deg[n]+self.offset[n] for n in ARM},
                'gripper.pos': self.closed+pose.gripper_open*(self.open-self.closed)}

    def from_lerobot(self, obs):
        opening = (float(obs['gripper.pos'])-self.closed)/(self.open-self.closed)
        # Only tiny numerical tolerance is allowed, never hide a mapping error.
        if not -.005 <= opening <= 1.005:
            raise ValueError('Measured gripper is outside configured endpoints')
        return Pose({n: (float(obs[f'{n}.pos'])-self.offset[n])/self.sign[n] for n in ARM},
                    min(1., max(0., opening)))


def validate_limits(pose, limits, tolerance_deg=0.):
    for name in ARM:
        lo, hi = limits[name]
        if not lo-tolerance_deg <= pose.joints_deg[name] <= hi+tolerance_deg:
            raise ValueError(f'{name} outside [{lo}, {hi}] degrees')


def interpolate(start, end, alpha):
    s = alpha*alpha*(3-2*alpha)
    return Pose({n: start.joints_deg[n]+s*(end.joints_deg[n]-start.joints_deg[n]) for n in ARM},
                start.gripper_open+s*(end.gripper_open-start.gripper_open))


def safe_duration(start, end, minimum, speed_deg_s, gripper_speed_s):
    # Cubic smoothstep has maximum derivative 1.5.
    return max(minimum, *(1.5*abs(end.joints_deg[n]-start.joints_deg[n])/speed_deg_s for n in ARM),
               1.5*abs(end.gripper_open-start.gripper_open)/gripper_speed_s)
