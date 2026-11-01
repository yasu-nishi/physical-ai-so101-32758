"""One bounded joint-space step. No camera depth, IK or object-state access."""
import math

from .joints import ARM, Pose


def step_target(current, deltas, gripper_delta, settings):
    if not settings or settings.get('enabled') is not True:
        raise ValueError('Incremental steps are not enabled in the local configuration')
    if not isinstance(deltas, dict) or not set(deltas) <= set(ARM):
        raise ValueError('Use only the five named arm joints, in degrees')
    values = [*deltas.values(), gripper_delta]
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in values):
        raise ValueError('Step deltas must be finite numeric values')
    if not any(values):
        raise ValueError('A zero step does not require a motion program')
    # Separate grasp/release decisions from arm motion and reobserve between them.
    if any(deltas.values()) and gripper_delta:
        raise ValueError('Move the arm or the gripper in one step, then observe again')
    if any(abs(v) > settings['max_joint_delta_deg'] for v in deltas.values()):
        raise ValueError('Per-joint incremental step limit exceeded')
    if sum(abs(v) for v in deltas.values()) > settings['max_total_joint_delta_deg']:
        raise ValueError('Total incremental joint step limit exceeded')
    if abs(gripper_delta) > settings['max_gripper_delta']:
        raise ValueError('Incremental gripper step limit exceeded')
    return Pose({n: current.joints_deg[n]+deltas.get(n, 0.) for n in ARM},
                current.gripper_open+gripper_delta)
