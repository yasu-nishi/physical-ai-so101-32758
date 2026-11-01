"""Fixed camera placement for viewpoint experiments; no object-state access."""
import numpy as np


def look_at_rotation(position_m, target_m, up=(0., 0., 1.)):
    """Camera +X right, +Y up, -Z forward, expressed in work coordinates."""
    position, target, up = (np.asarray(v, dtype=float) for v in (position_m, target_m, up))
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in (position, target, up)):
        raise ValueError('Camera position, target and up must be finite XYZ vectors')
    forward = target-position
    if np.linalg.norm(forward) < 1e-6 or np.linalg.norm(up) < 1e-6:
        raise ValueError('Camera target must differ from position; up must be nonzero')
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up/np.linalg.norm(up))
    if np.linalg.norm(right) < 1e-6:
        raise ValueError('Camera up must not be parallel to the viewing direction')
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(right, forward), -forward))


def apply_camera_override(scene, name, settings):
    """Apply before compiling physics; camera-only settings never move objects."""
    import mujoco
    if set(settings)-{'position_m', 'target_m', 'up', 'fovy_deg'}:
        raise ValueError('Unknown camera override setting')
    rotation = look_at_rotation(settings['position_m'], settings['target_m'], settings.get('up', [0, 0, 1]))
    fovy = float(settings['fovy_deg'])
    if not np.isfinite(fovy) or not 10 <= fovy <= 120:
        raise ValueError('Camera vertical field of view must be in [10,120] degrees')
    camera = scene.camera(name)
    if camera is None:
        raise ValueError('Unknown fixed simulation camera')
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    camera.pos = settings['position_m']
    # MJCF overview uses xyaxes; its active alternative would override quat.
    camera.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
    camera.quat = quat
    camera.fovy = fovy
