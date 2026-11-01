"""Independent ground truth scorer. Never registered as an MCP tool."""
import numpy as np


def evaluate_sim(backend):
    m, d = backend.model, backend.data
    box = d.body('box').xpos
    results = {}
    for color in ('red', 'blue'):
        body = m.body(color+'_cube')
        pos = d.xpos[body.id]
        # Full horizontal footprint inside the walls; centre in the accepted
        # vertical band. A tall block may protrude above this open box's rim.
        # Speed and absence of gripper contact distinguish held from placed.
        local = d.xmat[body.id].reshape(3, 3)
        geom=m.body_geomadr[body.id]
        half_sizes=m.geom_size[geom].copy()
        ext = np.abs(local) @ half_sizes
        j = m.body_jntadr[body.id]
        velocity = d.qvel[m.jnt_dofadr[j]:m.jnt_dofadr[j]+6]
        touching = False
        for c in d.contact:
            a, b = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (a == body.id and b in backend.fingers) or (b == body.id and a in backend.fingers):
                touching = True
        results[color] = {
            'position_m': pos.tolist(),
            'full_size_m':(2*half_sizes).tolist(),
            'bottom_z_m':float(pos[2]-ext[2]),
            'top_z_m':float(pos[2]+ext[2]),
            'speed_norm':float(np.linalg.norm(velocity)),
            'inside_box': bool(np.all(np.abs(pos[:2]-box[:2])+ext[:2] < .045)
                               and .002 < pos[2]-box[2] < .04
                               and np.linalg.norm(velocity) < .02 and not touching),
            'contact_steps': backend.audit['contact_steps'][color],
            'max_height_m': backend.audit['max_height_m'][color],
            'gripper_contact_at_end': touching,
        }
    return {'evaluator': 'independent_ground_truth_not_exposed_to_ai', 'blocks': results,
            'simulation_s': backend.elapsed}
