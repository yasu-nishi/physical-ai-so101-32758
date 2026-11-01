"""Independent camera-position audit, after every host session is closed.

Initial scene truth is reconstructed here for evaluation only. It is never
provided to the host's measurement or control tools.
"""
import csv
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from so101_mcp.evaluation import evaluate_sim
from so101_mcp.sim import SimBackend


def main():
    cases = json.loads((ROOT/'config/physical_ai/camera_views/cases.json').read_text())
    expected = {'camera_front_01','camera_side_01','camera_low_01','camera_top_01'}
    if {c['case_id'] for c in cases} != expected:
        raise SystemExit('This audit scores four archived camera cases; add explicit rules for additional trials')
    if any(not (ROOT/'artifacts/runs'/c['case_id']/'case_result.json').is_file() for c in cases):
        raise SystemExit('Close all host sessions before reading or reconstructing evaluation truth')
    rows = []
    for case in cases:
        run = ROOT/'artifacts/runs'/case['case_id']
        manifest = json.loads((run/'run_manifest.json').read_text())
        host = json.loads((run/'case_result.json').read_text())
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        obs = [e for e in events if e['event']=='observation']
        actions = [e for e in events if e['event']=='action_finished']
        programs = [e for e in events if e['event']=='cartesian_program_prepared']
        measures = [e for e in events if e['event']=='rgb_height_measured']
        assert all(not o['capabilities']['depth_sampling'] and o['physics_during_deliberation'] for o in obs)
        assert all(len(p['trajectory'])==1 and not p['automatic_next_step'] for p in programs)
        assert not any(e['event'] in ('depth_sampled','program_prepared') for e in events)
        for action in actions:
            assert any(o['utc']>action['utc'] and o['status']['last_action']['action_id']==action['action_id'] for o in obs)
        with tempfile.TemporaryDirectory() as tmp:
            backend = SimBackend(manifest['config'], tmp, seed=manifest['seed'], record=False)
            try:
                backend.connect()
                initial = evaluate_sim(backend)['blocks']
            finally:
                backend.close()
        height_checks=[]
        for color, measurement in zip(('red','blue'), measures[:2]):
            truth = initial[color]['full_size_m'][2]
            lo, hi = measurement['height_interval_m']
            height_checks.append({'color':color,'estimated_mm':measurement['estimated_height_m']*1000,
                'actual_mm':truth*1000,'error_mm':(measurement['estimated_height_m']-truth)*1000,
                'interval_mm':[lo*1000,hi*1000],'actual_in_pixel_interval':lo<=truth<=hi})
        physical={}
        if actions:
            blocks=json.loads((run/'evaluation_private.json').read_text())['blocks']
            red,blue=blocks['red'],blocks['blue']
            physical={'red_inside_box':red['inside_box'],'red_at_rest':red['speed_norm']<.02,
                'red_released':not red['gripper_contact_at_end'],
                'blue_displacement_mm':float(np.linalg.norm(np.array(blue['position_m'])-initial['blue']['position_m'])*1000),
                'blue_gripper_contact_steps':blue['contact_steps'],
                'host_observed_uncommanded_drops':2,'controlled_transfer_success':False}
        rows.append({'case_id':case['case_id'],'camera_override':manifest['config']['camera_override'],
            'host_outcome':host['host_outcome'],'host_reason':host['host_reason'],
            'commands':len(actions),'commands_completed':sum(a['command_status']=='completed' for a in actions),
            'trajectory_s':round(sum(a['trajectory_s'] for a in actions),3),
            'controller_wall_s':round(sum(a['wall_s'] for a in actions),3),
            'last_observation_sim_s':round(obs[-1]['simulation_time_s'],3),
            'measurements':len(measures),'initial_height_checks':height_checks,'physical_checks':physical,
            'before_image':str((run/f"observe_{obs[0]['observation_id']}.png").relative_to(ROOT)),
            'after_image':str((run/f"observe_{obs[-1]['observation_id']}.png").relative_to(ROOT)),
            'video':str((run/'simulation.mp4').relative_to(ROOT)),
            'note':'Initial truth reconstructed only after all four host decisions finished'})
    checks=[c for row in rows for c in row['initial_height_checks']]
    data={'date':'2026-11-01','scope':'camera-position development checks, not a held-out benchmark',
        'runs':4,'distinct_object_arrangements':1,'camera_poses':4,'initial_rgb_height_measurements':len(checks),
        'height_mean_absolute_error_mm':float(np.mean([abs(c['error_mm']) for c in checks])),
        'height_maximum_absolute_error_mm':max(abs(c['error_mm']) for c in checks),
        'height_in_pixel_interval':sum(c['actual_in_pixel_interval'] for c in checks),
        'height_comparison_note':'Same two objects viewed from three cameras; not six independent objects. Ideal simulated calibration.',
        'commands':sum(r['commands'] for r in rows),'commands_completed':sum(r['commands_completed'] for r in rows),
        'trajectory_s':round(sum(r['trajectory_s'] for r in rows),3),
        'transport_trials':1,'controlled_transport_successes':0,'final_target_in_box_after_drops':1,
        'unobservable_top_view_no_motion':1,'host_observed_uncommanded_drops':2,
        'real_connection':'unverified; OS did not enumerate configured USB serial','real_camera_calibration':'not performed',
        'real_motion':'unverified','physics_contact_only':True,'depth_used':False,'llm_calls_in_server':0,
        'all_host_decisions_finished_before_offline_scoring':True,'cases':rows,
        'limitations':['Viewpoint changes are ideal simulated cameras; real lighting, blur, distortion and calibration error need physical tests.',
            'Height requires visible matching vertical-edge endpoints and a known support plane; top view was not measurable.',
            'Two losses of grasp occurred; final placement alone is not counted as controlled transport success.',
            'Moving wrist-camera calibration is not implemented. No real motion or real height accuracy has been verified.']}
    (ROOT/'artifacts/camera_view_experiments.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    with (ROOT/'artifacts/camera_view_experiments.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['case_id','host_outcome','commands','commands_completed','trajectory_s','measurements'])
        writer.writeheader();writer.writerows({k:r[k] for k in writer.fieldnames} for r in rows)
    print(json.dumps({k:v for k,v in data.items() if k not in ('cases','limitations')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
