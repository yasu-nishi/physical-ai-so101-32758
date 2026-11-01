"""Verify the logged input contract and aggregate RGB-only host trials offline."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    trials=[]
    for run in sorted((ROOT/'artifacts/runs').glob('rgb_portable_*')):
        events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        observations=[e for e in events if e['event']=='observation']
        for observed in observations:
            assert not observed['capabilities']['depth_sampling']
            assert not observed['sensors']['end_effector']['available']
            assert [im['role'] for im in observed['images']]==['rgb']
        assert not any(e['event']=='depth_sampled' for e in events)
        start=next(e for e in events if e['event']=='action_started')
        end=next(e for e in events if e['event']=='action_finished' and e['action_id']==start['action_id'])
        verdict=next(e for e in events if e['event']=='ai_visual_verdict' and e['action_id']==start['action_id'])
        plan=next(e for e in events if e['event']=='program_prepared' and e['program_id']==start['args']['program_id'])
        target='red' if '赤' in start['instruction'] else 'blue' if '青' in start['instruction'] else None
        if target is None: raise ValueError('Unknown target instruction')
        other='blue' if target=='red' else 'red'
        # Only after the host's visual verdict exists, read independent truth.
        evaluation=json.loads((run/'evaluation_private.json').read_text())
        def rel(path): return str(path.relative_to(ROOT))
        trials.append({'trial':run.name,'instruction':start['instruction'],'target':target,
            'pixel_xy':plan['localization']['pixel_xy'], 'estimated_xy_m':plan['localization']['pick_xy_m'],
            'height_source':'known 30 mm task geometry; not measured', 'depth_returned':False,
            'sim_only_end_effector_returned':False, 'command_status':end['command_status'],
            'ai_visual_verdict':verdict['ai_visual_verdict'], 'visual_reason':verdict['reason'],
            'independent_success':evaluation['blocks'][target]['inside_box'] and not evaluation['blocks'][other]['inside_box'],
            'trajectory_s':round(end['trajectory_s'],3),'wall_s':round(end['wall_s'],3),'failure':end['error'],
            'before_image':rel(run/f'observe_{start["observation_id"]}.png'),
            'after_image':rel(run/f'observe_{verdict["observation_id"]}.png'),
            'preview_image':rel(run/f'programs/{plan["program_id"]}_preview.png'),
            'video':rel(run/'simulation.mp4'),'physical_evidence':evaluation['blocks'][target]})
    summary={'date':'2026-11-01','scope':'RGB + joints + known camera calibration / known object height; no depth or sim-only tool pose',
             'attempts':len(trials),'successes':sum(t['independent_success'] for t in trials),
             'real_camera_calibration':'tooling prepared; hardware calibration not performed',
             'real_motion':'unverified; disconnected','trials':trials}
    (ROOT/'artifacts/rgb_portable_experiments.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='trials'},indent=2))


if __name__=='__main__': main()
