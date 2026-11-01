"""Offline scoring, only after the host has finished and recorded every case."""
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    cases=json.loads((ROOT/'config/physical_ai/suite/cases.json').read_text())
    # Refuse partial scoring that could leak object truth into later decisions.
    for case in cases:
        assert (ROOT/'artifacts/runs'/case['case_id']/'case_result.json').is_file(), 'Finish all host decisions first'
    rows=[]
    for case in cases:
        run=ROOT/'artifacts/runs'/case['case_id']
        host=json.loads((run/'case_result.json').read_text())
        events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        obs=[e for e in events if e['event']=='observation']
        observations={e['observation_id']:e for e in obs}
        actions=[e for e in events if e['event']=='action_finished']
        picks=[e for e in events if e['event']=='program_prepared']
        increments=[e for e in events if e['event']=='incremental_program_prepared']
        verdicts=[e for e in events if e['event']=='ai_visual_verdict']
        clients=[json.loads(p.read_text()) for p in sorted((run/'client').glob('*.json'))]
        rejections=[str(t) for c in clients if c['isError'] for t in c['text']]
        assert all(not o['capabilities']['depth_sampling'] and not o['sensors']['end_effector']['available'] for o in obs)
        assert not any(e['event']=='depth_sampled' for e in events)
        for program in increments:
            assert len(program['trajectory'])==1 and program['automatic_next_step'] is False
            assert observations[program['observation_id']]['utc'] < program['utc']
        truth_path=run/'evaluation_private.json'
        truth=json.loads(truth_path.read_text()) if truth_path.exists() else None
        physical=None
        if case['category']=='pick_place':
            assert picks and verdicts and actions[-1]['action_id']==verdicts[-1]['action_id']
            assert verdicts[-1]['utc']>observations[verdicts[-1]['observation_id']]['utc']
            target=case['target'];other='blue' if target=='red' else 'red'
            physical=bool(truth['blocks'][target]['inside_box'] and not truth['blocks'][other]['inside_box']
                          and truth['blocks'][target]['contact_steps']>0)
        guard=None
        if case['category']=='absent_target':
            guard=not actions and host['host_outcome']=='expected_abstention'
        elif case['category']=='outside_workspace':
            guard=not actions and any('workspace' in t for t in rejections)
        elif case['category']=='stale_observation':
            guard=len(actions)==1 and any('moved since this image' in t for t in rejections)
        elif case['category']=='stop_during_motion':
            guard=len(actions)==1 and actions[-1]['command_status']=='stopped' and any('latched' in t for t in rejections)
        choice=picks[-1]['observation_id'] if picks else obs[0]['observation_id']
        row={'case_id':case['case_id'],'category':case['category'],'instruction':case['instruction'],
             'target':case['target'],'condition':case['condition'],'host_outcome':host['host_outcome'],
             'host_reason':host['host_reason'],'physical_pick_success':physical,'guard_pass':guard,
             'commands':len(actions),'commands_completed':sum(a['command_status']=='completed' for a in actions),
             'incremental_steps':len(increments),'trajectory_s':round(sum(a['trajectory_s'] for a in actions),3),
             'wall_s':round(sum(a['wall_s'] for a in actions),3),
             'pixel_xy':picks[-1]['localization']['pixel_xy'] if picks else None,
             'before_image':str((run/f'observe_{obs[0]["observation_id"]}.png').relative_to(ROOT)),
             'selection_image':str((run/f'observe_{choice}.png').relative_to(ROOT)),
             'after_image':str((run/f'observe_{obs[-1]["observation_id"]}.png').relative_to(ROOT)),
             'video':str((run/'simulation.mp4').relative_to(ROOT)) if (run/'simulation.mp4').exists() else None,
             'rejections':rejections,'feedback':[e['visual_reason'] for e in increments],
             'ai_verdicts':[{'verdict':v['ai_visual_verdict'],'reason':v['reason']} for v in verdicts],
             'physical_evidence':truth['blocks'][case['target']] if physical is not None else None}
        if (run/'stop_probe.json').exists():row['stop_probe']=json.loads((run/'stop_probe.json').read_text())
        rows.append(row)
    pick=[r for r in rows if r['category']=='pick_place']
    steps=[r for r in rows if r['category']=='incremental']
    guard=[r for r in rows if r['guard_pass'] is not None]
    data={'date':'2026-11-01','new_cases':len(rows),
          'pick_place':{'attempts':len(pick),'physical_successes':sum(r['physical_pick_success'] for r in pick),
                        'visual_successes':sum(r['host_outcome']=='success' for r in pick),
                        'cases_with_visibility_adjustment':sum(r['incremental_steps']>0 for r in pick),
                        'height_assumption_mm':30},
          'incremental':{'cases':len(steps),'feedback_verified':sum(r['host_outcome']=='feedback_verified' for r in steps),
                         'stopped_occlusion':sum(r['host_outcome']=='stopped_occlusion' for r in steps),
                         'grasp_or_transport_tested':False},
          'guards':{'cases':len(guard),'passed':sum(r['guard_pass'] for r in guard)},
          'real_connection':'unverified','real_motion':'unverified','depth_used':False,
          'limitations':['Known upright 30 mm cubes and calibrated ideal cameras for full pick-place',
                         'Incremental actions tested visibility/direction; complete incremental grasp not tested',
                         'During host deliberation the simulator is paused; held-object latency not tested',
                         'No real sensor noise, varied physical object heights, or real hardware validation'],
          'cases':rows}
    (ROOT/'artifacts/scenario_experiments.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    with (ROOT/'artifacts/scenario_experiments.csv').open('w',newline='') as out:
        keys=['case_id','category','target','host_outcome','physical_pick_success','guard_pass','commands',
              'commands_completed','incremental_steps','trajectory_s','wall_s','host_reason']
        writer=csv.DictWriter(out,fieldnames=keys);writer.writeheader()
        writer.writerows({k:r[k] for k in keys} for r in rows)
    print(json.dumps({k:v for k,v in data.items() if k not in ('cases','limitations')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
