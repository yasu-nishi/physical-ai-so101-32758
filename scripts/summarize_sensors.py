"""Aggregate recorded MCP sensor trials only after the host's visual verdict."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    trials = []
    for run in sorted((ROOT/'artifacts/runs').glob('sensors_*')):
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        start = next(e for e in events if e['event'] == 'action_started')
        end = next(e for e in events if e['event'] == 'action_finished' and e['action_id'] == start['action_id'])
        verdict = next(e for e in events if e['event'] == 'ai_visual_verdict' and e['action_id'] == start['action_id'])
        plan = next(e for e in events if e['event'] == 'program_prepared' and e['program_id'] == start['args']['program_id'])
        target = 'blue' if '青' in start['instruction'] else 'red' if '赤' in start['instruction'] else None
        if target is None:
            raise ValueError('Unrecognized instruction; label the experiment locally')
        other = 'red' if target == 'blue' else 'blue'
        # Independent evaluation is read only after a recorded host verdict exists.
        evaluation = json.loads((run/'evaluation_private.json').read_text())
        def rel(path): return str(path.relative_to(ROOT))
        before, after = start['observation_id'], verdict['observation_id']
        trials.append({'trial': run.name, 'instruction': start['instruction'], 'target': target,
            'visual_selection_reason': plan['visual_reason'],
            'command_status': end['command_status'], 'ai_visual_verdict': verdict['ai_visual_verdict'],
            'independent_success': evaluation['blocks'][target]['inside_box'] and not evaluation['blocks'][other]['inside_box'],
            'trajectory_s': round(end['trajectory_s'], 3), 'wall_s': round(end['wall_s'], 3),
            'failure': end['error'], 'visual_result_reason': verdict['reason'],
            'depth_samples': [e for e in events if e['event'] == 'depth_sampled'],
            'before_image': rel(run/f'observe_{before}.png'),
            'before_depth_image': rel(run/f'observe_{before}_depth.png'),
            'after_image': rel(run/f'observe_{after}.png'),
            'after_depth_image': rel(run/f'observe_{after}_depth.png'),
            'video': rel(run/'simulation.mp4'),
            'physical_evidence': evaluation['blocks'][target]})
    result = {'date': '2026-11-01', 'attempts': len(trials),
              'adoption_status': 'SIM_ONLY_DIAGNOSTIC_REJECTED_FOR_COMMON_PROFILE: not evidence of real hardware depth or sim-to-real success',
              'successes': sum(t['independent_success'] for t in trials),
              'protocol': 'Host views RGB + virtual depth, selects RGB pixels, samples depth, checks the known-height condition, executes generated runner, views post-action RGB + depth',
              'height_measurement': 'simulated visible-surface depth + calibrated table plane',
              'motion_height_source': 'existing known 30 mm block skill; no depth-adaptive trajectory',
              'real_depth': 'unavailable; hardware not connected or calibrated',
              'trials': trials}
    (ROOT/'artifacts/sensor_verification.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'trials'}, indent=2))


if __name__ == '__main__':
    main()
