"""Post-hoc evaluator. Run only AFTER the AI has recorded its visual verdicts."""
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    trials = []
    for run in sorted((ROOT/'artifacts/runs').glob('vision_*')):
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        start = next(x for x in events if x['event'] == 'action_started')
        end = next(x for x in events if x['event'] == 'action_finished' and x['action_id'] == start['action_id'])
        verdict = next(x for x in events if x['event'] == 'ai_visual_verdict' and x['action_id'] == start['action_id'])
        evaluation = json.loads((run/'evaluation_private.json').read_text())
        target = 'red' if '赤' in start['instruction'] else 'blue' if '青' in start['instruction'] else None
        if target is None:
            raise ValueError('Unrecognized instruction in offline evaluation')
        other = 'blue' if target == 'red' else 'red'
        before = run/f'observe_{start["observation_id"]}.png'
        after = run/f'observe_{verdict["observation_id"]}.png'
        independent = evaluation['blocks'][target]['inside_box'] and not evaluation['blocks'][other]['inside_box']
        trials.append({'trial': run.name, 'instruction': start['instruction'], 'target': target,
                       'selected_slot': start['args']['slot'], 'command_status': end['command_status'],
                       'ai_visual_verdict': verdict['ai_visual_verdict'], 'independent_success': independent,
                       'trajectory_s': round(end['trajectory_s'], 3), 'wall_s': round(end['wall_s'], 3),
                       'failure': end['error'], 'visual_reason': verdict['reason'],
                       'before_image': str(before.relative_to(ROOT)), 'after_image': str(after.relative_to(ROOT)),
                       'before_sha256': hashlib.sha256(before.read_bytes()).hexdigest(),
                       'after_sha256': hashlib.sha256(after.read_bytes()).hexdigest(),
                       'video': str((run/'simulation.mp4').relative_to(ROOT)),
                       'physical_evidence': evaluation['blocks'][target]})
    result = {'date': '2026-10-31', 'protocol': 'active Codex vision via transparent MCP client bridge',
              'host_model_id': 'not recorded; Astra-specific test unverified',
              'attempts': len(trials), 'successes': sum(t['independent_success'] for t in trials),
              'real_connection': 'unverified: configured USB serial port absent', 'real_motion': 'unverified',
              'scope': 'two 30 mm blocks, two fixed slots, one box; deterministic initial poses; no noise',
              'other_checks': 'development contact probe and automated tests are not counted as AI trials',
              'trials': trials}
    (ROOT/'artifacts/experiments.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    columns = ['trial', 'instruction', 'target', 'selected_slot', 'command_status', 'ai_visual_verdict',
               'independent_success', 'trajectory_s', 'wall_s', 'failure']
    with (ROOT/'artifacts/experiments.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader(); writer.writerows(trials)
    print(json.dumps({k: v for k, v in result.items() if k != 'trials'}, ensure_ascii=False, indent=2))
    for t in trials:
        print(t['trial'], t['target'], t['selected_slot'], t['independent_success'], t['wall_s'], t['physical_evidence'])


if __name__ == '__main__': main()
