"""Summarize the recorded host feedback sequence, without reading object truth."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    run = ROOT/'artifacts/runs/incremental_01'
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    observations = {e['observation_id']: e for e in events if e['event'] == 'observation'}
    programs = [e for e in events if e['event'] == 'incremental_program_prepared']
    steps = []
    for program in programs:
        pid = program['program_id']
        execution = json.loads((run/f'programs/{pid}_execution.json').read_text())
        result = execution['status']['last_action']
        post = execution['observation']
        verdict = next(e for e in events if e['event'] == 'ai_visual_verdict' and e['action_id'] == result['action_id'])
        if steps:
            assert program['observation_id'] == steps[-1]['after_observation_id']
            # The next decision was generated only after the prior image/verdict.
            assert program['utc'] > steps[-1]['verdict_utc']
        for observation in (observations[program['observation_id']], post):
            assert not observation['capabilities']['depth_sampling']
            assert not observation['sensors']['end_effector']['available']
        assert len(program['trajectory']) == 1 and program['automatic_next_step'] is False
        steps.append({'program_id': pid, 'joint_delta_deg': program['joint_delta_deg'],
                      'before_observation_id': program['observation_id'],
                      'after_observation_id': post['observation_id'],
                      'before_image': str((run/f'observe_{program["observation_id"]}.png').relative_to(ROOT)),
                      'after_image': str((run/f'programs/{pid}_after.png').relative_to(ROOT)),
                      'visual_reason': program['visual_reason'],
                      'expected_visual_change': program['expected_visual_change'],
                      'observed_feedback': verdict['reason'], 'verdict_utc': verdict['utc'],
                      'command_status': result['command_status'], 'trajectory_s': result['trajectory_s'],
                      'wall_s': round(result['wall_s'], 3), 'ai_task_verdict': verdict['ai_visual_verdict']})
    assert len(steps) == 3
    assert not any(e['event'] in ('depth_sampled', 'known_plane_localized', 'program_prepared') for e in events)
    result = {'date': '2026-11-01', 'scope': 'incremental visual direction feedback; not a grasp/transport trial',
              'trials': 1, 'steps': steps, 'commands_completed': sum(s['command_status'] == 'completed' for s in steps),
              'direction_prediction_errors': 1, 'correction': 'Observed first +2 degree turn moved left; chose -2 then -1 only after new RGB',
              'height_measured': False, 'task_success': 'not attempted: no grasp or transport',
              'included_in_pick_place_success_count': False,
              'real_motion': 'unverified; dry-run adapter uses mock RGB and measured-state fixture',
              'video': 'artifacts/runs/incremental_01/simulation.mp4'}
    (ROOT/'artifacts/incremental_experiments.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ('trials', 'commands_completed', 'direction_prediction_errors', 'task_success')}))


if __name__ == '__main__':
    main()
