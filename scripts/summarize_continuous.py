"""Offline-only scoring AFTER all visual verdicts; never an MCP tool.

Reconstruct the initial scene solely to measure localization error, and verify
that its rendered observation is byte-identical to the recorded input image.
Ground-truth coordinates are never provided to the image selection or planner.
"""
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from so101_mcp.sim import SimBackend


def main():
    trials = []
    for run in sorted((ROOT/'artifacts/runs').glob('continuous_*')):
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        start = next(e for e in events if e['event'] == 'action_started')
        end = next(e for e in events if e['event'] == 'action_finished' and e['action_id'] == start['action_id'])
        verdict = next(e for e in events if e['event'] == 'ai_visual_verdict' and e['action_id'] == start['action_id'])
        prepared = next(e for e in events if e['event'] == 'program_prepared' and e['program_id'] == start['args']['program_id'])
        before = run/f'observe_{start["observation_id"]}.png'
        after = run/f'observe_{verdict["observation_id"]}.png'
        manifest = json.loads((run/'run_manifest.json').read_text())
        target = 'red' if '赤' in start['instruction'] else 'blue' if '青' in start['instruction'] else None
        if target is None:
            raise ValueError('Unknown instruction')
        other = 'blue' if target == 'red' else 'red'
        with tempfile.TemporaryDirectory() as directory:
            replay = SimBackend(manifest['config'], Path(directory), seed=manifest['seed'], record=False)
            try:
                replay.connect()
                if replay.capture() != before.read_bytes():
                    raise ValueError('Initial scene replay image differs: cannot report localization error')
                ground_truth = replay.data.body(target+'_cube').xpos.copy()
            finally:
                replay.close()
        estimated = prepared['localization']['pick_xy_m']
        error_mm = float(np.linalg.norm(np.asarray(estimated)-ground_truth[:2])*1000)
        evaluation = json.loads((run/'evaluation_private.json').read_text())
        independent = evaluation['blocks'][target]['inside_box'] and not evaluation['blocks'][other]['inside_box']
        def relative(path): return str(path.relative_to(ROOT))
        pid = prepared['program_id']
        trials.append({'trial': run.name, 'instruction': start['instruction'], 'target': target,
                       'pixel_xy': prepared['localization']['pixel_xy'], 'estimated_xy_m': estimated,
                       'z_source': 'assumed_known_30_mm_height_not_measured',
                       'independent_initial_xy_m': ground_truth[:2].tolist(),
                       'initial_image_replay_matches': True, 'independent_xy_error_mm': error_mm,
                       'command_status': end['command_status'], 'ai_visual_verdict': verdict['ai_visual_verdict'],
                       'independent_success': independent, 'trajectory_s': round(end['trajectory_s'],3),
                       'wall_s': round(end['wall_s'],3), 'failure': end['error'], 'visual_reason': verdict['reason'],
                       'before_image': relative(before), 'after_image': relative(after),
                       'before_sha256': hashlib.sha256(before.read_bytes()).hexdigest(),
                       'after_sha256': hashlib.sha256(after.read_bytes()).hexdigest(),
                       'preview_image': relative(run/f'programs/{pid}_preview.png'),
                       'generated_python': relative(run/f'programs/{pid}.py'),
                       'generated_plan': relative(run/f'programs/{pid}.json'),
                       'video': relative(run/'simulation.mp4'),
                       'physical_evidence': evaluation['blocks'][target]})
    result = {'date': '2026-10-31', 'protocol': 'Codex visual pixel choice -> calibrated XY -> generated Python runner over MCP',
              'attempts': len(trials), 'successes': sum(t['independent_success'] for t in trials),
              'xy_error_mm_max': max(t['independent_xy_error_mm'] for t in trials),
              'height_measurement': 'NOT USED IN THESE 2026-10-31 TRIALS: Z=30 mm is assumed; later sensor trials are reported separately',
              'generation': 'deterministic IK and bounded MCP Python runner export; no arbitrary Python execution in server',
              'scope': 'two seeded continuous arrangements x two colors; upright 30 mm cubes, calibrated ideal camera, no noise',
              'real_continuous_picking': 'unsupported: camera and kinematic calibration absent',
              'trials': trials}
    (ROOT/'artifacts/continuous_experiments.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    columns = ['trial','target','pixel_xy','estimated_xy_m','independent_xy_error_mm','z_source',
               'command_status','ai_visual_verdict','independent_success','trajectory_s','wall_s','failure']
    with (ROOT/'artifacts/continuous_experiments.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(trials)
    print(json.dumps({k:v for k,v in result.items() if k != 'trials'}, indent=2))
    for t in trials:
        print(t['trial'], t['target'], t['estimated_xy_m'], 'error_mm', round(t['independent_xy_error_mm'],3),
              'success',t['independent_success'], 'wall_s',t['wall_s'])


if __name__ == '__main__': main()
