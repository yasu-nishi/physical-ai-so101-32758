"""Curate completed API simulation trials for offline review. Never calls an LLM.

Only run after all selected trials have ended. Independent physics state is read
here, never by the API agent, and is explicitly scoped to the last action end.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from so101_mcp.config import ROOT
from scripts.export_submission import check_private_text


def summarize(runs):
    rows = []
    for index, run in enumerate(runs, 1):
        run = Path(run).resolve()
        result = json.loads((run/'result.json').read_text())
        assert result['backend'] == 'sim' and result['stop_requested']
        assert result['requested_model'] == 'gpt-6-astra'
        assert result['returned_models'] == ['gpt-6-astra']
        events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
        sim_events = [json.loads(line) for line in (run/'sim/events.jsonl').read_text().splitlines()]
        responses = [e for e in events if e['event'] == 'api_response']
        finished = [e for e in sim_events if e['event'] == 'action_finished']
        observed = [e for e in events if e.get('tool') == 'observe' and e.get('images')]
        # This file is available to this offline scorer only, not a model tool.
        evaluation = json.loads((run/'sim/evaluation_private.json').read_text())
        assert evaluation['action_id'] == finished[-1]['action_id']
        target_inside = evaluation['blocks']['red']['inside_box']
        other_outside = not evaluation['blocks']['blue']['inside_box']
        success = (result['outcome'] == 'finished' and result['ai_visual_verdict'] == 'success'
                   and target_inside and other_outside)
        destination = ROOT/'artifacts/api_evidence'/f'trial_{index:02d}'
        marker = destination/'.curated_api_evidence'
        if destination.exists() and not marker.exists():
            raise ValueError('Refusing to replace an unmarked directory')
        if destination.exists():
            shutil.rmtree(destination)
        destination.mkdir(parents=True)
        marker.write_text('Completed sim API trial; curated without credentials or server stderr\n')
        selected = [run/'events.jsonl', run/'result.json', run/'agent_manifest.json', *run.glob('*.png'),
                    run/'sim/events.jsonl', run/'sim/evaluation_private.json',
                    run/'sim/evaluation_history_private.jsonl', run/'sim/run_manifest.json',
                    run/'sim/simulation.mp4', *[p for p in (run/'sim/programs').glob('*') if p.is_file()]]
        hashes = []
        for source in selected:
            relative = source.relative_to(run)
            check_private_text(source, str(relative))
            dest = destination/relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, dest)
            hashes.append({'path':str(relative),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
        (destination/'evidence_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
        prefix = str(destination.relative_to(ROOT))
        rows.append({
            'trial':index, 'evidence_dir':prefix, 'model':result['returned_models'][0],
            'outcome':result['outcome'], 'error':result.get('error'),
            'ai_visual_verdict':result['ai_visual_verdict'], 'ai_reason':result.get('reason'),
            'api_responses':len(responses), 'motion_requests':result['motion_requests'],
            'commands_completed':sum(e['command_status']=='completed' for e in finished),
            'trajectory_s':sum(e['trajectory_s'] for e in finished),
            'elapsed_wall_s':result['elapsed_s'],
            'api_wait_s':sum(e['elapsed_api_s'] for e in responses),
            'total_tokens':result['total_tokens'],
            'input_tokens':sum(e['usage'].get('input_tokens',0) for e in responses),
            'output_tokens':sum(e['usage'].get('output_tokens',0) for e in responses),
            'cached_input_tokens':sum(e['usage'].get('input_tokens_details',{}).get('cached_tokens',0) for e in responses),
            'cache_write_tokens':sum(e['usage'].get('input_tokens_details',{}).get('cache_write_tokens',0) for e in responses),
            'rejected_tool_calls':sum(e.get('is_error',False) for e in events if e['event']=='mcp'),
            'settings':result['settings'],
            'independent_state_scope':'at last action completion, before final visual deliberation',
            'target_inside_box':target_inside, 'other_block_outside_box':other_outside,
            'target_contact_steps':evaluation['blocks']['red']['contact_steps'],
            'target_final_gripper_contact':evaluation['blocks']['red']['gripper_contact_at_end'],
            'task_success':success, 'stop_requested':result['stop_requested'],
            'before_image':prefix+'/'+observed[0]['images'][0]['path'],
            'after_image':prefix+'/'+observed[-1]['images'][0]['path'],
            'video':prefix+'/sim/simulation.mp4',
            'source_hash_manifest':prefix+'/evidence_hashes.json',
        })
    data={'scope':'Live OpenAI Responses API -> existing MCP -> actual MuJoCo contact physics',
          'model':'gpt-6-astra','backend':'sim','real_hardware_access':False,
          'instruction':'赤いブロックを箱に入れて。画像から対象と位置を選び、必要なら高さを測り、一段ずつ実行して結果を確認してください。',
          'trial_count':len(rows),'success_count':sum(r['task_success'] for r in rows),
          'general_success_rate_claim':False,
          'conditions':'Same initial sim configuration and seed; independent model decisions from RGB each run. Development retry, not randomized benchmark.',
          'historical_seven_runs_included':False,
          'ground_truth_used_only_after_all_selected_trials_closed':True,
          'llm_calls_inside_mcp_server':0,'trials':rows}
    output=ROOT/'artifacts/api_live_verification.json'
    output.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    return data


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('runs',type=Path,nargs='+')
    args=p.parse_args()
    data=summarize(args.runs)
    print(json.dumps({k:v for k,v in data.items() if k!='trials'},ensure_ascii=False,indent=2))
