"""Actual MuJoCo + MCP + Responses wire format, with an explicitly mocked model.

This is a transport/control regression, NEVER an AI perception or task trial.
No OpenAI request, API key, object labels, evaluator or real robot is used.
"""
import argparse
import asyncio
import base64
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import httpx

from so101_mcp.api_agent import APIAgent, ResponsesClient, Settings, connect
from so101_mcp.config import ROOT


class ProtocolFixture:
    def __init__(self):
        self.requests = 0
        self.image_hashes = set()

    def __call__(self, request):
        assert str(request.url) == 'https://api.openai.com/v1/responses'
        body = json.loads(request.content)
        assert body['store'] is False and body['parallel_tool_calls'] is False
        assert body['model'] == 'gpt-6-astra'
        meta, program = None, None
        for item in body['input']:
            for part in item.get('content', []) if isinstance(item.get('content'), list) else []:
                if part['type'] == 'input_image':
                    raw = base64.b64decode(part['image_url'].split(',', 1)[1])
                    assert raw.startswith(b'\x89PNG\r\n\x1a\n')
                    self.image_hashes.add(hashlib.sha256(raw).hexdigest())
                if part['type'] == 'input_text':
                    data = json.loads(part['text'].split('\n', 1)[1])
                    if 'captured_utc' in data:
                        meta = data
                    if 'program_id' in data:
                        program = data
        assert meta and self.image_hashes
        assert meta['backend'] == 'sim' and meta['images'][0]['role'] == 'rgb'
        assert not any(s in json.dumps(meta) for s in ('inside_box', 'object_positions', 'seed'))
        if self.requests == 0:
            name = 'prepare_step'
            args = {'observation_id': meta['observation_id'], 'joint_delta_deg': {'shoulder_pan': 1.0},
                    'gripper_delta': 0., 'visual_reason': 'Protocol fixture: robot-only one-degree test, no object selection',
                    'expected_visual_change': 'robot pan changes; no task success claim'}
        elif self.requests == 1:
            assert program and len(self.image_hashes) >= 2  # RGB and generated preview.
            name = 'execute_skill'
            args = {'name': 'prepared_step', 'args': {'program_id': program['program_id']},
                    'observation_id': meta['observation_id']}
        else:
            assert self.requests == 2
            assert meta['status']['last_action']['command_status'] == 'completed'
            assert meta['observation_id'] != program['observation_id']
            assert len(self.image_hashes) >= 3  # New physical render after motion.
            name = 'finish_task'
            args = {'observation_id': meta['observation_id'], 'verdict': 'uncertain',
                    'reason': 'Scripted protocol fixture; no model perception or task evaluation performed'}
        self.requests += 1
        return httpx.Response(200, json={'id': f'mock_response_{self.requests}',
            'model': 'scripted-protocol-fixture', 'status': 'completed', 'usage': {'total_tokens': 0},
            'output': [{'type': 'function_call', 'call_id': f'fixture_{self.requests}',
                        'name': name, 'arguments': json.dumps(args)}]})


async def verify(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    args = SimpleNamespace(url=None, output=output, sim_config=ROOT/'config/physical_ai/sim_staged.json')
    fixture = ProtocolFixture()
    settings = Settings(max_responses=4, max_motion_steps=1, max_elapsed_s=40)
    async with httpx.AsyncClient(transport=httpx.MockTransport(fixture)) as http:
        model = ResponsesClient('mock-local-fixture', settings, http)
        async with connect(args) as session:
            result = await APIAgent(session, model, settings, output,
                'Software protocol verification only: shoulder_pan +1 degree, then finish uncertain.').run()
    assert result['outcome'] == 'finished', result
    assert result['ai_visual_verdict'] == 'uncertain'
    assert result['stop_requested'] and result['motion_requests'] == 1
    assert list((output/'sim/programs').glob('*.py'))
    return {'status': 'passed', 'scope': 'mock Responses endpoint + actual MCP stdio + actual MuJoCo physics',
        'live_api_requests': 0, 'live_astra_trials': 0, 'real_hardware_access': False,
        'model_decision_fixture': True, 'task_successes': None,
        'mock_api_responses': fixture.requests, 'completed_sim_motion_steps': 1,
        'distinct_pngs_delivered_to_mock_api': len(fixture.image_hashes),
        'generated_python_saved': True, 'execution_via_common_mcp_program_id': True,
        'post_motion_rgb_before_next_decision': True, 'stop_requested': True,
        'api_key_available_at_implementation': False,
        'live_model_validation': 'not_performed: OPENAI_API_KEY unavailable',
        'historical_demo_agent': 'conversation-hosted Codex; exact model not recorded; not API/Astra trial',
        'references_checked': '2026-11-01',
        'api_reference': 'https://developers.openai.com/api/docs/guides/function-calling',
        'model_reference': 'https://developers.openai.com/api/docs/models/gpt-6-astra'}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True, help='New local directory')
    p.add_argument('--summary', type=Path, default=ROOT/'artifacts/api_client_verification.json')
    args = p.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        p.error('Use an empty/new output directory')
    result = asyncio.run(verify(args.output))
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
