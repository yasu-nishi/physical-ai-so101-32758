import asyncio
import base64
import io
import json
from types import SimpleNamespace

import httpx
from mcp.types import CallToolResult, ImageContent, TextContent
from PIL import Image
import pytest

from so101_mcp.api_agent import APIAgent, AgentError, ResponsesClient, Settings
from so101_mcp.server import make_server


def result(data, image=False, error=False):
    parts = [TextContent(type='text', text=json.dumps(data))]
    if image:
        buf = io.BytesIO()
        Image.new('RGB', (8, 8), 'red').save(buf, format='PNG')
        parts.append(ImageContent(type='image', mimeType='image/png', data=base64.b64encode(buf.getvalue()).decode()))
    return CallToolResult(content=parts, isError=error)


class Session:
    def __init__(self, backend='sim', fail_motion=False, mode='simulation', stopped=False):
        self.calls = []
        self.oid = 0
        self.backend, self.fail_motion, self.mode, self.stopped = backend, fail_motion, mode, stopped
        self.last = None

    async def list_tools(self):
        return SimpleNamespace(tools=await make_server(None).list_tools())

    async def call_tool(self, name, args):
        self.calls.append((name, args))
        status = {'backend': self.backend, 'mode': self.mode, 'connection_verified': True,
                  'state': 'completed' if self.last else 'idle', 'stop_latched': self.stopped,
                  'last_action': self.last}
        if name == 'get_status':
            return result(status)
        if name == 'observe':
            self.oid += 1
            return result({'backend': self.backend, 'observation_id': str(self.oid),
                           'images': [{'role': 'rgb'}], 'status': status}, True)
        if name == 'prepare_step':
            return result({'program_id': 'a'*12, 'execution_skill': 'prepared_step',
                           'observation_id': str(self.oid)}, True)
        if name == 'execute_skill':
            self.last = {'action_id': 'action1', 'command_status': 'failed' if self.fail_motion else
                         ('dry_run' if self.mode == 'dry_run' else 'completed')}
            return result({'action_id': 'action1', 'accepted': True})
        if name == 'stop':
            return result({'stop_requested': True})
        if name == 'report_result':
            return result(args)
        raise AssertionError(name)


def call(name, args, call_id='c1'):
    return {'status': 'completed', 'model': 'mock-only', 'usage': {'total_tokens': 10},
            'output': [{'type': 'function_call', 'name': name, 'arguments': json.dumps(args), 'call_id': call_id}]}


def plan(oid='1'):
    return call('prepare_step', {'observation_id': oid, 'joint_delta_deg': {'shoulder_pan': 1.},
        'gripper_delta': 0., 'visual_reason': 'test', 'expected_visual_change': 'test'}, 'plan')


def execute(oid='1'):
    return call('execute_skill', {'name': 'prepared_step', 'args': {'program_id': 'a'*12},
                                 'observation_id': oid}, 'execute')


class Model:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.seen = []

    async def create(self, history, tools):
        self.seen.append(json.loads(json.dumps(history)))
        response = next(self.responses)
        if isinstance(response, Exception):
            raise response
        return response


def run(tmp_path, responses, session=None, settings=None):
    session = session or Session()
    model = Model(responses)
    agent = APIAgent(session, model, settings or Settings(), tmp_path, 'test original instruction')
    out = asyncio.run(agent.run())
    return out, session, model


def test_preview_execution_fresh_rgb_and_visual_verdict(tmp_path):
    out, session, model = run(tmp_path, [plan(), execute(), call('finish_task', {
        'observation_id': '2', 'verdict': 'success', 'reason': 'mock visible outcome'}, 'finish')])
    assert out['ai_visual_verdict'] == 'success' and out['stop_requested']
    names = [c[0] for c in session.calls]
    assert names == ['get_status', 'observe', 'prepare_step', 'execute_skill', 'get_status', 'observe', 'report_result', 'stop']
    sent = next(a for n, a in session.calls if n == 'execute_skill')
    assert sent['instruction'] == 'test original instruction' and sent['request_id'].endswith('-execute')
    assert sum('data:image/png;base64,' in json.dumps(x) for x in model.seen[-1]) == 3


@pytest.mark.parametrize('responses', [
    [execute()],  # Unprepared program, even when syntactically valid.
    [plan(), call('observe', {}, 'observe'), execute()],  # Image invalidates old plan.
    [call('sample_depth', {'observation_id': '1', 'pixel_x': 1, 'pixel_y': 1})],
    [call('execute_skill', {'name': 'pick_place', 'args': {}, 'observation_id': '1'})],
    [call('prepare_step', {'observation_id': '1', 'joint_delta_deg': {'shoulder_pan': float('nan')},
                          'gripper_delta': 0., 'visual_reason': 'bad', 'expected_visual_change': 'bad'})],
    [call('finish_task', {'observation_id': '1', 'verdict': 'success', 'reason': 'no motion'})],
])
def test_invalid_decision_never_moves(tmp_path, responses):
    out, session, _ = run(tmp_path, responses)
    assert out['outcome'] == 'aborted' and out['ai_visual_verdict'] == 'unverified'
    assert not any(n == 'execute_skill' for n, _ in session.calls)
    assert out['stop_requested']


@pytest.mark.parametrize('kind', ['parallel', 'incomplete', 'duplicate'])
def test_rejects_ambiguous_or_repeated_decisions(tmp_path, kind):
    response = plan()
    responses = [response]
    if kind == 'parallel':
        response['output'].append(execute()['output'][0])
    elif kind == 'incomplete':
        response['status'] = 'incomplete'
    else:
        responses = [plan(), plan()]
    out, session, _ = run(tmp_path, responses)
    assert out['outcome'] == 'aborted'
    assert not any(n == 'execute_skill' for n, _ in session.calls)


@pytest.mark.parametrize('session', [Session(backend='real'), Session(stopped=True)])
def test_wrong_or_stopped_server_untouched(tmp_path, session):
    out, session, model = run(tmp_path, [], session)
    assert out['outcome'] == 'aborted' and not out['stop_requested']
    assert [n for n, _ in session.calls] == ['get_status'] and not model.seen


def test_motion_failure_stops_without_next_model_decision(tmp_path):
    out, session, model = run(tmp_path, [plan(), execute()], Session(fail_motion=True))
    assert out['outcome'] == 'aborted' and len(model.seen) == 2
    assert [n for n, _ in session.calls].count('execute_skill') == 1
    assert session.calls[-2][0] == 'stop' and session.calls[-1][0] == 'observe'


def test_dry_run_cannot_claim_physical_success(tmp_path):
    out, _, _ = run(tmp_path, [plan(), execute(), call('finish_task', {
        'observation_id': '2', 'verdict': 'success', 'reason': 'dry'})], Session(mode='dry_run'))
    assert out['outcome'] == 'aborted' and out['ai_visual_verdict'] == 'unverified'


@pytest.mark.parametrize('error', [httpx.ReadTimeout('private detail'), RuntimeError('private detail')])
def test_api_failure_stops_and_does_not_log_exception_secret(tmp_path, error):
    out, _, _ = run(tmp_path, [error])
    assert out['stop_requested'] and out['outcome'] == 'aborted'
    assert 'private detail' not in (tmp_path/'result.json').read_text()


def test_response_budget_stops(tmp_path):
    out, session, _ = run(tmp_path, [plan()], settings=Settings(max_responses=1))
    assert out['outcome'] == 'aborted' and out['stop_requested']
    assert not any(n == 'execute_skill' for n, _ in session.calls)


def test_wall_clock_deadline_stops_without_waiting_for_model(tmp_path):
    class HangingModel:
        async def create(self, *args):
            await asyncio.sleep(10)
    agent = APIAgent(Session(), HangingModel(), Settings(max_elapsed_s=.03), tmp_path, 'deadline fixture')
    out = asyncio.run(agent.run())
    assert out['outcome'] == 'aborted' and out['error_type'] == 'TimeoutError'
    assert out['elapsed_s'] < 1 and out['stop_requested']


def test_cancellation_requests_stop_and_writes_result(tmp_path):
    async def check():
        class CancelledModel:
            async def create(self, *args):
                raise asyncio.CancelledError()
        session = Session()
        with pytest.raises(asyncio.CancelledError):
            await APIAgent(session, CancelledModel(), Settings(), tmp_path, 'cancel fixture').run()
        assert session.calls[-1][0] == 'stop'
        assert json.loads((tmp_path/'result.json').read_text())['error_type'] == 'CancelledError'
    asyncio.run(check())


def test_missing_key_fails_before_starting_server(tmp_path, monkeypatch):
    import so101_mcp.api_agent as module
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    monkeypatch.setattr('sys.argv', ['api_agent', '--instruction', 'test'])
    with pytest.raises(SystemExit) as exc:
        module.main()
    assert exc.value.code == 2 and not (tmp_path/'artifacts').exists()


def test_responses_rest_schema_and_http_error_redaction():
    async def check():
        def handler(request):
            body = json.loads(request.content)
            assert body['model'] == 'gpt-6-astra' and body['store'] is False
            assert body['include'] == ['reasoning.encrypted_content']
            assert body['parallel_tool_calls'] is False and 'temperature' not in body
            assert request.headers['Authorization'] == 'Bearer private-fixture'
            return httpx.Response(401, json={'message': 'private-fixture'})
        async with httpx.AsyncClient(transport=httpx.MockTransport(handler)) as http:
            with pytest.raises(AgentError, match='HTTP 401') as exc:
                await ResponsesClient('private-fixture', Settings(), http).create([], [])
            assert 'private-fixture' not in str(exc.value)
    asyncio.run(check())


def test_real_mujoco_mcp_and_mock_api_roundtrip(tmp_path):
    from scripts.verify_api_client import verify
    verification = asyncio.run(verify(tmp_path))
    assert verification['completed_sim_motion_steps'] == 1
    assert verification['distinct_pngs_delivered_to_mock_api'] >= 3
    assert verification['live_api_requests'] == 0
