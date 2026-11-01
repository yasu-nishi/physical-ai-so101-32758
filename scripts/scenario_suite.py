"""Mechanical sim experiment runner. Decisions and pixels must come from the host AI.

No LLM client, object detector, segmentation or object truth is used here. Each
case runs a separate localhost MCP server. The host inspects every returned PNG.
"""
import argparse
import asyncio
import base64
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT/'tmp/scenario_suite.local.json'
CASES = ROOT/'config/physical_ai/suite/cases.json'


async def call(tool, arguments=None):
    client = httpx.AsyncClient(timeout=15.)
    async with client:
        async with streamable_http_client('http://127.0.0.1:8765/mcp', http_client=client) as (read, write, _):
            async with ClientSession(read, write) as session:
                await session.initialize()
                result = await session.call_tool(tool, arguments or {})
    state = json.loads(STATE.read_text())
    directory = ROOT/state['run_dir']/'client'
    directory.mkdir(exist_ok=True)
    ordinal = len(list(directory.glob('*.json')))
    texts, images = [], []
    for block in result.content:
        if block.type == 'text':
            try: texts.append(json.loads(block.text))
            except json.JSONDecodeError: texts.append(block.text)
        elif block.type == 'image':
            path = directory/f'{ordinal:03}_{tool}_{len(images)}.png'
            path.write_bytes(base64.b64decode(block.data))
            images.append(str(path.relative_to(ROOT)))
    response = {'isError': result.isError, 'text': texts, 'images': images}
    (directory/f'{ordinal:03}_{tool}.json').write_text(json.dumps(response,ensure_ascii=False,indent=2)+'\n')
    return response


def compact(response):
    if response['isError']:
        return response
    texts = []
    keys = ('observation_id', 'backend', 'joints_deg', 'gripper_open', 'state', 'action_id',
            'last_action', 'stop_latched', 'program_id', 'planned_duration_s', 'localization',
            'files_relative_to_run_dir', 'command_status', 'accepted', 'ai_visual_verdict', 'reason',
            'stop_requested', 'joint_delta_deg', 'expected_visual_change')
    for data in response['text']:
        if not isinstance(data, dict): texts.append(data); continue
        texts.append({k: data[k] for k in keys if k in data})
    return {**response, 'text': texts}


def start(case_id):
    if STATE.exists():
        raise SystemExit('Finish the current suite case before starting another')
    cases = json.loads(CASES.read_text())
    case = next(c for c in cases if c['case_id'] == case_id)
    run = ROOT/'artifacts/runs'/case_id
    if run.exists() and any(run.iterdir()):
        raise SystemExit('Case already exists; retain it and choose a new ID for a repeat')
    logdir = ROOT/'tmp/suite_logs'
    logdir.mkdir(parents=True,exist_ok=True)
    with (logdir/f'{case_id}.log').open('w') as log:
        process = subprocess.Popen([sys.executable,'-m','so101_mcp','serve','--backend','sim',
            '--config',case['config'],'--transport','streamable-http','--run-dir',str(run),
            '--seed',str(case['seed'])],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=log,
            start_new_session=True)
    STATE.parent.mkdir(exist_ok=True)
    STATE.write_text(json.dumps({'pid':process.pid,'run_dir':str(run.relative_to(ROOT)),
                                 'case':case},ensure_ascii=False,indent=2)+'\n')
    for _ in range(50):
        if process.poll() is not None:
            raise RuntimeError('Simulation server exited; inspect local suite log')
        try:
            result = asyncio.run(call('observe'))
            if result['isError']: raise RuntimeError(str(result['text']))
            print(json.dumps({'case_id':case_id,'category':case['category'],
                              'instruction':case['instruction'],**compact(result)},ensure_ascii=False))
            return
        except (Exception, BaseExceptionGroup):
            time.sleep(.2)
    raise TimeoutError('Simulation server did not become ready')


def run_program(pid):
    if len(pid) != 12 or any(c not in '0123456789abcdef' for c in pid):
        raise ValueError('Use the returned program ID')
    state = json.loads(STATE.read_text())
    path = ROOT/state['run_dir']/f'programs/{pid}.py'
    result = subprocess.run([sys.executable,str(path)],cwd=ROOT,capture_output=True,text=True,timeout=110)
    if result.returncode:
        print(result.stdout)
        raise RuntimeError(result.stderr)
    data = json.loads(result.stdout)
    obs = data['observation']
    print(json.dumps({'action':data['status']['last_action'], 'observation_id':obs['observation_id'],
                      'joints_deg':obs['joints_deg'],'gripper_open':obs['gripper_open'],
                      'tool':obs.get('sensors',{}).get('end_effector'),
                      'image':str(path.with_name(pid+'_after.png').relative_to(ROOT))},ensure_ascii=False))


def finish(outcome, reason):
    state = json.loads(STATE.read_text())
    run = ROOT/state['run_dir']
    stop = asyncio.run(call('stop'))
    time.sleep(.1)
    status = asyncio.run(call('get_status'))
    # Avoid signalling an unrelated process if a stale PID file was retained.
    command = subprocess.run(['ps','-p',str(state['pid']),'-o','args='],capture_output=True,text=True).stdout
    if str(run) not in command or 'so101_mcp' not in command:
        raise RuntimeError('Server process identity changed; do not signal this PID')
    os.kill(state['pid'], signal.SIGINT)
    for _ in range(50):
        command = subprocess.run(['ps','-p',str(state['pid']),'-o','stat='],capture_output=True,text=True).stdout.strip()
        if not command or command.startswith('Z'): break
        time.sleep(.1)
    else: raise TimeoutError('Server did not close; retain state for inspection')
    result = {'case_id':state['case']['case_id'],'category':state['case']['category'],
              'instruction':state['case']['instruction'],'host_outcome':outcome,'host_reason':reason,
              'stop_status':status,'scope':'simulation only; AI inspected RGB; no real hardware'}
    (run/'case_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    STATE.unlink()
    print(json.dumps({'case_id':result['case_id'],'host_outcome':outcome,'closed':True},ensure_ascii=False))


def main():
    p=argparse.ArgumentParser()
    sub=p.add_subparsers(dest='command',required=True)
    q=sub.add_parser('start');q.add_argument('case_id')
    q=sub.add_parser('call');q.add_argument('tool');q.add_argument('--args',default='{}')
    q=sub.add_parser('step');q.add_argument('--args',required=True)
    q=sub.add_parser('run');q.add_argument('program_id')
    q=sub.add_parser('finish');q.add_argument('outcome');q.add_argument('reason')
    a=p.parse_args()
    if a.command=='start': start(a.case_id)
    elif a.command=='call': print(json.dumps(compact(asyncio.run(call(a.tool,json.loads(a.args)))),ensure_ascii=False))
    elif a.command=='run': run_program(a.program_id)
    elif a.command=='step':
        response=asyncio.run(call('prepare_step',json.loads(a.args)))
        if response['isError']: raise RuntimeError(str(response['text']))
        run_program(response['text'][0]['program_id'])
    else: finish(a.outcome,a.reason)


if __name__=='__main__': main()
