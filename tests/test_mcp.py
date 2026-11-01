import asyncio
import base64
import json
import sys
import time
import pytest

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


@pytest.mark.parametrize('stop_method', ['mcp', 'file'])
def test_stdio_protocol_returns_real_png_and_common_tools(tmp_path, stop_method):
    async def check():
        params = StdioServerParameters(command=sys.executable,
            args=['-m', 'so101_mcp', 'serve', '--backend', 'sim', '--no-record', '--realtime', '--run-dir', str(tmp_path/'mcp')])
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                listed = await session.list_tools()
                assert {t.name for t in listed.tools} == {'observe', 'sample_depth', 'locate_on_plane', 'measure_height', 'prepare_cartesian_step', 'prepare_pick', 'prepare_step', 'execute_skill', 'get_status', 'stop', 'report_result'}
                result = await session.call_tool('observe')
                assert not result.isError
                images = [c for c in result.content if c.type == 'image']
                assert len(images) == 1
                assert all(c.mimeType == 'image/png' and base64.b64decode(c.data).startswith(b'\x89PNG') for c in images)
                meta = json.loads(next(c.text for c in result.content if c.type == 'text'))
                assert meta['backend'] == 'sim'
                assert 'seed' not in json.dumps(meta) and 'inside_box' not in json.dumps(meta)
                assert not meta['capabilities']['depth_sampling']
                assert not meta['sensors']['end_effector']['available']
                assert meta['capabilities']['known_plane_localization']
                sample = await session.call_tool('sample_depth', {'observation_id': meta['observation_id'],
                                                                 'pixel_x': 480, 'pixel_y': 500})
                assert sample.isError  # default profile must not leak virtual depth
                located = await session.call_tool('locate_on_plane', {'observation_id': meta['observation_id'],
                    'pixel_x': 480, 'pixel_y': 500, 'plane_z_m': .03})
                assert not located.isError
                measured = json.loads(next(c.text for c in located.content if c.type == 'text'))
                assert measured['height_measured'] is False
                assert measured['work_position_m'][2] == pytest.approx(.03)
                assert len(measured['work_position_m']) == 3
                step = await session.call_tool('prepare_step', {'observation_id': meta['observation_id'],
                    'joint_delta_deg': {'shoulder_pan': 1.}, 'gripper_delta': 0.,
                    'instruction': 'software transport test', 'visual_reason': 'test fixture',
                    'expected_visual_change': 'must reobserve before another step'})
                assert not step.isError
                step_meta = json.loads(next(c.text for c in step.content if c.type == 'text'))
                assert step_meta['execution_skill'] == 'prepared_step'
                assert len(step_meta['trajectory']) == 1
                assert step_meta['automatic_next_step'] is False
                assert any(c.type == 'image' for c in step.content)
                action = await session.call_tool('execute_skill', {
                    'name': 'pick_place', 'args': {'slot': 'slot_a', 'destination': 'box'},
                    'observation_id': meta['observation_id'], 'instruction': 'software stop test, not AI trial',
                    'request_id': 'stop-test'})
                assert not action.isError
                began = time.monotonic()
                if stop_method == 'mcp':
                    assert not (await session.call_tool('stop')).isError
                else:
                    (tmp_path/'mcp/STOP').touch()
                while time.monotonic()-began < 1:
                    status = await session.call_tool('get_status')
                    data = json.loads(next(c.text for c in status.content if c.type == 'text'))
                    if data['state'] == 'stopped': break
                    await asyncio.sleep(.01)
                assert data['state'] == 'stopped'
                assert data['last_action']['command_status'] == 'stopped'
                assert time.monotonic()-began < .5
    asyncio.run(check())


@pytest.mark.parametrize('orientation,locked,dz',[
    ('preserve_tilt',None,.01),('position_only',['elbow_flex'],.005)])
def test_cartesian_orientation_roundtrip_returns_fresh_png(tmp_path,orientation,locked,dz):
    """Protocol + real MuJoCo stepping; this is a motion test, not task success."""
    import numpy as np
    from so101_mcp.config import ROOT,load_config
    from so101_mcp.joints import ARM
    cfg=load_config('sim',ROOT/'config/physical_ai/sim_staged.json')
    cfg['home']={'joints_deg':dict(zip(ARM,[-38.,-81.,76.,34.,5.],strict=True)),'gripper_open':.34}
    # Isolate an empty-space robot-motion fixture within the existing scene.
    cfg['idle_physics']=False
    config_path=tmp_path/'angled.json';config_path.write_text(json.dumps(cfg))
    async def check():
        params=StdioServerParameters(command=sys.executable,args=['-m','so101_mcp','serve',
            '--backend','sim','--no-record','--config',str(config_path),'--run-dir',str(tmp_path/'run')])
        async with stdio_client(params) as (read,write):
            async with ClientSession(read,write) as session:
                await session.initialize()
                async def call(name,args=None):
                    result=await session.call_tool(name,args or {})
                    assert not result.isError,result
                    meta=json.loads(next(x.text for x in result.content if x.type=='text'))
                    return result,meta
                _,before=await call('observe')
                xyz=np.array(before['sensors']['end_effector']['position_m'])
                target=(xyz+[0,0,dz]).tolist()
                assert orientation in before['cartesian_tool_orientations']
                _,plan=await call('prepare_cartesian_step',{'observation_id':before['observation_id'],
                    'target_xyz_m':target,'gripper_open':None,'instruction':'software protocol fixture',
                    'visual_reason':'robot-only test pose; no object selection',
                    'expected_visual_change':f'{dz*1000:g} mm upward translation, inspect the next RGB',
                    'tool_orientation':orientation,'locked_joints':locked})
                assert plan['tool_orientation']==orientation
                assert plan['locked_joints']==(locked or [])
                if locked:
                    assert plan['trajectory'][0]['target']['joints_deg']['elbow_flex']==before['joints_deg']['elbow_flex']
                assert len(plan['trajectory'])==1 and not plan['automatic_next_step']
                _,action=await call('execute_skill',{'name':'prepared_cartesian_step',
                    'args':{'program_id':plan['program_id']},'observation_id':before['observation_id'],
                    'instruction':plan['instruction'],'request_id':orientation+'-test'})
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    _,status=await call('get_status')
                    if status['state']!='running':break
                    await asyncio.sleep(.02)
                assert status['state']=='completed'
                result,after=await call('observe')
                assert after['observation_id']!=before['observation_id']
                assert after['sensors']['end_effector']['position_m']==pytest.approx(target,abs=.002)
                if locked:
                    assert after['joints_deg']['elbow_flex']==pytest.approx(before['joints_deg']['elbow_flex'],abs=.2)
                assert any(x.type=='image' and base64.b64decode(x.data).startswith(b'\x89PNG') for x in result.content)
                assert after['status']['last_action']['task_success']=='unverified'
    asyncio.run(check())
