"""Export an inspected, bounded motion plan as JSON + executable MCP client.

This exporter is deterministic. The connected AI supplies the image selection
and reasoning. The server never evaluates Python or invokes a second LLM.
"""
import hashlib
import io
import json

from PIL import Image, ImageDraw


RUNNER = '''"""Generated for one observed scene. Run from the project Python environment.
The server validates the observation age, robot generation and exact program ID.
This file cannot replay an old scene on a restarted server.
"""
import argparse
import asyncio
import base64
import json
from pathlib import Path
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

REQUEST = __REQUEST__


async def main(url):
    async with streamable_http_client(url) as (read, write, _):
        async with ClientSession(read, write) as session:
            await session.initialize()
            async def call(name, args=None):
                result = await session.call_tool(name, args or {})
                texts = [p.text for p in result.content if p.type == 'text']
                if result.isError:
                    raise RuntimeError(' | '.join(texts))
                return result, json.loads(texts[0])
            _, action = await call('execute_skill', REQUEST)
            for _ in range(1000):
                _, status = await call('get_status')
                if status['state'] != 'running':
                    break
                await asyncio.sleep(.1)
            else:
                await call('stop')
                raise TimeoutError('Generated runner watchdog: stop requested')
            if (status.get('last_action') or {}).get('action_id') != action['action_id']:
                raise RuntimeError('Action result does not match this program')
            result, observation = await call('observe')
            prefix = Path(__file__).with_suffix('')
            image_index = 0
            for part in result.content:
                if part.type == 'image':
                    suffix = '_after.png' if image_index == 0 else f'_after_{image_index}.png'
                    prefix.with_name(prefix.name+suffix).write_bytes(base64.b64decode(part.data))
                    image_index += 1
            evidence = {'action': action, 'status': status, 'observation': observation,
                        'task_success': 'unverified: host must inspect the new PNG and report_result'}
            prefix.with_name(prefix.name+'_execution.json').write_text(
                json.dumps(evidence, ensure_ascii=False, indent=2)+'\\n')
            print(json.dumps(evidence, ensure_ascii=False, indent=2))
            if status['state'] != 'completed':
                raise RuntimeError('Robot command did not complete')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8765/mcp')
    asyncio.run(main(parser.parse_args().url))
'''


def export_program(run_dir, program, before_png):
    directory = run_dir/'programs'
    directory.mkdir(exist_ok=True)
    pid = program['program_id']
    source = RUNNER.replace('__REQUEST__', repr({
        'name': program.get('execution_skill', 'prepared_pick'), 'args': {'program_id': pid},
        'observation_id': program['observation_id'], 'instruction': program['instruction'],
        'request_id': 'generated-'+pid}))
    code = directory/f'{pid}.py'
    code.write_text(source)
    program['script_sha256'] = hashlib.sha256(source.encode()).hexdigest()
    description = directory/f'{pid}.json'
    description.write_text(json.dumps(program, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    im = Image.open(before_png).convert('RGB')
    draw = ImageDraw.Draw(im)
    if 'localization' in program:
        x, y = program['localization']['pixel_xy']
        draw.ellipse([x-7, y-7, x+7, y+7], outline='yellow', width=2)
        draw.line([x-13, y, x+13, y], fill='yellow', width=1)
        draw.line([x, y-13, x, y+13], fill='yellow', width=1)
        wx, wy = program['localization']['pick_xy_m']
        label = f'AI selected ({x:.1f}, {y:.1f}) px -> ({wx*1000:.1f}, {wy*1000:.1f}) mm'
    elif 'target_xyz_m' in program:
        target=program['target_xyz_m']
        label=f'ONE step: XYZ {target} m' if target is not None else f"ONE step: gripper {program['target_gripper_open']:.3f}"
        label+=' | observe again before deciding next action'
    else:
        increments = ', '.join(f'{n} {v:+g} deg' for n, v in program['joint_delta_deg'].items())
        label = f'ONE step: {increments or "gripper " + format(program["gripper_delta"], "+g")} | observe again; no automatic next step'
    draw.rectangle([0, im.height-28, im.width, im.height], fill='#102033')
    draw.text((12, im.height-20), label, fill='yellow')
    stream = io.BytesIO()
    im.save(stream, format='PNG')
    preview = stream.getvalue()
    (directory/f'{pid}_preview.png').write_bytes(preview)
    return {'script': f'programs/{pid}.py', 'plan': f'programs/{pid}.json',
            'preview': f'programs/{pid}_preview.png',
            'script_sha256': program['script_sha256'],
            'plan_sha256': hashlib.sha256(description.read_bytes()).hexdigest()}, preview
