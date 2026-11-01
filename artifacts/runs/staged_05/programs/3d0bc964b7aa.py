"""Generated for one observed scene. Run from the project Python environment.
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

REQUEST = {'name': 'prepared_cartesian_step', 'args': {'program_id': '3d0bc964b7aa'}, 'observation_id': '7f51066459f4', 'instruction': '高い方のブロックを箱へ、低い方を机の空いた場所へ移して。画像から高さを測って選び、各段階で確認する。', 'request_id': 'generated-3d0bc964b7aa'}


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
                json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
            print(json.dumps(evidence, ensure_ascii=False, indent=2))
            if status['state'] != 'completed':
                raise RuntimeError('Robot command did not complete')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8765/mcp')
    asyncio.run(main(parser.parse_args().url))
