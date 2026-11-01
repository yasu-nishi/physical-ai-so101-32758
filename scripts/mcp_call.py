"""Transparent MCP client bridge; never chooses a block or calls an LLM.

Each invocation is a real MCP initialize/list_tools or tools/call over HTTP.
Images are decoded verbatim for a host that cannot attach new tools mid-session.
"""
import argparse
import asyncio
import base64
import json
from pathlib import Path
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client


async def call(args):
    async with streamable_http_client(args.url) as (read, write, _):
        async with ClientSession(read, write) as session:
            await session.initialize()
            if args.tool == 'list':
                print((await session.list_tools()).model_dump_json(indent=2))
                return 0
            result = await session.call_tool(args.tool, json.loads(args.args))
            out = {'isError': result.isError, 'text': [], 'images': []}
            for part in result.content:
                if part.type == 'text':
                    try:
                        out['text'].append(json.loads(part.text))
                    except json.JSONDecodeError:
                        out['text'].append(part.text)
                elif part.type == 'image':
                    args.output.mkdir(parents=True, exist_ok=True)
                    path = args.output/f'{args.tool}_{len(out["images"])}.png'
                    path.write_bytes(base64.b64decode(part.data))
                    out['images'].append({'path': str(path), 'mimeType': part.mimeType})
            print(json.dumps(out, indent=2, ensure_ascii=False))
            return 1 if result.isError else 0


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('tool')
    p.add_argument('--args', default='{}', help='JSON tool arguments, never a policy')
    p.add_argument('--url', default='http://127.0.0.1:8765/mcp')
    p.add_argument('--output', type=Path, default=Path('artifacts/client'))
    raise SystemExit(asyncio.run(call(p.parse_args())))
