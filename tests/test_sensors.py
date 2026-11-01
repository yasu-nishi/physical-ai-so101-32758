"""Depth geometry and sensor transport tests, not host vision-success trials."""
import asyncio
import base64
import io
import json

import numpy as np
from PIL import Image
import pytest

from so101_mcp.config import load_config
from so101_mcp.controller import Controller
from so101_mcp.geometry import pixel_to_depth
from so101_mcp.programs import RUNNER
from so101_mcp.sensors import depth_preview, sample_depth
from so101_mcp.sim import SimBackend


def calibration():
    return {'width': 640, 'height': 480, 'focal_px': 600.,
            'camera_position_m': [.2, -.1, .6], 'camera_to_work_rotation': np.eye(3).tolist(),
            'work_frame': 'robot_base_table_z0'}


def test_camera_forward_depth_is_not_euclidean_range():
    c = calibration()
    assert pixel_to_depth(c, 380, 180, .5) == pytest.approx([.25, -.05, .1])
    result = sample_depth(np.full((480, 640), .5), c, 380, 180)
    assert result['depth_m'] == .5
    assert result['range_m'] == pytest.approx(np.sqrt(.5**2+2*.05**2))
    assert result['height_above_table_m'] == pytest.approx(.1)


@pytest.mark.parametrize('depth', [float('nan'), float('inf'), 0., -.1])
def test_invalid_depth_is_not_substituted(depth):
    with pytest.raises(ValueError, match='positive'):
        sample_depth(np.full((480, 640), depth), calibration(), 320, 240)


def test_pixel_centres_and_rotated_camera():
    c = calibration()
    c['principal_point_px'] = [319.5, 239.5]
    c['camera_to_work_rotation'] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    assert pixel_to_depth(c, 379.5, 179.5, .5) == pytest.approx([.15, -.05, .1])
    with pytest.raises(ValueError, match='integer'):
        sample_depth(np.full((480, 640), .5), c, 379.5, 179.5)


def test_preview_preserves_shape_and_handles_invalid_values():
    depth = np.full((480, 640), .55)
    depth[100, 100] = np.nan
    image = Image.open(io.BytesIO(depth_preview(depth)))
    assert image.size == (640, 480)
    assert image.getpixel((100, 100)) == (0, 0, 0)


def test_snapshot_alignment_height_and_robot_pose(tmp_path):
    config = load_config('sim', 'config/physical_ai/sim_continuous.json')
    config.update(depth_camera=True, portable_observations=False)  # isolated sim-only diagnostic
    backend = SimBackend(config, tmp_path, seed=42, record=False)
    control = Controller(backend, config, tmp_path)
    try:
        meta, images = control.observe_bundle()
        oid = meta['observation_id']
        assert [im['role'] for im in meta['images']] == ['rgb', 'depth_visualization']
        assert len(images) == 2
        assert backend.elapsed == 0
        # Test-only independent surface truth. Never used by the host policy.
        cal = meta['camera_calibration']
        for name in ['red_cube', 'blue_cube']:
            top = backend.data.xpos[backend.model.body(name).id].copy()
            top[2] += .015
            local = np.asarray(cal['camera_to_work_rotation']).T @ (top-np.asarray(cal['camera_position_m']))
            center = np.asarray(cal['principal_point_px'])
            pixel = np.rint(center+cal['focal_px']*np.array([local[0], -local[1]])/-local[2]).astype(int)
            measured = control.sample_depth(oid, *pixel)
            assert measured['work_position_m'] == pytest.approx(top, abs=.0007)
            assert measured['height_above_table_m'] == pytest.approx(.03, abs=.0001)
        # A table pixel proves height is measured, not a constant 30 mm.
        assert control.sample_depth(oid, 570, 500)['height_above_table_m'] == pytest.approx(0, abs=1e-5)
        eff = meta['sensors']['end_effector']
        assert eff['position_m'] == pytest.approx(backend.data.site_xpos[backend.site])
        assert np.linalg.norm(eff['quaternion_wxyz']) == pytest.approx(1)
        with pytest.raises(ValueError, match='positive'):
            control.sample_depth(oid, 100, 10)  # RGB header is not a visible surface
        assert np.load(tmp_path/f'observe_{oid}_depth.npz')['depth_m'].shape == (720, 960)
        control.execute_skill('observe_pose', {}, oid, 'sensor staleness software test', 'move')
        with pytest.raises(ValueError, match='moved'):
            control.sample_depth(oid, 570, 500)
    finally:
        control.close()


def test_unavailable_depth_is_reported_explicitly(tmp_path):
    config = load_config('sim')
    config['depth_camera'] = False
    backend = SimBackend(config, tmp_path, record=False)
    control = Controller(backend, config, tmp_path)
    try:
        meta, images = control.observe_bundle()
        assert len(images) == 1
        assert not meta['capabilities']['depth_sampling']
        with pytest.raises(ValueError, match='unavailable'):
            control.sample_depth(meta['observation_id'], 480, 500)
    finally:
        control.close()


def test_generated_runner_preserves_rgb_and_depth_as_separate_files(tmp_path, monkeypatch):
    """Exercise the exported runner's I/O using a fake MCP transport."""
    from contextlib import asynccontextmanager
    from types import SimpleNamespace
    import mcp
    import mcp.client.streamable_http
    from mcp.types import ImageContent, TextContent
    @asynccontextmanager
    async def transport(url):
        yield None, None, None
    class Session:
        def __init__(self, *args): pass
        async def __aenter__(self): return self
        async def __aexit__(self, *args): pass
        async def initialize(self): pass
        async def call_tool(self, name, args):
            data = {'execute_skill': {'action_id': 'test'},
                    'get_status': {'state': 'completed', 'last_action': {'action_id': 'test'}},
                    'observe': {'observation_id': 'after'}}[name]
            content = [TextContent(type='text', text=json.dumps(data))]
            if name == 'observe':
                content += [ImageContent(type='image', mimeType='image/png', data=base64.b64encode(v).decode())
                            for v in [b'rgb', b'depth']]
            return SimpleNamespace(content=content, isError=False)
    monkeypatch.setattr(mcp, 'ClientSession', Session)
    monkeypatch.setattr(mcp.client.streamable_http, 'streamable_http_client', transport)
    namespace = {'__name__': 'generated_runner_test', '__file__': str(tmp_path/'program.py')}
    exec(compile(RUNNER.replace('__REQUEST__', '{}'), '<generated-test>', 'exec'), namespace)
    asyncio.run(namespace['main']('test://fake'))
    assert (tmp_path/'program_after.png').read_bytes() == b'rgb'
    assert (tmp_path/'program_after_1.png').read_bytes() == b'depth'
