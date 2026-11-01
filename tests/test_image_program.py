"""Meaningful geometry / generated-program checks; not AI vision trials."""
import ast
import json
import time

import numpy as np
import pytest

from so101_mcp.config import load_config
from so101_mcp.controller import Controller
from so101_mcp.geometry import locate_top, pixel_to_plane
from so101_mcp.sim import SimBackend


def calibration():
    return {'width': 640, 'height': 480, 'focal_px': 600.,
            'camera_position_m': [.2, -.1, .6], 'camera_to_work_rotation': np.eye(3).tolist()}


def test_plane_localization_known_geometry_and_height_dependency():
    c = calibration()
    assert pixel_to_plane(c, 380, 180, 0.) == pytest.approx([.26, -.04, 0.])
    assert pixel_to_plane(c, 380, 180, .1) == pytest.approx([.25, -.05, .1])
    # Z is supplied, NOT measured. A height error also changes off-axis XY.
    assert pixel_to_plane(c, 320, 240, .03) == pytest.approx([.2, -.1, .03])


@pytest.mark.parametrize('u,v,z', [(float('nan'), 240, 0), (-1, 240, 0), (640, 240, 0),
                                   (320, 480, 0), (320, 240, .7)])
def test_invalid_rays_rejected(u, v, z):
    with pytest.raises(ValueError):
        pixel_to_plane(calibration(), u, v, z)


def test_parallel_and_invalid_calibration_rejected():
    c = calibration()
    c['camera_to_work_rotation'] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    with pytest.raises(ValueError, match='parallel'):
        pixel_to_plane(c, 320, 240, 0.)
    c['camera_to_work_rotation'] = np.zeros((3, 3)).tolist()
    with pytest.raises(ValueError, match='calibration'):
        pixel_to_plane(c, 320, 240, 0.)


def test_workspace_includes_pixel_tolerance():
    settings = {'object_top_z_m': .03, 'pixel_tolerance_px': 3,
                'pick_xy_bounds_m': [[.2, .3], [-.2, 0.]]}
    with pytest.raises(ValueError, match='workspace'):
        locate_top(calibration(), 320, 240, settings)  # center allowed, envelope crosses edge


@pytest.fixture
def continuous(tmp_path):
    config = load_config('sim', 'config/physical_ai/sim_continuous.json')
    backend = SimBackend(config, tmp_path, record=False)
    controller = Controller(backend, config, tmp_path)
    yield controller, backend
    controller.close()


def test_different_image_points_generate_different_inspectable_plans_without_motion(continuous):
    controller, backend = continuous
    meta, _ = controller.observe()
    oid = meta['observation_id']
    first, preview = controller.prepare_pick(oid, 400, 417, 'test block', 'software test pixel, not AI')
    second, _ = controller.prepare_pick(oid, 380, 409, 'test block', 'second software test pixel')
    assert preview.startswith(b'\x89PNG')
    assert backend.elapsed == 0  # planning must not advance physics or issue motion
    assert controller.get_status()['state'] == 'idle'
    assert first['localization']['pick_xy_m'] != second['localization']['pick_xy_m']
    assert first['trajectory'][0]['target'] != second['trajectory'][0]['target']
    assert 'seed' not in json.dumps(first) and 'red_cube' not in json.dumps(first)
    source = controller.run_dir/first['files_relative_to_run_dir']['script']
    ast.parse(source.read_text())
    assert first['program_id'] in source.read_text()
    assert first['localization']['assumed_top_z_m'] == .03


def test_generated_program_cannot_bypass_observation_or_stop(continuous):
    controller, backend = continuous
    meta, _ = controller.observe()
    oid = meta['observation_id']
    program, _ = controller.prepare_pick(oid, 400, 417, 'test block', 'software test pixel')
    args = {'program_id': program['program_id']}
    with pytest.raises(ValueError, match='another image or instruction'):
        controller.execute_skill('prepared_pick', args, oid, 'different instruction', 'wrong')
    with pytest.raises(ValueError, match='direct coordinates'):
        controller.execute_skill('pick_place_xy', {'x_m': .24, 'y_m': -.05, 'destination': 'box'}, oid, 'test', 'bypass')
    controller.execute_skill('observe_pose', {}, oid, 'software test', 'move')
    deadline = time.monotonic()+5
    while controller.get_status()['state'] == 'running' and time.monotonic() < deadline:
        time.sleep(.01)
    with pytest.raises(ValueError, match='moved'):
        controller.execute_skill('prepared_pick', args, oid, 'test block', 'stale')
    controller.stop()
    with pytest.raises(RuntimeError, match='latched'):
        controller.prepare_pick(oid, 400, 417, 'test', 'software test')


def test_bad_pixel_preparation_writes_no_program_and_moves_nothing(continuous):
    controller, backend = continuous
    oid = controller.observe()[0]['observation_id']
    with pytest.raises(ValueError, match='workspace'):
        controller.prepare_pick(oid, 600, 300, 'test', 'software test outside pick area')
    assert not (controller.run_dir/'programs').exists()
    assert backend.elapsed == 0
