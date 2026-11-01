import json
import time
from pathlib import Path

import pytest

from so101_mcp.config import load_config
from so101_mcp.controller import Controller
from so101_mcp.sim import SimBackend


@pytest.mark.parametrize('slot', ['slot_a', 'slot_b'])
def test_physical_pick_transfer_release_without_teleport(tmp_path, slot):
    config = load_config('sim')
    backend = SimBackend(config, tmp_path, seed=0, record=False)
    controller = Controller(backend, config, tmp_path)
    try:
        meta, png = controller.observe()
        assert png.startswith(b'\x89PNG')
        assert all(key not in json.dumps(meta) for key in ['red_cube', 'blue_cube', 'inside_box'])
        # Robot/camera poses are permitted proprioception/calibration, never object poses.
        assert not meta['sensors']['end_effector']['available']
        assert not meta['sensors']['depth']['available']
        controller.execute_skill('pick_place', {'slot': slot, 'destination': 'box'}, meta['observation_id'],
                                 'integration test: fixed slot, NOT an AI vision trial', 'test')
        deadline = time.monotonic()+30
        while controller.get_status()['state'] == 'running' and time.monotonic() < deadline:
            time.sleep(.02)
        assert controller.get_status()['state'] == 'completed', controller.get_status()
        # Evaluation stays outside the MCP/selection path.
        deadline = time.monotonic()+2
        while not (tmp_path/'evaluation_private.json').exists() and time.monotonic() < deadline:
            time.sleep(.01)
        result = json.loads((tmp_path/'evaluation_private.json').read_text())
        target = 'red' if slot == 'slot_a' else 'blue'
        other = 'blue' if target == 'red' else 'red'
        assert result['blocks'][target]['inside_box']
        assert result['blocks'][target]['contact_steps'] > 100
        assert result['blocks'][target]['max_height_m'] > .05
        assert not result['blocks'][target]['gripper_contact_at_end']
        assert not result['blocks'][other]['inside_box']
    finally:
        controller.close()
