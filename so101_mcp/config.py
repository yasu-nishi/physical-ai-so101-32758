import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def real_camera_dimensions(config, env):
    """Reuse the existing local camera settings for both calibration and operation."""
    result = {}
    for key in ('width', 'height', 'fps'):
        result[key] = int(env.get(config.get(f'camera_{key}_env', ''), config[f'camera_{key}']))
    if result['width'] < 64 or result['height'] < 64 or not 1 <= result['fps'] <= 120:
        raise ValueError('Invalid real RGB camera resolution or frame rate')
    return result


def load_config(backend, path=None):
    if backend not in ('sim', 'real'):
        raise ValueError('Explicit backend sim or real is required')
    p = Path(path) if path else ROOT / f'config/physical_ai/{backend}.json'
    config = json.loads(p.read_text())
    if config['backend'] != backend:
        raise ValueError('Config backend differs from selected backend')
    return config
