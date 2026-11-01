"""Inspect real RGB geometry without opening a robot connection or enabling motion."""
import hashlib
import io

import numpy as np
from PIL import Image, ImageDraw

from .metrology import measure_vertical
from .rgb_calibration import rectify_png


def inspect_height(raw_png, calibration, bottom, top, center=None, support_z_m=0.,
                   pixel_error_px=2., reference_height_mm=None, tolerance_mm=3.):
    """Pixels refer to the rectified image. Reference height is evaluation only."""
    if not np.isfinite(tolerance_mm) or tolerance_mm <= 0:
        raise ValueError('Independent comparison tolerance must be positive')
    if reference_height_mm is not None and (not np.isfinite(reference_height_mm) or reference_height_mm <= 0):
        raise ValueError('Independent reference height must be positive')
    png, meta = rectify_png(raw_png, calibration)
    result = {'mode': 'offline_camera_inspection', 'motor_connections': 0, 'motor_goal_writes': 0,
              'calibration_verified': calibration.get('verified') is True,
              'calibration_sha256': calibration['calibration_sha256'],
              'input_image_sha256': hashlib.sha256(raw_png).hexdigest(),
              'pixel_image': 'rectified full-resolution image; not original distorted pixels',
              'motion_enabled': False, 'automatic_verification_change': False}
    try:
        # Do not pass the reference height to the estimator.
        measured = measure_vertical(meta, bottom, top, center, support_z_m, pixel_error_px)
        result.update(measurement_status='estimated', measurement=measured)
        if reference_height_mm is not None:
            error = abs(measured['estimated_height_m']*1000-reference_height_mm)
            result['independent_comparison'] = {'reference_height_mm': reference_height_mm,
                'absolute_error_mm': error, 'tolerance_mm': tolerance_mm, 'within_tolerance': error <= tolerance_mm,
                'scope': 'This selected edge only; not whole-camera or robot verification'}
    except ValueError as error:
        result.update(measurement_status='rejected', reason=str(error))
    im = Image.open(io.BytesIO(png)).convert('RGB')
    draw = ImageDraw.Draw(im)
    for label, point in [('bottom', bottom), ('top', top), ('centre', center)]:
        if point is None:
            continue
        point = np.asarray(point, dtype=float)
        if point.shape == (2,) and np.isfinite(point).all() and 0 <= point[0] < im.width and 0 <= point[1] < im.height:
            u, v = point
            draw.ellipse([u-4, v-4, u+4, v+4], outline='yellow', width=2)
            draw.text((u+6, v+4), label, fill='yellow', stroke_width=1, stroke_fill='black')
    draw.rectangle([0, im.height-40, im.width, im.height], fill='#102033')
    message = (f"Estimated {result['measurement']['estimated_height_m']*1000:.2f} mm"
               if result['measurement_status'] == 'estimated' else 'Measurement rejected: inspect selected edge/view')
    draw.text((8, im.height-34), message, fill='yellow')
    draw.text((8, im.height-18), 'OFFLINE CHECK ONLY | no motor connection | no automatic calibration approval', fill='white')
    stream = io.BytesIO(); im.save(stream, format='PNG')
    return result, stream.getvalue()
