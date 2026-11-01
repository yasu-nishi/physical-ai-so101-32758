"""Offline check of robot-only FK against independent real measurements.

No serial connection, motor writes, calibration fitting or automatic ARM.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from so101_mcp.config import load_config
from so101_mcp.joints import Pose
from so101_mcp.kinematics import RobotKinematics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--output',type=Path,default=Path('artifacts/kinematics_check.local.json'))
    p.add_argument('--position-tolerance-mm',type=float,default=3.)
    p.add_argument('--axis-tolerance-deg',type=float,default=3.)
    a=p.parse_args()
    samples=json.loads(a.samples.read_text())['samples']
    if len(samples)<6:raise SystemExit('At least six independently measured poses are required')
    kin=RobotKinematics(load_config('real',a.config))
    rows=[];refs=[]
    for i,s in enumerate(samples):
        fk=kin.forward(Pose.from_dict(s['pose']))
        ref=np.asarray(s['reference_position_m'],dtype=float)
        axis=np.asarray(s['reference_tool_x_axis'],dtype=float)
        if ref.shape!=(3,) or axis.shape!=(3,) or not np.isfinite([ref,axis]).all() or np.linalg.norm(axis)<.9:
            raise SystemExit('Finite measured reference position and unit tool X axis required')
        axis=axis/np.linalg.norm(axis)
        error=1000*float(np.linalg.norm(np.asarray(fk['position_m'])-ref))
        angle=float(np.rad2deg(np.arccos(np.clip(np.asarray(fk['local_axes_in_work'])[:,0]@axis,-1,1))))
        rows.append({'sample':i,'position_error_mm':error,'axis_error_deg':angle})
        refs.append(ref)
    refs=np.asarray(refs)
    coverage=bool(np.linalg.matrix_rank(refs-refs.mean(axis=0),tol=.001)>=2 and np.ptp(refs[:,2])>=.020)
    passed=coverage and all(r['position_error_mm']<=a.position_tolerance_mm and r['axis_error_deg']<=a.axis_tolerance_deg for r in rows)
    result={'passed_offline_check':passed,'samples':rows,'coverage_two_directions_and_20mm_height':coverage,
            'position_tolerance_mm':a.position_tolerance_mm,'axis_tolerance_deg':a.axis_tolerance_deg,
            'maximum_position_error_mm':max(r['position_error_mm'] for r in rows),
            'mean_position_error_mm':float(np.mean([r['position_error_mm'] for r in rows])),
            'maximum_axis_error_deg':max(r['axis_error_deg'] for r in rows),
            'motor_writes':0,'automatic_enable':False,
            'limitation':'Does not verify collision clearance, gripping, stop latency, camera extrinsics or real task success'}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
