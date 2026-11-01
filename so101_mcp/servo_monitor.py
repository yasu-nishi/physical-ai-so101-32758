"""Read-only servo feedback for the existing single bus owner.

Register values are raw device reports, not contact/force measurements.
No writes or control-limit changes are performed. A hardware error flag
raises immediately. A later normal voltage reading cannot establish that
an earlier under-voltage report was harmless during a physical movement.
"""
import json
from pathlib import Path
import time

from .joints import ARM
from .servo_diagnostics import CheckedReads, timestamped


class ServoMonitor:
    PERIOD_S = .25
    REGISTERS = ('Present_Position', 'Goal_Position', 'Goal_Position_2',
                 'Present_Velocity', 'Present_Load', 'Present_Current',
                 'Present_Voltage', 'Present_Temperature', 'Status', 'Torque_Enable')

    def __init__(self, bus, run_dir, *, reader=None, clock=time.monotonic):
        self.bus, self.clock = bus, clock
        self.path = Path(run_dir)/'servo_feedback.local.jsonl'
        self.audit = CheckedReads(bus, self._record) if reader is None else None
        self._raw_read = self.audit.read if self.audit else reader
        try:
            self.limits = {key: self.read(key) for key in
                           ('Min_Voltage_Limit', 'Max_Voltage_Limit', 'Protection_Current')}
        except Exception:
            self.close()
            raise
        self.last_time = -float('inf')
        self.last = None

    def read(self, *args, **kwargs):
        return self._raw_read(*args, **kwargs)

    def _record(self, row):
        with self.path.open('a') as stream:
            stream.write(json.dumps(timestamped(row))+'\n')

    def sample(self, *, require_torque=False, force=False):
        if not force and self.clock()-self.last_time < self.PERIOD_S:
            return self.last
        values = {key: self.read(key) for key in self.REGISTERS}
        issues = []
        names = (*ARM, 'gripper')
        for key, row in {**self.limits, **values}.items():
            if set(row) != set(names) or any(isinstance(v, bool) or not isinstance(v, int) for v in row.values()):
                raise ValueError(f'Incomplete or invalid servo feedback: {key}')
        for name in names:
            if values['Status'][name] != 0:
                issues.append(f'{name}: hardware status {values["Status"][name]}')
            if require_torque and values['Torque_Enable'][name] != 1:
                issues.append(f'{name}: torque disabled')
            if not self.limits['Min_Voltage_Limit'][name]+1 < values['Present_Voltage'][name] < self.limits['Max_Voltage_Limit'][name]-1:
                issues.append(f'{name}: voltage outside register guard')
            if values['Present_Temperature'][name] > 55:
                issues.append(f'{name}: temperature above 55 C')
            if abs(values['Present_Current'][name]) > self.limits['Protection_Current'][name]/2 or abs(values['Present_Load'][name]) > 500:
                issues.append(f'{name}: current/load exceeds commissioning guard')
        self.last_time = self.clock()
        self.last = {'available': True, 'monotonic_s': self.last_time, 'values_raw': values,
                     'issues': issues, 'source': 'servo registers; not external force/voltage measurements'}
        self._record({'event': 'servo_sample', **self.last})
        if require_torque and issues:
            raise RuntimeError('; '.join(issues))
        return self.last

    def close(self):
        if self.audit:
            self.audit.restore()
