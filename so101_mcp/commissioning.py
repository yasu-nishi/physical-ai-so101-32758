"""Explicit local joint commissioning; never asserts task calibration verified.

An initially folded joint may be outside task limits. Such a joint may only
move toward the interval; other joints retain their measured starting goals.
No Cartesian coordinates, saved poses, gripper commands or calibration writes.
"""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from .joints import ARM

TICKS_PER_DEGREE = 4095 / 360
# ST documentation: steps/s units still have 50 steps/s effective resolution.
# Lower values are not a valid way to command slower motion. A step is one
# goal of at most MAX_STEP_DEG that the servo profiles at this speed.
SERVO_SPEED = 50
NAMES = (*ARM, 'gripper')
# Operator asked for visibly larger motion (2026-11-01 evening); paths were
# checked in the MJCF for table clearance before raising these.
SESSION_BUDGET_DEG = {'elbow_flex': 45., 'wrist_flex': 45., 'shoulder_lift': 30., 'shoulder_pan': 20.}
MAX_STEP_DEG = 5.
MAX_SESSION_STEPS = 60
MAX_SESSION_TRAVEL_DEG = 150.
# Gripper commissioning: open fraction 0..1 over the stored calibration range.
GRIPPER_SPEED = 400
GRIPPER_TIMEOUT_S = 8.
GRIPPER_SETTLE_TICKS = 20
# The supply intermittently trips the servo under-voltage bit (error flag 1) on
# otherwise valid replies while Present_Voltage reads normal. Only that bit is
# retried, a bounded number of times; every other flag still stops at once,
# and the measured-voltage guard in health() is unchanged.
VOLTAGE_FLAG_MESSAGE = 'Servo error flags in response: 1'
VOLTAGE_FLAG_RETRIES = 2
VOLTAGE_FLAG_SESSION_LIMIT = 60
# One servo-profiled goal per step. The 2-tick interpolation with an 8-tick
# tracking abort stopped before the folded elbow built enough position error
# to break away (run 20261101_03: goal 3152, measured 3160, load 40).
# The servos run P-only (I=0, P=16); a folded elbow then stalls short of the
# goal under gravity (run 20261101_04: stalled 14 ticks short, load 68). The
# host adds lead beyond the target 1 tick per 50 ms, capped, and caps it again
# to STEP_SETTLE_TICKS once the joint is inside the settle band.
# Run 12: stick-slip, breakaway near load 300 then a 7-tick overshoot.
STEP_ENVELOPE_TICKS = 12
STEP_SETTLE_TICKS = 6
# Run 07: 43 ticks of error gave only load 184 (18%). Motion stays capped at
# SERVO_SPEED and the envelope stop, so a larger lead does not raise velocity.
STEP_MAX_LEAD_TICKS = 90
STEP_TIMEOUT_S = 12.
MAX_HANDOVERS = 10
# Operator-approved 2026-11-01 after run 05 stopped at raw 42 while lifting:
# margin above Min_Voltage_Limit lowered from 3 to 1 (0.1 V units).
VOLTAGE_GUARD_MARGIN = 1
# P-only servos sag a few ticks under gravity while holding (run 05: lift goal
# 848, measured 851). Hold checks use the same bound as the tracking guard.
HOLD_TOLERANCE_TICKS = 8
# While holding, a joint that sags HOLD_COMP_DEADBAND ticks or more gets its
# goal nudged 1 tick per HOLD_COMP_PERIOD_S toward undoing the sag, never more
# than HOLD_COMP_MAX_TICKS from the commanded position (run 09: elbow sagged
# 7 ticks after a completed step).
HOLD_COMP_DEADBAND = 3
HOLD_COMP_MAX_TICKS = 15
HOLD_COMP_PERIOD_S = .05  # gravity sag after a goal reset takes ~1 s
# A hold goal may differ from the sagging pose by tracking bound plus the
# compensation cap; the pose itself must still match the record within 8.
HOLD_GOAL_TOLERANCE_TICKS = HOLD_TOLERANCE_TICKS+HOLD_COMP_MAX_TICKS


def distance_outside(value, limits):
    low, high = limits
    return max(low-value, value-high, 0.)


def validate_step(current, initial, joint, delta, limits):
    if joint not in SESSION_BUDGET_DEG:
        raise ValueError('Only shoulder pan/lift, elbow and wrist pitch commissioning is allowed')
    if isinstance(delta, bool) or not isinstance(delta, (int, float)) or not math.isfinite(delta):
        raise ValueError('A finite numeric degree increment is required')
    if not .2 <= abs(delta) <= MAX_STEP_DEG:
        raise ValueError(f'Each step must be between 0.2 and {MAX_STEP_DEG:g} degrees')
    target = current[joint]+delta
    before = distance_outside(current[joint], limits[joint])
    after = distance_outside(target, limits[joint])
    if (before == 0 and after > 0) or (before > 0 and after >= before):
        raise ValueError('A step must stay in task limits or strictly reduce an existing violation')
    if abs(target-initial[joint]) > SESSION_BUDGET_DEG[joint]+1e-6:
        raise ValueError('Commissioning session joint excursion exhausted')
    return target


class JointCommissioner:
    """One owner of the LeRobot bus. All writes are logged and locally bounded."""
    def __init__(self, bus, mapping, limits, run_dir, camera_fresh, sleep=time.sleep, adopt_stopped=None, read_feedback=None):
        self.bus, self.mapping, self.limits = bus, mapping, limits
        self.run_dir = Path(run_dir)
        self.camera_fresh, self.sleep = camera_fresh, sleep
        self.stop_path = self.run_dir/'STOP'
        self.phase = 'read_only'
        self.armed = False
        self.latched = False
        self.goal_writes = 0
        self.register_writes = 0
        self.register_write_attempts = 0
        self.last_result = None
        self.expected = None
        self.hold_goal = {}
        self.gripper_enabled = bool(adopt_stopped and adopt_stopped.get('gripper_enabled'))
        self._voltage_dips = set()
        self._last_compensation = 0.
        self.initial = None
        self.last_raw = None
        self.steps = 0
        self.total_travel_deg = 0.
        self.health_snapshot = None
        self.torque_writes = 0
        self.energized = set()
        self._last_health = 0.
        self._last_status = 0.
        self.adopt_stopped = adopt_stopped
        self.read_feedback = read_feedback
        self.voltage_flags_tolerated = 0
        self.moving = None

    def event(self, name, **values):
        row = {'time_utc': datetime.now(timezone.utc).isoformat(), 'event': name, **values}
        with (self.run_dir/'events.jsonl').open('a') as f:
            f.write(json.dumps(row)+'\n')

    def read(self, register):
        for attempt in range(VOLTAGE_FLAG_RETRIES+1):
            try:
                values = (self.read_feedback(register) if self.read_feedback is not None else
                          self.bus.sync_read(register, normalize=False))
                break
            except ValueError as error:
                if (str(error) != VOLTAGE_FLAG_MESSAGE or attempt == VOLTAGE_FLAG_RETRIES or
                        self.voltage_flags_tolerated >= VOLTAGE_FLAG_SESSION_LIMIT):
                    raise
                self.voltage_flags_tolerated += 1
                self.event('voltage_flag_tolerated', register=register, attempt=attempt+1,
                           session_count=self.voltage_flags_tolerated)
                self.sleep(.02)
        if set(values) != set(NAMES) or not all(
                not isinstance(v, bool) and isinstance(v, (int, float)) and
                math.isfinite(v) and v == int(v) for v in values.values()):
            raise ValueError(f'Missing or invalid {register} feedback')
        return values

    def raw(self):
        values = self.read('Present_Position')
        values = {n:int(v) for n,v in values.items()}
        for n,v in values.items():
            c = self.bus.calibration[n]
            if not c.range_min <= v <= c.range_max:
                raise ValueError(f'{n} outside stored physical encoder calibration range')
        self.last_raw = values
        return values

    def common(self, raw):
        motor = {}
        for n in ARM:
            c = self.bus.calibration[n]
            motor[n+'.pos'] = (raw[n]-(c.range_min+c.range_max)/2)/TICKS_PER_DEGREE
        c = self.bus.calibration['gripper']
        motor['gripper.pos'] = (raw['gripper']-c.range_min)*100/(c.range_max-c.range_min)
        return self.mapping.from_lerobot(motor).joints_deg

    def _write(self, register, motor, value, stopping=False):
        if not stopping and (self.latched or self.stop_path.exists()):
            raise ValueError('Local STOP requested before register write')
        if (motor not in ARM and not (motor == 'gripper' and self.armed)) or \
                register not in ('Goal_Position', 'Goal_Velocity', 'Torque_Enable'):
            raise ValueError('Register write outside commissioning scope')
        self.register_write_attempts += 1
        self.event('write_attempt', register=register, motor=motor, value=int(value))
        self.bus.write(register, motor, int(value), normalize=False)
        self.register_writes += 1
        if register == 'Goal_Position':
            self.goal_writes += 1
        if register == 'Torque_Enable':
            self.torque_writes += 1
        self.event('write_ack', register=register, motor=motor, value=int(value))

    def status(self, force=False):
        if not force and time.monotonic()-self._last_status < .2:
            return
        data = {'mode':'local_joint_commissioning', 'phase':self.phase, 'armed':self.armed,
                'stop_latched':self.latched, 'goal_writes':self.goal_writes,
                'register_writes':self.register_writes, 'completed_steps':self.steps,
                'register_write_attempts':self.register_write_attempts,
                'torque_writes':self.torque_writes, 'total_travel_deg':self.total_travel_deg,
                'voltage_flags_tolerated':self.voltage_flags_tolerated,
                'gripper_enabled':self.gripper_enabled,
                'health':self.health_snapshot,
                'adoption_count':self.adopt_stopped.get('adoption_count',0)+1 if self.adopt_stopped else 0,
                'raw_positions':self.last_raw,
                'session_initial_joints_deg':self.initial,
                'joints_deg':self.common(self.last_raw) if self.last_raw else None,
                'last_result':self.last_result, 'task_calibration_verified':False,
                'time_utc':datetime.now(timezone.utc).isoformat()}
        path = self.run_dir/'status.tmp'
        path.write_text(json.dumps(data,indent=2)+'\n')
        path.replace(self.run_dir/'status.json')
        self._last_status = time.monotonic()

    def preflight(self):
        if self.stop_path.exists():
            raise ValueError('Local STOP already requested')
        if not self.bus.is_calibrated or set(self.bus.calibration) != set(NAMES):
            raise ValueError('Existing six-motor calibration must match hardware')
        torque = self.read('Torque_Enable')
        if self.adopt_stopped:
            previous = self.adopt_stopped
            # Chained handovers keep the first run's excursion envelope and budgets.
            # An unverified hold is accepted only if the live checks below pass.
            if previous.get('phase') not in ('stopped_holding', 'stopped_hold_unverified') or \
                    previous.get('stop_latched') is not True or \
                    previous.get('adoption_count',0) >= MAX_HANDOVERS or 'session_initial_joints_deg' not in previous:
                raise ValueError('Handover requires a verified latched hold within the handover limit')
            if any(torque[n] != 1 for n in ARM) or torque['gripper'] != int(self.gripper_enabled):
                raise ValueError('Stopped hold torque state changed')
        elif any(torque.values()):
            raise ValueError('Refuse takeover of an already energized arm')
        if any(self.read('Operating_Mode').values()):
            raise ValueError('Position mode must already be configured; no mode writes')
        duration = self.read('Goal_Time')
        velocity_units = self.read('Velocity_Unit_factor')
        if any(duration[n] != 0 for n in ARM):
            raise ValueError('Time-based trajectory register must already be zero')
        if any(velocity_units[n] != 50 for n in ARM):
            raise ValueError('Unrecognized servo velocity units')
        phase = self.read('Phase')
        if any(phase[n] != 12 for n in ARM):
            raise ValueError('Commissioning requires the inspected ST servo phase configuration')
        if getattr(self.bus.calibration['gripper'], 'drive_mode', 0):
            raise ValueError('Inverted gripper calibration unsupported in commissioning')
        self.initial_raw = self.raw()
        self.initial = self.common(self.initial_raw)
        if self.adopt_stopped:
            previous = self.adopt_stopped
            goals = self.read('Goal_Position')
            if any(abs(self.initial_raw[n]-previous['raw_positions'][n]) > HOLD_GOAL_TOLERANCE_TICKS or
                   abs(self.initial_raw[n]-goals[n]) > HOLD_GOAL_TOLERANCE_TICKS for n in ARM):
                raise ValueError('Stopped hold no longer matches the recorded pose and goals')
            self.steps = previous['completed_steps']
            self.total_travel_deg = previous['total_travel_deg']
            # Preserve the original excursion envelope across the explicit handover.
            self.initial = previous['session_initial_joints_deg']
        if any(distance_outside(self.initial[n],self.limits[n]) > 5 for n in ARM):
            raise ValueError('Folded pose exceeds this commissioning start envelope')
        self.min_voltage = self.read('Min_Voltage_Limit')
        self.max_voltage = self.read('Max_Voltage_Limit')
        self.protection_current = self.read('Protection_Current')
        self.health()
        self.event('preflight_passed', initial_raw=self.initial_raw, joints_deg=self.initial,
                   task_limits=self.limits, task_calibration_verified=False)
        self.status(force=True)

    def health(self):
        if not self.camera_fresh():
            raise ValueError('Workspace camera is not fresh')
        voltage = self.read('Present_Voltage')
        temperature = self.read('Present_Temperature')
        current = self.read('Present_Current')
        load = self.read('Present_Load')
        servo_status = self.read('Status')
        self.health_snapshot = {'voltage_raw':voltage, 'temperature_c':temperature,
                                'current_raw':current, 'load_raw':load, 'servo_status':servo_status}
        if any(servo_status.values()):
            raise ValueError('Servo reports a hardware error status')
        if self.armed:
            torque = self.read('Torque_Enable')
            mode = self.read('Operating_Mode')
            speed = self.read('Goal_Velocity')
            duration = self.read('Goal_Time')
            self.health_snapshot['control_registers'] = {'torque':torque,'mode':mode,'speed':speed,'time':duration}
            if torque['gripper'] != int(self.gripper_enabled) or any(torque[n] != 1 or mode[n] != 0 or
                    speed[n] != SERVO_SPEED or duration[n] != 0 for n in ARM) or \
                    (self.gripper_enabled and (speed['gripper'] != GRIPPER_SPEED or mode['gripper'] != 0)):
                raise ValueError('Servo torque, position mode or velocity guard changed')
        low = [n for n in NAMES if not self.min_voltage[n]+VOLTAGE_GUARD_MARGIN < voltage[n]]
        # Run 11: wrist_roll alone read 41 for one sample while every other
        # servo read 48-49 at zero current. A single low sample on one servo
        # must repeat on the next health read; several servos low or a repeat
        # stops.
        # Run 13: the unpowered gripper alone read 40 (others 48) once.
        isolated = self.armed and len(low) == 1 and low[0] not in self._voltage_dips
        self._voltage_dips = set(low)
        if isolated:
            self.event('voltage_dip_pending', motor=low[0], voltage_raw=dict(voltage))
            low = []
        for n in NAMES:
            if n in low or not voltage[n] < self.max_voltage[n]-1:
                raise ValueError(f'{n}: supply outside guarded register limits: raw={voltage[n]}, '
                                 f'required {self.min_voltage[n]+VOLTAGE_GUARD_MARGIN} < value < {self.max_voltage[n]-1}')
            if temperature[n] > 55:
                raise ValueError(f'{n}: temperature above commissioning limit')
            if abs(current[n]) > self.protection_current[n]/2 or abs(load[n]) > 500:
                raise ValueError(f'{n}: current/load above commissioning limit')
        self._last_health = time.monotonic()

    def arm(self):
        self.preflight()
        # Re-read immediately before seeding. Goal 0 is NEVER used to energize.
        raw = self.raw()
        if any(abs(raw[n]-self.initial_raw[n]) > 2 for n in NAMES):
            raise ValueError('Arm moved during preflight')
        self.expected = dict(raw)
        seed = dict(raw)
        if self.adopt_stopped:
            # Keep the energized hold goal (it may lead the sagging pose);
            # re-seeding at the measured pose drops that lead and the joint sags
            # (run 14: elbow sagged over 8 ticks during startup).
            held = self.read('Goal_Position')
            for n in ARM:
                if abs(held[n]-raw[n]) <= HOLD_GOAL_TOLERANCE_TICKS:
                    seed[n] = held[n]
        self.hold_goal = {n:seed[n] for n in ARM}
        if self.adopt_stopped:
            self.armed = True
            self.energized = set(ARM)
        for n in ARM:
            if self.stop_path.exists():
                raise ValueError('Stopped before torque enable')
            self._write('Goal_Velocity', n, SERVO_SPEED)
            self._write('Goal_Position', n, seed[n])
        goals = self.read('Goal_Position')
        speeds = self.read('Goal_Velocity')
        if any(goals[n] != seed[n] or speeds[n] != SERVO_SPEED for n in ARM):
            raise ValueError('Seed goals / speed readback mismatch; torque remains off')
        if self.stop_path.exists():
            raise ValueError('Stopped before torque enable')
        raw_now = self.raw()
        moved_limit = HOLD_TOLERANCE_TICKS if self.adopt_stopped else 2
        if any(abs(raw_now[n]-raw[n]) > moved_limit for n in ARM):
            raise ValueError('Arm moved after goal seeding')
        self.armed = True  # Include partial-enable errors in the hold recovery path.
        for n in (() if self.adopt_stopped else ARM):
            if self.stop_path.exists():
                raise ValueError('Stopped during torque enable')
            self.energized.add(n)  # A lost acknowledgement may still have enabled torque.
            self._write('Torque_Enable', n, 1)
        torque = self.read('Torque_Enable')
        if any(torque[n] != 1 for n in ARM) or torque['gripper'] != int(self.gripper_enabled):
            raise ValueError('Unexpected torque state')
        self.phase = 'holding'
        for _ in range(10):
            self.tick()
            self.sleep(.05)
        self.event('hold_enabled', gripper_torque_enabled=self.gripper_enabled)
        self.status(force=True)

    def monitor_stopped(self):
        """Keep refreshing the local display after a latch, without any writes."""
        try:
            self.raw()
            self.health()
            if self.armed:self.verify_hold()
            self.last_result.pop('monitor_error',None)
            self.phase = 'stopped_holding' if self.armed else 'stopped_unarmed'
        except Exception as error:
            self.last_result['monitor_error']=type(error).__name__+': '+str(error)
            self.phase = 'stopped_hold_unverified' if self.armed else 'stopped_unarmed'
        self.status(force=True)

    def verify_hold(self):
        torque=self.read('Torque_Enable')
        goals=self.read('Goal_Position')
        if torque['gripper']!=int(self.gripper_enabled) or any(torque[n]!=1 for n in ARM):
            raise ValueError('Stopped hold torque could not be verified')
        if self.last_raw is None or any(abs(goals[n]-self.last_raw[n])>HOLD_GOAL_TOLERANCE_TICKS for n in ARM):
            raise ValueError('Stopped hold goals do not match measured position')

    def tick(self):
        if self.stop_path.exists():
            raise ValueError('Local STOP requested')
        actual = self.raw()
        for n in (ARM if self.armed else ()):
            if self.moving and self.moving[0] == n:
                low, high = self.moving[1]
                if not low <= actual[n] <= high:
                    raise ValueError(f'{n} left the commanded step envelope: raw={actual[n]}')
            elif abs(actual[n]-self.expected[n]) > 8:
                raise ValueError('Position tracking error exceeds 8 encoder ticks (~0.7 deg)')
        if time.monotonic()-self._last_health > .25:
            self.health()
        if self.phase == 'holding' and not self.moving and not self.latched:
            self.compensate_hold(actual)
        self.status()
        return actual

    def compensate_hold(self, actual):
        if time.monotonic()-self._last_compensation < HOLD_COMP_PERIOD_S:
            return
        self._last_compensation = time.monotonic()
        for n in ARM:
            goal = self.hold_goal.get(n, self.expected[n])
            sag = self.expected[n]-actual[n]
            if abs(sag) >= HOLD_COMP_DEADBAND:
                new = goal+(1 if sag > 0 else -1)
            else:
                continue
            c = self.bus.calibration[n]
            new = min(max(new, self.expected[n]-HOLD_COMP_MAX_TICKS, c.range_min),
                      self.expected[n]+HOLD_COMP_MAX_TICKS, c.range_max)
            if new != goal:
                self._write('Goal_Position', n, new)
                self.hold_goal[n] = new

    def step(self, command):
        if self.latched or not self.armed:
            raise ValueError('Commissioning is stopped or not armed')
        if isinstance(command, dict) and set(command) == {'gripper_open'}:
            return self.gripper(command)
        if not isinstance(command, dict) or set(command) != {'joint', 'delta_deg'}:
            raise ValueError('Only joint and delta_deg may be submitted')
        actual = self.tick()
        current = self.common(actual)
        n, delta = command['joint'], command['delta_deg']
        target_deg = validate_step(current, self.initial, n, delta, self.limits)
        if self.steps >= MAX_SESSION_STEPS or self.total_travel_deg+abs(delta) > MAX_SESSION_TRAVEL_DEG:
            raise ValueError('Initial-pose commissioning session budget exhausted')
        target_raw = actual[n]+round(delta*self.mapping.sign[n]*TICKS_PER_DEGREE)
        c = self.bus.calibration[n]
        if not c.range_min <= target_raw <= c.range_max:
            raise ValueError('Target exceeds physical calibration range')
        actual_target = current[n]+(target_raw-actual[n])/TICKS_PER_DEGREE/self.mapping.sign[n]
        before = distance_outside(current[n],self.limits[n])
        after = distance_outside(actual_target,self.limits[n])
        if (before == 0 and after > 0) or (before > 0 and after >= before) or \
                abs(actual_target-self.initial[n]) > SESSION_BUDGET_DEG[n]:
            raise ValueError('Rounded encoder target exceeds commissioning envelope')
        start = actual[n]
        # Held joints keep the last command, rather than accumulating measured sag.
        self.phase = 'moving'
        self.event('step_started', command=command, start_raw=start, target_raw=target_raw,
                   nominal_target_deg=target_deg, profile='single_servo_goal', speed_steps_s=SERVO_SPEED)
        self.moving = (n, (min(start,target_raw)-STEP_ENVELOPE_TICKS, max(start,target_raw)+STEP_ENVELOPE_TICKS))
        self._write('Goal_Position', n, target_raw)
        self.hold_goal[n] = target_raw
        self.expected[n] = target_raw
        goal = target_raw
        direction = 1 if target_raw >= start else -1
        began = time.monotonic()
        settled = 0
        previous = start
        while settled < 5:
            if time.monotonic()-began > STEP_TIMEOUT_S:
                raise ValueError(f'Step failed to settle within {STEP_SETTLE_TICKS} encoder ticks '
                                 f'in {STEP_TIMEOUT_S:.0f} s: raw={actual[n]}, target={target_raw}, goal={goal}')
            actual = self.tick()
            remaining = (target_raw-actual[n])*direction
            lead = (goal-target_raw)*direction
            progressing = (actual[n]-previous)*direction >= 1
            previous = actual[n]
            if remaining < -STEP_SETTLE_TICKS:
                new_goal = target_raw
            elif remaining <= STEP_SETTLE_TICKS:
                new_goal = target_raw+direction*min(lead, STEP_SETTLE_TICKS)
            elif progressing:
                # Moving: stop adding lead and shrink it with the remaining
                # distance so stored lead cannot carry the joint past the target
                # (run 08 overshot by 10 ticks from a 64-tick lead).
                new_goal = target_raw+direction*max(0, min(lead, remaining))
            else:
                new_goal = target_raw+direction*min(lead+1, STEP_MAX_LEAD_TICKS)
                new_goal = min(max(new_goal, c.range_min), c.range_max)
            settled = settled+1 if abs(remaining) <= STEP_SETTLE_TICKS else 0
            if new_goal != goal:
                self._write('Goal_Position', n, new_goal)
                self.hold_goal[n] = goal = new_goal
            self.sleep(.05)
        self.moving = None
        self.hold_goal[n] = goal
        self.steps += 1
        self.total_travel_deg += abs(delta)
        self.phase = 'holding'
        self.last_result = {'status':'completed','command':command,
                            'start_raw':start,'end_raw':actual[n],'target_raw':target_raw,
                            'hold_goal_raw':goal,
                            'measured_delta_deg':(actual[n]-start)/TICKS_PER_DEGREE/self.mapping.sign[n]}
        self.event('step_completed', **self.last_result)
        self.status(force=True)

    def gripper(self, command):
        fraction = command['gripper_open']
        if isinstance(fraction, bool) or not isinstance(fraction, (int, float)) or not 0 <= fraction <= 1:
            raise ValueError('gripper_open must be a number from 0 to 1')
        c = self.bus.calibration['gripper']
        target = round(c.range_min+fraction*(c.range_max-c.range_min))
        actual = self.tick()
        if not self.gripper_enabled:
            # Seed at the measured opening before torque; never energize at goal 0.
            self._write('Goal_Velocity', 'gripper', GRIPPER_SPEED)
            self._write('Goal_Position', 'gripper', actual['gripper'])
            goals = self.read('Goal_Position')
            if goals['gripper'] != actual['gripper']:
                raise ValueError('Gripper seed goal readback mismatch')
            self.gripper_enabled = True  # A lost acknowledgement may still enable it.
            self._write('Torque_Enable', 'gripper', 1)
            self.event('gripper_enabled', raw=actual['gripper'])
        start = actual['gripper']
        self.phase = 'moving'
        self.event('gripper_started', command=command, start_raw=start, target_raw=target)
        self._write('Goal_Position', 'gripper', target)
        began = time.monotonic();last=start;still=0
        while True:
            actual = self.tick()
            if abs(actual['gripper']-target) <= GRIPPER_SETTLE_TICKS:
                status = 'reached'
                break
            still = still+1 if abs(actual['gripper']-last) <= 2 else 0
            last = actual['gripper']
            if still >= 20:
                status = 'stalled'  # e.g. jaws closed on an object or a hard end
                break
            if time.monotonic()-began > GRIPPER_TIMEOUT_S:
                raise ValueError('Gripper did not settle in time')
            self.sleep(.05)
        self.phase = 'holding'
        self.last_result = {'status':'completed','command':command,'gripper_result':status,
                            'start_raw':start,'end_raw':actual['gripper'],'target_raw':target}
        self.event('gripper_completed', **self.last_result)
        self.status(force=True)

    def halt(self, reason):
        was_moving = self.moving is not None or self.phase == 'moving'
        self.latched = True
        self.moving = None
        self.phase = 'stopped_hold_unverified' if self.armed else 'stopped_unarmed'
        self.last_result = {'status':'stopped','reason':reason,'health_at_stop':self.health_snapshot}
        if self.armed:
            try:
                raw = self.raw()
                # A stop during motion freezes at the measured pose. A stop while
                # holding keeps a hold goal that leads gravity sag (no write).
                goals = {n:(self.hold_goal[n] if not was_moving and n in self.hold_goal and
                            abs(self.hold_goal[n]-raw[n]) <= HOLD_GOAL_TOLERANCE_TICKS else raw[n])
                         for n in ARM}
                for n in ARM:
                    if n in self.energized and goals[n] != self.hold_goal.get(n):
                        self._write('Goal_Position', n, goals[n], stopping=True)
                self.expected = {**raw, **{n:(self.expected or raw)[n] for n in ARM if goals[n] != raw[n]}}
                self.hold_goal = goals
                self.verify_hold()
                self.phase = 'stopped_holding'
            except Exception as error:
                self.last_result['hold_error'] = type(error).__name__+': '+str(error)
        if self.armed and self.gripper_enabled:
            try:
                raw_g = self.raw()['gripper']
                self._write('Goal_Position', 'gripper', raw_g, stopping=True)
            except Exception as error:
                self.last_result['gripper_hold_error'] = type(error).__name__+': '+str(error)
        self.event('stop_latched', **self.last_result)
        self.status(force=True)
