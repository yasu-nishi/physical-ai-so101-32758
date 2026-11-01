"""LeRobot 0.5.1 adapter; observe/dry-run opens the bus without configure()."""
import io
import json
import math
import os
import time
from pathlib import Path

from PIL import Image
from dotenv import dotenv_values

from .config import ROOT, real_camera_dimensions
from .joints import ARM, JointMap, Pose, validate_limits


class RealBackend:
    # The STS3215's normalized degree readings have one 360/4095 degree tick.
    # This tolerance only admits quantized readback of a bounded held target.
    ENCODER_TICK_DEG = 360/4095
    MAX_HOLD_OFFSET_DEG = 2.
    def __init__(self, config, run_dir, enable_motion=False):
        self.config, self.run_dir = config, Path(run_dir)
        self.enabled = enable_motion
        self.mode = 'motion_enabled' if enable_motion else 'dry_run'
        self.robot = None
        self.mapping = JointMap(config['mapping'])
        self.writes = 0
        self.last_command = None
        self._last_goal = None
        self._hold_offset_deg = dict.fromkeys(ARM, 0.)
        self._moving = False
        self._gripper_moving = False
        self.rgb_calibration = None
        self.connection_stage = 'configuration'
        self.feedback_gain={n:1. for n in ARM}
        gains=config.get('servo_feedback',{}).get('joint_error_gain',{})
        if not isinstance(gains,dict) or not set(gains)<=set(ARM):
            raise ValueError('Servo feedback gains require named arm joints')
        if any(isinstance(g,bool) or not isinstance(g,(int,float)) or not math.isfinite(g) or not 1<=g<=4 for g in gains.values()):
            raise ValueError('Servo feedback gain must be finite and between 1 and 4')
        self.feedback_gain.update(gains)
        self.verify_servo_io = config.get('servo_feedback', {}).get('verify_goal_readback', False)
        self.suppress_repeated_goals = config.get('servo_feedback', {}).get('suppress_repeated_goals', False)
        self._sent_motor_goals = None
        self._servo_monitor = None

    def _validate_motion_setup(self):
        if not self.enabled:
            return
        c = self.config
        if not (c['mapping_verified'] and c['workspace_verified'] and c['camera_role'] != 'unverified'):
            raise ValueError('Verify real mapping, workspace and camera locally before enabling motion')
        steps = c.get('incremental_steps', {})
        verified_steps = steps.get('enabled') is True and steps.get('real_motion_verified') is True
        cart = c.get('cartesian_steps', {})
        verified_cart = cart.get('enabled') is True and cart.get('real_kinematics_verified') is True
        if verified_cart and c['camera_role'] not in ('fixed_workspace', 'overhead', 'front'):
            raise ValueError('Cartesian operation requires a fixed workspace camera; moving wrist-camera hand-eye calibration is not implemented')
        if verified_cart and self.rgb_calibration is None:
            raise ValueError('Calibrated real RGB is required for verified Cartesian operation')
        if not (ROOT / c['taught_skills_file']).is_file() and not (verified_steps or verified_cart):
            raise ValueError('Teach real waypoints or locally verify incremental/Cartesian motion before enabling motion')

    def connect(self, *, cameras=True):
        if not cameras and self.enabled:
            raise ValueError('Joints-only inspection cannot enable real motion')
        from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
        from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
        from lerobot.robots.so_follower.so_follower import SO101Follower
        env = {**dotenv_values(ROOT / '.env'), **os.environ}
        c = self.config
        path = ROOT/c.get('rgb_calibration_file', 'config/physical_ai/camera_calibration.local.json')
        if cameras and path.exists():
            from .rgb_calibration import load_verified_calibration
            self.rgb_calibration = load_verified_calibration(path)
        port, rid = env.get(c['port_env']), env.get(c['robot_id_env'])
        if not port or not rid or not Path(port).exists():
            raise ConnectionError('Connect the configured SO-101 follower USB serial port')
        self._validate_motion_setup()
        index = env.get(c['camera_index_env'])
        if cameras and index is None:
            raise ValueError('Set the verified robot camera index')
        warmup = c.get('camera_warmup_s', 3)
        if not isinstance(warmup, int) or isinstance(warmup, bool) or not 1 <= warmup <= 10:
            raise ValueError('Camera startup warmup must be 1–10 seconds')
        camera_configs = ({'workspace': OpenCVCameraConfig(index_or_path=int(index), warmup_s=warmup,
                                                         **real_camera_dimensions(c, env))}
                          if cameras else {})
        self.robot = SO101Follower(SOFollowerRobotConfig(
            port=port, id=rid, use_degrees=True, disable_torque_on_disconnect=False,
            max_relative_target={**{n: 2. for n in ARM}, 'gripper': 5.},
            cameras=camera_configs))
        if set(self.robot.calibration) != set((*ARM, 'gripper')):
            raise ValueError('Existing six-motor calibration is required; no automatic calibration')
        # SOFollower.connect() would configure motors and toggle torque. The
        # LeRobot bus/camera APIs below preserve motor state during inspection.
        self.connection_stage = 'motor_bus'
        self.robot.bus.connect()
        self.robot.bus.set_timeout(100)
        self.connection_stage = 'stored_calibration'
        if not self.robot.bus.is_calibrated:
            raise ValueError('Stored calibration does not match motor registers; check the existing robot ID before recalibrating. No calibration settings were written')
        if self.verify_servo_io:
            from .servo_monitor import ServoMonitor
            self._servo_monitor = ServoMonitor(self.robot.bus, self.run_dir)
        self.connection_stage = 'camera'
        for camera in self.robot.cameras.values():
            camera.connect()
        self.connection_stage = 'joint_limits'
        for name in ARM:
            cal = self.robot.calibration[name]
            half = (cal.range_max-cal.range_min)*180/4095
            for endpoint in c['limits_deg'][name]:
                motor = self.mapping.sign[name]*endpoint+self.mapping.offset[name]
                if not -half <= motor <= half:
                    raise ValueError('Configured limit exceeds measured calibration range')
        if self.enabled:
            if self._servo_monitor:
                self._servo_monitor.sample(require_torque=False, force=True)
            self.connection_stage = 'measured_pose'
            pose = self.state()
            validate_limits(pose, c['limits_deg'])
            bus = self.robot.bus
            self.connection_stage = 'torque_state'
            torque = bus.sync_read('Torque_Enable', normalize=False)
            arm_on = all(torque[n] == 1 for n in ARM)
            if not arm_on and any(torque[n] for n in ARM):
                raise ValueError('Partially energized arm; resolve locally before enabling motion')
            # Never call configure(): it disables torque (an unfolded arm drops
            # under gravity) and resets P to 16, at which the folded elbow
            # stalls (local commissioning 2026-11-01). Motor modes and EEPROM
            # An energized joint may need a small goal/position difference to
            # hold gravity. Re-seeding measured positions discards that hold
            # (observed during commissioning run 14). Adopt the existing goals.
            if arm_on:
                self.connection_stage = 'held_goals'
                held = self.goal_state()
                self._check_hold(pose, held)
                self._last_goal = held
                if torque['gripper'] != 1:
                    # Only seed the unpowered jaw. Never rewrite arm goals.
                    bus.write('Goal_Position', 'gripper', self.mapping.to_lerobot(pose)['gripper.pos'])
                    self.writes += 1
                    self._last_goal = Pose(held.joints_deg, pose.gripper_open)
            else:
                self.robot.send_action(self.mapping.to_lerobot(pose))
                self.writes += 1
                self._last_goal = pose
            runtime = c.get('servo_runtime', {})
            self.connection_stage = 'runtime_parameters'
            for name, value in runtime.get('goal_velocity', {}).items():
                bus.write('Goal_Velocity', name, int(value), normalize=False)
            for name, value in runtime.get('p_coefficient', {}).items():
                bus.write('P_Coefficient', name, int(value), normalize=False)
            if not arm_on:
                bus.enable_torque()
            elif torque['gripper'] != 1:
                bus.write('Torque_Enable', 'gripper', 1, normalize=False)
            self.last_command = pose
        self.connection_stage = 'connected'

    def goal_state(self):
        values = (self._servo_monitor.read('Goal_Position', normalize=True) if self._servo_monitor
                  else self.robot.bus.sync_read('Goal_Position'))
        return self.mapping.from_lerobot({f'{n}.pos': v for n, v in values.items()})

    def _check_hold(self, measured, held):
        validate_limits(held, self.config['limits_deg'])
        # Do not reinterpret a
        # distant unfinished command as a stationary load compensation.
        if max(abs(held.joints_deg[n]-measured.joints_deg[n]) for n in ARM) > self.MAX_HOLD_OFFSET_DEG+self.ENCODER_TICK_DEG+1e-8:
            raise ValueError('Existing arm goal differs by more than 2 degrees; verify the hold locally')

    def begin_motion(self, waypoints):
        """Adopt a bounded held-goal offset once, without writing or integrating it.

        Plans are still expressed in measured joint coordinates. A fixed offset
        translates that plan into servo goals, preserving the previously held
        load at the first tick. Re-observation is required between host steps.
        """
        if not self.enabled:
            return None
        start = waypoints[0][1]
        held = self.goal_state()
        self._check_hold(start, held)
        if self._servo_monitor:
            self._servo_monitor.sample(require_torque=True, force=True)
        self._sent_motor_goals = self.mapping.to_lerobot(held)
        offset = {n: held.joints_deg[n]-start.joints_deg[n] for n in ARM}
        # Reject the entire step before any writes if compensation would cross
        # a physical joint limit, including intermediate phase endpoints.
        for _, initial, target, _ in waypoints:
            for point in (initial, target):
                validate_limits(Pose({n: point.joints_deg[n]+offset[n] for n in ARM},
                                     point.gripper_open), self.config['limits_deg'])
        self._hold_offset_deg = offset
        self.last_command = start
        self._last_goal = held
        self._moving = True
        self._gripper_moving = any(abs(initial.gripper_open-target.gripper_open) > 1e-6
                                  for _, initial, target, _ in waypoints)
        return {'held_goal_offset_deg': offset, 'maximum_offset_deg': self.MAX_HOLD_OFFSET_DEG,
                'encoder_tolerance_deg':self.ENCODER_TICK_DEG,
                'nominal_relative_limit_deg':{n:self.robot.config.max_relative_target[n] for n in ARM},
                'maximum_servo_error_deg':{n:abs(offset[n])+self.robot.config.max_relative_target[n] for n in ARM},
                'method': 'fixed pre-step goal-minus-measurement; no adaptive compensation'}

    def end_motion(self):
        if self.enabled and any(g>1 for g in self.feedback_gain.values()):
            # Remove transient tracking effort once the measured pose is within
            # tolerance. Freeze it using only the original bounded load offset;
            # never adopt an unachieved feedback target as a new gravity offset.
            # Successful completion must retain the accepted jaw goal. Using
            # the lagging/contact-limited measured opening here releases a
            # grasp and disagrees with the next arm-only trajectory.
            self.halt(preserve_gripper=True)
        self._moving = False

    @property
    def held_gripper_open(self):
        """Accepted servo goal, distinct from measured opening under contact."""
        return self._last_goal.gripper_open if self._last_goal is not None else None

    def state(self):
        observation = (self._servo_monitor.read('Present_Position', normalize=True) if self._servo_monitor
                       else self.robot.bus.sync_read('Present_Position'))
        return self.mapping.from_lerobot({f'{n}.pos': v for n, v in observation.items()})

    def capture(self):
        if not self.robot or 'workspace' not in self.robot.cameras:
            raise RuntimeError('Camera is not connected in joints-only inspection')
        # The LeRobot capture thread owns the camera. A bounded fresh-frame
        # wait cannot hold the robot worker for read()'s default 10 seconds.
        camera = self.robot.cameras['workspace']
        camera.new_frame_event.clear()
        frame = camera.async_read(timeout_ms=200)
        output = io.BytesIO()
        Image.fromarray(frame).save(output, format='PNG')
        return output.getvalue()

    def capture_bundle(self):
        bundle = {'images': [self.capture()], 'image_roles': ['rgb'], 'depth_m': None,
                'sensors': {
                    'rgb': {'available': True, 'source': 'LeRobot OpenCV camera'},
                    'depth': {'available': False,
                              'reason': 'No calibrated RGB-D device configured in the real backend'},
                    'end_effector': {'available': False,
                        'reason': 'Real joint-to-tool kinematics and work-frame calibration are not verified'}}}
        if self.rgb_calibration is not None:
            from .rgb_calibration import rectify_png
            bundle['images'][0], bundle['camera_calibration'] = rectify_png(bundle['images'][0], self.rgb_calibration)
            bundle['sensors']['rgb']['rectified'] = True
        if self._servo_monitor:
            bundle['sensors']['servos'] = self._servo_monitor.sample(force=True)
        return bundle

    def plan(self, name, args):
        if name == 'observe_pose' and not args:
            key = name
        elif name == 'pick_place' and set(args) == {'slot', 'destination'} and args['slot'] in ('slot_a', 'slot_b') and args['destination'] == 'box':
            key = 'pick_place_'+args['slot']
        else:
            raise ValueError('Unknown real skill or arguments')
        path = ROOT / self.config['taught_skills_file']
        if not path.exists():
            raise ValueError('No real taught skills: capture waypoints locally first')
        skills = json.loads(path.read_text())
        if skills.get('backend') != 'real' or skills.get('units') != 'degrees_and_gripper_fraction':
            raise ValueError('Taught skills must explicitly declare real backend and units')
        points = skills['skills'].get(key)
        if not points:
            raise ValueError('Requested skill has not been taught on this real arm')
        expected = ['observe_pose'] if key == 'observe_pose' else [
            'approach', 'descend', 'close', 'lift', 'transfer', 'release', 'observe_pose']
        if [p['phase'] for p in points] != expected:
            raise ValueError('Real skill must contain all taught phases in the documented order')
        return [(p['phase'], Pose.from_dict(p['pose']), float(p['duration_s'])) for p in points]

    def command(self, pose):
        validate_limits(pose, self.config['limits_deg'])
        if not self.enabled:
            # Real measured state stays measured; a dry run does not fake motion.
            self.last_command = pose
            return None
        if self._servo_monitor:
            self._servo_monitor.sample(require_torque=True)
        if self.last_command:
            dt = 1/self.config['control_hz']
            if max(abs(pose.joints_deg[n]-self.last_command.joints_deg[n]) for n in ARM) > self.config['speed_deg_s']*dt+1e-5:
                raise ValueError('Local per-command speed limit exceeded')
            if abs(pose.gripper_open-self.last_command.gripper_open) > self.config['gripper_speed_s']*dt+1e-5:
                raise ValueError('Local gripper speed limit exceeded')
        goal = Pose({n: pose.joints_deg[n]+self._hold_offset_deg[n] for n in ARM}, pose.gripper_open)
        validate_limits(goal, self.config['limits_deg'])
        bounded, measured = self._bounded_goal(pose)
        if self._last_goal:
            dt = 1/self.config['control_hz']
            # Feedback can change faster than the nominal trajectory when a
            # joint breaks static friction. Slew-limit that correction locally.
            bounded=Pose({n:min(max(bounded.joints_deg[n],
                                self._last_goal.joints_deg[n]-self.config['speed_deg_s']*dt),
                                self._last_goal.joints_deg[n]+self.config['speed_deg_s']*dt)
                          if self.feedback_gain[n]>1 else bounded.joints_deg[n] for n in ARM},bounded.gripper_open)
            self._check_servo_bound(bounded,measured)
            if max(abs(bounded.joints_deg[n]-self._last_goal.joints_deg[n]) for n in ARM) > self.config['speed_deg_s']*dt+1e-5:
                raise ValueError('Local servo-goal speed limit exceeded')
            if abs(bounded.gripper_open-self._last_goal.gripper_open) > self.config['gripper_speed_s']*dt+1e-5:
                raise ValueError('Local servo gripper speed limit exceeded')
        self._write_goal(bounded)
        self._last_goal = bounded
        # Nominal trajectory rate and actual servo-goal rate are independent.
        self.last_command = pose
        with (self.run_dir/'servo_commands.local.jsonl').open('a') as stream:
            stream.write(json.dumps({'monotonic_s':time.monotonic(), 'nominal':pose.as_dict(), 'requested_goal':goal.as_dict(),
                                     'feedback_gain':self.feedback_gain,
                                     'sent_goal':bounded.as_dict(), 'measured':measured.as_dict()})+'\n')
        return self._last_goal

    def _bounded_goal(self, nominal):
        # Limit the NOMINAL measured-joint target before adding the fixed
        # held-goal offset. Clamping the compensated servo goal to +/-2 degrees
        # consumes all motion authority when gravity already needs that gap.
        # The servo gap may now reach |held offset| + 2 degrees (up to ~4.09).
        # It is bounded separately, as are physical limits and per-tick rate.
        # No gain, torque, integral term or automatic offset growth is added.
        from lerobot.robots.utils import ensure_safe_goal_position
        measured = self.state()
        correction=Pose({n:measured.joints_deg[n]+self.feedback_gain[n]*(nominal.joints_deg[n]-measured.joints_deg[n])
                         for n in ARM},nominal.gripper_open)
        desired = self.mapping.to_lerobot(correction)
        actual = self.mapping.to_lerobot(measured)
        bounded = ensure_safe_goal_position(
            {key.removesuffix('.pos'): (value, actual[key]) for key, value in desired.items()},
            self.robot.config.max_relative_target)
        limited = self.mapping.from_lerobot({f'{key}.pos': value for key,value in bounded.items()})
        pose = Pose({n:limited.joints_deg[n]+self._hold_offset_deg[n] for n in ARM},limited.gripper_open)
        self._check_servo_bound(pose,measured)
        return pose, measured

    def _check_servo_bound(self,pose,measured):
        for n in ARM:
            if abs(self._hold_offset_deg[n])>self.MAX_HOLD_OFFSET_DEG+self.ENCODER_TICK_DEG+1e-8:
                raise ValueError('Held-goal compensation exceeds its independent limit')
            maximum=abs(self._hold_offset_deg[n])+self.robot.config.max_relative_target[n]
            if abs(pose.joints_deg[n]-measured.joints_deg[n])>maximum+1e-8:
                raise ValueError('Compensated servo-goal tracking bound exceeded')
        validate_limits(pose, self.config['limits_deg'])

    def _write_goal(self, goal):
        values = self.mapping.to_lerobot(goal)
        changed = {key.removesuffix('.pos'): value for key, value in values.items()
                   if not self.suppress_repeated_goals or self._sent_motor_goals is None
                   or value != self._sent_motor_goals.get(key)}
        if changed:
            self.robot.bus.sync_write('Goal_Position', changed)
            self.writes += 1
        # Broadcast write success means bytes sent, not that a servo accepted
        # them. Check its goal register, including when a repeated write is skipped.
        if self.verify_servo_io:
            actual = self.goal_state()
            error = max(abs(actual.joints_deg[n]-goal.joints_deg[n]) for n in ARM)
            jaw = self.robot.calibration['gripper']
            jaw_tick = 1/(jaw.range_max-jaw.range_min)
            matched = error <= self.ENCODER_TICK_DEG+1e-7 and abs(actual.gripper_open-goal.gripper_open) <= jaw_tick+1e-7
            with (self.run_dir/'servo_goal_readback.local.jsonl').open('a') as stream:
                stream.write(json.dumps({'monotonic_s':time.monotonic(), 'written_motors':list(changed),
                    'requested':goal.as_dict(), 'readback':actual.as_dict(), 'matched':matched})+'\n')
            if not matched:
                raise RuntimeError('Servo goal register readback differs from the validated command')
        self._sent_motor_goals = values

    def advance(self, dt):
        pass  # Real-time pacing belongs to the controller, never the AI.

    def halt(self, *, preserve_gripper=False):
        if self.enabled and self._moving and self.robot and self.robot.bus.is_connected:
            pose = self.state()
            # _bounded_goal adds the held offset exactly once. Clip the nominal
            # freeze pose so the resulting servo target remains in joint limits.
            nominal = Pose({n: min(max(pose.joints_deg[n],
                                   self.config['limits_deg'][n][0]-self._hold_offset_deg[n]),
                                   self.config['limits_deg'][n][1]-self._hold_offset_deg[n])
                         for n in ARM}, pose.gripper_open if (self._gripper_moving and not preserve_gripper) or self._last_goal is None
                         else self._last_goal.gripper_open)
            bounded, _ = self._bounded_goal(nominal)
            self._write_goal(bounded)
            self._last_goal = bounded
            self.last_command = pose
            self._moving = False
        # An idle stop/close must preserve already-held goals, not erase their
        # gravity offset. An active stop freezes the measured nominal pose.
        # Default is hold, not torque-off: gravity could drop the arm/load.

    def close(self):
        if self.robot:
            for camera in self.robot.cameras.values():
                if camera.is_connected:
                    camera.disconnect()
            if self.robot.bus.is_connected:
                self.robot.bus.disconnect(disable_torque=False)
        if self._servo_monitor:
            self._servo_monitor.close()
