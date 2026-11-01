"""Single owner for hardware/GL, bounded local trajectories and latched stop."""
from concurrent.futures import Future
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

from .joints import ARM, Pose, interpolate, safe_duration, validate_limits


class Controller:
    def __init__(self, backend, config, run_dir, realtime=False):
        self.backend, self.config = backend, config
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.stop_path = self.run_dir / 'STOP'
        self.realtime = realtime or config['backend'] == 'real'
        self._queue = queue.Queue()
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._shutdown = threading.Event()
        self._ready = threading.Event()
        self._observations = {}
        self._requests = {}
        self._programs = {}
        self._job = None
        self._generation = 0
        self._connected = False
        self._gripper_command = None
        self._kinematics = None
        cartesian = config.get('cartesian_steps', {})
        if cartesian.get('enabled') and (config['backend'] == 'sim' or cartesian.get('real_kinematics_verified') is True):
            from .kinematics import RobotKinematics
            self._kinematics = RobotKinematics(config)
        self._status = {'backend': config['backend'], 'mode': backend.mode, 'state': 'initializing',
                        'connection_verified': False, 'last_action': None, 'task_success': 'unverified',
                        'stop_latched': False}
        self._thread = threading.Thread(target=self._loop, name='robot-owner', daemon=True)
        self._thread.start()
        if not self._ready.wait(30):
            self.stop()
            raise TimeoutError('Backend did not initialize within 30 seconds')

    def _log(self, event, **payload):
        entry = {'utc': datetime.now(timezone.utc).isoformat(), 'event': event, **payload}
        with (self.run_dir / 'events.jsonl').open('a') as f:
            f.write(json.dumps(entry, ensure_ascii=False, allow_nan=False)+'\n')

    def get_status(self):
        with self._lock:
            return json.loads(json.dumps(self._status))

    def _update(self, **values):
        with self._lock:
            self._status.update(values)

    def _call(self, method, *args):
        if not self._thread.is_alive():
            raise RuntimeError('Control worker is unavailable; restart locally')
        future = Future()
        self._queue.put((method, args, future))
        return future.result(timeout=20)

    def observe(self):
        # Keep the local single-RGB interface for existing scripts/evaluators.
        meta, images = self.observe_bundle()
        return meta, images[0]

    def observe_bundle(self):
        return self._call('observe')

    def sample_depth(self, observation_id, pixel_x, pixel_y):
        return self._call('sample_depth', observation_id, pixel_x, pixel_y)

    def locate_on_plane(self, observation_id, pixel_x, pixel_y, plane_z_m):
        return self._call('locate_on_plane', observation_id, pixel_x, pixel_y, plane_z_m)

    def measure_height(self, observation_id, bottom_pixel, top_pixel, center_pixel, support_z_m, visual_reason):
        return self._call('measure_height', observation_id, bottom_pixel, top_pixel, center_pixel, support_z_m, visual_reason)

    def prepare_cartesian_step(self, observation_id, target_xyz_m, gripper_open, instruction, visual_reason, expected_visual_change, tool_orientation='down', locked_joints=None):
        return self._call('prepare_cartesian_step', observation_id, target_xyz_m, gripper_open, instruction, visual_reason, expected_visual_change, tool_orientation, locked_joints)

    def execute_skill(self, name, args, observation_id, instruction, request_id):
        return self._call('execute', name, args, observation_id, instruction, request_id)

    def prepare_pick(self, observation_id, pixel_x, pixel_y, instruction, visual_reason):
        return self._call('prepare', observation_id, pixel_x, pixel_y, instruction, visual_reason)

    def prepare_step(self, observation_id, joint_delta_deg, gripper_delta, instruction,
                     visual_reason, expected_visual_change):
        return self._call('prepare_step', observation_id, joint_delta_deg, gripper_delta,
                          instruction, visual_reason, expected_visual_change)

    def report_result(self, action_id, observation_id, verdict, reason):
        return self._call('report', action_id, observation_id, verdict, reason)

    def stop(self):
        # This call never waits for AI, the work queue, image rendering or a motion.
        self._stop.set()
        self._update(stop_latched=True)
        return {'stop_requested': True, 'stop_latched': True,
                'note': 'Check get_status for local halt completion. Restart locally to clear.'}

    def _require_observation(self, oid):
        if self._stop.is_set() or self.stop_path.exists():
            raise RuntimeError('Stop is latched; local restart required')
        if not self._connected or self._job:
            raise RuntimeError('Backend is disconnected or another action is running')
        return self._recent_observation(oid)

    def _recent_observation(self, oid):
        record = self._observations.get(oid)
        maximum_age = self.config.get('observation_max_age_s',300)
        if record is None or time.monotonic()-record['monotonic'] > maximum_age:
            raise ValueError(f'A recent observe() result is required (maximum age {maximum_age} s)')
        if record['generation'] != self._generation:
            raise ValueError('Robot moved since this image; observe again')
        return record

    @staticmethod
    def _require_instruction(instruction):
        if not isinstance(instruction, str) or not 1 <= len(instruction) <= 1000:
            raise ValueError('User instruction is required, up to 1000 characters')

    def _validate_plan(self, plan):
        current = self.backend.state()
        validate_limits(current, self.config['limits_deg'],self.config.get('measured_limit_tolerance_deg',0.))
        # Measurement tolerance must NEVER become an out-of-range command.
        from .joints import Pose
        current=Pose({n:min(max(current.joints_deg[n],self.config['limits_deg'][n][0]),self.config['limits_deg'][n][1])
                      for n in ARM},current.gripper_open)
        waypoints = []
        for label, target, minimum in plan:
            validate_limits(target, self.config['limits_deg'])
            duration = safe_duration(current, target, minimum, self.config['speed_deg_s'], self.config['gripper_speed_s'])
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError('Invalid trajectory duration')
            waypoints.append((label, current, target, duration))
            current = target
        total = sum(p[3] for p in waypoints)
        if not waypoints or total > self.config['max_skill_s']:
            raise ValueError('Trajectory exceeds local time limit or is empty')
        return waypoints, total

    def _handle(self, kind, *args):
        if kind == 'observe':
            if not self._connected:
                raise RuntimeError('Backend is not connected; inspect get_status')
            if self._job:
                raise RuntimeError('Action is running; poll get_status, then observe after completion')
            pose = self.backend.state()
            if hasattr(self.backend, 'capture_bundle'):
                bundle = self.backend.capture_bundle()
            else:
                bundle = {'images': [self.backend.capture()], 'image_roles': ['rgb'], 'depth_m': None,
                          'sensors': {'depth': {'available': False}, 'end_effector': {'available': False}}}
            images = bundle['images']
            oid = uuid.uuid4().hex[:12]
            status = self.get_status()
            meta = {'observation_id': oid, 'captured_utc': datetime.now(timezone.utc).isoformat(),
                    'backend': self.config['backend'], 'camera': self.config.get('camera', self.config.get('camera_role')),
                    **pose.as_dict(), 'status': status,
                    'available_skills': {'pick_place': {'slot': ['slot_a', 'slot_b'], 'destination': ['box']},
                                         'observe_pose': {}},
                    'capabilities': {'continuous_image_pick': bool(self.config.get('continuous_pick')),
                                     'known_plane_localization': 'camera_calibration' in bundle,
                                     'depth_sampling': bundle['depth_m'] is not None},
                    'sensors': bundle['sensors'],
                    'images': [{'index': i, 'role': role, 'mime_type': 'image/png'}
                               for i, role in enumerate(bundle['image_roles'])],
                    'instruction': 'Use image pixels and the user instruction to select the object. State is not proof of task success.'}
            if 'camera_calibration' in bundle:
                meta['camera_calibration'] = bundle['camera_calibration']
            if 'simulation_time_s' in bundle:
                meta['simulation_time_s'] = bundle['simulation_time_s']
                meta['physics_during_deliberation'] = bool(self.config.get('idle_physics',False))
            meta['capabilities']['rgb_vertical_edge_height'] = 'camera_calibration' in bundle
            meta['capabilities']['cartesian_step'] = self._kinematics is not None
            if self._kinematics is not None:
                meta['sensors']['end_effector'] = {'available':True,**self._kinematics.forward(pose)}
                meta['cartesian_step_limits'] = self.config['cartesian_steps']
                meta['cartesian_tool_orientations'] = ['down','preserve_tilt','position_only']
                meta['position_only_constraints'] = {'wrist_roll':'held at measured angle',
                    'locked_joints':'optionally one additional arm joint',
                    'max_tool_axis_change_deg':self.config['cartesian_steps'].get('max_tool_axis_change_deg',5.),
                    'note':'orientation may change; not collision or grasp verification'}
                meta['available_skills'] = {'observe_pose':{},'prepared_cartesian_step':{'program_id':'returned by prepare_cartesian_step'}}
                meta['instruction'] += ' Unknown-height upright objects: measure_height from visible upper/lower endpoints of the SAME vertical edge. Prepare only one Cartesian or gripper step; inspect fresh RGB before the next step. No automatic task rollout.'
            if bundle['depth_m'] is not None:
                meta['instruction'] += ' Image 0 is RGB, image 1 is visualized depth. Call sample_depth on a visually selected RGB pixel for metric XYZ and surface height. Depth availability does not expand the supported motion workspace or object shapes.'
            if self.config.get('continuous_pick'):
                meta['camera_calibration'] = self.backend.camera_calibration()
                meta['image_pick_constraints'] = self.config['continuous_pick']
                meta['available_skills'] = {'prepared_pick': {'program_id': 'returned by prepare_pick'}, 'observe_pose': {}}
                meta['instruction'] += ' Select the TOP FACE CENTER in original image pixels; call prepare_pick, inspect its preview, then execute the generated program.'
            steps = self.config.get('incremental_steps', {})
            meta['capabilities']['incremental_joint_step'] = steps.get('enabled', False)
            if steps.get('enabled'):
                meta['incremental_step_limits'] = steps
                meta['available_skills']['prepared_step'] = {'program_id': 'returned by prepare_step'}
                meta['instruction'] += ' For iterative visual feedback, prepare_step creates ONE bounded joint-relative step. Inspect the new image after each step before choosing the next. Degrees are not Cartesian millimetres; do not infer contact or height from command completion.'
            record = {'meta': meta, 'monotonic': time.monotonic(), 'generation': self._generation,
                      'depth_m': bundle['depth_m']}
            self._observations[oid] = record
            # Bound dense depth memory; old IDs expire just like old images.
            while len(self._observations) > 16:
                self._observations.pop(next(iter(self._observations)))
            (self.run_dir / f'observe_{oid}.png').write_bytes(images[0])
            if record['depth_m'] is not None:
                import numpy as np
                (self.run_dir / f'observe_{oid}_depth.png').write_bytes(images[1])
                np.savez_compressed(self.run_dir / f'observe_{oid}_depth.npz', depth_m=record['depth_m'])
            self._log('observation', **meta)
            return meta, images
        if kind == 'locate_on_plane':
            oid, u, v, z = args
            record = self._recent_observation(oid)
            calibration = record['meta'].get('camera_calibration')
            if calibration is None:
                raise ValueError('RGB camera is not calibrated in the work frame; calibrate locally first')
            from .geometry import pixel_to_plane
            result = {'observation_id': oid, 'pixel_xy': [u, v],
                      'work_position_m': pixel_to_plane(calibration, u, v, z),
                      'work_frame': calibration['work_frame'], 'assumed_plane_z_m': z,
                      'height_measured': False,
                      'method': 'RGB-selected pixel ray intersected with an explicitly supplied known plane',
                      'limitation': 'Z is supplied, not observed. Wrong height or calibration changes XY. No robot movement.'}
            self._log('known_plane_localized', **result)
            return result
        if kind == 'measure_height':
            oid,bottom,top,center,support,reason=args
            record=self._require_observation(oid)
            calibration=record['meta'].get('camera_calibration')
            if calibration is None:
                raise ValueError('Calibrate the real RGB camera and support plane first')
            if not isinstance(reason,str) or not 1<=len(reason)<=2000:
                raise ValueError('Describe the visible vertical edge and support plane')
            from .metrology import measure_vertical
            from PIL import Image,ImageDraw
            import io
            result={'measurement_id':uuid.uuid4().hex[:12],'observation_id':oid,'visual_reason':reason,
                    **measure_vertical(calibration,bottom,top,center,support)}
            im=Image.open(self.run_dir/f'observe_{oid}.png').convert('RGB')
            draw=ImageDraw.Draw(im)
            draw.line([tuple(bottom),tuple(top)],fill='yellow',width=2)
            for u,v in ([bottom,top] if center is None else [bottom,top,center]):
                draw.ellipse([u-4,v-4,u+4,v+4],outline='yellow',width=2)
            draw.rectangle([0,im.height-28,im.width,im.height],fill='#102033')
            draw.text((12,im.height-20),f"RGB edge height: {result['estimated_height_m']*1000:.1f} mm (not supplied) | verify same edge endpoints",fill='yellow')
            stream=io.BytesIO();im.save(stream,format='PNG')
            (self.run_dir/f"measurement_{result['measurement_id']}.png").write_bytes(stream.getvalue())
            self._log('rgb_height_measured',**result)
            return result,stream.getvalue()
        if kind == 'sample_depth':
            oid, u, v = args
            record = self._recent_observation(oid)
            if record['depth_m'] is None:
                raise ValueError('Depth sensor is unavailable for this observation; no inferred values substituted')
            from .sensors import sample_depth
            result = {'observation_id': oid,
                      **sample_depth(record['depth_m'], record['meta']['camera_calibration'], u, v)}
            self._log('depth_sampled', **result)
            return result
        if kind == 'prepare':
            oid, u, v, instruction, reason = args
            record = self._require_observation(oid)
            self._require_instruction(instruction)
            if not isinstance(reason, str) or not 1 <= len(reason) <= 2000:
                raise ValueError('Describe your visual selection in 1..2000 characters')
            settings = self.config.get('continuous_pick')
            if not settings or 'camera_calibration' not in record['meta']:
                raise ValueError('Continuous image picking is not calibrated/enabled for this backend')
            from .geometry import locate_top
            from .programs import export_program
            localization = locate_top(record['meta']['camera_calibration'], u, v, settings)
            x, y = localization['pick_xy_m']
            plan = self.backend.plan('pick_place_xy', {'x_m': x, 'y_m': y, 'destination': 'box'})
            waypoints, total = self._validate_plan(plan)
            pid = uuid.uuid4().hex[:12]
            program = {'program_id': pid, 'observation_id': oid, 'instruction': instruction,
                       'visual_reason': reason, 'localization': localization,
                       'camera_calibration': record['meta']['camera_calibration'],
                       'planned_duration_s': total, 'destination': 'configured box',
                       'trajectory': [{'phase': n, 'start': start.as_dict(), 'target': p.as_dict(), 'duration_s': t}
                                      for n, start, p, t in waypoints],
                       'validation': ['image age and generation', 'pixel bounds and known-height plane',
                                      'workspace including assumed 3 px tolerance', 'IK residual <= 1 mm',
                                      'joint limits', 'cubic peak velocity', 'duration limit'],
                       'not_validated': ['occluded objects', 'arbitrary object shapes/rotations', 'general collision avoidance'],
                       'generator': 'host AI selects pixels; deterministic local IK and Python runner exporter; no LLM in server',
                       'expires': '300 s after observation, or any robot motion/server restart'}
            files, preview = export_program(self.run_dir, program, self.run_dir/f'observe_{oid}.png')
            self._programs[pid] = {'program': program, 'waypoints': waypoints, 'total': total}
            self._log('program_prepared', **program, files=files)
            return {**program, 'files_relative_to_run_dir': files,
                    'next': 'Inspect the yellow marker and generated script; run the Python file or execute_skill(prepared_pick).'}, preview
        if kind == 'prepare_cartesian_step':
            oid,xyz,opening,instruction,reason,expectation,orientation,locked=args
            self._require_observation(oid)
            self._require_instruction(instruction)
            if self._kinematics is None:
                raise ValueError('Cartesian model is not enabled/verified locally; real FK and work-frame calibration are required')
            if self.config['backend']=='real' and self.config.get('camera_role') not in ('fixed_workspace','overhead','front'):
                raise ValueError('Use a calibrated fixed workspace camera for real Cartesian steps; moving wrist camera is unsupported')
            if any(not isinstance(s,str) or not 1<=len(s)<=2000 for s in (reason,expectation)):
                raise ValueError('Visible evidence and expected change are required')
            from .kinematics import cartesian_step_plan
            from .programs import export_program
            settings=self.config['cartesian_steps']
            plan=cartesian_step_plan(self._kinematics,self.backend.state(),xyz,opening,settings,orientation,locked)
            waypoints,total=self._validate_plan(plan)
            if xyz is not None:
                # Contact holds the measured jaw open against a smaller command.
                # Re-commanding that measured opening would remove grasp force.
                from .joints import Pose
                waypoints=[(n,Pose(start.joints_deg,self._gripper_command),
                            Pose(target.joints_deg,self._gripper_command),t)
                           for n,start,target,t in waypoints]
            if total>settings['max_duration_s']:
                raise ValueError('Cartesian step duration limit exceeded')
            pid=uuid.uuid4().hex[:12]
            program={'program_id':pid,'execution_skill':'prepared_cartesian_step','observation_id':oid,
                     'instruction':instruction,'visual_reason':reason,'expected_visual_change':expectation,
                     'target_xyz_m':xyz,'target_gripper_open':opening,'automatic_next_step':False,
                     'tool_orientation':orientation if xyz is not None else None,
                     'locked_joints':locked or [],
                     'maintained_gripper_command':self._gripper_command if xyz is not None else None,
                     'planned_duration_s':total,'generator':'host selects ONE next action; robot-only IK and Python exporter',
                     'trajectory':[{'phase':n,'start':start.as_dict(),'target':p.as_dict(),'duration_s':t}
                                   for n,start,p,t in waypoints],
                     'validation':['fresh RGB','workspace','step displacement','joint limits','speed','duration','separate arm/gripper'],
                     'not_validated':['general obstacle avoidance','vision correctness','contact','task success']}
            files,preview=export_program(self.run_dir,program,self.run_dir/f'observe_{oid}.png')
            self._programs[pid]={'program':program,'waypoints':waypoints,'total':total}
            self._log('cartesian_program_prepared',**program,files=files)
            return {**program,'files_relative_to_run_dir':files},preview
        if kind == 'prepare_step':
            oid, deltas, grip, instruction, reason, expectation = args
            self._require_observation(oid)
            self._require_instruction(instruction)
            for text in (reason, expectation):
                if not isinstance(text, str) or not 1 <= len(text) <= 2000:
                    raise ValueError('Visual reason and expected visual change are required (1..2000 characters)')
            settings = self.config.get('incremental_steps', {})
            if self.config['backend'] == 'real' and self.backend.mode != 'dry_run' and settings.get('real_motion_verified') is not True:
                raise ValueError('Real incremental motion must be verified and enabled locally')
            from .incremental import step_target
            from .programs import export_program
            current = self.backend.state()
            target = step_target(current, deltas, grip, settings)
            waypoints, total = self._validate_plan([('incremental_step', target, settings['minimum_duration_s'])])
            if total > settings['max_duration_s']:
                raise ValueError('Incremental step duration limit exceeded')
            pid = uuid.uuid4().hex[:12]
            program = {'program_id': pid, 'execution_skill': 'prepared_step',
                       'observation_id': oid, 'instruction': instruction,
                       'visual_reason': reason, 'expected_visual_change': expectation,
                       'joint_delta_deg': deltas, 'gripper_delta': grip,
                       'planned_duration_s': total,
                       'trajectory': [{'phase': n, 'start': start.as_dict(), 'target': p.as_dict(), 'duration_s': t}
                                      for n, start, p, t in waypoints],
                       'height_measured': False, 'automatic_next_step': False,
                       'validation': ['fresh observation and robot generation', 'small joint/gripper increment',
                                      'separate arm and gripper steps', 'joint limits', 'speed and duration limits'],
                       'not_validated': ['Cartesian displacement', 'height or contact', 'collision-free path', 'task success'],
                       'generator': 'host AI chooses the next increment from RGB; deterministic one-step Python exporter; no LLM in server',
                       'expires': '300 s after observation, or any robot motion/server restart'}
            files, preview = export_program(self.run_dir, program, self.run_dir/f'observe_{oid}.png')
            self._programs[pid] = {'program': program, 'waypoints': waypoints, 'total': total}
            self._log('incremental_program_prepared', **program, files=files)
            return {**program, 'files_relative_to_run_dir': files,
                    'next': 'Execute this single step, inspect the returned NEW RGB, then decide whether to generate another step or stop. No automatic continuation.'}, preview
        if kind == 'execute':
            name, skill_args, oid, instruction, rid = args
            if not isinstance(rid, str) or not 1 <= len(rid) <= 120:
                raise ValueError('A nonempty request_id up to 120 characters is required')
            signature = json.dumps([name, skill_args, oid, instruction], sort_keys=True)
            if rid in self._requests:
                old_sig, result = self._requests[rid]
                if old_sig != signature:
                    raise ValueError('request_id already used with different arguments')
                return {**result, 'idempotent_replay': True}
            self._require_observation(oid)
            self._require_instruction(instruction)
            if name in ('prepared_pick', 'prepared_step', 'prepared_cartesian_step'):
                if set(skill_args) != {'program_id'} or skill_args['program_id'] not in self._programs:
                    raise ValueError('Unknown prepared program; prepare from a fresh observation')
                stored = self._programs[skill_args['program_id']]
                program = stored['program']
                if name != program.get('execution_skill', 'prepared_pick'):
                    raise ValueError('Prepared program skill type does not match')
                if oid != program['observation_id'] or instruction != program['instruction']:
                    raise ValueError('Prepared program belongs to another image or instruction')
                waypoints, total = stored['waypoints'], stored['total']
                current, initial = self.backend.state(), waypoints[0][1]
                validate_limits(current, self.config['limits_deg'],self.config.get('measured_limit_tolerance_deg',0.))
                maintains_grasp=name=='prepared_cartesian_step' and program.get('target_xyz_m') is not None
                if max(abs(current.joints_deg[n]-initial.joints_deg[n]) for n in ARM) > .5 or (not maintains_grasp and abs(current.gripper_open-initial.gripper_open) > .03):
                    raise ValueError('Measured pose changed since preparation; observe and prepare again')
            else:
                if name not in ('pick_place', 'observe_pose'):
                    raise ValueError('Use a named skill or prepared_pick; direct coordinates require prepare_pick')
                waypoints, total = self._validate_plan(self.backend.plan(name, skill_args))
            motion_context = None
            if hasattr(self.backend, 'begin_motion'):
                motion_context = self.backend.begin_motion(waypoints)
            action_id = uuid.uuid4().hex[:12]
            result = {'action_id': action_id, 'accepted': True, 'planned_duration_s': total,
                      'command_status': 'running', 'task_success': 'unverified'}
            self._requests[rid] = signature, result
            self._generation += 1
            self._job = {'action_id': action_id, 'name': name, 'args': skill_args, 'plan': waypoints,
                         'index': 0, 't': 0., 'elapsed': 0., 'started': time.monotonic(),
                         'monitor_cartesian': (name == 'prepared_cartesian_step'
                             and program.get('target_xyz_m') is not None
                             and self.config['backend'] == 'real' and self.backend.mode != 'dry_run'),
                         'instruction': instruction, 'request_id': rid, 'observation_id': oid}
            self._update(state='running', active_action=result, phase=waypoints[0][0], task_success='unverified')
            self._log('action_started', **result, name=name, args=skill_args, instruction=instruction,
                      request_id=rid, observation_id=oid, motion_context=motion_context,
                      trajectory=[{'phase': n, 'target': p.as_dict(), 'duration_s': t} for n, _, p, t in waypoints])
            return result
        if kind == 'report':
            action_id, oid, verdict, reason = args
            if verdict not in ('success', 'failure', 'uncertain') or not isinstance(reason, str) or len(reason) > 2000:
                raise ValueError('Use success, failure or uncertain with a concise visual reason')
            record = self._observations.get(oid)
            if record is None:
                raise ValueError('Unknown observation_id')
            status = record['meta']['status']
            last = status.get('last_action') or {}
            if status['state'] == 'running' or last.get('action_id') != action_id:
                raise ValueError('A post-action observation of this action is required')
            result = {'action_id': action_id, 'observation_id': oid, 'ai_visual_verdict': verdict,
                      'reason': reason, 'independent_evaluation': 'separate_offline_file'}
            self._log('ai_visual_verdict', **result)
            self._update(task_success={'source': 'ai_visual_verdict', 'verdict': verdict})
            return result
        raise ValueError('Unknown local request')

    def _finish(self, state, error=None):
        job = self._job
        if job is None:
            self._update(state=state)
            return
        result = {'action_id': job['action_id'], 'name': job['name'], 'args': job['args'],
                  'command_status': 'dry_run' if state == 'completed' and self.backend.mode == 'dry_run' else state,
                  'trajectory_s': job['elapsed'], 'wall_s': time.monotonic()-job['started'],
                  'task_success': 'unverified', 'error': error}
        self._job = None
        if state == 'completed' and hasattr(self.backend, 'end_motion'):
            self.backend.end_motion()
        self._update(state=state, active_action=None, phase=None, last_action=result)
        self._log('action_finished', **result)
        if self.config['backend'] == 'sim':
            from .evaluation import evaluate_sim
            evaluation = evaluate_sim(self.backend)
            evaluation['action_id'] = result['action_id']
            (self.run_dir / 'evaluation_private.json').write_text(json.dumps(evaluation, indent=2)+'\n')
            with (self.run_dir/'evaluation_history_private.jsonl').open('a') as f:
                f.write(json.dumps(evaluation)+'\n')

    def _tick(self, dt):
        job = self._job
        if not job:
            return
        if time.monotonic()-job['started'] > self.config['max_skill_s']+10:
            raise TimeoutError('Local action watchdog expired')
        label, start, target, duration = job['plan'][job['index']]
        job['t'] = min(duration, job['t']+dt)
        pose = interpolate(start, target, job['t']/duration)
        if job.get('monitor_cartesian'):
            pose = self._coordinate_upward_reference(job, start, target, pose, dt)
        validate_limits(pose, self.config['limits_deg'])
        accepted=self.backend.command(pose)
        # The real adapter may cap a closing command at its relative limit.
        # Keep that accepted goal across stages, not an unsent nominal target
        # or the contact-limited measured jaw opening. Sim returns None.
        self._gripper_command=accepted.gripper_open if accepted is not None else pose.gripper_open
        self.backend.advance(dt)
        job['elapsed'] += dt
        measured = self.backend.state()
        validate_limits(measured, self.config['limits_deg'],self.config.get('measured_limit_tolerance_deg',0.))
        self._update(joints_deg=measured.joints_deg, gripper_open=measured.gripper_open, phase=label)
        if job.get('monitor_cartesian'):
            self._check_cartesian_tracking(start,target,pose,measured)
        if job['t']+1e-9 >= duration:
            error = max(abs(measured.joints_deg[n]-target.joints_deg[n]) for n in ARM)
            tolerance=self.config['tracking_tolerance_deg']
            # A 1-degree check must not pass with a 6-degree commissioning band.
            # Brief settling keeps the target fixed locally; it never generates
            # another task decision or increases the goal/velocity limits.
            if self.config['backend']=='real' and self.backend.mode!='dry_run':
                change=max(abs(target.joints_deg[n]-start.joints_deg[n]) for n in ARM)
                if change>.15:
                    tolerance=min(tolerance,max(.15,min(.5,change*.5)))
            if self.backend.mode != 'dry_run' and error > tolerance:
                job['settling_s']=job.get('settling_s',0.)+dt
                if self.config['backend']=='real' and job['settling_s']<=2.:
                    return
                raise RuntimeError(f'Arm tracking error {error:.2f} degrees at {label} (limit {tolerance:.2f})')
            if job.get('monitor_cartesian'):
                self._check_cartesian_tracking(start,target,target,measured)
            self._log('phase_finished', action_id=job['action_id'], phase=label,
                      arm_error_deg=error, measured=measured.as_dict())
            job['index'] += 1
            job['t'] = 0.
            job['settling_s']=0.
            job.pop('following_progress',None)
            job.pop('reference_pose',None)
            if job['index'] == len(job['plan']):
                self._finish('completed')

    def _coordinate_upward_reference(self, job, start, target, planned, dt):
        """For a locally enabled vertical retreat, wait for the lifting joint.

        A slow elbow and a fast wrist can otherwise turn an upward reference
        into downward actual motion. Peers follow measured elbow progress;
        the elbow reference leads by at most 0.5 degree to build bounded effort.
        Existing servo caps, endpoint, time and Cartesian stop guards still apply.
        This is reference pacing, not another host decision or a task retry.
        """
        settings=self.config.get('cartesian_steps',{})
        joint=settings.get('upward_progress_joint')
        if not joint or self.config['backend']!='real' or self.backend.mode=='dry_run':
            return planned
        if joint not in ARM:
            raise ValueError('Upward progress joint must name an arm joint')
        lead=settings.get('max_reference_lead_deg',.5)
        if isinstance(lead,bool) or not isinstance(lead,(float,int)) or not math.isfinite(lead) or not 0<lead<=.5:
            raise ValueError('Upward reference lead must be in (0, 0.5] degrees')
        origin=self._kinematics.forward(start)['position_m']
        goal=self._kinematics.forward(target)['position_m']
        delta={n:target.joints_deg[n]-start.joints_deg[n] for n in ARM}
        if goal[2]-origin[2]<.0001 or math.dist(goal[:2],origin[:2])>.0001 or abs(delta[joint])<.2:
            return planned
        measured=self.backend.state()
        progress=max(0.,min(1.,(measured.joints_deg[joint]-start.joints_deg[joint])/delta[joint]))
        scheduled=max(0.,min(1.,(planned.joints_deg[joint]-start.joints_deg[joint])/delta[joint]))
        previous=job.get('following_progress',0.)
        peer_change=max(abs(delta[n]) for n in ARM if n!=joint)
        slew=self.config['speed_deg_s']*dt/max(peer_change,1e-8)
        following=max(previous,min(scheduled,progress,previous+slew))
        job['following_progress']=following
        raw_progress=(measured.joints_deg[joint]-start.joints_deg[joint])/delta[joint]
        lifting=min(scheduled,max(0.,raw_progress+lead/abs(delta[joint])))
        values={n:start.joints_deg[n]+delta[n]*(lifting if n==joint else following) for n in ARM}
        # Actual stick-slip may jump ahead: the nominal reference still obeys
        # the same per-tick speed bound, including the lifting joint.
        prior=job.get('reference_pose',start)
        values={n:max(prior.joints_deg[n]-self.config['speed_deg_s']*dt,
                      min(prior.joints_deg[n]+self.config['speed_deg_s']*dt,v)) for n,v in values.items()}
        governed=Pose(values,planned.gripper_open)
        point=self._kinematics.forward(governed)['position_m']
        if self.config.get('workspace_verified') and any(
                not low<=value<=high for value,(low,high) in zip(point,settings['workspace_xyz_m'],strict=True)):
            raise RuntimeError('Coordinated reference leaves the locally verified workspace')
        job['reference_pose']=governed
        with (self.run_dir/'reference_governor.local.jsonl').open('a') as stream:
            stream.write(json.dumps({'action_id':job['action_id'],'joint':joint,
                'measured_progress':progress,'following_progress':following,
                'scheduled':planned.as_dict(),'reference':governed.as_dict(),
                'measured':measured.as_dict()})+'\n')
        return governed

    def _check_cartesian_tracking(self,start,target,expected,measured):
        """Fail locally if measured joints imply a stalled or reversed tool path.

        This checks relative motion in the robot model, not absolute calibration
        accuracy, collision clearance, force or task success.
        """
        settings=self.config['cartesian_steps']
        tolerance=settings.get('tracking_tolerance_m',.003)
        reverse=settings.get('reverse_tolerance_m',.001)
        if any(not isinstance(v,(float,int)) or not math.isfinite(v) or v<=0 for v in (tolerance,reverse)):
            raise ValueError('Positive finite Cartesian tracking tolerances required')
        origin=self._kinematics.forward(start)['position_m']
        goal=self._kinematics.forward(target)['position_m']
        planned=self._kinematics.forward(expected)['position_m']
        actual=self._kinematics.forward(measured)['position_m']
        if self.config.get('workspace_verified') and any(
                not low<=value<=high for point in (planned,actual)
                for value,(low,high) in zip(point,settings['workspace_xyz_m'],strict=True)):
            raise RuntimeError('Cartesian pose leaves the locally verified workspace')
        distance=math.dist(origin,goal)
        progress=sum((a-o)*(g-o) for a,o,g in zip(actual,origin,goal,strict=True))/distance if distance>1e-6 else 0.
        if progress < -reverse:
            raise RuntimeError(f'Cartesian reverse motion {progress*1000:.2f} mm from measured joints')
        error=math.dist(actual,planned)
        if error>tolerance:
            raise RuntimeError(f'Cartesian tracking error {error*1000:.2f} mm from measured joints')

    def _loop(self):
        try:
            self.backend.connect()
            self._connected = True
            pose = self.backend.state()
            held=getattr(self.backend,'held_gripper_open',None)
            self._gripper_command=held if held is not None else pose.gripper_open
            self._update(state='idle', connection_verified=True, **pose.as_dict())
            self._log('backend_connected', backend=self.config['backend'], mode=self.backend.mode)
        except Exception as exc:
            # Do not write arbitrary exception text (device paths, identifiers) into public logs.
            stage = getattr(self.backend, 'connection_stage', 'unknown')
            self._update(state='failed', initialization_stage=stage,
                         error=f'{type(exc).__name__}: backend initialization failed at {stage}; inspect initialization_error.local.json')
            self._log('connection_failed', error_type=type(exc).__name__, initialization_stage=stage)
            # Diagnostic text stays local; public exporters use an allowlist.
            import traceback
            private = self.run_dir/'initialization_error.local.json'
            with private.open('x', encoding='utf-8') as stream:
                private.chmod(0o600)
                json.dump({'stage':stage, 'error_type':type(exc).__name__,
                           'error':str(exc), 'traceback':traceback.format_exc()}, stream, indent=2)
        finally:
            self._ready.set()
        dt = 1/self.config['control_hz']
        halted = False
        idle_clock=time.monotonic()
        try:
            while not self._shutdown.is_set():
                started = time.monotonic()
                if self.stop_path.exists():
                    self._stop.set()
                if self._stop.is_set() and not halted:
                    if self._connected:
                        try:
                            self.backend.halt()
                        except Exception as exc:
                            self._update(halt_error=type(exc).__name__)
                    self._finish('stopped')
                    self._update(stop_latched=True)
                    self._log('stop_applied')
                    halted = True
                try:
                    item = self._queue.get(timeout=0 if self._job else .02)
                except queue.Empty:
                    item = None
                if item:
                    method, args, future = item
                    try:
                        future.set_result(self._handle(method, *args))
                    except Exception as exc:
                        future.set_exception(exc)
                if self._job and not self._stop.is_set():
                    try:
                        self._tick(dt)
                    except Exception as exc:
                        try:
                            self.backend.halt()
                        except Exception as halt_exc:
                            self._update(halt_error=type(halt_exc).__name__)
                        self._finish('failed', f'{type(exc).__name__}: {exc}')
                        self._stop.set()
                    idle_clock=time.monotonic()
                elif self._connected and self.config.get('idle_physics') and not self._stop.is_set():
                    # Continue physical settling/slip while the host thinks. No object state
                    # enters the controller; the backend simply steps its existing dynamics.
                    now=time.monotonic()
                    ticks=min(5,int((now-idle_clock)/dt))
                    if ticks:
                        try:
                            for _ in range(ticks):self.backend.advance(dt)
                        except Exception as exc:
                            self._update(state='failed',error=f'{type(exc).__name__}: idle physics failed')
                            self._stop.set()
                        idle_clock+=ticks*dt
                if self.realtime:
                    self._shutdown.wait(max(0., dt-(time.monotonic()-started)))
        finally:
            if self._job:
                self._finish('stopped', 'local shutdown')
            if self._connected:
                try:
                    self.backend.halt()
                finally:
                    self.backend.close()
            else:
                self.backend.close()

    def close(self):
        self.stop()
        self._shutdown.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise RuntimeError('Control worker did not stop; check hardware locally')
