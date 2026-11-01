"""MuJoCo backend. Only actuator targets change during a trial.

Object poses are set in MjSpec before compilation. The separate kinematics data
used by IK never changes the live physics data. No welds, mocap or grasp latch.
"""
import io
import math
import random
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scipy.optimize import least_squares

from .config import ROOT
from .joints import ARM, Pose


class SimBackend:
    mode = 'simulation'

    def __init__(self, config, run_dir, seed=0, record=True):
        self.config = config
        self.run_dir = Path(run_dir)
        self.record = record
        self.seed = seed
        self.writer = None

    def connect(self):
        scene = mujoco.MjSpec.from_file(str(ROOT / 'config/scene.xml'))
        if 'camera_override' in self.config:
            from .camera_views import apply_camera_override
            apply_camera_override(scene, self.config['camera'], self.config['camera_override'])
        colors = ['red', 'blue']
        rng = random.Random(self.seed)
        rng.shuffle(colors)
        positions = list(self.config['slots'].values())
        if self.config.get('layout') == 'continuous':
            # Scene initialization only. Never used by the perception/control API.
            bounds = self.config['spawn_xy_bounds_m']
            positions = []
            for _ in colors:
                for _ in range(1000):
                    candidate = [rng.uniform(*interval) for interval in bounds]
                    if all(np.linalg.norm(np.array(candidate)-p) >= .075 for p in positions):
                        positions.append(candidate)
                        break
                else:
                    raise ValueError('Cannot initialize separated blocks in spawn workspace')
            for marker in ('slot_1', 'slot_2'):
                scene.geom(marker).rgba = [0, 0, 0, 0]
        for color, xy in zip(colors, positions, strict=True):
            scene.body(f'{color}_cube').pos = [*xy, .015]
        # Hidden experiment initialization only. Heights are intentionally absent
        # from observe/tool outputs; the host must estimate them from visible RGB.
        randomization=self.config.get('object_randomization')
        if randomization:
            shape_rng=random.Random(self.seed+97431)
            for color in colors:
                height=shape_rng.uniform(*randomization['height_range_m'])
                width=shape_rng.uniform(*randomization['width_range_m'])
                body=scene.body(f'{color}_cube')
                for geom in body.geoms:geom.size=[width/2,width/2,height/2]
                body.pos=[*body.pos[:2],height/2]
        if self.config.get('initial_stack')=='red_on_blue':
            bottom=scene.body('blue_cube');top=scene.body('red_cube')
            top.pos=[*bottom.pos[:2],2*bottom.pos[2]+top.pos[2]+.0002]
        arm = mujoco.MjSpec.from_file(str(ROOT / 'assets/so101_tabletop/robotstudio_so101/so101.xml'))
        for prop in ('timestep', 'iterations', 'ls_iterations', 'cone', 'impratio', 'integrator'):
            setattr(arm.option, prop, getattr(scene.option, prop))
        for mat in arm.materials:
            if not mat.name.startswith('sts3215'):
                mat.rgba = [.30, .62, .68, 1]
        scene.attach(arm, site='so101_mount', prefix='robot/')
        self.model = scene.compile()
        self.data = mujoco.MjData(self.model)
        self.kin = mujoco.MjData(self.model)
        self.qadr = np.array([self.model.joint('robot/'+n).qposadr[0] for n in (*ARM, 'gripper')])
        self.act = np.array([self.model.actuator('robot/'+n).id for n in (*ARM, 'gripper')])
        self.site = self.model.site('robot/gripperframe').id
        self.glo, self.ghi = self.model.actuator_ctrlrange[self.act[-1]]
        # Initial robot pose only; all subsequent movement is through mj_step.
        self.command(Pose.from_dict(self.config['home']))
        self.data.qpos[self.qadr] = self.data.ctrl[self.act]
        mujoco.mj_forward(self.model, self.data)
        self.renderer = mujoco.Renderer(self.model, self.config['image_height'], self.config['image_width'])
        self.vis = mujoco.MjvOption()
        self.vis.geomgroup[3:] = 0
        self.next_frame = 0.
        self.audit = {'contact_steps': {'red': 0, 'blue': 0}, 'max_height_m': {'red': 0., 'blue': 0.}}
        self.fingers = {self.model.body('robot/'+n).id for n in ('gripper', 'moving_jaw_so101_v1')}
        self.block_ids = {self.model.body(c+'_cube').id: c for c in ('red', 'blue')}
        # Settle before the first observation, using gravity and contact only.
        for _ in range(250):
            mujoco.mj_step(self.model, self.data)
        self.epoch = self.data.time
        if self.record:
            import imageio.v2 as imageio
            self.writer = imageio.get_writer(self.run_dir / 'simulation.mp4', fps=25, codec='libx264', quality=8)

    @property
    def elapsed(self):
        return float(self.data.time-self.epoch)

    def state(self):
        q = self.data.qpos[self.qadr]
        return Pose(dict(zip(ARM, np.rad2deg(q[:5]).tolist(), strict=True)),
                    float(np.clip((q[5]-self.glo)/(self.ghi-self.glo), 0., 1.)))

    def command(self, pose):
        self.data.ctrl[self.act] = [*(math.radians(pose.joints_deg[n]) for n in ARM),
                                    self.glo+pose.gripper_open*(self.ghi-self.glo)]
        return pose

    def advance(self, dt):
        for _ in range(round(dt/self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
            # Independent evaluator instrumentation, never returned to MCP.
            for body, color in self.block_ids.items():
                self.audit['max_height_m'][color] = max(self.audit['max_height_m'][color], float(self.data.xpos[body, 2]))
            touching = set()
            for c in self.data.contact:
                a, b = self.model.geom_bodyid[c.geom1], self.model.geom_bodyid[c.geom2]
                if a in self.block_ids and b in self.fingers:
                    touching.add(self.block_ids[a])
                if b in self.block_ids and a in self.fingers:
                    touching.add(self.block_ids[b])
            for color in touching:
                self.audit['contact_steps'][color] += 1
        if self.writer is not None and self.elapsed+1e-8 >= self.next_frame:
            self.writer.append_data(np.asarray(self.render(self.config.get('video_camera'))))
            self.next_frame = self.elapsed+.04

    def _project(self, xyz, camera_name=None):
        camera = self.model.camera(camera_name or self.config['camera']).id
        local = self.data.cam_xmat[camera].reshape(3, 3).T @ (np.array(xyz)-self.data.cam_xpos[camera])
        h, w = self.config['image_height'], self.config['image_width']
        f = .5*h/math.tan(math.radians(self.model.cam_fovy[camera])/2)
        return int(w/2+f*local[0]/-local[2]), int(h/2-f*local[1]/-local[2])

    def camera_calibration(self):
        camera = self.model.camera(self.config['camera']).id
        h, w = self.config['image_height'], self.config['image_width']
        return {'width': w, 'height': h, 'pixel_origin': 'top_left_u_right_v_down',
                'principal_point_px': [(w-1)/2, (h-1)/2],
                'pixel_coordinates': 'integer indices denote pixel centres',
                'focal_px': .5*h/math.tan(math.radians(self.model.cam_fovy[camera])/2),
                'camera_position_m': self.data.cam_xpos[camera].tolist(),
                'camera_to_work_rotation': self.data.cam_xmat[camera].reshape(3, 3).tolist(),
                'source': 'simulator camera calibration, not object pose or depth',
                'distortion': 'none', 'work_frame': 'robot_base_table_z0'}

    def render(self, camera_name=None):
        camera_name = camera_name or self.config['camera']
        self.renderer.update_scene(self.data, camera=camera_name, scene_option=self.vis)
        im = Image.fromarray(self.renderer.render())
        draw = ImageDraw.Draw(im)
        for name, xy in (self.config['slots'].items() if self.config.get('layout') != 'continuous' else []):
            x, y = self._project([*xy, .001], camera_name)
            draw.rounded_rectangle([x-31, y+15, x+31, y+34], 4, fill='#112233')
            draw.text((x-24, y+17), name, fill='white')
        draw.rectangle([0, 0, im.width, 34], fill='#102033')
        draw.text((12, 10), f'SO-101 | MuJoCo | simulation time {self.elapsed:6.2f} s | 1x simulation time', fill='white')
        if self.config.get('layout') == 'continuous' and camera_name == self.config['camera']:
            # Fixed pixel rulers assist visual pointing; no scene object queries.
            for u in range(100, im.width, 100):
                draw.line([u, 34, u, 43], fill='#112233', width=2)
                draw.text((u-8, 45), str(u), fill='#112233')
            for v in range(100, im.height, 100):
                draw.line([0, v, 10, v], fill='#112233', width=2)
                draw.text((13, v-4), str(v), fill='#112233')
        return im

    def capture(self):
        stream = io.BytesIO()
        self.render().save(stream, format='PNG')
        return stream.getvalue()

    def capture_bundle(self):
        """RGB + optional metric depth at one frozen physics instant."""
        from .sensors import depth_preview
        rgb = self.capture()
        calibration = self.camera_calibration()
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[self.site])
        sensors = {'rgb': {'available': True, 'source': 'MuJoCo RGB renderer'},
                   'depth': {'available': False, 'reason': 'depth_camera not enabled'},
                   'end_effector': {'available': True,
                       'source': 'forward kinematics from simulated joint positions',
                       'frame': 'robot_base_table_z0', 'tool_frame': 'robot/gripperframe',
                       'position_m': self.data.site_xpos[self.site].tolist(),
                       'quaternion_wxyz': quat.tolist(),
                       'local_axes_in_work': self.data.site_xmat[self.site].reshape(3, 3).tolist(),
                       'note': 'Robot tool-frame pose, not a block location or contact measurement.'}}
        if self.config.get('portable_observations', True):
            sensors['end_effector'] = {'available': False,
                'reason': 'RGB/common-input profile: real tool-frame calibration is not verified'}
        bundle = {'images': [rgb], 'sensors': sensors,
                  'camera_calibration': calibration, 'simulation_time_s': self.elapsed,
                  'image_roles': ['rgb'], 'depth_m': None}
        if self.config.get('depth_camera'):
            self.renderer.update_scene(self.data, camera=self.config['camera'], scene_option=self.vis)
            self.renderer.enable_depth_rendering()
            try:
                depth = self.renderer.render().copy()
            finally:
                self.renderer.disable_depth_rendering()
            # MuJoCo returns metric camera-forward depth. Background at the far
            # clip and out-of-sensor-range pixels are invalid, never fake heights.
            near = self.model.vis.map.znear*self.model.stat.extent
            far = self.model.vis.map.zfar*self.model.stat.extent
            limits = [max(float(near), .05), min(float(far)*.999, 2.5)]
            depth[(~np.isfinite(depth)) | (depth <= limits[0]) | (depth >= limits[1])] = np.nan
            # RGB annotations hide these pixels. Do not return a surface behind
            # a label as though it had been visually selected by the host.
            depth[:34, :] = np.nan
            if self.config.get('layout') == 'continuous':
                depth[:65, :] = np.nan
                depth[:, :40] = np.nan
            else:
                for xy in self.config['slots'].values():
                    u, v = self._project([*xy, .001])
                    depth[max(0, v+15):min(depth.shape[0], v+35),
                          max(0, u-31):min(depth.shape[1], u+32)] = np.nan
            sensors['depth'] = {'available': True, 'virtual_sensor': True,
                'source': 'MuJoCo Renderer metric depth buffer', 'aligned_to': 'rgb',
                'units': 'metres', 'convention': 'positive camera-forward Z, not Euclidean range',
                'valid_range_m': limits, 'noise_model': 'none; ideal simulation',
                'numeric_tool': 'sample_depth', 'table_plane_z_m': 0.,
                'note': 'Rendered visible surfaces only; no object IDs, segmentation or object poses.'}
            bundle['depth_m'] = depth
            bundle['images'].append(depth_preview(depth))
            bundle['image_roles'].append('depth_visualization')
        return bundle

    def _ik(self, xyz, opening):
        # Geometric IK, with no object identities/poses as inputs.
        def residual(q):
            self.kin.qpos[self.qadr[:5]] = q
            mujoco.mj_kinematics(self.model, self.kin)
            axis = self.kin.site_xmat[self.site].reshape(3, 3)[:, 0]
            return np.r_[self.kin.site_xpos[self.site]-xyz, .1*(axis-[0, 0, -1]), .03*q[4]]
        lo, hi = np.deg2rad([self.config['limits_deg'][n] for n in ARM]).T
        result = least_squares(residual, np.deg2rad([0, -25, 40, 70, 0]), bounds=(lo, hi),
                               ftol=1e-11, xtol=1e-11, gtol=1e-11)
        if np.linalg.norm(residual(result.x)) > .001:
            raise ValueError('Requested location is unreachable with a downward grasp')
        return Pose(dict(zip(ARM, np.rad2deg(result.x).tolist(), strict=True)), opening)

    def plan(self, name, args):
        c = self.config
        if name == 'observe_pose':
            if args:
                raise ValueError('observe_pose takes no arguments')
            return [('observe_pose', Pose.from_dict(c['home']), 4.)]
        if name == 'pick_place_xy':
            if not c.get('continuous_pick') or set(args) != {'x_m', 'y_m', 'destination'} or args['destination'] != 'box':
                raise ValueError('Continuous picking requires configured workspace and x_m, y_m, destination=box')
            xy = [args['x_m'], args['y_m']]
            for value, (lo, hi) in zip(xy, c['continuous_pick']['pick_xy_bounds_m'], strict=True):
                if not math.isfinite(value) or not lo <= value <= hi:
                    raise ValueError('Pick coordinate is outside the configured workspace')
            return self._plan_xy(xy)
        if c.get('layout') == 'continuous':
            raise ValueError('Continuous layout requires prepare_pick from an image pixel, then prepared_pick')
        if name != 'pick_place' or set(args) != {'slot', 'destination'}:
            raise ValueError('Use pick_place with slot and destination, or observe_pose')
        if args['slot'] not in c['slots'] or args['destination'] != 'box':
            raise ValueError('Only configured slots and destination=box are supported')
        return self._plan_xy(c['slots'][args['slot']])

    def _plan_xy(self, xy):
        c = self.config
        op, cl = c['gripper_open'], c['gripper_closed']
        return [
            ('approach', self._ik([*xy, c['approach_z']], op), 4.),
            ('descend', self._ik([*xy, c['pick_z']], op), 2.),
            ('close', self._ik([*xy, c['pick_z']], cl), 2.),
            ('lift', self._ik([*xy, c['carry_z']], cl), 3.),
            ('transfer', self._ik([*c['box_xy'], c['carry_z']], cl), 5.),
            ('release', self._ik([*c['box_xy'], c['carry_z']], op), 2.),
            ('observe_pose', Pose.from_dict(c['home']), 4.)]

    def halt(self):
        self.command(self.state())

    def close(self):
        if self.writer:
            self.writer.close()
        if hasattr(self, 'renderer'):
            self.renderer.close()
