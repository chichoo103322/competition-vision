"""按键路线的真实视觉对齐测试。对齐、收回后放行，不冒充实际抓取/搬运。"""
import time
import math
from urllib.parse import urlsplit
from pi_vision.arm_alignment import ArmCalibration, MechanismPose, number
from pi_vision.task_confirmation import parse_competition_task


class MaterialUnavailable(RuntimeError):
    pass


def station_calibration(config, station):
    return ArmCalibration(config['calibrations'][station['calibration_id']])


def validate_config(config, required_stations=range(102, 108)):
    if config.get('enabled') is not True:
        raise ValueError('arm alignment must be explicitly enabled')
    if config.get('mode') != 'alignment_only' or config.get('firmware_and_clearances_verified') is not True:
        raise ValueError('alignment test requires verified firmware/origin/clearance settings')
    url = urlsplit(config['vision_state_url'])
    if url.scheme != 'http' or url.hostname not in ('127.0.0.1', 'localhost') or url.username or url.password:
        raise ValueError('reuse local vision state service; no remote/stale snapshots')
    for field in ('station_timeout_s', 'motion_timeout_s', 'observation_timeout_s', 'settle_s',
                  'tolerance_px', 'confirm_span_s', 'max_frame_gap_s', 'max_step_mm',
                  'max_yaw_step_deg', 'max_total_mm'):
        if not number(config.get(field)) or config[field] <= 0:
            raise ValueError('positive measured alignment setting required: ' + field)
    if type(config.get('confirm_frames')) is not int or config['confirm_frames'] < 3:
        raise ValueError('at least 3 distinct alignment frames required')
    if type(config.get('max_adjustments')) is not int or config['max_adjustments'] < 1:
        raise ValueError('positive max_adjustments required')
    if set(config['stations']) != {str(i) for i in required_stations}:
        raise ValueError('all required managed stations must be explicit; no timed fallback')
    for key, station in config['stations'].items():
        c = station_calibration(config, station)
        target = station['target']
        if station.get('plane_id') != c.data['plane_id']:
            raise ValueError('station plane/calibration mismatch')
        if target.get('kind') not in ('material', 'ring'):
            raise ValueError('explicit material or ring target required')
        if station.get('layer') != 0:
            raise ValueError('stacked-layer alignment has not been commissioned')
        if 'search_poses' in station:
            poses = station['search_poses']
            if key not in ('102', '105') or target['kind'] != 'material':
                raise ValueError('search poses are for raw-material stations only')
            if not isinstance(poses, list) or len(poses) > config['max_adjustments']:
                raise ValueError('bounded measured search poses required')
            for pose in poses:
                if not isinstance(pose, dict) or set(pose) != {'z_mm', 'yaw_deg', 'reach_mm'}:
                    raise ValueError('search pose requires absolute mechanism coordinates')
                c.motor_pose(MechanismPose(**pose))
            if not number(station.get('search_wait_s')) or station['search_wait_s'] <= 0:
                raise ValueError('positive search observation wait required')
        if 'task_group' in target:
            allowed = ('group1', 'group3') if target['kind'] == 'material' else ('group2', 'group4')
            if target['task_group'] not in allowed or type(target.get('task_index')) is not int or not 0 <= target['task_index'] < 3:
                raise ValueError('invalid QR target selector')
            if key == '107':
                raise ValueError('second staging must use same-color first-batch base; group4 is not a staging position')
        else:
            ident = target.get('color_id' if target['kind'] == 'material' else 'position_id')
            if type(ident) is not int or not 1 <= ident <= (6 if target['kind'] == 'material' else 3):
                raise ValueError('invalid business target identity')
    sources = {station_calibration(config, s).data['camera_source'] for s in config['stations'].values()}
    if len(sources) != 1:
        raise ValueError('one shared material camera source required')


def validate_source(state, calibration, now, max_age):
    c = calibration
    if (state.get('camera_source') != c.data['camera_source'] or
            state.get('hsv_profile_sha256') != c.data['hsv_profile_sha256'] or
            state.get('coordinate_frame') != 'camera_image' or state.get('coordinate_units') != 'pixels' or
            (state['preview']['source_width'], state['preview']['source_height']) != (c.width, c.height)):
        raise RuntimeError('vision camera/profile/resolution differs from measured calibration')
    capture = state.get('capture', {})
    acquired = capture.get('acquired_monotonic_s')
    if capture.get('valid') is not True or not number(acquired) or not 0 <= now-acquired < max_age:
        raise RuntimeError('material camera capture unavailable or stale')


class ArmAlignmentWork:
    def __init__(self, client, config, snapshot, *, clock=time.monotonic,
                 wait_for_material=False, hold_on_alignment=False, required_stations=range(102, 108)):
        validate_config(config, required_stations)
        self.wait_for_material, self.hold_on_alignment = wait_for_material, hold_on_alignment
        self.client, self.config, self.snapshot, self.clock = client, config, snapshot, clock
        self.context = None
        self.phase = 'idle'
        self.last_tick = None
        self.camera_session = None
        self.results = []

    def preflight(self):
        arm = self.client.arm_status()
        if arm['state_name'] != 'READY' or arm['fault'] or not arm['origins_valid']:
            raise RuntimeError('arm not READY with manually verified origins; never auto-zero: '
                               f"state={arm['state_name']}, fault={arm['fault']}, "
                               f"origins_valid={arm['origins_valid']}")
        state = self.snapshot()
        for station in self.config['stations'].values():
            validate_source(state, station_calibration(self.config, station), self.clock(),
                            self.config['observation_timeout_s'])
            kind = station['target']['kind']
            if state.get('detection', {}).get('materials' if kind == 'material' else 'rings') is not True:
                raise RuntimeError('required material/ring detector not active')
            if kind == 'ring' and state.get('position_identity_enabled') is not True:
                raise RuntimeError('numbered ring identity not active')
        return True

    def start(self, status, qr_result=None):
        if self.phase not in ('idle', 'done') or status['state'] != 3 or status['phase'] != 4:
            raise RuntimeError('arm alignment requires a new stationary WAIT_WORK')
        self.context = tuple(status[k] for k in ('route_id', 'point_index', 'work_id'))
        self.station = self.config['stations'][str(status['work_id'])]
        self.calibration = station_calibration(self.config, self.station)
        self.target = dict(self.station['target'])
        if 'task_group' in self.target:
            if not qr_result or qr_result.get('confirmed') is not True or qr_result.get('route_id') != status['route_id']:
                raise RuntimeError('this route has no confirmed QR task')
            code = parse_competition_task(qr_result['raw'])
            self.target['color_id' if self.target['kind'] == 'material' else 'position_id'] = int(
                getattr(code, self.target['task_group'])[self.target['task_index']])
        self.started_at = self.clock()
        self.deadline = self.started_at + self.config['station_timeout_s']
        self.motion_deadline = self.started_at + self.config['motion_timeout_s']
        self.phase, self.command_id, self.aligned_id = 'preparing', 0, None
        self.last_sequence, self.stable, self.adjustments, self.total_mm = 0, [], 0, 0.
        self.camera_session, self.last_tick = None, None
        self.search_index = 0
        self.acquired_target, self.approach_frames = False, []

    def _command(self, opcode, pose=None):
        self.command_id += 1
        if pose is None:
            self.client.arm_finish(*self.context, self.command_id)
        else:
            motor = self.calibration.motor_pose(pose)
            self.client.arm_align(*self.context, self.command_id,
                                  motor.z_deg, motor.yaw_deg, motor.reach_deg)
        self.motion_deadline = self.clock() + self.config['motion_timeout_s']
        self.phase = 'parking' if opcode == 5 else 'moving'
        self.stable = []

    def _observe_after_stop(self):
        self.floor = self.clock() + self.config['settle_s']
        self.phase, self.stable = 'observing', []

    def next_material(self, color_id):
        if self.phase != 'aligned_hold' or type(color_id) is not int or not 1 <= color_id <= 6:
            raise RuntimeError('next material requires a confirmed alignment and business color')
        self.target = dict(kind='material', color_id=color_id)
        self.aligned_id = None
        self.acquired_target, self.approach_frames = False, []
        self._observe_after_stop()

    def finish(self):
        if self.phase != 'aligned_hold':
            raise RuntimeError('park only after visual alignment')
        self._command(5)

    def _result(self):
        return dict(mode='alignment_only', visual_alignment_confirmed=True,
            physical_grasp_completed=False, physical_place_completed=False,
            route_id=self.context[0], point_index=self.context[1], work_id=self.context[2],
            target=dict(self.target), adjustments=self.adjustments, search_poses_used=self.search_index,
            total_mm=self.total_mm, calibration_id=self.calibration.data['calibration_id'])

    def _observation(self):
        state = self.snapshot()
        c, now = self.calibration, self.clock()
        validate_source(state, c, now, self.config['observation_timeout_s'])
        session = state.get('session_id')
        if not isinstance(session, str) or not session:
            raise RuntimeError('vision session identity missing')
        if self.camera_session is not None and session != self.camera_session:
            raise RuntimeError('vision restarted; old alignment invalid')
        self.camera_session = session
        material = self.target['kind'] == 'material'
        obs = state['material_observation' if material else 'ring_observation']
        acquired, sequence = obs.get('acquired_monotonic_s'), obs.get('source_sequence')
        if obs.get('status') == 'waiting' and not sequence:
            return None
        if obs.get('valid') is not True or obs.get('status') not in ('ok', 'no_target'):
            raise RuntimeError('invalid vision observation: '+str(obs.get('status')))
        if not number(acquired) or not 0 <= now-acquired < self.config['observation_timeout_s']:
            raise RuntimeError('stale vision target')
        if type(sequence) is not int or sequence < 1:
            raise RuntimeError('invalid source sequence')
        if acquired < self.floor or sequence <= self.last_sequence:
            return None
        self.last_sequence = sequence
        if material:
            policy = state.get('material_selection')
            if policy == 'unique_color_in_search_window':
                if self.context[2] not in (102, 105) or state.get('material_search_color') != self.target['color_id']:
                    raise RuntimeError('raw search color/station differs from this task')
            elif policy != 'nearest_window_center':
                raise RuntimeError('explicit HSV window selection required')
            if state.get('material_backend') != 'HSVMaterialDetector' or not state.get('material_target_window'):
                raise RuntimeError('explicit HSV window selection required')
            targets = state['materials']
            if targets and (len(targets) != 1 or targets[0]['color_id'] != self.target['color_id']):
                raise RuntimeError('material result does not match the unique requested color')
            if not targets:
                raise MaterialUnavailable('requested material absent, ambiguous or top face unavailable')
            point = targets[0]
            if point.get('point_kind') != 'top_face_ellipse_center':
                raise RuntimeError('independent top-face center required')
        else:
            if state.get('position_identity_enabled') is not True:
                raise RuntimeError('real position 1/2/3 identity not enabled')
            targets = [r for r in state['rings'] if r.get('position_id') == self.target['position_id']
                       and r.get('position_status') == 'candidate']
            if len(targets) != 1:
                raise RuntimeError('position absent or ambiguous; ring_no is not position_id')
            point = targets[0]
        u, v = point['center_x'], point['center_y']
        if not all(number(x) for x in (u, v)) or not 0 <= u < c.width or not 0 <= v < c.height:
            raise RuntimeError('target outside full image')
        return (u-c.reference[0], v-c.reference[1]), acquired

    def _search_next_pose(self):
        poses = self.station.get('search_poses', [])
        if self.search_index >= len(poses):
            raise MaterialUnavailable('material search poses exhausted; no confirmed top-face target')
        target = MechanismPose(**poses[self.search_index])
        angle = math.radians(target.yaw_deg-self.pose.yaw_deg)
        radius = self.calibration.radius0+self.pose.reach_mm
        next_radius = self.calibration.radius0+target.reach_mm
        distance = math.hypot(next_radius*math.cos(angle)-radius, next_radius*math.sin(angle))
        if (target.z_mm != self.pose.z_mm or abs(math.degrees(angle)) > self.config['max_yaw_step_deg']
                or distance > self.config['max_step_mm'] or next_radius <= 0
                or self.total_mm+distance > self.config['max_total_mm']):
            raise RuntimeError('search pose exceeds fixed-height or measured step/travel limits')
        self.search_index += 1
        self.total_mm += distance
        self._command(2, target)

    def poll(self, status):
        if self.clock() >= self.deadline:
            raise TimeoutError('arm station alignment timeout')
        if tuple(status[k] for k in ('route_id', 'point_index', 'work_id')) != self.context or status['state'] != 3:
            raise RuntimeError('station changed while arm active')
        arm = self.client.arm_status()
        if tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != self.context:
            raise RuntimeError('arm route/point/work context changed')
        if arm['fault'] or not arm['origins_valid'] or arm['state_name'] in (
                'STOPPED', 'FAULT', 'DEBUG_MOVING', 'DEBUG_HOLD'):
            raise RuntimeError('arm fault, debug operation or manual origin invalid: '+arm['state_name'])
        tick = arm['tick']
        if self.last_tick is not None and (tick-self.last_tick) & 0xffffffff >= 0x80000000:
            raise RuntimeError('arm tick regressed; possible MCU reboot')
        self.last_tick = tick
        if self.phase == 'preparing':
            if arm['command_id']:
                raise RuntimeError('existing arm command: inspect rather than repeat after reconnect')
            if arm['state_name'] == 'WAIT_VISION':
                self.pose = self.calibration.mechanism_pose(arm)
                self._observe_after_stop()
            elif arm['state_name'] != 'PREPARING':
                raise RuntimeError('unexpected initial arm state')
        elif self.phase in ('moving', 'parking'):
            if arm['command_id'] != self.command_id:
                raise RuntimeError('another arm command replaced this action')
            done = 'READY_LEAVE' if self.phase == 'parking' else 'ALIGNED'
            if arm['state_name'] == done:
                if self.phase == 'parking':
                    self.phase = 'done'
                    result = self._result()
                    self.results.append(result)
                    return result
                if arm['aligned_command_id'] != self.command_id:
                    raise RuntimeError('aligned command reference mismatch')
                self.aligned_id = self.command_id
                self.pose = self.calibration.mechanism_pose(arm)
                self._observe_after_stop()
            elif self.clock() >= self.motion_deadline:
                raise TimeoutError('arm motion completion timeout')
        elif self.phase == 'observing':
            if arm['state_name'] not in ('WAIT_VISION', 'ALIGNED') or arm['command_id'] != self.command_id:
                raise RuntimeError('arm moved while observing')
            if self.clock() < self.floor:
                return None
            try:
                observation = self._observation()
            except MaterialUnavailable:
                self.stable = []
                self.acquired_target, self.approach_frames = False, []
                if not self.station.get('search_poses'):
                    if self.wait_for_material:
                        return None
                    raise
                if self.clock()-self.floor >= self.station['search_wait_s']:
                    self._search_next_pose()
                return None
            if observation is None:
                return None
            error, acquired = observation
            if self.wait_for_material and not self.acquired_target:
                if self.approach_frames and (acquired <= self.approach_frames[-1] or
                        acquired-self.approach_frames[-1] > self.config['max_frame_gap_s']):
                    self.approach_frames = []
                self.approach_frames.append(acquired)
                if (len(self.approach_frames) < self.config['confirm_frames'] or
                        acquired-self.approach_frames[0] < self.config['confirm_span_s']):
                    return None
                self.acquired_target = True
            if max(abs(v) for v in error) <= self.config['tolerance_px']:
                # Even initially centred targets need an actual ALIGNED state and fresh re-observation.
                if self.aligned_id is None:
                    self._command(2, self.pose)
                    return None
                if self.stable and (acquired <= self.stable[-1] or acquired-self.stable[-1] > self.config['max_frame_gap_s']):
                    self.stable = []
                self.stable.append(acquired)
                if len(self.stable) >= self.config['confirm_frames'] and acquired-self.stable[0] >= self.config['confirm_span_s']:
                    if self.hold_on_alignment:
                        self.phase = 'aligned_hold'
                        return self._result()
                    self._command(5)
            else:
                self.stable = []
                if self.adjustments >= self.config['max_adjustments']:
                    raise RuntimeError('arm adjustment count exhausted')
                target, distance = self.calibration.correct(self.pose, error,
                    self.config['max_step_mm'], self.config['max_yaw_step_deg'])
                if self.total_mm+distance > self.config['max_total_mm']:
                    raise RuntimeError('arm adjustment travel budget exhausted')
                self.adjustments += 1
                self.total_mm += distance
                self._command(2, target)
        if self.phase == 'preparing' and self.clock() >= self.motion_deadline:
            raise TimeoutError('arm photo pose timeout')
        return None

    def close(self):
        # No separate UART/camera owner; the route runner handles STOP on failure.
        pass
