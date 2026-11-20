"""原料区按任务码等待转盘目标，复用云台闭环；夹爪动作另行接入。"""
import time

from pi_vision.arm_alignment import number
from pi_vision.task_confirmation import parse_competition_task
from tools.arm_alignment_work import ArmAlignmentWork

COLORS = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}


class TurntableWork:
    def __init__(self, client, config, snapshot, select_color, publish, *, clock=time.monotonic):
        if config.get('enabled') is not True:
            raise ValueError('turntable work must be explicitly enabled')
        for key in ('station_timeout_s', 'observation_timeout_s', 'confirm_span_s', 'max_frame_gap_s'):
            if not number(config.get(key)) or config[key] <= 0:
                raise ValueError('positive turntable setting required: '+key)
        if type(config.get('confirm_frames')) is not int or config['confirm_frames'] < 3:
            raise ValueError('three distinct approach frames required')
        if not str(config.get('camera_source', '')).startswith('/dev/v4l/by-id/'):
            raise ValueError('explicit material camera identity required')
        self.client, self.config, self.snapshot = client, config, snapshot
        self.select_color, self.publish, self.clock = select_color, publish, clock
        self.base = None
        if config.get('alignment') is not None:
            self.base = ArmAlignmentWork(client, config['alignment'], snapshot, clock=clock,
                wait_for_material=True, hold_on_alignment=True, required_stations=(102, 105))
        self.results = []
        self.context = None
        self.phase = 'idle'

    def _source(self):
        state = self.snapshot()
        now = self.clock()
        capture = state.get('capture', {})
        stamp = capture.get('acquired_monotonic_s')
        if (state.get('camera_source') != self.config['camera_source']
                or state.get('material_backend') != 'HSVMaterialDetector'
                or state.get('coordinate_frame') != 'camera_image'
                or state.get('coordinate_units') != 'pixels'
                or not state.get('material_target_window')
                or capture.get('valid') is not True or not number(stamp)
                or not 0 <= now-stamp < self.config['observation_timeout_s']):
            raise RuntimeError('fresh dedicated HSV material camera required')
        return state

    def preflight(self):
        arm = self.client.arm_status()
        if arm['state_name'] != 'READY' or not arm['origins_valid'] or arm['fault']:
            raise RuntimeError('云台尚未就绪：'+arm['state_name']+
                               '，三轴原点有效='+str(arm['origins_valid'])+'；请先按约定记录原点')
        self._source()
        if self.base is not None:
            self.base.preflight()
        return True

    def _report(self, **details):
        self.publish(dict(phase=self.phase, route_id=self.context[0], point_index=self.context[1],
            work_id=self.context[2], order=self.order, step=len(self.aligned)+1,
            color_id=self.color, color_name=COLORS[self.color],
            aligned_colors=list(self.aligned), calibrated=self.base is not None,
            physical_grasp_completed=False, updated_unix_s=time.time(), **details))

    def start(self, status, qr_result):
        if status['work_id'] not in (102, 105):
            raise RuntimeError('数字圆环作业尚未标定，停止等待接入；禁止模拟作业完成')
        if status['state'] != 3 or status['phase'] != 4:
            raise RuntimeError('turntable work requires stationary WAIT_WORK')
        if not qr_result or qr_result.get('confirmed') is not True or qr_result.get('route_id') != status['route_id']:
            raise RuntimeError('current route QR task required')
        code = parse_competition_task(qr_result['raw'])
        self.order = list(map(int, code.group1 if status['work_id'] == 102 else code.group3))
        self.context = tuple(status[k] for k in ('route_id', 'point_index', 'work_id'))
        self.color, self.aligned = self.order[0], []
        self.deadline = self.clock()+self.config['station_timeout_s']
        self.floor, self.last_sequence, self.samples = self.clock(), 0, []
        self.camera_session = None
        self.phase = 'waiting_target'
        self.select_color(self.color)
        if self.base is not None:
            self.base.config['stations'][str(status['work_id'])]['target'] = dict(kind='material', color_id=self.color)
            self.base.start(status, qr_result)
        self._report()

    def poll(self, status):
        if self.clock() >= self.deadline:
            raise TimeoutError('turntable target/alignment timeout; no work completion')
        if tuple(status[k] for k in ('route_id', 'point_index', 'work_id')) != self.context or status['state'] != 3:
            raise RuntimeError('turntable station context changed')
        if self.base is not None:
            result = self.base.poll(status)
            self.phase = ('parking' if self.base.phase == 'parking' else
                          'aligning' if self.base.acquired_target else 'waiting_target')
            if result is not None:
                if self.base.phase == 'done':
                    self.phase = 'done'
                    result = dict(result, order=self.order, aligned_colors=list(self.aligned))
                    self.results.append(result)
                    self._report()
                    return result
                self.aligned.append(self.color)
                if len(self.aligned) == len(self.order):
                    self.base.finish()
                    self.phase = 'parking'
                else:
                    self.color = self.order[len(self.aligned)]
                    self.select_color(self.color)
                    self.base.next_material(self.color)
                    self.phase = 'waiting_target'
            self._report()
            return None
        # 未标定时仍可在真实到站后等待/观察目标，保持机械臂静止。
        arm = self.client.arm_status()
        if (tuple(arm[k] for k in ('route_id', 'point_index', 'work_id')) != self.context
                or arm['fault'] or not arm['origins_valid']
                or arm['state_name'] not in ('PREPARING', 'WAIT_VISION')):
            raise RuntimeError('arm not available for stationary target observation')
        state = self._source()
        session = state.get('session_id')
        if not session or self.camera_session is not None and session != self.camera_session:
            raise RuntimeError('material camera session changed')
        self.camera_session = session
        obs = state.get('material_observation', {})
        stamp, seq = obs.get('acquired_monotonic_s'), obs.get('source_sequence')
        if obs.get('status') == 'waiting' and not seq:
            self._report(target_visible=False)
            return None
        if (not obs.get('valid') or not number(stamp)
                or not 0 <= self.clock()-stamp < self.config['observation_timeout_s']):
            raise RuntimeError('material observation stale or invalid')
        if state.get('material_search_color') != self.color:
            raise RuntimeError('camera target color differs from QR task')
        if type(seq) is not int or seq < 1:
            raise RuntimeError('source sequence missing')
        if seq <= self.last_sequence or stamp <= self.floor:
            return None
        self.last_sequence = seq
        targets = state.get('materials', [])
        visible = (arm['state_name'] == 'WAIT_VISION' and len(targets) == 1
            and targets[0]['color_id'] == self.color and targets[0]['point_kind'] == 'top_face_ellipse_center')
        if not visible:
            self.samples = []
            self.phase = 'waiting_target'
        else:
            if self.samples and stamp-self.samples[-1] > self.config['max_frame_gap_s']:
                self.samples = []
            self.samples.append(stamp)
            self.samples = self.samples[-self.config['confirm_frames']:]
            if (len(self.samples) >= self.config['confirm_frames']
                    and stamp-self.samples[0] >= self.config['confirm_span_s']):
                self.phase = 'calibration_required'
        self._report(localization=state.get('material_localization_status'), target_visible=visible)
        return None

    def close(self):
        if self.context is not None and self.phase != 'done':
            self.phase = 'stopped'
            self._report()
