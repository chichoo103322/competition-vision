"""Bounded observe/adjust/stop/re-observe coordinator; no hardware imports.

This is a separate controller, not an automatic change to the mission planner.
Correction matrices must be measured for the exact camera pose and target plane.
"""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math
import time
import uuid


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class AlignmentCalibration:
    calibration_id: str
    camera_id: str
    pose_id: str
    plane_id: str
    point_kind: str
    hsv_profile_sha256: str
    width: int
    height: int
    reference_pixel: tuple
    correction_matrix_mm_per_pixel: tuple
    simulation_only: bool

    def __post_init__(self):
        if any(not isinstance(getattr(self, k), str) or not getattr(self, k).strip()
               for k in ('calibration_id', 'camera_id', 'pose_id', 'plane_id', 'point_kind', 'hsv_profile_sha256')):
            raise ValueError('calibration identity/pose/plane/profile fields required')
        if (type(self.simulation_only) is not bool
                or any(type(v) is not int or not 1 <= v <= 65535 for v in (self.width, self.height))):
            raise ValueError('invalid calibration mode or resolution')
        ref, matrix = self.reference_pixel, self.correction_matrix_mm_per_pixel
        if (len(ref) != 2 or not all(finite(v) for v in ref)
                or not 0 <= ref[0] < self.width or not 0 <= ref[1] < self.height
                or len(matrix) != 2 or any(len(row) != 2 for row in matrix)
                or not all(finite(v) for row in matrix for v in row)):
            raise ValueError('invalid reference pixel or correction matrix')
        determinant = matrix[0][0]*matrix[1][1]-matrix[0][1]*matrix[1][0]
        if not math.isfinite(determinant) or abs(determinant) < 1e-12:
            raise ValueError('singular correction matrix')
        object.__setattr__(self, 'reference_pixel', tuple(ref))
        object.__setattr__(self, 'correction_matrix_mm_per_pixel', tuple(tuple(row) for row in matrix))


class AlignmentController:
    def __init__(self, calibration=None, *, simulation=False, tolerance_px=3.,
                 confirm_frames=3, confirm_span_s=.2, max_frame_gap_s=.7,
                 max_step_mm=10., max_total_mm=60., max_adjustments=8,
                 observation_timeout_s=1., request_timeout_s=4., total_timeout_s=30.,
                 settle_s=.1, clock=time.monotonic, wall_clock=time.time):
        values = (tolerance_px, confirm_span_s, max_frame_gap_s, max_step_mm,
                  max_total_mm, observation_timeout_s, request_timeout_s, total_timeout_s)
        if (type(simulation) is not bool or any(not finite(v) or v <= 0 for v in values)
                or not finite(settle_s) or settle_s < 0
                or type(confirm_frames) is not int or confirm_frames < 2
                or type(max_adjustments) is not int or max_adjustments < 1):
            raise ValueError('invalid alignment limits')
        self.calibration, self.simulation = calibration, simulation
        self.tolerance_px, self.confirm_frames = tolerance_px, confirm_frames
        self.confirm_span_s, self.max_frame_gap_s = confirm_span_s, max_frame_gap_s
        self.max_step_mm, self.max_total_mm = max_step_mm, max_total_mm
        self.max_adjustments = max_adjustments
        self.observation_timeout_s = observation_timeout_s
        self.request_timeout_s, self.total_timeout_s = request_timeout_s, total_timeout_s
        self.settle_s, self.clock, self.wall_clock = settle_s, clock, wall_clock
        self.phase, self.error = 'idle', None
        self.pending = None
        self.run_id = ''
        self.stop_confirmed = True
        self.target = None
        self.outbox = deque()
        self.events = deque(maxlen=128)
        self._source = None
        self._sequences = {}
        self._stable = []
        self.adjustments, self.total_mm = 0, 0.

    def _event(self, kind, **data):
        self.events.append(dict(event=kind, at_monotonic_s=self.clock(), **data))

    def _send(self, channel, operation, params):
        command = dict(schema_version=1, simulation=self.simulation, run_id=self.run_id,
                       command_id=uuid.uuid4().hex, operation=operation, params=deepcopy(params),
                       issued_at_unix_s=self.wall_clock(), timeout_s=self.request_timeout_s)
        self.outbox.append(dict(channel=channel, command=command))
        return command

    def _issue(self, channel, operation, params):
        self.pending = self._send(channel, operation, params)
        self._pending_channel = channel
        self._deadline = self.clock()+self.request_timeout_s
        self._event('request', channel=channel, operation=operation)

    def drain_commands(self):
        commands = list(self.outbox)
        self.outbox.clear()
        return commands

    def start(self, color_id, *, zone='raw'):
        if self.phase not in ('idle', 'aligned', 'expired', 'fault', 'cancelled') or not self.stop_confirmed:
            raise ValueError('alignment active or physical stop unconfirmed')
        c = self.calibration
        if c is None or c.simulation_only is not self.simulation:
            raise ValueError('matching measured calibration required; simulation calibration cannot move hardware')
        if type(color_id) is not int or not 1 <= color_id <= 6 or zone != 'raw':
            raise ValueError('only raw-area business colors 1..6 are supported')
        self.run_id = uuid.uuid4().hex
        self.color_id, self.zone = color_id, zone
        self.phase, self.error, self.target = 'positioning', None, None
        self.stop_confirmed = False
        self.adjustments, self.total_mm = 0, 0.
        self.outbox.clear()
        self._source, self._sequences, self._stable = None, {}, []
        self._total_deadline = self.clock()+self.total_timeout_s
        self._issue('motion', 'hold_observation_pose', self._identity())

    def _identity(self):
        c = self.calibration
        return dict(calibration_id=c.calibration_id, pose_id=c.pose_id, plane_id=c.plane_id,
                    requires_calibrated_backend=True)

    def _observe(self):
        self.phase = 'observing'
        self._stable = []
        self._issue('vision', 'scan_material', dict(zone=self.zone, color_id=self.color_id))

    def _stop(self, reason, terminal='fault'):
        if self.phase == 'stopping':
            return
        self.phase, self.error, self.target = 'stopping', reason, None
        self.stop_confirmed = False
        self._terminal = terminal
        self.outbox.clear()
        self._send('vision', 'stop', {'reason': reason})
        self._issue('motion', 'stop', {'reason': reason, 'cancel_all': True})

    def cancel(self):
        if self.phase in ('positioning', 'settling', 'observing', 'moving'):
            self._stop('cancelled_by_request', 'cancelled')

    def tick(self):
        now = self.clock()
        if self.phase == 'aligned':
            if not 0 <= self.wall_clock()-self.target['acquired_unix_s'] < self.observation_timeout_s:
                self.phase, self.target = 'expired', None
            return
        if self.phase in ('idle', 'fault', 'cancelled', 'expired'):
            return
        if self.phase != 'stopping' and now >= self._total_deadline:
            self._stop('alignment_total_timeout')
        elif self.pending is not None and now >= self._deadline:
            if self.phase == 'stopping':
                self.phase, self.pending = 'fault', None
                self.error += ':stop_unconfirmed'
            else:
                self._stop('request_timeout:'+self.pending['operation'])
        elif self.phase == 'settling' and now >= self._settled_at:
            self._observe()

    def feedback(self, channel, message):
        self.tick()
        p = self.pending
        if (p is None or channel != self._pending_channel or not isinstance(message, dict)
                or type(message.get('schema_version')) is not int or message['schema_version'] != 1
                or message.get('simulation') is not self.simulation
                or any(message.get(k) != p[k] for k in ('run_id', 'command_id'))):
            return False
        status, data = message.get('status'), message.get('data')
        if status == 'accepted':
            return False
        if not isinstance(data, dict):
            return False
        if self.phase == 'stopping':
            if status == 'done' and data.get('stopped') is True:
                self.phase, self.pending, self.stop_confirmed = self._terminal, None, True
                return True
            return False
        if status == 'failed':
            self._stop('backend_failed:'+p['operation'])
            return True
        if channel == 'motion':
            if status != 'done':
                return False
            c = self.calibration
            if (data.get('stopped') is not True or data.get('pose_id') != c.pose_id
                    or data.get('calibration_id') != c.calibration_id
                    or p['operation'] == 'adjust_xy' and data.get('adjusted') is not True):
                self._stop('motion_completion_not_verified')
                return True
            self.pending = None
            self.stop_confirmed = True
            self.phase = 'settling'
            self._settled_at = self.clock()+self.settle_s
            # Re-observe only frames captured after receipt of stopped feedback + settle.
            self._capture_floor = self.wall_clock()+self.settle_s
            return True
        if status != 'observation':
            return False
        return self._observation(data)

    def _observation(self, data):
        c = self.calibration
        if data.get('status') in ('camera_error', 'error'):
            self._stop('camera_fault')
            return True
        acquired, seq = data.get('acquired_unix_s'), data.get('source_sequence')
        source = (data.get('camera_id'), data.get('source_session_id'))
        if (data.get('valid') is not True or data.get('status') not in ('ok', 'no_target')
                or not finite(acquired) or not max(self.pending['issued_at_unix_s'], self._capture_floor)
                   <= acquired <= self.wall_clock()
                or self.wall_clock()-acquired >= self.observation_timeout_s
                or type(seq) is not int or seq <= 0
                or not isinstance(source[1], str) or not source[1]
                or seq <= self._sequences.get(source, 0)):
            self._stable = []
            return False
        if (source[0] != c.camera_id or data.get('camera_role') != 'material'
                or data.get('coordinate_frame') != 'camera_image' or data.get('units') != 'pixels'
                or type(data.get('width')) is not int or data['width'] != c.width
                or type(data.get('height')) is not int or data['height'] != c.height
                or data.get('hsv_profile_sha256') != c.hsv_profile_sha256 or data.get('zone') != self.zone):
            self._stop('calibration_observation_mismatch')
            return True
        if self._source is not None and source != self._source:
            self._stop('camera_session_changed')
            return True
        self._source = source
        self._sequences[source] = seq
        targets = data.get('targets')
        if data['status'] == 'no_target' or not isinstance(targets, list):
            self._stop('target_lost')
            return True
        matches = [t for t in targets if isinstance(t, dict) and type(t.get('color_id')) is int
                   and t['color_id'] == self.color_id]
        if len(matches) != 1:
            self._stop('target_ambiguous_or_missing')
            return True
        t = matches[0]
        if (t.get('point_kind') != c.point_kind or not finite(t.get('u')) or not finite(t.get('v'))
                or not 0 <= t['u'] < c.width or not 0 <= t['v'] < c.height):
            self._stop('invalid_target_point')
            return True
        error = (t['u']-c.reference_pixel[0], t['v']-c.reference_pixel[1])
        if max(abs(v) for v in error) <= self.tolerance_px:
            if self._stable and (acquired <= self._stable[-1] or acquired-self._stable[-1] > self.max_frame_gap_s):
                self._stable = []
            if len(self._stable) < self.confirm_frames:
                self._stable.append(acquired)
            else:
                self._stable[-1] = acquired  # Keep first time: high FPS must still satisfy span.
            if (len(self._stable) >= self.confirm_frames
                    and self._stable[-1]-self._stable[0] >= self.confirm_span_s):
                self.target = deepcopy(data)
                self.phase, self.pending = 'aligned', None
                self.outbox.clear()
                self._send('vision', 'stop', {'reason': 'alignment_confirmed'})
                self._event('alignment_confirmed', pixel_error=error)
            return True
        self._stable = []
        if self.adjustments >= self.max_adjustments:
            self._stop('adjustment_limit')
            return True
        matrix = c.correction_matrix_mm_per_pixel
        delta = [sum(row[i]*error[i] for i in (0, 1)) for row in matrix]
        length = math.hypot(*delta)
        if not math.isfinite(length) or length <= 0:
            self._stop('invalid_correction')
            return True
        scale = min(1., self.max_step_mm/length)
        delta = [v*scale for v in delta]
        length *= scale
        if self.total_mm+length > self.max_total_mm:
            self._stop('travel_budget_exceeded')
            return True
        self.adjustments += 1
        self.total_mm += length
        self.phase, self.stop_confirmed = 'moving', False
        self._send('vision', 'stop', {'reason': 'adjusting'})
        self._issue('motion', 'adjust_xy', dict(self._identity(), dx_mm=delta[0], dy_mm=delta[1],
                    coordinate_frame='calibrated_motion_xy', units='mm',
                    source=deepcopy(data), target_expires_at_unix_s=acquired+self.observation_timeout_s))
        return True

    def snapshot(self):
        self.tick()
        return dict(schema_version=1, simulation=self.simulation, run_id=self.run_id,
                    phase=self.phase, error=self.error, stop_confirmed=self.stop_confirmed,
                    adjustments=self.adjustments, total_requested_mm=self.total_mm,
                    stable_frames=len(self._stable), target=deepcopy(self.target),
                    target_expires_at_unix_s=self.target['acquired_unix_s']+self.observation_timeout_s
                    if self.target else None,
                    pending=deepcopy(self.pending))
