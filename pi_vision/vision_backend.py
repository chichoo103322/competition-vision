"""Request-correlated real perception adapter. No ROS or hardware imports.

The source is the existing FrameStreamer; the adapter never opens a device.
Only raw-area material observations and QR candidates are supported for now.
"""

from copy import deepcopy
import math
import time


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class VisionBackend:
    def __init__(self, source, camera_id, *, wall_clock=time.time, clock=time.monotonic):
        self.source = source
        self.camera_id = camera_id
        self.wall_clock, self.clock = wall_clock, clock
        self.pending = None
        self.deadline = 0.
        self.seen = set()
        self.last_issued = -1.
        self.last_source = None
        self._position_confirmation = None

    @staticmethod
    def reply(command, status, data):
        return {key: command[key] for key in
                ('schema_version', 'simulation', 'run_id', 'command_id')} | {
                    'status': status, 'data': data}

    def cancel(self):
        self.pending = None
        self._position_confirmation = None
        self.source.set_scan_detection(None)

    def command(self, command):
        """Reject replay/simulation before changing modes. Never ACK robot stop."""
        if not isinstance(command, dict):
            return []
        if command.get('operation') not in ('read_task', 'scan_material', 'locate_position',
                'display_task', 'navigate', 'pick_to_tray', 'place_from_tray', 'report_counts', 'stop'):
            return []
        issued, timeout = command.get('issued_at_unix_s'), command.get('timeout_s')
        now = self.wall_clock()
        if (type(command.get('schema_version')) is not int or command['schema_version'] != 1
                or command.get('simulation') is not False
                or any(not isinstance(command.get(k), str) or not command[k] or len(command[k]) > 256
                       for k in ('run_id', 'command_id', 'operation'))
                or not isinstance(command.get('params'), dict)
                or not finite(issued) or not finite(timeout) or not 0 < timeout <= 3600
                or not 0 <= now-issued < timeout or issued < self.last_issued):
            return []
        key = (command['run_id'], command['command_id'])
        if key in self.seen:
            return []
        if len(self.seen) >= 4096:
            self.cancel()
            return [self.reply(command, 'failed', {'reason': 'deduplication_limit_restart_required'})]
        self.seen.add(key)
        self.last_issued = issued
        self.cancel()
        operation, params = command['operation'], command['params']
        if operation not in ('read_task', 'scan_material', 'locate_position'):
            return []  # Other backends own navigation, stop and physical actions.
        if operation == 'scan_material' and (params.get('zone') != 'raw' or 'position_id' in params):
            return [self.reply(command, 'failed', {'reason': 'position_identity_not_implemented'})]
        if operation == 'locate_position':
            if self.source.get_state().get('position_identity_enabled') is not True:
                return [self.reply(command, 'failed', {'reason': 'position_identity_not_implemented'})]
            if (type(params.get('position_id')) is not int or not 1 <= params['position_id'] <= 3
                    or params.get('zone') not in ('rough', 'staging')
                    or type(params.get('layer', 0)) is not int or params.get('layer', 0) not in (0, 1)):
                return [self.reply(command, 'failed', {'reason': 'invalid_position_request'})]
            if params.get('layer') == 1:
                return [self.reply(command, 'failed', {'reason': 'stack_base_verification_not_implemented'})]
        if operation == 'scan_material' and (
                type(params.get('color_id')) is not int or not 1 <= params['color_id'] <= 6):
            return [self.reply(command, 'failed', {'reason': 'invalid_business_color'})]
        self.pending = deepcopy(command)
        self.deadline = self.clock() + timeout-(now-issued)
        self.last_source = None
        # Existing scan constants are QR=0 and material=1; no serial port is opened.
        self.source.set_scan_detection(0 if operation == 'read_task' else 2 if operation == 'locate_position' else 1)
        return [self.reply(command, 'accepted', {'backend': 'real_camera_vision'})]

    def poll(self):
        command = self.pending
        if command is None:
            return []
        if self.clock() >= self.deadline:
            self.cancel()
            return [self.reply(command, 'failed', {'reason': 'vision_request_timeout'})]
        state = self.source.get_state()
        qr = command['operation'] == 'read_task'
        position = command['operation'] == 'locate_position'
        obs = state['qr_observation' if qr else 'ring_observation' if position else 'material_observation']
        if obs['status'] in ('error', 'camera_error', 'camera_stale', 'camera_stopped', 'timeout'):
            self.cancel()
            return [self.reply(command, 'failed', {'reason': obs['status'], 'error': obs.get('error')})]
        acquired = obs.get('acquired_unix_s')
        sequence = obs.get('source_sequence')
        key = (state['session_id'], sequence)
        if (obs.get('valid') is not True or obs['status'] not in ('ok', 'no_target')
                or not finite(acquired) or not command['issued_at_unix_s'] <= acquired <= self.wall_clock()
                or type(sequence) is not int or sequence <= 0
                or self.last_source is not None and key[0] == self.last_source[0]
                   and sequence <= self.last_source[1]):
            if position and obs.get('valid') is not True:
                self._position_confirmation = None
            return []
        self.last_source = key
        data = {'status': obs['status'], 'valid': True, 'acquired_unix_s': acquired,
                'source_sequence': sequence, 'source_session_id': state['session_id'],
                'camera_id': self.camera_id, 'camera_role': 'qr' if qr else 'ring' if position else 'material'}
        if qr:
            # Candidate belongs to this atomic, fresh observation. Never use latched task.
            data['task_code'] = state['qr_candidate']
            data['status'] = 'ok' if data['task_code'] else 'no_target'
        elif position:
            targets, reason = self._position_targets(state, acquired)
            data.update(status='ok' if targets else 'no_target', targets=targets, position_reason=reason,
                        zone=command['params']['zone'], coordinate_frame='camera_image', units='pixels',
                        width=state['preview']['source_width'], height=state['preview']['source_height'],
                        pixel_origin='top_left',
                        metric_coordinates_valid=False, calibration_required=True,
                        coordinate_semantics='concentric_ring_center_candidate_not_grasp_point',
                        identity_method='concentric_ring_center_rotation_aware_digit_template',
                        hsv_profile_sha256=state['hsv_profile_sha256'])
        else:
            targets = []
            for item in state['materials']:
                if item['color_id'] != command['params']['color_id']:
                    continue
                target = {'u': item['center_x'], 'v': item['center_y'], 'color_id': item['color_id'],
                          'confidence': item.get('confidence'),
                          'point_kind': item.get('point_kind', 'contour_bounding_box_center')}
                if type(item.get('track_id')) is int:
                    target.update(track_id=item['track_id'],
                                  tracking_status=item.get('tracking_status', 'updated'))
                targets.append(target)
            data.update(status='ok' if targets else 'no_target', targets=targets,
                        zone=command['params']['zone'], coordinate_frame='camera_image', units='pixels',
                        width=state['preview']['source_width'], height=state['preview']['source_height'],
                        pixel_origin='top_left',
                        metric_coordinates_valid=False, calibration_required=True,
                        coordinate_semantics=('full_frame_contour_box_center_not_grasp_point'
                            if all(t['point_kind'] == 'contour_bounding_box_center' for t in targets)
                            else 'full_frame_see_target_point_kind_not_grasp_point'),
                        hsv_profile_sha256=state['hsv_profile_sha256'])
        return [self.reply(command, 'observation', data)]

    def _position_targets(self, state, acquired):
        position_id = self.pending['params']['position_id']
        rings = state['rings']
        known = [r.get('position_id') for r in rings if r.get('position_id') is not None]
        matches = [r for r in rings if type(r.get('position_id')) is int and r['position_id'] == position_id
                   and r.get('position_status') == 'candidate']
        if len(matches) != 1 or len(known) != len(set(known)):
            self._position_confirmation = None
            return [], 'position_uncertain'
        ring = matches[0]
        width, height = state['preview']['source_width'], state['preview']['source_height']
        if (not finite(ring.get('center_x')) or not finite(ring.get('center_y'))
                or not 0 <= ring['center_x'] < width or not 0 <= ring['center_y'] < height
                or ring.get('point_kind') != 'ring_candidate_center'):
            self._position_confirmation = None
            return [], 'position_uncertain'
        previous = self._position_confirmation
        if (previous is None or previous['session'] != state['session_id']
                or not 0 < acquired-previous['last'] <= 1.):
            previous = dict(session=state['session_id'], first=acquired, count=0)
        previous.update(last=acquired, count=previous['count']+1)
        self._position_confirmation = previous
        if previous['count'] < 3 or acquired-previous['first'] < .2:
            return [], 'position_unconfirmed'
        return [dict(position_id=position_id, u=ring['center_x'], v=ring['center_y'],
                     radius_px=ring['radius'], digit_orientation_deg=ring['digit_orientation_deg'],
                     center_method=ring.get('center_method', 'legacy_ring_fit'),
                     point_kind='ring_candidate_center', digit_score=ring['digit_score'],
                     digit_margin=ring['digit_margin'])], 'confirmed_new_frames'
