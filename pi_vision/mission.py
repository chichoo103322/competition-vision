"""Headless preliminary-round task coordinator; no camera/serial/motor imports.

Actions describe intent, never measured distances or successful physical work.
Exactly one action is in flight. Only correlated completion feedback advances it.
"""

from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass
from enum import Enum
import math
import time
import uuid

from .task_confirmation import TaskCodeConfirmation, parse_competition_task


class Phase(str, Enum):
    IDLE = 'idle'
    READING_TASK = 'reading_task'
    RUNNING = 'running'
    STOPPING = 'stopping'
    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    FAULT = 'fault'


@dataclass(frozen=True)
class Step:
    operation: str
    params: dict


@dataclass(frozen=True)
class Command:
    run_id: str
    command_id: str
    step_index: int
    operation: str
    params: dict
    issued_at_unix_s: float
    timeout_s: float
    simulation: bool
    schema_version: int = 1

    def to_dict(self):
        return deepcopy(asdict(self))


def _integer(value, low, high):
    return type(value) is int and low <= value <= high


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def build_plan(task_code, capacity=1, home_id='start'):
    """All rough placements finish before rough re-picking; cargo capacity 1..3.

    home_id/zone/position are symbolic goals, NOT map coordinates. The actual
    navigation/arm backends must resolve them and reject uncalibrated commands.
    """
    task = parse_competition_task(str(task_code))
    if not _integer(capacity, 1, 3):
        raise ValueError('capacity must be 1, 2 or 3')
    if not isinstance(home_id, str) or not home_id.strip():
        raise ValueError('home_id must identify the drawn start/stop area')
    first_positions = dict(zip(map(int, task.group1), map(int, task.group2)))
    steps = [Step('display_task', {'task_code': str(task)})]

    def navigate(zone, batch):
        steps.append(Step('navigate', {'zone': zone, 'batch': batch}))

    def pick(job, zone, slot):
        params = dict(job, zone=zone, tray_slot=slot)
        steps.extend((Step('scan_material', params), Step('pick_to_tray', params)))

    def place(job, zone, slot, position, layer):
        params = dict(job, zone=zone, tray_slot=slot, position_id=position, layer=layer)
        steps.extend((Step('locate_position', params), Step('place_from_tray', params)))

    for batch, colors, positions in ((1, task.group1, task.group2),
                                     (2, task.group3, task.group4)):
        jobs = [dict(batch=batch, order=i+1, color_id=int(color), rough_position=int(pos))
                for i, (color, pos) in enumerate(zip(colors, positions))]
        chunks = [jobs[i:i+capacity] for i in range(0, 3, capacity)]
        for chunk in chunks:
            navigate('raw', batch)
            for slot, job in enumerate(chunk, 1):
                pick(job, 'raw', slot)
            navigate('rough', batch)
            for slot, job in enumerate(chunk, 1):
                place(job, 'rough', slot, job['rough_position'], 0)
        for chunk in chunks:
            navigate('rough', batch)
            for slot, job in enumerate(chunk, 1):
                pick(dict(job, position_id=job['rough_position']), 'rough', slot)
            navigate('staging', batch)
            for slot, job in enumerate(chunk, 1):
                place(job, 'staging', slot, first_positions[job['color_id']], batch-1)
    steps.extend((Step('navigate', {'zone': 'home', 'home_id': home_id}),
                  Step('report_counts', {})))
    return tuple(steps)


class TaskCoordinator:
    """Single-threaded event-driven coordinator, reusable from CLI and ROS 2.

    ROS feedback acquisition timestamps are Unix seconds. Hosts must synchronize
    clocks. Action deadlines use a local monotonic clock; neither image pixel
    coordinates nor the fixed legacy distance become arm/motor setpoints here.
    """

    DEFAULT_TIMEOUTS = {'read_task': 10., 'display_task': 3., 'navigate': 30.,
                        'scan_material': 5., 'locate_position': 5.,
                        'pick_to_tray': 15., 'place_from_tray': 15.,
                        'report_counts': 3., 'stop': 2.}
    PERCEPTION = {'read_task', 'scan_material', 'locate_position'}

    def __init__(self, *, capacity=1, simulation=False, observation_timeout=1.,
                 timeouts=None, clock=time.monotonic, wall_clock=time.time):
        if not _integer(capacity, 1, 3) or type(simulation) is not bool:
            raise ValueError('invalid capacity or simulation flag')
        if not _finite(observation_timeout) or observation_timeout <= 0:
            raise ValueError('observation_timeout must be finite and positive')
        self.timeouts = dict(self.DEFAULT_TIMEOUTS)
        if timeouts:
            if set(timeouts) - set(self.timeouts):
                raise ValueError('unknown operation timeout')
            self.timeouts.update(timeouts)
        if any(not _finite(t) or t <= 0 for t in self.timeouts.values()):
            raise ValueError('timeouts must be finite and positive')
        self.capacity, self.simulation = capacity, simulation
        self.observation_timeout = observation_timeout
        self.clock, self.wall_clock = clock, wall_clock
        self.phase = Phase.IDLE
        self.run_id = ''
        self.task = None
        self.plan = ()
        self.step_index = -1
        self.pending = None
        self.tray = {}
        self.counts = {'picked': 0, 'placed': 0, 'stacked': 0}
        self.error = None
        self.stop_confirmed = True
        self._outbox = deque()
        self.events = deque(maxlen=256)
        self._source_sequences = {}
        self._camera_sessions = {}
        self._target = None
        self._qr = TaskCodeConfirmation()
        self._terminal_after_stop = Phase.FAULT

    def _event(self, event, **data):
        self.events.append({'event': event, 'at_monotonic_s': self.clock(), **data})

    def start(self, home_id='start'):
        if self.phase not in (Phase.IDLE, Phase.COMPLETED, Phase.CANCELLED, Phase.FAULT):
            raise ValueError('a mission is already active')
        if not self.stop_confirmed:
            raise ValueError('stop is not confirmed; reconcile the robot before restarting')
        if not isinstance(home_id, str) or not home_id.strip():
            raise ValueError('home_id is required')
        self.run_id = uuid.uuid4().hex
        self.home_id = home_id
        self.phase = Phase.READING_TASK
        self.task, self.plan, self.step_index = None, (), -1
        self.tray = {}
        self.counts = {'picked': 0, 'placed': 0, 'stacked': 0}
        self.error, self._target = None, None
        self._source_sequences.clear()
        self._camera_sessions.clear()
        self._outbox.clear()
        self._qr.reset()
        self._event('started', run_id=self.run_id)
        self._issue(Step('read_task', {'camera_role': 'qr'}))

    def _issue(self, step):
        params = deepcopy(step.params)
        if step.operation in ('pick_to_tray', 'place_from_tray'):
            target = self._target
            age = self.wall_clock() - target['acquired_unix_s'] if target else math.inf
            if not 0 <= age < self.observation_timeout:
                self.fail('target_expired_before_action')
                return
            params['target'] = deepcopy(target)
            params['requires_calibrated_backend'] = True
            params['target_expires_at_unix_s'] = target['acquired_unix_s'] + self.observation_timeout
        if step.operation == 'report_counts':
            params.update(self.counts)
        self.pending = Command(self.run_id, uuid.uuid4().hex, self.step_index,
                               step.operation, params, self.wall_clock(),
                               self.timeouts[step.operation], self.simulation)
        self._deadline = self.clock() + self.pending.timeout_s
        self._outbox.append(self.pending)
        self._event('command', operation=step.operation, command_id=self.pending.command_id)

    def drain_commands(self):
        commands = [command.to_dict() for command in self._outbox]
        self._outbox.clear()
        return commands

    def tick(self):
        if self.pending is None or self.phase in (Phase.FAULT, Phase.CANCELLED, Phase.COMPLETED):
            return
        if self.clock() >= self._deadline:
            if self.pending.operation == 'stop':
                self.error = (self.error or 'cancelled') + ':stop_unconfirmed'
                self.phase = Phase.FAULT
                self._event('stop_timeout')
            else:
                self.fail('timeout:' + self.pending.operation)

    def _stop(self, reason, terminal):
        self.error = reason
        self.phase = Phase.STOPPING
        self.stop_confirmed = False
        self._terminal_after_stop = terminal
        self._target = None
        self._outbox.clear()  # Never dispatch an unsent old motion command after stop.
        self._issue(Step('stop', {'reason': reason, 'cancel_all': True}))

    def fail(self, reason):
        if self.phase not in (Phase.STOPPING, Phase.FAULT, Phase.CANCELLED, Phase.COMPLETED):
            self._stop(str(reason), Phase.FAULT)

    def cancel(self):
        if self.phase in (Phase.READING_TASK, Phase.RUNNING):
            self._stop('cancelled_by_request', Phase.CANCELLED)

    def _advance(self):
        self.step_index += 1
        if self.step_index == len(self.plan):
            self.pending = None
            self.phase = Phase.COMPLETED
            self._event('completed', counts=dict(self.counts))
        else:
            self._issue(self.plan[self.step_index])

    def _sample(self, data):
        if not isinstance(data, dict):
            return None
        if data.get('status') in ('camera_error', 'error'):
            self.fail('perception_fault:' + data['status'])
            return None
        acquired, sequence = data.get('acquired_unix_s'), data.get('source_sequence')
        session, camera = data.get('source_session_id'), data.get('camera_id')
        now = self.wall_clock()
        role = 'qr' if self.pending.operation == 'read_task' else 'material'
        if (data.get('valid') is not True or not _finite(acquired)
                or data.get('status') not in ('ok', 'no_target')
                or data.get('camera_role') != role
                or not self.pending.issued_at_unix_s <= acquired <= now
                or now-acquired >= self.observation_timeout
                or not _integer(sequence, 1, 2**63-1)
                or not isinstance(session, str) or not session
                or not isinstance(camera, str) or not camera):
            return None
        source = (camera, session)
        if sequence <= self._source_sequences.get(source, 0):
            return None
        if camera in self._camera_sessions and self._camera_sessions[camera] != session:
            self._qr.reset()
        self._camera_sessions[camera] = session
        self._source_sequences[source] = sequence
        return data

    def _observation(self, data):
        sample = self._sample(data)
        if sample is None:
            if isinstance(data, dict) and data.get('valid') is not True:
                self._qr.reset()
            return False
        if self.pending.operation == 'read_task':
            raw = sample.get('task_code') if sample['status'] == 'ok' else None
            try:
                code = parse_competition_task(raw) if isinstance(raw, str) else None
            except ValueError:
                code = None
            try:
                locked = self._qr.update(code, sample['acquired_unix_s'])
            except ValueError:
                self._qr.reset()
                return False
            if locked is not None:
                self.task = locked
                self.plan = build_plan(locked, self.capacity, self.home_id)
                self.phase = Phase.RUNNING
                self._event('task_locked', task_code=str(locked))
                self._advance()
            return locked is not None
        params = self.pending.params
        if sample['status'] == 'no_target':
            return False
        key = 'color_id' if self.pending.operation == 'scan_material' else 'position_id'
        expected = params[key]
        targets = sample.get('targets')
        if not isinstance(targets, list):
            return False
        matches = [t for t in targets if isinstance(t, dict) and type(t.get(key)) is int
                   and t[key] == expected]
        if len(matches) != 1:
            return False
        target = matches[0]
        if (sample.get('coordinate_frame') != 'camera_image' or sample.get('units') != 'pixels'
                or not _integer(sample.get('width'), 1, 65535)
                or not _integer(sample.get('height'), 1, 65535)
                or not _finite(target.get('u')) or not _finite(target.get('v'))
                or not 0 <= target['u'] < sample['width']
                or not 0 <= target['v'] < sample['height']
                or sample.get('zone') != params['zone']):
            return False
        if 'position_id' in params and (type(target.get('position_id')) is not int
                                       or target['position_id'] != params['position_id']):
            return False
        if params.get('layer') == 1 and (type(target.get('base_color_id')) is not int
                                       or target['base_color_id'] != params['color_id']):
            return False
        if not isinstance(target.get('point_kind'), str) or not target['point_kind']:
            return False
        self._target = {k: deepcopy(sample[k]) for k in
                        ('acquired_unix_s', 'source_sequence', 'source_session_id',
                         'camera_id', 'coordinate_frame', 'units', 'width', 'height')}
        self._target['point'] = deepcopy(target)
        self._advance()
        return True

    def feedback(self, message):
        """True only when a correlated result changes progress, not an ACK.

        Late stop acknowledgements may confirm safety, but no other late result
        advances the plan. Simulation and real feedback cannot be mixed.
        """
        self.tick()
        command = self.pending
        if (not isinstance(message, dict) or command is None
                or not _integer(message.get('schema_version'), 1, 1)
                or message.get('simulation') is not self.simulation
                or message.get('run_id') != self.run_id
                or message.get('command_id') != command.command_id):
            return False
        status, data = message.get('status'), message.get('data', {})
        if command.operation == 'stop':
            if status == 'done' and isinstance(data, dict) and data.get('stopped') is True:
                self.stop_confirmed = True
                self.pending = None
                self.phase = self._terminal_after_stop
                self._event('stop_confirmed')
                return True
            return False
        if self.phase not in (Phase.READING_TASK, Phase.RUNNING):
            return False
        if status == 'failed':
            self.fail('backend_failed:' + command.operation)
            return True
        if command.operation in self.PERCEPTION:
            return self._observation(data) if status == 'observation' else False
        if status != 'done' or not isinstance(data, dict):
            return False
        required = {'display_task': ('displayed',), 'navigate': ('arrived', 'stopped'),
                    'pick_to_tray': ('grasped', 'loaded'),
                    'place_from_tray': ('released', 'placement_verified'),
                    'report_counts': ('displayed',)}[command.operation]
        if any(data.get(flag) is not True for flag in required):
            self.fail('completion_not_verified:' + command.operation)
            return True
        params = command.params
        if command.operation == 'pick_to_tray':
            slot = params['tray_slot']
            if slot in self.tray or len(self.tray) >= self.capacity:
                self.fail('cargo_state_conflict')
                return True
            self.tray[slot] = {'color_id': params['color_id'], 'batch': params['batch']}
            self.counts['picked'] += 1
        elif command.operation == 'place_from_tray':
            cargo = self.tray.get(params['tray_slot'])
            if cargo != {'color_id': params['color_id'], 'batch': params['batch']}:
                self.fail('cargo_state_conflict')
                return True
            del self.tray[params['tray_slot']]
            self.counts['placed'] += 1
            self.counts['stacked'] += int(params['layer'] == 1)
        self._event('action_completed', operation=command.operation)
        self._advance()
        return True

    def snapshot(self):
        return {'schema_version': 1, 'simulation': self.simulation,
                'run_id': self.run_id, 'phase': self.phase.value,
                'task_code': str(self.task) if self.task else None,
                'step_index': self.step_index, 'step_count': len(self.plan),
                'capacity': self.capacity, 'cargo': deepcopy(self.tray),
                'counts': dict(self.counts), 'error': self.error,
                'stop_confirmed': self.stop_confirmed,
                'pending': self.pending.to_dict() if self.pending else None}
