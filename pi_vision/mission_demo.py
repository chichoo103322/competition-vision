"""Explicitly simulated end-to-end mission. Never imports hardware backends."""

import argparse
import json

from .mission import Phase, TaskCoordinator
from .task_confirmation import parse_competition_task


class MockBackend:
    """Test-only observations and acknowledgements, always tagged simulation."""

    def __init__(self, task_code, fail_operation=None):
        self.task_code = str(parse_competition_task(task_code))
        self.sequence = 0
        self.fail_operation = fail_operation

    def reply(self, command, now):
        if command['simulation'] is not True:
            raise ValueError('mock backend refuses real commands')
        operation, params = command['operation'], command['params']
        result = {'schema_version': 1, 'simulation': True,
                  'run_id': command['run_id'], 'command_id': command['command_id'],
                  'status': 'done', 'data': {}}
        if operation == self.fail_operation:
            result['status'] = 'failed'
            return result
        if operation in TaskCoordinator.PERCEPTION:
            self.sequence += 1
            result['status'] = 'observation'
            data = {'status': 'ok', 'valid': True, 'acquired_unix_s': now,
                    'source_sequence': self.sequence, 'source_session_id': 'mock-camera-session',
                    'camera_id': 'mock-qr' if operation == 'read_task' else 'mock-material',
                    'camera_role': 'qr' if operation == 'read_task' else 'material',
                    'coordinate_frame': 'camera_image', 'units': 'pixels',
                    'width': 1280, 'height': 720}
            if operation == 'read_task':
                data['task_code'] = self.task_code
            else:
                point = {'u': 600., 'v': 350., 'point_kind': 'simulated_point',
                         'color_id': params['color_id']}
                if 'position_id' in params:
                    point['position_id'] = params['position_id']
                if params.get('layer') == 1:
                    point['base_color_id'] = params['color_id']
                data.update(zone=params['zone'], targets=[point])
            result['data'] = data
        else:
            result['data'] = {'display_task': {'displayed': True},
                              'report_counts': {'displayed': True},
                              'navigate': {'arrived': True, 'stopped': True},
                              'pick_to_tray': {'grasped': True, 'loaded': True},
                              'place_from_tray': {'released': True, 'placement_verified': True},
                              'stop': {'stopped': True}}[operation]
        return result


def simulate(task_code='156+123+516+231', capacity=1, fail_operation=None):
    now = [1000.]
    coordinator = TaskCoordinator(capacity=capacity, simulation=True,
                                  clock=lambda: now[0], wall_clock=lambda: now[0]+1700000000)
    backend = MockBackend(task_code, fail_operation)
    coordinator.start('drawn_start_area')
    trace = []
    for _ in range(1000):
        trace.extend(coordinator.drain_commands())
        if coordinator.pending is None:
            break
        now[0] += .15
        coordinator.feedback(backend.reply(coordinator.pending.to_dict(), now[0]+1700000000))
        if coordinator.phase in (Phase.COMPLETED, Phase.FAULT, Phase.CANCELLED):
            break
    else:
        raise RuntimeError('simulation did not terminate')
    return coordinator, trace


def main(argv=None):
    parser = argparse.ArgumentParser(description='任务框架完整模拟；不打开相机、网页、串口或电机')
    parser.add_argument('--task-code', default='156+123+516+231')
    parser.add_argument('--capacity', type=int, choices=(1, 2, 3), default=1)
    parser.add_argument('--fail-operation', choices=tuple(TaskCoordinator.DEFAULT_TIMEOUTS))
    parser.add_argument('--trace', action='store_true', help='输出逐动作命令，均为 simulation=true')
    args = parser.parse_args(argv)
    try:
        coordinator, trace = simulate(args.task_code, args.capacity, args.fail_operation)
    except ValueError as error:
        parser.error(str(error))
    if args.trace:
        for command in trace:
            print(json.dumps(command, ensure_ascii=False))
    print(json.dumps({'evidence': 'software_simulation_only', **coordinator.snapshot()}, ensure_ascii=False))
    return 0 if coordinator.phase == Phase.COMPLETED else 1


if __name__ == '__main__':
    raise SystemExit(main())
