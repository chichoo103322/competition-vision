"""Prewarmed, repeatable one-button chassis service (optional real QR/arm alignment, no gripper).

Run before the start command. Every NEW key after READY starts a fresh route.
"""
import argparse
from contextlib import ExitStack
import fcntl
import json
import importlib
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import signal
import subprocess
import time

from pi_vision import electrical_protocol as protocol
from pi_vision.electrical_link import ElectricalClient
from pi_vision.electrical_coordinates import direction_review, direction_review_html
from pi_vision.electrical_routes import compile_chassis_test, configure_work_heading_mode
from tools.electrical_route_test import (acquire_plan, checked_port, endpoints, http_json,
    run_closed_loop, validate_geometry, validate_profile, verify_before_start)


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    with temp.open('w') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temp.replace(path)


def new_tick(tick, baseline):
    # STM32 uint32 millisecond wrap; startup/reboot must establish a new baseline.
    return 0 <= ((tick - baseline) & 0xffffffff) < 0x80000000


class WireLog:
    def __init__(self, port, emit):
        self.port, self.emit = port, emit

    def write(self, data):
        self.emit(event='TX', hex=data.hex(' '), bytes=len(data))
        return self.port.write(data)

    def read(self, size):
        data = self.port.read(size)
        if data:
            self.emit(event='RX_RAW', hex=data.hex(' '), bytes=len(data))
        return data


class RunnerLog(list):
    def __init__(self, emit):
        super().__init__()
        self.emit = emit

    def append(self, item):
        super().append(item)
        self.emit(event='RUNNER', **item)


class StandbyRearm(RuntimeError):
    """Before an attempt only: discard the old readiness/key baseline."""


def controller_idle(status):
    return (status['state'] not in (2, 3, 6) and
            not status['fault'] and not status['chassis_fault'])


def wait_fresh_key(client, baseline, health, emit, *, cancelled=lambda: False,
                   sleep=time.sleep):
    """Keep the already-open UART warm indefinitely. Never replay a pending key."""
    previous = baseline
    while not cancelled():
        status = client.status()
        if not new_tick(status['tick'], previous):
            raise StandbyRearm('MCU rebooted during standby; re-arm before a new round')
        previous = status['tick']
        if not controller_idle(status):
            raise StandbyRearm('controller became active/faulted; return to idle before rearming')
        healthy = health()
        if not healthy:
            raise StandbyRearm('lidar not ready; discard pre-readiness keys')
        try:
            selection = client.wait_start_area(.2, cancelled)
        except TimeoutError:
            sleep(.05)
            continue
        if not new_tick(selection['tick'], baseline):
            emit(event='IGNORED_OLD_KEY', selection=selection)
            continue
        if not healthy:
            raise RuntimeError('lidar not ready at key press; round not started')
        return selection
    raise InterruptedError('standby stopped')


def wait_ready_key(client, health, state, emit, *, cancelled=lambda: False, sleep=time.sleep):
    """Wait for idle hardware, then accept only a key newer than the READY baseline."""
    while not cancelled():
        state('WAIT_CONTROLLER', message='等待电控通信和传感器就绪；恢复后须按新键')
        try:
            while not cancelled():
                try:
                    status = client.status()
                except TimeoutError:
                    sleep(.2)
                    continue
                if not controller_idle(status):
                    sleep(.2)
                    continue
                if health():
                    break
                sleep(.2)
            if cancelled():
                raise InterruptedError('standby stopped')
            state('READY', message='就绪：KEY1区1、KEY0区2；请按下并松开一次',
                  controller_tick=status['tick'])
            selection = wait_fresh_key(client, status['tick'], health, emit,
                                       cancelled=cancelled, sleep=sleep)
            return client, selection
        except (TimeoutError, StandbyRearm) as exc:
            emit(event='STANDBY_RECONNECT', reason=str(exc))
            client = ElectricalClient(client.transport, clock=client.clock)
        except RuntimeError as exc:
            # _record_area can detect a reboot before the matching STATUS.
            if 'START_AREA identity regressed' not in str(exc):
                raise
            emit(event='STANDBY_RECONNECT', reason=str(exc))
            client = ElectricalClient(client.transport, clock=client.clock)
    raise InterruptedError('standby stopped')


def lidar_fresh(url):
    try:
        state = http_json(endpoints(url)[0])
        return (state.get('valid') is True and type(state.get('age_ms')) in (int, float) and
                0 <= state['age_ms'] < 1000 and bool(state.get('frame')))
    except (OSError, ValueError):
        return False


def ensure_lidar(config, emit, cancelled):
    """Reuse an existing collector; launch only if HTTP has no existing service."""
    url = config['state_url']
    owned = None
    try:
        http_json(endpoints(url)[0])
    except OSError:
        command = list(config['lidar_command'])
        # The saved command is explicit deployment config, never shell input.
        if '--seconds' not in command or command[command.index('--seconds')+1] != '0':
            raise ValueError('lidar command must use --seconds 0')
        log_path = Path(config['output_root']) / 'lidar_service.log'
        with log_path.open('ab') as log:
            owned = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT)
        emit(event='LIDAR_STARTED', pid=owned.pid)
    deadline = time.monotonic() + 30
    try:
        while time.monotonic() < deadline and not cancelled():
            if owned is not None and owned.poll() is not None:
                raise RuntimeError('lidar collector exited during startup')
            if lidar_fresh(url):
                return owned
            time.sleep(.2)
        raise TimeoutError('lidar not ready before standby')
    except BaseException:
        if owned is not None and owned.poll() is None:
            owned.terminate()
            owned.wait(timeout=5)
        raise


def load_competition_backend(spec, config):
    """Explicit real backend; never default to MockBackend or the chassis test."""
    module, separator, factory = spec.partition(':')
    if not separator or not module or not factory:
        raise ValueError('competition backend must be module:factory')
    backend = getattr(importlib.import_module(module), factory)(config)
    if any(not callable(getattr(backend, name, None)) for name in ('ready', 'plan', 'on_work')):
        raise ValueError('competition backend requires ready/plan/on_work')
    if getattr(backend, 'simulation', None) is not False:
        raise ValueError('competition backend must explicitly declare simulation=False')
    return backend


def test_turn_override_enabled(config, zone):
    return config['profiles'][str(zone)].get('test_ignore_turn_obstacles') is True


def make_test_station_work(client, config, emit, backend=None):
    from tools.route_station_dwell import TimedStationWork, timed_station_test
    if not timed_station_test(config):
        return None
    if backend is not None:
        raise ValueError('timed station work is test-only, not a competition backend')
    return TimedStationWork(client,config['test_station_work'],emit)


def run_attempt(client, config, selected, emit, cancelled, backend=None):
    from tools.qr_station_work import QRStationWork
    qr_settings = dict(config.get('qr_station', {}))
    if config.get('output_root'):
        qr_settings['preview_dir'] = str(Path(config['output_root'])/'qr_camera')
    qr_work = (QRStationWork(qr_settings)
               if backend is None and 'qr_station' in config else None)
    try:
        if qr_work is not None:
            qr_work.prewarm()
            emit(event='QR_CAMERA_PREWARMING')
        return _run_attempt(client, config, selected, emit, cancelled, backend, qr_work)
    finally:
        if qr_work is not None:
            qr_work.close()
            emit(event='QR_CAMERA_CLOSED')


def _run_attempt(client, config, selected, emit, cancelled, backend=None, qr_work=None):
    arm_work = make_test_station_work(client,config,emit,backend)
    arm_config = config.get('arm_alignment') or {}
    if backend is not None and arm_config.get('enabled'):
        raise ValueError('alignment-only test cannot be used as competition grasp feedback')
    if arm_work is None and arm_config.get('enabled') is True:
        from tools.arm_alignment_work import ArmAlignmentWork
        arm_work = ArmAlignmentWork(client, arm_config,
            lambda: http_json(arm_config['vision_state_url']))
    turntable = config.get('turntable_work') or {}
    if getattr(arm_work,'mode',None)!='timed_dwell' and turntable.get('enabled') is True:
        if backend is not None or arm_work is not None:
            raise ValueError('choose one explicitly configured station-work backend')
        from tools.turntable_work import TurntableWork
        previous_phase = [None]
        def publish(value):
            atomic_json(Path(config['output_root'])/'material_work.json', value)
            signature = (value['phase'], value['color_id'], value['step'])
            if signature != previous_phase[0]:
                emit(event='MATERIAL_WORK', **value)
                previous_phase[0] = signature
        url = turntable['vision_state_url']
        if url != 'http://127.0.0.1:8090/state':
            raise ValueError('turntable uses the dedicated local material camera service')
        arm_work = TurntableWork(client, turntable, lambda: http_json(url),
            lambda color: http_json('http://127.0.0.1:8090/material-target', 'POST', {'color_id': color}), publish)
    root = Path(config['output_root'])
    old_id = int((root/'route_id').read_text()) if (root/'route_id').exists() else 0
    status = client.check_start_area(selected)
    route_id = max(int(time.time()), old_id+1, status['route_id']+1,
                   config.get('route_id_floor', 0)+1)
    protocol.integer(route_id, 1, 0xffffffff, 'route_id')
    atomic_json(root/'route_id', route_id)
    def check():
        if cancelled():
            raise InterruptedError('service stopping')
        return client.check_start_area(selected)
    state = acquire_plan(config['state_url'], timeout_s=30, poll_s=.2,
                         start_zone=selected['area'], check_current=check)
    field_map = http_json(endpoints(config['state_url'])[2])
    if backend is None:
        profile = config['profiles'][str(selected['area'])]
        route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                    route_id=route_id, **profile)
        if test_turn_override_enabled(config, selected['area']):
            emit(event='TEST_TURN_OBSTACLES_IGNORED', area=selected['area'], route_id=route_id)
    else:
        route = backend.plan(state=state, selection=dict(selected), route_id=route_id,
                             check_current=check)
        if (route.get('test_mode') or route.get('valid_for_motion') is not True or
                route.get('route_id') != route_id or route.get('start_zone') != selected['area']):
            raise ValueError('competition backend must return a commissioned, key-bound real route')
        configure_work_heading_mode(
            route, config['profiles'][str(selected['area'])]['work_heading_mode_enabled'])
    validate_geometry(state, route, field_map)
    review = direction_review(protocol.upload_payload(route), work_heading_mode_enabled=route[
        'work_heading_mode_enabled'])
    prepared = dict(route=route, state=state, selection=selected, direction_review=review)
    atomic_json(root/'prepared_live.json', prepared)
    (root/'direction_review.html').write_text(direction_review_html(review), encoding='utf-8')
    emit(event='PREPARED', route_id=route_id, points=len(route['points']),
         test_ignore_turn_obstacles=route.get('test_ignore_turn_obstacles', False))

    def before_start():
        check()
        if arm_work is not None:
            arm_work.preflight()
        if qr_work is not None and not qr_work.ready():
            raise RuntimeError('QR camera prewarm failed or frames stale; do not START')
        if cancelled():
            raise InterruptedError('service stopping')
        current = verify_before_start(config['state_url'], state)
        validate_geometry(current, route, field_map)

    if backend is not None:
        final_status = client.run_selected_route(selected, route, backend.on_work,
            poll_work=getattr(backend, 'poll_work', None),
            before_start=before_start, cancelled=cancelled)
        return dict(result='done', status=final_status, mode='competition',
                    simulated_work=False)
    return run_closed_loop(client, route, qr_work=qr_work, arm_work=arm_work, station_dwell_s=2, poll_s=.2,
        timeout_s=600, log=RunnerLog(emit), before_start=before_start,
        cancelled=cancelled, start_request_id=selected['request_id'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--test-only-route', action='store_true')
    modes.add_argument('--competition-backend', help='explicit real module:factory; no timed work')
    args = parser.parse_args(argv)
    config = json.loads(args.config.read_text())
    root = Path(config['output_root'])
    root.mkdir(parents=True, exist_ok=True)
    cancel = [False]
    signal.signal(signal.SIGTERM, lambda *_: cancel.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: cancel.__setitem__(0, True))
    with ExitStack() as stack:
        lock = stack.enter_context((root/'service.lock').open('w'))
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not args.test_only_route and not args.competition_backend:
            parser.error('choose --test-only-route or --competition-backend module:factory')
        backend = (load_competition_backend(args.competition_backend, config)
                   if args.competition_backend else None)
        handler = RotatingFileHandler(root/'wire.jsonl', maxBytes=10_000_000,
                                      backupCount=3, encoding='utf-8')
        stack.callback(handler.close)
        logger = logging.getLogger('chassis_button_wire')
        logger.handlers = [handler]
        logger.setLevel(logging.INFO)
        logger.propagate = False

        def emit(**item):
            item = dict(at=time.time(), **item)
            logger.info(json.dumps(item, ensure_ascii=False))
            if item['event'] not in ('TX', 'RX_RAW'):
                print(json.dumps(item, ensure_ascii=False), flush=True)

        def state(label, **details):
            atomic_json(root/'status.json', dict(state=label, at=time.time(), **details))
            emit(event=label, **details)

        owned = None
        try:
            from tools.route_station_dwell import timed_station_test
            timed_work = timed_station_test(config)
            if timed_work and backend is not None:
                raise ValueError('timed station work is test-only')
            if not timed_work and (config.get('arm_alignment') or {}).get('enabled') is True:
                from tools.arm_alignment_work import validate_config
                validate_config(config['arm_alignment'])
                if config.get('qr_station', {}).get('camera') in {
                        c['camera_source'] for c in config['arm_alignment']['calibrations'].values()}:
                    raise ValueError('QR and material cameras must be distinct devices')
                if backend is not None:
                    raise ValueError('alignment-only configuration is test-only')
            for zone in ('1', '2'):
                validate_profile(config['profiles'][zone])
            from tools.qr_station_work import QRStationWork
            qr_config = (QRStationWork(config['qr_station'])
                         if backend is None and 'qr_station' in config else None)
            qr_ready = lambda: qr_config is None or Path(qr_config.camera).exists()
            # Open UART before lidar startup. No ACK/route/START is sent yet.
            import serial
            state('WAIT_DEVICES')
            while not (Path(config['port']).exists() and Path(config['lidar_device']).exists()):
                if cancel[0]:
                    raise InterruptedError('startup stopped')
                time.sleep(.25)
            port = stack.enter_context(serial.Serial(str(checked_port(config['port'])),
                115200, timeout=.05, write_timeout=1, exclusive=True))
            client = ElectricalClient(WireLog(port, emit))
            state('WARMING_UP')
            owned = ensure_lidar(config, emit, lambda: cancel[0])
            client, selection = wait_ready_key(client,
                lambda: qr_ready() and lidar_fresh(config['state_url']) and (backend is None or backend.ready() is True),
                state, emit, cancelled=lambda: cancel[0])
            while not cancel[0]:
                state('SCANNING', selection=selection)
                try:
                    result = run_attempt(client, config, selection, emit,
                                         lambda: cancel[0], backend)
                    atomic_json(root/'result.json', result)
                    state('DONE' if result['result'] == 'done' else 'FAILED',
                          result=result['result'], error=result.get('error'))
                except Exception as exc:
                    if cancel[0]:
                        raise
                    atomic_json(root/'result.json', dict(result='failed',
                        error=f'{type(exc).__name__}: {exc}'))
                    state('FAILED', error=f'{type(exc).__name__}: {exc}')
                if cancel[0]:
                    break
                # No persistent attempt lock: a fresh post-READY key starts
                # another complete scan/plan from whichever zone it selects.
                client, selection = wait_ready_key(client,
                    lambda: (qr_ready() and lidar_fresh(config['state_url']) and
                             (backend is None or backend.ready() is True)), state, emit,
                    cancelled=lambda: cancel[0])
        except (Exception, KeyboardInterrupt) as exc:
            state('FAILED', error=f'{type(exc).__name__}: {exc}')
            raise
        finally:
            if owned is not None and owned.poll() is None:
                owned.terminate()
                owned.wait(timeout=5)


if __name__ == '__main__':
    main()
