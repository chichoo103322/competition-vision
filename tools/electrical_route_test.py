"""Eight-stage chassis-only test. Default: prepare; --execute sends motion.

Uses the existing lidar HTTP collector and STM32 encoder/IMU controller.
Station completion means a timed test dwell, never a vision/gripper result.
"""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
import signal
import time
from urllib.parse import urlsplit, urlunsplit
from urllib.request import Request, urlopen

from pi_vision.electrical_link import ElectricalClient
from pi_vision.chassis_geometry import (validate_footprint, validate_fixed_heading_boundary,
                                        validate_fixed_heading_obstacles)
from pi_vision.electrical_coordinates import (direction_review, require_electrical_frame,
    electrical_to_nominal_pose, FIELD_SIZE_MM)
from pi_vision.electrical_routes import compile_chassis_test, segment_for_status
from pi_vision.electrical_protocol import frame, UPLOAD, upload_payload, scaled, integer
from tools.lidar_navigation import occupied_cell_edges, static_blocked_edges, grid_id


def http_json(url, method='GET', payload=None, timeout=2.0):
    body = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=body, method=method,
                      headers={'Content-Type': 'application/json'} if body else {})
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode())


def endpoints(url):
    parts = urlsplit(url)
    if parts.scheme != 'http' or parts.hostname not in ('localhost', '127.0.0.1', '::1'):
        raise ValueError('use the local lidar service or an SSH localhost tunnel')
    base = urlunsplit((parts.scheme, parts.netloc, '', '', '')).rstrip('/')
    return base + '/state', base + '/configure', base + '/field-map'


def positive(value, name, minimum=0):
    if type(value) not in (float, int) or not math.isfinite(value) or value <= minimum:
        raise ValueError(f'{name} must be finite and > {minimum}')


def live_state(state, zone):
    nav = state.get('navigation') or {}
    age = state.get('age_ms')
    if (state.get('valid') is not True or type(age) not in (int, float) or
            not math.isfinite(age) or not 0 <= age < 1000 or not state.get('frame') or
            nav.get('settings', {}).get('start_zone') != zone or not state.get('session_id')):
        raise ValueError('fresh lidar frame and matching explicit start zone required')
    return nav


def acquire_plan(state_url, *, timeout_s, poll_s, start_zone, request=http_json,
                 clock=time.monotonic, sleep=time.sleep, check_current=lambda: None,
                 first_stage_only=False):
    """Configure first, collect NEW scans for the evidence window, plan once."""
    positive(timeout_s, 'scan timeout')
    positive(poll_s, 'poll interval')
    state_url, config_url, _ = endpoints(state_url)
    if type(start_zone) is not int or start_zone not in (1, 2):
        raise ValueError('explicit start zone 1/2 required; never inherit a debug default')
    check_current()
    request(config_url, 'POST', {'start_zone': start_zone})
    baseline = request(state_url)
    session = baseline['session_id']
    initial_seq = (baseline.get('frame') or {}).get('seq', 0)
    previous_plan = (baseline.get('mission_plan') or {}).get('plan_id')
    settings = baseline['navigation']['settings']
    window_s = max(settings['evidence_ttl_s'], settings['far_evidence_ttl_s'])
    started = clock()
    seen = set()
    while clock() - started < timeout_s:
        check_current()
        state = request(state_url)
        if state.get('session_id') != session:
            raise ValueError('lidar service restarted during startup scan')
        try:
            nav = live_state(state, start_zone)
        except ValueError:
            seen.clear()
            sleep(poll_s)
            continue
        if nav['settings'] != settings:
            raise ValueError('lidar configuration changed during startup scan')
        seq = state['frame']['seq']
        if seq > initial_seq:
            seen.add(seq)
        if len(seen) >= 3 and nav['unique_scans'] >= 3 and clock() - started >= window_s:
            check_current()
            request(config_url, 'POST', {'action': 'plan_mission'})
            planned = request(state_url)
            live_state(planned, start_zone)
            plan = planned.get('mission_plan') or {}
            if (planned['session_id'] != session or plan.get('session_id') != session or
                    plan.get('plan_id') in (None, previous_plan) or plan.get('source_seq', 0) < seq or
                    planned['navigation']['settings'] != settings or plan.get('settings') != settings or
                    (not first_stage_only and plan.get('status') != 'ready') or
                    (first_stage_only and (not plan.get('segments') or
                                           plan['segments'][0].get('status') != 'path')) or
                    plan.get('evidence_status') != 'snapshot_preview'):
                raise ValueError('new eight-stage plan unavailable/blocked; no upload')
            check_current()
            return planned
        sleep(poll_s)
    raise TimeoutError('startup scan did not produce sufficient new evidence')


def validate_geometry(state, route, field_map):
    """Check graph provenance and all expanded axis legs, including start/home connectors."""
    nav, plan = state['navigation'], state['mission_plan']
    require_electrical_frame(route.get('coordinate_frame'))
    if field_map.get('boundary_mm') != [0, 0, FIELD_SIZE_MM, FIELD_SIZE_MM]:
        raise ValueError('electrical frame transform is defined only for the 2400 mm field')
    if plan['settings'] != nav['settings']:
        raise ValueError('plan/configuration mismatch')
    footprint = route.get('footprint')
    radius = nav['settings']['clearance_m']
    if footprint is not None:
        # Replace ONLY the circular field-boundary test with the measured
        # rectangle. The original obstacle clearance is retained, with an
        # additional check of the measured swept rectangle.
        validate_fixed_heading_boundary(route, footprint, field_map['boundary_mm'])
        # Keep the original centreline clearance AND check the actual swept
        # rectangle. A circumscribed disc would falsely block narrow lanes.
        nominal_route = dict(initial_pose=electrical_to_nominal_pose(route['initial_pose']),
                             points=[electrical_to_nominal_pose(p) for p in route['points']])
        rectangles = [r['rect_mm'] for r in field_map.get('floor_exclusions', []) + field_map.get('work_zones', [])]
        rectangles += [[v*1000 for v in g['rect']] for g in nav['grid_rects']
                       if g['id'] in nav['occupied_cells']]
        validate_fixed_heading_obstacles(nominal_route, footprint, rectangles)
    initial_nominal = electrical_to_nominal_pose(route['initial_pose'])
    if grid_id(initial_nominal['x_mm'] / 1000, initial_nominal['y_mm'] / 1000) != (
            4 if route['start_zone'] == 1 else 0):
        raise ValueError('actual chassis start is outside the selected home grid cell')
    blocked = set(nav['blocked_edges']) | set(nav['static_blocked_edges'])
    edges = {e['id']: {e['a'], e['b']} for e in nav['edges']}
    for segment in plan['segments'][:len(route['segments'])]:
        for a, b, edge in zip(segment['nodes'], segment['nodes'][1:], segment['edges']):
            if edge in blocked or edges.get(edge) != {a, b}:
                raise ValueError('route edge blocked or does not connect its nodes')
    previous = initial_nominal
    for point in route['points']:
        point = electrical_to_nominal_pose(point)
        a = [previous['x_mm'] / 1000, previous['y_mm'] / 1000]
        b = [point['x_mm'] / 1000, point['y_mm'] / 1000]
        previous = point
        if a == b:
            continue
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError('non-axis-aligned motion')
        nodes = [dict(id=0, xy=a), dict(id=1, xy=b)]
        leg = [dict(id=0, a=0, b=1)]
        if (static_blocked_edges(field_map, nodes, leg, radius, check_boundary=footprint is None) or
                occupied_cell_edges(nodes, leg, nav['occupied_cells'])):
            raise ValueError('expanded route/start connector intersects an occupied or excluded area')


def verify_before_start(state_url, prepared, request=http_json):
    current = request(endpoints(state_url)[0])
    live_state(current, prepared['mission_plan']['settings']['start_zone'])
    old, new = prepared['mission_plan'], current.get('mission_plan') or {}
    if (current['session_id'] != prepared['session_id'] or new.get('plan_id') != old['plan_id'] or
            new.get('evidence_status') != 'snapshot_preview' or new.get('segments') != old['segments'] or
            current['navigation']['settings'] != prepared['navigation']['settings'] or
            current['navigation']['nodes'] != prepared['navigation']['nodes']):
        raise ValueError('startup plan changed/invalidated before START')
    used = {e for s in old['segments'] for e in s['edges']}
    if used.intersection(current['navigation']['blocked_edges']):
        raise ValueError('route newly blocked before START')
    return current


def at_target(status, point):
    yaw_error = (status['yaw_deg'] - point['yaw_deg'] + 180) % 360 - 180
    return (math.hypot(status['x_mm'] - point['x_mm'], status['y_mm'] - point['y_mm']) <= 8 and
            abs(yaw_error) <= 2)


def run_closed_loop(client, route, *, station_dwell_s, poll_s, timeout_s, log,
                    before_start=lambda: None, clock=time.monotonic, sleep=time.sleep,
                    cancelled=lambda: False, start_request_id=None):
    """Only explicit test dwell completion advances the eight station gates.

    No camera/vision access. Faults, cancellation and timeout attempt route STOP;
    a lost link is reported as stop unconfirmed, not as a successful stop.
    """
    positive(station_dwell_s, 'station dwell')
    positive(poll_s, 'poll interval')
    positive(timeout_s, 'run timeout')
    if route.get('test_mode') != 'chassis_route_dry_work':
        raise ValueError('not a chassis-only test route')
    gates = len(route['segments'])
    if gates not in (1, 8) or [s['segment'] for s in route['segments']] != list(range(1, gates+1)):
        raise ValueError('only the first stage or the full eight-stage test is supported')
    deadline = clock() + timeout_s
    touched = False
    completed = 0
    last_index = -1
    last_tick = None
    wait_since = None
    last_key = None

    def check_deadline():
        if cancelled():
            raise InterruptedError('test cancelled')
        if clock() >= deadline:
            raise TimeoutError('route test deadline exceeded')

    try:
        check_deadline()
        initial = client.status()
        if initial['state'] in (2, 3):
            raise ValueError('controller already executing; do not take over')
        if route['route_id'] <= initial['route_id']:
            raise ValueError('use a new increasing route_id for this test')
        before_start()
        check_deadline()
        touched = True
        if start_request_id is None:
            reply = client.upload(route, test_only=True)
        else:
            reply = client.upload(route, test_only=True, start_request_id=start_request_id)
        if reply['state'] != 1 or reply['route_id'] != route['route_id'] or reply['point_count'] != len(route['points']):
            raise ValueError('UPLOAD did not produce matching READY route')
        log.append(dict(action='upload_ready', route_id=route['route_id'], points=len(route['points'])))
        before_start()
        check_deadline()
        reply = (client.start(route['route_id']) if start_request_id is None else
                 client.start_when_ready(route['route_id'], sleep=sleep, cancelled=cancelled))
        if reply['route_id'] != route['route_id'] or reply['state'] not in (2, 3):
            raise ValueError('START not accepted for matching route')
        log.append(dict(action='started'))
        while True:
            check_deadline()
            status = client.status()
            check_deadline()
            segment = segment_for_status(route, status)  # checks route id/count before even DONE
            if status['fault'] or status['chassis_fault'] or status['state'] in (0, 1, 5, 6):
                raise RuntimeError(f'controller no longer running: {status}')
            if status['point_index'] < last_index:
                raise RuntimeError('point index regressed; possible reset')
            last_index = status['point_index']
            tick = status['tick']
            if last_tick is not None and ((tick - last_tick) & 0xffffffff) >= 0x80000000:
                raise RuntimeError('controller tick regressed; possible reboot')
            new_tick = tick != last_tick
            last_tick = tick
            key = (status['state'], status['point_index'], status['work_id'])
            if key != last_key:
                log.append(dict(action='status', **status))
                last_key = key
            if status['state'] == 4:
                if completed != gates or not at_target(status, route['points'][-1]):
                    raise RuntimeError('premature DONE or wrong final estimated pose')
                return dict(result='done', test_stations_completed=completed,
                            status=status, log=log, physical_stop_independently_measured=False)
            if completed >= gates:
                raise RuntimeError('expected DONE after last station')
            expected = route['segments'][completed]
            if status['point_index'] > expected['last_point_index']:
                raise RuntimeError('station skipped without this test runner releasing it')
            if status['state'] == 3:
                if segment['segment'] != completed + 1 or status['phase'] != 4:
                    raise RuntimeError('unexpected station gate')
                if not at_target(status, route['points'][status['point_index']]):
                    raise RuntimeError('WAIT_WORK pose outside controller arrival tolerances')
                if wait_since is None:
                    wait_since = clock()
                    log.append(dict(action='test_station_dwell_start', segment=completed + 1,
                                    point_index=status['point_index'], dwell_s=station_dwell_s))
                elif new_tick and clock() - wait_since >= station_dwell_s:
                    ack = client.work_done(route['route_id'], status['point_index'], success=True)
                    if ack['route_id'] != route['route_id'] or ack['point_index'] != status['point_index'] + 1:
                        raise RuntimeError('WORK_DONE did not advance this test station')
                    completed += 1
                    log.append(dict(action='test_station_released', segment=completed,
                                    synthetic_work_done=True, point_index=status['point_index']))
                    wait_since = None
            else:
                if wait_since is not None:
                    raise RuntimeError('station left before test dwell release')
            sleep(poll_s)  # also polls throughout the dwell, never sleeps the whole dwell
    except (Exception, KeyboardInterrupt) as exc:
        result = dict(result='failed', error=f'{type(exc).__name__}: {exc}',
                      test_stations_completed=completed, stop_attempted=touched,
                      controller_stop_confirmed=False, log=log)
        if touched:
            try:
                client.stop(route['route_id'])
                stopped = client.status()
                result['stop_status'] = stopped
                result['controller_stop_confirmed'] = (stopped['route_id'] == route['route_id'] and
                                                        stopped['state'] in (5, 6))
            except Exception as stop_error:
                result['stop_error'] = str(stop_error)
        return result


def checked_port(port):
    device = Path(port).resolve()
    lidar = Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    if (lidar.exists() and device == lidar.resolve()) or (not lidar.exists() and device.name == 'ttyUSB0'):
        raise ValueError('ttyUSB0/CP2102 is reserved for this project lidar')
    cmdline = Path('/proc/cmdline')
    if cmdline.exists() and any(token.split(',')[0] == f'console={device.name}'
                                for token in cmdline.read_text().split()):
        raise ValueError('target UART is a kernel console; configure the electrical UART first')
    return device


def validate_profile(profile):
    require_electrical_frame(profile.get('coordinate_frame'))
    if profile.get('stop_strategy', 'controller_deceleration') not in ('controller_deceleration', 'split_approach'):
        raise ValueError('unknown stop strategy')
    if 'footprint' in profile:
        validate_footprint(profile['footprint'])
    for key in ('x_mm', 'y_mm'):
        scaled(profile['initial_pose'][key], 1, 100000, key)
    scaled(profile['initial_pose']['yaw_deg'], 100, 180, 'initial yaw')
    if type(profile['heading_deg']) not in (int, float) or profile['heading_deg'] not in (-180, -90, 0, 90, 180):
        raise ValueError('fixed heading must be a cardinal map direction')
    for key in ('move_rpm', 'turn_rpm', 'approach_rpm'):
        integer(profile[key], 1, 120, key)
    integer(profile['approach_distance_mm'], 1, 300, 'approach_distance_mm')
    if profile['approach_rpm'] >= profile['move_rpm']:
        raise ValueError('approach RPM must be below cruise RPM')
    if profile['initial_pose']['yaw_deg'] != profile['heading_deg']:
        raise ValueError('initial yaw must match the fixed test heading')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state-url', default='http://127.0.0.1:8766/state')
    parser.add_argument('--state-json', type=Path, help='offline dry-run only')
    parser.add_argument('--field-map', type=Path, default=Path(__file__).resolve().parents[1] / 'config/field_map_national_2027.json')
    parser.add_argument('--profile', required=True, type=Path, help='single profile; with --wait-key use {"1":profile1,"2":profile2}')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--port')
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('--start-zone', type=int, choices=(1, 2), help='offline/manual planning only')
    selection.add_argument('--wait-key', action='store_true', help='KEY1=zone1, KEY0=zone2 from STM32')
    parser.add_argument('--key-timeout', type=float, default=120)
    parser.add_argument('--route-id', type=int, default=1, help='new id for this MCU boot; increment on each run')
    parser.add_argument('--scan-timeout', type=float, default=30)
    parser.add_argument('--poll', type=float, default=.2)
    parser.add_argument('--station-dwell', type=float, default=2.0)
    parser.add_argument('--run-timeout', type=float, default=600)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--test-only-route', action='store_true')
    args = parser.parse_args()
    if args.wait_key and (not args.port or args.state_json):
        parser.error('--wait-key requires an explicit electrical --port and live scanner')
    if args.execute and (not args.test_only_route or not args.wait_key):
        parser.error('execution requires --test-only-route and --wait-key')
    for value, label in [(args.poll, 'poll'), (args.scan_timeout, 'scan timeout'), (args.key_timeout, 'key timeout'),
                         (args.run_timeout, 'run timeout'), (args.station_dwell, 'station dwell')]:
        positive(value, label)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = dict(mode='chassis_route_dry_work', vision_used=False, gripper_used=False,
                  valid_for_competition=False, execute_requested=args.execute)
    cancel = [False]
    signal.signal(signal.SIGTERM, lambda *_: cancel.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: cancel.__setitem__(0, True))
    try:
        profiles = json.loads(args.profile.read_text())
        with ExitStack() as stack:
            client = None
            area_signal = None
            state = field_map = None
            if args.wait_key:
                # Reject missing/placeholder profiles before ACKing a physical key.
                for zone in ('1', '2'):
                    validate_profile(profiles[zone])
                import serial
                port = stack.enter_context(serial.Serial(str(checked_port(args.port)), 115200,
                                           timeout=.05, write_timeout=1, exclusive=True))
                client = ElectricalClient(port)
                print('等待按键：KEY1=启停区1，KEY0=启停区2；短按后松开。', flush=True)
                def planner(selected, check_current):
                    nonlocal state, field_map
                    print(f"收到 KEY{selected['key']}：启停区{selected['area']}，请求{selected['request_id']}，开始扫描规划。", flush=True)
                    state = acquire_plan(args.state_url, timeout_s=args.scan_timeout, poll_s=args.poll,
                                         start_zone=selected['area'], check_current=check_current)
                    field_map = http_json(endpoints(args.state_url)[2])
                    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                                 route_id=args.route_id, **profiles[str(selected['area'])])
                    validate_geometry(state, route, field_map)
                    return route
                area_signal, route = client.plan_after_key(planner, timeout=args.key_timeout,
                                                          cancelled=lambda: cancel[0])
                result['start_area_signal'] = area_signal
            else:
                validate_profile(profiles)
                if args.state_json:
                    state = json.loads(args.state_json.read_text())
                    field_map = json.loads(args.field_map.read_text())
                else:
                    state = acquire_plan(args.state_url, timeout_s=args.scan_timeout, poll_s=args.poll,
                                         start_zone=args.start_zone)
                    field_map = http_json(endpoints(args.state_url)[2])
                if state['mission_plan']['settings']['start_zone'] != args.start_zone:
                    raise ValueError('snapshot does not match explicit start zone')
                route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                             route_id=args.route_id, **profiles)
                validate_geometry(state, route, field_map)
            review = direction_review(upload_payload(route))
            result.update(route=route, startup_snapshot=state, direction_review=review)
            # Save before START so the exact planned directions remain reviewable
            # even if the runner or SSH connection is interrupted during motion.
            result['result'] = 'prepared'
            args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
            print('下发坐标：照片同向场地，左下原点，右+X/上+Y，毫米，逆时针为正；车体初始位姿：' +
                  json.dumps(review['initial_pose'], ensure_ascii=False), flush=True)
            for leg in review['legs']:
                print(f"点 {leg['point_index']} → {leg['target']}：" +
                      json.dumps(leg['actions'], ensure_ascii=False), flush=True)
            if not args.execute:
                from pi_vision.electrical_protocol import UPLOAD_FOR_AREA, area_upload_payload
                packet = (frame(UPLOAD_FOR_AREA, 1, area_upload_payload(route, area_signal['request_id']))
                          if area_signal else frame(UPLOAD, 1, upload_payload(route)))
                result.update(result='dry_run_only', upload_frame_hex_for_review=packet.hex())
            else:
                def before_start():
                    client.check_start_area(area_signal)
                    current = verify_before_start(args.state_url, state)
                    validate_geometry(current, route, field_map)
                result.update(run_closed_loop(client, route,
                              station_dwell_s=args.station_dwell, poll_s=args.poll,
                              timeout_s=args.run_timeout, log=[], before_start=before_start,
                              cancelled=lambda: cancel[0], start_request_id=area_signal['request_id']))
    except (Exception, KeyboardInterrupt) as exc:
        result.update(result='failed', error=f'{type(exc).__name__}: {exc}')
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(args.output.resolve()), result=result['result'],
                          execute=args.execute, error=result.get('error')), ensure_ascii=False))
    if result['result'] == 'failed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
