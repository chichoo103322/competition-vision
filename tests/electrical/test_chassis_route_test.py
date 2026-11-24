import copy
import math

import pytest

from pi_vision.electrical_routes import (compile_chassis_test, configure_work_heading_mode,
                                         first_segment_only, work_pose_error)
from tools.chassis_route_sim import synthetic_plan
from tools.electrical_route_test import acquire_plan, run_closed_loop, validate_geometry, verify_before_start


class Clock:
    now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, dt):
        self.now += dt


def prepared(zone=2):
    state, profile, config = synthetic_plan(zone)
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=7, **profile)
    return state, route, profile, config


@pytest.mark.parametrize('zone', [1, 2])
def test_full_geometry_and_speed_gates(zone):
    state, route, profile, config = prepared(zone)
    validate_geometry(state, route, config)
    assert not route['valid_for_motion']
    assert len(route['points']) <= 64
    assert [p['work_id'] for p in route['points'] if p['work_id']] == list(range(101, 109))
    for segment in route['segments']:
        end = segment['last_point_index']
        assert route['points'][end]['move_rpm'] == 40
    legacy = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                  route_id=7, stop_strategy='split_approach', **profile)
    assert len(route['points']) < len(legacy['points'])
    assert [(p['x_mm'], p['y_mm'], p['work_id']) for p in route['points'] if p['work_id']] == [
        (p['x_mm'], p['y_mm'], p['work_id']) for p in legacy['points'] if p['work_id']]


def test_legacy_split_approach_is_explicit_opt_in():
    state, _, profile, _ = prepared()
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                 route_id=7, stop_strategy='split_approach', **profile)
    for segment in route['segments']:
        a, b = route['points'][segment['last_point_index']-1:segment['last_point_index']+1]
        assert a['move_rpm'] == 40 and b['move_rpm'] == 12
        assert abs(a['x_mm']-b['x_mm']) + abs(a['y_mm']-b['y_mm']) == 120


@pytest.mark.parametrize('feature', ['straight', 'corner', 'reverse', 'speed', 'heading', 'work'])
def test_compaction_preserves_corners_gates_and_motion_changes(feature):
    from pi_vision.electrical_routes import compact_straight_points
    _, route, _, _ = prepared()
    def point(x, y, **kw):
        return dict(x_mm=x, y_mm=y, yaw_deg=0, move_rpm=40, turn_rpm=20, work_id=0, **kw)
    a, b, c = point(0, 0), point(100, 0), point(200, 0)
    if feature == 'corner': c.update(x_mm=100, y_mm=100)
    if feature == 'reverse': c['x_mm'] = 50
    if feature == 'speed': c['move_rpm'] = 20
    if feature == 'heading': c['yaw_deg'] = 90
    if feature == 'work': b['work_id'] = 102
    c['work_id'] = 103
    route.update(initial_pose=a, points=[b, c], segments=[dict(
        segment=1, first_point_index=0, last_point_index=1, work_id=103)])
    original = copy.deepcopy(route)
    compact = compact_straight_points(route)
    assert route == original
    assert len(compact['points']) == (1 if feature == 'straight' else 2)
    assert compact['points'][-1] == c
    assert compact['segments'][0]['last_point_index'] == len(compact['points'])-1


@pytest.mark.parametrize('zone', [1, 2])
def test_compact_preserves_entire_axis_path_and_segment_mapping(zone):
    from pi_vision.electrical_routes import compact_straight_points
    state, route, profile, config = prepared(zone)
    legacy = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                  route_id=7, stop_strategy='split_approach', **profile)
    def visited(r):
        prev = r['initial_pose']
        traversed = []
        for p in r['points']:
            dx, dy = p['x_mm']-prev['x_mm'], p['y_mm']-prev['y_mm']
            length = abs(dx)+abs(dy)
            traversed.extend((prev['x_mm'] + i*(1 if dx>0 else -1 if dx<0 else 0),
                              prev['y_mm'] + i*(1 if dy>0 else -1 if dy<0 else 0))
                             for i in range(1, length+1))
            prev = p
        return traversed
    assert visited(route) == visited(legacy)
    assert compact_straight_points(route)['points'] == route['points']
    for seg in route['segments']:
        assert route['points'][seg['last_point_index']]['work_id'] == seg['work_id']
    validate_geometry(state, route, config)


def test_actual_start_connector_is_explicit_and_returns_to_actual_start():
    state, _, profile, config = prepared()
    profile['initial_pose'].update(x_mm=320, y_mm=2150)
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=7, **profile)
    validate_geometry(state, route, config)
    prev = profile['initial_pose']
    for point in route['points']:
        assert prev['x_mm'] == point['x_mm'] or prev['y_mm'] == point['y_mm']
        prev = point
    assert (prev['x_mm'], prev['y_mm'], prev['work_id']) == (320, 2150, 108)


@pytest.mark.parametrize('change', ['yaw', 'speed', 'nan', 'blocked', 'edge', 'connector'])
def test_bad_route_parameters_rejected(change):
    state, route, profile, config = prepared()
    if change == 'yaw':
        profile['heading_deg'] = profile['initial_pose']['yaw_deg'] = 45
    elif change == 'speed':
        profile['approach_rpm'] = 40
    elif change == 'nan':
        profile['initial_pose']['x_mm'] = math.nan
    elif change == 'blocked':
        state['navigation']['occupied_cells'].append(0)
    elif change == 'edge':
        state['navigation']['edges'][0]['a'] = 99
    elif change == 'connector':
        profile['initial_pose']['x_mm'] = 2350
    with pytest.raises(ValueError):
        route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=7, **profile)
        validate_geometry(state, route, config)


class PlannerHTTP:
    def __init__(self, clock):
        self.clock = clock
        self.state, _, _ = synthetic_plan()
        self.old = copy.deepcopy(self.state['mission_plan'])
        self.posts = []
        self.seq = 10
        self.restarted = False
        self.freeze = False

    def __call__(self, url, method='GET', payload=None):
        if method == 'POST':
            self.posts.append(payload)
            if payload.get('action') == 'plan_mission':
                self.state['mission_plan']['plan_id'] = 'new-request'
                self.state['mission_plan']['source_seq'] = self.seq
            return {'ok': True}
        if not self.freeze:
            self.seq += 1
        self.state['frame']['seq'] = self.seq
        if self.restarted and self.clock.now > .2:
            self.state['session_id'] = 'restarted'
        return copy.deepcopy(self.state)


def test_scan_always_builds_new_plan_after_evidence_window_even_with_old_ready():
    clock = Clock()
    request = PlannerHTTP(clock)
    result = acquire_plan('http://127.0.0.1:8766', timeout_s=10, poll_s=.2, start_zone=2,
                          request=request, clock=clock, sleep=clock.sleep)
    assert request.posts == [{'start_zone': 2}, {'action': 'plan_mission'}]
    assert result['mission_plan']['plan_id'] == 'new-request'
    assert clock.now >= request.state['navigation']['settings']['far_evidence_ttl_s']


@pytest.mark.parametrize('problem', ['freeze', 'restarted'])
def test_old_frames_or_server_restart_never_plan(problem):
    clock = Clock()
    request = PlannerHTTP(clock)
    setattr(request, problem, True)
    with pytest.raises((TimeoutError, ValueError)):
        acquire_plan('http://127.0.0.1:8766/state', timeout_s=5, poll_s=.2, start_zone=2,
                     request=request, clock=clock, sleep=clock.sleep)
    assert not any(p.get('action') == 'plan_mission' for p in request.posts)


def test_pre_start_rechecks_live_plan_invalidation():
    state, _, _, _ = prepared()
    changed = copy.deepcopy(state)
    changed['mission_plan']['evidence_status'] = 'new_obstacle_replan_required'
    with pytest.raises(ValueError):
        verify_before_start('http://127.0.0.1:8766', state, request=lambda _: changed)


class FakeChassis:
    def __init__(self, route, clock, failure=None):
        self.route, self.clock, self.failure = route, clock, failure
        self.state, self.stage, self.route_id = 0, 0, 0
        self.index, self.count, self.work = 0, 0, 0
        self.releases, self.stops, self.uploads = [], 0, 0
        self.turn_start = None

    def report(self):
        point = (self.route['points'][-1] if self.state == 4 else
                 self.route['points'][min(self.index, len(self.route['points'])-1)])
        phase = 6 if self.state == 2 and self.failure in ('turning', 'turn_timeout') else (4 if self.state == 3 else 0)
        yaw = point.get('expected_work_yaw_deg', point['yaw_deg']) if self.state in (3, 4) else point['yaw_deg']
        return dict(request=0x13, code=0, state=self.state, phase=phase, route_id=self.route_id,
                    point_index=self.index, point_count=self.count, work_id=self.work,
                    fault=0, chassis_fault=0, tick=round(self.clock.now * 1000),
                    x_mm=point['x_mm'], y_mm=point['y_mm'], yaw_deg=yaw)

    def status(self):
        self.clock.now += .01
        if self.state == 2:
            if self.failure == 'turning':
                self.turn_start = self.clock.now if self.turn_start is None else self.turn_start
                if self.clock.now - self.turn_start < 1:
                    return self.report()
            if self.failure == 'turn_timeout':
                return self.report()
            if self.failure == 'no_arrival':
                return self.report()
            self.state = 3
            gate = self.route['segments'][self.stage]
            self.index, self.work = gate['last_point_index'], gate['work_id']
        result = self.report()
        if self.state == 3:
            if self.failure == 'other_route':
                result['route_id'] = 123
            elif self.failure == 'premature_done':
                result.update(state=4, point_index=self.count)
            elif self.failure == 'wrong_pose':
                result['x_mm'] += 30
            elif self.failure == 'fault_dwell' and self.clock.now > .5:
                result.update(state=6, fault=1)
            elif self.failure == 'disconnect' and self.clock.now > .5:
                raise TimeoutError('UART lost')
        return result

    def upload(self, route, *, test_only=False):
        assert test_only
        self.uploads += 1
        self.route_id, self.count, self.state = route['route_id'], len(route['points']), 1
        return self.report()

    def start(self, route_id):
        assert route_id == self.route_id
        self.state = 2
        return self.report()

    def work_done(self, route_id, index, *, success):
        assert self.state == 3 and self.index == index and success is True
        self.releases.append((self.stage + 1, self.clock.now))
        self.stage += 1
        self.index += 1
        self.work = 0
        self.state = 4 if self.stage == 8 else 2
        return self.report()

    def stop(self, _):
        self.stops += 1
        if self.failure == 'disconnect':
            raise TimeoutError('STOP lost')
        self.state = 5
        return self.report()


def test_full_runner_waits_at_all_eight_stations_and_only_then_done():
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock)
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=60,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'done' and result['test_stations_completed'] == 8
    assert [s for s, _ in client.releases] == list(range(1, 9))
    assert clock.now >= 16 and not client.stops
    assert all(b[1] - a[1] >= 2 for a, b in zip(client.releases, client.releases[1:]))


@pytest.mark.parametrize('zone',[1,2])
def test_eight_stage_qr_then_timed_work_with_firmware_arm_departure_gate(zone):
    from tools.route_station_dwell import TimedStationWork
    _,route,_,_=prepared(zone)
    clock=Clock()
    class ParkingChassis(FakeChassis):
        arm_key=None
        arm_state='READY'
        arm_command=0
        finishes=[]
        def arm_status(self):
            key=(self.route_id,self.index,self.work)
            if self.state==3 and 102<=self.work<=107:
                if key!=self.arm_key:
                    self.arm_key=key;self.arm_state='WAIT_VISION';self.arm_command=0
                if self.arm_state=='PARKING' and clock.now>=self.park_until:
                    self.arm_state='READY_LEAVE'
            return dict(zip(('route_id','point_index','work_id'),key),
                state_name=self.arm_state,command_id=self.arm_command,
                origins_valid=True,fault=0,standalone_test_mode=False)
        def arm_finish(self,route_id,index,work,command):
            assert (route_id,index,work)==self.arm_key
            assert self.arm_state=='WAIT_VISION'
            self.finishes.append(work)
            self.arm_command=command;self.arm_state='PARKING';self.park_until=clock.now+.4
        def work_done(self,route_id,index,*,success):
            if 102<=self.work<=107:assert self.arm_state=='READY_LEAVE'
            return super().work_done(route_id,index,success=success)
    class ConfirmedQR:
        def start(self):self.started=clock.now
        def poll(self):
            return (dict(confirmed=True,raw='231+312+321+231')
                    if clock.now-self.started>=.4 else None)
        def close(self):pass
    client=ParkingChassis(route,clock);client.finishes=[]
    worker=TimedStationWork(client,dict(dwell_s=2),clock=clock)
    result=run_closed_loop(client,route,qr_work=ConfirmedQR(),arm_work=worker,
        station_dwell_s=2,poll_s=.2,timeout_s=60,log=[],clock=clock,sleep=clock.sleep,
        before_start=worker.preflight)
    assert result['result']=='done' and len(client.releases)==8 and not client.stops
    assert client.finishes==list(range(102,108))
    assert result['qr_task_code']['raw']=='231+312+321+231'
    assert result['arm_alignment_results']==[] and len(result['station_work_results'])==6
    assert all(r['visual_alignment_confirmed'] is False for r in result['station_work_results'])
    assert all(r['physical_grasp_completed'] is False for r in result['station_work_results'])
    assert all(b[1]-a[1]>=2 for a,b in zip(client.releases,client.releases[1:]))


def test_first_stage_upload_has_no_second_stage_and_finishes_after_one_stop():
    _, full, _, _ = prepared(1)
    route = first_segment_only(full)
    assert len(full['segments']) == 8 and len(route['segments']) == 1
    assert [p['work_id'] for p in route['points'] if p['work_id']] == [101]
    class FirstOnly(FakeChassis):
        def work_done(self, route_id, index, *, success):
            result = super().work_done(route_id, index, success=success)
            self.state = 4
            return self.report()
    clock = Clock()
    client = FirstOnly(route, clock)
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=60,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'done' and result['test_stations_completed'] == 1
    assert len(client.releases) == 1


def test_first_stage_can_compile_while_later_stage_has_no_path():
    state, profile, config = synthetic_plan(1)
    state['mission_plan']['status'] = 'no_path'
    state['mission_plan']['segments'][1].update(status='no_path', nodes=[], edges=[])
    route = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                 route_id=8, stage_limit=1, **profile)
    validate_geometry(state, route, config)
    assert len(route['segments']) == 1 and route['points'][-1]['work_id'] == 101
    with pytest.raises(ValueError):
        compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=8, **profile)
    state['mission_plan']['segments'][0]['status'] = 'no_path'
    with pytest.raises(ValueError):
        compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                             route_id=8, stage_limit=1, **profile)


@pytest.mark.parametrize('failure', ['other_route', 'premature_done', 'wrong_pose',
                                     'fault_dwell', 'disconnect', 'no_arrival'])
def test_faults_never_release_or_claim_completion_and_attempt_stop(failure):
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock, failure)
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=5,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'failed' and result['test_stations_completed'] == 0
    assert not client.releases and client.stops == 1
    assert result['controller_stop_confirmed'] == (failure != 'disconnect')


def test_does_not_take_over_already_running_controller():
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock, 'no_arrival')
    client.state = 2
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=5,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'failed' and not client.stops and not client.uploads


def test_cancel_during_dwell_stops_without_work_done():
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock)
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=5,
                             log=[], clock=clock, sleep=clock.sleep, cancelled=lambda: clock.now > .5)
    assert result['result'] == 'failed' and result['controller_stop_confirmed']
    assert not client.releases


def test_qr_pose_uses_effective_work_heading_and_keeps_stop_for_bad_yaw():
    route = dict(initial_pose=dict(x_mm=350, y_mm=350, yaw_deg=90), points=[
        dict(x_mm=350, y_mm=1200, yaw_deg=90, work_id=101)])
    configure_work_heading_mode(route, True)
    assert route['points'][0]['yaw_deg'] == 90  # unchanged binary-compatible field
    assert route['points'][0]['expected_work_yaw_deg'] == -180
    good = dict(point_index=0, work_id=101, x_mm=350, y_mm=1197, yaw_deg=179.16)
    assert work_pose_error(good, route, 0) is None
    assert work_pose_error(dict(good, yaw_deg=-180), route, 0) is None
    bad = dict(good, yaw_deg=90)
    assert work_pose_error(bad, route, 0) == 'WAIT_WORK work-heading outside arrival tolerance'


def test_work_heading_wrap_and_absolute_station_offsets_do_not_accumulate():
    route = dict(initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=170), points=[
        dict(x_mm=1, y_mm=1, yaw_deg=170, work_id=101),
        dict(x_mm=2, y_mm=2, yaw_deg=170, work_id=102),
        dict(x_mm=3, y_mm=3, yaw_deg=170, work_id=105),
        dict(x_mm=4, y_mm=4, yaw_deg=170, work_id=103),
        dict(x_mm=5, y_mm=5, yaw_deg=170, work_id=106)])
    configure_work_heading_mode(route, True)
    points = route['points']
    assert points[0]['expected_work_yaw_deg'] == -100
    assert points[1]['expected_work_yaw_deg'] == 80
    assert points[2]['expected_work_yaw_deg'] == 80
    assert points[3]['expected_work_yaw_deg'] == -100
    assert points[4]['expected_work_yaw_deg'] == -100
    assert work_pose_error(dict(point_index=0, work_id=101, x_mm=1, y_mm=1,
                                yaw_deg=-100), route, 0) is None


def test_phase6_waits_through_turn_and_only_then_releases_test_work():
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock, 'turning')
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=60,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'done'
    assert client.releases[0][1] >= 3
    assert len(client.releases) == 8 and client.stops == 0


def test_phase6_over_70_seconds_stops_and_never_sends_work_done():
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock, 'turn_timeout')
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=100,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'failed' and 'work-area turn timeout' in result['error']
    assert client.stops == 1 and not client.releases


def test_60_second_controller_turn_has_communication_margin():
    _, route, _, _ = prepared()
    class LongTurn(FakeChassis):
        def status(self):
            if self.state == 2 and self.clock.now < 61:
                self.clock.now += .01
                return dict(self.report(), phase=6)
            return super().status()
    clock = Clock()
    client = LongTurn(route, clock)
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=100,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == 'done' and not client.stops
    assert client.releases[0][1] >= 63


def test_repeated_wait_work_after_ack_does_not_release_twice_or_restart_dwell():
    _, route, _, _ = prepared()
    class Duplicate(FakeChassis):
        old_wait = None
        def status(self):
            if self.old_wait is not None:
                self.clock.now += .01
                result, self.old_wait = self.old_wait, None
                return dict(result, tick=round(self.clock.now*1000))
            return super().status()
        def work_done(self, route_id, index, *, success):
            self.old_wait = self.report()
            return super().work_done(route_id, index, success=success)
    clock = Clock()
    client = Duplicate(route, clock)
    log = []
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=60,
                             log=log, clock=clock, sleep=clock.sleep)
    assert result['result'] == 'done' and len(client.releases) == 8 and not client.stops
    assert sum(e['action'] == 'test_station_dwell_start' for e in log) == 8


def test_non_work_heading_route_preserves_legacy_per_point_yaw():
    route = dict(initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=90), points=[
        dict(x_mm=10, y_mm=0, yaw_deg=0, work_id=0),
        dict(x_mm=20, y_mm=0, yaw_deg=45, work_id=999)])
    configure_work_heading_mode(route, True)
    assert route['work_heading_mode_active'] is False
    assert [p['travel_yaw_deg'] for p in route['points']] == [0, 45]
    assert 'expected_work_yaw_deg' not in route['points'][1]


@pytest.mark.parametrize('reported_yaw,expected_result,stops', [
    (179.16, 'done', 0), (-180, 'done', 0), (180, 'done', 0), (90, 'failed', 1)])
def test_actual_qr_report_passes_correct_heading_without_stop(reported_yaw, expected_result, stops):
    state, full, profile, _ = prepared(1)
    profile['heading_deg'] = profile['initial_pose']['yaw_deg'] = 90
    full = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'], route_id=7, **profile)
    route = first_segment_only(full)
    assert route['segments'][0]['last_point_index'] == 1
    assert (route['points'][1]['x_mm'], route['points'][1]['y_mm']) == (350, 1200)
    class ActualQr(FakeChassis):
        def report(self):
            status = super().report()
            if self.state in (3, 4):
                status.update(x_mm=350, y_mm=1197, yaw_deg=reported_yaw)
            return status
        def work_done(self, route_id, index, *, success):
            super().work_done(route_id, index, success=success)
            self.state = 4
            return self.report()
    clock = Clock()
    client = ActualQr(route, clock)
    result = run_closed_loop(client, route, station_dwell_s=2, poll_s=.2, timeout_s=20,
                             log=[], clock=clock, sleep=clock.sleep)
    assert result['result'] == expected_result and client.stops == stops
    if expected_result == 'done':
        assert len(client.releases) == 1
    else:
        assert not client.releases


def test_mixed_unknown_work_falls_back_to_wire_yaw_and_disabled_known_stays_legacy():
    route = dict(initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=90), points=[
        dict(x_mm=0, y_mm=0, yaw_deg=90, work_id=101),
        dict(x_mm=0, y_mm=0, yaw_deg=25, work_id=999),
        dict(x_mm=0, y_mm=0, yaw_deg=90, work_id=0)])
    configure_work_heading_mode(route, True)
    assert route['work_heading_mode_active']
    assert route['points'][1]['travel_yaw_deg'] == -180
    assert route['points'][1]['expected_work_yaw_deg'] == 25
    assert route['points'][2]['travel_yaw_deg'] == 25
    configure_work_heading_mode(route, False)
    assert not route['work_heading_mode_active']
    assert all('expected_work_yaw_deg' not in p for p in route['points'])
    assert route['points'][1]['travel_yaw_deg'] == 25


@pytest.mark.parametrize('outcome', ['success', 'timeout', 'invalid', 'fault', 'cancel'])
def test_qr_gate_requires_real_confirmation_and_keeps_polling(outcome):
    from unittest.mock import Mock
    _, route, _, _ = prepared()
    clock = Clock()
    client = FakeChassis(route, clock, 'fault_dwell' if outcome == 'fault' else None)
    worker = Mock()
    worker.result = None
    def poll():
        if clock.now < .8:
            return None
        if outcome == 'timeout':
            raise TimeoutError('QR timeout')
        return dict(confirmed=True, raw='bad' if outcome == 'invalid' else '156+123+516+231')
    worker.poll.side_effect = poll
    log = []
    result = run_closed_loop(client, route, qr_work=worker, station_dwell_s=2,
                             poll_s=.2, timeout_s=60, log=log, clock=clock,
                             sleep=clock.sleep,
                             cancelled=lambda: outcome == 'cancel' and clock.now > .5)
    worker.start.assert_called_once()
    worker.close.assert_called_once()
    if outcome == 'success':
        assert result['result'] == 'done' and len(client.releases) == 8
        assert client.releases[0][1] < 2  # Real confirmation replaces the dummy dwell.
        assert result['qr_task_code']['raw'] == '156+123+516+231'
        assert result['qr_task_code']['route_id'] == route['route_id']
        released = [e for e in log if e['action'] == 'test_station_released']
        assert released[0]['synthetic_work_done'] is False
        assert all(e['synthetic_work_done'] is True for e in released[1:])
    else:
        assert result['result'] == 'failed' and client.stops == 1
        assert not client.releases


@pytest.mark.parametrize('outcome',['success','pending','failure','wrong_context'])
def test_arm_alignment_replaces_six_dwell_gates_without_faking_grasp(outcome):
    from unittest.mock import Mock
    _,route,_,_=prepared()
    clock=Clock()
    client=FakeChassis(route,clock)
    worker=Mock()
    worker.results=[]
    starts=[]
    def start(status,qr):
        starts.append((status['work_id'],clock.now))
    def poll(status):
        if outcome=='failure': raise RuntimeError('arm failed')
        if outcome=='pending': return None
        result=dict(mode='alignment_only',visual_alignment_confirmed=True,
            physical_grasp_completed=False,physical_place_completed=False,
            **{k:status[k] for k in ('route_id','point_index','work_id')})
        if outcome=='wrong_context': result['route_id']+=1
        worker.results.append(result)
        return result
    worker.start.side_effect=start
    worker.poll.side_effect=poll
    log=[]
    result=run_closed_loop(client,route,arm_work=worker,station_dwell_s=2,poll_s=.2,
        timeout_s=30,log=log,clock=clock,sleep=clock.sleep)
    worker.close.assert_called_once()
    if outcome=='success':
        assert result['result']=='done' and not client.stops
        assert [work for work,_ in starts]==list(range(102,108))
        assert len(result['arm_alignment_results'])==6
        assert all(r['physical_grasp_completed'] is False for r in worker.results)
        assert sum(e['action']=='test_station_dwell_start' for e in log)==2
        assert sum(e['action']=='arm_alignment_complete' for e in log)==6
    else:
        assert result['result']=='failed' and client.stops==1
        assert [stage for stage,_ in client.releases]==[1]  # QR test dwell only; no arm fallback.


@pytest.mark.parametrize('zone,scan_result', [(1, 'search'), (2, 'return_station'), (2, None)])
def test_qr_sweep_returns_to_original_gate_before_releasing_stage(zone, scan_result):
    from pi_vision.electrical_routes import add_qr_search

    state, profile, field = synthetic_plan(zone)
    profile.update(heading_deg=90, test_ignore_turn_obstacles=True, footprint=dict(
        front_mm=143, rear_mm=143, left_mm=128.5, right_mm=167.78, margin_mm=2))
    profile['initial_pose']['yaw_deg'] = 90
    baseline = compile_chassis_test(state['mission_plan'], state['navigation']['nodes'],
                                    route_id=7, **profile)
    route = add_qr_search(baseline, field, 12)
    validate_geometry(state, route, field)
    near, far, original = (route['qr_search'][key]
                           for key in ('near_index', 'far_index', 'return_index'))
    points = route['points']
    base_qr = baseline['segments'][0]['last_point_index']
    assert [p['y_mm'] for p in [points[near], points[far], points[original]]] == (
        [1100, 1300, 1200] if zone == 1 else [1300, 1100, 1200])
    assert [p['travel_yaw_deg'] for p in points[near:original+1]] == [90, -180, -180, -180]
    assert points[near]['x_mm'] == points[far]['x_mm'] == 300
    assert points[original]['x_mm'] == baseline['points'][base_qr]['x_mm']
    assert points[original+1:] == baseline['points'][base_qr+1:]
    clock = Clock()

    class SweepChassis(FakeChassis):
        gate_indexes = [near, far] + [s['last_point_index'] for s in route['segments']]
        passed = []
        moving_sample = False

        def status(self):
            clock.now += .01
            if self.state == 2:
                if self.moving_sample:
                    self.moving_sample = False
                    return self.report()
                self.index = self.gate_indexes[len(self.passed)]
                self.work = points[self.index]['work_id']
                self.state = 3
            return self.report()

        def work_done(self, route_id, index, *, success):
            assert self.state == 3 and self.index == index and success is True
            self.passed.append(index)
            self.index += 1
            self.work = 0
            self.state = 4 if len(self.passed) == len(self.gate_indexes) else 2
            self.moving_sample = True
            return self.report()

    client = SweepChassis(route, clock)

    class SweepQR:
        timeout = 1
        started = False
        polls_after_return = 0

        def start(self, *, timeout_s):
            assert client.state == 3 and client.index == near and not self.started
            self.started = True

        def poll(self):
            assert self.started and near in client.passed and original not in client.passed
            found = scan_result == 'search' and client.state == 2 and client.index == far
            if client.state == 3 and client.index == original:
                self.polls_after_return += 1
                found = scan_result == 'return_station' and self.polls_after_return >= 2
            return dict(confirmed=True, raw='231+312+321+231') if found else None

        def close(self):
            pass

    result = run_closed_loop(client, route, qr_work=SweepQR(), station_dwell_s=.2,
                             poll_s=.1, timeout_s=20, log=[], clock=clock, sleep=clock.sleep)
    if scan_result is None:
        assert result['result'] == 'failed'
        assert client.passed == [near, far]
        assert result['controller_stop_confirmed']
    else:
        assert result['result'] == 'done' and result['test_stations_completed'] == 8
        assert client.passed == client.gate_indexes and not client.stops
        assert result['qr_task_code']['scan_source'] == scan_result
        assert result['qr_task_code']['point_index'] == original
