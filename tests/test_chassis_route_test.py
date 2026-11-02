import copy
import math

import pytest

from pi_vision.electrical_routes import compile_chassis_test, first_segment_only
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

    def report(self):
        point = self.route['points'][min(self.index, len(self.route['points'])-1)]
        return dict(state=self.state, phase=4 if self.state == 3 else 0, route_id=self.route_id,
                    point_index=self.index, point_count=self.count, work_id=self.work,
                    fault=0, chassis_fault=0, tick=round(self.clock.now * 1000),
                    x_mm=point['x_mm'], y_mm=point['y_mm'], yaw_deg=point['yaw_deg'])

    def status(self):
        self.clock.now += .01
        if self.state == 2:
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
