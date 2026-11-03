import struct

import pytest

from pi_vision import electrical_protocol as p
from pi_vision.electrical_link import ElectricalClient, AreaSuperseded, Rejected


class Clock:
    now = 0.0
    def __call__(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


class Device:
    def __init__(self, clock):
        self.clock = clock
        self.buffer = bytearray()
        self.writes = []
        self.selected = None
        self.acknowledged = False
        self.state = 0
        self.route_id = 0
        self.count = self.index = 0
        self.held_attempts = 0
        self.next_area_on_status = None

    def select(self, area, request_id=1):
        self.selected = dict(request_id=request_id, tick=round(self.clock.now*1000)+10,
                             area=area, key=0 if area == 2 else 1)
        self.acknowledged = False
        self.state = self.route_id = self.count = self.index = 0
        self.repeat()

    def repeat(self):
        s = self.selected
        self.buffer.extend(p.frame(p.START_AREA, 19, struct.pack('<IIBBH',
                            s['request_id'], s['tick'], s['area'], s['key'], 0)))

    def status_bytes(self, request, code=0):
        return struct.pack('<BBBBIHHHBBIIiihH', request, code, self.state,
                           4 if self.state == 3 else 0, self.route_id, self.index,
                           self.count, 101 if self.state == 3 else 0,
                           0, 0, round(self.clock.now*1000), 0, 0, 0, 0, 0)

    def write(self, packet):
        self.writes.append(packet)
        kind, seq, body = p.Decoder().feed(packet)[0]
        code = 0
        if kind == p.AREA_ACK:
            request_id, area = struct.unpack('<IB3x', body)
            if (request_id, area) != (self.selected['request_id'], self.selected['area']):
                code = 6
            else:
                self.acknowledged = True
        elif kind == p.UPLOAD_FOR_AREA:
            request_id, = struct.unpack_from('<I', body)
            if request_id != self.selected['request_id']:
                code = 6
            elif not self.acknowledged:
                code = 7
            else:
                self.route_id, self.count = struct.unpack_from('<IH', body, 4)
                self.state = 1
        elif kind == p.START:
            if self.held_attempts:
                self.held_attempts -= 1
                code = 7
            else:
                self.state = 2
        elif kind == p.STATUS:
            if self.next_area_on_status:
                self.select(self.next_area_on_status, self.selected['request_id']+1)
                self.next_area_on_status = None
            if self.state == 2:
                self.state = 3
        elif kind == p.WORK_DONE:
            assert self.state == 3
            self.state, self.index = 4, self.count
        elif kind == p.STOP:
            self.state = 5
        self.buffer.extend(p.frame(p.ACK, seq, self.status_bytes(kind, code)))
        return len(packet)

    def read(self, count):
        self.clock.now += .005
        data = bytes(self.buffer[:13])
        del self.buffer[:13]
        return data


def setup():
    clock = Clock()
    device = Device(clock)
    return ElectricalClient(device, clock=clock), device, clock


def route(area, count=1):
    return dict(route_id=7, start_zone=area, valid_for_motion=True, coordinate_frame="field_photo_ccw",
                initial_pose=dict(x_mm=0, y_mm=0, yaw_deg=0),
                points=[dict(x_mm=0, y_mm=0, yaw_deg=0, work_id=101)]*count)


@pytest.mark.parametrize('area,key', [(1, 1), (2, 0)])
def test_key_mapping_ack_before_plan_and_upload_token(area, key):
    client, device, _ = setup()
    device.select(area)
    def planner(signal, check):
        assert device.acknowledged and signal['key'] == key
        check()
        return route(area)
    selected, planned = client.plan_after_key(planner)
    client.upload(planned, start_request_id=selected['request_id'])
    kind, _, body = p.Decoder().feed(device.writes[-1])[0]
    assert kind == p.UPLOAD_FOR_AREA
    assert struct.unpack_from('<I', body)[0] == 1
    assert len(body) == 44


def test_duplicate_key_frame_never_replans():
    client, device, _ = setup()
    device.select(2)
    client.wait_start_area()
    device.repeat()
    with pytest.raises(TimeoutError):
        client.wait_start_area(.1)
    assert len([x for x in device.writes if x[3] == p.AREA_ACK]) == 1


def test_new_selection_during_scan_discards_result_and_replans_latest_area():
    client, device, _ = setup()
    device.select(2)
    calls = []
    def planner(selected, check):
        calls.append(selected['area'])
        if len(calls) == 1:
            device.next_area_on_status = 1
        check()
        return route(selected['area'])
    selected, planned = client.plan_after_key(planner)
    assert calls == [2, 1] and selected['request_id'] == 2 and planned['start_zone'] == 1
    assert not any(x[3] in (p.UPLOAD, p.UPLOAD_FOR_AREA, p.START) for x in device.writes)


def test_old_upload_or_wrong_zone_never_sent():
    client, device, _ = setup()
    device.select(2)
    client.wait_start_area()
    with pytest.raises(ValueError):
        client.upload(route(1), start_request_id=1)
    device.select(1, 2)
    with pytest.raises(AreaSuperseded):
        client.upload(route(2), start_request_id=1)


def test_key_frame_interleaved_with_status_ack_is_preserved():
    client, device, _ = setup()
    device.select(1)
    assert client.status()['state'] == 0
    assert client.wait_start_area()['area'] == 1


def test_cancelled_selection_rejected_not_used_for_plan():
    client, device, _ = setup()
    device.select(1)
    # A newer request is current but only the old notification is queued.
    device.selected = dict(device.selected, request_id=2, area=2, key=0)
    with pytest.raises(TimeoutError):
        client.wait_start_area(.2)


def test_larger_maximum_bound_upload_frame():
    payload = p.area_upload_payload(route(2, 64), 9)
    assert len(payload) == 1304
    packet = p.frame(p.UPLOAD_FOR_AREA, 8, payload)
    assert len(packet) == 1314
    assert p.Decoder().feed(packet) == [(p.UPLOAD_FOR_AREA, 8, payload)]
    with pytest.raises(ValueError):
        p.frame(p.UPLOAD_FOR_AREA, 8, payload+b'\x00')


@pytest.mark.parametrize('body', [b'\x00'*20, struct.pack('<IIBBH', 0, 1, 1, 1, 0),
                                  struct.pack('<IIBBH', 1, 1, 1, 0, 0),
                                  struct.pack('<IIBBH', 1, 1, 2, 0, 1)])
def test_old_proposed_zone_protocol_and_bad_fields_rejected(body):
    with pytest.raises(ValueError):
        p.start_area_payload(body)


def test_controller_reboot_regressed_request_requires_fresh_client():
    client, device, _ = setup()
    device.select(2, 3)
    client.wait_start_area()
    device.select(1, 1)
    with pytest.raises(RuntimeError, match='reboot'):
        client.wait_start_area()


def test_production_work_callback_has_no_timed_success():
    client, device, clock = setup()
    device.select(1)
    device.held_attempts = 2
    polls = []
    def real_work(c, status):
        polls.append(status['point_index'])
        return True if len(polls) == 3 else None
    result = client.run_after_key(lambda signal, check: route(signal['area']), real_work,
                                  sleep=clock.sleep)
    assert result['state'] == 4 and polls == [0, 0, 0]
    assert len([x for x in device.writes if x[3] == p.WORK_DONE]) == 1


def test_production_without_real_feedback_waits_then_stops_on_timeout():
    client, device, clock = setup()
    device.select(2)
    with pytest.raises(TimeoutError):
        client.run_after_key(lambda signal, check: route(signal['area']),
                             run_timeout=1, sleep=clock.sleep)
    assert not any(x[3] == p.WORK_DONE for x in device.writes)
    assert device.state == 5


@pytest.mark.parametrize('area', [1, 2])
def test_prewarmed_production_uses_selected_key_without_waiting_for_second_key(area):
    client, device, clock = setup()
    device.select(area)
    selected = client.wait_start_area()
    client.wait_start_area = lambda *a, **k: pytest.fail('must not wait for a second key')
    polls = []
    def on_work(c, status):
        polls.append(status['work_id'])
        return True if len(polls) == 3 else None
    status = client.run_selected_route(selected, route(area), on_work, sleep=clock.sleep)
    assert status['state'] == 4 and polls == [101, 101, 101]
    assert sum(x[3] == p.START for x in device.writes) == 1
    assert sum(x[3] == p.WORK_DONE for x in device.writes) == 1


def test_prewarmed_production_rejects_test_route_before_upload():
    client, device, clock = setup()
    device.select(1)
    selected = client.wait_start_area()
    with pytest.raises(ValueError, match='test route'):
        client.run_selected_route(selected, dict(route(1), test_mode='chassis_route_dry_work'),
                                  lambda *_: True, sleep=clock.sleep)
    assert not any(x[3] in (p.UPLOAD_FOR_AREA, p.START) for x in device.writes)


def test_prewarmed_production_rechecks_before_start_and_stops_on_invalidation():
    client, device, clock = setup()
    device.select(1)
    selected = client.wait_start_area()
    def invalidated():
        raise ValueError('plan invalidated')
    with pytest.raises(ValueError, match='invalidated'):
        client.run_selected_route(selected, route(1), lambda *_: True,
                                  before_start=invalidated, sleep=clock.sleep)
    assert not any(x[3] == p.START for x in device.writes)
    assert device.state == 5
