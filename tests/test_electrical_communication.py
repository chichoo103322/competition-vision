"""UART-free protocol, workflow boundaries and saved-plan conversion checks."""
import copy
import struct

import pytest

from pi_vision import electrical_protocol as p
from pi_vision.electrical_link import ElectricalClient, Rejected
from pi_vision.electrical_routes import STATIONS, compile_preview, segment_for_status


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


def report(request=p.STATUS, code=0, state=3, route_id=7, index=2, count=10, work=101):
    return struct.pack('<BBBBIHHHBBIIiihH', request, code, state, 4 if state == 3 else 0,
                       route_id, index, count, work, 0, 0, 12345, 0, -100, 2400, -9000, 0)


class Transport:
    def __init__(self, clock, handler):
        self.clock, self.handler = clock, handler
        self.writes, self.buffer = [], bytearray()

    def write(self, packet):
        self.writes.append(packet)
        self.buffer.extend(self.handler(packet))
        return len(packet)

    def read(self, _size):
        self.clock.now += .01
        data = bytes(self.buffer[:7])  # partial serial reads, including mixed ACK/events
        del self.buffer[:7]
        return data


def client(handler):
    clock = Clock()
    transport = Transport(clock, handler)
    return ElectricalClient(transport, clock=clock), transport, clock


def test_crc_and_wire_golden_status_frame():
    assert p.crc16(b'123456789') == 0x29B1
    # CRC cross-checked independently with binascii.crc_hqx.
    assert p.frame(p.STATUS, 0x1234).hex() == 'a55a011434120000f70d'


def test_decoder_noise_corrupt_fragmented_and_back_to_back():
    good = p.frame(p.STATUS, 5)
    corrupt = bytearray(good)
    corrupt[-1] ^= 1
    decoder = p.Decoder()
    assert decoder.feed(b'noise' + corrupt + good[:4]) == []
    assert decoder.feed(good[4:] + good) == [(p.STATUS, 5, b'')] * 2


def test_decoder_timeout_and_empty_poll_do_not_keep_partial_frame_alive():
    clock = Clock()
    decoder = p.Decoder(clock)
    packet = p.frame(p.STATUS, 1)
    assert decoder.feed(packet[:5]) == []
    clock.now = .4
    decoder.feed(b'')
    clock.now = .6
    assert decoder.feed(packet[5:] + packet) == [(p.STATUS, 1, b'')]


@pytest.mark.parametrize('header', [b'\xa5\x5a\x02\x14\x01\x00\x00\x00',
                                    b'\xa5\x5a\x01\x14\x01\x00\xff\xff'])
def test_bad_version_or_length_resynchronizes(header):
    assert p.Decoder().feed(header + p.frame(p.STATUS, 2)) == [(p.STATUS, 2, b'')]


def test_ack_requires_sequence_and_request_and_events_do_not_refresh_pose():
    def handler(packet):
        kind, seq, _ = p.Decoder().feed(packet)[0]
        return (p.frame(p.EVENT, 12, report(0, 4)) +
                p.frame(p.ACK, (seq-1) & 65535, report(kind)) +
                p.frame(p.ACK, seq, report(p.START)) +
                p.frame(p.ACK, seq, b'bad') + p.frame(p.ACK, seq, report(kind)))
    c, _, clock = client(handler)
    assert c.status()['x_mm'] == -100
    assert len(c.events) == 1
    pose = c.pose_snapshot()
    assert pose['pose'] == dict(x_mm=-100, y_mm=2400, yaw_deg=-90)
    assert pose['frame_id'] == 'field_photo_ccw'
    assert pose['transport_fresh'] and not pose['valid_for_motion'] and not pose['continuous']
    clock.now += .51
    assert c.pose_snapshot()['pose'] is None


def test_retry_is_identical_and_timeout_invalidates_state():
    c, transport, _ = client(lambda _: b'')
    c.selected_area = dict(area=1)
    with pytest.raises(TimeoutError):
        c._exchange(p.STATUS, timeout=.03, retries=3)
    assert len(transport.writes) == 3 and len(set(transport.writes)) == 1
    assert c.selected_area is None and not c.pose_snapshot()['transport_fresh']



def geometry():
    paths = [[8, 5], [5, 4, 7], [7, 4, 1], [1, 4, 3],
             [3, 4, 7], [7, 4, 1], [1, 4, 3], [3, 4, 5, 8]]
    segments = [dict(index=i, nodes=path, edges=list(range(len(path)-1)), status='path',
                     from_station=STATIONS[i-1], to_station=STATIONS[i])
                for i, path in enumerate(paths, 1)]
    plan = dict(status='ready', segments=segments, settings=dict(start_zone=1),
                plan_id='saved', session_id='source', source_seq=20, blocked_edges=[],
                valid_for_motion=False)
    nodes = [dict(id=y*3+x, xy=[.35+.85*x, .35+.85*y]) for y in range(3) for x in range(3)]
    profile = dict(route_id=7, coordinate_frame="field_photo_ccw",
                   initial_pose=dict(x_mm=330, y_mm=200, yaw_deg=90),
                   segment_yaws_deg=[90]*8, move_rpm=40, turn_rpm=20)
    return plan, nodes, profile


def test_eight_segment_preview_keeps_gates_units_and_initial_pose():
    plan, nodes, profile = geometry()
    before = copy.deepcopy(plan)
    route = compile_preview(plan, nodes, **profile)
    assert plan == before and route['initial_pose'] == profile['initial_pose']
    assert route['points'][0]['x_mm'] == 350
    assert [x['work_id'] for x in route['points'] if x['work_id']] == list(range(101, 109))
    assert route['valid_for_motion'] is False
    payload = p.upload_payload(route)
    assert len(payload) == 20 + 20 * len(route['points'])
    assert struct.unpack_from('<IHHiii', payload) == (7, len(route['points']), 0, 330, 200, 9000)
    for s in route['segments']:
        status = dict(route_id=7, point_count=len(route['points']), state=3,
                      point_index=s['last_point_index'], work_id=s['work_id'])
        assert segment_for_status(route, status)['segment'] == s['segment']
    c, transport, _ = client(lambda _: b'')
    with pytest.raises(ValueError, match='preview'):
        c.upload(route)
    assert transport.writes == []


@pytest.mark.parametrize('defect', ['stale', 'disconnected', 'heading', 'limit', 'blocked', 'order', 'zone'])
def test_invalid_preview_rejected(defect):
    plan, nodes, profile = geometry()
    if defect == 'stale':
        plan['evidence_status'] = 'scan_unavailable'
    elif defect == 'disconnected':
        plan['segments'][2]['nodes'][0] = 8
    elif defect == 'heading':
        profile['segment_yaws_deg'][2] = float('nan')
    elif defect == 'limit':
        plan['segments'][0]['nodes'] = [8]*65 + [5]
        plan['segments'][0]['edges'] = [0]*65
    elif defect == 'blocked':
        plan['blocked_edges'] = [0]
    elif defect == 'order':
        plan['segments'][1]['to_station'] = 'storage'
    elif defect == 'zone':
        plan['settings']['start_zone'] = True
    with pytest.raises(ValueError):
        compile_preview(plan, nodes, **profile)


def test_work_done_requires_real_boolean_and_status_empty_has_no_pose():
    def handler(packet):
        kind, seq, _ = p.Decoder().feed(packet)[0]
        return p.frame(p.ACK, seq, report(kind, state=0, route_id=0, index=0, count=0, work=0))
    c, transport, _ = client(handler)
    with pytest.raises(ValueError):
        c.work_done(7, 2, success='yes')
    assert not transport.writes
    c.status()
    assert c.pose_snapshot()['transport_fresh'] and c.pose_snapshot()['pose'] is None
