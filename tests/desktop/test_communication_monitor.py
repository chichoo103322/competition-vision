import json
import struct
import time

from pi_vision import electrical_protocol as p
from tools.communication_monitor import LogSource, interpret, create_server


def status_body(**changes):
    values = dict(request=p.STATUS, code=0, state=2, phase=6, route_id=12,
                  point_index=1, point_count=20, work_id=101, chassis_fault=0,
                  fault_address=0, tick=100, fault=0, x=350, y=1200, yaw=17916, reserved=0)
    values.update(changes)
    return struct.pack('<BBBBIHHHBBIIiihH', *values.values())


def line(data, at=None, event='RX_RAW'):
    return json.dumps(dict(at=at or time.time(), event=event, hex=data.hex()))+'\n'


def test_split_frames_and_crc_do_not_invent_fresh_status(tmp_path):
    source = LogSource(tmp_path)
    packet = p.frame(p.ACK, 25, status_body())
    log = tmp_path/'wire.jsonl'
    log.write_text(line(packet[:17]))
    assert source.snapshot()['controller'] is None
    with log.open('a') as f:
        f.write(line(packet[17:]))
    state = source.snapshot()
    assert state['controller_recent'] is True
    assert state['controller']['phase_name'] == '工作区原地转向'
    assert state['controller']['yaw_deg'] == 179.16
    broken = packet[:-1]+bytes([packet[-1] ^ 1])
    with log.open('a') as f:
        f.write(line(broken))
    assert source.snapshot()['last_status_at'] == state['last_status_at']


def test_partial_json_rotation_and_stale_data(tmp_path):
    source = LogSource(tmp_path)
    log = tmp_path/'wire.jsonl'
    payload = line(p.frame(p.ACK, 1, status_body()), at=time.time()-60)
    log.write_text(payload[:20])
    assert source.snapshot()['controller'] is None
    with log.open('a') as f:
        f.write(payload[20:])
    assert source.snapshot()['controller_recent'] is False
    log.rename(tmp_path/'wire.jsonl.1')
    log.write_text(line(p.frame(p.START_AREA, 2, struct.pack('<IIBBH', 4, 42, 2, 0, 0))))
    state = source.snapshot()
    assert state['last_key']['key'] == 0
    assert state['last_key']['area'] == 2
    assert state['controller_recent'] is False  # A key is not a pose update.


def test_rejection_fault_and_work_done_are_visible():
    name, details, routine = interpret(p.ACK, status_body(code=7, state=6, chassis_fault=3))
    assert 'BAD_STATE' in name
    assert not routine
    assert details['chassis_fault'] == 3
    _, details, _ = interpret(p.WORK_DONE, struct.pack('<IHBB', 12, 1, 1, 0))
    assert details == dict(route_id=12, point_index=1, success=1, reserved=0)
    name, details, _ = interpret(p.EVENT, status_body(request=0, code=4, state=3))
    assert name == '事件 · WORK_REQUIRED'
    assert details['event_name'] == 'WORK_REQUIRED'


def test_missing_source_and_malformed_lines(tmp_path):
    source = LogSource(tmp_path)
    assert source.snapshot()['warning']
    (tmp_path/'wire.jsonl').write_text('invalid\n'+json.dumps({'at': time.time(), 'event': 'READY'})+'\n')
    state = source.snapshot()
    assert state['warning']
    assert state['entries'][-1]['name'] == 'READY'


def test_replay_decodes_route_points_and_keeps_bounded_history(tmp_path):
    body = struct.pack('<IHHiii', 42, 1, 0, 150, 150, 9000)
    body += struct.pack('<iiiHHHH', 350, 1200, 9000, 120, 15, 101, 1)
    _, details, _ = interpret(p.UPLOAD_FOR_AREA, struct.pack('<I', 3)+body)
    assert details['points'][0]['work_id'] == 101
    assert details['initial_pose']['yaw_deg'] == 90
    source = LogSource(tmp_path)
    (tmp_path/'wire.jsonl').write_text(line(p.frame(p.STATUS, 1))*700)
    assert len(source.snapshot()['entries']) == 600


def test_read_only_http_sources_download_and_unknown_paths(tmp_path):
    import threading
    import urllib.request
    import urllib.error
    test = tmp_path/'test'
    test.mkdir()
    log = line(p.frame(p.ACK, 1, status_body()))
    (test/'wire.jsonl').write_text(log)
    server = create_server({'test': LogSource(test), 'competition': LogSource(tmp_path/'missing')}, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    try:
        with urllib.request.urlopen(base+'/api/state') as response:
            assert json.load(response)['controller']['route_id'] == 12
        request = urllib.request.Request(base+'/api/state', headers={'Accept-Encoding': 'gzip'})
        with urllib.request.urlopen(request) as response:
            import gzip
            assert response.headers['Content-Encoding'] == 'gzip'
            assert json.loads(gzip.decompress(response.read()))['controller']['route_id'] == 12
        with urllib.request.urlopen(base+'/api/state?source=competition') as response:
            assert json.load(response)['controller'] is None
        with urllib.request.urlopen(base+'/download') as response:
            assert response.read().decode() == log
        for path, method, expected in [('/../../etc/passwd', 'GET', 404),
                                       ('/api/state?source=bad', 'GET', 400),
                                       ('/start', 'POST', 501)]:
            try:
                urllib.request.urlopen(urllib.request.Request(base+path, method=method))
                assert False, 'request should be rejected'
            except urllib.error.HTTPError as error:
                assert error.code == expected
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
