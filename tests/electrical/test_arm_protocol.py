import struct
import pytest
from pi_vision import arm_protocol as a
from pi_vision import electrical_protocol as p
from pi_vision.electrical_link import ElectricalClient
from test_electrical_communication import Transport, Clock, report


def arm_body(state=6, command=10, aligned=10, context=(7,2,102)):
    return struct.pack('<IHHIIBBBBHHiiiiiiI', *context, command, aligned,
        state,0,255,2,5,0,1000,-2000,3000,1000,-2000,3000,1234)


def test_absolute_motor_pose_wire_layout_matches_new_firmware():
    payload=a.arm_command_payload(7,2,102,10,2,1.5,-2,360)
    assert len(payload)==32
    assert struct.unpack('<IHHIBBHiiiI',payload)==(7,2,102,10,2,0,0,150,-200,36000,0)
    assert struct.unpack_from('<I',a.arm_command_payload(7,2,102,11,3,reference_id=10),28)[0]==10
    assert a.arm_status_payload(arm_body())['yaw_deg']==-20
    assert a.arm_status_payload(arm_body())['state_name']=='ALIGNED'


@pytest.mark.parametrize('kwargs',[dict(yaw_deg=float('nan')),dict(yaw_deg=True),
    dict(yaw_deg=30000000),dict(reference_id=7)])
def test_invalid_motor_pose_refused(kwargs):
    with pytest.raises(ValueError):
        a.arm_command_payload(7,2,102,10,2,**kwargs)


@pytest.mark.parametrize('mutate', ['length','reserved','state','axis'])
def test_bad_reports_refused(mutate):
    body=bytearray(arm_body())
    if mutate=='length': body.pop()
    if mutate=='reserved': body[22]=1
    if mutate=='state': body[16]=99
    if mutate=='axis': body[18]=3
    with pytest.raises(ValueError): a.arm_status_payload(body)


def test_fault_report_with_extra_upper_flags_bits_is_parsed():
    body = bytearray(arm_body(state=14))
    body[16:24] = struct.pack('<BBBBHH', 14, 1, 2, 4, 11580, 0)
    report = a.arm_status_payload(body)
    assert report['state_name'] == 'FAULT'
    assert (report['fault'], report['active_axis'], report['opcode']) == (1, 2, 4)
    assert report['flags'] == 11580
    assert report['tray_enabled'] and report['tray_uncertain'] and report['grip_done_required']
    assert report['standalone_test_mode'] and not report['origins_valid']


def test_interleaved_arm_events_do_not_satisfy_route_ack_and_missing_report_fails():
    clock=Clock()
    def handler(packet):
        kind,seq,_=p.Decoder().feed(packet)[0]
        return p.frame(a.ARM_EVENT,77,arm_body())+p.frame(p.ACK,seq,report(request=kind))
    transport=Transport(clock,handler)
    client=ElectricalClient(transport,clock=clock)
    assert client.arm_status()['state_name']=='ALIGNED'
    assert client.status()['request']==p.STATUS
    assert client.arm_generation==2
    assert len(transport.writes)==2
    assert client.arm_events[-1]['yaw_deg']==-20
    def no_arm(packet):
        kind,seq,_=p.Decoder().feed(packet)[0]
        return p.frame(p.ACK,seq,report(request=kind))
    transport.handler=no_arm
    with pytest.raises(TimeoutError,match='missing'): client.arm_status()


def test_retry_keeps_same_command_id_and_frame():
    clock=Clock()
    packets=[]
    def handler(packet):
        packets.append(packet)
        if len(packets)==1: return b''
        kind,seq,_=p.Decoder().feed(packet)[0]
        return p.frame(p.ACK,seq,report(request=kind))
    transport=Transport(clock,handler)
    client=ElectricalClient(transport,clock=clock)
    client.arm_align(7,2,102,20,180,270,159.15)
    assert len(packets)==2 and packets[0]==packets[1]


@pytest.mark.parametrize('state,name', [(15, 'DEBUG_MOVING'), (16, 'DEBUG_HOLD')])
def test_new_can_firmware_debug_status_query_uses_only_status_requests(state, name, monkeypatch, capsys):
    import json
    import sys
    from types import SimpleNamespace
    from tools import electrical_comm

    clock = Clock()
    transport = Transport(clock, lambda packet: (
        p.frame(a.ARM_EVENT, 77, arm_body(state=state))
        + p.frame(p.ACK, p.Decoder().feed(packet)[0][1],
                  report(request=p.Decoder().feed(packet)[0][0], state=0))))
    client = ElectricalClient(transport, clock=clock)

    class SerialSession:
        def __enter__(self):
            return transport

        def __exit__(self, *args):
            pass

    monkeypatch.setitem(sys.modules, 'serial', SimpleNamespace(Serial=lambda *args, **kwargs: SerialSession()))
    monkeypatch.setattr(electrical_comm, 'ElectricalClient', lambda port: client)
    from tools import electrical_route_test
    monkeypatch.setattr(electrical_route_test, 'checked_port', lambda path: path)
    monkeypatch.setattr(sys, 'argv', ['electrical_comm', '--port', '/dev/test-electrical', 'arm-status'])
    electrical_comm.main()
    result = json.loads(capsys.readouterr().out)
    assert result['arm']['state_name'] == name
    assert [p.Decoder().feed(packet)[0][0] for packet in transport.writes] == [p.STATUS, a.ARM_STATUS]


def test_standalone_test_begin_and_firmware_mode_flag():
    payload = a.arm_command_payload(44, 0, 102, 1, 6)
    assert struct.unpack('<IHHIBBHiiiI', payload) == (44, 0, 102, 1, 6, 0, 0, 0, 0, 0, 0)
    data = bytearray(arm_body())
    data[19], data[20] = 6, 13
    report = a.arm_status_payload(data)
    assert report['standalone_test_mode'] and report['origins_valid']
    with pytest.raises(ValueError, match='TEST_BEGIN'):
        a.arm_command_payload(44, 1, 102, 1, 6)
    with pytest.raises(ValueError, match='TEST_BEGIN'):
        a.arm_command_payload(44, 0, 102, 2, 6)


@pytest.mark.parametrize('state,fault,opcode,count,slot', [
    (2, 0, 1, 0, 0), (17, 0, 4, 1, 0), (19, 0, 7, 2, 1), (14, 5, 4, 1, 0),
])
def test_v7_reports_tray_state_inventory_and_pwm_fault(state, fault, opcode, count, slot):
    body = bytearray(arm_body())
    body[16:24] = bytes([state, fault, 255, opcode, 61, 0, count, slot])
    report = a.arm_status_payload(body)
    assert report['tray_enabled'] and report['tray_uncertain'] and report['grip_done_required']
    assert (report['state'], report['fault'], report['opcode']) == (state, fault, opcode)
    assert (report['tray_count'], report['destination_slot']) == (count, slot)


@pytest.mark.parametrize('count,slot,flags', [(4, 0, 16), (1, 4, 16), (0, 0, 32)])
def test_invalid_v7_tray_fields_are_rejected(count, slot, flags):
    body = bytearray(arm_body())
    body[20:24] = bytes([flags, 0, count, slot])
    with pytest.raises(ValueError, match='invalid ARM'):
        a.arm_status_payload(body)


def test_grip_abort_opcode_9_command_and_status_are_supported():
    payload = a.arm_command_payload(7, 0, 102, 12, 9, reference_id=10)
    assert len(payload) == 32
    assert struct.unpack('<IHHIBBHiiiI', payload) == (7, 0, 102, 12, 9, 0, 0, 0, 0, 0, 10)
    with pytest.raises(ValueError, match='GRIP_ABORT'):
        a.arm_command_payload(7, 0, 102, 12, 9, reference_id=0)
    body = bytearray(arm_body(state=4))
    body[16:24] = struct.pack('<BBBBHH', 4, 0, 255, 9, 29, 0)
    report = a.arm_status_payload(body)
    assert report['state_name'] == 'WAIT_VISION' and report['opcode'] == 9


def test_align_pick_opcode_10_command_and_status_are_supported():
    payload = a.arm_command_payload(7, 0, 102, 12, 10, -594.5, 20.5, -105.4, auto_load=True)
    assert len(payload) == 32
    assert struct.unpack('<IHHIBBHiiiI', payload) == (7, 0, 102, 12, 10, 1, 0, -59450, 2050, -10540, 0)
    payload_wait = a.arm_command_payload(7, 0, 102, 13, 10, -594.5, 20.5, -105.4, auto_load=False)
    assert struct.unpack('<IHHIBBHiiiI', payload_wait)[5] == 0
    with pytest.raises(ValueError, match='auto_load'):
        a.arm_command_payload(7, 0, 102, 12, 2, 0, 0, 0, auto_load=True)
    with pytest.raises(ValueError, match='GRIP_ABORT'):
        a.arm_command_payload(7, 0, 102, 12, 10, -594.5, 0, 0, reference_id=10)
    body = bytearray(arm_body(state=8))
    body[16:24] = struct.pack('<BBBBHH', 8, 0, 255, 10, 29, 0)
    report = a.arm_status_payload(body)
    assert report['state_name'] == 'WAIT_GRIP' and report['opcode'] == 10
