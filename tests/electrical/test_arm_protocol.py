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


@pytest.mark.parametrize('mutate', ['length','reserved','state','axis','flags'])
def test_bad_reports_refused(mutate):
    body=bytearray(arm_body())
    if mutate=='length': body.pop()
    if mutate=='reserved': body[22]=1
    if mutate=='state': body[16]=99
    if mutate=='axis': body[18]=3
    if mutate=='flags': body[20]=128
    with pytest.raises(ValueError): a.arm_status_payload(body)


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
