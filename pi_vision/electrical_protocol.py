"""STM32 car_code1 route V1; independent of the legacy AA vision protocol.

Wire layout adapted from user-supplied car_code1/tools/route_client.py (2026-11-02).
No hardware access. Key extension matches car_code1 2/README_start_area.md.
"""
import math
import struct
import time

UPLOAD, START, STOP, WORK_DONE, STATUS, VISION = 0x10, 0x11, 0x12, 0x13, 0x14, 0x30
ACK, EVENT = 0x80, 0x81
STATES = ['EMPTY', 'READY', 'RUNNING', 'WAIT_WORK', 'DONE', 'STOPPED', 'FAULT']
EVENTS = {1:'READY', 2:'STARTED', 3:'ARRIVED', 4:'WORK_REQUIRED', 5:'DONE',
          6:'STOPPED', 7:'FAULT', 8:'POINT_DONE'}
RESULTS = ['OK', 'BAD_TYPE', 'BAD_LENGTH', 'BAD_CRC', 'BAD_VALUE', 'BUSY',
           'BAD_ID', 'BAD_STATE', 'CONFLICT']
MAX_PAYLOAD = 1304
AREA_ACK, UPLOAD_FOR_AREA, START_AREA = 0x15, 0x16, 0x82


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def frame(kind, sequence, payload=b''):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too large')
    body = struct.pack('<BBHH', 1, kind, sequence, len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class Decoder:
    """Incremental decoder: tolerates partial frames, noise and corrupt frames."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.buffer = bytearray()
        self.last_feed = self.clock()

    def feed(self, data):
        now = self.clock()
        if not data:
            return []
        if now - self.last_feed > 0.5:
            self.buffer.clear()
        self.last_feed = now
        self.buffer.extend(data)
        messages = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xa5\x5a':
                del self.buffer[0]
                continue
            if len(self.buffer) < 8:
                break
            version, kind, sequence, size = struct.unpack_from('<BBHH', self.buffer, 2)
            if version != 1 or size > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            end = 10 + size
            if len(self.buffer) < end:
                break
            if crc16(self.buffer[2:end-2]) != struct.unpack_from('<H', self.buffer, end-2)[0]:
                del self.buffer[0]
                continue
            messages.append((kind, sequence, bytes(self.buffer[8:end-2])))
            del self.buffer[:end]
        return messages


def integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')
    return value


def scaled(value, scale, limit, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if not -limit <= value <= limit:
        raise ValueError(f'{name} outside +/-{limit}')
    return round(value * scale)


def upload_payload(route):
    from .electrical_coordinates import require_electrical_frame
    require_electrical_frame(route.get('coordinate_frame'))
    route_id = integer(route['route_id'], 1, 0xffffffff, 'route_id')
    initial = route['initial_pose']
    points = route['points']
    integer(len(points), 1, 64, 'point count')
    def pose(point):
        return (scaled(point['x_mm'], 1, 100000, 'x_mm'),
                scaled(point['y_mm'], 1, 100000, 'y_mm'),
                scaled(point['yaw_deg'], 100, 180, 'yaw_deg'))
    payload = struct.pack('<IHHiii', route_id, len(points), 0, *pose(initial))
    for point in points:
        move = integer(point.get('move_rpm', 40), 1, 120, 'move_rpm')
        turn = integer(point.get('turn_rpm', 20), 1, 120, 'turn_rpm')
        work = integer(point.get('work_id', 0), 0, 65535, 'work_id')
        payload += struct.pack('<iiiHHHH', *pose(point), move, turn, work, int(work != 0))
    return payload


def status_payload(payload):
    if len(payload) != 36:
        raise ValueError('expected 36-byte status')
    fields = struct.unpack('<BBBBIHHHBBIIiihH', payload)
    names = ('request', 'code', 'state', 'phase', 'route_id', 'point_index',
             'point_count', 'work_id', 'chassis_fault', 'fault_address', 'tick',
             'fault', 'x_mm', 'y_mm', 'yaw_cdeg', 'reserved')
    result = dict(zip(names, fields))
    if result['state'] > 6 or result['phase'] > 4 or result['reserved'] != 0:
        raise ValueError('invalid V1 status fields')
    if result['point_count'] > 64 or result['point_index'] > result['point_count']:
        raise ValueError('invalid V1 point index/count')
    result['state_name'] = STATES[result['state']] if result['state'] < len(STATES) else 'UNKNOWN'
    result['yaw_deg'] = result['yaw_cdeg'] / 100
    return result


def start_area_payload(payload):
    if len(payload) != 12:
        raise ValueError('expected 12-byte START_AREA from car_code1 2')
    request_id, tick, area, key, reserved = struct.unpack('<IIBBH', payload)
    if not request_id or area not in (1, 2) or key != (0 if area == 2 else 1) or reserved:
        raise ValueError('invalid START_AREA')
    return dict(request_id=request_id, tick=tick, area=area, key=key)


def area_ack_payload(signal):
    return struct.pack('<IB3x', integer(signal['request_id'], 1, 0xffffffff, 'request_id'),
                       integer(signal['area'], 1, 2, 'area'))


def area_upload_payload(route, request_id):
    return struct.pack('<I', integer(request_id, 1, 0xffffffff, 'request_id')) + upload_payload(route)
