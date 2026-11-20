"""car_code1 2 arm_work.c 的三轴协议；角度是电机轴绝对角度。"""
import struct
from .electrical_protocol import integer, scaled

ARM_COMMAND, ARM_STATUS, ARM_EVENT = 0x31, 0x32, 0x83
ARM_STATES = ('NEED_ORIGIN', 'ZEROING', 'READY', 'PREPARING', 'WAIT_VISION',
              'ALIGNING', 'ALIGNED', 'LOWERING', 'WAIT_GRIP', 'LIFTING',
              'PICKED', 'PARKING', 'READY_LEAVE', 'STOPPED', 'FAULT',
              'DEBUG_MOVING', 'DEBUG_HOLD')


def arm_command_payload(route_id, point_index, work_id, command_id, opcode,
                        z_deg=0, yaw_deg=0, reach_deg=0, reference_id=0):
    values = [scaled(v, 100, 21474836, n) for v, n in
              zip((z_deg, yaw_deg, reach_deg), ('z_deg', 'yaw_deg', 'reach_deg'))]
    integer(opcode, 1, 6, 'opcode')
    integer(reference_id, 0, 0xffffffff, 'reference_id')
    if opcode != 2 and any(values):
        raise ValueError('only ALIGN accepts motor angles')
    if opcode in (3, 4) and not reference_id or opcode not in (3, 4) and reference_id:
        raise ValueError('PICK/GRIP_DONE require the last aligned command ID only')
    if opcode == 1:
        if route_id or point_index or work_id:
            raise ValueError('ZERO requires empty context; never zero automatically')
    else:
        integer(route_id, 1, 0xffffffff, 'route_id')
        integer(point_index, 0, 63, 'point_index')
        integer(work_id, 102, 107, 'work_id')
        if opcode == 6 and (point_index != 0 or command_id != 1):
            raise ValueError('TEST_BEGIN requires point_index=0 and command_id=1')
    return struct.pack('<IHHIBBHiiiI', integer(route_id, 0, 0xffffffff, 'route_id'),
        integer(point_index, 0, 63, 'point_index'), integer(work_id, 0, 65535, 'work_id'),
        integer(command_id, 1, 0xffffffff, 'command_id'), opcode, 0, 0, *values, reference_id)


def arm_status_payload(payload):
    if len(payload) != 52:
        raise ValueError('expected 52-byte ARM status, not route status')
    names = ('route_id', 'point_index', 'work_id', 'command_id', 'aligned_command_id',
             'state', 'fault', 'active_axis', 'opcode', 'flags', 'reserved',
             'z_cdeg', 'yaw_cdeg', 'reach_cdeg',
             'target_z_cdeg', 'target_yaw_cdeg', 'target_reach_cdeg', 'tick')
    result = dict(zip(names, struct.unpack('<IHHIIBBBBHHiiiiiiI', payload)))
    if (result['state'] >= len(ARM_STATES) or result['fault'] > 4 or result['reserved'] or
            result['flags'] & ~15 or result['active_axis'] not in (0, 1, 2, 255) or
            result['opcode'] > 6 or result['point_index'] > 63):
        raise ValueError('invalid ARM status fields')
    result['state_name'] = ARM_STATES[result['state']]
    result['origins_valid'] = bool(result['flags'] & 1)
    result['standalone_test_mode'] = bool(result['flags'] & 8)
    for name in ('z', 'yaw', 'reach', 'target_z', 'target_yaw', 'target_reach'):
        result[name + '_deg'] = result[name + '_cdeg'] / 100
    return result
