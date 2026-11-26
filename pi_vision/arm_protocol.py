"""STM32 三轴及托盘状态协议；角度是电机轴绝对角度。"""
import struct
from .electrical_protocol import integer, scaled

ARM_COMMAND, ARM_STATUS, ARM_EVENT = 0x31, 0x32, 0x83
ARM_STATES = ('NEED_ORIGIN', 'ZEROING', 'READY', 'PREPARING', 'WAIT_VISION',
              'ALIGNING', 'ALIGNED', 'LOWERING', 'WAIT_GRIP', 'LIFTING',
              'PICKED', 'PARKING', 'READY_LEAVE', 'STOPPED', 'FAULT',
              'DEBUG_MOVING', 'DEBUG_HOLD', 'TRAY_LOADING', 'TRAY_UNLOADING', 'PLACED')


def arm_command_payload(route_id, point_index, work_id, command_id, opcode,
                        z_deg=0, yaw_deg=0, reach_deg=0, reference_id=0):
    values = [scaled(v, 100, 21474836, n) for v, n in
              zip((z_deg, yaw_deg, reach_deg), ('z_deg', 'yaw_deg', 'reach_deg'))]
    integer(opcode, 1, 9, 'opcode')
    if opcode in (7, 8):
        raise ValueError('tray place/reload opcodes require dedicated slot payload')
    integer(reference_id, 0, 0xffffffff, 'reference_id')
    if opcode != 2 and any(values):
        raise ValueError('only ALIGN accepts motor angles')
    if opcode in (3, 4, 9) and not reference_id or opcode not in (3, 4, 9) and reference_id:
        raise ValueError('PICK/GRIP_DONE/GRIP_ABORT require the last aligned command ID only')
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
        raise ValueError(f'expected 52-byte ARM status, got {len(payload)} bytes')
    names = ('route_id', 'point_index', 'work_id', 'command_id', 'aligned_command_id',
             'state', 'fault', 'active_axis', 'opcode', 'flags', 'reserved',
             'z_cdeg', 'yaw_cdeg', 'reach_cdeg',
             'target_z_cdeg', 'target_yaw_cdeg', 'target_reach_cdeg', 'tick')
    result = dict(zip(names, struct.unpack('<IHHIIBBBBHHiiiiiiI', payload)))
    tray_enabled = bool(result['flags'] & 16)
    tray_uncertain = bool(result['flags'] & 32)
    tray_count, destination_slot = result['reserved'] & 255, result['reserved'] >> 8
    if (result['state'] >= (len(ARM_STATES) if tray_enabled else 17)
            or result['fault'] > (5 if tray_enabled else 4)
            or result['active_axis'] not in (0, 1, 2, 255)
            or result['opcode'] > (9 if tray_enabled else 6) or result['point_index'] > 63
            or (tray_enabled and (tray_count > 3 or destination_slot > 3))
            or (not tray_enabled and (tray_uncertain or result['reserved']))):
        raise ValueError(f"invalid ARM status fields: state={result['state']}, fault={result['fault']}, "
                         f"axis={result['active_axis']}, opcode={result['opcode']}, "
                         f"flags={result['flags']}, reserved={result['reserved']}, "
                         f"point_index={result['point_index']}")
    result.update(tray_enabled=tray_enabled, tray_uncertain=tray_uncertain,
                  tray_count=tray_count, destination_slot=destination_slot,
                  grip_done_required=bool(result['flags'] & 4))
    result['state_name'] = ARM_STATES[result['state']]
    result['origins_valid'] = bool(result['flags'] & 1)
    result['standalone_test_mode'] = bool(result['flags'] & 8)
    for name in ('z', 'yaw', 'reach', 'target_z', 'target_yaw', 'target_reach'):
        result[name + '_deg'] = result[name + '_cdeg'] / 100
    return result
