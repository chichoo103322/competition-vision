"""Direction review of the actual V1 wire payload; never a motion controller.

Wire coordinates use the agreed photo-aligned field, independently of the UI
checkbox. Lidar installation angles are not chassis headings. The review assumes ideal arrival at each point;
STM32 uses its live estimated pose and configured tolerances during execution.
"""
import math
import struct

NOMINAL_FRAME = 'field_nominal'
ELECTRICAL_FRAME = 'field_photo_ccw'
FIELD_SIZE_MM = 2400


def require_electrical_frame(frame):
    if frame != ELECTRICAL_FRAME:
        raise ValueError(f'explicit coordinate_frame={ELECTRICAL_FRAME} required; migrate old X/Y/yaw together')


def nominal_xy_to_electrical_mm(x_m, y_m):
    """Only planner nodes pass here: nominal metres -> photo-frame millimetres."""
    return FIELD_SIZE_MM - round(x_m * 1000), FIELD_SIZE_MM - round(y_m * 1000)


def _rotate_pose(pose):
    for key in ('x_mm', 'y_mm', 'yaw_deg'):
        if type(pose[key]) not in (int, float) or not math.isfinite(pose[key]):
            raise ValueError('frame conversion requires finite mm/degree chassis pose')
    if abs(pose['yaw_deg']) > 180:
        raise ValueError('yaw must be within +/-180 degrees')
    return dict(pose, x_mm=FIELD_SIZE_MM-pose['x_mm'], y_mm=FIELD_SIZE_MM-pose['y_mm'],
                yaw_deg=(pose['yaw_deg'] + 360) % 360 - 180)


def nominal_to_electrical_pose(pose):
    """Convert a chassis pose, NOT an uncalibrated lidar sensor pose."""
    return _rotate_pose(pose)


def electrical_to_nominal_pose(pose):
    """Inverse for planner collision checks; 180-degree field rotation is its own inverse."""
    return _rotate_pose(pose)


def direction_review(payload):
    """Decode ordinary UPLOAD bytes (without the start-area request prefix)."""
    if len(payload) < 20:
        raise ValueError('incomplete route header')
    route_id, count, reserved, x, y, yaw = struct.unpack_from('<IHHiii', payload)
    if reserved or not 1 <= count <= 64 or len(payload) != 20 + count * 20:
        raise ValueError('invalid route payload')
    initial = dict(x_mm=x, y_mm=y, yaw_deg=yaw / 100)
    legs = []
    for index in range(count):
        nx, ny, nyaw, move, turn, work, flags = struct.unpack_from('<iiiHHHH', payload, 20 + index * 20)
        rotation = ((nyaw - yaw) / 100 + 180) % 360 - 180
        if rotation == -180:
            rotation = 180  # route.c chooses positive rotation at exactly 180.
        theta = math.radians(nyaw / 100)  # Firmware turns BEFORE translating.
        dx, dy = nx - x, ny - y
        forward = math.cos(theta) * dx + math.sin(theta) * dy
        left = -math.sin(theta) * dx + math.cos(theta) * dy
        forward, left = round(forward, 6), round(left, 6)
        movements = [dict(action='前进' if forward >= 0 else '后退', distance_mm=abs(forward)),
                     dict(action='左移' if left >= 0 else '右移', distance_mm=abs(left))]
        movements.sort(key=lambda item: -item['distance_mm'])  # forward first on ties
        actions = ([] if not rotation else [dict(action='逆时针转向' if rotation > 0 else '顺时针转向',
                                                 angle_deg=abs(rotation))])
        actions += [item for item in movements if item['distance_mm'] > 0]
        legs.append(dict(point_index=index, target=dict(x_mm=nx, y_mm=ny, yaw_deg=nyaw / 100),
                         dx_mm=dx, dy_mm=dy, forward_mm=forward, left_mm=left,
                         actions=actions, work_id=work))
        x, y, yaw = nx, ny, nyaw
    return dict(route_id=route_id, coordinate_frame=ELECTRICAL_FRAME,
                axes='photo-aligned field: origin lower left, +X right, +Y up', yaw='CCW positive; 0 = +X',
                wire_units=dict(x_y='mm', yaw='0.01 degree'),
                assumption='ideal waypoint poses; actual controller uses feedback and tolerances',
                initial_pose=initial, legs=legs)
